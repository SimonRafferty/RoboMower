// ══════════════════════════════════════════════════════════════════════════════
//  perimeter.cpp — RoboMower perimeter recording and polygon management
//
//  See perimeter.h for the full module description.
//
//  Static state layout:
//    s_rec_x / s_rec_y   — raw recording buffer (MAX_PERIMETER_POINTS floats each)
//    s_rec_count         — waypoints accumulated so far
//    s_rec_last_x/y      — position of last committed waypoint (distance gate)
//    s_recording         — true while a recording session is active
//    s_valid             — true when all three polygons are ready for use
//    s_perimeter         — raw perimeter polygon (in-memory copy)
//    s_nav_boundary      — nav exclusion polygon (in-memory copy)
//    s_working_area      — working area polygon (in-memory copy)
//
//  References:
//    perimeter.h, config.h, geometry.h, nvs_storage.h
//    HANDOFFS/13_perimeter/HANDOFF.md
// ══════════════════════════════════════════════════════════════════════════════

#include "perimeter.h"
#include "config.h"
#include "mower_config.h"
#include "clipper_offset.h"
#include "rtk_gps.h"
#include <cmath>
#include <cfloat>
#include <cstring>

// ── Private constants ─────────────────────────────────────────────────────────

/** Minimum travel distance between consecutive recorded waypoints (metres). */
static constexpr float RECORD_DIST_M = 0.2f;

/** Minimum polygon area to pass validation (m²). */
static constexpr float MIN_RECORD_AREA_M2 = 5.0f;

/** Minimum number of waypoints required to form a valid polygon. */
static constexpr int   MIN_RECORD_POINTS = 3;

/** Grid resolution for unreachable-zone scan (metres). */
static constexpr float UNREACHABLE_SCAN_STEP_M = 1.0f;

/** Clustering radius for unreachable-zone grouping (metres). */
static constexpr float UNREACHABLE_CLUSTER_RADIUS_M = 2.0f;

/** Maximum unreachable grid points to collect (limits heap/stack usage). */
static constexpr int   MAX_UNREACHABLE_PTS = 300;

/**
 * Distance from the last recorded waypoint to the first recorded waypoint
 * at which perimeter_can_close() returns true. Chosen to match the span
 * of the PERIMETER_CLOSE_WINDOW search (15 points × 0.2 m = 3 m), ensuring
 * perimeter_close_track() will always find a good join when the flag is set.
 * NOTE: the perimeter is now SPARSE (one node per corner, no 0.2 m
 * densification), so "15 points × 0.2 m" is a legacy dense-spacing rationale;
 * the 3 m constant is retained.
 */
static constexpr float PERIMETER_CLOSE_DIST_M = 3.0f;


// ── Module state ──────────────────────────────────────────────────────────────

/** Raw recording buffer — x coordinates (ENU east, metres). */
static float s_rec_x[MAX_PERIMETER_POINTS];

/** Raw recording buffer — y coordinates (ENU north, metres). */
static float s_rec_y[MAX_PERIMETER_POINTS];

/** Number of waypoints accumulated in the current recording. */
static int s_rec_count = 0;

/** ENU east coordinate of the last committed waypoint. */
static float s_rec_last_x = 0.0f;

/** ENU north coordinate of the last committed waypoint. */
static float s_rec_last_y = 0.0f;

/** True while a recording session is active. */
static bool s_recording = false;

/** True when all three polygons are loaded and ready for use. */
static bool s_valid = false;

/** In-memory raw perimeter polygon (CCW). */
static Polygon s_perimeter;

/** In-memory navigation boundary polygon (CCW). */
static Polygon s_nav_boundary;

/** In-memory working area polygon (CCW). */
static Polygon s_working_area;

// ── Canonical (lat/lon) perimeter storage ─────────────────────────────────────
// The perimeter is persisted as absolute WGS-84 lat/lon (origin-independent) and
// re-derived to the ENU s_perimeter on boot against the current origin, so it can
// never drift relative to the live GPS position. Per-point accuracy feeds a
// confidence-aware breach margin. Default accuracy for points of unknown
// provenance (migrated legacy / PWA-drawn perimeters).
static constexpr float PERIM_LEGACY_ACC_M = 0.05f;

/** Scratch buffers for lat/lon (de)serialisation — reused at boot and on save. */
static double s_ll_lat[MAX_PERIMETER_POINTS];
static double s_ll_lon[MAX_PERIMETER_POINTS];
static float  s_ll_acc[MAX_PERIMETER_POINTS];

/** Worst-case (max) accuracy over the active perimeter points (m); 0 = perfect. */
static float  s_perim_acc_max = 0.0f;

/** Worst-case fix accuracy seen during the current recording session (m). */
static float  s_rec_acc_worst = 0.0f;

/** Map a GPS fix type to a conservative horizontal accuracy estimate (m). */
static float accuracy_for_fix(int fix_type) {
    switch (fix_type) {
        case GPS_FIX_RTK_FIXED: return 0.03f;  // ~3 cm
        case GPS_FIX_RTK_FLOAT: return 0.50f;  // ~30–50 cm
        case GPS_FIX_DGPS:      return 1.00f;
        case GPS_FIX_AUTO:      return 2.50f;
        default:                return 5.00f;  // no fix — very low confidence
    }
}

/**
 * @brief Derive the canonical lat/lon store from an ENU polygon and persist it.
 *
 * Converts each ENU vertex to absolute lat/lon via the current origin (the same
 * flat-earth transform live fixes use) and writes the "perim2" blob. Updates the
 * in-memory worst-case accuracy used by the breach margin. No-op (returns false)
 * if no origin is set or the polygon is out of range.
 */
bool perimeter_save_canonical(const Polygon &enu_poly, float acc) {
    GpsOrigin org = rtk_gps_get_origin();
    int m = (int)enu_poly.pts.size();
    if (!org.set || m < 3 || m > MAX_PERIMETER_POINTS) return false;
    for (int i = 0; i < m; i++) {
        double la, lo;
        rtk_gps_enu_to_latlon(enu_poly.pts[i].x, enu_poly.pts[i].y, &la, &lo);
        s_ll_lat[i] = la;
        s_ll_lon[i] = lo;
        s_ll_acc[i] = acc;
    }
    s_perim_acc_max = acc;
    return nvs_save_perimeter_ll(s_ll_lat, s_ll_lon, s_ll_acc, m);
}


// ── Private helpers ───────────────────────────────────────────────────────────

/**
 * @brief Build a Polygon from the recording buffers.
 *
 * Copies s_rec_count points from s_rec_x / s_rec_y into a new Polygon.
 * Does not validate or modify winding.
 */
static Polygon build_polygon_from_recording() {
    Polygon p;
    p.pts.reserve(s_rec_count);
    for (int i = 0; i < s_rec_count; i++) {
        p.pts.push_back({s_rec_x[i], s_rec_y[i]});
    }
    return p;
}

/**
 * @brief Clear all in-memory polygon state and the valid flag.
 */
static void clear_in_memory_state() {
    s_valid = false;
    s_perimeter.pts.clear();
    s_nav_boundary.pts.clear();
    s_working_area.pts.clear();
}


// ── Module lifecycle ──────────────────────────────────────────────────────────

void perimeter_init() {
    clear_in_memory_state();
    s_recording     = false;
    s_rec_count     = 0;
    s_perim_acc_max = 0.0f;

    // The ENU origin is restored in rtk_gps_init() (runs before this in setup()),
    // so it is available here for lat/lon ↔ ENU conversion.
    GpsOrigin origin = rtk_gps_get_origin();

    Polygon perim;  // ENU polygon we will navigate against

    // ── Preferred: canonical lat/lon store, re-derived to ENU against the
    //    current origin (origin-independent — fixes the power-up map mismatch). ──
    int n = 0;
    if (origin.set) {
        n = nvs_load_perimeter_ll(s_ll_lat, s_ll_lon, s_ll_acc, MAX_PERIMETER_POINTS);
    }

    if (n >= 3) {
        float acc_max = 0.0f;
        perim.pts.reserve(n);
        for (int i = 0; i < n; i++) {
            float e = 0.0f, no = 0.0f;
            rtk_gps_latlon_to_enu(s_ll_lat[i], s_ll_lon[i], &e, &no);
            perim.pts.push_back({e, no});
            if (s_ll_acc[i] > acc_max) acc_max = s_ll_acc[i];
        }
        s_perim_acc_max = acc_max;
        DBG_PRINTF("[PERIM] Loaded canonical lat/lon perimeter: %d pts, acc<=%.2f m\n",
                      n, (double)acc_max);
    } else {
        // ── Fallback: legacy ENU perimeter. Migrate it to lat/lon storage so the
        //    next boot is origin-independent. ──
        if (!nvs_has_valid_perimeter()) {
            DBG_PRINTLN("[PERIM] No valid perimeter in NVS — learn one before using AUTO mode");
            return;
        }
        perim = nvs_load_perimeter();
        if (perim.pts.empty()) {
            DBG_PRINTLN("[PERIM] NVS load failed for perimeter blob — re-learn perimeter");
            return;
        }
        if (origin.set) {
            perim.ensureCCW();
            if (perimeter_save_canonical(perim, PERIM_LEGACY_ACC_M)) {
                DBG_PRINTF("[PERIM] Migrated %d-pt legacy ENU perimeter to lat/lon storage\n",
                              (int)perim.pts.size());
            }
        } else {
            DBG_PRINTLN("[PERIM] No origin yet — using legacy ENU perimeter (will migrate once origin set)");
            s_perim_acc_max = PERIM_LEGACY_ACC_M;
        }
    }

    // ── Derive nav boundary + working area from the origin-consistent ENU
    //    perimeter (always recompute so they can never be origin-stale). ──
    perim.ensureCCW();
    Polygon nav  = insetPolygonClipper(perim, mower_config_nav_inset_m());
    Polygon work = insetPolygonClipper(nav,   mower_config_headland_m());
    if (nav.pts.empty() || work.pts.empty()) {
        DBG_PRINTLN("[PERIM] inset failed — perimeter may be too small; re-learn perimeter");
        return;
    }
    // Re-persist the origin-consistent ENU blobs so NVS-reading consumers (e.g.
    // ble_server build_map_json) get the same geometry the mower navigates — if
    // the origin shifted since the last save, the old ENU "perim" would otherwise
    // make the PWA map drift again. Sparse perimeter (~10 pts) → tiny writes.
    nvs_save_perimeter(perim);
    nvs_save_nav_boundary(nav);
    nvs_save_working_area(work);

    s_perimeter    = perim;
    s_nav_boundary = nav;
    s_working_area = work;
    s_valid        = true;

    DBG_PRINTF("[PERIM] Loaded: perim=%d pts, nav=%d pts, work=%d pts\n",
                  (int)s_perimeter.pts.size(),
                  (int)s_nav_boundary.pts.size(),
                  (int)s_working_area.pts.size());

    perimeter_log_unreachable_zones();
}


// ── Recording API ─────────────────────────────────────────────────────────────

void perimeter_start_recording() {
    // Clear any existing perimeter — new recording replaces old
    nvs_clear_perimeter();
    clear_in_memory_state();

    s_rec_count     = 0;
    s_rec_last_x    = 0.0f;
    s_rec_last_y    = 0.0f;
    s_rec_acc_worst = 0.0f;
    s_recording     = true;
    DBG_PRINTLN("[PERIM] Recording started (previous perimeter cleared)");
}

bool perimeter_record_point(float x, float y, int fix_type, bool force) {
    if (!s_recording) return false;

    // fix_type does NOT gate recording (position is used regardless of quality) —
    // it only feeds the worst-case accuracy used by the confidence-aware breach.
    if (!force) {
        // Distance gate: only record if far enough from last waypoint
        float dist = 0.0f;
        if (s_rec_count > 0) {
            float dx = x - s_rec_last_x;
            float dy = y - s_rec_last_y;
            dist = sqrtf(dx * dx + dy * dy);
        } else {
            // First point — always accept (treat as infinite distance)
            dist = RECORD_DIST_M;
        }
        if (dist < RECORD_DIST_M) return false;
    }
    if (s_rec_count >= MAX_PERIMETER_POINTS)        return false;

    s_rec_x[s_rec_count] = x;
    s_rec_y[s_rec_count] = y;
    s_rec_count++;
    s_rec_last_x = x;
    s_rec_last_y = y;

    float acc = accuracy_for_fix(fix_type);
    if (acc > s_rec_acc_worst) s_rec_acc_worst = acc;
    return true;
}

/**
 * @brief Post-process the recording buffer to close the perimeter track.
 *
 * Searches the first and last PERIMETER_CLOSE_WINDOW points for the closest
 * pair and discards surplus points outside that pair. Then appends a closing
 * point (copy of the first point) to seal the polygon.
 *
 * Called once when CH5 is deactivated, before polygon validation and save.
 * Modifies s_rec_x[], s_rec_y[], s_rec_count in-place.
 */
static void perimeter_close_track()
{
    int n = s_rec_count;
    if (n < MIN_RECORD_POINTS) {
        // Too short — leave unchanged; subsequent validation will reject.
        return;
    }

    int window = PERIMETER_CLOSE_WINDOW;
    if (window > n / 5) window = n / 5;
    if (window < 3)     window = 3;

    float min_dist = FLT_MAX;
    int best_i = 0;
    int best_j = n - 1;

    for (int i = 0; i < window; i++) {
        for (int j = n - window; j < n; j++) {
            if (j <= i) continue;
            float dx = s_rec_x[j] - s_rec_x[i];
            float dy = s_rec_y[j] - s_rec_y[i];
            float d  = sqrtf(dx * dx + dy * dy);
            if (d < min_dist) {
                min_dist = d;
                best_i   = i;
                best_j   = j;
            }
        }
    }

    int retained = best_j - best_i + 1;

    // Shift retained segment to the front of the buffer.
    if (best_i > 0) {
        memmove(s_rec_x, s_rec_x + best_i, retained * sizeof(float));
        memmove(s_rec_y, s_rec_y + best_i, retained * sizeof(float));
    }
    s_rec_count = retained;

    // Close the polygon: append a copy of the first point.
    // Guard: MAX_PERIMETER_POINTS must have room (retained < MAX_PERIMETER_POINTS).
    if (s_rec_count < MAX_PERIMETER_POINTS) {
        s_rec_x[s_rec_count] = s_rec_x[0];
        s_rec_y[s_rec_count] = s_rec_y[0];
        s_rec_count++;
    }

    DBG_PRINTF("[PERIM] Closed: discarded %d start pts, %d end pts, "
                  "join dist %.3f m\n",
                  best_i, (n - 1 - best_j), min_dist);
}

bool perimeter_finish_recording(char *error_msg) {
    if (!s_recording) {
        strncpy(error_msg, "not recording", 64);
        return false;
    }

    // Close the track using closest-endpoint algorithm (IMP-01).
    perimeter_close_track();

    // ── Validation check 1: minimum point count ────────────────────────────
    if (s_rec_count < MIN_RECORD_POINTS) {
        snprintf(error_msg, 64, "too few points (%d, need %d)", s_rec_count, MIN_RECORD_POINTS);
        return false;
    }

    // Build polygon from recording buffer
    Polygon poly = build_polygon_from_recording();
    poly.ensureCCW();

    // ── Validation check 2: minimum area ──────────────────────────────────
    float area = poly.area();
    if (area < MIN_RECORD_AREA_M2) {
        snprintf(error_msg, 64, "area too small (%.1f m2, need %.1f)", area, MIN_RECORD_AREA_M2);
        return false;
    }

    // ── Validation check 3: auto-clean self-intersecting loops / spurs ───────
    // The robot may cross the recorded path at a gate or narrow entrance.
    // Split at intersections and keep the largest CCW sub-polygon (main garden).
    // CW-wound fragments (spurs/folds) are discarded because splitSelfIntersecting
    // now filters on signed area — negative area ⟹ discard.
    if (isSelfIntersecting(poly)) {
        auto cleaned = splitSelfIntersecting(poly);
        if (cleaned.empty()) {
            strncpy(error_msg, "polygon self-intersects (could not auto-clean)", 64);
            return false;
        }
        // Keep the sub-polygon with the largest positive (CCW) area.
        int best = 0;
        for (int i = 1; i < (int)cleaned.size(); i++) {
            if (cleaned[i].area() > cleaned[best].area()) best = i;
        }
        poly = cleaned[best];
        poly.ensureCCW();
        DBG_PRINTF("[PERIM] Self-intersecting perimeter auto-cleaned: %d pts → %d pts (%.1f m2)\n",
                   s_rec_count, (int)poly.pts.size(), poly.area());
    }

    // ── Validation check 4: NAV inset produces valid polygon ───────────────
    Polygon nav = insetPolygonClipper(poly, mower_config_nav_inset_m());
    if (nav.pts.empty() || nav.area() <= 0.0f) {
        strncpy(error_msg, "perimeter too small for robot (nav inset fails)", 64);
        return false;
    }

    // ── Derive working area ────────────────────────────────────────────────
    Polygon work = insetPolygonClipper(nav, mower_config_headland_m());
    if (work.pts.empty() || work.area() <= 0.0f) {
        strncpy(error_msg, "perimeter too small for robot (headland inset fails)", 64);
        return false;
    }

    // ── Save all three polygons to NVS atomically ─────────────────────────
    // "Atomically" here means all three are written in one finish operation;
    // individual NVS writes are not transactional but the sequence always runs
    // to completion or returns false so the caller can retry.
    if (!nvs_save_perimeter(poly)) {
        strncpy(error_msg, "NVS write failed for perimeter", 64);
        return false;
    }
    // Canonical lat/lon store (origin-independent). Worst-case fix accuracy from
    // this session feeds the confidence-aware breach margin. A failure here is
    // non-fatal: the ENU perimeter above still navigates; perimeter_init() will
    // migrate it on the next boot.
    if (!perimeter_save_canonical(poly,
            s_rec_acc_worst > 0.0f ? s_rec_acc_worst : PERIM_LEGACY_ACC_M)) {
        DBG_PRINTLN("[PERIM] WARNING: canonical lat/lon save failed (ENU perimeter still saved)");
    }
    if (!nvs_save_nav_boundary(nav)) {
        strncpy(error_msg, "NVS write failed for nav boundary", 64);
        return false;
    }
    if (!nvs_save_working_area(work)) {
        strncpy(error_msg, "NVS write failed for working area", 64);
        return false;
    }

    // ── Update in-memory state ─────────────────────────────────────────────
    s_perimeter    = poly;
    s_nav_boundary = nav;
    s_working_area = work;
    s_valid        = true;
    s_recording    = false;
    s_rec_count    = 0;

    DBG_PRINTF("[PERIM] Saved: perim=%d pts (%.1f m2), nav=%d pts, work=%d pts\n",
                  (int)poly.pts.size(), area,
                  (int)nav.pts.size(),
                  (int)work.pts.size());

    perimeter_log_unreachable_zones();
    return true;
}

void perimeter_abort_recording() {
    s_recording = false;
    s_rec_count = 0;
    DBG_PRINTLN("[PERIM] Recording aborted");
}

void perimeter_save_partial() {
    if (s_rec_count == 0) {
        perimeter_abort_recording();
        return;
    }
    Polygon poly = build_polygon_from_recording();
    nvs_save_perimeter(poly);
    // Store in memory but mark as NOT valid for navigation/planning
    s_perimeter = poly;
    s_nav_boundary.pts.clear();
    s_working_area.pts.clear();
    s_valid     = false;
    s_recording = false;
    DBG_PRINTF("[PERIM] Saved %d partial point(s) to NVS\n", (int)poly.pts.size());
    s_rec_count = 0;
}

bool perimeter_is_recording() {
    return s_recording;
}

int perimeter_recording_point_count() {
    return s_rec_count;
}

Polygon perimeter_get_recording_points() {
    Polygon p;
    if (s_recording && s_rec_count > 0) {
        p.pts.reserve(s_rec_count);
        for (int i = 0; i < s_rec_count; i++) {
            p.pts.push_back({s_rec_x[i], s_rec_y[i]});
        }
    }
    return p;
}

bool perimeter_can_close() {
    // Require a minimum number of points so the robot cannot trigger closure
    // immediately after starting (when it is still at the origin point).
    if (!s_recording || s_rec_count < MIN_RECORD_POINTS) return false;

    // Compare the last committed waypoint against the very first recorded point.
    // s_rec_last_x/y tracks where the robot most recently was (within 0.2 m).
    float dx = s_rec_last_x - s_rec_x[0];
    float dy = s_rec_last_y - s_rec_y[0];
    return sqrtf(dx * dx + dy * dy) <= PERIMETER_CLOSE_DIST_M;
}


// ── Stored polygon access ─────────────────────────────────────────────────────

bool perimeter_is_valid() {
    return s_valid;
}

Polygon perimeter_get_perimeter() {
    return s_perimeter;
}

Polygon perimeter_get_nav_boundary() {
    return s_nav_boundary;
}

Polygon perimeter_get_working_area() {
    return s_working_area;
}

float perimeter_get_accuracy_m() {
    return s_perim_acc_max;
}

void perimeter_clear() {
    nvs_clear_perimeter();
    clear_in_memory_state();
    DBG_PRINTLN("[PERIM] Perimeter cleared from NVS and memory");
}

void perimeter_recompute() {
    if (!nvs_has_valid_perimeter()) return;

    Polygon poly = nvs_load_perimeter();
    if (poly.pts.empty()) {
        DBG_PRINTLN("[PERIM] recompute: failed to load raw perimeter from NVS");
        return;
    }

    // NVS perimeter is already smoothed from perimeter_finish_recording().
    // No re-smoothing needed here — corners are already arcs.

    Polygon nav  = insetPolygonClipper(poly, mower_config_nav_inset_m());
    Polygon work = insetPolygonClipper(nav,  mower_config_headland_m());

    if (nav.pts.empty() || work.pts.empty()) {
        DBG_PRINTLN("[PERIM] recompute: inset failed — perimeter may be too small for new config");
        return;
    }

    nvs_save_nav_boundary(nav);
    nvs_save_working_area(work);

    s_nav_boundary = nav;
    s_working_area = work;

    DBG_PRINTF("[PERIM] Recomputed: nav=%d pts, work=%d pts\n",
                  (int)nav.pts.size(), (int)work.pts.size());
}


// ── Unreachable zone detection ────────────────────────────────────────────────

void perimeter_log_unreachable_zones() {
    if (!s_valid || s_perimeter.pts.empty() || s_nav_boundary.pts.empty()) return;

    // ── Compute bounding box of perimeter ─────────────────────────────────
    float minx = s_perimeter.pts[0].x;
    float maxx = minx;
    float miny = s_perimeter.pts[0].y;
    float maxy = miny;

    for (const auto &pt : s_perimeter.pts) {
        if (pt.x  < minx) minx = pt.x;
        if (pt.x  > maxx) maxx = pt.x;
        if (pt.y < miny) miny = pt.y;
        if (pt.y > maxy) maxy = pt.y;
    }

    // ── Scan at 1 m grid, collect unreachable points ───────────────────────
    // Unreachable = inside perimeter polygon but outside nav boundary polygon.
    // These are corner/edge regions < mower_config_nav_inset_m() from an edge.
    static float ux[MAX_UNREACHABLE_PTS];
    static float uy[MAX_UNREACHABLE_PTS];
    int ucnt = 0;

    // Start grid at minx+0.5 so samples are cell centres, not edges
    for (float gy = miny + 0.5f; gy <= maxy && ucnt < MAX_UNREACHABLE_PTS; gy += UNREACHABLE_SCAN_STEP_M) {
        for (float gx = minx + 0.5f; gx <= maxx && ucnt < MAX_UNREACHABLE_PTS; gx += UNREACHABLE_SCAN_STEP_M) {
            if (isInsidePolygon(s_perimeter, gx, gy) &&
               !isInsidePolygon(s_nav_boundary, gx, gy)) {
                ux[ucnt] = gx;
                uy[ucnt] = gy;
                ucnt++;
            }
        }
    }

    if (ucnt == 0) return;  // all interior is reachable — no need to log anything

    // ── Greedy radius-based clustering ────────────────────────────────────
    // Each unreachable point is assigned a cluster ID. We start by seeding an
    // unassigned point as a new cluster, then propagate repeatedly until no
    // more neighbours within CLUSTER_RADIUS are unassigned.
    static int cluster[MAX_UNREACHABLE_PTS];
    for (int i = 0; i < ucnt; i++) cluster[i] = -1;

    const float radius2 = UNREACHABLE_CLUSTER_RADIUS_M * UNREACHABLE_CLUSTER_RADIUS_M;
    int num_clusters = 0;

    for (int i = 0; i < ucnt; i++) {
        if (cluster[i] >= 0) continue;  // already assigned

        const int cid = num_clusters++;
        cluster[i] = cid;

        // Flood fill: keep propagating until no new members are added
        bool changed = true;
        while (changed) {
            changed = false;
            for (int j = 0; j < ucnt; j++) {
                if (cluster[j] >= 0) continue;  // already in a cluster
                // Check if j is close to any existing member of cid
                for (int k = 0; k < ucnt; k++) {
                    if (cluster[k] != cid) continue;
                    float dx = ux[j] - ux[k];
                    float dy = uy[j] - uy[k];
                    if (dx * dx + dy * dy <= radius2) {
                        cluster[j] = cid;
                        changed = true;
                        break;  // j is now in cid; move to next j
                    }
                }
            }
        }
    }

    // ── Compute cluster centres and log ───────────────────────────────────
    for (int c = 0; c < num_clusters; c++) {
        float cx = 0.0f, cy = 0.0f;
        int   cnt = 0;

        for (int i = 0; i < ucnt; i++) {
            if (cluster[i] == c) {
                cx += ux[i];
                cy += uy[i];
                cnt++;
            }
        }

        if (cnt == 0) continue;
        cx /= (float)cnt;
        cy /= (float)cnt;

        // Area approximation: each grid cell covers SCAN_STEP² m²
        float approx_area = (float)cnt * (UNREACHABLE_SCAN_STEP_M * UNREACHABLE_SCAN_STEP_M);

        DBG_PRINTF("[PERIM] Unreachable zone ~(%.1f, %.1f), approx area %.0f m²\n",
                      cx, cy, approx_area);
    }
}
