// ══════════════════════════════════════════════════════════════════════════════
//  coverage_planner.cpp — RoboMower complete mowing path planner (SPIRAL)
//
//  2026-06-13: REPLACED the boustrophedon strip planner with a concentric
//  inward spiral. The strip planner (scan-line strips + headland passes +
//  region-outline passes + spur reverses + U-turn transitions) repeatedly
//  produced corrupt plans — fans of lines converging to a couple of points,
//  paths leaving the perimeter — and never planned a sparse perimeter cleanly.
//
//  The spiral is dramatically simpler and is *native to a sparse perimeter*
//  (the perimeter is ~10 turn-point nodes joined by straight edges):
//
//    ring 0 = the perimeter itself (steering-centre limit; no nav-inset subtracted)
//    ring i = ring (i-1) inset inward by one strip step
//    … until the polygon collapses to nothing at the centre.
//
//  Each ring is emitted as a closed loop of waypoints at the polygon vertices;
//  the node follower drives straight between nodes and pivots at corners. The
//  next ring starts at the vertex nearest the previous ring's start, so the
//  inward steps stack on one side and the whole path reads as a spiral.
//
//  offsetPolygonClipper() (Clipper2) handles concave gardens that pinch into
//  multiple lobes as they shrink — each lobe is spiralled to its own centre
//  (depth-first) before the next.
//
//  Why ring 0 IS the perimeter (and not an inset of it): the recorded perimeter
//  is the steering-centre LIMIT — the path of the steering centre driven to its
//  maximum extent (body against the physical boundary), so the robot's
//  diagonal-radius margin is already baked into the recording. The steering
//  centre may drive right up to the perimeter, so no nav-inset is subtracted
//  (subtracting one would double-count the robot size and leave an oversized
//  outer border).
//
//  Source references:
//    clipper_offset.h  offsetPolygonClipper()
//    config.h    strip step (CUT_WIDTH_M − STRIP_OVERLAP_M), SPIRAL_RING_MIN_AREA_M2
// ══════════════════════════════════════════════════════════════════════════════

#include "coverage_planner.h"
#include "config.h"
#include "mower_config.h"
#include "obstacle_map.h"
#include "geometry.h"
#include "clipper_offset.h"

#include <Arduino.h>
#include <vector>
#include <cmath>

// ── Constants ─────────────────────────────────────────────────────────────────

// Skip radius for obstacle vicinity — derived from MIN_ZONE_AREA_M2 (see spec).
// sqrt(0.5 / π) ≈ 0.399 m → round to 0.40 m.
static constexpr float OBSTACLE_SKIP_RADIUS_M = 0.40f;

// Maximum unreachable zones we store (static allocation).
static constexpr int MAX_UNREACHABLE_ZONES = 32;

// Safety caps on the spiral generation.
//  MAX_PLAN_WP : leave headroom under the follower's 1000-waypoint buffer
//                (state_machine.cpp s_wp_buf[1000]); a sparse garden uses far
//                fewer (≈ rings × nodes_per_ring).
//  MAX_RINGS   : backstop against a degenerate inset that never collapses.
static constexpr int MAX_PLAN_WP = 960;
static constexpr int MAX_RINGS   = 400;

// Minimum enclosed area for a spiral ring to be emitted. Much smaller than
// MIN_ZONE_AREA_M2 (0.5 m²): the spiral must inset all the way to the centre, so
// rings keep going until the region is barely larger than the blade can cover in
// one pass. Stopping at 0.5 m² left a multi-cut-width void down the middle. At
// ~0.04 m² (≈ 0.2 m across) the last ring's blade (≈ cut_width/2 reach) covers
// the residual centre.
static constexpr float SPIRAL_RING_MIN_AREA_M2 = 0.04f;


// ── Module state ──────────────────────────────────────────────────────────────

static std::vector<Waypoint> g_waypoints;

// g_wp_index: index of the NEXT waypoint to return via coverage_planner_get_next().
// Ranges from 0 (plan just generated) to g_total_wp_count (plan complete).
static int g_wp_index           = 0;

// g_headland_wp_end_idx: exclusive upper bound of "headland" waypoints. For the
// spiral every ring is an edge-following pass, so the whole plan is headland —
// this is set equal to g_total_wp_count. (Kept for the progress getters and so
// the uncertainty strip-truncation in state_machine.cpp, which is exempt for
// headland waypoints, leaves the spiral intact near the boundary.)
static int g_headland_wp_end_idx = 0;

// g_total_wp_count: total waypoints in the plan (== g_waypoints.size()).
static int g_total_wp_count      = 0;

// Unreachable zone log — retained for API/diagnostic compatibility. The spiral
// covers everything an inset can reach, so this is left empty.
struct UnreachableZone {
    float cx;      ///< Centre ENU east  (m)
    float cy;      ///< Centre ENU north (m)
    float area_m2; ///< Approximate area (m²)
};
static UnreachableZone g_unreachable[MAX_UNREACHABLE_ZONES];
static int             g_unreachable_count = 0;


// ── Ring helper ────────────────────────────────────────────────────────────────

/** Area-weighted centroid of a polygon (falls back to vertex 0 if degenerate). */
static Point polygonCentroid(const Polygon &p) {
    int n = (int)p.pts.size();
    if (n == 0) return Point(0.0f, 0.0f);
    double cx = 0.0, cy = 0.0, a2 = 0.0;
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        double cr = (double)p.pts[i].x * p.pts[j].y - (double)p.pts[j].x * p.pts[i].y;
        a2 += cr;
        cx += (p.pts[i].x + p.pts[j].x) * cr;
        cy += (p.pts[i].y + p.pts[j].y) * cr;
    }
    if (fabs(a2) < 1e-6) return p.pts[0];
    return Point((float)(cx / (3.0 * a2)), (float)(cy / (3.0 * a2)));
}

/**
 * Append a full closed-loop traversal of `ring` (assumed CCW) to g_waypoints.
 *
 * Traversal starts at the vertex nearest `seed`, so consecutive spiral rings
 * join with a short inward step instead of a chord across the garden. Emits
 * n+1 waypoints (returns to the start vertex) so the final edge is driven.
 *
 * @return the start vertex of this ring — pass it as the seed for the next ring.
 */
static Point addRingWaypoints(const Polygon &ring, Point seed,
                              bool mowing, bool headland) {
    int n = (int)ring.pts.size();
    if (n < 2) return seed;

    // Start at the vertex nearest the seed (squared distance — no sqrt needed).
    int   start = 0;
    float best  = 1e30f;
    for (int i = 0; i < n; i++) {
        float dx = ring.pts[i].x - seed.x;
        float dy = ring.pts[i].y - seed.y;
        float d  = dx * dx + dy * dy;
        if (d < best) { best = d; start = i; }
    }

    for (int k = 0; k <= n; k++) {
        int ci = (start + k)     % n;
        int ni = (start + k + 1) % n;
        float dx = ring.pts[ni].x - ring.pts[ci].x;
        float dy = ring.pts[ni].y - ring.pts[ci].y;

        Waypoint wp = {};
        wp.x        = ring.pts[ci].x;
        wp.y        = ring.pts[ci].y;
        wp.heading  = atan2f(dy, dx);
        wp.mowing   = mowing;
        wp.headland = headland;
        wp.reverse  = false;
        g_waypoints.push_back(wp);
    }
    return ring.pts[start];
}


// ── coverage_planner_init ─────────────────────────────────────────────────────

void coverage_planner_init() {
    g_waypoints.clear();
    g_wp_index            = 0;
    g_headland_wp_end_idx = 0;
    g_total_wp_count      = 0;
    g_unreachable_count   = 0;
}


// ── coverage_planner_plan ─────────────────────────────────────────────────────

bool coverage_planner_plan(const Polygon &perimeter,
                           const Polygon &navBoundary,
                           const Polygon &workingArea) {
    (void)navBoundary;   // spiral & breach derive directly from the perimeter now
    (void)workingArea;   // working area / headland split is no longer used

    // ── Validate ────────────────────────────────────────────────────────────
    if (perimeter.pts.size() < 3) {
        DBG_PRINTLN("[CP] plan: perimeter < 3 pts");
        return false;
    }
    if (fabsf(perimeter.area()) < MIN_ZONE_AREA_M2) {
        DBG_PRINTLN("[CP] plan: perimeter too small");
        return false;
    }

    // ── Reset state ───────────────────────────────────────────────────────────
    g_waypoints.clear();
    g_waypoints.reserve(256);
    g_wp_index            = 0;
    g_headland_wp_end_idx = 0;
    g_total_wp_count      = 0;
    g_unreachable_count   = 0;

    const float step = mower_config_strip_step_m();   // CUT_WIDTH_M - STRIP_OVERLAP_M

    // -- Concentric inward spiral (Clipper2 offsetting) -----------------------
    // The recorded PERIMETER is the path of the robot's STEERING CENTRE driven to
    // its maximum extent (body against the physical boundary). The physical fence
    // is already ~one robot diagonal-radius OUTSIDE the perimeter, so that margin
    // is baked into the recording: the steering centre may drive right up to the
    // perimeter. Therefore:
    //   * ring 0 IS the perimeter (NO nav-inset subtracted -- subtracting it would
    //     double-count the robot size and leave an oversized outer border),
    //   * ring k>0 = the perimeter inset by k*step (CUT_WIDTH - OVERLAP),
    // giving uniform ring spacing from the edge inward.
    //
    // Offsets keep ALL sub-regions, so a garden that pinches into separate areas
    // (e.g. a neck to a side arm) is fully covered -- each area gets its own
    // concentric spiral. Clipper2 is robust where the hand-rolled insetPolygon was
    // not: concave notches round off (no inward spike), an over-shrink collapses
    // to EMPTY (loop ends), and a pinch returns multiple regions. Rings continue
    // down to SPIRAL_RING_MIN_AREA_M2, then plunges to the centre.
    //
    // DEPTH-FIRST so the path is a continuous spiral: each ring is the previous
    // ring inset by one step (Clipper offset of the previous ring, not of the
    // original), followed inward without lifting. Consecutive rings are joined by
    // a short radial BRIDGE (the ~step gap between a ring's start and the next
    // ring's start is just driven, drawn connected). Only a LONG jump — to a
    // different area when the garden pinches into lobes — gets a mowing=false
    // break so the PWA doesn't draw a line across the garden. A stack holds
    // not-yet-spiralled child regions, so each lobe is completed before the next.
    const float BRIDGE_MAX = 3.0f * step;   // hops longer than this break instead of bridge

    std::vector<Polygon> stack;
    Polygon outer = perimeter;
    outer.ensureCCW();
    stack.push_back(outer);

    Point seed     = outer.pts.empty() ? Point(0.0f, 0.0f) : outer.pts[0];
    Point prevEnd  = seed;
    bool  havePrev = false;
    int   rings    = 0;
    bool  capped   = false;

    while (!stack.empty()) {
        if ((int)g_waypoints.size() >= MAX_PLAN_WP || rings >= MAX_RINGS) {
            capped = true;
            break;
        }

        Polygon ring = stack.back();
        stack.pop_back();
        if (ring.pts.size() < 3 || fabsf(ring.area()) < SPIRAL_RING_MIN_AREA_M2) continue;

        // Where this ring's loop will start (vertex nearest the running seed).
        int   n = (int)ring.pts.size(), startIdx = 0;
        float best = 1e30f;
        for (int i = 0; i < n; i++) {
            float dx = ring.pts[i].x - seed.x, dy = ring.pts[i].y - seed.y;
            float d  = dx * dx + dy * dy;
            if (d < best) { best = d; startIdx = i; }
        }
        Point startPt = ring.pts[startIdx];

        // Bridge (short, continuous) vs break (long jump to another lobe).
        if (havePrev) {
            float dx = startPt.x - prevEnd.x, dy = startPt.y - prevEnd.y;
            if (sqrtf(dx * dx + dy * dy) > BRIDGE_MAX) {
                Waypoint brk = g_waypoints.back();
                brk.mowing = false;
                g_waypoints.push_back(brk);
            }
        }

        // Whole spiral flagged headland=true: edge-following passes that are
        // exempt from the uncertainty strip-truncation in state_machine.cpp.
        seed     = addRingWaypoints(ring, seed, /*mowing=*/true, /*headland=*/true);
        prevEnd  = startPt;   // closed loop ends where it started
        havePrev = true;
        rings++;

        // Inset this ring by one step → child ring(s) further in.
        std::vector<Polygon> kids = offsetPolygonClipper(ring, step);
        int pushed = 0;
        for (auto &c : kids) {
            if (c.pts.size() >= 3 && fabsf(c.area()) >= SPIRAL_RING_MIN_AREA_M2) {
                stack.push_back(c);
                pushed++;
            }
        }
        if (pushed == 0) {
            // Innermost ring of this area: plunge to its centre so the residual
            // middle (inside the last ring, beyond the blade's reach) is cut.
            Point c = polygonCentroid(ring);
            Waypoint wp = {};
            wp.x = c.x; wp.y = c.y; wp.heading = 0.0f;
            wp.mowing = true; wp.headland = true; wp.reverse = false;
            g_waypoints.push_back(wp);
            prevEnd = c;
        }
    }

    // ── Finalise ──────────────────────────────────────────────────────────────
    g_headland_wp_end_idx = (int)g_waypoints.size();   // entire plan is edge-following
    g_total_wp_count      = (int)g_waypoints.size();

    if (capped) {
        DBG_PRINTF("[CP] spiral CAPPED at %d rings / %d wp (garden larger than buffer)\n",
                      rings, g_total_wp_count);
    }
    DBG_PRINTF("[CP] spiral plan: %d rings, %d waypoints, step=%.2f m\n",
                  rings, g_total_wp_count, step);

    return g_total_wp_count > 0;
}


// ── coverage_planner_get_next ─────────────────────────────────────────────────

bool coverage_planner_get_next(Waypoint &wp) {
    if (g_wp_index >= g_total_wp_count) return false;
    wp = g_waypoints[g_wp_index++];
    return true;
}


// ── coverage_planner_report_obstacle ─────────────────────────────────────────

void coverage_planner_report_obstacle(float x, float y, float approach_heading) {
    // Record obstacle in the obstacle map (also marks grid cell as OBSTACLE).
    obstacle_add(x, y, approach_heading);

    // Skip any remaining waypoints within OBSTACLE_SKIP_RADIUS_M of (x,y).
    // Advance g_wp_index forward past the blocked region so the robot does
    // not attempt to re-enter the obstacle vicinity.
    while (g_wp_index < g_total_wp_count) {
        const Waypoint &wp = g_waypoints[g_wp_index];
        float dx = wp.x - x;
        float dy = wp.y - y;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist > OBSTACLE_SKIP_RADIUS_M) break;  // safe waypoint reached
        g_wp_index++;
    }
}


// ── coverage_planner_reset_to_nearest ────────────────────────────────────────

void coverage_planner_reset_to_nearest(float x, float y) {
    if (g_total_wp_count == 0) return;

    // Search the ENTIRE waypoint list for the nearest point. g_wp_index is NOT a
    // reliable "progress" cursor here: load_waypoints_from_planner() drains the planner
    // via get_next(), leaving g_wp_index == g_total_wp_count. Searching only from
    // g_wp_index (the old behaviour) scanned an empty range, left the index at the end,
    // and made the caller's subsequent reload return 0 waypoints — silently ending
    // AUTO. The state machine's own s_wp_index is the authoritative progress tracker,
    // so resuming at the globally-nearest waypoint and re-handing the tail
    // [nearest .. end] is the correct recovery.
    float best_dist = 1e9f;
    int   best_idx  = 0;

    for (int i = 0; i < g_total_wp_count; i++) {
        float dx = g_waypoints[i].x - x;
        float dy = g_waypoints[i].y - y;
        float d  = sqrtf(dx * dx + dy * dy);
        if (d < best_dist) {
            best_dist = d;
            best_idx  = i;
        }
    }

    g_wp_index = best_idx;
}


// ── Progress reporting ────────────────────────────────────────────────────────

float coverage_planner_get_headland_progress() {
    if (g_headland_wp_end_idx <= 0) return 1.0f;  // no headland planned
    int visited = (g_wp_index < g_headland_wp_end_idx)
                  ? g_wp_index
                  : g_headland_wp_end_idx;
    return (float)visited / (float)g_headland_wp_end_idx;
}

float coverage_planner_get_strip_progress() {
    int strip_total = g_total_wp_count - g_headland_wp_end_idx;
    if (strip_total <= 0) return 1.0f;
    int strip_visited = g_wp_index - g_headland_wp_end_idx;
    if (strip_visited < 0) strip_visited = 0;
    if (strip_visited > strip_total) strip_visited = strip_total;
    return (float)strip_visited / (float)strip_total;
}

float coverage_planner_get_total_progress() {
    if (g_total_wp_count <= 0) return 1.0f;
    int idx = (g_wp_index < g_total_wp_count) ? g_wp_index : g_total_wp_count;
    return (float)idx / (float)g_total_wp_count;
}

bool coverage_planner_is_complete() {
    return (g_total_wp_count > 0) && (g_wp_index >= g_total_wp_count);
}


// ── Waypoint access ──────────────────────────────────────────────────────────

const std::vector<Waypoint>& coverage_planner_get_waypoints() {
    return g_waypoints;
}

int coverage_planner_get_waypoint_count() {
    return g_total_wp_count;
}

// ── Diagnostics ───────────────────────────────────────────────────────────────

int coverage_planner_get_unreachable_zone_count() {
    return g_unreachable_count;
}

void coverage_planner_log_unreachable_zones() {
    if (g_unreachable_count == 0) {
        DBG_PRINTLN("[CP] unreachable zones: none");
        return;
    }
    DBG_PRINT("[CP] unreachable zones (");
    DBG_PRINT(g_unreachable_count);
    DBG_PRINTLN("):");
    for (int i = 0; i < g_unreachable_count; i++) {
        DBG_PRINT("  [");
        DBG_PRINT(i);
        DBG_PRINT("] centre=(");
        DBG_PRINT(g_unreachable[i].cx, 2);
        DBG_PRINT(", ");
        DBG_PRINT(g_unreachable[i].cy, 2);
        DBG_PRINT(") area=");
        DBG_PRINT(g_unreachable[i].area_m2, 2);
        DBG_PRINTLN(" m2");
    }
}

uint32_t coverage_planner_get_session_mowed_dm2() {
    return grid_get_session_mowed_dm2();
}

int coverage_planner_get_waypoint_index() {
    return g_wp_index;
}

void coverage_planner_skip_to_waypoint(int idx) {
    if (idx < 0) idx = 0;
    if (idx > g_total_wp_count) idx = g_total_wp_count;
    g_wp_index = idx;
}
