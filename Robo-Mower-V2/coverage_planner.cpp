// ══════════════════════════════════════════════════════════════════════════════
//  coverage_planner.cpp — RoboMower complete mowing path planner
//
//  Implements the five-step path generation algorithm:
//    Step 1  Headland passes    (outermost → inward, CW traversal)
//    Step 2  Optimal mow angle  (convex hull edge enumeration)
//    Step 3  Strip generation   (scan-line intersection in rotated frame)
//    Step 4  Boustrophedon seq  (alternating direction, concave-safe)
//    Step 5  Transitions        (guided 2-point turn for arc configs)
//
//  Arc-sweep credit (Steps 6/7) is disabled for STEER_CENTRE_TO_CUT_CENTRE_M <= 0.
//
//  Source references:
//    Spec: Robo_Mower_claudecode_prompt_v3.md §Coverage Planner
//    Assumptions: ASSUMPTIONS.md A09, A19
//    Handoff: HANDOFFS/15_coverage_planner/HANDOFF.md
// ══════════════════════════════════════════════════════════════════════════════

#include "coverage_planner.h"
#include "config.h"
#include "mower_config.h"
#include "obstacle_map.h"
#include "geometry.h"

#include <Arduino.h>
#include <vector>
#include <algorithm>
#include <climits>
#include <cmath>

// ── Constants ─────────────────────────────────────────────────────────────────

// Skip radius for obstacle vicinity — derived from MIN_ZONE_AREA_M2 (see spec).
// sqrt(0.5 / π) ≈ 0.399 m → round to 0.40 m.
static constexpr float OBSTACLE_SKIP_RADIUS_M = 0.40f;

// Maximum unreachable zones we store (static allocation).
static constexpr int MAX_UNREACHABLE_ZONES = 32;

// Arc-sweep credit is disabled for STEER_CENTRE_TO_CUT_CENTRE_M <= 0 (spec §Step 6).
// Current value = 0.0f (front-drive config), so no arc-sweep credit is applied.

// Spacing between consecutive strips — computed at runtime from mower_config.
// Use mower_config_strip_step_m() at each call site.


// ── Module state ──────────────────────────────────────────────────────────────

static std::vector<Waypoint> g_waypoints;

// g_wp_index: index of the NEXT waypoint to return via coverage_planner_get_next().
// Ranges from 0 (plan just generated) to g_total_wp_count (plan complete).
static int g_wp_index           = 0;

// g_headland_wp_end_idx: exclusive upper bound of headland waypoints.
// Waypoints [0 .. g_headland_wp_end_idx-1] are headland passes.
// Waypoints [g_headland_wp_end_idx .. g_total_wp_count-1] are strips + transitions.
static int g_headland_wp_end_idx = 0;

// g_total_wp_count: total waypoints in the plan (== g_waypoints.size()).
static int g_total_wp_count      = 0;

// Unreachable zone log (working-area sub-regions with no valid strip segments).
struct UnreachableZone {
    float cx;      ///< Centre ENU east  (m)
    float cy;      ///< Centre ENU north (m)
    float area_m2; ///< Approximate area (m²)
};
static UnreachableZone g_unreachable[MAX_UNREACHABLE_ZONES];
static int             g_unreachable_count = 0;


// ── Internal geometry helpers ─────────────────────────────────────────────────

/** Rotate point (x,y) by angle radians CCW around the origin. */
static Point rotatePoint(float x, float y, float angle) {
    float ca = cosf(angle);
    float sa = sinf(angle);
    return { x * ca - y * sa,
             x * sa + y * ca };
}

/**
 * Intersect horizontal line y = yl with the polygon defined by pts.
 *
 * Uses the even-odd scan-line rule:  an edge (y0→y1) contributes when
 * exactly one endpoint is strictly below yl.  This avoids double-counting
 * at shared vertices and is consistent with the standard polygon fill rule.
 *
 * @return Sorted vector of x-intercepts. Length is always even for a
 *         well-formed polygon, forming (x_enter, x_exit) pairs.
 */
static std::vector<float> hLineIntersect(
        const std::vector<Point> &pts, float yl) {
    std::vector<float> xs;
    int n = (int)pts.size();
    for (int i = 0; i < n; i++) {
        float x0 = pts[i].x,          y0 = pts[i].y;
        float x1 = pts[(i + 1) % n].x, y1 = pts[(i + 1) % n].y;
        // Edge contributes when yl lies strictly between y0 and y1.
        if ((y0 < yl && y1 >= yl) || (y1 < yl && y0 >= yl)) {
            float t = (yl - y0) / (y1 - y0);
            xs.push_back(x0 + t * (x1 - x0));
        }
    }
    std::sort(xs.begin(), xs.end());
    return xs;
}


/**
 * Intersect vertical line x = xl with the polygon defined by pts.
 *
 * Same even-odd rule as hLineIntersect, transposed: an edge (x0→x1) contributes
 * when exactly one endpoint is strictly left of xl.
 *
 * @return Sorted vector of y-intercepts. Length is always even for a
 *         well-formed polygon, forming (y_enter, y_exit) pairs.
 */
static std::vector<float> vLineIntersect(
        const std::vector<Point> &pts, float xl) {
    std::vector<float> ys;
    int n = (int)pts.size();
    for (int i = 0; i < n; i++) {
        float x0 = pts[i].x,          y0 = pts[i].y;
        float x1 = pts[(i + 1) % n].x, y1 = pts[(i + 1) % n].y;
        if ((x0 < xl && x1 >= xl) || (x1 < xl && x0 >= xl)) {
            float t = (xl - x0) / (x1 - x0);
            ys.push_back(y0 + t * (y1 - y0));
        }
    }
    std::sort(ys.begin(), ys.end());
    return ys;
}


// ── Headland pass helpers ─────────────────────────────────────────────────────

/**
 * Append waypoints for a clockwise traversal of poly to g_waypoints.
 *
 * poly is assumed CCW (as returned by insetPolygon).  Reversing gives CW.
 * Adds n+1 waypoints: n vertices (CW order) plus a closing waypoint that
 * returns to vertex 0, ensuring the final edge is fully traversed.
 *
 * No corner smoothing is applied at the planning level.  Sharp corners are
 * emitted as-is; the path controller handles actual cornering by slowing
 * down (OpenMower-style).  This ensures the planned path never crosses the
 * perimeter — it follows inset polygon vertices exactly.
 */
static void addPolygonWaypoints(const Polygon &poly, bool mowing, bool headland) {
    int n = (int)poly.pts.size();
    if (n < 2) return;

    // CW index helper: maps sequential i to original CCW array index (reversed).
    auto cwIdx = [n](int i) -> int { return (n - 1) - (((i % n) + n) % n); };

    for (int i = 0; i <= n; i++) {
        int ci = cwIdx(i);
        int ni = cwIdx(i + 1);

        float dx = poly.pts[ni].x - poly.pts[ci].x;
        float dy = poly.pts[ni].y - poly.pts[ci].y;

        Waypoint wp = {};
        wp.x       = poly.pts[ci].x;
        wp.y       = poly.pts[ci].y;
        wp.heading = atan2f(dy, dx);
        wp.mowing  = mowing;
        wp.headland = headland;
        wp.reverse = false;
        g_waypoints.push_back(wp);
    }
}


// ── Transition helpers ────────────────────────────────────────────────────────

/**
 * Append transit waypoints between the end of one strip and the start of the next.
 *
 * Two cases:
 *  (a) Same-direction transit (segment gap within a concave strip level):
 *      heading_diff < 0.5 rad — the robot continues in the same direction;
 *      a single direct transit waypoint to P_start is sufficient.
 *
 *  (b) U-turn (consecutive strip levels, heading reverses by ≈ π):
 *      The robot drives forward into the headland to a pivot/arc midpoint,
 *      then navigates to P_start.  The midpoint is placed at the geometric
 *      centre between P_end and P_start, projected forward (into the headland)
 *      by half the available headland depth.
 *
 *      Rear-swing constraint: rear_clearance = sqrt(R² + mower_config_get().robot_rear_m²) - R.
 *      The midpoint is capped so the steering centre does not advance further
 *      than (mower_config_headland_m() - rear_clearance) from the strip end.
 *
 *      NARROW_SPACE: when mower_config_min_turn_radius_m() > mower_config_headland_m(), the
 *      standard single-arc U-turn cannot fit.  A three-waypoint sequence is
 *      used to guide the path follower through a wider Ω-turn.
 *
 * All transit waypoints have mowing = false.
 */
static void addTransition(const Waypoint &P_end, const Waypoint &P_start) {
    float heading_diff = fabsf(wrapAngle(P_start.heading - P_end.heading));

    // ── Case (a): same-direction short transit ────────────────────────────────
    if (heading_diff < 0.5f) {
        Waypoint wp  = P_start;
        wp.mowing    = false;
        wp.headland  = false;
        wp.reverse   = false;
        g_waypoints.push_back(wp);
        return;
    }

    // ── Case (b): U-turn transition ───────────────────────────────────────────
    float R = mower_config_min_turn_radius_m();

    // Rear swing radius during the arc turn.
    float R_rear = (R > 1e-3f)
                   ? sqrtf(R * R + mower_config_get().robot_rear_m * mower_config_get().robot_rear_m)
                   : mower_config_get().robot_rear_m;
    float rear_clearance = R_rear - R;  // additional lateral excursion of rear

    // Maximum safe forward advance into the headland before starting the turn.
    float max_fwd = mower_config_headland_m() - rear_clearance;
    if (max_fwd < 0.05f) max_fwd = 0.05f;

    // Geometric midpoint between the two strip ends (world frame).
    float mid_world_x = (P_end.x + P_start.x) * 0.5f;
    float mid_world_y = (P_end.y + P_start.y) * 0.5f;

    // Project midpoint forward (in P_end heading direction) by half headland.
    float fwd_frac = fminf(mower_config_headland_m() * 0.5f, max_fwd);
    float fwd_cos  = cosf(P_end.heading);
    float fwd_sin  = sinf(P_end.heading);

    float turn_x = mid_world_x + fwd_cos * fwd_frac;
    float turn_y = mid_world_y + fwd_sin * fwd_frac;

    // Heading at the turn midpoint: aim toward P_start.
    float dx_to_start = P_start.x - turn_x;
    float dy_to_start = P_start.y - turn_y;
    float len_to_start = sqrtf(dx_to_start * dx_to_start + dy_to_start * dy_to_start);
    float mid_heading = (len_to_start > 0.01f)
                        ? atan2f(dy_to_start, dx_to_start)
                        : P_start.heading;

    if (R <= 1e-3f) {
        // Pivot turn: single transit waypoint directly at P_start.
        Waypoint wp  = P_start;
        wp.mowing    = false;
        wp.headland  = false;
        wp.reverse   = false;
        g_waypoints.push_back(wp);
        return;
    }

    if (R_rear <= mower_config_headland_m()) {
        // Standard arc turn fits in the headland: one midpoint + P_start.
        Waypoint wp_mid = {};
        wp_mid.x        = turn_x;
        wp_mid.y        = turn_y;
        wp_mid.heading  = mid_heading;
        wp_mid.mowing   = false;
        wp_mid.headland = false;
        g_waypoints.push_back(wp_mid);
    } else if (max_fwd < R * 0.5f) {
        // VERY NARROW: headland too tight for any forward arc.
        // K-turn: drive forward, reverse back, drive forward to next strip.

        // wp1 — forward as far as safe.
        Waypoint wp1 = {};
        wp1.x        = P_end.x + fwd_cos * max_fwd;
        wp1.y        = P_end.y + fwd_sin * max_fwd;
        wp1.heading  = P_end.heading;
        wp1.mowing   = false;
        wp1.headland = false;
        wp1.reverse  = false;
        g_waypoints.push_back(wp1);

        // wp2 — reverse back past P_end.
        Waypoint wp_rev = {};
        wp_rev.x       = P_end.x - fwd_cos * max_fwd;
        wp_rev.y       = P_end.y - fwd_sin * max_fwd;
        wp_rev.heading = P_end.heading + (float)M_PI;
        wp_rev.mowing  = false;
        wp_rev.headland = false;
        wp_rev.reverse = true;
        g_waypoints.push_back(wp_rev);

        // wp3 — forward to P_start.
        Waypoint wp_fwd = P_start;
        wp_fwd.mowing  = false;
        wp_fwd.headland = false;
        wp_fwd.reverse = false;
        g_waypoints.push_back(wp_fwd);
        return;
    } else {
        // NARROW_SPACE: arc turn does not fit cleanly.
        // Use a 3-waypoint Ω-turn sequence to guide the path follower:
        //   wp1: drive forward toward nav boundary (stay within headland)
        //   wp2: lateral midpoint at headland depth, aimed toward next strip
        //   wp3: P_start

        // wp1 — forward approach to near the nav boundary.
        Waypoint wp1 = {};
        wp1.x        = P_end.x + fwd_cos * max_fwd;
        wp1.y        = P_end.y + fwd_sin * max_fwd;
        wp1.heading  = P_end.heading;
        wp1.mowing   = false;
        wp1.headland = false;
        g_waypoints.push_back(wp1);

        // wp2 — lateral midpoint in the headland zone.
        Waypoint wp2 = {};
        wp2.x        = turn_x;
        wp2.y        = turn_y;
        wp2.heading  = mid_heading;
        wp2.mowing   = false;
        wp2.headland = false;
        g_waypoints.push_back(wp2);
    }

    // Final transition waypoint: P_start itself (mowing=false).
    Waypoint wp_s  = P_start;
    wp_s.mowing    = false;
    wp_s.reverse   = false;
    g_waypoints.push_back(wp_s);
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
    // ── Validate inputs ───────────────────────────────────────────────────────
    if (perimeter.pts.size() < 3 || workingArea.pts.empty()) {
        DBG_PRINTLN("[CP] plan: invalid polygon (< 3 pts or empty workingArea)");
        return false;
    }
    if (fabsf(workingArea.area()) < MIN_ZONE_AREA_M2) {
        DBG_PRINTLN("[CP] plan: workingArea too small");
        return false;
    }

    // ── Reset state ───────────────────────────────────────────────────────────
    g_waypoints.clear();
    g_waypoints.reserve(512);  // typical garden: 200–600 waypoints
    g_wp_index            = 0;
    g_headland_wp_end_idx = 0;
    g_total_wp_count      = 0;
    g_unreachable_count   = 0;

    // ──────────────────────────────────────────────────────────────────────────
    //  Step 1 — Headland passes (EXECUTED FIRST)
    //
    //  The perimeter was recorded by driving the mower around the garden edge,
    //  so the perimeter polygon IS where the cut centre was.  The first headland
    //  pass (i=0) is placed directly on the perimeter (offset = 0).  Each
    //  subsequent pass steps inward by strip_step_m.
    // ──────────────────────────────────────────────────────────────────────────
    int n_headland_passes = (int)ceilf(mower_config_headland_m() / mower_config_strip_step_m());
    if (n_headland_passes < 1) n_headland_passes = 1;

    for (int i = 0; i < n_headland_passes; i++) {
        Polygon pass_poly;
        if (i == 0) {
            // Pass 0: use the raw perimeter (insetPolygon with offset=0 drops
            // concave corner vertices because the zero-radius arc is empty).
            pass_poly = perimeter;
            pass_poly.ensureCCW();
        } else {
            float offset = i * mower_config_strip_step_m();
            pass_poly = insetPolygon(perimeter, offset);
        }
        if (pass_poly.pts.empty()) {
            break;
        }
        // Insert a segment break (mowing=false) between headland passes
        // so the mow_path polyline doesn't draw a straight line between loops.
        if (i > 0 && !g_waypoints.empty()) {
            Waypoint brk = g_waypoints.back();
            brk.mowing = false;
            g_waypoints.push_back(brk);
        }
        addPolygonWaypoints(pass_poly, /*mowing=*/true, /*headland=*/true);
    }
    // Segment break after headland block (before strip regions)
    if (!g_waypoints.empty()) {
        Waypoint brk = g_waypoints.back();
        brk.mowing = false;
        g_waypoints.push_back(brk);
    }
    g_headland_wp_end_idx = (int)g_waypoints.size();

    // ──────────────────────────────────────────────────────────────────────────
    //  Compute working-area sub-regions.
    //  The strip region starts where the headland passes end: inset from the
    //  perimeter by n_headland_passes * strip_step.  For concave shapes (L, T, U)
    //  this inset may split into multiple disconnected regions.
    // ──────────────────────────────────────────────────────────────────────────
    float headland_total = n_headland_passes * mower_config_strip_step_m();
    std::vector<Polygon> regions = insetPolygonMulti(perimeter, headland_total);
    if (regions.empty()) {
        // Fallback: use the passed single workingArea (may miss disconnected regions)
        if (workingArea.pts.size() >= 3 && fabsf(workingArea.area()) >= MIN_ZONE_AREA_M2) {
            regions.push_back(workingArea);
        }
    }
    if (regions.empty()) {
        DBG_PRINTLN("[CP] plan: no valid working-area regions");
        g_total_wp_count = (int)g_waypoints.size();
        return true;  // headland-only plan
    }

    DBG_PRINTF("[CP] %d working-area region(s)\n", (int)regions.size());

    // ──────────────────────────────────────────────────────────────────────────
    //  Steps 2–5 — Per-region: optimal angle, strip generation, sequencing
    //
    //  Each region computes its own optimal mow angle from its convex hull.
    //  This ensures narrow peninsulas mow along their length (perpendicular
    //  to the shortest edge) rather than using a global angle that produces
    //  unusable short strips.  Between regions, nav-boundary-following transit
    //  waypoints keep the mower within bounds.
    // ──────────────────────────────────────────────────────────────────────────
    for (int ri = 0; ri < (int)regions.size(); ri++) {
        const Polygon &region = regions[ri];

        // ── Transit from previous region via perimeter ─────────────────────
        // Uses the perimeter (not nav boundary) because the nav boundary may
        // collapse at narrow necks, forcing the transit the wrong way around.
        // The perimeter was recorded by driving, so the mower can follow it.
        if (ri > 0 && !g_waypoints.empty()) {
            const Waypoint &last_wp = g_waypoints.back();
            int n_perim = (int)perimeter.pts.size();

            // Nearest perimeter vertex to last waypoint
            float best_s = 1e9f;
            int idx_s = 0;
            for (int i = 0; i < n_perim; i++) {
                float dx = perimeter.pts[i].x - last_wp.x;
                float dy = perimeter.pts[i].y - last_wp.y;
                float d = dx*dx + dy*dy;
                if (d < best_s) { best_s = d; idx_s = i; }
            }

            // Nearest perimeter vertex to centroid of next region
            float rcx = 0.0f, rcy = 0.0f;
            for (auto &p : region.pts) { rcx += p.x; rcy += p.y; }
            rcx /= (float)region.pts.size();
            rcy /= (float)region.pts.size();

            float best_e = 1e9f;
            int idx_e = 0;
            for (int i = 0; i < n_perim; i++) {
                float dx = perimeter.pts[i].x - rcx;
                float dy = perimeter.pts[i].y - rcy;
                float d = dx*dx + dy*dy;
                if (d < best_e) { best_e = d; idx_e = i; }
            }

            if (idx_s != idx_e) {
                int cw_steps  = (idx_e - idx_s + n_perim) % n_perim;
                int ccw_steps = (idx_s - idx_e + n_perim) % n_perim;
                bool walk_cw  = (cw_steps <= ccw_steps);
                int steps = walk_cw ? cw_steps : ccw_steps;
                int dir   = walk_cw ? 1 : -1;

                for (int s = 0; s <= steps; s++) {
                    int ci = (idx_s + s * dir + n_perim) % n_perim;
                    int ni = (s < steps)
                             ? (idx_s + (s + 1) * dir + n_perim) % n_perim
                             : ci;
                    float dx = perimeter.pts[ni].x - perimeter.pts[ci].x;
                    float dy = perimeter.pts[ni].y - perimeter.pts[ci].y;

                    Waypoint wp = {};
                    wp.x       = perimeter.pts[ci].x;
                    wp.y       = perimeter.pts[ci].y;
                    wp.heading = atan2f(dy, dx);
                    wp.mowing  = false;
                    wp.headland = false;
                    wp.reverse = false;
                    g_waypoints.push_back(wp);
                }
                DBG_PRINTF("[CP] transit: region %d → %d (%d perim vertices)\n",
                              ri - 1, ri, steps + 1);
            }
        }

        // ── Region outline pass (Slic3r-style innermost perimeter) ──────────
        //  Strips are parallel lines spaced strip_step apart: wherever the
        //  region boundary is NOT parallel to the strips, the last strip can
        //  sit up to a full step from the edge, leaving an unmowed wedge
        //  (clearly visible on boundary edges at a shallow angle to the mow
        //  direction).  Tracing the region outline first seals every such
        //  wedge — with strip_step ≤ cut width, no interior point is further
        //  than half a cut width from either a strip or this outline pass,
        //  exactly like slicer infill meeting the innermost perimeter loop.
        {
            if (!g_waypoints.empty()) {
                Waypoint brk = g_waypoints.back();
                brk.mowing = false;
                g_waypoints.push_back(brk);
            }
            addPolygonWaypoints(region, /*mowing=*/true, /*headland=*/false);
            // Segment break before the strips begin
            Waypoint brk2 = g_waypoints.back();
            brk2.mowing = false;
            g_waypoints.push_back(brk2);
        }

        // ── Step 2 — Per-region optimal mow angle ──────────────────────────
        //  Convex hull edge scan: choose the edge direction that minimises
        //  the number of strips for THIS region.  Narrow peninsulas will
        //  naturally get strips along their length.
        float mow_angle = 0.0f;
        {
            Polygon hull = convexHull(region);
            int   n_hull     = (int)hull.pts.size();
            int   min_strips = INT_MAX;

            for (int e = 0; e < n_hull; e++) {
                float ax = hull.pts[(e + 1) % n_hull].x - hull.pts[e].x;
                float ay = hull.pts[(e + 1) % n_hull].y - hull.pts[e].y;
                if (fabsf(ax) < 1e-6f && fabsf(ay) < 1e-6f) continue;

                float angle = atan2f(ay, ax);

                float y_min_h =  1e9f;
                float y_max_h = -1e9f;
                for (auto &p : hull.pts) {
                    float ry = -p.x * sinf(angle) + p.y * cosf(angle);
                    if (ry < y_min_h) y_min_h = ry;
                    if (ry > y_max_h) y_max_h = ry;
                }

                float range      = y_max_h - y_min_h;
                int   n_strips_e = (int)ceilf(range / mower_config_strip_step_m());

                if (n_strips_e < min_strips) {
                    min_strips = n_strips_e;
                    mow_angle  = angle;
                }
            }
        }

        // Nav boundary rotated into this region's mow frame (for spur detection).
        std::vector<Point> rot_nav_pts;
        rot_nav_pts.reserve(navBoundary.pts.size());
        for (auto &p : navBoundary.pts) {
            rot_nav_pts.push_back(rotatePoint(p.x, p.y, -mow_angle));
        }

        auto isSpurTip = [&](float xc) -> bool {
            std::vector<float> ys = vLineIntersect(rot_nav_pts, xc);
            if (ys.size() < 2) return false;
            float span = ys.back() - ys.front();
            return span < mower_config_spur_min_turn_width_m();
        };

        // ── Step 3 — Rotate region vertices, scan-line intersect ─────────────
        std::vector<Point> rot_pts;
        rot_pts.reserve(region.pts.size());
        for (auto &p : region.pts) {
            rot_pts.push_back(rotatePoint(p.x, p.y, -mow_angle));
        }

        float y_min_r =  1e9f;
        float y_max_r = -1e9f;
        for (auto &p : rot_pts) {
            if (p.y < y_min_r) y_min_r = p.y;
            if (p.y > y_max_r) y_max_r = p.y;
        }

        float y_start = y_min_r + mower_config_get().cut_disc_radius_m;
        float y_stop  = y_max_r - mower_config_get().cut_disc_radius_m;

        if (y_stop < y_start) {
            DBG_PRINTF("[CP] region %d: too narrow for strips\n", ri);
            if (g_unreachable_count < MAX_UNREACHABLE_ZONES) {
                float rcx = 0.0f, rcy = 0.0f;
                for (auto &p : region.pts) { rcx += p.x; rcy += p.y; }
                rcx /= (float)region.pts.size();
                rcy /= (float)region.pts.size();
                g_unreachable[g_unreachable_count++] = {
                    rcx, rcy, fabsf(region.area())
                };
            }
            continue;
        }

        // Collect strip segments for this region
        struct StripSeg {
            float y_rot;
            float x_left;
            float x_right;
        };
        std::vector<StripSeg> segments;
        segments.reserve(64);

        for (float ys = y_start; ys <= y_stop + 1e-4f; ys += mower_config_strip_step_m()) {
            std::vector<float> xs = hLineIntersect(rot_pts, ys);

            if (xs.size() % 2 != 0) {
                if (g_unreachable_count < MAX_UNREACHABLE_ZONES) {
                    float x_mid = 0.0f;
                    for (float x : xs) x_mid += x;
                    if (!xs.empty()) x_mid /= (float)xs.size();
                    auto wc = rotatePoint(x_mid, ys, mow_angle);
                    g_unreachable[g_unreachable_count++] = { wc.x, wc.y, 0.0f };
                }
                continue;
            }

            for (int k = 0; k + 1 < (int)xs.size(); k += 2) {
                float xl = xs[k];
                float xr = xs[k + 1];
                if (xr - xl < mower_config_get().cut_disc_radius_m * 2.0f) {
                    if (g_unreachable_count < MAX_UNREACHABLE_ZONES) {
                        auto wc = rotatePoint((xl + xr) * 0.5f, ys, mow_angle);
                        float area = (xr - xl) * mower_config_strip_step_m();
                        g_unreachable[g_unreachable_count++] = { wc.x, wc.y, area };
                    }
                    continue;
                }
                segments.push_back({ ys, xl, xr });
            }
        }

        if (segments.empty()) {
            DBG_PRINTF("[CP] region %d: no valid strip segments\n", ri);
            continue;
        }

        // ── Step 4 — Boustrophedon sequencing for this region ────────────────
        struct StripWP {
            Waypoint start;
            Waypoint end;
            bool     start_spur;
            bool     end_spur;
        };
        std::vector<StripWP> strip_wps;
        strip_wps.reserve(segments.size());

        for (int s = 0; s < (int)segments.size(); s++) {
            const StripSeg &seg = segments[s];
            bool ltr = (s % 2 == 0);

            float rot_xs_x = ltr ? seg.x_left  : seg.x_right;
            float rot_xe_x = ltr ? seg.x_right : seg.x_left;
            float heading  = ltr ? mow_angle
                                 : wrapAngle(mow_angle + (float)M_PI);

            auto ws = rotatePoint(rot_xs_x, seg.y_rot, mow_angle);
            auto we = rotatePoint(rot_xe_x, seg.y_rot, mow_angle);

            Waypoint wp_s = {};
            wp_s.x        = ws.x;
            wp_s.y        = ws.y;
            wp_s.heading  = heading;
            wp_s.mowing   = true;
            wp_s.headland = false;

            Waypoint wp_e = {};
            wp_e.x        = we.x;
            wp_e.y        = we.y;
            wp_e.heading  = heading;
            wp_e.mowing   = true;
            wp_e.headland = false;

            bool start_spur = isSpurTip(rot_xs_x);
            bool end_spur   = isSpurTip(rot_xe_x);

            strip_wps.push_back({ wp_s, wp_e, start_spur, end_spur });
        }

        // ── Step 5 — Append strip waypoints and transitions ─────────────────
        for (int s = 0; s < (int)strip_wps.size(); s++) {
            const StripWP &sw = strip_wps[s];

            if (sw.start_spur && sw.end_spur) {
                if (g_unreachable_count < MAX_UNREACHABLE_ZONES) {
                    float scx = (sw.start.x + sw.end.x) * 0.5f;
                    float scy = (sw.start.y + sw.end.y) * 0.5f;
                    float dx = sw.end.x - sw.start.x;
                    float dy = sw.end.y - sw.start.y;
                    g_unreachable[g_unreachable_count++] = {
                        scx, scy, sqrtf(dx*dx + dy*dy) * mower_config_strip_step_m()
                    };
                }
                continue;
            }

            Waypoint fwd_s  = sw.start;  fwd_s.reverse = false;
            Waypoint fwd_e  = sw.end;    fwd_e.reverse = false;
            g_waypoints.push_back(fwd_s);
            g_waypoints.push_back(fwd_e);

            if (sw.end_spur) {
                Waypoint rev    = sw.start;
                rev.mowing      = true;
                rev.reverse     = true;
                g_waypoints.push_back(rev);
                DBG_PRINTF("[CP] spur reverse wp at (%.2f, %.2f)\n",
                              rev.x, rev.y);

                if (s + 1 < (int)strip_wps.size()) {
                    Waypoint exit_wp  = sw.start;
                    exit_wp.mowing    = false;
                    exit_wp.reverse   = false;
                    addTransition(exit_wp, strip_wps[s + 1].start);
                }
            } else {
                if (s + 1 < (int)strip_wps.size()) {
                    addTransition(sw.end, strip_wps[s + 1].start);
                }
            }
        }

        DBG_PRINTF("[CP] region %d: %d strips\n", ri, (int)segments.size());
    }  // end region loop

    // ── Safety clamp: move any out-of-bounds waypoint to the nearest nav boundary point ──
    // Catches waypoints that escaped due to concave-corner inset overshoot or
    // any other geometry edge case.  O(waypoints × boundary_edges) — fast enough.
    {
        const int nb = (int)navBoundary.pts.size();
        if (nb >= 3) {
            int clamped = 0;
            for (auto &wp : g_waypoints) {
                if (!isInsidePolygon(navBoundary, wp.x, wp.y)) {
                    float best_d2 = 1e30f;
                    float best_x  = wp.x, best_y = wp.y;
                    for (int ei = 0; ei < nb; ei++) {
                        int ej = (ei + 1) % nb;
                        float ax = navBoundary.pts[ei].x, ay = navBoundary.pts[ei].y;
                        float bx = navBoundary.pts[ej].x, by = navBoundary.pts[ej].y;
                        float dx = bx - ax, dy = by - ay;
                        float lenSq = dx*dx + dy*dy;
                        float t = (lenSq > 1e-6f)
                                  ? clampf(((wp.x-ax)*dx + (wp.y-ay)*dy) / lenSq, 0.0f, 1.0f)
                                  : 0.0f;
                        float px = ax + t*dx, py = ay + t*dy;
                        float ex = wp.x - px, ey = wp.y - py;
                        float d2 = ex*ex + ey*ey;
                        if (d2 < best_d2) { best_d2 = d2; best_x = px; best_y = py; }
                    }
                    wp.x = best_x;
                    wp.y = best_y;
                    clamped++;
                }
            }
            if (clamped > 0) {
                DBG_PRINTF("[CP] clamped %d waypoints to nav boundary\n", clamped);
            }
        }
    }

    // ── Finalise ──────────────────────────────────────────────────────────────
    g_total_wp_count = (int)g_waypoints.size();

    DBG_PRINTF("[CP] plan complete: %d headland, %d strip/transit, %d regions, %d unreachable\n",
                  g_headland_wp_end_idx,
                  g_total_wp_count - g_headland_wp_end_idx,
                  (int)regions.size(),
                  g_unreachable_count);

    return true;
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

    float best_dist = 1e9f;
    int   best_idx  = g_wp_index;  // default: stay at current index

    // Search only unvisited waypoints (from current index onward).
    for (int i = g_wp_index; i < g_total_wp_count; i++) {
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
