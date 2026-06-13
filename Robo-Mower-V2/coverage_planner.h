#pragma once
#include <Arduino.h>
#include <vector>
#include "geometry.h"
#include "obstacle_map.h"

// ══════════════════════════════════════════════════════════════════════════════
//  coverage_planner.h — RoboMower complete mowing path planner
//
//  Generates the full mowing plan before execution begins.
//
//  Execution order (HEADLAND_FIRST = 1, always):
//    1. Headland perimeter passes   — outermost first, inward
//    2. Boustrophedon strip passes  — across the working area
//    3. Strip-end transitions       — arc/pivot turns within headland
//
//  Call sequence:
//    coverage_planner_init()
//    coverage_planner_plan(perimeter, navBoundary, workingArea)
//    while (!coverage_planner_is_complete()) {
//        Waypoint wp;
//        if (coverage_planner_get_next(wp)) { follow wp; }
//    }
//
//  Source references:
//    Spec: Robo_Mower_claudecode_prompt_v3.md §Coverage Planner
//    Assumptions: ASSUMPTIONS.md A07, A09, A19
//    Handoff: HANDOFFS/15_coverage_planner/HANDOFF.md
// ══════════════════════════════════════════════════════════════════════════════


/** A single mowing waypoint — position of the steering centre in ENU metres. */
struct Waypoint {
    float x;        ///< Steering centre ENU east (m)
    float y;        ///< Steering centre ENU north (m)
    float heading;  ///< Desired heading at this point (rad, ENU)
    bool  mowing;   ///< true = blade engaged (cut pass), false = transit
    bool  headland; ///< true = headland perimeter pass, false = working strip
    bool  reverse;  ///< true = robot drives backward to reach this waypoint (spur exit)
};


// ── Lifecycle ─────────────────────────────────────────────────────────────────

/** Initialise the coverage planner. Must be called before any other function.
 *  Clears waypoint list, indices, and unreachable-zone log. */
void coverage_planner_init();

/** Generate the complete mowing plan. Call once on entering AUTO_MOWING.
 *
 *  Planning steps (in order):
 *    1. Headland passes — n_passes inset polygons, outermost first
 *    2. Optimal mowing angle — minimise strip count via convex hull edge scan
 *    3. Strip generation — horizontal intersection in rotated frame
 *    4. Boustrophedon sequencing — alternating direction per strip
 *    5. Strip-end transitions — arc or pivot, with rear-swing validation
 *
 *  Arc-sweep credit (Step 6/7) is disabled when
 *  STEER_CENTRE_TO_CUT_CENTRE_M <= 0 (front-drive configuration).
 *
 *  @param perimeter    Recorded perimeter polygon (≥ 3 vertices, CCW)
 *  @param navBoundary  Navigation boundary (inset of perimeter)
 *  @param workingArea  Strip mowing area (inset of navBoundary by HEADLAND_WIDTH_M)
 *  @return true on success; false if polygons are invalid or working area is empty
 */
bool coverage_planner_plan(const Polygon &perimeter,
                           const Polygon &navBoundary,
                           const Polygon &workingArea);


// ── Waypoint iteration ────────────────────────────────────────────────────────

/** Get the next waypoint and advance the internal index.
 *
 *  @param[out] wp  Filled with the next unvisited waypoint on success.
 *  @return true if a waypoint was returned; false if the plan is complete.
 */
bool coverage_planner_get_next(Waypoint &wp);


// ── Obstacle handling ─────────────────────────────────────────────────────────

/** Report an obstacle at (x,y) encountered with the given approach heading.
 *
 *  Records the obstacle in the obstacle map and marks the corresponding
 *  grid cell as CELL_OBSTACLE. Advances the waypoint index to skip any
 *  remaining waypoints within sqrt(MIN_ZONE_AREA_M2 / π) metres of (x,y).
 *
 *  @param x               Obstacle ENU east position (m)
 *  @param y               Obstacle ENU north position (m)
 *  @param approach_heading Robot heading at time of contact (rad)
 */
void coverage_planner_report_obstacle(float x, float y, float approach_heading);

/** Reset the waypoint index to the nearest unvisited waypoint to (x,y).
 *
 *  Used after obstacle avoidance or cross-track error recovery to resume
 *  mowing from the closest point rather than from the beginning.
 *
 *  @param x  Current steering centre ENU east (m)
 *  @param y  Current steering centre ENU north (m)
 */
void coverage_planner_reset_to_nearest(float x, float y);


// ── Progress reporting ────────────────────────────────────────────────────────

/** Fraction of headland passes completed (0.0–1.0). */
float coverage_planner_get_headland_progress();

/** Fraction of working-area strips completed (0.0–1.0). */
float coverage_planner_get_strip_progress();

/** Fraction of total plan (headland + strips) completed (0.0–1.0). */
float coverage_planner_get_total_progress();

/** True when all waypoints in the plan have been visited. */
bool coverage_planner_is_complete();


// ── Session resume helpers ────────────────────────────────────────────────────

/**
 * @brief Return the index of the next unvisited waypoint.
 *
 * Used by the state machine to save progress to NVS on MOTORS_OFFLINE entry,
 * so that mowing can resume from the correct position after a battery swap.
 *
 * @return Current waypoint index (0 = plan just generated; equals total count
 *         when plan is complete).
 */
int coverage_planner_get_waypoint_index();

/**
 * @brief Skip forward to a specific waypoint index.
 *
 * Called on session resume: coverage_planner_plan() is called first to
 * regenerate the waypoint list, then this function advances past already-
 * completed waypoints.  The index is clamped to [0, total_count].
 *
 * @param idx  Waypoint index to resume from.
 */
void coverage_planner_skip_to_waypoint(int idx);


// ── Waypoint access ──────────────────────────────────────────────────────────

/** Return a read-only reference to the planned waypoint list.
 *  Empty if no plan has been generated yet. */
const std::vector<Waypoint>& coverage_planner_get_waypoints();

/** Return the total number of waypoints in the current plan. */
int coverage_planner_get_waypoint_count();

// ── Diagnostics ───────────────────────────────────────────────────────────────

/** Return the count of unreachable zones logged during planning.
 *
 *  Unreachable zones are working-area sub-regions that produced no valid
 *  strip segments (e.g., due to polygon degeneracy or insufficient width). */
int coverage_planner_get_unreachable_zone_count();

/** Print unreachable zone centres and areas to Serial (debug). */
void coverage_planner_log_unreachable_zones();

/** Return the area mowed this session in square decimetres (dm²).
 *
 *  Reads from the obstacle_map coverage grid — only cells marked
 *  CELL_MOWED_STRIP during execution are counted.  Counter resets in
 *  coverage_planner_plan() (not at power-on), so BOG_RECOVERY and
 *  RETRACE do not zero the counter. See ASSUMPTIONS.md A09. */
uint32_t coverage_planner_get_session_mowed_dm2();
