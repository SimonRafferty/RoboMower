#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <vector>
#include <utility>      // std::pair
#include "geometry.h"   // Polygon type

// Forward declaration — Waypoint is defined in coverage_planner.h.
// We use a forward declaration here rather than #include to avoid a circular
// dependency (coverage_planner.h includes obstacle_map.h for grid queries).
// obstacle_map.cpp includes coverage_planner.h to access the full definition.
struct Waypoint;

// ══════════════════════════════════════════════════════════════════════════════
//  obstacle_map.h — RoboMower obstacle record system and headland coverage grid
//
//  Two independent subsystems share this module:
//    1. Obstacle records — ring buffer of up to MAX_OBSTACLES positions with
//       opposite-direction retry tracking.
//    2. Coverage grid — 2D raster of CellState values that credits both
//       strip-mowing and arc-sweep passes. Used by coverage_planner to
//       determine when the headland zone has been fully mowed.
// ══════════════════════════════════════════════════════════════════════════════

// ── Cell state enumeration ────────────────────────────────────────────────────

/** State of a single coverage grid cell. Stored as uint8_t. */
enum CellState : uint8_t {
    CELL_UNKNOWN     = 0, ///< Not yet initialised (grid reset state)
    CELL_UNMOWED     = 1, ///< Within mow zone, not yet mowed
    CELL_MOWED_STRIP = 2, ///< Mowed during a boustrophedon strip pass
    CELL_MOWED_ARC   = 3, ///< Mowed during a headland arc-sweep turn
    CELL_OBSTACLE    = 4  ///< Occupied by a detected obstacle — not mowable
};

// ── Obstacle record ───────────────────────────────────────────────────────────

/** One recorded obstacle position with retry state. */
struct ObstacleRecord {
    float x;                ///< Steering centre X at time of detection (m, ENU)
    float y;                ///< Steering centre Y at time of detection (m, ENU)
    float approach_heading; ///< Robot heading at time of contact (rad, ENU)
    bool  retry_pending;    ///< true = opposite-direction retry not yet attempted
    bool  retry_succeeded;  ///< true = retry pass completed the strip successfully
    bool  active;           ///< true = slot is occupied (false = empty/overwritten)
};

// ══════════════════════════════════════════════════════════════════════════════
//  Lifecycle
// ══════════════════════════════════════════════════════════════════════════════

/** Initialise the obstacle map using the bounding box of @p perimeter.
 *
 *  Grid cell size = CUT_WIDTH_M / 4. If the bounding box is too large to
 *  fit within 256×256 cells at that resolution, the cell size is
 *  automatically coarsened to CUT_WIDTH_M / 2 or CUT_WIDTH_M and a
 *  warning is printed to Serial.
 *
 *  Must be called before any grid or obstacle functions. Safe to call
 *  again to reinitialise after a new perimeter is recorded.
 *
 *  @param perimeter  Recorded perimeter polygon. At least 3 points required.
 */
void obstacle_map_init(const Polygon &perimeter);

/** Reset all data — clears obstacle records and coverage grid cells.
 *  Does NOT change grid layout (dimensions/origin/cell-size). */
void obstacle_map_reset();

// ══════════════════════════════════════════════════════════════════════════════
//  Obstacle records
// ══════════════════════════════════════════════════════════════════════════════

/** Record a new obstacle at (@p x, @p y) with the given @p approach_heading.
 *
 *  Sets retry_pending = true, retry_succeeded = false. Also marks the
 *  corresponding grid cell as CELL_OBSTACLE.
 *
 *  If MAX_OBSTACLES records are already active, the oldest record is
 *  overwritten (ring buffer wrap). */
void obstacle_add(float x, float y, float approach_heading);

/** Query whether a retry waypoint is available for the obstacle nearest to
 *  (@p x, @p y) within 1.0 m.
 *
 *  If found and retry_pending is true:
 *    - Fills @p retry_wp with a waypoint one CUT_WIDTH_M ahead in the
 *      opposite approach direction.
 *    - Clears retry_pending (retry is consumed — will not be returned again).
 *    - Returns true.
 *
 *  Returns false if no suitable obstacle is found or retry already consumed.
 *
 *  @param[in]  x         Query X (m)
 *  @param[in]  y         Query Y (m)
 *  @param[out] retry_wp  Filled with the retry target waypoint on success
 */
bool obstacle_get_retry(float x, float y, Waypoint &retry_wp);

/** Mark the retry as succeeded for the obstacle nearest to (@p x, @p y).
 *  Sets retry_succeeded = true on the matching record (within 1.0 m). */
void obstacle_mark_retry_succeeded(float x, float y);

/** Return the number of currently active obstacle records. */
int obstacle_get_count();

/** Print all active obstacle records to Serial (debug). */
void obstacle_dump();

// ══════════════════════════════════════════════════════════════════════════════
//  Coverage grid
// ══════════════════════════════════════════════════════════════════════════════

/** Mark all grid cells whose centres fall within @p radius_m of (@p cx, @p cy)
 *  as CELL_MOWED_ARC. Cells already marked CELL_OBSTACLE are not changed. */
void grid_mark_arc_swept(float cx, float cy, float radius_m);

/** Mark the grid cell containing world point (@p x, @p y) as CELL_MOWED_STRIP.
 *  Increments the session mowed cell counter if the cell was not already
 *  CELL_MOWED_STRIP. Cells marked CELL_OBSTACLE are not changed. */
void grid_mark_strip(float x, float y);

/** Mark the grid cell containing (@p x, @p y) as CELL_OBSTACLE. */
void grid_mark_obstacle(float x, float y);

/** Return true if the cell at (@p x, @p y) is CELL_MOWED_STRIP or
 *  CELL_MOWED_ARC.  Returns false for out-of-bounds coordinates. */
bool grid_is_mowed(float x, float y);

/** Return the fraction of non-OBSTACLE cells that have been mowed (0.0–1.0).
 *  Returns 0.0 if the grid has not been initialised. */
float grid_get_headland_coverage();

/** Print a compact ASCII representation of the coverage grid to Serial.
 *  Legend: '.' = UNMOWED, '#' = MOWED_STRIP, 'o' = MOWED_ARC,
 *          'X' = OBSTACLE, '?' = UNKNOWN */
void grid_dump_ascii();

/** Return the mowed area accumulated during the current mowing session,
 *  in square decimetres (dm²).
 *
 *  Counts cells that transitioned to CELL_MOWED_STRIP during this session.
 *  Counter resets in obstacle_map_reset(). */
uint32_t grid_get_session_mowed_dm2();

/** Fill @p out with (col, row) pairs of every cell that is CELL_MOWED_STRIP
 *  or CELL_MOWED_ARC. Clears @p out first. Limited to 2048 entries to keep
 *  the BLE map JSON within a manageable size. */
void grid_enumerate_mowed(std::vector<std::pair<uint8_t,uint8_t>> &out);

/** Return the active grid column count (0 if not initialised). */
int   grid_get_cols();

/** Return the active grid row count (0 if not initialised). */
int   grid_get_rows();

/** Return the cell side length in metres (0.0 if not initialised). */
float grid_get_cell_size_m();

/** Return the world X (ENU east) of the left edge of cell column 0. */
float grid_get_origin_x();

/** Return the world Y (ENU north) of the bottom edge of cell row 0. */
float grid_get_origin_y();
