// obstacle_map.cpp — RoboMower obstacle record system and headland coverage grid
//
// Two subsystems:
//   1. Obstacle ring buffer (MAX_OBSTACLES = 50) — circular, overwrites oldest.
//   2. Static 2D coverage grid — up to 256×256 cells, 1 byte each (64 KB max).
//
// Grid resolution: mower_config_get().cut_width_m / 4 per cell (≈95 mm at default cut width).
// If the perimeter bounding box exceeds 256 cells in either dimension, the
// resolution is automatically coarsened and a warning is logged to Serial.
//
// See obstacle_map.h for full API documentation.

#include "obstacle_map.h"
#include "coverage_planner.h"   // full Waypoint definition
#include "config.h"
#include "mower_config.h"
#include <math.h>

// ── Grid constants ────────────────────────────────────────────────────────────

static constexpr int   MAX_GRID_DIM   = 256;
static constexpr float RETRY_SEARCH_RADIUS_M = 1.0f; ///< Max search distance for retry queries

// ── Grid state ────────────────────────────────────────────────────────────────

/// Coverage grid. g_grid[row][col] indexed row-major (row = Y axis, col = X axis).
static uint8_t g_grid[MAX_GRID_DIM][MAX_GRID_DIM];

static int   g_grid_width  = 0;     ///< Active column count
static int   g_grid_height = 0;     ///< Active row count
static float g_grid_origin_x = 0.0f; ///< World X of cell (0,0) lower-left corner (m)
static float g_grid_origin_y = 0.0f; ///< World Y of cell (0,0) lower-left corner (m)
static float g_cell_size_m   = 0.0f; ///< Side length of each cell (m)

/// Cells that transitioned to CELL_MOWED_STRIP during the current session.
static uint32_t g_session_mowed_cells = 0;

// ── Obstacle ring buffer ──────────────────────────────────────────────────────

static ObstacleRecord g_obstacles[MAX_OBSTACLES];

/// Index of the slot that will be written next (ring buffer write pointer).
static int g_obs_write_idx = 0;

/// Number of active (occupied) obstacle slots, capped at MAX_OBSTACLES.
static int g_obs_active = 0;

// ── Private helpers ───────────────────────────────────────────────────────────

// wrapAngle() is provided by geometry.h (included via obstacle_map.h)

/** Convert world coordinates to grid column and row indices.
 *  @return true if (col, row) falls within the active grid area. */
static bool worldToGrid(float x, float y, int &col, int &row) {
    if (g_cell_size_m <= 0.0f) return false;
    col = (int)((x - g_grid_origin_x) / g_cell_size_m);
    row = (int)((y - g_grid_origin_y) / g_cell_size_m);
    if (col < 0 || col >= g_grid_width || row < 0 || row >= g_grid_height) return false;
    return true;
}

/** Convert grid (col, row) to world coordinates of the cell centre. */
static void gridToWorld(int col, int row, float &cx, float &cy) {
    cx = g_grid_origin_x + (col + 0.5f) * g_cell_size_m;
    cy = g_grid_origin_y + (row + 0.5f) * g_cell_size_m;
}

/** Find the active obstacle record nearest to (qx, qy).
 *  @return pointer to the nearest record within RETRY_SEARCH_RADIUS_M, or nullptr. */
static ObstacleRecord *findNearest(float qx, float qy) {
    ObstacleRecord *best = nullptr;
    float best_dist2 = RETRY_SEARCH_RADIUS_M * RETRY_SEARCH_RADIUS_M;
    for (int i = 0; i < MAX_OBSTACLES; ++i) {
        if (!g_obstacles[i].active) continue;
        float dx = g_obstacles[i].x - qx;
        float dy = g_obstacles[i].y - qy;
        float d2 = dx * dx + dy * dy;
        if (d2 < best_dist2) {
            best_dist2 = d2;
            best = &g_obstacles[i];
        }
    }
    return best;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Lifecycle
// ══════════════════════════════════════════════════════════════════════════════

void obstacle_map_init(const Polygon &perimeter) {
    if (perimeter.pts.empty()) {
        DBG_PRINTLN(F("[obstacle_map] ERROR: empty perimeter — grid not initialised"));
        return;
    }

    // ── Compute bounding box ──────────────────────────────────────────────────
    float min_x = perimeter.pts[0].x,  max_x = min_x;
    float min_y = perimeter.pts[0].y, max_y = min_y;
    for (const auto &pt : perimeter.pts) {
        if (pt.x  < min_x) min_x = pt.x;
        if (pt.x  > max_x) max_x = pt.x;
        if (pt.y < min_y) min_y = pt.y;
        if (pt.y > max_y) max_y = pt.y;
    }

    const float bbox_w = max_x - min_x;
    const float bbox_h = max_y - min_y;

    // Reject degenerate perimeters (all points at same location or collinear)
    // — would produce cell_size ≈ ∞ and a useless 1×1 grid. (MED-6 fix)
    if (bbox_w < 0.5f || bbox_h < 0.5f) {
        DBG_PRINTF("[obstacle_map] ERROR: degenerate perimeter bbox %.2fm×%.2fm "
                      "(need ≥ 0.5m×0.5m) — grid not initialised\n", bbox_w, bbox_h);
        return;
    }

    // ── Choose cell size ──────────────────────────────────────────────────────
    // Try mower_config_get().cut_width_m/4 first; fall back to /2 then /1 if grid would exceed 256×256.
    // We reserve 1 extra cell margin on each side, hence "+2" below.
    float cell = mower_config_get().cut_width_m / 4.0f;

    auto requiredCols = [&]() { return (int)ceilf(bbox_w / cell) + 2; };
    auto requiredRows = [&]() { return (int)ceilf(bbox_h / cell) + 2; };

    if (requiredCols() > MAX_GRID_DIM || requiredRows() > MAX_GRID_DIM) {
        cell = mower_config_get().cut_width_m / 2.0f;
        DBG_PRINT(F("[obstacle_map] WARNING: resolution degraded to "));
        DBG_PRINT(cell * 1000.0f, 0);
        DBG_PRINTLN(F(" mm/cell (bounding box > 256×256 at mower_config_get().cut_width_m/4)"));
    }

    if (requiredCols() > MAX_GRID_DIM || requiredRows() > MAX_GRID_DIM) {
        cell = mower_config_get().cut_width_m;
        DBG_PRINT(F("[obstacle_map] WARNING: resolution further degraded to "));
        DBG_PRINT(cell * 1000.0f, 0);
        DBG_PRINTLN(F(" mm/cell (bounding box > 256×256 at mower_config_get().cut_width_m/2)"));
    }

    // Final hard cap — should not be reached for any realistic garden.
    int cols = requiredCols();
    int rows = requiredRows();
    if (cols > MAX_GRID_DIM) cols = MAX_GRID_DIM;
    if (rows > MAX_GRID_DIM) rows = MAX_GRID_DIM;

    // ── Store grid parameters ─────────────────────────────────────────────────
    g_cell_size_m   = cell;
    g_grid_width    = cols;
    g_grid_height   = rows;
    g_grid_origin_x = min_x - cell;  // 1-cell margin on the lower-left
    g_grid_origin_y = min_y - cell;

    // Clear grid and obstacles.
    obstacle_map_reset();

    // Pre-fill all cells as CELL_UNMOWED so coverage fraction is meaningful.
    for (int r = 0; r < g_grid_height; ++r)
        for (int c = 0; c < g_grid_width; ++c)
            g_grid[r][c] = CELL_UNMOWED;

    DBG_PRINT(F("[obstacle_map] init: "));
    DBG_PRINT(g_grid_width);
    DBG_PRINT('x');
    DBG_PRINT(g_grid_height);
    DBG_PRINT(F(" cells @ "));
    DBG_PRINT(g_cell_size_m * 1000.0f, 1);
    DBG_PRINT(F(" mm  origin=("));
    DBG_PRINT(g_grid_origin_x, 2);
    DBG_PRINT(F(", "));
    DBG_PRINT(g_grid_origin_y, 2);
    DBG_PRINTLN(F(") m"));
}

void obstacle_map_reset() {
    // Clear grid cells to CELL_UNKNOWN (0).
    memset(g_grid, CELL_UNKNOWN, sizeof(g_grid));

    // Clear obstacle records.
    for (int i = 0; i < MAX_OBSTACLES; ++i) {
        g_obstacles[i] = {};  // zero-initialise all fields (active = false)
    }
    g_obs_write_idx = 0;
    g_obs_active    = 0;

    // Reset session counter.
    g_session_mowed_cells = 0;
}

// ══════════════════════════════════════════════════════════════════════════════
//  Obstacle records
// ══════════════════════════════════════════════════════════════════════════════

void obstacle_add(float x, float y, float approach_heading) {
    int slot = g_obs_write_idx;
    g_obstacles[slot].x                = x;
    g_obstacles[slot].y                = y;
    g_obstacles[slot].approach_heading = approach_heading;
    g_obstacles[slot].retry_pending    = true;
    g_obstacles[slot].retry_succeeded  = false;
    g_obstacles[slot].active           = true;

    // Advance write pointer (wraps around — overwrites oldest entry when full).
    g_obs_write_idx = (g_obs_write_idx + 1) % MAX_OBSTACLES;
    if (g_obs_active < MAX_OBSTACLES) ++g_obs_active;

    // Mark the corresponding grid cell as CELL_OBSTACLE.
    grid_mark_obstacle(x, y);

    DBG_PRINT(F("[obstacle_map] obstacle added at ("));
    DBG_PRINT(x, 2); DBG_PRINT(F(", ")); DBG_PRINT(y, 2);
    DBG_PRINT(F(") hdg="));
    DBG_PRINT(approach_heading * 57.2958f, 1);
    DBG_PRINT(F("°  slot="));
    DBG_PRINTLN(slot);
}

bool obstacle_get_retry(float x, float y, Waypoint &retry_wp) {
    ObstacleRecord *obs = findNearest(x, y);
    if (obs == nullptr || !obs->retry_pending) return false;

    const float retry_heading = wrapAngle(obs->approach_heading + (float)M_PI);

    retry_wp.x       = obs->x + cosf(retry_heading) * mower_config_get().cut_width_m;
    retry_wp.y       = obs->y + sinf(retry_heading) * mower_config_get().cut_width_m;
    retry_wp.heading = retry_heading;
    retry_wp.mowing  = true;

    // Consume the retry — it will not be returned again.
    obs->retry_pending = false;

    DBG_PRINT(F("[obstacle_map] retry WP: ("));
    DBG_PRINT(retry_wp.x, 2); DBG_PRINT(F(", ")); DBG_PRINT(retry_wp.y, 2);
    DBG_PRINT(F(") hdg="));
    DBG_PRINT(retry_heading * 57.2958f, 1);
    DBG_PRINTLN(F("°"));
    return true;
}

void obstacle_mark_retry_succeeded(float x, float y) {
    ObstacleRecord *obs = findNearest(x, y);
    if (obs != nullptr) {
        obs->retry_succeeded = true;
        DBG_PRINTLN(F("[obstacle_map] retry marked succeeded"));
    }
}

int obstacle_get_count() {
    return g_obs_active;
}

void obstacle_dump() {
    DBG_PRINTLN(F("=== Obstacle Records ==="));
    int printed = 0;
    for (int i = 0; i < MAX_OBSTACLES; ++i) {
        if (!g_obstacles[i].active) continue;
        DBG_PRINT(F("  ["));
        DBG_PRINT(i);
        DBG_PRINT(F("] ("));
        DBG_PRINT(g_obstacles[i].x, 2);
        DBG_PRINT(F(", "));
        DBG_PRINT(g_obstacles[i].y, 2);
        DBG_PRINT(F(") hdg="));
        DBG_PRINT(g_obstacles[i].approach_heading * 57.2958f, 1);
        DBG_PRINT(F("°  retry_pending="));
        DBG_PRINT(g_obstacles[i].retry_pending ? 'Y' : 'N');
        DBG_PRINT(F("  retry_ok="));
        DBG_PRINTLN(g_obstacles[i].retry_succeeded ? 'Y' : 'N');
        ++printed;
    }
    if (printed == 0) DBG_PRINTLN(F("  (none)"));
    DBG_PRINT(F("Total active: "));
    DBG_PRINTLN(g_obs_active);
}

// ══════════════════════════════════════════════════════════════════════════════
//  Coverage grid
// ══════════════════════════════════════════════════════════════════════════════

void grid_mark_arc_swept(float cx, float cy, float radius_m) {
    if (g_cell_size_m <= 0.0f || g_grid_width == 0) return;

    // Determine bounding box of circle in grid coordinates.
    int col_min = (int)floorf((cx - radius_m - g_grid_origin_x) / g_cell_size_m);
    int col_max = (int)ceilf ((cx + radius_m - g_grid_origin_x) / g_cell_size_m);
    int row_min = (int)floorf((cy - radius_m - g_grid_origin_y) / g_cell_size_m);
    int row_max = (int)ceilf ((cy + radius_m - g_grid_origin_y) / g_cell_size_m);

    // Clamp to grid bounds.
    if (col_min < 0)              col_min = 0;
    if (col_max >= g_grid_width)  col_max = g_grid_width  - 1;
    if (row_min < 0)              row_min = 0;
    if (row_max >= g_grid_height) row_max = g_grid_height - 1;

    const float r2 = radius_m * radius_m;

    for (int r = row_min; r <= row_max; ++r) {
        for (int c = col_min; c <= col_max; ++c) {
            // Use cell centre for circle membership test.
            float wcx, wcy;
            gridToWorld(c, r, wcx, wcy);
            float dx = wcx - cx;
            float dy = wcy - cy;
            if ((dx * dx + dy * dy) <= r2) {
                uint8_t &cell = g_grid[r][c];
                if (cell != CELL_OBSTACLE) {
                    cell = CELL_MOWED_ARC;
                }
            }
        }
    }
}

void grid_mark_strip(float x, float y) {
    int col, row;
    if (!worldToGrid(x, y, col, row)) return;

    uint8_t &cell = g_grid[row][col];
    if (cell == CELL_OBSTACLE) return;

    if (cell != CELL_MOWED_STRIP) {
        cell = CELL_MOWED_STRIP;
        ++g_session_mowed_cells;  // only count fresh transitions
    }
}

void grid_mark_obstacle(float x, float y) {
    int col, row;
    if (!worldToGrid(x, y, col, row)) return;
    g_grid[row][col] = CELL_OBSTACLE;
}

bool grid_is_mowed(float x, float y) {
    int col, row;
    if (!worldToGrid(x, y, col, row)) return false;
    uint8_t s = g_grid[row][col];
    return (s == CELL_MOWED_STRIP || s == CELL_MOWED_ARC);
}

float grid_get_headland_coverage() {
    if (g_grid_width == 0 || g_grid_height == 0) return 0.0f;

    long total_mowable = 0;
    long mowed         = 0;

    for (int r = 0; r < g_grid_height; ++r) {
        for (int c = 0; c < g_grid_width; ++c) {
            uint8_t s = g_grid[r][c];
            if (s == CELL_OBSTACLE || s == CELL_UNKNOWN) continue;
            ++total_mowable;
            if (s == CELL_MOWED_STRIP || s == CELL_MOWED_ARC) ++mowed;
        }
    }

    if (total_mowable == 0) return 0.0f;
    return (float)mowed / (float)total_mowable;
}

void grid_dump_ascii() {
    if (g_grid_width == 0 || g_grid_height == 0) {
        DBG_PRINTLN(F("[obstacle_map] grid not initialised"));
        return;
    }

    DBG_PRINT(F("=== Coverage Grid ("));
    DBG_PRINT(g_grid_width); DBG_PRINT('x'); DBG_PRINT(g_grid_height);
    DBG_PRINT(F(")  cell="));
    DBG_PRINT(g_cell_size_m * 1000.0f, 0);
    DBG_PRINTLN(F("mm ==="));
    DBG_PRINTLN(F("  (. UNMOWED  # STRIP  o ARC  X OBSTACLE  ? UNKNOWN)"));

    // Print rows top-to-bottom (high row index = top of map).
    for (int r = g_grid_height - 1; r >= 0; --r) {
        // Row label every 16 rows to keep output manageable.
        if (r % 16 == 0 || r == g_grid_height - 1) {
            char buf[8];
            snprintf(buf, sizeof(buf), "%3d| ", r);
            DBG_PRINT(buf);
        } else {
            DBG_PRINT(F("    | "));
        }

        for (int c = 0; c < g_grid_width; ++c) {
            char ch;
            switch (g_grid[r][c]) {
                case CELL_UNMOWED:     ch = '.'; break;
                case CELL_MOWED_STRIP: ch = '#'; break;
                case CELL_MOWED_ARC:   ch = 'o'; break;
                case CELL_OBSTACLE:    ch = 'X'; break;
                default:               ch = '?'; break;
            }
            DBG_PRINT(ch);
        }
        DBG_PRINTLN();
    }

    DBG_PRINT(F("Coverage: "));
    DBG_PRINT(grid_get_headland_coverage() * 100.0f, 1);
    DBG_PRINTLN('%');
}

uint32_t grid_get_session_mowed_dm2() {
    // Cell area in dm²: (cell_size_m × 10 dm/m)²
    float cell_dm = g_cell_size_m * 10.0f;  // metres → decimetres
    float cell_area_dm2 = cell_dm * cell_dm;
    return (uint32_t)((float)g_session_mowed_cells * cell_area_dm2);
}

void grid_enumerate_mowed(std::vector<std::pair<uint8_t,uint8_t>> &out) {
    out.clear();
    if (g_grid_width == 0 || g_grid_height == 0) return;
    static constexpr int MAX_MOWED_OUT = 2048;
    for (int row = 0; row < g_grid_height && (int)out.size() < MAX_MOWED_OUT; row++) {
        for (int col = 0; col < g_grid_width && (int)out.size() < MAX_MOWED_OUT; col++) {
            uint8_t cell = g_grid[row][col];
            if (cell == CELL_MOWED_STRIP || cell == CELL_MOWED_ARC) {
                out.emplace_back((uint8_t)col, (uint8_t)row);
            }
        }
    }
}

int   grid_get_cols()        { return g_grid_width; }
int   grid_get_rows()        { return g_grid_height; }
float grid_get_cell_size_m() { return g_cell_size_m; }
float grid_get_origin_x()    { return g_grid_origin_x; }
float grid_get_origin_y()    { return g_grid_origin_y; }
