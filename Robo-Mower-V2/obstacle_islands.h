#pragma once
#include "geometry.h"   // Point, Polygon, distanceToNearestEdge, clampf

// ── RAM-only obstacle island overlay (planning keep-out, never a safety boundary).
//    Source-tagged so a future LIDAR/RADAR node or PWA no-go zone is just another
//    source — the detour logic below does not change. No NVS, no Arduino deps.

#define OBSTACLE_ISLANDS_MAX 32

enum IslandSource {
    ISLAND_COLLISION = 0,
    ISLAND_TILT,
    ISLAND_SLIP,
    ISLAND_SENSOR,   // reserved: future LIDAR/RADAR (do not implement now)
    ISLAND_NOGO,     // reserved: future PWA-drawn no-go zone (do not implement now)
};

struct ObstacleIsland {
    float        x;     // ENU east  (m)
    float        y;     // ENU north (m)
    float        r;     // radius (m) — pivot-sweep radius from mower_config_nav_inset_m()
    IslandSource src;
};

// ── List management (cleared only on a true AUTO-cycle restart; see spec §5.5).
void obstacle_islands_clear();
bool obstacle_islands_add(float x, float y, float r, IslandSource src); // false if list full
int  obstacle_islands_count();
bool obstacle_islands_get(int i, ObstacleIsland *out);                  // false if i out of range

// ── Pure geometry (host-testable; tuning passed in, not read from config).

// Index of the FIRST active island the straight leg [P->W] enters (centre within
// r + clearance of the segment), or -1 if the leg is clear of all islands.
int  obstacle_islands_segment_blocked(float px, float py, float wx, float wy,
                                    float clearance);

// Plan a detour from P to W that avoids all active islands, staying inside `perim`.
// Writes up to max_vias via-points into vias_out (caller-allocated Point[max_vias]).
// Returns: number of via-points written (0 = straight leg already clear), or
//          -1 if it cannot be cleared within max_vias / no safe interior side
//          exists (caller PAUSEs). Deterministic; no global state except the list.
int  obstacle_islands_plan_detour(float px, float py, float wx, float wy,
                                const Polygon &perim,
                                float clearance, float offset_margin, int max_vias,
                                Point *vias_out);
