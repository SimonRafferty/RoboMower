#pragma once
#include "geometry.h"   // Point, Polygon, distanceToNearestEdge, clampf

// ── RAM-only obstacle disc overlay (planning keep-out, never a safety boundary).
//    Source-tagged so a future LIDAR/RADAR node or PWA no-go zone is just another
//    source — the detour logic below does not change. No NVS, no Arduino deps.

#define OBSTACLE_DISCS_MAX 32

enum DiscSource {
    DISC_COLLISION = 0,
    DISC_TILT,
    DISC_SLIP,
    DISC_SENSOR,   // reserved: future LIDAR/RADAR (do not implement now)
    DISC_NOGO,     // reserved: future PWA-drawn no-go zone (do not implement now)
};

struct ObstacleDisc {
    float      x;     // ENU east  (m)
    float      y;     // ENU north (m)
    float      r;     // radius (m) — pivot-sweep radius from mower_config_nav_inset_m()
    DiscSource src;
};

// ── List management (cleared only on a true AUTO-cycle restart; see spec §5.5).
void obstacle_discs_clear();
bool obstacle_discs_add(float x, float y, float r, DiscSource src); // false if list full
int  obstacle_discs_count();
bool obstacle_discs_get(int i, ObstacleDisc *out);                  // false if i out of range

// ── Pure geometry (host-testable; tuning passed in, not read from config).

// Index of the FIRST active disc the straight leg [P->W] enters (centre within
// r + clearance of the segment), or -1 if the leg is clear of all discs.
int  obstacle_discs_segment_blocked(float px, float py, float wx, float wy,
                                    float clearance);

// Plan a detour from P to W that avoids all active discs, staying inside `perim`.
// Writes up to max_vias via-points into vias_out (caller-allocated Point[max_vias]).
// Returns: number of via-points written (0 = straight leg already clear), or
//          -1 if it cannot be cleared within max_vias / no safe interior side
//          exists (caller PAUSEs). Deterministic; no global state except the list.
int  obstacle_discs_plan_detour(float px, float py, float wx, float wy,
                                const Polygon &perim,
                                float clearance, float offset_margin, int max_vias,
                                Point *vias_out);
