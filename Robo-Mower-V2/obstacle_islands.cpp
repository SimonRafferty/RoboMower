#include "obstacle_islands.h"
#include <cmath>

static ObstacleIsland s_island[OBSTACLE_ISLANDS_MAX];
static int            s_count = 0;

void obstacle_islands_clear() { s_count = 0; }

bool obstacle_islands_add(float x, float y, float r, IslandSource src) {
    if (s_count >= OBSTACLE_ISLANDS_MAX) return false;
    s_island[s_count++] = ObstacleIsland{ x, y, r, src };
    return true;
}

int obstacle_islands_count() { return s_count; }

bool obstacle_islands_get(int i, ObstacleIsland *out) {
    if (i < 0 || i >= s_count || !out) return false;
    *out = s_island[i];
    return true;
}

// Shortest distance from point (px,py) to segment [a,b]; foot returned if requested.
static float seg_point_dist(float px, float py,
                            float ax, float ay, float bx, float by,
                            float *footx, float *footy) {
    float dx = bx - ax, dy = by - ay;
    float L2 = dx * dx + dy * dy;
    float t  = (L2 > 1e-9f) ? ((px - ax) * dx + (py - ay) * dy) / L2 : 0.0f;
    t = clampf(t, 0.0f, 1.0f);
    float fx = ax + t * dx, fy = ay + t * dy;
    if (footx) *footx = fx;
    if (footy) *footy = fy;
    float ex = px - fx, ey = py - fy;
    return sqrtf(ex * ex + ey * ey);
}

int obstacle_islands_segment_blocked(float px, float py, float wx, float wy,
                                   float clearance) {
    for (int i = 0; i < s_count; i++) {
        float d = seg_point_dist(s_island[i].x, s_island[i].y, px, py, wx, wy, nullptr, nullptr);
        if (d < s_island[i].r + clearance) return i;
    }
    return -1;
}

// Two via-points skirting island D on the perimeter-interior side, offset from the
// island CENTRE (not the leg line) so clearance holds even when D is off the leg.
// Returns false if neither side keeps both points inside perim.
static bool island_bracket(const ObstacleIsland &D,
                         float px, float py, float wx, float wy,
                         float offset_margin, const Polygon &perim,
                         Point *v1, Point *v2) {
    float dx = wx - px, dy = wy - py;
    float L  = sqrtf(dx * dx + dy * dy);
    if (L < 1e-4f) return false;
    float ux = dx / L, uy = dy / L;                 // unit along the original leg
    float nx = -uy, ny = ux;                         // left normal
    float off  = D.r + offset_margin;                // perpendicular offset FROM THE CENTRE
    float half = D.r + offset_margin;                // along-leg bracket half-length (~45 deg)

    // Pick the side (sign of normal) with greater interior clearance, measured at
    // the offset point relative to the island centre.
    float cl_l = distanceToNearestEdge(perim, D.x + nx * off, D.y + ny * off);
    float cl_r = distanceToNearestEdge(perim, D.x - nx * off, D.y - ny * off);
    float sgn  = (cl_l >= cl_r) ? 1.0f : -1.0f;
    float bnx = nx * sgn, bny = ny * sgn;

    // Bracket straddles the island CENTRE along the leg, offset perpendicular from it.
    Point a{ D.x - ux * half + bnx * off, D.y - uy * half + bny * off };
    Point b{ D.x + ux * half + bnx * off, D.y + uy * half + bny * off };
    if (distanceToNearestEdge(perim, a.x, a.y) <= 0.0f) return false;
    if (distanceToNearestEdge(perim, b.x, b.y) <= 0.0f) return false;
    *v1 = a; *v2 = b;
    return true;
}

int obstacle_islands_plan_detour(float px, float py, float wx, float wy,
                               const Polygon &perim,
                               float clearance, float offset_margin, int max_vias,
                               Point *vias_out) {
    float cx = px, cy = py;
    int n = 0;
    for (int guard = 0; guard < 64; guard++) {       // guard: belt-and-braces; real bound is max_vias
        int idx = obstacle_islands_segment_blocked(cx, cy, wx, wy, clearance);
        if (idx < 0) return n;                       // leg to W is clear
        if (n + 2 > max_vias) return -1;             // can't fit another bracket
        Point v1, v2;
        if (!island_bracket(s_island[idx], px, py, wx, wy, offset_margin, perim, &v1, &v2))
            return -1;                               // no safe interior side
        // Re-validate the two NEW legs we are about to commit; bail to PAUSE if
        // either is still blocked (e.g. another island in the way).
        if (obstacle_islands_segment_blocked(cx, cy, v1.x, v1.y, clearance) != -1) return -1;
        if (obstacle_islands_segment_blocked(v1.x, v1.y, v2.x, v2.y, clearance) != -1) return -1;
        vias_out[n++] = v1;
        vias_out[n++] = v2;
        cx = v2.x; cy = v2.y;                        // re-check v2 -> W on the next iteration
    }
    return -1;                                        // iteration guard tripped -> PAUSE
}
