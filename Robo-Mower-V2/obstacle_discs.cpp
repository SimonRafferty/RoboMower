#include "obstacle_discs.h"
#include <cmath>

static ObstacleDisc s_disc[OBSTACLE_DISCS_MAX];
static int          s_count = 0;

void obstacle_discs_clear() { s_count = 0; }

bool obstacle_discs_add(float x, float y, float r, DiscSource src) {
    if (s_count >= OBSTACLE_DISCS_MAX) return false;
    s_disc[s_count++] = ObstacleDisc{ x, y, r, src };
    return true;
}

int obstacle_discs_count() { return s_count; }

bool obstacle_discs_get(int i, ObstacleDisc *out) {
    if (i < 0 || i >= s_count || !out) return false;
    *out = s_disc[i];
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

int obstacle_discs_segment_blocked(float px, float py, float wx, float wy,
                                   float clearance) {
    for (int i = 0; i < s_count; i++) {
        float d = seg_point_dist(s_disc[i].x, s_disc[i].y, px, py, wx, wy, nullptr, nullptr);
        if (d < s_disc[i].r + clearance) return i;
    }
    return -1;
}

// Two via-points skirting disc D on the perimeter-interior side of leg [P->W].
// Returns false if neither side keeps both points inside perim.
static bool disc_bracket(const ObstacleDisc &D,
                         float px, float py, float wx, float wy,
                         float offset_margin, const Polygon &perim,
                         Point *v1, Point *v2) {
    float dx = wx - px, dy = wy - py;
    float L  = sqrtf(dx * dx + dy * dy);
    if (L < 1e-4f) return false;
    float ux = dx / L, uy = dy / L;                 // unit along leg
    float tt = (D.x - px) * ux + (D.y - py) * uy;   // disc-centre projection onto line
    float fx = px + tt * ux, fy = py + tt * uy;     // foot on the infinite line
    float off  = D.r + offset_margin;               // perpendicular offset
    float half = D.r + offset_margin;               // along-line bracket half-length (~45 deg)
    float nx = -uy, ny = ux;                         // left normal

    // Pick the side (sign of normal) with greater interior clearance at the foot.
    float cl_l = distanceToNearestEdge(perim, fx + nx * off, fy + ny * off);
    float cl_r = distanceToNearestEdge(perim, fx - nx * off, fy - ny * off);
    float sgn  = (cl_l >= cl_r) ? 1.0f : -1.0f;
    float bnx = nx * sgn, bny = ny * sgn;

    Point a{ fx - ux * half + bnx * off, fy - uy * half + bny * off };
    Point b{ fx + ux * half + bnx * off, fy + uy * half + bny * off };
    if (distanceToNearestEdge(perim, a.x, a.y) <= 0.0f) return false;
    if (distanceToNearestEdge(perim, b.x, b.y) <= 0.0f) return false;
    *v1 = a; *v2 = b;
    return true;
}

int obstacle_discs_plan_detour(float px, float py, float wx, float wy,
                               const Polygon &perim,
                               float clearance, float offset_margin, int max_vias,
                               Point *vias_out) {
    float cx = px, cy = py;
    int n = 0;
    for (int guard = 0; guard < 64; guard++) {
        int idx = obstacle_discs_segment_blocked(cx, cy, wx, wy, clearance);
        if (idx < 0) return n;                       // leg to W is clear
        if (n + 2 > max_vias) return -1;             // can't fit another bracket
        Point v1, v2;
        if (!disc_bracket(s_disc[idx], px, py, wx, wy, offset_margin, perim, &v1, &v2))
            return -1;                               // no safe interior side
        vias_out[n++] = v1;
        vias_out[n++] = v2;
        cx = v2.x; cy = v2.y;                        // continue from the bracket exit
    }
    return -1;                                        // iteration guard tripped -> PAUSE
}
