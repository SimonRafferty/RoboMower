// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  geometry.cpp â€” RoboMower polygon mathematics
//
//  See geometry.h for public API and algorithm descriptions.
//
//  Float precision: all geometry uses float (ESP32-S3 hardware FPU).
//  Epsilons:
//    EPSILON_VERTEX   1e-4f  â€” vertex coincidence (0.1 mm)
//    EPSILON_PARALLEL 1e-6f  â€” line intersection denominator guard
//
//  Dependency: config.h for MIN_ZONE_AREA_M2
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

#include "geometry.h"
#include "host_config.h"

#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>

using std::vector;

// â”€â”€ Local constants â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€
static constexpr float EPSILON_VERTEX   = 1e-4f;  ///< vertex coincidence check (m)
static constexpr float EPSILON_PARALLEL = 1e-6f;  ///< line intersection denominator guard
static constexpr float ARC_INTERVAL_M   = 0.05f;  ///< arc fillet sample spacing (m)

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  Utility helpers (file-local)
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

/** 2D cross product of vectors (ax,ay) and (bx,by) */
static inline float cross2(float ax, float ay, float bx, float by)
{
    return ax * by - ay * bx;
}

/** Squared distance between two points */
static inline float dist2(Point a, Point b)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return dx*dx + dy*dy;
}

/** Euclidean distance between two points */
static inline float dist(Point a, Point b)
{
    return sqrtf(dist2(a, b));
}

/**
 * Compute the intersection of two infinite lines.
 * Line 1: passes through p1 with direction d1.
 * Line 2: passes through p2 with direction d2.
 * Returns false if lines are parallel (|denom| < EPSILON_PARALLEL).
 */
static bool lineIntersect(float p1x, float p1y, float d1x, float d1y,
                           float p2x, float p2y, float d2x, float d2y,
                           float &outX, float &outY)
{
    float denom = cross2(d1x, d1y, d2x, d2y);
    if (fabsf(denom) < EPSILON_PARALLEL) {
        return false;  // parallel lines
    }
    float dx = p2x - p1x;
    float dy = p2y - p1y;
    float t = cross2(dx, dy, d2x, d2y) / denom;
    outX = p1x + t * d1x;
    outY = p1y + t * d1y;
    return true;
}

/**
 * Test whether segment (p,p+r) intersects segment (q,q+s).
 * If they intersect, t and u are the parameters (0..1).
 * Returns true if proper intersection found.
 */
static bool segmentIntersectParam(float px, float py, float rx, float ry,
                                   float qx, float qy, float sx, float sy,
                                   float &t, float &u)
{
    float denom = cross2(rx, ry, sx, sy);
    float dx = qx - px;
    float dy = qy - py;
    if (fabsf(denom) < EPSILON_PARALLEL) {
        return false;  // parallel
    }
    t = cross2(dx, dy, sx, sy) / denom;
    u = cross2(dx, dy, rx, ry) / denom;
    return (t > EPSILON_PARALLEL && t < 1.0f - EPSILON_PARALLEL &&
            u > EPSILON_PARALLEL && u < 1.0f - EPSILON_PARALLEL);
}

/** Distance from point (px,py) to the segment (ax,ay)-(bx,by) */
static float pointToSegmentDist(float px, float py,
                                 float ax, float ay, float bx, float by)
{
    float dx = bx - ax;
    float dy = by - ay;
    float lenSq = dx*dx + dy*dy;
    if (lenSq < EPSILON_PARALLEL) {
        float ex = px - ax;
        float ey = py - ay;
        return sqrtf(ex*ex + ey*ey);
    }
    float t = ((px - ax)*dx + (py - ay)*dy) / lenSq;
    t = clampf(t, 0.0f, 1.0f);
    float cx = ax + t*dx;
    float cy = ay + t*dy;
    float ex = px - cx;
    float ey = py - cy;
    return sqrtf(ex*ex + ey*ey);
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  Public utility functions
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

float wrapAngle(float angle_rad)
{
    return atan2f(sinf(angle_rad), cosf(angle_rad));
}

float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  Polygon member functions
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

float Polygon::area() const
{
    const int n = static_cast<int>(pts.size());
    if (n < 3) return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        sum += pts[i].x * pts[j].y;
        sum -= pts[j].x * pts[i].y;
    }
    return sum * 0.5f;
}

bool Polygon::isClockwise() const
{
    return area() < 0.0f;
}

void Polygon::ensureCCW()
{
    if (isClockwise()) {
        std::reverse(pts.begin(), pts.end());
    }
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  isInsidePolygon â€” ray-casting
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

bool isInsidePolygon(const Polygon &poly, float px, float py)
{
    const int n = static_cast<int>(poly.pts.size());
    if (n < 3) return false;

    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        float xi = poly.pts[i].x, yi = poly.pts[i].y;
        float xj = poly.pts[j].x, yj = poly.pts[j].y;

        float segLen = pointToSegmentDist(px, py, xi, yi, xj, yj);
        if (segLen < EPSILON_VERTEX) {
            return true;  // on boundary counts as inside
        }

        bool yBrackets = ((yi > py) != (yj > py));
        if (yBrackets) {
            float xIntersect = (xj - xi) * (py - yi) / (yj - yi) + xi;
            if (px < xIntersect) {
                inside = !inside;
            }
        }
    }
    return inside;
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  distanceToNearestEdge
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

float distanceToNearestEdge(const Polygon &poly, float px, float py)
{
    const int n = static_cast<int>(poly.pts.size());
    if (n < 2) return 0.0f;

    float minDist = 1e30f;
    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        float d = pointToSegmentDist(px, py,
                                     poly.pts[i].x, poly.pts[i].y,
                                     poly.pts[j].x, poly.pts[j].y);
        if (d < minDist) minDist = d;
    }

    return isInsidePolygon(poly, px, py) ? minDist : -minDist;
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  sampleArc
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

std::vector<Point> sampleArc(float cx, float cy, float r,
                              float start_angle, float end_angle,
                              float arc_interval_m)
{
    vector<Point> result;

    if (r < EPSILON_PARALLEL || arc_interval_m < EPSILON_PARALLEL) {
        return result;
    }

    float totalAngle = end_angle - start_angle;
    float angStep = arc_interval_m / r;
    if (angStep < EPSILON_PARALLEL) return result;

    int nSteps = static_cast<int>(fabsf(totalAngle) / angStep);
    if (nSteps < 1) nSteps = 1;

    float step = totalAngle / static_cast<float>(nSteps);

    for (int i = 0; i <= nSteps; ++i) {
        float angle = start_angle + step * static_cast<float>(i);
        result.push_back(Point(cx + r * cosf(angle), cy + r * sinf(angle)));
    }
    return result;
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  isSelfIntersecting
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

bool isSelfIntersecting(const Polygon &poly)
{
    const int n = static_cast<int>(poly.pts.size());
    if (n < 4) return false;

    for (int i = 0; i < n; ++i) {
        int i1 = (i + 1) % n;
        float px = poly.pts[i].x,  py = poly.pts[i].y;
        float rx = poly.pts[i1].x - px;
        float ry = poly.pts[i1].y - py;

        for (int j = i + 2; j < n; ++j) {
            if (j == n - 1 && i == 0) continue;

            int j1 = (j + 1) % n;
            float qx = poly.pts[j].x,  qy = poly.pts[j].y;
            float sx = poly.pts[j1].x - qx;
            float sy = poly.pts[j1].y - qy;

            float t, u;
            if (segmentIntersectParam(px, py, rx, ry, qx, qy, sx, sy, t, u)) {
                return true;
            }
        }
    }
    return false;
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  splitSelfIntersecting
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

/** Recursive worker â€” assumes the TOP-LEVEL input was normalised to CCW.
 *  Sub-polygons produced by splitting inherit their winding from the split:
 *  a CW-wound piece (signed area < 0) is a spur or fold â€” discard it. */
static std::vector<Polygon> splitSelfIntersectingRec(const Polygon &poly)
{
    const int n = static_cast<int>(poly.pts.size());

    if (n < 3) return {};

    // Signed area: CW (negative) â‡’ spur/fold piece from a parent split.
    float a = poly.area();
    if (a < MIN_ZONE_AREA_M2) return {};

    if (!isSelfIntersecting(poly)) {
        Polygon p = poly;
        p.ensureCCW();
        return {p};
    }

    for (int i = 0; i < n; ++i) {
        int i1 = (i + 1) % n;
        float px = poly.pts[i].x,  py = poly.pts[i].y;
        float rx = poly.pts[i1].x - px;
        float ry = poly.pts[i1].y - py;

        for (int j = i + 2; j < n; ++j) {
            if (j == n - 1 && i == 0) continue;

            int j1 = (j + 1) % n;
            float qx = poly.pts[j].x,  qy = poly.pts[j].y;
            float sx = poly.pts[j1].x - qx;
            float sy = poly.pts[j1].y - qy;

            float t, u;
            if (segmentIntersectParam(px, py, rx, ry, qx, qy, sx, sy, t, u)) {
                float ix = px + t * rx;
                float iy = py + t * ry;

                Polygon a_poly, b_poly;
                a_poly.pts.push_back(Point(ix, iy));
                for (int k = i1; k <= j; ++k) {
                    a_poly.pts.push_back(poly.pts[k]);
                }

                b_poly.pts.push_back(Point(ix, iy));
                for (int k = j1; k != i1; k = (k + 1) % n) {
                    b_poly.pts.push_back(poly.pts[k]);
                }

                vector<Polygon> result;
                auto subA = splitSelfIntersectingRec(a_poly);
                auto subB = splitSelfIntersectingRec(b_poly);
                result.insert(result.end(), subA.begin(), subA.end());
                result.insert(result.end(), subB.begin(), subB.end());
                return result;
            }
        }
    }

    Polygon p = poly;
    p.ensureCCW();
    return {p};
}

std::vector<Polygon> splitSelfIntersecting(const Polygon &poly)
{
    // Normalise the CALLER's winding before splitting. The signed-area spur
    // filter in the recursion must only ever see winding produced by the
    // split itself â€” a CW-wound but otherwise valid input (e.g. a perimeter
    // blob recorded by older firmware, or a hand-drawn PWA polygon) would
    // otherwise be discarded wholesale, returning an empty result.
    Polygon work = poly;
    if (work.pts.size() >= 3) work.ensureCCW();
    return splitSelfIntersectingRec(work);
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  insetPolygon
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

Polygon insetPolygon(const Polygon &poly, float inset_m)
{
    Polygon work = poly;
    work.ensureCCW();

    const int n = static_cast<int>(work.pts.size());
    if (n < 3) return Polygon{};

    struct OffsetEdge {
        float ox, oy;
        float dx, dy;
        float nx, ny;
    };

    vector<OffsetEdge> edges;
    edges.reserve(n);

    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        float ex = work.pts[j].x - work.pts[i].x;
        float ey = work.pts[j].y - work.pts[i].y;
        float len = sqrtf(ex*ex + ey*ey);
        if (len < EPSILON_VERTEX) {
            if (!edges.empty()) edges.push_back(edges.back());
            continue;
        }
        float ux = ex / len, uy = ey / len;
        float inx = -uy, iny = ux;

        OffsetEdge e;
        e.dx = ux; e.dy = uy;
        e.nx = inx; e.ny = iny;
        e.ox = work.pts[i].x + inx * inset_m;
        e.oy = work.pts[i].y + iny * inset_m;
        edges.push_back(e);
    }

    const int ne = static_cast<int>(edges.size());
    if (ne < 3) return Polygon{};

    Polygon result;
    result.pts.reserve(ne + 32);

    for (int i = 0; i < ne; ++i) {
        int prev = (i - 1 + ne) % ne;
        const OffsetEdge &eA = edges[prev];
        const OffsetEdge &eB = edges[i];

        float ix, iy;
        bool intersected = lineIntersect(eA.ox, eA.oy, eA.dx, eA.dy,
                                          eB.ox, eB.oy, eB.dx, eB.dy,
                                          ix, iy);

        // Use the intersection for BOTH convex and concave corners.
        // At a concave corner (crossAB < 0) the inset lines still converge to a
        // valid point inside the polygon â€” using the intersection is correct and
        // avoids the arc extending outside the original polygon boundary.
        // Only fall back to an arc when the lines are parallel (no intersection).
        if (!intersected) {
            float cx = work.pts[i].x;
            float cy = work.pts[i].y;

            float startAngle = atan2f(eA.ny, eA.nx);
            float endAngle   = atan2f(eB.ny, eB.nx);
            float sweep = wrapAngle(endAngle - startAngle);

            auto arcPts = sampleArc(cx, cy, inset_m, startAngle, startAngle + sweep,
                                     ARC_INTERVAL_M);
            for (const auto &ap : arcPts) {
                result.pts.push_back(ap);
            }
        } else {
            result.pts.push_back(Point(ix, iy));
        }
    }

    // Remove coincident consecutive vertices
    {
        const int rn = static_cast<int>(result.pts.size());
        vector<Point> deduped;
        deduped.reserve(rn);
        for (int i = 0; i < rn; ++i) {
            int j = (i + 1) % rn;
            if (dist(result.pts[i], result.pts[j]) >= EPSILON_VERTEX) {
                deduped.push_back(result.pts[i]);
            }
        }
        result.pts = deduped;
    }

    if (static_cast<int>(result.pts.size()) < 3) return Polygon{};
    if (fabsf(result.area()) < MIN_ZONE_AREA_M2) return Polygon{};

    if (isSelfIntersecting(result)) {
        auto subpolys = splitSelfIntersecting(result);
        if (subpolys.empty()) return Polygon{};

        Polygon *largest = &subpolys[0];
        for (auto &sp : subpolys) {
            if (fabsf(sp.area()) > fabsf(largest->area())) {
                largest = &sp;
            }
        }
        Polygon ret = *largest;
        ret.ensureCCW();
        return ret;
    }

    result.ensureCCW();
    return result;
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  insetPolygonMulti â€” returns ALL sub-polygons (not just the largest)
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

std::vector<Polygon> insetPolygonMulti(const Polygon &poly, float inset_m)
{
    // Build the raw inset polygon (same code as insetPolygon up to self-intersection check)
    Polygon work = poly;
    work.ensureCCW();

    const int n = static_cast<int>(work.pts.size());
    if (n < 3) return {};

    struct OffsetEdge {
        float ox, oy;
        float dx, dy;
        float nx, ny;
    };

    vector<OffsetEdge> edges;
    edges.reserve(n);

    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        float ex = work.pts[j].x - work.pts[i].x;
        float ey = work.pts[j].y - work.pts[i].y;
        float len = sqrtf(ex*ex + ey*ey);
        if (len < EPSILON_VERTEX) {
            if (!edges.empty()) edges.push_back(edges.back());
            continue;
        }
        float ux = ex / len, uy = ey / len;
        float inx = -uy, iny = ux;

        OffsetEdge e;
        e.dx = ux; e.dy = uy;
        e.nx = inx; e.ny = iny;
        e.ox = work.pts[i].x + inx * inset_m;
        e.oy = work.pts[i].y + iny * inset_m;
        edges.push_back(e);
    }

    const int ne = static_cast<int>(edges.size());
    if (ne < 3) return {};

    Polygon result;
    result.pts.reserve(ne + 32);

    for (int i = 0; i < ne; ++i) {
        int prev = (i - 1 + ne) % ne;
        const OffsetEdge &eA = edges[prev];
        const OffsetEdge &eB = edges[i];

        float ix, iy;
        bool intersected = lineIntersect(eA.ox, eA.oy, eA.dx, eA.dy,
                                          eB.ox, eB.oy, eB.dx, eB.dy,
                                          ix, iy);

        // Same fix as insetPolygon: use intersection for both convex and concave
        // corners; arc only when lines are parallel.
        if (!intersected) {
            float cx = work.pts[i].x;
            float cy = work.pts[i].y;

            float startAngle = atan2f(eA.ny, eA.nx);
            float endAngle   = atan2f(eB.ny, eB.nx);
            float sweep = wrapAngle(endAngle - startAngle);

            auto arcPts = sampleArc(cx, cy, inset_m, startAngle, startAngle + sweep,
                                     ARC_INTERVAL_M);
            for (const auto &ap : arcPts) {
                result.pts.push_back(ap);
            }
        } else {
            result.pts.push_back(Point(ix, iy));
        }
    }

    // Remove coincident consecutive vertices
    {
        const int rn = static_cast<int>(result.pts.size());
        vector<Point> deduped;
        deduped.reserve(rn);
        for (int i = 0; i < rn; ++i) {
            int j = (i + 1) % rn;
            if (dist(result.pts[i], result.pts[j]) >= EPSILON_VERTEX) {
                deduped.push_back(result.pts[i]);
            }
        }
        result.pts = deduped;
    }

    if (static_cast<int>(result.pts.size()) < 3) return {};

    if (isSelfIntersecting(result)) {
        auto subpolys = splitSelfIntersecting(result);
        // Keep ALL valid sub-polygons (not just the largest)
        std::vector<Polygon> valid;
        for (auto &sp : subpolys) {
            if (sp.pts.size() >= 3 && fabsf(sp.area()) >= MIN_ZONE_AREA_M2) {
                sp.ensureCCW();
                valid.push_back(sp);
            }
        }
        return valid;
    }

    if (fabsf(result.area()) < MIN_ZONE_AREA_M2) return {};
    result.ensureCCW();
    return { result };
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  convexHull â€” Graham scan
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

Polygon convexHull(const Polygon &poly)
{
    const int n = static_cast<int>(poly.pts.size());
    if (n < 3) {
        Polygon p = poly;
        return p;
    }

    vector<Point> pts = poly.pts;

    // Find lowest-y (then lowest-x) point as pivot
    int pivotIdx = 0;
    for (int i = 1; i < n; ++i) {
        if (pts[i].y < pts[pivotIdx].y ||
           (pts[i].y == pts[pivotIdx].y && pts[i].x < pts[pivotIdx].x)) {
            pivotIdx = i;
        }
    }
    std::swap(pts[0], pts[pivotIdx]);
    Point pivot = pts[0];

    // Sort remaining points by polar angle from pivot
    std::sort(pts.begin() + 1, pts.end(),
        [&pivot](const Point &a, const Point &b) {
            float ax = a.x - pivot.x, ay = a.y - pivot.y;
            float bx = b.x - pivot.x, by = b.y - pivot.y;
            float c = cross2(ax, ay, bx, by);
            if (fabsf(c) > EPSILON_PARALLEL) return c > 0.0f;
            float da = ax*ax + ay*ay;
            float db = bx*bx + by*by;
            return da < db;
        });

    // Graham scan
    vector<Point> hull;
    hull.reserve(n);
    for (int i = 0; i < n; ++i) {
        while (hull.size() >= 2) {
            auto &a = hull[hull.size() - 2];
            auto &b = hull[hull.size() - 1];
            float ax = b.x - a.x, ay = b.y - a.y;
            float bx = pts[i].x - a.x, by = pts[i].y - a.y;
            float c = cross2(ax, ay, bx, by);
            if (c <= 0.0f) {
                hull.pop_back();
            } else {
                break;
            }
        }
        hull.push_back(pts[i]);
    }

    Polygon result;
    result.pts = hull;
    result.ensureCCW();
    return result;
}

// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
//  smoothPolygonCorners â€” round convex corners to min turn radius
// â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•

Polygon smoothPolygonCorners(const Polygon &poly, float min_turn_radius)
{
    Polygon work = poly;
    work.ensureCCW();

    const int n = static_cast<int>(work.pts.size());
    if (n < 3 || min_turn_radius < EPSILON_VERTEX) return work;

    const float R = min_turn_radius;

    Polygon result;
    result.pts.reserve(n * 3);  // generous â€” arcs add points

    for (int i = 0; i < n; ++i) {
        int prev = (i - 1 + n) % n;
        int next = (i + 1) % n;

        const Point &A = work.pts[prev];
        const Point &B = work.pts[i];
        const Point &C = work.pts[next];

        // Edge vectors into and out of B
        float d1x = B.x - A.x, d1y = B.y - A.y;
        float d2x = C.x - B.x, d2y = C.y - B.y;

        float len1 = sqrtf(d1x*d1x + d1y*d1y);
        float len2 = sqrtf(d2x*d2x + d2y*d2y);

        if (len1 < EPSILON_VERTEX || len2 < EPSILON_VERTEX) {
            result.pts.push_back(B);
            continue;
        }

        // Normalise
        float u1x = d1x / len1, u1y = d1y / len1;  // unit Aâ†’B
        float u2x = d2x / len2, u2y = d2y / len2;  // unit Bâ†’C

        // Cross product: positive = left turn = convex corner in CCW polygon
        float cross = cross2(u1x, u1y, u2x, u2y);

        if (cross <= 0.0f) {
            // Concave corner (right turn) â€” keep sharp
            result.pts.push_back(B);
            continue;
        }

        // Turn angle between incoming and outgoing edges
        float dot = u1x * u2x + u1y * u2y;
        dot = clampf(dot, -1.0f, 1.0f);
        float turn_angle = acosf(dot);  // angle of turn (0..PI)

        if (turn_angle < 0.16f) {
            // Very gentle turn (~9Â°) â€” no smoothing needed
            result.pts.push_back(B);
            continue;
        }

        // Tangent distance: how far from B along each edge the arc starts/ends
        float half_turn = turn_angle * 0.5f;
        float td = R / tanf(half_turn);

        // Skip if tangent distance exceeds half either edge length
        if (td > len1 * 0.5f || td > len2 * 0.5f) {
            result.pts.push_back(B);
            continue;
        }

        // Tangent points
        // P1 = B - td * u1  (on incoming edge, before B)
        // P2 = B + td * u2  (on outgoing edge, after B)
        float p1x = B.x - td * u1x, p1y = B.y - td * u1y;
        float p2x = B.x + td * u2x, p2y = B.y + td * u2y;

        // Arc centre: offset from P1 perpendicular into the polygon interior.
        // For CCW polygon, the inward normal of edge Aâ†’B is (-u1y, u1x)
        // (left of the edge direction). The arc centre sits R metres inward
        // from P1 along this normal.
        float cx = p1x - R * u1y;
        float cy = p1y + R * u1x;

        // Compute arc angles
        float startAngle = atan2f(p1y - cy, p1x - cx);
        float endAngle   = atan2f(p2y - cy, p2x - cx);

        // For CCW traversal at a convex corner, the inscribed arc sweeps CCW
        // (positive). Force positive sweep if wrapAngle gives negative.
        float sweep = wrapAngle(endAngle - startAngle);
        if (sweep < 0.0f) sweep += 2.0f * (float)M_PI;

        auto arcPts = sampleArc(cx, cy, R, startAngle, startAngle + sweep,
                                ARC_INTERVAL_M);
        for (const auto &ap : arcPts) {
            result.pts.push_back(ap);
        }
    }

    // Remove coincident consecutive vertices
    {
        const int rn = static_cast<int>(result.pts.size());
        vector<Point> deduped;
        deduped.reserve(rn);
        for (int i = 0; i < rn; ++i) {
            int j = (i + 1) % rn;
            if (dist(result.pts[i], result.pts[j]) >= EPSILON_VERTEX) {
                deduped.push_back(result.pts[i]);
            }
        }
        result.pts = deduped;
    }

    if (static_cast<int>(result.pts.size()) < 3) return work;  // fallback to original

    result.ensureCCW();
    return result;
}

