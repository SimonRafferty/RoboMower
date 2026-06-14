// ══════════════════════════════════════════════════════════════════════════════
//  clipper_offset.cpp — see clipper_offset.h
// ══════════════════════════════════════════════════════════════════════════════

#include "clipper_offset.h"
#include "src/clipper2/clipper.h"
#include <cmath>

// Import only the specific Clipper2 names used here — a blanket `using namespace`
// pulls in Clipper2Lib::Point, which collides with our geometry Point.
using Clipper2Lib::Path64;
using Clipper2Lib::Paths64;
using Clipper2Lib::Point64;
using Clipper2Lib::InflatePaths;
using Clipper2Lib::JoinType;
using Clipper2Lib::EndType;
using Clipper2Lib::RamerDouglasPeucker;

// Clipper works on an integer grid. Scale metres → 0.1 mm units (×10000). A
// garden a few hundred metres across stays well within int64. Arc tolerance of
// 5 cm keeps round joins from over-tessellating (plenty fine for a mower path).
static constexpr double CLIP_SCALE = 10000.0;
static constexpr double CLIP_ARC_TOL_M = 0.05;   // round-join arc accuracy
// Ramer–Douglas–Peucker simplification of each ring: collapses round-join
// over-tessellation and near-collinear vertices to sparse turn-point nodes
// (the node follower drives straight between them). Keeps point counts well
// under the follower's waypoint buffer.
static constexpr double CLIP_SIMPLIFY_M = 0.08;

std::vector<Polygon> offsetPolygonClipper(const Polygon &poly, float inset_m) {
    std::vector<Polygon> out;
    if (poly.pts.size() < 3 || inset_m <= 0.0f) return out;

    // ── Polygon → Clipper Path64 (scaled) ────────────────────────────────────
    Path64 in;
    in.reserve(poly.pts.size());
    for (const auto &p : poly.pts) {
        in.push_back(Point64((int64_t)llround((double)p.x * CLIP_SCALE),
                             (int64_t)llround((double)p.y * CLIP_SCALE)));
    }
    Paths64 subject;
    subject.push_back(in);

    // ── Inward offset (negative delta) with round joins ──────────────────────
    // EndType::Polygon = closed region; JoinType::Round = the standard inward
    // offset rule (bounded arc at reflex corners, no spike).
    Paths64 sol = InflatePaths(subject, -(double)inset_m * CLIP_SCALE,
                               JoinType::Round, EndType::Polygon,
                               2.0 /*miter limit, unused for round*/,
                               CLIP_ARC_TOL_M * CLIP_SCALE);

    // ── Clipper Paths64 → Polygons ───────────────────────────────────────────
    // Keep only solid (positive-area) contours; drop holes (negative) and
    // degenerate slivers. Caller applies the real minimum-area threshold.
    for (const auto &path0 : sol) {
        if (path0.size() < 3) continue;
        Path64 path = RamerDouglasPeucker(path0, CLIP_SIMPLIFY_M * CLIP_SCALE);
        if (path.size() < 3) continue;
        Polygon pg;
        pg.pts.reserve(path.size());
        for (const auto &pt : path) {
            pg.pts.push_back(Point((float)((double)pt.x / CLIP_SCALE),
                                   (float)((double)pt.y / CLIP_SCALE)));
        }
        float a = pg.area();           // CCW (solid) = positive
        if (a > 0.01f) {               // solid contour, not a hole or sliver
            out.push_back(pg);
        }
    }
    return out;
}

Polygon insetPolygonClipper(const Polygon &poly, float inset_m) {
    std::vector<Polygon> regions = offsetPolygonClipper(poly, inset_m);
    Polygon best;
    float best_area = 0.0f;
    for (auto &r : regions) {
        float a = fabsf(r.area());
        if (a > best_area) { best_area = a; best = r; }
    }
    return best;   // empty Polygon if regions was empty (inset collapsed)
}
