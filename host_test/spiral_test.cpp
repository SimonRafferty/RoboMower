// Standalone host test for the concentric-spiral coverage planner (2026-06-13).
// Replicates coverage_planner_plan()'s spiral loop EXACTLY against the real
// geometry.cpp insetPolygonMulti(), to prove that repeatedly insetting a SPARSE
// perimeter produces sane, in-bounds, terminating concentric rings.
//
// Build (from host_test/):
//   zig c++ -std=c++17 -O1 -o spiraltest.exe spiral_test.cpp geometry.cpp
//   ./spiraltest.exe          # exit code = number of failures
#include "geometry.h"
#include "host_config.h"
#include <cstdio>
#include <cmath>
#include <vector>

static int g_fails = 0;
static void check(bool ok, const char *what) {
    printf("%-62s %s\n", what, ok ? "PASS" : "FAIL");
    if (!ok) g_fails++;
}

// ── Mirror of coverage_planner.cpp constants/logic ──────────────────────────────
// MIN_ZONE_AREA_M2 comes from host_config.h (matches firmware config.h).
static constexpr int   MAX_PLAN_WP      = 960;
static constexpr int   MAX_RINGS        = 400;

struct WP { float x, y; bool mowing; };

struct SpiralResult {
    std::vector<WP> wps;
    int  rings;
    bool capped;
};

// EXACT replica of the planner's spiral generation (addRingWaypoints + loop).
static SpiralResult spiral(const Polygon &navBoundary, float step) {
    SpiralResult R; R.rings = 0; R.capped = false;

    std::vector<Polygon> stack;
    Polygon outer = navBoundary; outer.ensureCCW();
    stack.push_back(outer);

    Point seed = outer.pts[0];

    while (!stack.empty()) {
        if ((int)R.wps.size() >= MAX_PLAN_WP || R.rings >= MAX_RINGS) { R.capped = true; break; }

        Polygon ring = stack.back(); stack.pop_back();
        if (ring.pts.size() < 3 || fabsf(ring.area()) < MIN_ZONE_AREA_M2) continue;

        if (!R.wps.empty()) { WP brk = R.wps.back(); brk.mowing = false; R.wps.push_back(brk); }

        // addRingWaypoints: start at vertex nearest seed, emit n+1 (closed loop).
        int n = (int)ring.pts.size();
        int start = 0; float best = 1e30f;
        for (int i = 0; i < n; i++) {
            float dx = ring.pts[i].x - seed.x, dy = ring.pts[i].y - seed.y;
            float d = dx*dx + dy*dy; if (d < best) { best = d; start = i; }
        }
        for (int k = 0; k <= n; k++) {
            int ci = (start + k) % n;
            R.wps.push_back({ ring.pts[ci].x, ring.pts[ci].y, true });
        }
        seed = ring.pts[start];
        R.rings++;

        std::vector<Polygon> children = insetPolygonMulti(ring, step);
        for (auto &c : children)
            if (c.pts.size() >= 3 && fabsf(c.area()) >= MIN_ZONE_AREA_M2)
                stack.push_back(c);
    }
    return R;
}

// Run the full pipeline for one garden: perimeter -> nav(inset 0.35) -> spiral.
static void run_case(const char *name, Polygon perim, float step) {
    printf("\n===== %s =====\n", name);
    perim.ensureCCW();
    printf("  perimeter: %d pts, area %.2f m2\n", (int)perim.pts.size(), perim.area());

    Polygon nav = insetPolygon(perim, 0.35f);   // NAV_EXCLUSION_INSET_M ~ 0.35
    check(!nav.pts.empty() && nav.area() > 0.0f, "nav boundary valid");
    if (nav.pts.empty()) return;
    printf("  nav boundary: %d pts, area %.2f m2\n", (int)nav.pts.size(), nav.area());

    SpiralResult R = spiral(nav, step);
    printf("  spiral: %d rings, %d waypoints, capped=%d\n",
           R.rings, (int)R.wps.size(), (int)R.capped);

    check(R.rings > 0,            "produced >= 1 ring");
    check(!R.capped,              "terminated naturally (not capped)");
    check((int)R.wps.size() < MAX_PLAN_WP, "waypoint count under buffer cap");

    // Every emitted waypoint must lie inside (or on) the nav boundary.
    int outside = 0;
    for (auto &w : R.wps)
        if (!isInsidePolygon(nav, w.x, w.y)) outside++;
    check(outside == 0, "all waypoints inside nav boundary");
    if (outside) printf("    -> %d waypoints OUTSIDE nav boundary!\n", outside);

    // No giant jumps between consecutive same-segment waypoints: the largest
    // gap should be comparable to a perimeter edge, not a chord across the plot.
    // (A break waypoint duplicates the previous point, so gaps are 0 there.)
    float maxgap = 0.0f;
    for (size_t i = 1; i < R.wps.size(); i++) {
        float dx = R.wps[i].x - R.wps[i-1].x, dy = R.wps[i].y - R.wps[i-1].y;
        float d = sqrtf(dx*dx + dy*dy);
        if (d > maxgap) maxgap = d;
    }
    printf("  largest consecutive gap: %.2f m\n", maxgap);
}

int main() {
    const float step = 0.38f - 0.02f;   // CUT_WIDTH_M - STRIP_OVERLAP_M = 0.36

    // 1) Sparse irregular convex quad (≈ the shape in PathError.jpg), 4 nodes.
    {
        Polygon p; p.pts = { {2.0f,1.0f}, {18.0f,3.0f}, {16.0f,13.0f}, {1.0f,11.0f} };
        run_case("sparse irregular quad (4 nodes)", p, step);
    }

    // 2) Sparse axis-aligned rectangle, 4 nodes, 20 x 12 m.
    {
        Polygon p; p.pts = { {0.0f,0.0f}, {20.0f,0.0f}, {20.0f,12.0f}, {0.0f,12.0f} };
        run_case("sparse rectangle 20x12 (4 nodes)", p, step);
    }

    // 3) Concave garden with a notch (tests lobe-splitting as it shrinks).
    {
        Polygon p; p.pts = {
            { 0.0f, 0.0f}, {20.0f, 0.0f}, {20.0f,10.0f}, {18.0f,12.0f},
            {12.0f,12.0f}, {12.0f, 8.0f}, { 8.0f, 8.0f}, { 8.0f,12.0f}, {0.0f,12.0f}
        };
        run_case("concave garden w/ notch (9 nodes)", p, step);
    }

    // 4) Small plot — few rings before collapse.
    {
        Polygon p; p.pts = { {0.0f,0.0f}, {4.0f,0.0f}, {4.0f,3.0f}, {0.0f,3.0f} };
        run_case("small plot 4x3 (4 nodes)", p, step);
    }

    printf("\n===== spiral test failures: %d =====\n", g_fails);
    return g_fails;
}
