// Host runner for the firmware geometry unit tests, plus extra scenarios
// probing the 2026-06-09/10 insetPolygon concave-corner rewrite with
// realistic perimeter shapes and the chained insets the firmware performs
// (perimeter → nav boundary → working area → headland passes).
#include "geometry.h"
#include "geometry_test.h"
#include "host_config.h"
#include <cstdio>
#include <cmath>

static int g_extra_fails = 0;

static void check(bool ok, const char *what) {
    printf("[EXTRA] %-58s %s\n", what, ok ? "PASS" : "FAIL");
    if (!ok) g_extra_fails++;
}

static void report_poly(const char *name, const Polygon &p) {
    printf("[EXTRA]   %s: %d pts, area %.3f m2, selfX=%d\n",
           name, (int)p.pts.size(), p.area(),
           p.pts.size() >= 3 ? (int)isSelfIntersecting(p) : 0);
}

// Garden-like concave perimeter: 20 x 12 m rectangle with a 4 m-deep concave
// notch (gateway/flowerbed) in the top edge and a chamfered corner.
static Polygon make_garden() {
    Polygon g;
    g.pts = {
        { 0.0f,  0.0f}, {20.0f,  0.0f}, {20.0f, 10.0f}, {18.0f, 12.0f},
        {12.0f, 12.0f}, {12.0f,  8.0f},   // notch right wall (concave corners)
        { 8.0f,  8.0f}, { 8.0f, 12.0f},   // notch left wall
        { 0.0f, 12.0f}
    };
    return g;
}

int main() {
    printf("===== firmware geometry_test_runAll() =====\n");
    int fw_fails = geometry_test_runAll();

    printf("\n===== extra: concave garden, firmware-style chained insets =====\n");
    Polygon garden = make_garden();
    garden.ensureCCW();
    report_poly("perimeter", garden);
    check(garden.area() > 200.0f, "garden polygon valid (area > 200 m2)");

    // perimeter_recompute(): nav = inset(perim, ~0.45), work = inset(nav, ~0.8)
    Polygon nav = insetPolygon(garden, 0.45f);
    report_poly("nav boundary (inset 0.45)", nav);
    check(!nav.pts.empty(), "nav boundary non-empty");
    check(nav.pts.empty() || nav.area() > 0.0f, "nav boundary CCW (positive area)");
    check(nav.pts.empty() || !isSelfIntersecting(nav), "nav boundary not self-intersecting");

    Polygon work = insetPolygon(nav, 0.8f);
    report_poly("working area (inset 0.8)", work);
    check(!work.pts.empty(), "working area non-empty");
    check(work.pts.empty() || work.area() > 0.0f, "working area CCW (positive area)");
    check(work.pts.empty() || !isSelfIntersecting(work), "working area not self-intersecting");

    // Headland passes: planner insets the nav boundary repeatedly
    if (!nav.pts.empty()) {
        Polygon pass = nav;
        for (int i = 1; i <= 3; i++) {
            pass = insetPolygon(pass, 0.3f);
            char what[64];
            snprintf(what, sizeof(what), "headland pass %d valid or cleanly empty", i);
            check(pass.pts.empty() || (pass.area() > 0.0f && !isSelfIntersecting(pass)), what);
            if (pass.pts.empty()) break;
        }
    }

    // insetPolygonMulti on the concave shape (used for region splitting)
    {
        auto regions = insetPolygonMulti(garden, 1.5f);
        printf("[EXTRA]   insetPolygonMulti(1.5): %d region(s)\n", (int)regions.size());
        check(!regions.empty(), "insetPolygonMulti returns >= 1 region");
        for (auto &r : regions) {
            check(r.area() > 0.0f && !isSelfIntersecting(r),
                  "multi-inset region CCW + not self-intersecting");
        }
    }

    // splitSelfIntersecting must still accept a plain CW-wound polygon when
    // callers forget ensureCCW (NVS-loaded blobs from older firmware may be CW)
    {
        Polygon cw = make_garden();
        cw.ensureCCW();
        // reverse to CW
        Polygon rev; rev.pts.assign(cw.pts.rbegin(), cw.pts.rend());
        auto out = splitSelfIntersecting(rev);
        printf("[EXTRA]   splitSelfIntersecting(CW input): %d sub-poly(s)\n", (int)out.size());
        check(!out.empty(), "CW-wound (but valid) polygon survives splitSelfIntersecting");
    }

    printf("\n===== summary: firmware fails=%d extra fails=%d =====\n",
           fw_fails, g_extra_fails);
    return fw_fails + g_extra_fails;
}
