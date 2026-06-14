// Reproduce the "missed arm" bug: a garden that pinches into 2 regions under the
// nav inset. Shows (a) largest-region-only nav drops a region, (b) offsetting the
// spiral FROM THE PERIMETER (all regions) covers both. Build from host_test/:
//   zig c++ -std=c++17 -O1 -I../Robo-Mower-V2 -I../Robo-Mower-V2/src/clipper2 -o mr.exe \
//     clipper_multiregion_test.cpp ../Robo-Mower-V2/clipper_offset.cpp geometry.cpp \
//     ../Robo-Mower-V2/src/clipper2/clipper.engine.cpp \
//     ../Robo-Mower-V2/src/clipper2/clipper.offset.cpp \
//     ../Robo-Mower-V2/src/clipper2/clipper.rectclip.cpp
#include "clipper_offset.h"
#include <cstdio>
#include <cmath>

int main() {
    const float nav_inset = 0.51f, step = 0.36f;
    // Dumbbell: left block (0..4) + right block (6..10), each 0..10 tall, joined
    // by a 0.8 m-tall bridge (y 4.6..5.4). Inset 0.51 pinches the bridge → 2 regions.
    Polygon perim; perim.pts = {
        {0,0},{4,0},{4,4.6f},{6,4.6f},{6,0},{10,0},{10,10},{6,10},{6,5.4f},{4,5.4f},{4,10},{0,10}
    };
    perim.ensureCCW();
    printf("perimeter area %.1f\n", perim.area());

    // (a) largest-region-only (what nav boundary currently does)
    Polygon navLargest = insetPolygonClipper(perim, nav_inset);
    printf("insetPolygonClipper (largest only): area %.2f  <-- DROPS the other block\n",
           navLargest.area());

    // (b) all regions from the perimeter (the fix)
    auto navAll = offsetPolygonClipper(perim, nav_inset);
    printf("offsetPolygonClipper(perimeter, nav_inset): %d region(s)\n", (int)navAll.size());

    // From-perimeter spiral, all regions per ring:
    int rings=0, totalRegionsEmitted=0, totalpts=0;
    for (int k=0;;k++) {
        auto regs = offsetPolygonClipper(perim, nav_inset + k*step);
        std::vector<Polygon> valid;
        for (auto&r:regs) if (r.pts.size()>=3 && fabsf(r.area())>=0.5f) valid.push_back(r);
        if (valid.empty()) break;
        printf("ring %2d: %d region(s)  areas:", k, (int)valid.size());
        for (auto&r:valid){ printf(" %.1f", r.area()); totalpts+=r.pts.size(); }
        printf("\n");
        totalRegionsEmitted += valid.size();
        rings++; if (k>50) break;
    }
    printf("\nrings %d  regions emitted %d  waypoints %d\n", rings, totalRegionsEmitted, totalpts);
    printf("PASS if both blocks covered (>=2 regions on outer rings)\n");
    return (navAll.size() >= 2) ? 0 : 1;
}
