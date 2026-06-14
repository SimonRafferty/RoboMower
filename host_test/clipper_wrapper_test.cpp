// End-to-end test of the REAL firmware wrapper offsetPolygonClipper() driving the
// from-original spiral on the deep-notch garden. Build from host_test/:
//   zig c++ -std=c++17 -O1 -I../Robo-Mower-V2 -I../Robo-Mower-V2/src/clipper2 -o wrap.exe \
//     clipper_wrapper_test.cpp ../Robo-Mower-V2/clipper_offset.cpp geometry.cpp \
//     ../Robo-Mower-V2/src/clipper2/clipper.engine.cpp \
//     ../Robo-Mower-V2/src/clipper2/clipper.offset.cpp \
//     ../Robo-Mower-V2/src/clipper2/clipper.rectclip.cpp
#include "clipper_offset.h"   // firmware wrapper + geometry.h
#include <cstdio>
#include <cmath>

static float maxEdge(const Polygon &p){ float m=0; int n=p.pts.size();
    for(int i=0;i<n;i++){int j=(i+1)%n;float dx=p.pts[j].x-p.pts[i].x,dy=p.pts[j].y-p.pts[i].y;
        float d=sqrtf(dx*dx+dy*dy); if(d>m)m=d;} return m; }

int main() {
    const float step = 0.36f;
    Polygon perim; perim.pts = {
        {0,0},{7,0},{8,2},{9,0},{10,0},{10,10},{0,10}
    };
    perim.ensureCCW();
    printf("perimeter: %d pts area %.2f\n", (int)perim.pts.size(), perim.area());

    // New model: ring 0 = perimeter itself; ring k>0 = perimeter inset k*step.
    int fails=0, rings=0, totalpts=0; float prevA=1e9f;
    for (int k=0;;k++) {
        std::vector<Polygon> regs;
        if (k==0) regs.push_back(perim);
        else      regs = offsetPolygonClipper(perim, k*step);
        std::vector<Polygon> valid;
        for (auto&r:regs) if (r.pts.size()>=3 && fabsf(r.area())>=0.04f) valid.push_back(r);
        if (valid.empty()) break;
        for (auto&r:valid) {
            float a=r.area(), me=maxEdge(r);
            printf("ring %2d: %2d pts  area %6.2f  maxEdge %5.2f\n", k,(int)r.pts.size(),a,me);
            totalpts += r.pts.size();
            if (a > prevA + 0.01f && k>0) { printf("  *** AREA GREW ***\n"); fails++; }
            // every vertex must be inside the perimeter (steering-centre limit)
            for (auto&pt:r.pts) if (!isInsidePolygon(perim, pt.x, pt.y)) { fails++; break; }
            prevA=a;
        }
        rings++; if (k>200) break;
    }
    printf("\nrings %d  total waypoints %d  FAILURES %d\n", rings, totalpts, fails);
    return fails;
}
