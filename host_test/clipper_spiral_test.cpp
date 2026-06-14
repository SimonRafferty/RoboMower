// Validate Clipper2 offsetting for the spiral on the deep-notch garden that made
// the hand-rolled inset spike + invert. Calls InflatePaths directly (the firmware
// wrapper is the same conversion). Build from host_test/:
//   zig c++ -std=c++17 -O1 -I../Robo-Mower-V2/src/clipper2 -o clip.exe \
//     clipper_spiral_test.cpp ../Robo-Mower-V2/src/clipper2/clipper.engine.cpp \
//     ../Robo-Mower-V2/src/clipper2/clipper.offset.cpp \
//     ../Robo-Mower-V2/src/clipper2/clipper.rectclip.cpp
#include "clipper.h"
#include <cstdio>
#include <cmath>
#include <vector>
using namespace Clipper2Lib;

static constexpr double S = 10000.0;       // 0.1 mm grid
static constexpr double ARC = 0.05 * S;    // 5 cm arc tolerance

static double areaM2(const Path64 &p) {
    double a = 0; size_t n = p.size();
    for (size_t i = 0; i < n; i++) { size_t j = (i+1)%n;
        a += (double)p[i].x*p[j].y - (double)p[j].x*p[i].y; }
    return 0.5 * a / (S*S);
}
static double maxEdgeM(const Path64 &p) {
    double m = 0; size_t n = p.size();
    for (size_t i = 0; i < n; i++) { size_t j=(i+1)%n;
        double dx=(double)(p[j].x-p[i].x)/S, dy=(double)(p[j].y-p[i].y)/S;
        double d=sqrt(dx*dx+dy*dy); if(d>m)m=d; }
    return m;
}
static void bboxM(const Path64 &p,double&mnx,double&mny,double&mxx,double&mxy){
    mnx=mny=1e9;mxx=mxy=-1e9;
    for(auto&q:p){double x=(double)q.x/S,y=(double)q.y/S;
        if(x<mnx)mnx=x;if(y<mny)mny=y;if(x>mxx)mxx=x;if(y>mxy)mxy=y;}
}

int main() {
    const double step = 0.36;
    // Deep concave-notch garden (10x10, bottom-right notch), scaled to int grid.
    std::vector<std::pair<double,double>> pf = {
        {0,0},{7,0},{8,2},{9,0},{10,0},{10,10},{0,10}
    };
    Path64 perim;
    for (auto&pr:pf) perim.push_back(Point64((int64_t)llround(pr.first*S),(int64_t)llround(pr.second*S)));

    // nav boundary = perim inset 0.51 (the spiral's outer ring)
    Paths64 navp = InflatePaths(Paths64{perim}, -0.51*S, JoinType::Round, EndType::Polygon, 2.0, ARC);
    if (navp.empty()) { printf("nav empty!\n"); return 1; }
    double bx0,by0,bx1,by1; bboxM(navp[0],bx0,by0,bx1,by1);
    printf("nav: %d pts area %.2f bbox [%.2f,%.2f]-[%.2f,%.2f]\n",
           (int)navp[0].size(), areaM2(navp[0]), bx0,by0,bx1,by1);

    int fails = 0, rings = 0, totalpts = 0;
    // Offset-from-original: ring k = nav inset by k*step (no point compounding).
    // Simplify each ring (RDP, 8 cm) to collapse arc over-tessellation.
    for (int k = 0; ; k++) {
        Paths64 regs = (k==0) ? navp
                              : InflatePaths(navp, -k*step*S, JoinType::Round, EndType::Polygon, 2.0, ARC);
        // keep solid regions only
        Paths64 solid;
        for (auto&r:regs) if (r.size()>=3 && areaM2(r) >= 0.5) solid.push_back(r);
        if (solid.empty()) break;
        for (auto&r0:solid) {
            Path64 r = RamerDouglasPeucker(r0, 0.08*S);
            double a = areaM2(r), me = maxEdgeM(r);
            double mnx,mny,mxx,mxy; bboxM(r,mnx,mny,mxx,mxy);
            bool oob = (mnx<bx0-0.05)||(mny<by0-0.05)||(mxx>bx1+0.05)||(mxy>by1+0.05);
            printf("ring %2d: %3d pts (raw %3d)  area %6.2f  maxEdge %5.2f%s\n",
                   k, (int)r.size(), (int)r0.size(), a, me, oob?"  *** OUT OF BOUNDS ***":"");
            if (oob) fails++;
            totalpts += (int)r.size();
        }
        rings++;
        if (k > 100) break;
    }
    printf("\ntotal rings: %d   total waypoints: %d   FAILURES: %d\n", rings, totalpts, fails);
    return fails;
}
