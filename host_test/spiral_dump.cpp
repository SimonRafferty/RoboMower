// Spiral rings via SINGLE offset-from-original (not inset-of-previous), with the
// round-join fix in geometry.cpp. Summary per ring; flags out-of-bounds / spikes.
//   zig c++ -std=c++17 -O1 -o spiraldump.exe spiral_dump.cpp geometry.cpp
#include "geometry.h"
#include "host_config.h"
#include <cstdio>
#include <cmath>
#include <vector>

static float maxEdge(const Polygon &p) {
    float m = 0; int n = p.pts.size();
    for (int i = 0; i < n; i++) {
        int j = (i+1)%n;
        float dx = p.pts[j].x-p.pts[i].x, dy = p.pts[j].y-p.pts[i].y;
        float d = sqrtf(dx*dx+dy*dy); if (d > m) m = d;
    }
    return m;
}

static void bbox(const Polygon &p, float &minx,float &miny,float &maxx,float &maxy){
    minx=miny=1e9; maxx=maxy=-1e9;
    for (auto &q:p.pts){ if(q.x<minx)minx=q.x; if(q.y<miny)miny=q.y;
                         if(q.x>maxx)maxx=q.x; if(q.y>maxy)maxy=q.y; }
}

int main() {
    const float step = 0.36f;

    Polygon perim; perim.pts = {
        {0.0f,0.0f}, {7.0f,0.0f}, {8.0f,2.0f}, {9.0f,0.0f}, {10.0f,0.0f},
        {10.0f,10.0f}, {0.0f,10.0f}
    };
    perim.ensureCCW();
    Polygon nav = insetPolygon(perim, 0.51f);
    float nminx,nminy,nmaxx,nmaxy; bbox(nav,nminx,nminy,nmaxx,nmaxy);
    printf("nav: %d pts, area %.2f, bbox [%.1f,%.1f]-[%.1f,%.1f]\n",
           (int)nav.pts.size(), nav.area(), nminx,nminy,nmaxx,nmaxy);

    // Ring k = single offset of nav by k*step (k=0 -> nav itself).
    for (int k = 0; k < 30; k++) {
        std::vector<Polygon> regs = (k==0)
            ? std::vector<Polygon>{nav}
            : insetPolygonMulti(nav, k*step);
        if (regs.empty()) { printf("ring %2d: (collapsed) -> stop\n", k); break; }
        for (size_t r=0;r<regs.size();r++){
            Polygon &p = regs[r];
            float a=fabsf(p.area());
            if (p.pts.size()<3 || a<MIN_ZONE_AREA_M2) { printf("ring %2d.%zu: tiny (a=%.2f) skip\n",k,r,a); continue; }
            float me=maxEdge(p);
            float mnx,mny,mxx,mxy; bbox(p,mnx,mny,mxx,mxy);
            bool oob = (mnx<nminx-0.1f)||(mny<nminy-0.1f)||(mxx>nmaxx+0.1f)||(mxy>nmaxy+0.1f);
            printf("ring %2d.%zu: %3d pts  area %6.2f  maxEdge %6.2f  selfX=%d%s\n",
                   k, r, (int)p.pts.size(), a, me, (int)isSelfIntersecting(p),
                   oob ? "   *** OUT OF BOUNDS ***" : "");
        }
    }
    return 0;
}
