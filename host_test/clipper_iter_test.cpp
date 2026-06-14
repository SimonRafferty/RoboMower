// Test ITERATIVE depth-first spiral (offset each ring from the previous) using the
// real wrapper, to check vertex counts stay bounded and it reaches the centre.
// Build from host_test/:
//   zig c++ -std=c++17 -O1 -I../Robo-Mower-V2 -I../Robo-Mower-V2/src/clipper2 -o it.exe \
//     clipper_iter_test.cpp ../Robo-Mower-V2/clipper_offset.cpp geometry.cpp \
//     ../Robo-Mower-V2/src/clipper2/clipper.engine.cpp \
//     ../Robo-Mower-V2/src/clipper2/clipper.offset.cpp \
//     ../Robo-Mower-V2/src/clipper2/clipper.rectclip.cpp
#include "clipper_offset.h"
#include <cstdio>
#include <cmath>
#include <vector>

static Point centroid(const Polygon&p){ double cx=0,cy=0,a=0; int n=p.pts.size();
    for(int i=0;i<n;i++){int j=(i+1)%n; double cr=(double)p.pts[i].x*p.pts[j].y-(double)p.pts[j].x*p.pts[i].y;
        a+=cr; cx+=(p.pts[i].x+p.pts[j].x)*cr; cy+=(p.pts[i].y+p.pts[j].y)*cr;}
    a*=0.5; if(fabs(a)<1e-6) return p.pts[0]; return Point((float)(cx/(6*a)),(float)(cy/(6*a))); }

int main(){
    const float step=0.36f, FLOOR=0.04f;
    Polygon perim; perim.pts={{0,0},{7,0},{8,2},{9,0},{10,0},{10,10},{0,10}};
    perim.ensureCCW();
    printf("perimeter area %.1f pts %d\n", perim.area(), (int)perim.pts.size());

    // depth-first stack of rings to continue insetting
    std::vector<Polygon> stack{perim};
    int rings=0, maxpts=0, total=0, areas=0;
    while(!stack.empty() && rings<400){
        Polygon r=stack.back(); stack.pop_back();
        if(r.pts.size()<3 || fabsf(r.area())<FLOOR) { continue; }
        rings++; total+=r.pts.size(); if((int)r.pts.size()>maxpts)maxpts=r.pts.size();
        // inside-perimeter check
        for(auto&pt:r.pts) if(!isInsidePolygon(perim,pt.x,pt.y)){ printf("  OOB ring %d!\n",rings); }
        std::vector<Polygon> kids=offsetPolygonClipper(r, step);
        int valid=0;
        for(auto&k:kids) if(k.pts.size()>=3 && fabsf(k.area())>=FLOOR){ stack.push_back(k); valid++; }
        if(valid==0){ // innermost of this area -> would add centroid here
            areas++;
            Point c=centroid(r);
            printf("  area #%d innermost ring area %.2f, centre (%.2f,%.2f)\n", areas, r.area(), c.x, c.y);
        }
    }
    printf("rings %d  areas %d  total wp %d  max pts/ring %d\n", rings, areas, total, maxpts);
    return 0;
}
