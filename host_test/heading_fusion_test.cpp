// Host unit tests for heading_fusion.cpp. Build with Zig clang (see run cmd).
// Exit code = number of failures.
#include "heading_fusion.h"
#include "geometry.h"   // wrapAngle()
#include <cstdio>
#include <cmath>

static int g_fail = 0;
static void check(bool cond, const char* name) {
    if (!cond) { printf("FAIL: %s\n", name); g_fail++; }
    else       { printf("ok:   %s\n", name); }
}
static bool near_rad(float a, float b, float tol) {
    return fabsf(wrapAngle(a - b)) < tol;
}

int main() {
    const float MAXTURN = 0.20f, MINDIST = 0.30f, K = 20.0f;

    // Gate: a turn beyond the straight limit disqualifies regardless of distance.
    check(!heading_offset_segment_qualifies(0.30f, 5.0f, 0.01f, MAXTURN, MINDIST, K),
          "turned segment disqualified");
    // Gate: straight + far enough (RTK fixed, sigma 1 cm → thresh 0.30 m).
    check(heading_offset_segment_qualifies(0.05f, 0.50f, 0.01f, MAXTURN, MINDIST, K),
          "straight+far qualifies");
    // Gate: straight but too short.
    check(!heading_offset_segment_qualifies(0.05f, 0.20f, 0.01f, MAXTURN, MINDIST, K),
          "straight+short disqualified");
    // Gate: float fix (sigma 0.15 m → thresh = 3.0 m), 1 m is too short.
    check(!heading_offset_segment_qualifies(0.05f, 1.0f, 0.15f, MAXTURN, MINDIST, K),
          "float fix needs 3 m");

    // compose wraps.
    check(near_rad(heading_compose(3.0f, 0.5f), wrapAngle(3.5f), 1e-4f),
          "compose wraps");

    // EMA converges offset so compose(bno, offset) → z_hdg.
    // True heading z = 1.00 rad; BNO reads 0.70 rad → offset should approach 0.30.
    float off = 0.0f;
    for (int i = 0; i < 200; i++) off = heading_offset_ema(off, 1.00f, 0.70f, 0.1f);
    check(near_rad(off, 0.30f, 1e-3f), "EMA converges to offset");
    check(near_rad(heading_compose(0.70f, off), 1.00f, 1e-3f),
          "composed heading matches truth");

    // EMA wrap-safe near ±π: z = +3.10, bno = -3.10 → small positive offset.
    float off2 = 0.0f;
    for (int i = 0; i < 200; i++) off2 = heading_offset_ema(off2, 3.10f, -3.10f, 0.1f);
    check(near_rad(heading_compose(-3.10f, off2), 3.10f, 1e-3f),
          "EMA wrap-safe across ±pi");

    printf("\n%d failure(s)\n", g_fail);
    return g_fail;
}
