// Host-test stand-in for config.h — just the constants geometry.cpp needs,
// with debug macros routed to stdout. Values MUST match Robo-Mower-V2/config.h.
#pragma once
#include <cstdio>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MIN_ZONE_AREA_M2  0.50f   // matches Robo-Mower-V2/config.h:367

#define DBG_PRINTF(...)   printf(__VA_ARGS__)
#define DBG_PRINTLN(s)    printf("%s\n", s)
#define DBG_PRINT(s)      printf("%s", s)
