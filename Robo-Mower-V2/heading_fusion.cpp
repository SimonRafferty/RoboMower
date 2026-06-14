#include "heading_fusion.h"
#include "geometry.h"   // wrapAngle()
#include <cmath>

bool heading_offset_segment_qualifies(float turn_accum_rad, float dist_m,
                                      float sigma_m, float straight_max_turn_rad,
                                      float min_dist_m, float dist_sigma_k) {
    if (turn_accum_rad > straight_max_turn_rad) return false;   // not straight
    float thresh = fmaxf(min_dist_m, dist_sigma_k * sigma_m);   // measurable
    return dist_m > thresh;
}

float heading_offset_ema(float offset, float z_hdg, float bno_hdg, float gain) {
    float inst = wrapAngle(z_hdg - bno_hdg);            // instantaneous offset
    return wrapAngle(offset + gain * wrapAngle(inst - offset));  // wrap-safe EMA
}

float heading_compose(float bno_hdg, float offset) {
    return wrapAngle(bno_hdg + offset);
}
