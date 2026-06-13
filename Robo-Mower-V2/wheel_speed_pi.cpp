// ══════════════════════════════════════════════════════════════════════════════
//  wheel_speed_pi.cpp — Per-wheel ESP32-side PI speed controller
//
//  See wheel_speed_pi.h for public API.
// ══════════════════════════════════════════════════════════════════════════════

#include "wheel_speed_pi.h"
#include "vesc_can.h"
#include "mower_config.h"
#include "config.h"
#include "geometry.h"

static float s_integral[2]    = {0.0f, 0.0f};
static float s_last_output[2] = {0.0f, 0.0f};

void wheel_speed_pi_init() {
    wheel_speed_pi_reset();
}

void wheel_speed_pi_reset() {
    s_integral[0] = s_integral[1] = 0.0f;
    s_last_output[0] = s_last_output[1] = 0.0f;
}

int32_t wheel_speed_pi_compute(uint8_t vesc_id, float desired_ms, float dt) {
    const MowerConfig &mc = mower_config_get();
    int idx = (vesc_id == VESC_ID_LEFT) ? 0 : 1;

    // Actual velocity from VESC CAN STATUS frames (~50 Hz)
    float actual_ms = vesc_erpm_to_velocity(vesc_get_status(vesc_id).erpm);
    float error_ms  = desired_ms - actual_ms;

    // Integrate with anti-windup: clamp so integral term alone ≤ max_current_a
    float max_integral = (mc.wheel_pi_ki > 0.001f)
                         ? (mc.max_current_a / mc.wheel_pi_ki)
                         : 0.0f;
    s_integral[idx] = clampf(s_integral[idx] + error_ms * dt,
                             -max_integral, max_integral);

    float output_A = mc.wheel_pi_kp * error_ms + mc.wheel_pi_ki * s_integral[idx];
    output_A = clampf(output_A, -mc.max_current_a, mc.max_current_a);

    s_last_output[idx] = output_A;
    return (int32_t)(output_A * 1000.0f);
}

void wheel_speed_pi_get_last_output(float *left_A, float *right_A) {
    if (left_A)  *left_A  = s_last_output[0];
    if (right_A) *right_A = s_last_output[1];
}
