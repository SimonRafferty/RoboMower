// ══════════════════════════════════════════════════════════════════════════════
//  wheel_duty_ramp.cpp — Per-wheel duty-cycle ramp for AUTO_MOWING
//
//  See wheel_duty_ramp.h for API.
// ══════════════════════════════════════════════════════════════════════════════

#include "wheel_duty_ramp.h"
#include "vesc_can.h"
#include "mower_config.h"
#include "config.h"
#include "geometry.h"   // for clampf()
#include "sys_log.h"

static float s_duty[2] = {0.0f, 0.0f};  // [LEFT=0, RIGHT=1]

void wheel_duty_ramp_reset() {
    s_duty[0] = 0.0f;
    s_duty[1] = 0.0f;
}

float wheel_duty_ramp_compute(uint8_t vesc_id, float desired_ms) {
    const MowerConfig mc = mower_config_get();
    int idx = (vesc_id == VESC_ID_LEFT) ? 0 : 1;

    // ── Feedback validity gate ────────────────────────────────────────────────
    // A VESC that has never sent a STATUS frame (last_update_ms == 0) or has
    // gone quiet reads eRPM = 0 forever. Integrating duty against that frozen
    // zero is the max-speed runaway failure: error never closes, duty winds up
    // to the clamp. With no feedback, fall back to bounded OPEN-LOOP duty
    // proportional to the demanded speed — slow but safe in both directions.
    VescStatus st = vesc_get_status(vesc_id);
    bool feedback_ok = (st.last_update_ms != 0) &&
                       (millis() - st.last_update_ms) <= DUTY_FEEDBACK_STALE_MS;

    if (!feedback_ok) {
        static bool s_fallback_logged = false;
        if (!s_fallback_logged) {
            s_fallback_logged = true;
            sys_log_push("DUTY: no drive eRPM feedback - OPEN-LOOP fallback (slow)");
        }
        float max_ms = (mc.max_wheel_speed_ms > 0.1f) ? mc.max_wheel_speed_ms : 1.0f;
        float duty   = (desired_ms / max_ms) * mc.manual_max_duty;
        duty = clampf(duty, -mc.manual_max_duty, mc.manual_max_duty);
        s_duty[idx] = duty;
        vesc_set_duty(vesc_id, duty);
        return duty;
    }

    // Use the SCALED velocity so the closed loop runs on the same calibrated
    // odometry the EKF and follower use (otherwise it would settle at
    // desired/scale ground speed). See odo_calib.h.
    float actual_ms = vesc_erpm_to_velocity_scaled(st.erpm);
    float error_ms  = desired_ms - actual_ms;

    // Step duty toward target — one fixed step per tick, no proportional jump
    if (error_ms > 0.01f) {
        s_duty[idx] += DUTY_RAMP_STEP;
    } else if (error_ms < -0.01f) {
        s_duty[idx] -= DUTY_RAMP_STEP;
    }

    // Clamp duty to [-max_duty, +max_duty]
    s_duty[idx] = clampf(s_duty[idx], -mc.manual_max_duty, mc.manual_max_duty);

    vesc_set_duty(vesc_id, s_duty[idx]);
    return s_duty[idx];
}

void wheel_duty_ramp_get_last(float *left_duty, float *right_duty) {
    if (left_duty)  *left_duty  = s_duty[0];
    if (right_duty) *right_duty = s_duty[1];
}
