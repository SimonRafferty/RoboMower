// ══════════════════════════════════════════════════════════════════════════════
//  heading_controller.cpp — Gyro-stabilised heading hold with AGC
//
//  See heading_controller.h for usage.
// ══════════════════════════════════════════════════════════════════════════════

#include "heading_controller.h"
#include "config.h"
#include <math.h>

// ── AGC state ────────────────────────────────────────────────────────────────

static float s_gain           = 1.0f;   // adaptive gain multiplier [AGC_GAIN_MIN, AGC_GAIN_MAX]
static float s_prev_error     = 0.0f;   // previous heading error for zero-crossing detection
static int   s_zero_crossings = 0;      // count of error sign changes in current window
static int   s_tick_count     = 0;      // ticks elapsed in current AGC window
static float s_error_sum      = 0.0f;   // accumulated |heading_error| for mean computation
static bool  s_prev_valid     = false;  // true once s_prev_error has been set

// ─────────────────────────────────────────────────────────────────────────────

void heading_controller_reset() {
    s_gain           = 1.0f;
    s_prev_error     = 0.0f;
    s_zero_crossings = 0;
    s_tick_count     = 0;
    s_error_sum      = 0.0f;
    s_prev_valid     = false;
}

float heading_controller_compute(float heading_error, float yaw_rate_error,
                                  float kp, float kd, float dt) {
    (void)dt;  // dt not needed for proportional/derivative — gains are already in correct units

    // ── PD correction ────────────────────────────────────────────────────────
    float correction = s_gain * (kp * heading_error + kd * yaw_rate_error);

    // ── AGC bookkeeping ──────────────────────────────────────────────────────
    // Track zero-crossings of heading_error (sign changes indicate oscillation)
    if (s_prev_valid) {
        if ((heading_error > 0.0f && s_prev_error < 0.0f) ||
            (heading_error < 0.0f && s_prev_error > 0.0f)) {
            s_zero_crossings++;
        }
    }
    s_prev_error = heading_error;
    s_prev_valid = true;

    // Accumulate |error| for mean computation
    s_error_sum += fabsf(heading_error);
    s_tick_count++;

    // ── AGC evaluation at end of window ──────────────────────────────────────
    if (s_tick_count >= AGC_WINDOW_TICKS) {
        float osc_score  = (float)s_zero_crossings / (float)AGC_WINDOW_TICKS;
        float mean_error = s_error_sum / (float)AGC_WINDOW_TICKS;

        if (osc_score > AGC_OSCILLATION_THRESHOLD) {
            // Oscillating — reduce gain
            s_gain *= AGC_DECAY_RATE;
        } else if (mean_error > AGC_ERROR_THRESHOLD) {
            // Persistent error — increase gain
            s_gain *= AGC_GROW_RATE;
        }

        // Clamp gain
        if (s_gain < AGC_GAIN_MIN) s_gain = AGC_GAIN_MIN;
        if (s_gain > AGC_GAIN_MAX) s_gain = AGC_GAIN_MAX;

        // Reset window
        s_zero_crossings = 0;
        s_tick_count     = 0;
        s_error_sum      = 0.0f;
    }

    return correction;
}

float heading_controller_get_gain() {
    return s_gain;
}
