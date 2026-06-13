#pragma once
#include <stdint.h>

// ══════════════════════════════════════════════════════════════════════════════
//  wheel_speed_pi.h — Per-wheel ESP32-side PI speed controller
//
//  Receives a desired wheel velocity (m/s) and reads actual velocity from
//  live VESC ERPM telemetry, then outputs a current command (mA) to pass to
//  vesc_set_current().
//
//  Used by both drive_manual() (manual RC/BLE drive) and
//  pure_pursuit_to_vesc_current() (autonomous mowing).
//
//  Gains (wheel_pi_kp, wheel_pi_ki) are stored in MowerConfig so they can
//  be tuned via WebUI without recompiling.  Defaults in config.h.
//
//  Thread safety: call only from the state machine task (Core 1).
// ══════════════════════════════════════════════════════════════════════════════

/**
 * Initialise the PI controller.  Must be called once from setup() after
 * mower_config_init() and vesc_can_init().
 */
void wheel_speed_pi_init();

/**
 * Reset both wheel integrators to zero.
 *
 * Call when the mower stops (throttle → 0), on any mode transition, or
 * whenever the wheel speed target changes discontinuously.  Prevents
 * integrator wind-up carrying over between drive sessions.
 */
void wheel_speed_pi_reset();

/**
 * Compute the current command for one wheel.
 *
 * Reads actual wheel velocity from the latest VESC STATUS telemetry,
 * computes velocity error, integrates with anti-windup, and returns the
 * motor current command in milliamps.
 *
 * @param vesc_id     VESC_ID_LEFT or VESC_ID_RIGHT
 * @param desired_ms  Target wheel surface velocity in m/s (signed)
 * @param dt          Time step in seconds since last call
 * @return            Current command in milliamps (positive = forward)
 */
int32_t wheel_speed_pi_compute(uint8_t vesc_id, float desired_ms, float dt);

/**
 * Return the last output current for both wheels (for telemetry / BLE).
 *
 * @param left_A   Receives last left wheel output in Amps (may be nullptr)
 * @param right_A  Receives last right wheel output in Amps (may be nullptr)
 */
void wheel_speed_pi_get_last_output(float *left_A, float *right_A);
