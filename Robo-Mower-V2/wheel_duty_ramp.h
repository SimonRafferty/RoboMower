#pragma once
#include <Arduino.h>

// ══════════════════════════════════════════════════════════════════════════════
//  wheel_duty_ramp.h — Per-wheel duty-cycle ramp for AUTO_MOWING
//
//  Replaces wheel_speed_pi.h/.cpp.  Rather than a PI current loop, this module
//  adjusts duty cycle by a fixed step each 100 ms tick to bring actual VESC
//  eRPM toward the target speed.  The ramp eliminates PI integral windup and
//  behaves predictably on variable ground (soft grass, wet, uphill).
//
//  Call wheel_duty_ramp_compute() once per tick for each drive VESC.
//  Call wheel_duty_ramp_reset() on AUTO_MOWING entry and any stop event.
//
//  Step size is DUTY_RAMP_STEP (config.h).  Duty ceiling is manual_max_duty
//  from MowerConfig.
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Reset per-wheel duty to zero.
 *
 * Call on AUTO_MOWING entry and whenever the drive is commanded to stop.
 */
void wheel_duty_ramp_reset();

/**
 * @brief Compute and apply a duty-cycle step toward the desired wheel speed.
 *
 * Compares desired_ms with actual VESC eRPM and increments or decrements the
 * stored per-wheel duty by DUTY_RAMP_STEP.  Sends the updated duty directly
 * to the VESC via vesc_set_duty().
 *
 * @param vesc_id     VESC_ID_LEFT or VESC_ID_RIGHT.
 * @param desired_ms  Target wheel surface velocity (m/s). Positive = forward.
 *                    Pass 0.0f to ramp duty back to zero.
 * @return Current duty value sent to VESC (useful for diagnostics).
 */
float wheel_duty_ramp_compute(uint8_t vesc_id, float desired_ms);

/**
 * @brief Get last computed duty values (for telemetry / diagnostics).
 *
 * @param left_duty   Output: last duty sent to VESC_ID_LEFT.
 * @param right_duty  Output: last duty sent to VESC_ID_RIGHT.
 */
void wheel_duty_ramp_get_last(float *left_duty, float *right_duty);
