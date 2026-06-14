#pragma once
// ══════════════════════════════════════════════════════════════════════════════
//  heading_controller.h — Gyro-stabilised heading hold with AGC
//
//  PD controller with Automatic Gain Control.  Two usage modes:
//
//    Manual heading hold (the only current caller, in state_machine.cpp):
//      Pass heading_error = wrapAngle(desired - actual)  and
//      yaw_rate_error = desired_yaw_rate - actual_yaw_rate.
//      Uses both Kp and Kd for full PD heading hold.
//
//    Yaw-damp-only mode:
//      Pass heading_error = 0  and
//      yaw_rate_error = commanded_yaw_rate - actual_yaw_rate.
//      Uses Kd only — heading is handled elsewhere (e.g. by path geometry).
//
//  AGC: monitors heading error zero-crossings (oscillation) and mean
//  magnitude (persistent error).  Slowly adjusts an internal gain
//  multiplier between AGC_GAIN_MIN and AGC_GAIN_MAX.
//
//  Thread safety: call only from the state machine task (Core 1).
// ══════════════════════════════════════════════════════════════════════════════

/// Reset the heading controller state and AGC gain to 1.0.
/// Call on mode entry (MANUAL, LEARN, AUTO_MOWING).
void heading_controller_reset();

/// Compute differential correction.
/// @param heading_error   wrapAngle(desired - actual), rad.  Pass 0 for yaw-damp only.
/// @param yaw_rate_error  (desired_yaw_rate - actual_yaw_rate), rad/s
/// @param kp              proportional gain (from MowerConfig)
/// @param kd              derivative gain (from MowerConfig)
/// @param dt              time step (s)
/// @return correction value (caller scales to amps or m/s as needed)
float heading_controller_compute(float heading_error, float yaw_rate_error,
                                  float kp, float kd, float dt);

/// Current AGC gain multiplier (for diagnostics).  1.0 = nominal.
float heading_controller_get_gain();
