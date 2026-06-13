#pragma once
#include <Arduino.h>
#include "coverage_planner.h"  // for Waypoint
#include "ekf_localiser.h"     // for Pose2D

// ══════════════════════════════════════════════════════════════════════════════
//  pure_pursuit.h — Pure Pursuit path following controller
//
//  Converts a waypoint sequence (from coverage_planner) into wheel velocity
//  commands for the VESC motor controllers.
//
//  Call sequence (from state machine task, 10 Hz, Core 1):
//    pure_pursuit_init()                          — once at startup
//    WheelCmd cmd = pure_pursuit_compute(...)     — every 100 ms
//    pure_pursuit_to_vesc_duty(cmd)               — ramp duty toward target
//    if (pure_pursuit_waypoint_reached(...)) ...  — advance wp_index
//    if (pure_pursuit_is_stalled()) ...           — trigger bog recovery
//
//  Heading convention: 0 = North (+Y in ENU), clockwise positive, ±π.
//  Curvature sign:     positive = turn right (CW).
//  Cross-track sign:   positive = robot is to the right of path.
//
//  Source references:
//    Spec: Robo_Mower_claudecode_prompt_v3.md §Pure Pursuit Path Controller
//    Config constants: config.h
//    Handoff: HANDOFFS/16_pure_pursuit/HANDOFF.md
// ══════════════════════════════════════════════════════════════════════════════


/** Wheel velocity command output */
struct WheelCmd {
    float left_ms;   ///< Left wheel velocity (m/s, signed; negative = reverse)
    float right_ms;  ///< Right wheel velocity (m/s, signed; negative = reverse)
    float v_cmd;     ///< Commanded forward speed (m/s)
    float kappa;     ///< Curvature (1/m, positive = turn right/CW)
};


/** Initialise pure pursuit controller — clear all state variables.
 *  Call once from setup() before entering AUTO_MOWING. */
void pure_pursuit_init();


/** Compute wheel velocity commands for current pose and active waypoint path.
 *
 *  Implements dynamic-lookahead Pure Pursuit:
 *    1. Compute lookahead distance scaled by current speed.
 *    2. Find the lookahead point on the path (along-path distance from robot).
 *    3. Transform to robot frame using EKF heading convention (0=North, CW+).
 *    4. Compute curvature κ = 2·y_right / L², clamped to 1/MIN_TURNING_RADIUS_M.
 *    5. Select speed from mowing/headland/long-grass/transit profile.
 *    6. Compute differential wheel velocities, scaled if either exceeds MAX_WHEEL_SPEED_MS.
 *
 *  Also updates the cross-track error and stall timer on each call.
 *
 *  Call at 10 Hz from state machine task.
 *
 *  @param pose          Current EKF pose (heading: 0=North, CW positive, ±π)
 *  @param current_speed Current EKF forward speed (m/s)
 *  @param wp_list       Pointer to waypoint array
 *  @param wp_count      Number of waypoints in wp_list
 *  @param wp_index      Current waypoint index (pursuit targets from here forward)
 *  @param cut_height_mm Current cut height (mm), for LONG_GRASS speed selection
 *  @return WheelCmd with wheel velocities and curvature to send to VESCs */
WheelCmd pure_pursuit_compute(const Pose2D &pose, float current_speed,
                               const Waypoint *wp_list, int wp_count, int wp_index,
                               float cut_height_mm, float speed_scale = 1.0f);


/** Check if the robot has arrived at a waypoint.
 *
 *  Returns true when the Euclidean distance from the robot steering centre
 *  to the waypoint is within WAYPOINT_ARRIVE_DIST_M.
 *
 *  @param pose  Current EKF pose (steering centre in ENU metres)
 *  @param wp    Waypoint to test
 *  @return true if within WAYPOINT_ARRIVE_DIST_M of wp */
bool pure_pursuit_waypoint_reached(const Pose2D &pose, const Waypoint &wp);


/** Get the signed cross-track error computed on the last pure_pursuit_compute() call.
 *
 *  Cross-track error is the perpendicular distance from the robot steering
 *  centre to the line through the previous and current waypoints.
 *
 *  Sign convention: positive = robot is to the right of the path direction.
 *
 *  The caller (state machine) is responsible for triggering
 *  coverage_planner_reset_to_nearest() if |error| > 0.5 m for > 2 s.
 *
 *  @return Signed cross-track error in metres */
float pure_pursuit_get_cross_track_error();


/** Check whether a forward-motion stall has been detected.
 *
 *  A stall is flagged when the commanded speed exceeds 0.1 m/s AND the
 *  measured speed remains below STALL_SPEED_THRESHOLD_MS for longer than
 *  STALL_DETECT_TIME_MS milliseconds.
 *
 *  @return true if stall condition is active */
bool pure_pursuit_is_stalled();


/** Reset the stall timer and clear the stall flag.
 *  Call when valid forward motion is detected (e.g., after bog recovery). */
void pure_pursuit_reset_stall();


/** Check whether wheel-slip has been detected.
 *
 *  Slip is flagged when the VESC eRPM reports the wheels spinning but the
 *  GPS/EKF speed is less than SLIP_RATIO_THRESHOLD × wheel_speed for longer
 *  than STALL_DETECT_TIME_MS.  This is an early-warning sign of the mower
 *  sinking into soft ground before a full stall occurs.
 *
 *  @return true if slip condition is active */
bool pure_pursuit_is_slipping();


/** Reset the session distance accumulator to zero.
 *  Call when AUTO_MOWING is entered fresh (not when resuming from PAUSED).
 *  Used by the collision baseline settle gate (BASELINE_SETTLE_M). */
void pure_pursuit_reset_session_distance();


/** Apply duty-cycle ramp toward desired wheel velocities.
 *
 *  Reads actual VESC eRPM, computes error vs desired speed, and increments or
 *  decrements per-wheel duty by DUTY_RAMP_STEP.  Sends duty directly to VESCs.
 *  No PI integral — just a gentle one-step-per-tick ramp.
 *
 *  @param cmd  Desired wheel velocity command from pure_pursuit_compute(). */
void pure_pursuit_to_vesc_duty(const WheelCmd &cmd);


/** Get the last duty values sent to the drive VESCs (for diagnostics).
 *  @param left_duty   Output: last duty sent to VESC_ID_LEFT  [-1, +1]
 *  @param right_duty  Output: last duty sent to VESC_ID_RIGHT [-1, +1] */
void pure_pursuit_get_last_duty(float *left_duty, float *right_duty);
