#pragma once
#include <Arduino.h>
#include "coverage_planner.h"  // for Waypoint
#include "ekf_localiser.h"     // for Pose2D

// ══════════════════════════════════════════════════════════════════════════════
//  node_follower.h — Node-to-node path follower (replaces pure_pursuit)
//
//  The path is a SPARSE sequence of nodes (perimeter corners, strip endpoints)
//  joined by straight lines. The follower:
//    • drives to the ACTUAL node (no lookahead-carrot projection, which cut
//      corners on sharp sparse nodes and produced orbiting),
//    • PIVOTS on the spot to aim at the node when the heading error is large,
//      then drives a heading-P-controlled arc toward it,
//    • backs toward reverse (spur-exit) waypoints with a proportional turn.
//
//  Kinematic constants come from odo_calib (GPS-calibrated): the differential
//  mix uses odo_cal_track_m(); stall/slip uses vesc_erpm_to_velocity_scaled().
//
//  Call sequence (state machine task, 10 Hz, Core 1):
//    node_follower_init()                          — once at startup
//    WheelCmd cmd = node_follower_compute(...)     — every 100 ms
//    node_follower_to_vesc_duty(cmd)               — ramp duty toward target
//    (waypoint advancement is handled in state_machine.cpp)
//
//  Heading convention: 0 = North (+Y ENU), clockwise positive, ±π.
//  Cross-track sign:    positive = robot is to the right of the path.
// ══════════════════════════════════════════════════════════════════════════════


/** Wheel velocity command output. */
struct WheelCmd {
    float left_ms;   ///< Left wheel velocity (m/s, signed; negative = reverse)
    float right_ms;  ///< Right wheel velocity (m/s, signed; negative = reverse)
    float v_cmd;     ///< Commanded forward speed (m/s)
    float kappa;     ///< Yaw rate command (rad/s, positive = turn right/CW) — diagnostic
};


/** Clear all follower state. Call once from setup() before AUTO_MOWING. */
void node_follower_init();


/** Compute wheel velocity commands for the current pose and active node.
 *
 *    1. Reverse (spur-exit) waypoints: proportional backing controller.
 *    2. Otherwise target wp_list[wp_index] directly:
 *       a. heading error to the node;
 *       b. if |error| > PIVOT_ENTER_DEG, spin in place until < PIVOT_EXIT_DEG;
 *       c. else drive: forward speed from the profile, yaw rate = KP × error,
 *          differential mix using the calibrated track width.
 *    Updates cross-track error and the stall/slip timers on each call.
 *
 *  @param pose          Current EKF pose (heading 0=North, CW+, ±π)
 *  @param current_speed Current EKF forward speed (m/s)
 *  @param wp_list       Waypoint array
 *  @param wp_count      Number of waypoints
 *  @param wp_index      Current target index (drives to this node)
 *  @param cut_height_mm Current cut height (mm), for LONG_GRASS speed selection
 *  @param speed_scale   Uncertainty-aware speed multiplier (1.0 = full)
 *  @return WheelCmd to send to the VESC duty ramp */
WheelCmd node_follower_compute(const Pose2D &pose, float current_speed,
                               const Waypoint *wp_list, int wp_count, int wp_index,
                               float cut_height_mm, float speed_scale = 1.0f);


/** True when the steering centre is within waypoint_arrive_dist_m of wp. */
bool node_follower_waypoint_reached(const Pose2D &pose, const Waypoint &wp);


/** Signed cross-track error (m) from the last compute call.
 *  Positive = robot is to the right of the directed path segment. */
float node_follower_get_cross_track_error();


/** True if a forward-motion stall is currently detected
 *  (commanded > creep, wheels not moving, GPS/EKF confirms no motion). */
bool node_follower_is_stalled();

/** Reset the stall/slip timers and clear the flags. */
void node_follower_reset_stall();

/** True if wheel-slip is detected (wheels spinning, GPS/EKF barely moving). */
bool node_follower_is_slipping();

/** True while a pivot-on-the-spot (tank turn) is in progress. */
bool node_follower_is_pivoting();

/** True while driving deliberately backward to reach a behind-node (hysteresis).
 *  Intent gate: callers suppress "not moving forward" detectors (stall/slip) while
 *  this is true, since the wheels are advancing in reverse by design. */
bool node_follower_is_reversing();

/** Reset the session distance accumulator (collision-baseline settle gate).
 *  Call when AUTO_MOWING is entered fresh (not on resume from PAUSED). */
void node_follower_reset_session_distance();


/** Apply the duty-cycle ramp toward the commanded wheel velocities (sends duty
 *  to both drive VESCs). */
void node_follower_to_vesc_duty(const WheelCmd &cmd);

/** Get the last duty values sent to the drive VESCs (diagnostics). */
void node_follower_get_last_duty(float *left_duty, float *right_duty);
