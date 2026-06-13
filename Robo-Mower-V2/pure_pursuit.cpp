#include "pure_pursuit.h"
#include "config.h"
#include "mower_config.h"
#include "geometry.h"
#include "imu_bmi270.h"
#include "collision_detect.h"
#include "wheel_duty_ramp.h"
#include "ekf_localiser.h"
#include "vesc_can.h"
#include <math.h>

// ══════════════════════════════════════════════════════════════════════════════
//  pure_pursuit.cpp — Pure Pursuit path following controller
//
//  Implements the pure pursuit algorithm for a differential-drive robot.
//  See pure_pursuit.h for public API and HANDOFFS/16_pure_pursuit/HANDOFF.md
//  for algorithm notes and design decisions.
//
//  IMPORTANT heading convention note:
//    The EKF uses 0=North, clockwise-positive heading.
//    The robot-frame transform uses the correct rotation for this convention:
//      forward_component = dx·sin(h) + dy·cos(h)
//      right_component   = dx·cos(h) − dy·sin(h)
//    This differs from the spec pseudocode which assumed 0=East, CCW-positive.
//    See HANDOFFS/16_pure_pursuit/HANDOFF.md §Heading transform for details.
// ══════════════════════════════════════════════════════════════════════════════


// ── Module-level state ────────────────────────────────────────────────────────

static float    g_cross_track_error    = 0.0f;  ///< Last computed CTE (m)
static bool     g_stall_detected       = false; ///< True when stall condition active (wheels not moving)
static bool     g_slip_detected        = false; ///< True when wheels spinning but robot not moving
static uint32_t g_stall_start_ms       = 0;     ///< millis() when stall timer started (0=inactive)
static uint32_t g_slip_start_ms        = 0;     ///< millis() when slip condition first detected
// Note: per-wheel duty state is owned by wheel_duty_ramp.cpp
static float    g_session_distance_m   = 0.0f;  ///< Cumulative distance driven this AUTO session (m)


// ── Internal helpers ──────────────────────────────────────────────────────────

/**
 * Select target forward speed based on waypoint type and proximity.
 *
 * Speed tiers (highest to lowest priority):
 *   1. Transit (not mowing): TRANSIT_SPEED_MS
 *   2. Headland pass:        HEADLAND_SPEED_MS
 *   3. Long grass:           LONG_GRASS_SPEED_MS  (cut_height_mm > LONG_GRASS_THRESHOLD_MM)
 *   4. Normal mowing:        MAX_MOWING_SPEED_MS
 *
 * Within 0.5 m of the waypoint the speed is linearly reduced to zero, then
 * floored at MIN_CREEP_SPEED_MS so the robot always keeps moving forward.
 */
static float computeTargetSpeed(float dist_to_wp, bool mowing, bool headland,
                                 float cut_height_mm) {
    const MowerConfig &mc = mower_config_get();
    float v;
    if (!mowing)
        v = mc.transit_speed_ms;
    else if (headland)
        v = mc.headland_speed_ms;
    else if (cut_height_mm > (float)LONG_GRASS_THRESHOLD_MM)
        v = LONG_GRASS_SPEED_MS;
    else
        v = mc.max_mowing_speed_ms;

    // Linearly ramp down within 0.5 m of the waypoint
    if (dist_to_wp < 0.5f)
        v *= (dist_to_wp / 0.5f);

    // Never stop entirely mid-path
    v = max(v, mc.min_creep_speed_ms);
    return v;
}


/**
 * Find the lookahead point on the waypoint path.
 *
 * Starting from the robot's current position, the algorithm accumulates
 * straight-line (chord) distances along consecutive path segments:
 *   segment 0: robot_pos → wp[wp_index]
 *   segment 1: wp[wp_index] → wp[wp_index+1]
 *   ...
 *
 * When the accumulated distance reaches `lookahead`, the point is
 * linearly interpolated on the segment that crosses the threshold.
 *
 * If the path runs out before the lookahead distance is reached, the
 * last waypoint is returned as the target (robot is near path end).
 *
 * The returned Waypoint carries the mowing/headland flags of the segment
 * endpoint — these are used for speed selection.
 */
static Waypoint findLookaheadPoint(const Pose2D &pose,
                                    const Waypoint *wp_list, int wp_count,
                                    int wp_index, float lookahead) {
    if (wp_index >= wp_count)
        return wp_list[wp_count - 1];

    float px  = pose.x;
    float py  = pose.y;
    float acc = 0.0f;

    for (int i = wp_index; i < wp_count; i++) {
        float dx  = wp_list[i].x - px;
        float dy  = wp_list[i].y - py;
        float seg = sqrtf(dx * dx + dy * dy);

        if (seg < 1e-6f) {
            // Coincident points — skip without advancing accumulator
            px = wp_list[i].x;
            py = wp_list[i].y;
            continue;
        }

        if (acc + seg >= lookahead) {
            // Lookahead point lies on this segment — interpolate
            float t      = (lookahead - acc) / seg;
            Waypoint result = wp_list[i];
            result.x     = px + t * dx;
            result.y     = py + t * dy;
            // heading interpolation not needed — we only use x/y/mowing/headland
            return result;
        }

        acc += seg;
        px   = wp_list[i].x;
        py   = wp_list[i].y;
    }

    // Path end reached before accumulating lookahead — use last waypoint
    return wp_list[wp_count - 1];
}


/**
 * Compute and store the signed cross-track error.
 *
 * The current path segment is defined as:
 *   A = wp_list[wp_index - 1]  (or robot position when wp_index == 0)
 *   B = wp_list[wp_index]
 *
 * Sign: positive when the robot is to the RIGHT of the directed line A→B.
 *
 * Formula derivation (ENU, CW heading convention):
 *   The 2D signed area of triangle ABP (with P = robot position) is:
 *     2·area = (B.x−A.x)·(P.y−A.y) − (B.y−A.y)·(P.x−A.x)
 *   Positive area → P is to the LEFT of A→B (standard maths / CCW convention).
 *   We negate to obtain "positive = right" as required by the spec.
 */
static void computeCrossTrackError(const Pose2D &pose,
                                    const Waypoint *wp_list, int wp_count,
                                    int wp_index) {
    if (wp_count == 0) {
        g_cross_track_error = 0.0f;
        return;
    }

    // Segment start: previous waypoint (or robot position at start of plan)
    float ax, ay;
    if (wp_index > 0) {
        ax = wp_list[wp_index - 1].x;
        ay = wp_list[wp_index - 1].y;
    } else {
        ax = pose.x;
        ay = pose.y;
    }

    // Segment end: current target waypoint
    int idx = (wp_index < wp_count) ? wp_index : (wp_count - 1);
    float bx = wp_list[idx].x;
    float by = wp_list[idx].y;

    float seg_len = sqrtf((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
    if (seg_len < 1e-4f) {
        g_cross_track_error = 0.0f;
        return;
    }

    // Signed perpendicular distance, negated for "positive = right" convention
    float cross = (bx - ax) * (pose.y - ay) - (by - ay) * (pose.x - ax);
    g_cross_track_error = -cross / seg_len;
}


// ── Public API ────────────────────────────────────────────────────────────────

void pure_pursuit_init() {
    g_cross_track_error  = 0.0f;
    g_stall_detected     = false;
    g_slip_detected      = false;
    g_stall_start_ms     = 0;
    g_slip_start_ms      = 0;
    g_session_distance_m = 0.0f;
    wheel_duty_ramp_reset();
}

void pure_pursuit_reset_session_distance() {
    g_session_distance_m = 0.0f;
}


WheelCmd pure_pursuit_compute(const Pose2D &pose, float current_speed,
                               const Waypoint *wp_list, int wp_count, int wp_index,
                               float cut_height_mm, float speed_scale) {
    // Guard: no waypoints available
    if (wp_count == 0 || wp_index >= wp_count) {
        WheelCmd stop = {0.0f, 0.0f, 0.0f, 0.0f};
        return stop;
    }

    // ── Reverse waypoint (spur exit) ──────────────────────────────────────────
    // Robot drives backward toward the waypoint; blade stays on (mowing=true).
    // Uses a proportional angular controller rather than lookahead pursuit.
    if (wp_list[wp_index].reverse) {
        const Waypoint &rwp = wp_list[wp_index];
        float dx   = rwp.x - pose.x;
        float dy   = rwp.y - pose.y;
        float dist = sqrtf(dx * dx + dy * dy);

        // Direction from robot to waypoint (0=North, CW+ convention): atan2(dx, dy).
        float dir_to_wp = atan2f(dx, dy);

        // Robot's current backing direction (opposite of heading).
        float back_dir  = wrapAngle(pose.heading + (float)M_PI);

        // Angular error: positive = target is CW from backing direction.
        float angle_err = wrapAngle(dir_to_wp - back_dir);

        // Reverse speed, ramped to zero within 0.5 m of waypoint.
        const MowerConfig &mc_r = mower_config_get();
        float spd = mc_r.max_mowing_speed_ms;  // reverse at mowing speed
        if (dist < 0.5f) spd *= (dist / 0.5f);
        spd = max(spd, mc_r.min_creep_speed_ms);
        float v = -spd;  // negative = reverse

        // P-controller on angle_err: positive error → steer CW while reversing.
        float R_min = mower_config_min_turn_radius_m();
        float T     = mower_config_track_width_m();
        float kappa_max_r = (R_min > 0.01f) ? 1.0f / R_min : 10.0f;
        float kappa_r = clampf(angle_err * kappa_max_r, -kappa_max_r, kappa_max_r);

        float v_left  = v * (1.0f - kappa_r * T / 2.0f);
        float v_right = v * (1.0f + kappa_r * T / 2.0f);

        // Scale proportionally if either wheel exceeds the speed limit.
        float max_w = max(fabsf(v_left), fabsf(v_right));
        if (max_w > mc_r.max_wheel_speed_ms) {
            float sc = mc_r.max_wheel_speed_ms / max_w;
            v_left *= sc;
            v_right *= sc;
            v      *= sc;
        }

        // Accumulate reverse distance for baseline gate (uses |v|).
        g_session_distance_m += fabsf(v) * 0.1f;

        computeCrossTrackError(pose, wp_list, wp_count, wp_index);
        return {v_left, v_right, v, kappa_r};
    }

    // ── 1. Dynamic lookahead distance ─────────────────────────────────────────
    // Scale with speed for stability at higher velocities.
    // Floor at base and at 1.5× minimum turning radius so the lookahead point
    // is always reachable within the robot's curvature capability.
    const MowerConfig &mc = mower_config_get();
    float lookahead = max(mc.pp_lookahead_base_m,
                          mc.pp_lookahead_k * current_speed);
    lookahead = max(lookahead, 1.5f * mower_config_min_turn_radius_m());

    // ── 2. Find lookahead point on path ───────────────────────────────────────
    Waypoint target = findLookaheadPoint(pose, wp_list, wp_count, wp_index, lookahead);

    // ── 3. Transform to robot frame ───────────────────────────────────────────
    // EKF heading: 0 = North (+Y ENU), clockwise positive.
    // Robot forward unit vector in ENU: [sin(h), cos(h)]
    // Robot right  unit vector in ENU: [cos(h), −sin(h)]
    float dx = target.x - pose.x;
    float dy = target.y - pose.y;
    // Only the lateral (right) component is needed for curvature.
    float local_y = dx * cosf(pose.heading) - dy * sinf(pose.heading);

    // ── 4. Curvature ──────────────────────────────────────────────────────────
    // κ = 2·y_right / L²  (standard pure pursuit formula)
    // Positive κ → turn right (CW) → right wheel faster.
    float L2    = lookahead * lookahead;
    float kappa = 2.0f * local_y / L2;

    // ── 5. Clamp curvature to vehicle capability ──────────────────────────────
    float R_min2    = mower_config_min_turn_radius_m();
    float kappa_max = (R_min2 > 0.01f) ? 1.0f / R_min2 : 10.0f;
    kappa = clampf(kappa, -kappa_max, kappa_max);

    // ── 6. Speed profile ──────────────────────────────────────────────────────
    float dist_to_target = sqrtf(dx * dx + dy * dy);
    float v_cmd = computeTargetSpeed(dist_to_target, target.mowing, target.headland,
                                     cut_height_mm);

    // Apply uncertainty-aware speed scaling (1.0 = full speed, <1.0 near perimeter)
    v_cmd *= speed_scale;

    // ── 7. Differential wheel velocities ─────────────────────────────────────
    // v_left  = v · (1 − κ · T/2)  → slower for right turn (inner wheel)
    // v_right = v · (1 + κ · T/2)  → faster for right turn (outer wheel)
    float T2      = mower_config_track_width_m();
    float v_left  = v_cmd * (1.0f - kappa * T2 / 2.0f);
    float v_right = v_cmd * (1.0f + kappa * T2 / 2.0f);

    // ── 8. Scale down proportionally if any wheel exceeds speed limit ─────────
    float max_wheel = max(fabsf(v_left), fabsf(v_right));
    if (max_wheel > mc.max_wheel_speed_ms) {
        float scale = mc.max_wheel_speed_ms / max_wheel;
        v_left  *= scale;
        v_right *= scale;
        v_cmd   *= scale;
    }

    // ── Update cross-track error ──────────────────────────────────────────────
    computeCrossTrackError(pose, wp_list, wp_count, wp_index);

    // ── Stall and slip detection ──────────────────────────────────────────────
    // Uses VESC ERPM and GPS/EKF speed as cross-validation:
    //   STALL: commanded > threshold AND wheels not moving AND GPS not moving
    //   SLIP:  wheels spinning fast BUT GPS/EKF says robot is barely moving
    //          (indicates wheels sinking into soft ground — early-warning bog)
    {
        float vl = vesc_erpm_to_velocity(vesc_get_status(VESC_ID_LEFT).erpm);
        float vr = vesc_erpm_to_velocity(vesc_get_status(VESC_ID_RIGHT).erpm);
        float wheel_speed  = fabsf((vl + vr) * 0.5f);
        float ekf_speed    = fabsf(ekf_get_speed());

        // STALL: wheels aren't moving AND GPS/EKF confirms no movement
        bool wheels_moving = (wheel_speed > STALL_SPEED_THRESHOLD_MS);
        bool robot_moving  = (ekf_speed   > STALL_SPEED_THRESHOLD_MS);

        if (v_cmd > mc.min_creep_speed_ms && !wheels_moving && !robot_moving) {
            if (g_stall_start_ms == 0) g_stall_start_ms = millis();
            if ((uint32_t)(millis() - g_stall_start_ms) > STALL_DETECT_TIME_MS) {
                g_stall_detected = true;
            }
        } else {
            g_stall_start_ms = 0;
            g_stall_detected = false;
        }

        // SLIP: wheels spinning but GPS/EKF speed is less than half of wheel speed
        // Only flag when actually moving to avoid false positives at rest.
        bool slipping = wheels_moving &&
                        (ekf_speed < wheel_speed * SLIP_RATIO_THRESHOLD) &&
                        (v_cmd > mc.min_creep_speed_ms);
        if (slipping) {
            if (g_slip_start_ms == 0) g_slip_start_ms = millis();
            if ((uint32_t)(millis() - g_slip_start_ms) > STALL_DETECT_TIME_MS) {
                g_slip_detected = true;
            }
        } else {
            g_slip_start_ms = 0;
            g_slip_detected = false;
        }
    }

    // ── Accumulate session distance and update collision baseline gate ────────
    // dt is fixed at 0.1s (10Hz call rate from state machine task).
    g_session_distance_m += fabsf(v_cmd) * 0.1f;

    // Allow baseline updates during straight mowing only:
    //   - not turning (|yaw rate| < 0.2 rad/s)
    //   - actually moving (speed above creep threshold)
    //   - sufficient distance driven since AUTO start (settle gate)
    float gz = imu_get_gz_rads();
    bool straight = (fabsf(gz) < 0.2f)
                 && (current_speed > mc.min_creep_speed_ms)
                 && (g_session_distance_m > BASELINE_SETTLE_M);
    collisionDetectSetBaselineUpdate(straight);

    return {v_left, v_right, v_cmd, kappa};
}


bool pure_pursuit_waypoint_reached(const Pose2D &pose, const Waypoint &wp) {
    float dx = wp.x - pose.x;
    float dy = wp.y - pose.y;
    return sqrtf(dx * dx + dy * dy) <= mower_config_get().waypoint_arrive_dist_m;
}


float pure_pursuit_get_cross_track_error() {
    return g_cross_track_error;
}


bool pure_pursuit_is_stalled() {
    return g_stall_detected;
}

bool pure_pursuit_is_slipping() {
    return g_slip_detected;
}


void pure_pursuit_reset_stall() {
    g_stall_start_ms = 0;
    g_stall_detected = false;
    g_slip_start_ms  = 0;
    g_slip_detected  = false;
}


void pure_pursuit_to_vesc_duty(const WheelCmd &cmd) {
    wheel_duty_ramp_compute(VESC_ID_LEFT,  cmd.left_ms);
    wheel_duty_ramp_compute(VESC_ID_RIGHT, cmd.right_ms);
}

void pure_pursuit_get_last_duty(float *left_duty, float *right_duty) {
    wheel_duty_ramp_get_last(left_duty, right_duty);
}
