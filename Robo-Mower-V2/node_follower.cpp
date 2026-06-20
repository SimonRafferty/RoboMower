#include "node_follower.h"
#include "config.h"
#include "mower_config.h"
#include "geometry.h"
#include "imu.h"
#include "collision_detect.h"
#include "wheel_duty_ramp.h"
#include "ekf_localiser.h"
#include "vesc_can.h"
#include "odo_calib.h"
#include "sys_log.h"
#include "cutting_monitor.h"   // Feature 2: RPM load for (disabled) adaptive speed
#include <math.h>

// ══════════════════════════════════════════════════════════════════════════════
//  node_follower.cpp — Node-to-node path follower
//
//  See node_follower.h for the public API and rationale.
//
//  Heading convention: EKF uses 0=North, clockwise-positive, ±π.
//    Robot-frame right component = dx·cos(h) − dy·sin(h).
//    Bearing to a point          = atan2(dx, dy)  (0=North, CW+).
//    +heading error → node is to the RIGHT (CW) → turn right → LEFT wheel faster.
// ══════════════════════════════════════════════════════════════════════════════


// ── Module-level state ────────────────────────────────────────────────────────
static float    g_cross_track_error  = 0.0f;
static bool     g_stall_detected     = false;
static bool     g_slip_detected      = false;
static uint32_t g_stall_start_ms     = 0;
static uint32_t g_slip_start_ms      = 0;
static bool     g_pivoting           = false;   ///< spinning in place to align
static bool     g_reversing          = false;   ///< driving backward to a behind-node (hysteresis)
static int      g_reverse_lockout    = 0;       ///< waypoints to force FORWARD after a reverse (anti-lock-in)
static int      g_nf_last_wp_index   = -1;      ///< previous wp_index, for detecting advancement
static float    g_session_distance_m = 0.0f;    ///< distance driven this AUTO session


// ── Internal helpers ──────────────────────────────────────────────────────────

/** Select target forward speed by waypoint type and proximity to the node.
 *  Tiers: transit > headland > long-grass > mowing; ramped to zero within 0.5 m
 *  of the node. The absolute min_creep_speed floor is applied by the CALLER,
 *  AFTER the uncertainty speed_scale (see node_follower_compute) — flooring here
 *  would be undone by the later ×speed_scale, and the old two-floor arrangement
 *  let the perimeter speed_scale cancel max_mowing_speed entirely. */
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

    if (dist_to_wp < 0.5f)
        v *= (dist_to_wp / 0.5f);

    return v;
}


/** Compute and store the signed cross-track error for the current segment.
 *  Segment: A = wp[index-1] (or robot position at index 0) → B = wp[index].
 *  Sign: positive when the robot is to the RIGHT of the directed line A→B. */
static void computeCrossTrackError(const Pose2D &pose,
                                    const Waypoint *wp_list, int wp_count,
                                    int wp_index) {
    if (wp_count == 0) { g_cross_track_error = 0.0f; return; }

    float ax, ay;
    if (wp_index > 0) {
        ax = wp_list[wp_index - 1].x;
        ay = wp_list[wp_index - 1].y;
    } else {
        ax = pose.x;
        ay = pose.y;
    }

    int idx = (wp_index < wp_count) ? wp_index : (wp_count - 1);
    float bx = wp_list[idx].x;
    float by = wp_list[idx].y;

    float seg_len = sqrtf((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
    if (seg_len < 1e-4f) { g_cross_track_error = 0.0f; return; }

    // Signed perpendicular distance, negated for "positive = right".
    float cross = (bx - ax) * (pose.y - ay) - (by - ay) * (pose.x - ax);
    g_cross_track_error = -cross / seg_len;
}


/** Stall and slip detection from VESC eRPM cross-validated against GPS/EKF speed.
 *  Uses the SCALED wheel velocity so it matches the calibrated odometry. */
static void updateStallSlip(float v_cmd) {
    const MowerConfig &mc = mower_config_get();
    float vl = vesc_erpm_to_velocity_scaled(vesc_get_status(VESC_ID_LEFT).erpm);
    float vr = vesc_erpm_to_velocity_scaled(vesc_get_status(VESC_ID_RIGHT).erpm);
    float wheel_speed = fabsf((vl + vr) * 0.5f);
    float ekf_speed   = fabsf(ekf_get_speed());

    bool wheels_moving = (wheel_speed > STALL_SPEED_THRESHOLD_MS);
    bool robot_moving  = (ekf_speed   > STALL_SPEED_THRESHOLD_MS);

    if (v_cmd > mc.min_creep_speed_ms && !wheels_moving && !robot_moving) {
        if (g_stall_start_ms == 0) g_stall_start_ms = millis();
        if ((uint32_t)(millis() - g_stall_start_ms) > STALL_DETECT_TIME_MS)
            g_stall_detected = true;
    } else {
        g_stall_start_ms = 0;
        g_stall_detected = false;
    }

    bool slipping = wheels_moving &&
                    (ekf_speed < wheel_speed * SLIP_RATIO_THRESHOLD) &&
                    (v_cmd > mc.min_creep_speed_ms);
    if (slipping) {
        if (g_slip_start_ms == 0) g_slip_start_ms = millis();
        if ((uint32_t)(millis() - g_slip_start_ms) > STALL_DETECT_TIME_MS)
            g_slip_detected = true;
    } else {
        g_slip_start_ms = 0;
        g_slip_detected = false;
    }
}


// ── Public API ────────────────────────────────────────────────────────────────

void node_follower_init() {
    g_cross_track_error  = 0.0f;
    g_stall_detected     = false;
    g_slip_detected      = false;
    g_stall_start_ms     = 0;
    g_slip_start_ms      = 0;
    g_pivoting           = false;
    g_reversing          = false;
    g_reverse_lockout    = 0;
    g_nf_last_wp_index   = -1;
    g_session_distance_m = 0.0f;
    wheel_duty_ramp_reset();
}

void node_follower_reset_session_distance() {
    g_session_distance_m = 0.0f;
}


WheelCmd node_follower_compute(const Pose2D &pose, float current_speed,
                               const Waypoint *wp_list, int wp_count, int wp_index,
                               float cut_height_mm, float speed_scale) {
    if (wp_count == 0 || wp_index >= wp_count) {
        WheelCmd stop = {0.0f, 0.0f, 0.0f, 0.0f};
        return stop;
    }

    const MowerConfig &mc = mower_config_get();
    const float T = odo_cal_track_m();   // calibrated kinematic track width

    // ── Reverse waypoint (spur exit) ──────────────────────────────────────────
    // Robot drives backward toward the waypoint; blade stays on (mowing=true).
    if (wp_list[wp_index].reverse) {
        const Waypoint &rwp = wp_list[wp_index];
        float dx   = rwp.x - pose.x;
        float dy   = rwp.y - pose.y;
        float dist = sqrtf(dx * dx + dy * dy);

        float dir_to_wp = atan2f(dx, dy);                      // 0=North, CW+
        float back_dir  = wrapAngle(pose.heading + (float)M_PI); // backing direction
        float angle_err = wrapAngle(dir_to_wp - back_dir);

        float spd = mc.max_mowing_speed_ms;
        if (dist < 0.5f) spd *= (dist / 0.5f);
        spd = max(spd, mc.min_creep_speed_ms);
        float v = -spd;

        float R_min = mower_config_min_turn_radius_m();
        float kappa_max_r = (R_min > 0.01f) ? 1.0f / R_min : 10.0f;
        float kappa_r = clampf(angle_err * kappa_max_r, -kappa_max_r, kappa_max_r);

        float v_left  = v * (1.0f - kappa_r * T / 2.0f);
        float v_right = v * (1.0f + kappa_r * T / 2.0f);

        float max_w = max(fabsf(v_left), fabsf(v_right));
        if (max_w > mc.max_wheel_speed_ms) {
            float sc = mc.max_wheel_speed_ms / max_w;
            v_left *= sc; v_right *= sc; v *= sc;
        }

        g_session_distance_m += fabsf(v) * 0.1f;
        computeCrossTrackError(pose, wp_list, wp_count, wp_index);
        return {v_left, v_right, v, kappa_r};
    }

    // ── Geometry to the target node (drive to the node itself) ────────────────
    const Waypoint &node = wp_list[wp_index];
    float dx = node.x - pose.x;
    float dy = node.y - pose.y;
    float dist = sqrtf(dx * dx + dy * dy);
    float bearing = atan2f(dx, dy);                     // 0=North, CW+
    float hdg_err = wrapAngle(bearing - pose.heading);  // +ve = node is CW (right)

    // ── Reverse availability: counter lock (geometry-independent) ─────────────
    // Reverse can help reach a behind-node, but must NEVER lock the mower into
    // driving backward indefinitely (a geometry lookahead was unreliable). Allow at
    // most ONE backward node, then force the next REVERSE_FWD_LOCK_WAYPOINTS forward.
    // Track waypoint advancement (the state machine owns wp_index) to count the lock
    // down. A reverse that just completed (g_reversing true at an advance) arms the
    // lock; a wp_index reset (plan reload) clears it.
    if (wp_index < g_nf_last_wp_index) {
        g_reverse_lockout = 0;
    } else if (g_nf_last_wp_index >= 0 && wp_index > g_nf_last_wp_index) {
        if (g_reversing) {
            g_reverse_lockout = REVERSE_FWD_LOCK_WAYPOINTS;   // finished a reverse → lock forward
        } else if (g_reverse_lockout > 0) {
            g_reverse_lockout -= (wp_index - g_nf_last_wp_index);
            if (g_reverse_lockout < 0) g_reverse_lockout = 0;
        }
    }
    g_nf_last_wp_index = wp_index;

    // ── Forward vs REVERSE selection ──────────────────────────────────────────
    // Node BEHIND (|heading error| > ~90°): align the BACK and drive backward (one
    // short node), unless the forward-lock is active. Otherwise face the node and
    // drive forward. Hysteresis around 90° prevents chatter. steer_err is the
    // alignment error of whichever END leads (front fwd / back rev) — ≤90° once set.
    {
        const float rev_enter = REV_ENTER_DEG * (float)M_PI / 180.0f;
        const float rev_exit  = REV_EXIT_DEG  * (float)M_PI / 180.0f;
        bool was_reversing = g_reversing;
        if (g_reverse_lockout > 0) {
            g_reversing = false;                 // forward-locked after a recent reverse
        } else if (!g_reversing) {
            if (fabsf(hdg_err) > rev_enter) g_reversing = true;
        } else {
            if (fabsf(hdg_err) < rev_exit)  g_reversing = false;
        }
        if (g_reversing && !was_reversing) sys_log_push("NF reverse 1 node (behind)");
        else if (!g_reversing && was_reversing) sys_log_push("NF forward");
    }
    // Alignment error and travel direction: reverse aligns the BACK (heading+π).
    float steer_err = g_reversing ? wrapAngle(hdg_err - (float)M_PI) : hdg_err;
    float dir_sign  = g_reversing ? -1.0f : 1.0f;   // -1 = drive backward

    // ── Pivot-on-the-spot (tracked-vehicle tank turn) ─────────────────────────
    // When the alignment error exceeds PIVOT_ENTER_DEG, spin in place (one track
    // forward, one reverse) until it drops below PIVOT_EXIT_DEG, then drive.
    // Hysteresis stops chatter. The differential-odometry EKF tracks heading
    // through the spin, so this closes the loop on the alignment.
    {
        const float enter_rad = PIVOT_ENTER_DEG * (float)M_PI / 180.0f;
        const float exit_rad  = PIVOT_EXIT_DEG  * (float)M_PI / 180.0f;

        bool was_pivoting = g_pivoting;
        if (!g_pivoting && fabsf(steer_err) > enter_rad)      g_pivoting = true;
        else if (g_pivoting && fabsf(steer_err) < exit_rad)   g_pivoting = false;

        if (g_pivoting) {
            if (!was_pivoting) {
                char l[48];
                snprintf(l, sizeof(l), "NF pivot start %+.0f deg%s",
                         (double)(steer_err * 180.0f / (float)M_PI),
                         g_reversing ? " (rev)" : "");
                sys_log_push(l);
            }
            // +steer_err → turn right (CW): left track forward, right track reverse.
            float dir = (steer_err >= 0.0f) ? 1.0f : -1.0f;
            float vl  =  dir * PIVOT_WHEEL_MS;
            float vr  = -dir * PIVOT_WHEEL_MS;
            // Spinning in place is not forward motion — clear stall/slip timers.
            g_stall_start_ms = 0; g_stall_detected = false;
            g_slip_start_ms  = 0; g_slip_detected  = false;
            computeCrossTrackError(pose, wp_list, wp_count, wp_index);
            return { vl, vr, 0.0f, dir * NODE_YAW_RATE_MAX };
        }
    }

    // ── Drive branch: heading P-controller + speed profile ────────────────────
    // Base speed (from config) → scaled down near the perimeter → floored at the
    // absolute creep speed. Order matters: the floor is applied AFTER speed_scale
    // so a slowdown can only reach min_creep, never zero, and so max_mowing_speed
    // is NOT cancelled by a min_creep/max_mowing fraction (the old bug — the mow
    // speed setting then had no effect anywhere margin <= 0, i.e. the outer rings).
    float v_cmd = computeTargetSpeed(dist, node.mowing, node.headland, cut_height_mm);
    v_cmd *= speed_scale;
#if BLADE_LOAD_ADAPTIVE_SPEED_ENABLED
    // Feature 2 (DISABLED by default): slow proportionally to RPM-based blade load.
    // Applied BEFORE the min_creep floor, so high load can only slow to creep, never
    // to a stall. factor = 1 at/below the knee, ramping to MIN_FACTOR at full load.
    {
        float load = cutting_monitor_get_rpm_load_fraction();
        float over = load - BLADE_LOAD_SPEED_KNEE;
        if (over > 0.0f) {
            float f = 1.0f - (over / (1.0f - BLADE_LOAD_SPEED_KNEE))
                               * (1.0f - BLADE_LOAD_SPEED_MIN_FACTOR);
            f = (f < BLADE_LOAD_SPEED_MIN_FACTOR) ? BLADE_LOAD_SPEED_MIN_FACTOR
              : (f > 1.0f ? 1.0f : f);
            v_cmd *= f;
        }
    }
#endif
    v_cmd = max(v_cmd, mc.min_creep_speed_ms);

    // Reversing to a behind-node uses the SAME speed profile/constraints as forward
    // (per operator): apply the direction sign here. v_cmd is now signed (negative =
    // backward); the body-twist differential mix below is direction-agnostic, so the
    // same formula steers the leading end (front fwd / back rev) toward the node.
    v_cmd *= dir_sign;

    // Yaw rate proportional to the alignment error, capped. omega>0 = turn right (CW).
    float omega = clampf(NODE_HEADING_KP * steer_err, -NODE_YAW_RATE_MAX, NODE_YAW_RATE_MAX);

    // Differential mix: half the differential speed onto each wheel.
    // +omega (CW/right) → left (outer) wheel faster, right (inner) slower.
    float half = omega * T * 0.5f;
    float v_left  = v_cmd + half;
    float v_right = v_cmd - half;

    // Scale down proportionally if a wheel exceeds the speed limit.
    float max_wheel = max(fabsf(v_left), fabsf(v_right));
    if (max_wheel > mc.max_wheel_speed_ms) {
        float scale = mc.max_wheel_speed_ms / max_wheel;
        v_left  *= scale;
        v_right *= scale;
        v_cmd   *= scale;
    }

    computeCrossTrackError(pose, wp_list, wp_count, wp_index);
    updateStallSlip(v_cmd);

    // ── Session distance + collision baseline settle gate ─────────────────────
    g_session_distance_m += fabsf(v_cmd) * 0.1f;   // dt fixed 0.1 s @ 10 Hz
    float gz = imu_get_gz_rads();
    bool straight = (fabsf(gz) < 0.2f)
                 && (current_speed > mc.min_creep_speed_ms)
                 && (g_session_distance_m > BASELINE_SETTLE_M);
    collisionDetectSetBaselineUpdate(straight);

    return {v_left, v_right, v_cmd, omega};
}


bool node_follower_waypoint_reached(const Pose2D &pose, const Waypoint &wp) {
    float dx = wp.x - pose.x;
    float dy = wp.y - pose.y;
    return sqrtf(dx * dx + dy * dy) <= mower_config_get().waypoint_arrive_dist_m;
}

float node_follower_get_cross_track_error() { return g_cross_track_error; }

bool node_follower_is_stalled()  { return g_stall_detected; }
bool node_follower_is_slipping() { return g_slip_detected; }

void node_follower_reset_stall() {
    g_stall_start_ms = 0; g_stall_detected = false;
    g_slip_start_ms  = 0; g_slip_detected  = false;
    g_pivoting       = false;
    g_reversing      = false;
    g_reverse_lockout  = 0;
    g_nf_last_wp_index = -1;
}

bool node_follower_is_pivoting() { return g_pivoting; }

void node_follower_to_vesc_duty(const WheelCmd &cmd) {
    wheel_duty_ramp_compute(VESC_ID_LEFT,  cmd.left_ms);
    wheel_duty_ramp_compute(VESC_ID_RIGHT, cmd.right_ms);
}

void node_follower_get_last_duty(float *left_duty, float *right_duty) {
    wheel_duty_ramp_get_last(left_duty, right_duty);
}
