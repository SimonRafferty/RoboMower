// ══════════════════════════════════════════════════════════════════════════════
//  blade_recovery.cpp — RoboMower RPM-load recovery procedure (Feature 2)
//
//  Non-blocking 10 Hz state machine hosted in STATE_RETRACE. See blade_recovery.h
//  for the strategy. Built but DISABLED by default (BLADE_RPM_RECOVERY_ENABLED 0);
//  the entry guard in state_machine.cpp keeps it unreachable until enabled.
//
//  Geometry (all ENU metres; heading = EKF convention, 0 = North/+Y, CW+):
//    fwd unit vector  = (sin h, cos h)        — matches ekf_predict()
//    g_back  = entry − backup_dist · fwd      (perimeter-clipped behind)
//    g_front = entry + fwd_extent  · fwd      (perimeter-clipped ahead)
//    progress(pose) = (pose − g_back) · fwd   → 0 at g_back, L at g_front
//  Passes run between g_back and g_front, stepping the deck down each pass.
//
//  No vTaskDelay()/delay() is used.
// ══════════════════════════════════════════════════════════════════════════════

#include "blade_recovery.h"
#include "config.h"
#include "mower_config.h"
#include "servo_output.h"
#include "vesc_can.h"
#include "geometry.h"
#include "sys_log.h"
#include <math.h>

// ── Tuning local to this module ───────────────────────────────────────────────
static constexpr float k_fwd_extent_m   = 0.30f;  // recut distance ahead of entry (perimeter-clipped)
static constexpr float k_perim_margin_m = 0.10f;  // keep the back-up this far inside the perimeter
static constexpr float k_clip_step_m    = 0.05f;  // perimeter-clip scan resolution
static constexpr float k_reach_eps_m    = 0.03f;  // "arrived at segment end" tolerance
static constexpr float k_min_segment_m  = 0.10f;  // below this the mower is boxed in → give up

// ── Phases ────────────────────────────────────────────────────────────────────
enum BRPhase {
    BR_REVERSE,   ///< Backing up from entry to g_back (deck at max)
    BR_DWELL,     ///< Brief settle pause between moves
    BR_PASS,      ///< A forward or backward cutting pass over [g_back, g_front]
};
enum BRDir { BR_FWD, BR_BACK };

// ── Module state ──────────────────────────────────────────────────────────────
static float    g_back_x, g_back_y;     ///< Rear end of the recut segment (ENU m)
static float    g_heading;              ///< Entry heading (rad, EKF convention)
static float    g_seg_len_m;            ///< Segment length |g_back → g_front| (m)
static float    g_desired_mm;           ///< Operator cut height to restore (mm)
static float    g_height_mm;            ///< Deck height of the current pass (mm)
static int      g_pass_count;           ///< Completed passes
static BRPhase  g_phase;                ///< Current phase
static BRDir    g_dir;                  ///< Current pass direction
static uint32_t g_dwell_start_ms;       ///< millis() when the current dwell began
static bool     g_boxed_in;             ///< true if no room to maneuver → give up on first tick

// ── Helpers ───────────────────────────────────────────────────────────────────
static inline float clampf_(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static void stop_drive() {
    vesc_set_current(VESC_ID_LEFT,  0);
    vesc_set_current(VESC_ID_RIGHT, 0);
}

/** Wheel surface velocity → VESC current command (mA), clamped (matches retrace). */
static int32_t vel_to_current_mA(float v_ms) {
    float I = (v_ms / MAX_MOWING_SPEED_MS) * MAX_CURRENT_A;
    I = clampf_(I, -MAX_CURRENT_A, MAX_CURRENT_A);
    return (int32_t)(I * 1000.0f);
}

/** Drive along g_heading at signed speed v (>0 forward, <0 reverse), holding heading. */
static void drive(const Pose2D &pose, float v) {
    float he = wrapAngle(g_heading - pose.heading);
    float vl, vr;
    if (v >= 0.0f) {              // forward: outer wheel faster on the turn-in side
        vl = v * (1.0f - he * 0.5f);
        vr = v * (1.0f + he * 0.5f);
    } else {                      // reverse: flip the differential so the rear tracks the line
        vl = v * (1.0f + he * 0.5f);
        vr = v * (1.0f - he * 0.5f);
    }
    vl = clampf_(vl, -MAX_WHEEL_SPEED_MS, MAX_WHEEL_SPEED_MS);
    vr = clampf_(vr, -MAX_WHEEL_SPEED_MS, MAX_WHEEL_SPEED_MS);
    vesc_set_current(VESC_ID_LEFT,  vel_to_current_mA(vl));
    vesc_set_current(VESC_ID_RIGHT, vel_to_current_mA(vr));
}

/** Signed progress from g_back along g_heading (0 at g_back, g_seg_len_m at g_front). */
static float progress(const Pose2D &pose) {
    return (pose.x - g_back_x) * sinf(g_heading)
         + (pose.y - g_back_y) * cosf(g_heading);
}

/**
 * @brief Largest t in [0, max_d] keeping (ox,oy)+t·(dx,dy) safely inside `perim`.
 *
 * Scans outward in k_clip_step_m steps. If the perimeter is not a valid polygon
 * (< 3 vertices) the full max_d is allowed (no clip). Returns 0 if even the first
 * step leaves the keep-in margin.
 */
static float clip_distance(const Polygon &perim, float ox, float oy,
                           float dx, float dy, float max_d) {
    if (perim.pts.size() < 3) return max_d;
    float chosen = 0.0f;
    for (float t = k_clip_step_m; t <= max_d + 1e-3f; t += k_clip_step_m) {
        float px = ox + t * dx;
        float py = oy + t * dy;
        if (distanceToNearestEdge(perim, px, py) >= k_perim_margin_m) chosen = t;
        else break;
    }
    return chosen;
}

// ── Public API ────────────────────────────────────────────────────────────────

void blade_recovery_enter(const Pose2D &pose, const Polygon &perimeter,
                          float desired_height_mm) {
    g_heading    = pose.heading;
    g_desired_mm = desired_height_mm;

    const float fx = sinf(g_heading);
    const float fy = cosf(g_heading);

    // Perimeter-clip the reverse (behind) and the forward recut (ahead).
    float backup_dist = clip_distance(perimeter, pose.x, pose.y, -fx, -fy,
                                      BLADE_RECOVERY_BACKUP_MAX_M);
    float fwd_extent  = clip_distance(perimeter, pose.x, pose.y,  fx,  fy,
                                      k_fwd_extent_m);

    g_back_x = pose.x - backup_dist * fx;
    g_back_y = pose.y - backup_dist * fy;
    g_seg_len_m = backup_dist + fwd_extent;   // g_front = g_back + g_seg_len_m·fwd

    g_height_mm = (float)CUT_HEIGHT_MAX_MM;    // raise deck immediately
    servo_set_height_mm(g_height_mm);

    g_pass_count = 0;
    g_dir        = BR_FWD;
    g_boxed_in   = (g_seg_len_m < k_min_segment_m);

    if (backup_dist > k_reach_eps_m) {
        g_phase = BR_REVERSE;                  // back up first
    } else {
        g_phase = BR_DWELL;                    // no room behind — go straight to passes
        g_dwell_start_ms = millis();
    }

    char l[SYS_LOG_MAX_LEN];
    snprintf(l, sizeof(l), "BLADE-REC enter: back %.2fm fwd %.2fm seg %.2fm h->%dmm",
             (double)backup_dist, (double)fwd_extent, (double)g_seg_len_m,
             (int)g_height_mm);
    sys_log_push(l);
}

BladeRecoveryResult blade_recovery_update(const Pose2D &pose, float current_speed,
                                          CuttingStatus status) {
    (void)current_speed;

    if (status == BLADE_FAULT) {
        stop_drive();
        return BLADE_RECOVERY_BLADE_FAULT;
    }

    // Boxed in (can't maneuver in either direction): restore height and give up.
    if (g_boxed_in) {
        stop_drive();
        servo_set_height_mm(g_desired_mm);
        sys_log_push("BLADE-REC boxed in (no room) - resuming");
        return BLADE_RECOVERY_GAVE_UP;
    }

    const float prog = progress(pose);

    switch (g_phase) {
        case BR_REVERSE: {
            float creep = mower_config_get().min_creep_speed_ms;
            drive(pose, -creep);
            if (prog <= k_reach_eps_m) {       // reached g_back
                stop_drive();
                g_phase = BR_DWELL;
                g_dwell_start_ms = millis();
            }
            break;
        }

        case BR_DWELL: {
            stop_drive();
            if (millis() - g_dwell_start_ms >= (uint32_t)BOG_PASS_DWELL_MS) {
                g_phase = BR_PASS;
            }
            break;
        }

        case BR_PASS: {
            bool complete = false;
            if (g_dir == BR_FWD) {
                drive(pose, LONG_GRASS_SPEED_MS);
                if (prog >= g_seg_len_m - k_reach_eps_m) complete = true;
            } else {
                drive(pose, -LONG_GRASS_SPEED_MS);
                if (prog <= k_reach_eps_m) complete = true;
            }

            if (complete) {
                stop_drive();
                g_pass_count++;

                // A full pass completed at the operator's height → recovered.
                if (g_height_mm <= g_desired_mm + 0.01f) {
                    servo_set_height_mm(g_desired_mm);
                    sys_log_push("BLADE-REC complete - resuming AUTO");
                    return BLADE_RECOVERY_COMPLETE;
                }

                // Retry budget exhausted while still above desired → give up.
                if (g_pass_count >= BLADE_RECOVERY_MAX_PASSES) {
                    servo_set_height_mm(g_desired_mm);
                    char l[SYS_LOG_MAX_LEN];
                    snprintf(l, sizeof(l),
                             "BLADE-REC gave up after %d passes (difficult patch) - resuming",
                             g_pass_count);
                    sys_log_push(l);
                    return BLADE_RECOVERY_GAVE_UP;
                }

                // Step the deck down toward the desired height, reverse, dwell.
                g_height_mm -= (float)BOG_HEIGHT_STEP_MM;
                if (g_height_mm < g_desired_mm) g_height_mm = g_desired_mm;
                servo_set_height_mm(g_height_mm);
                g_dir   = (g_dir == BR_FWD) ? BR_BACK : BR_FWD;
                g_phase = BR_DWELL;
                g_dwell_start_ms = millis();
            }
            break;
        }
    }

    return BLADE_RECOVERY_IN_PROGRESS;
}

void blade_recovery_exit() {
    stop_drive();
}
