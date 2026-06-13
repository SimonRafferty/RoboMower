// ══════════════════════════════════════════════════════════════════════════════
//  retrace.cpp — RoboMower RETRACE overload-recovery procedure
//
//  Implements a non-blocking state machine called at 10 Hz from the
//  RETRACE state in state_machine.cpp.
//
//  Overview:
//    On entry the robot's position is recorded as retrace_start and the
//    current strip end waypoint as retrace_end.  The deck is raised to
//    CUT_HEIGHT_MAX_MM immediately.
//
//    Each update tick drives the robot forward (or backward on alternate
//    passes) along the recorded heading, monitoring CuttingStatus:
//      - BLADE_FAULT       → RETRACE_BLADE_FAULT  (immediate)
//      - CUTTING_STALLED   → RETRACE_STALLED       (immediate, to BOG_RECOVERY)
//
//    When the robot reaches the extended end of the current pass segment,
//    the pass counter is incremented.  If the retry budget is not exhausted
//    and the current height is still above desired, the deck steps down by
//    BOG_HEIGHT_STEP_MM, direction is reversed, and the next pass begins.
//
//    Passes alternate FORWARD (start→end) and BACKWARD (end→start).
//    Each pass is extended by RETRACE_OVERLAP_M beyond each end of the
//    recorded segment so the blade cuts through any dense fringe material.
//
//  Heading convention throughout:
//    retrace_heading is stored from pose.heading at entry (EKF convention:
//    0 = North / +Y, clockwise positive, wrapped ±π).
//
//  Progress projection formula (matches bog_recovery.cpp convention):
//    progress = (pose.x − start_x) × cos(retrace_heading)
//             + (pose.y − start_y) × sin(retrace_heading)
//    (standard maths convention for cos/sin — consistent with bog_recovery.)
//
//  No vTaskDelay() is used anywhere in this file.
//
//  Source references:
//    Spec:   Robo_Mower_claudecode_prompt_v3.md §"RETRACE state"
//    Config: Robo-Mower-V2/config.h
//    HANDOFF:HANDOFFS/19_retrace/HANDOFF.md
// ══════════════════════════════════════════════════════════════════════════════

#include "retrace.h"
#include "config.h"
#include "servo_output.h"
#include "vesc_can.h"
#include "geometry.h"
#include <math.h>

// ── Direction enum ────────────────────────────────────────────────────────────

/** Drive direction for the current retrace pass */
enum RetraceDir {
    RDIR_FORWARD,   ///< Driving from retrace_start toward retrace_end (+ overlap)
    RDIR_BACKWARD,  ///< Driving from retrace_end back toward retrace_start (+ overlap)
};

// ── Module-level state ────────────────────────────────────────────────────────

static float       g_start_x;          ///< Retrace segment start ENU east (m)
static float       g_start_y;          ///< Retrace segment start ENU north (m)
static float       g_end_x;            ///< Retrace segment end ENU east (m)
static float       g_end_y;            ///< Retrace segment end ENU north (m)
static float       g_heading;          ///< Strip heading at entry (rad, EKF convention)
static float       g_desired_mm;       ///< Operator-commanded cut height to restore (mm)
static float       g_retrace_mm;       ///< Deck height for the current pass (mm)
static int         g_pass_count;       ///< Number of completed passes so far
static RetraceDir  g_direction;        ///< Current pass direction (FORWARD or BACKWARD)

/** Length of the retrace segment from start to end (metres), computed at entry */
static float       g_base_length_m;

// ── Internal helpers ──────────────────────────────────────────────────────────

/**
 * @brief Stop both drive motors (zero current).
 */
static void stop_drive() {
    vesc_set_current(VESC_ID_LEFT,  0);
    vesc_set_current(VESC_ID_RIGHT, 0);
}

/**
 * @brief Convert wheel surface velocity to VESC current command (mA).
 *
 * Linear mapping: I = (v / MAX_MOWING_SPEED_MS) × MAX_CURRENT_A
 * Clamped to ±MAX_CURRENT_A before conversion to milliamps.
 *
 * @param v_ms Desired wheel surface speed (m/s, signed).
 * @return Signed motor current in milliamps.
 */
static int32_t vel_to_current_mA(float v_ms) {
    float I = (v_ms / MAX_MOWING_SPEED_MS) * MAX_CURRENT_A;
    if (I >  MAX_CURRENT_A) I =  MAX_CURRENT_A;
    if (I < -MAX_CURRENT_A) I = -MAX_CURRENT_A;
    return (int32_t)(I * 1000.0f);
}

/**
 * @brief Issue proportional heading-correction drive commands.
 *
 * For FORWARD passes, commands positive wheel speeds toward retrace_end.
 * For BACKWARD passes, commands negative wheel speeds toward retrace_start.
 *
 * The heading error is:
 *   heading_error = wrap(retrace_heading − pose.heading)   [rad, CW+]
 *
 * Forward differential (positive v):
 *   v_left  = V × (1 − heading_error × 0.5)
 *   v_right = V × (1 + heading_error × 0.5)
 *
 * Backward differential (negative v, correction sign flipped so that
 * the rear of the robot — which leads during reverse motion — tracks
 * the strip line correctly):
 *   v_left  = −V × (1 + heading_error × 0.5)
 *   v_right = −V × (1 − heading_error × 0.5)
 *
 * Both wheel speeds are clamped to [−MAX_WHEEL_SPEED_MS, +MAX_WHEEL_SPEED_MS]
 * before conversion to milliamps.
 *
 * @param pose Current EKF pose.
 */
static void drive(const Pose2D &pose) {
    float heading_error = wrapAngle(g_heading - pose.heading);
    float v_left, v_right;

    if (g_direction == RDIR_FORWARD) {
        v_left  = LONG_GRASS_SPEED_MS * (1.0f - heading_error * 0.5f);
        v_right = LONG_GRASS_SPEED_MS * (1.0f + heading_error * 0.5f);
    } else {
        // Backward: flip differential correction so the robot's rear tracks the line
        v_left  = -LONG_GRASS_SPEED_MS * (1.0f + heading_error * 0.5f);
        v_right = -LONG_GRASS_SPEED_MS * (1.0f - heading_error * 0.5f);
    }

    // Clamp to safe wheel speed range
    if (v_left  >  MAX_WHEEL_SPEED_MS) v_left  =  MAX_WHEEL_SPEED_MS;
    if (v_left  < -MAX_WHEEL_SPEED_MS) v_left  = -MAX_WHEEL_SPEED_MS;
    if (v_right >  MAX_WHEEL_SPEED_MS) v_right =  MAX_WHEEL_SPEED_MS;
    if (v_right < -MAX_WHEEL_SPEED_MS) v_right = -MAX_WHEEL_SPEED_MS;

    vesc_set_current(VESC_ID_LEFT,  vel_to_current_mA(v_left));
    vesc_set_current(VESC_ID_RIGHT, vel_to_current_mA(v_right));
}

/**
 * @brief Compute signed progress along the retrace heading from the start point.
 *
 * Projects (pose − retrace_start) onto the retrace heading unit vector:
 *   progress = (pose.x − start_x) × cos(heading)
 *            + (pose.y − start_y) × sin(heading)
 *
 * Positive = forward from start toward end.
 * Negative = behind start (achieved on BACKWARD pass overlap extension).
 *
 * Uses standard maths convention for cos/sin (consistent with bog_recovery).
 *
 * @param pose Current EKF pose.
 * @return Signed forward distance from retrace_start (metres).
 */
static float progress(const Pose2D &pose) {
    return (pose.x - g_start_x) * cosf(g_heading)
         + (pose.y - g_start_y) * sinf(g_heading);
}

// ── Public API ────────────────────────────────────────────────────────────────

void retrace_enter(const Pose2D &pose,
                   float current_strip_end_x, float current_strip_end_y,
                   float desired_height_mm) {
    g_start_x   = pose.x;
    g_start_y   = pose.y;
    g_end_x     = current_strip_end_x;
    g_end_y     = current_strip_end_y;
    g_heading   = pose.heading;
    g_desired_mm = desired_height_mm;
    g_retrace_mm = (float)CUT_HEIGHT_MAX_MM;
    g_pass_count = 0;
    g_direction  = RDIR_FORWARD;

    // Precompute base segment length (start → end, not extended)
    float dx = g_end_x - g_start_x;
    float dy = g_end_y - g_start_y;
    g_base_length_m = sqrtf(dx * dx + dy * dy);

    // Raise deck to maximum height immediately for first pass
    servo_set_height_mm((float)CUT_HEIGHT_MAX_MM);
}

RetraceResult retrace_update(const Pose2D &pose, float current_speed,
                              CuttingStatus status) {
    (void)current_speed;  // Not required by this module

    // ── Fault checks (immediate returns) ────────────────────────────────────

    if (status == BLADE_FAULT) {
        stop_drive();
        return RETRACE_BLADE_FAULT;
    }
    if (status == CUTTING_STALLED) {
        stop_drive();
        return RETRACE_STALLED;
    }

    // ── Compute progress and segment boundaries ──────────────────────────────

    // Progress > 0: forward from start; negative: behind start.
    // Segment boundaries (extended by RETRACE_OVERLAP_M at each end):
    //   FORWARD  complete when progress >= base_length + RETRACE_OVERLAP_M
    //   BACKWARD complete when progress <= -RETRACE_OVERLAP_M
    const float segment_length = g_base_length_m + RETRACE_OVERLAP_M;
    float prog = progress(pose);

    bool pass_complete = false;
    if (g_direction == RDIR_FORWARD  && prog >= segment_length)  pass_complete = true;
    if (g_direction == RDIR_BACKWARD && prog <= -RETRACE_OVERLAP_M) pass_complete = true;

    // ── Handle pass completion ────────────────────────────────────────────────

    if (pass_complete) {
        stop_drive();
        g_pass_count++;

        // Check retry budget before lowering the deck further
        if (g_pass_count >= BOG_MAX_RETRIES && g_retrace_mm > g_desired_mm) {
            // Max passes exhausted and still above desired height — give up
            DBG_PRINTF("[RETRACE] Strip segment marked DIFFICULT at (%.2f, %.2f)\n",
                          g_start_x, g_start_y);
            servo_set_height_mm(g_desired_mm);
            coverage_planner_reset_to_nearest(pose.x, pose.y);
            return RETRACE_GAVE_UP;
        }

        if (g_retrace_mm > g_desired_mm) {
            // Step deck height down toward desired height, reverse direction
            g_retrace_mm -= (float)BOG_HEIGHT_STEP_MM;
            if (g_retrace_mm < g_desired_mm) {
                g_retrace_mm = g_desired_mm;
            }
            servo_set_height_mm(g_retrace_mm);
            g_direction = (g_direction == RDIR_FORWARD) ? RDIR_BACKWARD : RDIR_FORWARD;
            // Fall through to drive commands for the new pass direction
        } else {
            // Deck is at desired height and pass completed cleanly — success
            servo_set_height_mm(g_desired_mm);
            // Use current pose, not saved strip-end coords: robot may have drifted
            // during multi-pass retrace. (BUG-1 fix — g_end_x/y would be wrong.)
            coverage_planner_reset_to_nearest(pose.x, pose.y);
            return RETRACE_COMPLETE;
        }
    }

    // ── Issue drive commands for the current pass ────────────────────────────

    drive(pose);
    return RETRACE_IN_PROGRESS;
}

void retrace_exit() {
    stop_drive();
}
