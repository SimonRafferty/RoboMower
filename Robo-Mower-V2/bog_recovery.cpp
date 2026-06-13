// ══════════════════════════════════════════════════════════════════════════════
//  bog_recovery.cpp — RoboMower stall / bog recovery procedure
//
//  Implements a non-blocking state machine called at 10 Hz from the
//  BOG_RECOVERY state in state_machine.cpp.
//
//  State sequence:
//    DWELL_ENTRY  — initial pause to let the deck rise to CUT_HEIGHT_MAX_MM
//    LOWERING     — set deck height for this pass, set up pass geometry
//    FORWARD_PASS — open-loop forward drive along bog_heading, monitor status
//    DWELL_BETWEEN— pause between consecutive passes (retry or step-down)
//
//  All timing uses millis() — no vTaskDelay() — so the update function is
//  non-blocking and safe to call from the state-machine task at 10 Hz.
//
//  Source references:
//    Spec:   Robo_Mower_claudecode_prompt_v3.md §"Bog Recovery — bog_recovery.cpp"
//    Config: Robo-Mower-V2/config.h
// ══════════════════════════════════════════════════════════════════════════════

#include "bog_recovery.h"
#include "config.h"
#include "mower_config.h"
#include "servo_output.h"
#include "vesc_can.h"
#include "geometry.h"
#include <math.h>

// ── Internal state machine states ─────────────────────────────────────────────

/** Internal states of the bog recovery state machine */
enum BogState {
    BS_DWELL_ENTRY,    ///< Initial dwell after bog entry (deck rising to max)
    BS_LOWERING,       ///< Set deck height for this pass, transition to FORWARD_PASS
    BS_FORWARD_PASS,   ///< Drive forward, checking CuttingStatus each tick
    BS_DWELL_BETWEEN,  ///< Pause between passes (retry after stall or step-down)
};

// ── Module-level state ────────────────────────────────────────────────────────

static float    g_bog_x;              ///< Steering-centre ENU east at bog entry (m)
static float    g_bog_y;              ///< Steering-centre ENU north at bog entry (m)
static float    g_bog_heading;        ///< Robot heading at bog entry (rad)
static float    g_desired_height_mm;  ///< Operator-commanded cut height to restore (mm)
static float    g_current_height;     ///< Deck height commanded for the current pass (mm)
static int      g_retry_count;        ///< Number of retries / step-downs attempted so far
static BogState g_state;              ///< Current internal state
static uint32_t g_dwell_start_ms;     ///< millis() timestamp when the current dwell began

// ── Internal helpers ──────────────────────────────────────────────────────────

/**
 * @brief Convert a wheel surface velocity to a VESC current command (mA).
 *
 * Linear mapping: I = (v / MAX_MOWING_SPEED_MS) × MAX_CURRENT_A
 * Clamped to ±MAX_CURRENT_A before conversion to milliamps.
 *
 * @param v_ms Desired wheel surface speed (m/s, must be >= 0 for forward).
 * @return Signed motor current in milliamps.
 */
static int32_t vel_to_current_mA(float v_ms) {
    float I = (v_ms / MAX_MOWING_SPEED_MS) * MAX_CURRENT_A;
    if (I >  MAX_CURRENT_A) I =  MAX_CURRENT_A;
    if (I < -MAX_CURRENT_A) I = -MAX_CURRENT_A;
    return (int32_t)(I * 1000.0f);
}

/**
 * @brief Command both drive motors to zero current.
 */
static void stop_drive() {
    vesc_set_current(VESC_ID_LEFT,  0);
    vesc_set_current(VESC_ID_RIGHT, 0);
}

/**
 * @brief Issue proportional-heading-correction forward drive commands.
 *
 * Drives forward along g_bog_heading at LONG_GRASS_SPEED_MS, with a
 * proportional correction based on the error between the target heading
 * and the current EKF heading:
 *
 *   heading_error = wrap(bog_heading − pose.heading)   [rad, CW positive]
 *   v_left  = LONG_GRASS_SPEED_MS − (heading_error × TRACK_WIDTH_M × 0.5)
 *   v_right = LONG_GRASS_SPEED_MS + (heading_error × TRACK_WIDTH_M × 0.5)
 *
 * Both wheel speeds are clamped to [MIN_CREEP_SPEED_MS, LONG_GRASS_SPEED_MS]
 * so neither wheel stalls or overshoots during the correction.
 *
 * @param pose Current EKF pose (heading in rad, 0=North, CW positive).
 */
static void drive_forward(const Pose2D &pose) {
    // Heading error: positive when we need to turn right (CW) to reach bog_heading
    float heading_error = wrapAngle(g_bog_heading - pose.heading);

    float T = mower_config_track_width_m();
    float v_left  = LONG_GRASS_SPEED_MS - (heading_error * T * 0.5f);
    float v_right = LONG_GRASS_SPEED_MS + (heading_error * T * 0.5f);

    // Clamp to safe range — never stop either wheel completely or exceed target
    if (v_left  < MIN_CREEP_SPEED_MS)  v_left  = MIN_CREEP_SPEED_MS;
    if (v_left  > LONG_GRASS_SPEED_MS) v_left  = LONG_GRASS_SPEED_MS;
    if (v_right < MIN_CREEP_SPEED_MS)  v_right = MIN_CREEP_SPEED_MS;
    if (v_right > LONG_GRASS_SPEED_MS) v_right = LONG_GRASS_SPEED_MS;

    vesc_set_current(VESC_ID_LEFT,  vel_to_current_mA(v_left));
    vesc_set_current(VESC_ID_RIGHT, vel_to_current_mA(v_right));
}

/**
 * @brief Compute distance driven along bog_heading from bog entry point.
 *
 * Projects (pose − bog_entry) onto the bog_heading unit vector:
 *   dist = (pose.x − bog_x)·cos(bog_heading) + (pose.y − bog_y)·sin(bog_heading)
 *
 * @param pose Current EKF pose.
 * @return Signed forward distance in metres (positive = forward from bog entry).
 */
static float dist_driven(const Pose2D &pose) {
    return (pose.x - g_bog_x) * cosf(g_bog_heading)
         + (pose.y - g_bog_y) * sinf(g_bog_heading);
}

// ── Public API ────────────────────────────────────────────────────────────────

void bog_recovery_enter(const Pose2D &pose, float desired_height_mm) {
    g_bog_x             = pose.x;
    g_bog_y             = pose.y;
    g_bog_heading       = pose.heading;
    g_desired_height_mm = desired_height_mm;
    g_retry_count       = 0;

    // Raise deck to maximum height immediately so the first pass has clearance
    servo_set_height_mm((float)CUT_HEIGHT_MAX_MM);

    // Begin entry dwell — the deck needs a moment to travel to max height
    g_dwell_start_ms = millis();
    g_state          = BS_DWELL_ENTRY;
}

BogResult bog_recovery_update(const Pose2D &pose, float current_speed,
                               CuttingStatus status) {
    (void)current_speed;  // Available to caller; not required by this module

    switch (g_state) {

        // ── DWELL_ENTRY ──────────────────────────────────────────────────────
        case BS_DWELL_ENTRY:
            if (millis() - g_dwell_start_ms >= (uint32_t)BOG_PASS_DWELL_MS) {
                g_state = BS_LOWERING;
            }
            return BOG_IN_PROGRESS;

        // ── LOWERING ─────────────────────────────────────────────────────────
        case BS_LOWERING: {
            // Height for this retry: start at MAX and step down by BOG_HEIGHT_STEP_MM
            // per retry.  Never go below the operator-requested desired height.
            g_current_height = (float)CUT_HEIGHT_MAX_MM
                               - (float)(g_retry_count * BOG_HEIGHT_STEP_MM);
            if (g_current_height < g_desired_height_mm) {
                g_current_height = g_desired_height_mm;
            }
            servo_set_height_mm(g_current_height);
            g_state = BS_FORWARD_PASS;
            return BOG_IN_PROGRESS;
        }

        // ── FORWARD_PASS ─────────────────────────────────────────────────────
        case BS_FORWARD_PASS: {
            // BLADE_FAULT is an immediate abort — escalates to EMERGENCY_STOP
            if (status == BLADE_FAULT) {
                stop_drive();
                return BOG_BLADE_FAULT;
            }

            // Distance-based pass completion: distance from bog entry projected
            // onto bog_heading exceeds 1.5 × cut width, and we are not stalled.
            const float pass_target_m = 1.5f * mower_config_get().cut_width_m;
            bool pass_complete = (dist_driven(pose) >= pass_target_m)
                                 && (status != CUTTING_STALLED);

            if (status == CUTTING_STALLED) {
                // Stall recurred during this pass
                stop_drive();
                g_retry_count++;
                if (g_retry_count >= BOG_MAX_RETRIES) {
                    // Exhausted retries — mark as obstacle, resume planning around it
                    coverage_planner_report_obstacle(g_bog_x, g_bog_y, g_bog_heading);
                    coverage_planner_reset_to_nearest(g_bog_x, g_bog_y);
                    servo_set_height_mm(g_desired_height_mm);
                    return BOG_GAVE_UP;
                }
                // Dwell, then try again at a lower height (retry_count incremented)
                g_dwell_start_ms = millis();
                g_state = BS_DWELL_BETWEEN;
                return BOG_IN_PROGRESS;
            }

            if (pass_complete || status == CUTTING_NORMAL) {
                // Pass succeeded — deck cleared this height
                stop_drive();
                if (g_current_height > g_desired_height_mm) {
                    // Still above desired height — dwell then step down for next pass
                    g_retry_count++;
                    g_dwell_start_ms = millis();
                    g_state = BS_DWELL_BETWEEN;
                } else {
                    // Reached desired height cleanly — recovery complete
                    servo_set_height_mm(g_desired_height_mm);
                    coverage_planner_reset_to_nearest(g_bog_x, g_bog_y);
                    return BOG_RECOVERED;
                }
                return BOG_IN_PROGRESS;
            }

            // Pass still in progress — continue driving forward
            drive_forward(pose);
            return BOG_IN_PROGRESS;
        }

        // ── DWELL_BETWEEN ────────────────────────────────────────────────────
        case BS_DWELL_BETWEEN:
            if (millis() - g_dwell_start_ms >= (uint32_t)BOG_PASS_DWELL_MS) {
                g_state = BS_LOWERING;
            }
            return BOG_IN_PROGRESS;

    }  // switch

    // Unreachable — belt-and-suspenders for compiler
    return BOG_IN_PROGRESS;
}

void bog_recovery_exit() {
    stop_drive();
}
