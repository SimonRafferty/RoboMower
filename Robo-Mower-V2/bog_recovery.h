#pragma once
#include <Arduino.h>
#include "ekf_localiser.h"
#include "cutting_monitor.h"
#include "coverage_planner.h"

// ══════════════════════════════════════════════════════════════════════════════
//  bog_recovery.h — RoboMower stall / bog recovery procedure
//
//  Entered from BOG_RECOVERY state when CuttingStatus == CUTTING_STALLED.
//  Strategy: raise the cutting deck to its maximum height, make a short
//  forward pass, then step the deck height back down pass-by-pass until the
//  desired cut height is reached without stalling.
//
//  Call sequence:
//    bog_recovery_enter(pose, desired_height_mm)   ← on entering BOG_RECOVERY
//    while (true) {
//        BogResult r = bog_recovery_update(pose, speed, status);  // at 10 Hz
//        if (r != BOG_IN_PROGRESS) break;
//    }
//    bog_recovery_exit()                           ← on leaving BOG_RECOVERY
//
//  Source references:
//    Spec:         Robo_Mower_claudecode_prompt_v3.md §"Bog Recovery"
//    Config:       Robo-Mower-V2/config.h (BOG_*, CUT_HEIGHT_*, LONG_GRASS_SPEED_MS)
//    Dependencies: servo_output.h, vesc_can.h, coverage_planner.h
// ══════════════════════════════════════════════════════════════════════════════


/** Outcome of a bog recovery attempt */
enum BogResult {
    BOG_IN_PROGRESS,    ///< Still recovering — continue calling bog_recovery_update()
    BOG_RECOVERED,      ///< Recovery complete — resume AUTO_MOWING
    BOG_GAVE_UP,        ///< Max retries exceeded — obstacle logged, resume AUTO_MOWING
    BOG_BLADE_FAULT,    ///< BLADE_FAULT during recovery — go to STATE_PAUSED (event latch)
};


/**
 * @brief Call when entering BOG_RECOVERY state.
 *
 * Records the current steering-centre position and heading as the bog entry
 * point.  Immediately raises the cutting deck to CUT_HEIGHT_MAX_MM so the
 * first forward pass has maximum ground clearance, then begins the
 * BOG_PASS_DWELL_MS entry dwell before the first pass attempt.
 *
 * @param pose               Current EKF pose (steering centre in ENU metres).
 * @param desired_height_mm  Target cut height to restore after successful recovery
 *                           (the CH3 operator-commanded height, in mm).
 */
void bog_recovery_enter(const Pose2D &pose, float desired_height_mm);


/**
 * @brief Update function — call at 10 Hz while in BOG_RECOVERY state.
 *
 * Runs one tick of the non-blocking recovery state machine:
 *
 *   DWELL_ENTRY  → wait BOG_PASS_DWELL_MS after entry
 *   LOWERING     → set deck to current_height for this retry, begin pass
 *   FORWARD_PASS → drive forward 1.5 × CUT_WIDTH_M along bog_heading,
 *                  monitoring CuttingStatus each tick
 *   DWELL_BETWEEN→ wait BOG_PASS_DWELL_MS between consecutive passes
 *
 * Height stepping: starts at CUT_HEIGHT_MAX_MM and decreases by
 * BOG_HEIGHT_STEP_MM per successful pass, stopping at desired_height_mm.
 *
 * Stall retry: if CUTTING_STALLED recurs, retry_count is incremented.
 * After BOG_MAX_RETRIES stalls the position is reported as an obstacle and
 * BOG_GAVE_UP is returned.
 *
 * @param pose          Current EKF pose (steering centre in ENU metres).
 * @param current_speed Current EKF speed estimate (m/s); not used directly
 *                      by this module but provided for completeness.
 * @param status        Current cutting status from cutting_monitor_get_status().
 * @return BogResult indicating whether recovery is ongoing or complete.
 */
BogResult bog_recovery_update(const Pose2D &pose, float current_speed,
                               CuttingStatus status);


/**
 * @brief Call when leaving BOG_RECOVERY state (regardless of outcome).
 *
 * Stops both drive motors.  Should be called by the state machine whenever
 * BOG_RECOVERY is exited — whether via BOG_RECOVERED, BOG_GAVE_UP,
 * BOG_BLADE_FAULT, or an operator override.
 */
void bog_recovery_exit();
