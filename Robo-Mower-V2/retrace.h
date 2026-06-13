#pragma once
#include <Arduino.h>
#include "ekf_localiser.h"
#include "cutting_monitor.h"
#include "coverage_planner.h"

// ══════════════════════════════════════════════════════════════════════════════
//  retrace.h — RoboMower RETRACE overload-recovery procedure
//
//  Entered from the RETRACE state when CuttingStatus == CUTTING_OVERLOADED.
//  Strategy: raise the cutting deck to CUT_HEIGHT_MAX_MM and make repeated
//  passes back and forth along the current strip, stepping the deck height
//  down by BOG_HEIGHT_STEP_MM after each pass until the desired operator-
//  commanded cut height is reached without triggering overload.
//
//  If a stall occurs during retracing, escalates immediately to BOG_RECOVERY
//  by returning RETRACE_STALLED.  If BOG_MAX_RETRIES passes are exhausted
//  before reaching the desired height, the segment is logged as DIFFICULT
//  and RETRACE_GAVE_UP is returned so AUTO_MOWING can continue elsewhere.
//
//  Call sequence:
//    retrace_enter(pose, strip_end_x, strip_end_y, desired_height_mm)
//    while (true) {
//        RetraceResult r = retrace_update(pose, speed, status);  // at 10 Hz
//        if (r != RETRACE_IN_PROGRESS) break;
//    }
//    retrace_exit()
//
//  Source references:
//    Spec:   Robo_Mower_claudecode_prompt_v3.md §"RETRACE state"
//    Config: Robo-Mower-V2/config.h (BOG_*, CUT_HEIGHT_*, LONG_GRASS_SPEED_MS,
//            RETRACE_OVERLAP_M)
//    See:    HANDOFFS/19_retrace/HANDOFF.md for design decisions
// ══════════════════════════════════════════════════════════════════════════════


/** Outcome of a retrace update tick */
enum RetraceResult {
    RETRACE_IN_PROGRESS, ///< Still retracing — continue calling retrace_update()
    RETRACE_COMPLETE,    ///< Clean pass at desired height — resume AUTO_MOWING
    RETRACE_STALLED,     ///< Stall during retrace — go to BOG_RECOVERY
    RETRACE_BLADE_FAULT, ///< BLADE_FAULT detected — go to STATE_PAUSED (event latch)
    RETRACE_GAVE_UP,     ///< BOG_MAX_RETRIES exceeded — strip marked DIFFICULT, resume AUTO
};


/**
 * @brief Call when entering the RETRACE state (CUTTING_OVERLOADED detected).
 *
 * Records the retrace geometry from the current pose and the current strip's
 * end waypoint.  Immediately raises the cutting deck to CUT_HEIGHT_MAX_MM
 * so the first retrace pass has maximum ground clearance.
 *
 * State initialised on entry:
 *   - retrace_start = current pose (x, y)
 *   - retrace_heading = current pose heading
 *   - retrace_end = (current_strip_end_x, current_strip_end_y)
 *   - retrace_height = CUT_HEIGHT_MAX_MM (deck raised immediately)
 *   - retrace_count = 0
 *   - direction = FORWARD (toward retrace_end)
 *
 * @param pose                Current EKF pose (steering centre in ENU metres).
 * @param current_strip_end_x End of current strip, ENU east (m).
 * @param current_strip_end_y End of current strip, ENU north (m).
 * @param desired_height_mm   Operator-commanded cut height to restore (mm).
 */
void retrace_enter(const Pose2D &pose,
                   float current_strip_end_x, float current_strip_end_y,
                   float desired_height_mm);


/**
 * @brief Update function — call at 10 Hz while in the RETRACE state.
 *
 * Runs one tick of the non-blocking retrace state machine.  Each call:
 *   1. Checks for BLADE_FAULT and CUTTING_STALLED — immediate returns.
 *   2. Computes progress along the retrace segment.
 *   3. Detects pass completion (reached extended end of segment).
 *   4. On pass complete: increments retrace_count, checks retry budget,
 *      lowers deck by BOG_HEIGHT_STEP_MM, reverses direction.
 *   5. Issues proportional heading-correction drive commands toward the
 *      active end of the retrace segment (extended by RETRACE_OVERLAP_M).
 *
 * Pass completion uses a signed-projection criterion (see HANDOFF.md).
 * Drive speed is always LONG_GRASS_SPEED_MS.  No delay() is used.
 *
 * @param pose          Current EKF pose (steering centre in ENU metres).
 * @param current_speed Current EKF forward speed (m/s).
 * @param status        Current cutting status from cutting_monitor_get_status().
 * @return RetraceResult indicating whether retracing is ongoing or complete.
 */
RetraceResult retrace_update(const Pose2D &pose, float current_speed,
                              CuttingStatus status);


/**
 * @brief Call when leaving the RETRACE state (regardless of outcome).
 *
 * Stops both drive motors by commanding zero current to VESC_ID_LEFT and
 * VESC_ID_RIGHT.  Should be called by the state machine whenever RETRACE
 * is exited — whether via RETRACE_COMPLETE, RETRACE_STALLED,
 * RETRACE_BLADE_FAULT, RETRACE_GAVE_UP, or an operator override.
 */
void retrace_exit();
