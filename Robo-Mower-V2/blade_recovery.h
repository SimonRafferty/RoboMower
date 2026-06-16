#pragma once
#include <Arduino.h>
#include "ekf_localiser.h"   // Pose2D
#include "cutting_monitor.h" // CuttingStatus
#include "geometry.h"        // Polygon

// ══════════════════════════════════════════════════════════════════════════════
//  blade_recovery.h — RoboMower RPM-load recovery procedure (Feature 2, 2026-06-16)
//
//  Hosted in the existing STATE_RETRACE slot (no telemetry/enum churn). Entered
//  when the RPM-based blade load (cutting_monitor_get_rpm_load_fraction()) has
//  stayed high for BLADE_RPM_RECOVERY_CONFIRM_MS — see config.h
//  BLADE_RPM_RECOVERY_* — and ONLY when BLADE_RPM_RECOVERY_ENABLED is 1.
//
//  Strategy (the "hybrid" the operator asked for):
//    1. Raise the deck to CUT_HEIGHT_MAX_MM and reverse up to
//       BLADE_RECOVERY_BACKUP_MAX_M (1 m) at creep speed, CLIPPED so the
//       steering centre never crosses the perimeter behind it.
//    2. Make repeated back-and-forth passes over that short (≤1 m) segment,
//       stepping the deck height DOWN by BOG_HEIGHT_STEP_MM per pass toward the
//       operator-commanded height.
//    3. A clean pass completed at the desired height → BLADE_RECOVERY_COMPLETE.
//       BLADE_RECOVERY_MAX_PASSES exhausted → BLADE_RECOVERY_GAVE_UP (spot logged).
//
//  The BLADE IS LEFT ARMED throughout — this procedure only moves the DECK and
//  the drive wheels; it never toggles the blade VESC. (The blade-toggling of the
//  old, disabled RETRACE/BOG detectors is deliberately not reproduced here.)
//
//  Heading convention: EKF heading (0 = North/+Y, clockwise positive). The
//  forward unit vector is (sin θ, cos θ) — matching ekf_predict(). (The legacy
//  retrace.cpp used (cos θ, sin θ), which projects onto the wrong axis.)
//
//  Call sequence (mirrors retrace/bog):
//    blade_recovery_enter(pose, perimeter, desired_height_mm)
//    while (true) {
//        BladeRecoveryResult r = blade_recovery_update(pose, speed, status); // 10 Hz
//        if (r != BLADE_RECOVERY_IN_PROGRESS) break;
//    }
//    blade_recovery_exit()
//
//  No vTaskDelay()/delay() is used.
// ══════════════════════════════════════════════════════════════════════════════


/** Outcome of a blade-recovery update tick */
enum BladeRecoveryResult {
    BLADE_RECOVERY_IN_PROGRESS,  ///< Still recovering — keep calling blade_recovery_update()
    BLADE_RECOVERY_COMPLETE,     ///< Clean pass at desired height — resume AUTO_MOWING
    BLADE_RECOVERY_GAVE_UP,      ///< Max passes exceeded — spot logged, resume AUTO_MOWING
    BLADE_RECOVERY_BLADE_FAULT,  ///< BLADE_FAULT detected — go to STATE_PAUSED (event latch)
};


/**
 * @brief Call when entering the recovery (RPM load sustained high).
 *
 * Records the entry pose/heading, raises the deck to CUT_HEIGHT_MAX_MM, and
 * computes the perimeter-clipped reverse distance (≤ BLADE_RECOVERY_BACKUP_MAX_M)
 * so the rear of the back-up never crosses the perimeter.
 *
 * @param pose               Current EKF pose (steering centre, ENU metres).
 * @param perimeter          The perimeter polygon (steering-centre limit) used to
 *                           clip the reverse distance.
 * @param desired_height_mm  Operator-commanded cut height to restore (mm).
 */
void blade_recovery_enter(const Pose2D &pose, const Polygon &perimeter,
                          float desired_height_mm);

/**
 * @brief Update — call at 10 Hz while hosting the recovery in STATE_RETRACE.
 *
 * @param pose          Current EKF pose (steering centre, ENU metres).
 * @param current_speed Current EKF forward speed (m/s).
 * @param status        Current cutting status (for BLADE_FAULT detection).
 * @return BladeRecoveryResult for this tick.
 */
BladeRecoveryResult blade_recovery_update(const Pose2D &pose, float current_speed,
                                          CuttingStatus status);

/**
 * @brief Call when leaving the recovery (any outcome). Stops both drive motors.
 */
void blade_recovery_exit();
