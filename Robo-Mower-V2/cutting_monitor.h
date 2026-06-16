#pragma once
#include <Arduino.h>

// ══════════════════════════════════════════════════════════════════════════════
//  cutting_monitor.h — RoboMower Blade Load & Cutting Condition Monitor
//
//  Assesses blade loading at 10 Hz using a rolling average of voltage-
//  compensated blade current.  Classifies cutting conditions and manages
//  auto-calibration of the blade reference current via NVS.
//
//  Must be called from the state-machine task (Core 1) at exactly 10 Hz.
//
//  Thread safety:
//    cutting_monitor_get_status() and the other getters are protected by
//    an internal portMUX_TYPE spinlock so they may be read from Core 0
//    (e.g. safety task) without a dedicated mutex overhead.
//
//  References:
//    Spec:         Robo_Mower_claudecode_prompt_v3.md §"Cutting Condition Assessment"
//    Assumptions:  ASSUMPTIONS.md A03 (blade constants), A23 (voltage compensation)
//    Config:       Robo-Mower-V2/config.h (BLADE_LOAD_*, BLADE_CAL_*, BATTERY_LOW_V)
//    NVS:          nvs_storage.h  (nvs_get_float / nvs_set_float, key "blade_cal")
// ══════════════════════════════════════════════════════════════════════════════


// ── Cutting condition classification ─────────────────────────────────────────

/**
 * @brief Cutting condition, as assessed by cutting_monitor_update().
 *
 * The confirmed status returned by cutting_monitor_get_status() uses
 * hysteresis timers before promoting transient assessments:
 *   - CUTTING_OVERLOADED: confirmed after BLADE_LOAD_SAMPLE_WINDOW_MS (500 ms)
 *   - OBSTACLE_SUSPECTED: confirmed after OBSTACLE_CONFIRM_MS (500 ms)
 *   - CUTTING_STALLED:    immediate (no confirmation delay)
 *   - BLADE_FAULT:        immediate (no confirmation delay)
 */
enum CuttingStatus {
    CUTTING_NORMAL,       ///< Moving, blade load within bounds
    CUTTING_OVERLOADED,   ///< Moving, blade load HIGH (RETRACE — currently gated off by AUTO_FAULT_RESPONSES_ENABLED)
    CUTTING_STALLED,      ///< Not moving, blade load HIGH (BOG_RECOVERY — currently gated off by AUTO_FAULT_RESPONSES_ENABLED)
    OBSTACLE_SUSPECTED,   ///< Not moving, blade load normal (OBSTACLE_AVOID — currently gated off by AUTO_FAULT_RESPONSES_ENABLED)
    BLADE_FAULT,          ///< Blade current near zero when commanded (no longer E-stops; blade is RC-only — warns and drives on)
};


// ── Lifecycle ─────────────────────────────────────────────────────────────────

/**
 * @brief Initialise cutting monitor.
 *
 * Loads the auto-calibrated blade reference current from NVS key "blade_cal".
 * Falls back to BLADE_MAX_EXPECTED_CURRENT_A_DEFAULT (12.0 A) if absent.
 * Resets the rolling average buffer and all hysteresis timers.
 *
 * Must be called once from setup() before cutting_monitor_update() is used.
 */
void cutting_monitor_init();


// ── Per-cycle update ──────────────────────────────────────────────────────────

/**
 * @brief Update monitor — call at 10 Hz from the state-machine task (Core 1).
 *
 * Steps performed each call:
 *   1. Apply voltage compensation to blade_current_A (see ASSUMPTIONS.md A23).
 *   2. Push compensated current into the circular rolling-average buffer.
 *   3. Detect BLADE_FAULT immediately (no rolling-average delay).
 *   4. Compute raw CuttingStatus from rolling average and fused velocity.
 *   5. Apply hysteresis timers for CUTTING_OVERLOADED and OBSTACLE_SUSPECTED.
 *   6. Update auto-calibration state machine if calibration is active.
 *   7. Store confirmed CuttingStatus atomically.
 *
 * @param fused_velocity      EKF speed (m/s); sign not used — magnitude only.
 * @param blade_current_A     Blade VESC phase current (A) from VescStatus.current_A.
 * @param blade_actual_erpm   Actual blade ERPM from VescStatus.erpm.
 * @param blade_target_erpm   Target blade ERPM (BLADE_TARGET_RPM × BLADE_MOTOR_POLE_PAIRS).
 * @param blade_commanded     True when the blade VESC has been issued a non-zero RPM target.
 * @param battery_voltage_V   Battery voltage from ADC (V); used for load compensation.
 */
void cutting_monitor_update(float fused_velocity,
                            float blade_current_A,
                            float blade_actual_erpm,
                            float blade_target_erpm,
                            bool  blade_commanded,
                            float battery_voltage_V);


// ── Getters (thread-safe) ─────────────────────────────────────────────────────

/**
 * @brief Get current confirmed cutting status.
 *
 * Thread-safe: reads atomically via internal spinlock.
 * Hysteresis rules applied:
 *   - CUTTING_OVERLOADED promoted only after BLADE_LOAD_SAMPLE_WINDOW_MS.
 *   - OBSTACLE_SUSPECTED promoted only after OBSTACLE_CONFIRM_MS.
 *   - CUTTING_STALLED and BLADE_FAULT take effect immediately.
 *
 * @return Confirmed CuttingStatus.
 */
CuttingStatus cutting_monitor_get_status();

/**
 * @brief Get rolling average blade current (A).
 *
 * Returns the average of up to ROLLING_BUF_SIZE (8) recent compensated
 * current samples, normalised back to raw amperes.  The value updates at
 * 10 Hz.
 *
 * Thread-safe via internal spinlock.
 *
 * @return Rolling average current in amperes (voltage-compensated basis).
 */
float cutting_monitor_get_avg_current();

/**
 * @brief Get blade load fraction (0.0 – 1.0+).
 *
 * Computed as:  rolling_avg_compensated_current / BLADE_CURRENT_LIMIT_A (fixed 15 A);
 * g_blade_max_A is no longer used for the load fraction.
 *
 * Values above 1.0 are possible if current exceeds BLADE_CURRENT_LIMIT_A.
 * BLADE_LOAD_HIGH threshold (0.75) and BLADE_LOAD_MIN (0.05) are applied
 * against this fraction.
 *
 * Thread-safe via internal spinlock.
 *
 * @return Load fraction; 0.0 when blade is off, ≥1.0 when overloaded.
 */
float cutting_monitor_get_load_fraction();

/**
 * @brief Get RPM-based blade load fraction (0.0 – 1.0).  [Feature 2, 2026-06-16]
 *
 * Derived from blade RPM droop, EMA-smoothed:
 *     rpm_load = clamp((target_erpm - |actual_erpm|) / target_erpm, 0, 1)
 * = 0.0 at/above the target RPM (free spin), 1.0 at standstill. Because the blade
 * VESC is current-limited, RPM is a truer load proxy than current. This is the
 * value shown on the TX16S MOWER_STATUS load byte and the PWA bladeLoad bar.
 *
 * Reads 0.0 whenever the blade is not commanded or still within the spin-up grace
 * (BLADE_FAULT_GRACE_MS) — so a stopped blade never reads as fully loaded.
 *
 * Thread-safe via internal spinlock.
 *
 * @return RPM load fraction; 0.0 when the blade is off / spinning up.
 */
float cutting_monitor_get_rpm_load_fraction();


// ── Auto-calibration ──────────────────────────────────────────────────────────

/**
 * @brief Start the auto-calibration sequence.
 *
 * Call when entering AUTO_MOWING.  Clears the calibration sample buffer and
 * begins collecting blade current samples over BLADE_CAL_WARMUP_MS (8 s).
 * After the warmup period (and at least 10 samples), computes the P90
 * percentile and updates g_blade_max_A if the result exceeds
 * BLADE_CAL_MIN_VALID_A (2.0 A).  The calibrated value is saved to NVS.
 *
 * Calling again while calibration is still active restarts the sequence.
 */
void cutting_monitor_start_auto_cal();

/**
 * @brief Query whether auto-calibration has completed.
 *
 * Returns true after the warmup window has elapsed and the P90 result has
 * been evaluated (pass or fail).  Resets to false when
 * cutting_monitor_start_auto_cal() is called.
 *
 * @return true  Calibration is complete (not necessarily successful).
 * @return false Calibration has not been started or is still in progress.
 */
bool cutting_monitor_is_cal_complete();

/**
 * @brief Immediately write the current blade reference current to NVS.
 *
 * Normally the auto-cal sequence writes to NVS at the end of the warmup window.
 * This function forces an immediate save — call it on MOTORS_OFFLINE entry so
 * the most recent calibration is not lost during a battery swap.
 *
 * If no valid calibration has been collected yet (g_blade_max_A is still the
 * factory default), the write is skipped to avoid overwriting a previously
 * saved good calibration with the default value.
 *
 * Thread-safe: acquires the internal spinlock briefly.
 */
void cutting_monitor_force_save_cal();

/**
 * @brief Get the current reference current used for load calculations (A).
 *
 * Returns the runtime value of g_blade_max_A, which is either:
 *   - BLADE_MAX_EXPECTED_CURRENT_A_DEFAULT (12.0 A) if no NVS value exists, or
 *   - The most recently auto-calibrated value stored in NVS.
 *
 * @return Reference current in amperes.
 */
float cutting_monitor_get_cal_current();
