#pragma once
#include <Arduino.h>
#include "geometry.h"
#include "ekf_localiser.h"
#include "cutting_monitor.h"

// ══════════════════════════════════════════════════════════════════════════════
//  safety.h — RoboMower Safety Watchdog Module
//
//  Runs a FreeRTOS task on Core 1 at priority 20 (highest application level,
//  per ARCHITECTURE.md §2). The task fires every 50 ms and checks:
//    1. VESC CAN silence  — PILZ fired or battery disconnected → MOTORS_OFFLINE
//    2. IMU fault         — log warning; EKF continues GPS-only
//    3. Perimeter breach  — robot outside the PERIMETER → requests STATE_PAUSED
//    4. Battery state     — sets LED-overlay warning flag
//    5. GPS timeout       — log warning only
//
//  Physical E-stop: PILZ safety relay, triggered by a dedicated button on the
//  mower. The ESP32 has NO GPIO connected to the PILZ. The RC transmitter has a
//  dedicated pause channel that maps to the software STATE_PAUSED.
//
//  Software pause (perimeter breach):
//    Sets s_pause_requested flag polled by state_machine_update().
//    The state machine calls vesc_emergency_stop_all() + transitions to
//    STATE_PAUSED with an event latch (operator must acknowledge via pause switch).
//
//  References:
//    Architecture: ARCHITECTURE.md §2 (task priorities), §8 (safety arch)
//    Config:       Robo-Mower-V2/config.h (VESC_STATUS_TIMEOUT_MS,
//                  PERIMETER_BREACH_DIST_M, BATTERY_LOW_V, CH6_ARMED_THRESHOLD)
// ══════════════════════════════════════════════════════════════════════════════


// ── Lifecycle ─────────────────────────────────────────────────────────────────

/**
 * @brief Initialise safety module and start the watchdog task.
 *
 * Creates the nav-boundary mutex and the FreeRTOS "safety_task" pinned to
 * Core 1 at priority 20 (ARCHITECTURE.md §2), 4096-byte stack, 50 ms period.
 * Must be called from setup() after vesc_can_init(), crsf_input_init(),
 * imu_bmi270_init(), and servo_output_init() have been called.
 */
void safety_init();


// ── Arm status ────────────────────────────────────────────────────────────────

/**
 * @brief Returns true when the system is armed.
 *
 * Armed when ALL of:
 *   - CRSF failsafe is NOT active.
 *   - CH6 (arm switch) is above CH6_ARMED_THRESHOLD (496 raw CRSF ≈ 1198 µs).
 *
 * Note: CRSFChannels.ch[] stores values in microseconds (1000–2000 µs),
 * so the comparison converts the raw threshold via crsf_raw_to_us().
 * See HANDOFFS/05_crsf_input/HANDOFF.md §1.
 */
bool safety_is_armed();


// ── Perimeter and mode control ────────────────────────────────────────────────

/**
 * @brief Update the PERIMETER polygon used for breach detection.
 *
 * Copies the polygon into the safety module. Breach is measured against the
 * perimeter (not the inset nav boundary) so gardens that pinch into multiple
 * regions — where the largest-region nav boundary would exclude a side arm —
 * are handled correctly. The breach threshold accounts for the nav inset
 * internally. Call once after the perimeter is loaded/recorded.
 *
 * Thread-safe: protected by an internal FreeRTOS mutex.
 *
 * @param perimeter  The recorded perimeter polygon.
 */
void safety_set_perimeter(const Polygon &perimeter);

/**
 * @brief Enable or disable perimeter breach checking.
 *
 * @param in_auto  true = enable breach checking (call when entering AUTO states).
 *                 false = disable (call when leaving AUTO states).
 *
 * Thread-safe: writes a volatile bool (atomic on Xtensa LX7).
 */
void safety_set_auto_mode(bool in_auto);


// ── Battery warning ───────────────────────────────────────────────────────────

/**
 * @brief Returns true if the battery warning flag is set.
 *
 * The flag is set by the safety watchdog when battery_get_state() returns
 * BATTERY_WARNING (voltage < BATTERY_WARN_V but ≥ BATTERY_LOW_V).
 * A warning does NOT trigger an E-stop — only BATTERY_LOW does.
 *
 * The state machine should use this flag to overlay a fast-flash amber
 * LED pattern on top of the normal state LED, per ASSUMPTIONS.md A23.
 *
 * Thread-safe: reads a volatile bool.
 */
bool safety_is_battery_warning();


// ── Motors-offline detection ──────────────────────────────────────────────────

/**
 * @brief Returns true while the motors-offline condition is active.
 *
 * Set by the safety watchdog when any VESC CAN STATUS frame is overdue.
 * This indicates the PILZ has fired or the main battery has been disconnected.
 * Unlike a software E-stop, no CAN commands are sent — the VESCs are unpowered.
 *
 * Cleared by safety_clear_motors_offline() when the state machine confirms
 * the drive VESCs have come back online (vesc_all_drive_online() == true).
 *
 * Thread-safe: reads a volatile bool.
 */
bool safety_is_motors_offline();

/**
 * @brief Clear the motors-offline flag.
 *
 * Called by the state machine when transitioning out of STATE_MOTORS_OFFLINE
 * (after vesc_all_drive_online() has returned true).
 *
 * Thread-safe: writes a volatile bool (atomic on Xtensa LX7).
 */
void safety_clear_motors_offline();


// ── Pause request (perimeter breach) ─────────────────────────────────────────

/**
 * @brief Returns true if the safety task has requested a pause.
 *
 * Set by the safety watchdog on perimeter breach.  Polled by
 * state_machine_update() in the global-checks section each tick.
 * The state machine is responsible for acting on this flag and clearing it
 * via safety_clear_pause_request().
 *
 * Thread-safe: reads a volatile bool.
 */
bool safety_is_pause_requested();

/**
 * @brief Clear the pause-request flag after the state machine has handled it.
 *
 * Thread-safe: writes a volatile bool.
 */
void safety_clear_pause_request();


// ── Blade VESC stale ──────────────────────────────────────────────────────────

/**
 * @brief Returns true when the blade VESC (CAN ID 3) has been silent for
 *        BLADE_VESC_TIMEOUT_MS milliseconds.
 *
 * The state machine should respond by stopping the blade motor only.
 * Drive VESCs and all other controls are NOT affected.
 *
 * Thread-safe: reads a volatile bool.
 */
bool safety_is_blade_vesc_stale();


