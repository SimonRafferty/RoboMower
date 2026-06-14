#pragma once
#include <Arduino.h>

// ══════════════════════════════════════════════════════════════════════════════
//  battery_monitor.h — RoboMower 48V LiPo battery voltage monitor
//
//  Reads battery voltage from VESC CAN STATUS_5 (CAN_PACKET_STATUS_5, ID=27)
//  broadcast by VESC_ID_BLADE (CAN ID 3). Applies an IIR low-pass filter to
//  reject motor-start voltage sags, and classifies into one of three states.
//
//  No ADC or calibration offset required — the VESC measures v_in directly
//  from its power stage. GPIO4 and the resistor divider are no longer used.
//
//  Pack: 13S LiPo nominal. Update BATTERY_WARN_V / BATTERY_LOW_V in config.h
//  if a 14S pack is fitted.
//
//  See ASSUMPTIONS.md A23 for original ADC design; HANDOFFS/28_vesc_battery/
//  HANDOFF.md for migration details.
// ══════════════════════════════════════════════════════════════════════════════


// ─── Battery voltage states ──────────────────────────────────────────────────

/**
 * @brief Battery voltage operating states.
 *
 * Transitions:
 *   OK → WARNING  when filtered voltage falls below BATTERY_WARN_V
 *   WARNING → LOW when filtered voltage falls below BATTERY_LOW_V
 *   LOW → (any)   only after power-cycle; state is latched once LOW is reached
 *
 * Battery LOW is notification-only: this module latches the state but does NOT
 * stop the motors (auto-return / blade lockout were removed — operator decision).
 */
enum BatteryState {
    BATTERY_OK,       ///< Voltage ≥ BATTERY_WARN_V — normal operation
    BATTERY_WARNING,  ///< BATTERY_LOW_V ≤ voltage < BATTERY_WARN_V — warn operator
    BATTERY_LOW,      ///< Voltage < BATTERY_LOW_V — notify operator, charge required (no motor stop)
};


// ─── Public API ──────────────────────────────────────────────────────────────

/**
 * @brief Initialise battery monitor; seeds internal state for VESC voltage source.
 *
 * No ADC or NVS initialisation required — voltage is sourced from VESC STATUS_5.
 * Seeds the filtered voltage to 0.0f; the first non-zero reading from
 * vesc_get_battery_voltage() will populate it on the first update() call.
 *
 * Call from setup() before the state machine starts.
 */
void battery_monitor_init();

/**
 * @brief Sample VESC voltage, apply IIR filter, update state classification.
 *
 * Reads vesc_get_battery_voltage() and feeds the result through a first-order
 * IIR low-pass filter:
 *
 *   filtered_v = filtered_v × (1 − α) + raw_v × α
 *
 * With α = BATTERY_FILTER_ALPHA (0.10) at a 2 Hz call rate:
 *   time constant ≈ 1 / (2 × 0.10) = 5 seconds.
 *
 * If vesc_get_battery_voltage() returns 0.0f (no STATUS_5 received yet),
 * the update is skipped — state remains BATTERY_OK until a valid reading arrives.
 *
 * On the first valid call, the filter is seeded with the raw reading (no warm-up
 * delay). Once BATTERY_LOW is reached, the state is latched — it cannot
 * return to WARNING or OK without a power-cycle.
 *
 * Call at approximately 2 Hz from the state machine loop (non-blocking).
 */
void battery_monitor_update();

/**
 * @brief Get the current filtered + calibrated battery voltage in volts.
 *
 * Thread-safe: reads a single volatile float (32-bit aligned; atomic on
 * ARM Cortex-M/Xtensa LX7 without a mutex).
 *
 * @return Battery voltage in volts, or 0.0 before the first update() call.
 */
float battery_get_voltage();

/**
 * @brief Get the current battery state classification.
 *
 * @return BATTERY_OK, BATTERY_WARNING, or BATTERY_LOW.
 */
BatteryState battery_get_state();

/**
 * @brief Print battery diagnostics to Serial.
 *
 * Output format (single line):
 *   [BATTERY] Voltage: 48.3V  State: OK  (source: VESC CAN STATUS_5)
 */
void battery_dump();
