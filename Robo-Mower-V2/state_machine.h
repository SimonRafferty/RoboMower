#pragma once
#include <Arduino.h>
#include "coverage_planner.h"
#include "ekf_localiser.h"

// ══════════════════════════════════════════════════════════════════════════════
//  state_machine.h — RoboMower Top-Level State Machine
//
//  Orchestrates all subsystems on Core 1 at 10 Hz.
//  Responsibilities:
//    - All state transitions (11 states, per spec §"State Machine")
//    - LED control via FastLED (onboard GPIO48 + external GPIO7 strip)
//    - Serial command processing (USB Serial0)
//    - 2 Hz JSON telemetry on USB Serial0
//    - Manual RC drive mapping (MANUAL state)
//    - Battery monitor update at 2 Hz
//
//  Thread safety: state_machine_update() runs from the state machine task
//  on Core 1.  state_machine_get_state() may be called from any task.
//
//  References:
//    Spec:         Robo_Mower_claudecode_prompt_v3.md §"State Machine"
//    Assumptions:  ASSUMPTIONS.md A07 (LED in SM), A08 (CH5 latching),
//                  A10 (serial in SM), A23 (battery LED overlay / CALBAT)
//    Architecture: ARCHITECTURE.md §1, §2
//    Config:       Robo-Mower-V2/config.h
// ══════════════════════════════════════════════════════════════════════════════


// ── Robot state enumeration ──────────────────────────────────────────────────

/** All possible top-level robot states. */
enum RobotState {
    STATE_INIT,           ///< Boot: hardware init, IMU bias collection, waiting for RC+GPS
    STATE_IDLE,           ///< Armed, stationary, awaiting operator command
    STATE_MANUAL,         ///< Operator driving via RC (CH1/CH2)
    STATE_LEARN_PERIMETER,///< Recording boundary polygon from RC-driven pass
    STATE_AUTO_MOWING,    ///< Autonomous spiral coverage mowing via coverage planner
    STATE_RETRACE,        ///< CUTTING_OVERLOADED recovery: retrace strip at max height
    STATE_BOG_RECOVERY,   ///< CUTTING_STALLED recovery: progressive height raise
    STATE_OBSTACLE_AVOID, ///< Obstacle detected: back up and re-plan
    STATE_AUTO_RETURN,    ///< Plan complete: return to perimeter start point
    STATE_PAUSED,         ///< Auto mowing paused by operator (CH7); planner position held
    STATE_MOTORS_OFFLINE, ///< PILZ fired or battery disconnected; ESP32 running on supercap
};


// ── LED pattern enumeration ───────────────────────────────────────────────────

/**
 * @brief LED display patterns applied by showLeds().
 *
 * Timing is computed from millis() — no blocking delays.
 */
enum LedPattern {
    LED_SOLID,           ///< Constant on at full brightness
    LED_SLOW_PULSE,      ///< 0→100→0% brightness, 2 s period (smooth sine)
    LED_SLOW_FLASH,      ///< 500 ms ON, 500 ms OFF (1 Hz equal mark/space)
    LED_SINGLE_BLINK,    ///< 100 ms ON, 900 ms OFF
    LED_DOUBLE_BLINK,    ///< 100 ms ON, 100 ms OFF, 100 ms ON, 700 ms OFF
    LED_FAST_FLASH,      ///< 250 ms ON, 250 ms OFF
    LED_ALTERNATING,     ///< Via showLedsAlternating(): primary 1000ms, secondary 500ms, off 500ms
    LED_THREE_FLASH,     ///< 3× 100 ms flash then off (one-shot; for perimeter saved)
};


// ── LED control API ──────────────────────────────────────────────────────────

/**
 * @brief Set LED colour and pattern on both onboard and external strips.
 *
 * Called each tick from the state machine update.  The pattern timing is
 * derived from millis() so no state persists between calls — the pattern
 * simply reads the current phase.
 *
 * For LED_ALTERNATING, the second colour (for the off-phase) must be set via
 * showLedsAlternating() rather than this function.
 *
 * @param colour_rgb  24-bit RGB colour: 0xRRGGBB
 * @param pattern     Pattern to display
 */
void showLeds(uint32_t colour_rgb, LedPattern pattern);

/**
 * @brief Alternating LED: two colours, 500 ms each.
 *
 * @param colour_a  First colour  (0xRRGGBB)
 * @param colour_b  Second colour (0xRRGGBB)
 */
void showLedsAlternating(uint32_t colour_a, uint32_t colour_b);

/**
 * @brief Flash white 3× to confirm perimeter saved (blocking 1.2 s sequence).
 *
 * Used only on successful perimeter_finish_recording().  Each flash is
 * 100 ms on / 100 ms off × 3 = 600 ms, plus a 200 ms tail-off = ~800 ms.
 * Called once; returns after the sequence completes.
 */
void ledFlashWhite3x();


// ── Lifecycle ────────────────────────────────────────────────────────────────

/**
 * @brief Initialise state machine and FastLED.
 *
 * Configures FastLED for the onboard LED (GPIO48, 1 pixel) and the external
 * strip (LED_EXTERNAL_PIN / GPIO7, LED_EXTERNAL_COUNT pixels).
 * Sets initial state to STATE_INIT.
 * Must be called once from setup() after all subsystems are initialised.
 */
void state_machine_init();

/**
 * @brief Main update — call from Core 1 loop at exactly 10 Hz (100 ms period).
 *
 * Runs one tick of the state machine:
 *   1. Read RC channels and EKF pose.
 *   2. Execute current-state logic and check transitions.
 *   3. Update LEDs for current state (with battery warning overlay).
 *   4. Poll Serial for incoming commands (line-by-line).
 *   5. Emit 2 Hz JSON telemetry when due.
 *   6. Update battery monitor at 2 Hz.
 */
void state_machine_update();

/**
 * @brief Get current robot state. Thread-safe (reads volatile enum).
 * @return Current RobotState.
 */
RobotState state_machine_get_state();

/**
 * @brief Process one null-terminated serial command line.
 *
 * Called internally by state_machine_update() when a complete line is ready.
 * Also callable directly for testing.
 *
 * Supported commands:
 *   STATUS, PERIMETER, NAVBOUNDARY, WORKINGAREA, PLAN, GRID, UNREACHABLE,
 *   EKFSTATE, CALDUMP, OBSTACLES, ERRORS, CLEARPERIM [CONFIRM], RESETEKF,
 *   CALHEIGHT <sub>
 *
 * @param cmd  Null-terminated ASCII command string (trimmed of CR/LF).
 */
void state_machine_handle_serial(const char *cmd);


// ── BLE integration API ───────────────────────────────────────────────────────

/** Enqueue a BLE JSON command for processing in the next state machine tick.
 *  Safe to call from Core 0 (BLE callback context).
 *  Drops silently if the queue is full or @p len >= 600. */
void state_machine_enqueue_ble_cmd(const char *json, size_t len);

/** Return current cut-height setting in mm. */
float state_machine_get_cut_height_mm();

/** Return true if the RC arm switch (CH6) is currently in the armed position. */
bool state_machine_is_armed();

/** Return CH4 mode switch position: 0=manual, 1=auto, 2=auto-return. */
uint8_t state_machine_get_mode_sw();

/** Return true if CH7 pause switch is active. */
bool state_machine_is_paused_sw();

/** Return true if CH5 learn switch is active. */
bool state_machine_is_learn_sw();

/** Return true if a valid perimeter is loaded in NVS. */
bool state_machine_has_perimeter();

/** Return the area of the loaded perimeter in m² (0.0 if no perimeter). */
float state_machine_get_perimeter_area_m2();

/** Reset all BLE overlay state (drive, arm, pause, learn).
 *  Called from ble_server.cpp ServerCallbacks::onDisconnect() to ensure
 *  motors zero and pause/arm clear when the phone disconnects. */
void state_machine_ble_disconnected();

/** Return true if BLE pause is currently active. */
bool state_machine_is_ble_paused();

/** Return true if BLE arm (blade) is currently active. */
bool state_machine_is_ble_armed();

/** Return true if the blade is currently commanded to spin. */
bool state_machine_is_blade_commanded();

/** Retained for compatibility; the blade battery lockout was removed 2026-06-12,
 *  so this always returns false. */
bool state_machine_is_blade_lockout();

/** Return the raw CH6 arm-switch channel value in microseconds (1000–2000).
 *  Useful for diagnosing arm-switch decoding: ~1000 = disarmed, ~2000 = armed.
 *  Values near 1500 indicate a 3-position switch in the middle position. */
uint16_t state_machine_get_ch6_us();

/** Flags set by the BLE command handler to request immediate characteristic sends.
 *  Checked in state_machine_update() on every tick. */
extern volatile bool g_ble_map_pending;
extern volatile bool g_ble_status_pending;
extern volatile bool g_ble_diag_pending;
