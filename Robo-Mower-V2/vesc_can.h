#pragma once
#include <Arduino.h>
#include <stdint.h>

// ══════════════════════════════════════════════════════════════════════════════
//  vesc_can.h — RoboMower VESC CAN Bus Interface
//
//  Manages communication with three VESC motor controllers via the ESP32-S3
//  TWAI peripheral (CAN 2.0B, 250 kbit/s, 29-bit extended frames).
//
//  VESC IDs:  1 = Left drive,  2 = Right drive,  3 = Blade
//  CAN frame: identifier = (packet_id << 8) | vesc_id  (29-bit extended)
//
//  Thread safety:
//    - Status and odometry reads protected by an internal FreeRTOS mutex.
//    - All TX commands enqueued to xQueueVescTx (depth 8, declared here as
//      extern so safetyTask / stateMachineTask can access it directly if needed
//      — normal callers should prefer the API functions below).
//    - vesc_emergency_stop_all() enqueues to the FRONT of the TX queue so
//      stop frames overtake any pending motion commands.
//
//  Source references:
//    Spec:          Robo_Mower_claudecode_prompt_v3.md §"Motor Controllers"
//    Assumptions:   ASSUMPTIONS.md A17, A18
//    Architecture:  ARCHITECTURE.md §2 (task priorities), §3 (pin assignments)
//    Config:        Robo-Mower-V2/config.h (CAN_TX_PIN, CAN_RX_PIN, MOTOR_POLE_PAIRS,
//                   GEAR_RATIO, WHEEL_RADIUS_M, TRACK_WIDTH_M)
// ══════════════════════════════════════════════════════════════════════════════

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ── VESC CAN IDs ─────────────────────────────────────────────────────────────
// These match the values in config.h; redeclared here so vesc_can.h is
// self-contained for callers that only include this header.
#ifndef VESC_ID_LEFT
#define VESC_ID_LEFT    1   ///< Left drive motor VESC
#define VESC_ID_RIGHT   2   ///< Right drive motor VESC
#define VESC_ID_BLADE   3   ///< Blade motor VESC (Gtech CLM021)
#endif


// ── TX queue (extern) ─────────────────────────────────────────────────────────
/** FreeRTOS queue handle for VESC TX frames (depth 8).
 *  Declared extern so safety/state-machine tasks may inspect queue depth.
 *  Normal callers must use vesc_set_current() / vesc_set_rpm() / vesc_emergency_stop_all(). */
extern QueueHandle_t xQueueVescTx;


// ── Status & odometry structures ─────────────────────────────────────────────

/** Latest status data for one VESC (updated from CAN_PACKET_STATUS at ~50 Hz). */
struct VescStatus {
    float    erpm;            ///< Electrical RPM (signed; positive = forward)
    float    current_A;       ///< Motor phase current (A)
    float    duty;            ///< Duty cycle (-1.0 to +1.0)
    uint32_t last_update_ms;  ///< millis() timestamp of last received STATUS frame
};

/** Odometry state for a drive VESC.
 *  Updated at 50 Hz from CAN_PACKET_STATUS ERPM integration.
 *  vesc_poll_tachometer() sends no CAN frame; it only records a poll
 *  timestamp in last_poll_ms. */
struct VescOdometry {
    int32_t  tach_raw;        ///< Virtual tachometer counter (accumulated from ERPM)
    int32_t  tach_prev;       ///< Tachometer value at previous sample (for delta)
    float    dist_m;          ///< Total distance travelled since init (m)
    float    velocity_ms;     ///< Current wheel-surface velocity (m/s)
    uint32_t last_poll_ms;    ///< millis() of last vesc_poll_tachometer() call (no CAN frame sent)
};


// ── Initialisation ────────────────────────────────────────────────────────────

/** Initialise the TWAI peripheral and start the vesc_can_rx / vesc_can_tx
 *  FreeRTOS tasks.  Must be called once from setup() before any other
 *  vesc_can_* function.
 *
 *  @param tx_gpio  TWAI TX pin (connect to CAN transceiver, e.g. SN65HVD230)
 *  @param rx_gpio  TWAI RX pin (from CAN transceiver)
 *
 *  TWAI config: 250 kbit/s, normal mode, accept-all filter. */
void vesc_can_init(int tx_gpio, int rx_gpio);


// ── Command functions ─────────────────────────────────────────────────────────

/** Send CAN_PACKET_SET_CURRENT (packet ID 1) to a drive VESC.
 *  Enqueues to the TX queue; returns immediately.
 *
 *  @param vesc_id    VESC_ID_LEFT or VESC_ID_RIGHT
 *  @param current_mA Commanded phase current in milliamps (signed).
 *                    Positive = forward, negative = reverse.
 *                    Caller is responsible for limiting to ±(MAX_CURRENT_A * 1000). */
void vesc_set_current(uint8_t vesc_id, int32_t current_mA);

/** Send CAN_PACKET_SET_DUTY (packet ID 0) to a drive VESC.
 *  Enqueues to the TX queue; returns immediately.
 *  Open-loop voltage command — no speed feedback.
 *
 *  @param vesc_id  VESC_ID_LEFT or VESC_ID_RIGHT
 *  @param duty     Duty cycle in range [-1.0, +1.0].
 *                  Positive = forward, negative = reverse.
 *                  Encoded as int32 = duty × 100000. */
void vesc_set_duty(uint8_t vesc_id, float duty);

/** Send CAN_PACKET_SET_RPM (packet ID 3) to the blade VESC.
 *  Enqueues to the TX queue; returns immediately.
 *
 *  @param vesc_id  VESC_ID_BLADE
 *  @param erpm     Target electrical RPM (unsigned in practice; use
 *                  BLADE_TARGET_RPM * BLADE_MOTOR_POLE_PAIRS from config.h). */
void vesc_set_rpm(uint8_t vesc_id, int32_t erpm);

/** Send SET_CURRENT = 0 to all three VESCs.
 *  Frames are pushed to the FRONT of the TX queue so they overtake any
 *  pending motion commands.  Designed for software-initiated E-stop only —
 *  the physical PILZ relay cuts main bus power independently (A17).
 *
 *  Safe to call from any task or ISR context (uses xQueueSendToFrontFromISR
 *  variant is NOT used here; call from task context only). */
void vesc_emergency_stop_all();


// ── Status & odometry getters (thread-safe) ───────────────────────────────────

/** Return a snapshot of the latest status for the given VESC.
 *  Thread-safe: acquires the internal mutex briefly.
 *
 *  @param vesc_id  VESC_ID_LEFT, VESC_ID_RIGHT, or VESC_ID_BLADE
 *  @return         Copy of VescStatus; last_update_ms == 0 if no frame
 *                  has been received yet. */
VescStatus vesc_get_status(uint8_t vesc_id);

/** Return a snapshot of odometry for a drive VESC.
 *  Thread-safe: acquires the internal mutex briefly.
 *
 *  @param vesc_id  VESC_ID_LEFT or VESC_ID_RIGHT (blade has no odometry)
 *  @return         Copy of VescOdometry. */
VescOdometry vesc_get_odometry(uint8_t vesc_id);


// ── Polling ───────────────────────────────────────────────────────────────────

/** Record a tachometer poll timestamp for the given drive VESC.  Sends NO CAN
 *  frame — the tachometer is derived from ERPM integration in the STATUS RX
 *  path.  Called at 5 Hz from the state-machine or a dedicated timer.
 *  Updates last_poll_ms in the odometry struct.
 *
 *  @param vesc_id  VESC_ID_LEFT or VESC_ID_RIGHT */
void vesc_poll_tachometer(uint8_t vesc_id);


// ── Battery voltage (from VESC STATUS_5) ─────────────────────────────────────

/**
 * @brief True if both drive VESCs (IDs 1 and 2) have sent CAN_PACKET_STATUS
 *        within VESC_STATUS_TIMEOUT_MS.
 *
 * Used by the state machine to detect when the main 48V supply has been
 * restored after a PILZ E-stop or battery swap.  The blade VESC (ID 3) is
 * NOT required — it may be slow to start after power-up.
 *
 * Thread-safe: acquires the internal status mutex briefly.
 *
 * @return true  Both drive VESCs are online (or neither has ever been seen —
 *               startup grace period applies).
 * @return false At least one drive VESC has timed out after an initial frame.
 */
/** True if a VESC STATUS frame is overdue.
 *  Returns false before the CAN bus has gone live (startup grace).
 *  After the bus is live, a VESC with last_update_ms == 0 is flagged after
 *  VESC_STARTUP_GRACE_MS. Used by safety.cpp and all online-check functions.
 *  Thread-safe. */
bool vesc_is_stale(uint8_t vesc_id);

/** True if the blade VESC hasn't sent STATUS_1 within BLADE_VESC_TIMEOUT_MS.
 *  Longer timeout than vesc_is_stale() to accommodate the blade's lower broadcast rate. */
bool vesc_blade_is_stale();

bool vesc_all_drive_online();

/**
 * @brief True if ANY VESC (1, 2, or 3) that was previously seen has now gone stale.
 *
 * Complements vesc_all_drive_online() by also covering the blade VESC.
 * A VESC that has never sent a frame is ignored (startup grace).
 * Used for the error LED override — any VESC dropout shows red fast flash.
 *
 * Thread-safe: acquires the internal status mutex briefly.
 */
bool vesc_any_went_offline();


/**
 * @brief Get the most recently received battery input voltage from VESC STATUS_5.
 *
 * Updated whenever a CAN_PACKET_STATUS_5 frame arrives from VESC_ID_BLADE (CAN ID 3).
 * Returns 0.0f until the first STATUS_5 frame is received.
 *
 * Thread-safe: protected by the internal status mutex.
 *
 * @return Battery voltage in volts (V), or 0.0f if no STATUS_5 received yet.
 */
float vesc_get_battery_voltage();

/**
 * @brief Returns true if no STATUS_5 frame has been received for > 2 seconds
 *        after the first frame was seen.
 *
 * Use this to detect that VESC_ID_BLADE has gone offline (e.g. after a PILZ
 * E-stop with supercap power still present on ESP32).  Returns false during
 * startup grace period (before the first STATUS_5 frame).
 *
 * Thread-safe: protected by the internal status mutex.
 */
bool vesc_battery_voltage_stale();


// ── CAN bus health ────────────────────────────────────────────────────────────

/**
 * @brief Returns true if the TWAI (CAN) peripheral is in the RUNNING state.
 *
 * Uses twai_get_status_info() to query the hardware peripheral state.
 * Returns false if the bus is in BUS_OFF, STOPPED, or RECOVERING state,
 * which indicates a wiring fault, missing termination, or transceiver failure.
 *
 * Thread-safe: twai_get_status_info() is safe to call from any task.
 */
bool vesc_can_bus_ok();


// ── Conversion helpers ────────────────────────────────────────────────────────

/** Convert electrical RPM to wheel-surface velocity (m/s).
 *
 *  Formula:  vel = (erpm / MOTOR_POLE_PAIRS / 60) / GEAR_RATIO * 2π * WHEEL_RADIUS_M
 *
 *  @param erpm  Signed electrical RPM from CAN_PACKET_STATUS
 *  @return      Signed wheel velocity in m/s */
float vesc_erpm_to_velocity(float erpm);

/** Convert electrical RPM to wheel-surface velocity (m/s), then multiply by the
 *  GPS-referenced distance calibration (odo_cal_scale()). Use this at the
 *  motion-estimate consumers (EKF feed, duty-ramp feedback, follower stall/slip)
 *  so the calibrated scale is applied consistently. The raw (unscaled)
 *  vesc_erpm_to_velocity() remains for the cross-core RX-task odometry.
 *
 *  @param erpm  Signed electrical RPM from CAN_PACKET_STATUS
 *  @return      Signed, scale-corrected wheel velocity in m/s */
float vesc_erpm_to_velocity_scaled(float erpm);

/** Convert wheel-surface velocity (m/s) to electrical RPM — inverse of vesc_erpm_to_velocity.
 *
 *  Formula:  erpm = velocity_ms * GEAR_RATIO / (2π * WHEEL_RADIUS_M) * MOTOR_POLE_PAIRS * 60
 *
 *  @param velocity_ms  Signed wheel velocity in m/s
 *  @return             Signed ERPM */
float vesc_velocity_to_erpm(float velocity_ms);

/** Convert a tachometer counter delta to distance (m).
 *
 *  Formula:  dist = tach_delta / (MOTOR_POLE_PAIRS * 2) / GEAR_RATIO * 2π * WHEEL_RADIUS_M
 *
 *  @param tach_delta  Change in virtual tachometer counter
 *  @return            Distance in metres (signed) */
float vesc_tach_to_distance(int32_t tach_delta);
