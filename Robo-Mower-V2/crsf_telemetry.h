#pragma once
#include <Arduino.h>
#include <stdint.h>

// ── Beep request types (packed into MOWER_STATUS flags bits 7:6) ─────────────
constexpr uint8_t BEEP_NONE    = 0;  // no beep
constexpr uint8_t BEEP_CONFIRM = 1;  // short confirmation tone
constexpr uint8_t BEEP_WARNING = 2;  // attention / degraded state
constexpr uint8_t BEEP_FAULT   = 3;  // critical fault

// ══════════════════════════════════════════════════════════════════════════════
//  crsf_telemetry.h — CRSF Uplink Telemetry Transmitter
//  RadioMaster TX16S ← ER8 receiver ← ESP32-S3 via HardwareSerial Serial2
//
//  Transmits 5 CRSF frame types in rotating priority order:
//    FLIGHT_MODE (0x1E) → GPS (0x02) → BATTERY (0x08) →
//    MOWER_STATUS (0x80) → ATTITUDE (0x14) → repeat
//
//  All frames follow the standard CRSF frame format:
//    [0xC8][len][type][payload...][CRC8(type+payload, poly 0xD5)]
//  where len = payload_len + 2  (type byte + CRC byte).
//
//  Transmission only occurs during the inter-packet gap, detected via
//  crsf_telemetry_window_open() from crsf_input.h.
//
//  Update rate: crsf_telemetry_update() called at 10 Hz by state machine.
//  Service rate: crsf_telemetry_service() called by crsfTask when window open
//                (~100 Hz opportunity, one frame per gap, net ~20 Hz per type).
//
//  CRSF custom frame 0x80 (MOWER_STATUS) is not auto-discovered by EdgeTX.
//  It is readable by a Lua widget. Payload layout is documented in HANDOFF.md.
//
//  References: ARCHITECTURE.md §1, ASSUMPTIONS.md A12, A22
// ══════════════════════════════════════════════════════════════════════════════


/**
 * Data snapshot filled by the state machine each cycle and passed to
 * crsf_telemetry_update(). All fields should be populated before each call.
 *
 * The struct is copied atomically under a mutex so callers need not hold
 * any lock after crsf_telemetry_update() returns.
 */
struct TelemetryData {
    // ── GPS (frame 0x02) ──────────────────────────────────────────────────────
    double  lat_deg;           ///< WGS-84 latitude in degrees (EKF-fused)
    double  lon_deg;           ///< WGS-84 longitude in degrees (EKF-fused)
    float   groundspeed_ms;   ///< Ground speed in m/s (EKF velocity)
    float   heading_deg;       ///< True heading in degrees [0, 360)
    float   altitude_m;        ///< Altitude above datum in metres — 0.0 (mower is flat)
    uint8_t satellites;        ///< Number of GNSS satellites used in fix

    // ── Battery (frame 0x08) ─────────────────────────────────────────────────
    float    battery_voltage_V;      ///< Main battery voltage from battery_get_voltage()
    float    blade_current_A;        ///< Blade motor phase current (system current proxy)
    uint32_t capacity_mah_used;      ///< Running integral estimate (blade_current × time)
    uint8_t  battery_remaining_pct;  ///< Estimate: (V−LOW) / (FULL−LOW) × 100, clamped 0–100

    // ── Attitude (frame 0x14) ────────────────────────────────────────────────
    float   yaw_rad;           ///< EKF heading in radians [−π, +π]; pitch=roll=0 (flat mower)

    // ── Flight mode (frame 0x1E) ─────────────────────────────────────────────
    char    flight_mode[17];   ///< Null-terminated state name, max 16 chars. Examples:
                               ///<   "MANUAL", "AUTO-OK", "LEARN", "ESTOP", "RECOVERY"

    // ── Mower status (frame 0x80, custom 15-byte payload) ───────────────────
    uint8_t  state;            ///< RobotState enum value (0–10), see state_machine.h
    uint8_t  hprog;            ///< Headland progress 0–100 %
    uint8_t  sprog;            ///< Strip progress 0–100 %
    uint8_t  cut_height_mm;    ///< Current cut height in mm (from servo_output)
    uint8_t  blade_load_pct;   ///< Blade load fraction 0–100 % (cutting_monitor)
    uint8_t  fix_type;         ///< GPS fix type: 0=none,1=GPS,2=DGPS,4=RTK-fixed,5=RTK-float
    uint8_t  flags;            ///< Status bitfield:
                               ///<   bit 0 — armed
                               ///<   bit 1 — blade on
                               ///<   bit 2 — bog recovery active
                               ///<   bit 3 — retrace active
                               ///<   bit 4 — RTK float (degraded positioning)
                               ///<   bit 5 — obstacle near
    uint16_t obs_count;        ///< Total obstacles detected this session
    uint16_t ekf_unc_cm;       ///< EKF position uncertainty in cm (ekf_get_position_uncertainty() × 100)
    uint32_t session_mowed_dm2;///< Mowed area this session in dm² (coverage_planner)

    // ── Beep request (one-shot, packed into flags bits 7:6) ──────────────────
    uint8_t  beep_request;     ///< 0=none 1=confirm 2=warning 3=fault
                               ///< Set via request_beep(). Automatically cleared
                               ///< after the next MOWER_STATUS frame is sent.
};


// ── Public API ────────────────────────────────────────────────────────────────

/**
 * Initialise the telemetry module.
 *
 * Creates the internal FreeRTOS mutex protecting the telemetry data snapshot.
 * Must be called once from setup() after crsf_input_init().
 * Serial2 is already configured by crsf_input_init() — do not re-initialise here.
 */
void crsf_telemetry_init();

/**
 * Update the telemetry data snapshot.
 *
 * Called from stateMachineTask at 10 Hz. Acquires the internal mutex, copies
 * @p data into the module-level snapshot, then releases the mutex.
 *
 * @param data  Fully populated TelemetryData struct for the current cycle.
 */
void crsf_telemetry_update(const TelemetryData &data);

/**
 * Service the telemetry transmit queue.
 *
 * Must be called from the crsfTask (Core 1) each time a telemetry window
 * is detected. Returns immediately without transmitting if the window is not
 * open (crsf_telemetry_window_open() == false).
 *
 * When the window IS open, builds and transmits exactly ONE frame from the
 * rotating priority queue:
 *   0x1E (FLIGHT_MODE) → 0x02 (GPS) → 0x08 (BATTERY) →
 *   0x80 (MOWER_STATUS) → 0x14 (ATTITUDE) → repeat
 *
 * The entire frame is written to Serial2 before returning. At 420000 baud,
 * the longest frame (GPS, ~19 bytes) takes ≈ 360 µs, well within the
 * ~8 ms inter-packet gap available at 100 Hz CRSF frame rate.
 */
void crsf_telemetry_service();

/**
 * Request an audible beep on the TX16S via the next MOWER_STATUS frame.
 *
 * Uses raise-only semantics: a higher-severity request is never overwritten
 * by a lower one within the same telemetry cycle. The request is automatically
 * cleared after the MOWER_STATUS frame containing it has been transmitted.
 *
 * Thread-safe — protected by the same mutex as crsf_telemetry_update().
 *
 * @param type  One of BEEP_NONE, BEEP_CONFIRM, BEEP_WARNING, BEEP_FAULT.
 */
void request_beep(uint8_t type);
