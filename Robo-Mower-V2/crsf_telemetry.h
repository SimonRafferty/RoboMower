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

    // ── Mower status (frame 0x80, custom 20-byte payload) ───────────────────
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
    uint8_t  calib_status;     ///< BNO055 calibration: bits 7:6 sys, 5:4 gyro,
                               ///< 3:2 accel, 1:0 mag (each 0–3). 0x80 byte 19.
    uint8_t  auto_deny_code;   ///< AUTO-start gate reason (0=none/OK), 0x80 byte 20.
                               ///< 1=no perimeter 2=no position 3=heading not set
                               ///< 4=fix not RTK 5=EPE/uncertainty too high 6=outside perimeter.
                               ///< The Lua widget banners this for ~2 s.
    uint8_t  wp_bearing_half;  ///< Bearing to next AUTO waypoint, deg/2 (0–179 =
                               ///< 0–358°, 0=N CW+), 255=no waypoint. 0x80 byte 21.
                               ///< The Lua widget draws it as a yellow compass line.

    // ── Beep request (type in flags bits 7:6 + sequence counter, byte 22) ─────
    // The type is carried in EVERY MOWER_STATUS frame (no longer a one-shot bit);
    // beep_seq is bumped by request_beep() on each event. The Lua plays a tone
    // when beep_seq advances, so a single dropped frame can't lose a beep — the
    // next frame still carries the new sequence. Both fields are owned by the
    // telemetry module (preserved across crsf_telemetry_update snapshots).
    uint8_t  beep_request;     ///< 0=none 1=confirm 2=warning 3=fault (latest event)
    uint8_t  beep_seq;         ///< increments per request_beep(); 0x80 byte 22
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
 * the longest frame (MOWER_STATUS, 19-byte payload) takes ≈ 360 µs, well within the
 * ~8 ms inter-packet gap available at 100 Hz CRSF frame rate.
 */
void crsf_telemetry_service();

/**
 * Request an audible beep on the TX16S.
 *
 * Records the beep type (carried in MOWER_STATUS flags bits 7:6 of every frame)
 * and increments a sequence counter (byte 22). The Lua widget plays the tone
 * when the sequence advances, so delivery survives dropped frames — each call
 * produces exactly one beep on the TX. Latest call wins the type.
 *
 * Thread-safe — protected by the same mutex as crsf_telemetry_update().
 *
 * @param type  One of BEEP_CONFIRM, BEEP_WARNING, BEEP_FAULT (BEEP_NONE ignored).
 */
void request_beep(uint8_t type);
