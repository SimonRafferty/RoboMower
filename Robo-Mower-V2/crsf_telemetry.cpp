// ══════════════════════════════════════════════════════════════════════════════
//  crsf_telemetry.cpp — CRSF Uplink Telemetry Transmitter
//
//  Builds and transmits 5 CRSF telemetry frame types to the ER8 receiver in a
//  rotating priority queue. Transmission is gated by crsf_telemetry_window_open()
//  which is true only during the inter-packet silence gap (≥ 2 ms) detected by
//  the CRSF receive task. One frame is sent per window opportunity.
//
//  Frame format (CRSF standard):
//    [0xC8][len][type][payload...][CRC8(type+payload, poly 0xD5)]
//    len = payload_len + 2  (type byte + CRC byte, NOT including sync or len)
//
//  CRC8 polynomial 0xD5 (DVB-S2), same as used by crsf_input.cpp for
//  inbound frame validation. Reimplemented here as a static helper to avoid
//  coupling to crsf_input internals. At telemetry rates (~5 frames/sec per
//  type) a bitwise CRC is fast enough; no lookup table is needed.
//
//  References: ARCHITECTURE.md §1, ASSUMPTIONS.md A12, A22
//  HANDOFF: HANDOFFS/17_crsf_telemetry/HANDOFF.md
// ══════════════════════════════════════════════════════════════════════════════

#include "crsf_telemetry.h"
#include "crsf_input.h"   // crsf_telemetry_window_open()
#include "config.h"       // BATTERY_FULL_V, BATTERY_LOW_V

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>
#include <math.h>

// ── Module-level state ────────────────────────────────────────────────────────

/** Last snapshot written by crsf_telemetry_update(). Protected by s_mutex. */
static TelemetryData s_data;

/** Mutex protecting s_data between stateMachineTask (writer) and crsfTask (reader). */
static SemaphoreHandle_t s_mutex = nullptr;

/**
 * Frame rotation counter. Incremented after each successful transmission.
 * Index into s_frame_order[] via modulo. Not protected by mutex — only written
 * and read from crsfTask (Core 1).
 */
static int s_frame_idx = 0;

/**
 * Transmit priority order. FLIGHT_MODE first per spec — it is the primary
 * human-readable status indicator shown on the TX16S screen. ATTITUDE last
 * because pitch and roll are always zero for a flat mower (lowest information
 * value), so it is deprioritised to the end of the rotation.
 */
static const uint8_t s_frame_order[] = {0x1E, 0x02, 0x08, 0x80, 0x14};
#define N_FRAMES  (sizeof(s_frame_order) / sizeof(s_frame_order[0]))

// ── Internal helpers ──────────────────────────────────────────────────────────

/**
 * Compute CRC8 with polynomial 0xD5 (DVB-S2, CRSF standard).
 *
 * Covers exactly the bytes supplied — caller must pass [type][payload...].
 * Sync byte (0xC8) and length byte are excluded per the CRSF specification.
 *
 * @param data  Pointer to first byte to include (type byte of the frame).
 * @param len   Number of bytes to process.
 * @return      8-bit CRC.
 */
static uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0xD5);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

/** Write a uint16_t to buf in big-endian order. */
static inline void pack16be(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val >> 8);
    buf[1] = (uint8_t)(val & 0xFF);
}

/** Write a uint32_t to buf in big-endian order. */
static inline void pack32be(uint8_t *buf, uint32_t val)
{
    buf[0] = (uint8_t)(val >> 24);
    buf[1] = (uint8_t)((val >> 16) & 0xFF);
    buf[2] = (uint8_t)((val >>  8) & 0xFF);
    buf[3] = (uint8_t)(val & 0xFF);
}

/**
 * Assemble and transmit a complete CRSF frame on Serial2.
 *
 * Frame layout:
 *   [0xC8] [len] [type] [payload × payload_len] [CRC8(type+payload)]
 *   len = payload_len + 2
 *
 * Maximum supported payload_len is 61 bytes (64-byte frame buffer minus the
 * 3-byte header). In practice the largest frame here is MOWER_STATUS (19-byte payload).
 *
 * @param type         CRSF frame type byte.
 * @param payload      Pointer to payload bytes.
 * @param payload_len  Number of payload bytes.
 */
static void sendFrame(uint8_t type, const uint8_t *payload, uint8_t payload_len)
{
    // Maximum frame = 1 (sync) + 1 (len) + 1 (type) + 61 (payload) + 1 (CRC) = 65 bytes.
    // All frames here are well below this limit.
    uint8_t frame[64];

    frame[0] = 0xC8;                         // CRSF sync byte
    frame[1] = (uint8_t)(payload_len + 2);   // len = type(1) + payload + CRC(1)
    frame[2] = type;
    memcpy(&frame[3], payload, payload_len);

    // CRC covers type byte + payload bytes (NOT sync or len)
    frame[3 + payload_len] = crsf_crc8(&frame[2], (uint8_t)(payload_len + 1));

    Serial2.write(frame, (size_t)(4 + payload_len));
}

// ── Frame builders ────────────────────────────────────────────────────────────

/**
 * Build and send FLIGHT_MODE frame (0x1E).
 *
 * Payload: null-terminated ASCII string, max 16 bytes total (15 chars + '\0').
 * The string is taken from s_data.flight_mode, truncated to 15 chars if longer.
 *
 * Byte offsets:
 *   [0 .. N-1] : ASCII chars
 *   [N]        : null terminator (always present)
 */
static void sendFlightMode(const TelemetryData &d)
{
    // Include null terminator; cap at 16 bytes (15 printable chars + '\0')
    size_t slen = strnlen(d.flight_mode, 15);  // max 15 chars
    uint8_t payload[16];
    memcpy(payload, d.flight_mode, slen);
    payload[slen] = '\0';
    uint8_t payload_len = (uint8_t)(slen + 1);

    sendFrame(0x1E, payload, payload_len);
}

/**
 * Build and send GPS frame (0x02).
 *
 * Payload (15 bytes, big-endian):
 *   [0..3]  int32_t  lat  × 1e7
 *   [4..7]  int32_t  lon  × 1e7
 *   [8..9]  uint16_t groundspeed_kmh × 100  (= ms × 3.6 × 100)
 *   [10..11] uint16_t heading_deg × 100
 *   [12..13] uint16_t altitude_m + 1000  (offset to keep unsigned)
 *   [14]    uint8_t  satellites
 */
static void sendGPS(const TelemetryData &d)
{
    uint8_t payload[15];

    int32_t lat7 = (int32_t)(d.lat_deg * 1e7);
    int32_t lon7 = (int32_t)(d.lon_deg * 1e7);
    pack32be(&payload[0], (uint32_t)lat7);
    pack32be(&payload[4], (uint32_t)lon7);

    // Speed: m/s → km/h × 100, clamped to uint16_t max
    float speed_kmh100 = d.groundspeed_ms * 3.6f * 100.0f;
    uint16_t spd = (speed_kmh100 < 0.0f)    ? 0u
                 : (speed_kmh100 > 65535.0f) ? 65535u
                 : (uint16_t)speed_kmh100;
    pack16be(&payload[8], spd);

    // Heading: degrees × 100, wrap to [0, 36000)
    float hdg = d.heading_deg;
    while (hdg <    0.0f) hdg += 360.0f;
    while (hdg >= 360.0f) hdg -= 360.0f;
    uint16_t hdg100 = (uint16_t)(hdg * 100.0f);
    pack16be(&payload[10], hdg100);

    // Altitude: metres + 1000 offset (keeps 0 m at 1000, valid for flat terrain)
    float alt_off = d.altitude_m + 1000.0f;
    uint16_t alt = (alt_off < 0.0f)    ? 0u
                 : (alt_off > 65535.0f) ? 65535u
                 : (uint16_t)alt_off;
    pack16be(&payload[12], alt);

    payload[14] = d.satellites;

    sendFrame(0x02, payload, sizeof(payload));
}

/**
 * Build and send BATTERY_SENSOR frame (0x08).
 *
 * Payload (8 bytes, big-endian):
 *   [0..1]  uint16_t voltage_V × 100         (centi-volts)
 *   [2..3]  uint16_t blade_current_A × 100   (centi-amps)
 *   [4..6]  uint24_t capacity_mAh used        (3 bytes, MSB first)
 *   [7]     uint8_t  battery remaining %
 */
static void sendBattery(const TelemetryData &d)
{
    uint8_t payload[8];

    // Voltage: V × 100, clamped to uint16_t
    float v100 = d.battery_voltage_V * 100.0f;
    uint16_t volt = (v100 < 0.0f)    ? 0u
                  : (v100 > 65535.0f) ? 65535u
                  : (uint16_t)v100;
    pack16be(&payload[0], volt);

    // Current: A × 100, clamped to uint16_t
    float a100 = d.blade_current_A * 100.0f;
    uint16_t curr = (a100 < 0.0f)    ? 0u
                  : (a100 > 65535.0f) ? 65535u
                  : (uint16_t)a100;
    pack16be(&payload[2], curr);

    // Capacity (mAh): 24-bit big-endian, clamped to 16777215
    uint32_t cap = (d.capacity_mah_used > 0x00FFFFFFu) ? 0x00FFFFFFu : d.capacity_mah_used;
    payload[4] = (uint8_t)((cap >> 16) & 0xFF);
    payload[5] = (uint8_t)((cap >>  8) & 0xFF);
    payload[6] = (uint8_t)(cap & 0xFF);

    payload[7] = d.battery_remaining_pct;

    sendFrame(0x08, payload, sizeof(payload));
}

/**
 * Build and send ATTITUDE frame (0x14).
 *
 * Payload (6 bytes, big-endian, signed int16_t × 10000 radians):
 *   [0..1]  int16_t pitch × 10000  — always 0 (mower is flat)
 *   [2..3]  int16_t roll  × 10000  — always 0 (mower is flat)
 *   [4..5]  int16_t yaw   × 10000  — EKF heading in radians
 *
 * Range: int16_t holds ±3.2768. Yaw is in [−π, +π] ≈ [−3.14, +3.14] — safe.
 */
static void sendAttitude(const TelemetryData &d)
{
    uint8_t payload[6];

    // Pitch and roll are zero — mower is always flat
    pack16be(&payload[0], 0);
    pack16be(&payload[2], 0);

    // Yaw: clamp to int16_t range before scaling
    float yaw = d.yaw_rad;
    float yaw_scaled = yaw * 10000.0f;
    yaw_scaled = (yaw_scaled < -32768.0f) ? -32768.0f
               : (yaw_scaled >  32767.0f) ?  32767.0f
               : yaw_scaled;
    int16_t yaw16 = (int16_t)yaw_scaled;
    pack16be(&payload[4], (uint16_t)yaw16);

    sendFrame(0x14, payload, sizeof(payload));
}

/**
 * Build and send MOWER_STATUS frame (0x80) — custom frame, 19-byte payload.
 *
 * This is not an EdgeTX-standard frame type. It is readable by a Lua widget
 * on the TX16S via crossfireTelemetryPop().
 *
 * Byte layout matches the Lua widget (data[] is 1-based in Lua):
 *   [0]  (1)  uint8_t  state           — RobotState enum 0–10
 *   [1]  (2)  uint8_t  hprog           — headland progress 0–100 %
 *   [2]  (3)  uint8_t  sprog           — strip progress 0–100 %
 *   [3]  (4)  uint8_t  cut_height_mm
 *   [4]  (5)  uint8_t  blade_load_pct  — 0–100 %
 *   [5]  (6)  uint8_t  fix_type        — GPS fix quality
 *   [6]  (7)  uint8_t  flags           — status bitfield; bits 7:6 = beep request
 *   [7]  (8)  uint8_t  obs_count MSB   } uint16 big-endian obstacle count
 *   [8]  (9)  uint8_t  obs_count LSB   }
 *   [9]  (10) uint8_t  ekf_unc_cm MSB  } uint16 big-endian EKF uncertainty cm
 *   [10] (11) uint8_t  ekf_unc_cm LSB  }
 *   [11] (12) uint8_t  mowed MSB       }
 *   [12] (13) uint8_t  mowed byte 1    } uint32 big-endian session area dm²
 *   [13] (14) uint8_t  mowed byte 2    }
 *   [14] (15) uint8_t  mowed LSB       }
 *   [15] (16) uint8_t  battV MSB       } uint16 big-endian battery voltage ×100
 *   [16] (17) uint8_t  battV LSB       }   (carried here so the Lua does not
 *   [17] (18) uint8_t  heading MSB     } uint16 big-endian heading deg ×10
 *   [18] (19) uint8_t  heading LSB     }   depend on EdgeTX sensor discovery —
 *                                          RxBt/Yaw sensors can silently vanish)
 *   [19] (20) uint8_t  calib            sys<<6 | gyro<<4 | accel<<2 | mag (each 0–3)
 */
static void sendMowerStatus(const TelemetryData &d)
{
    uint8_t payload[20];

    payload[0] = d.state;
    payload[1] = d.hprog;
    payload[2] = d.sprog;
    payload[3] = d.cut_height_mm;
    payload[4] = d.blade_load_pct;
    payload[5] = d.fix_type;
    // Pack beep request into bits 7:6 of flags byte
    payload[6] = d.flags | (uint8_t)((d.beep_request & 0x03) << 6);
    pack16be(&payload[7], d.obs_count);
    pack16be(&payload[9], d.ekf_unc_cm);
    pack32be(&payload[11], d.session_mowed_dm2);

    // Direct-decode duplicates of values that otherwise rely on EdgeTX
    // auto-registered sensors (BATTERY 0x08 → RxBt, ATTITUDE 0x14 → Yaw).
    float v = d.battery_voltage_V;
    if (v < 0.0f) v = 0.0f;
    if (v > 655.0f) v = 655.0f;
    pack16be(&payload[15], (uint16_t)(v * 100.0f));

    float hdg = d.heading_deg;
    while (hdg < 0.0f)    hdg += 360.0f;
    while (hdg >= 360.0f) hdg -= 360.0f;
    pack16be(&payload[17], (uint16_t)(hdg * 10.0f));

    payload[19] = d.calib_status;   // BNO055 calibration (sys/gyro/accel/mag)
    sendFrame(0x80, payload, sizeof(payload));
}

// ── Public API ────────────────────────────────────────────────────────────────

void crsf_telemetry_init()
{
    s_mutex = xSemaphoreCreateMutex();
    // Initialise snapshot to safe defaults
    memset(&s_data, 0, sizeof(s_data));
    strncpy(s_data.flight_mode, "INIT", sizeof(s_data.flight_mode) - 1);
    s_data.flight_mode[sizeof(s_data.flight_mode) - 1] = '\0';
    s_frame_idx = 0;
}

void crsf_telemetry_update(const TelemetryData &data)
{
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        uint8_t pending_beep = s_data.beep_request;  // preserve pending beep
        s_data = data;
        // Raise-only: don't let the incoming snapshot downgrade a pending beep
        if (pending_beep > s_data.beep_request) {
            s_data.beep_request = pending_beep;
        }
        xSemaphoreGive(s_mutex);
    }
}

void crsf_telemetry_service()
{
    // Gate on inter-packet silence window — do not transmit mid-packet
    if (!crsf_telemetry_window_open()) {
        return;
    }

    // Take a local copy of the snapshot to minimise mutex hold time.
    // The mutex prevents a torn read if stateMachineTask is mid-update.
    TelemetryData local;
    if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
        local = s_data;
        xSemaphoreGive(s_mutex);
    } else {
        // Could not acquire mutex — skip this window rather than block crsfTask
        return;
    }

    // Advance rotation and dispatch one frame
    uint8_t type = s_frame_order[s_frame_idx % N_FRAMES];
    s_frame_idx++;

    switch (type) {
        case 0x1E:  sendFlightMode(local);   break;
        case 0x02:  sendGPS(local);          break;
        case 0x08:  sendBattery(local);      break;
        case 0x80:  sendMowerStatus(local);  break;
        case 0x14:  sendAttitude(local);     break;
        default:    break;  // unreachable with defined s_frame_order
    }

    // Auto-clear beep request after MOWER_STATUS has been transmitted.
    // One-shot: the beep is sent exactly once, then reset to BEEP_NONE.
    if (type == 0x80 && local.beep_request != BEEP_NONE) {
        if (xSemaphoreTake(s_mutex, 0) == pdTRUE) {
            s_data.beep_request = BEEP_NONE;
            xSemaphoreGive(s_mutex);
        }
        // If mutex unavailable, the beep is sent again next rotation — acceptable
    }
}

void request_beep(uint8_t type)
{
    if (type > BEEP_FAULT) return;  // ignore invalid values
    if (xSemaphoreTake(s_mutex, portMAX_DELAY) == pdTRUE) {
        // Raise-only: never downgrade a pending request within one cycle
        if (type > s_data.beep_request) {
            s_data.beep_request = type;
        }
        xSemaphoreGive(s_mutex);
    }
}
