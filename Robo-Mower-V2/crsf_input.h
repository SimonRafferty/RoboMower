#pragma once
#include <Arduino.h>
#include <stdint.h>

// ══════════════════════════════════════════════════════════════════════════════
//  crsf_input.h — CRSF Serial Protocol Receiver
//  RadioMaster ER8 receiver via HardwareSerial Serial2 at 420 000 baud.
//
//  Protocol: CRSF (Crossfire Serial Protocol)
//    Frame: [0xC8][len][type][payload...][CRC8(type+payload, poly 0xD5)]
//    RC frame type 0x16 (RC_CHANNELS_PACKED): 22-byte payload, 16 × 11-bit ch.
//    Failsafe frame type 0x1C: triggers failsafe immediately.
//
//  Thread safety: all shared state protected by a FreeRTOS mutex.
//  The receive task runs on Core 1 at priority 14 (ARCHITECTURE.md §2).
//
//  NOTE — CH6 arm comparison:
//    config.h defines CH6_ARMED_THRESHOLD = 496 in raw CRSF units.
//    CRSFChannels.ch[] stores values in MICROSECONDS (1000–2000 µs).
//    Convert before comparing: crsf_raw_to_us(CH6_ARMED_THRESHOLD) ≈ 1198 µs.
//    Use `channels.ch[CRSF_CH_ARM] > crsf_raw_to_us(CH6_ARMED_THRESHOLD)`.
// ══════════════════════════════════════════════════════════════════════════════

// ── Channel index constants (0-indexed into CRSFChannels.ch[]) ───────────────
#define CRSF_CH_STEERING    0   ///< CH1: 172=full left, 992=centre, 1811=full right
#define CRSF_CH_THROTTLE    1   ///< CH2: 172=full reverse, 992=stop, 1811=full forward
#define CRSF_CH_CUT_HEIGHT  2   ///< CH3: 172=lowest cut height, 1811=highest
#define CRSF_CH_MODE        3   ///< CH4: raw<496=MANUAL, 496-1316=AUTO, >1316=AUTO+RETURN
#define CRSF_CH_LEARN       4   ///< CH5: raw>1316=perimeter learning active
#define CRSF_CH_ARM         5   ///< CH6: raw>496=ARMED (see config.h CH6_ARMED_THRESHOLD)
#define CRSF_CH_SPARE7      6   ///< CH7: reserved for future use
#define CRSF_CH_LEARN_PT    7   ///< CH8: momentary — record single perimeter point

// ─────────────────────────────────────────────────────────────────────────────

/**
 * Snapshot of the 8 RC channel values plus failsafe state.
 *
 * ch[] stores values in **microseconds** (1000–2000 µs), converted from the
 * raw 11-bit CRSF range (172–1811) by crsf_raw_to_us().
 *
 * IMPORTANT: config.h channel thresholds (CH6_ARMED_THRESHOLD, CH5_LEARN_THRESHOLD,
 * CH4 mode thresholds) are expressed in raw CRSF units. Convert with crsf_raw_to_us()
 * before comparing against ch[] values.
 */
struct CRSFChannels {
    uint16_t ch[8];         ///< CH1–CH8 in microseconds (1000–2000 µs)
    bool     failsafe;      ///< true = failsafe active (no valid frames, or explicit 0x1C)
    uint32_t last_frame_ms; ///< millis() timestamp of last successfully validated frame
};

// ── Public API ────────────────────────────────────────────────────────────────

/**
 * Initialise Serial2 (CRSF UART) and start the FreeRTOS receive task.
 *
 * Must be called from setup(). The task is pinned to Core 1 at priority 14.
 * After return the task is running but failsafe remains active until the first
 * valid RC_CHANNELS_PACKED frame is received from the ER8.
 */
void crsf_input_init();

/**
 * Get a thread-safe snapshot of the latest channel data.
 *
 * Acquires the CRSF mutex, copies the global struct, and releases the mutex.
 * The returned copy may be used freely without holding any lock.
 *
 * @return CRSFChannels snapshot. failsafe=true if no valid frame for
 *         RC_FAILSAFE_TIMEOUT_MS, or if an explicit CRSF failsafe frame (0x1C)
 *         was received.
 */
CRSFChannels crsf_get_channels();

/**
 * Returns true if CRSF failsafe is currently active.
 *
 * Failsafe is set when:
 *   - No valid RC_CHANNELS_PACKED frame for RC_FAILSAFE_TIMEOUT_MS milliseconds, OR
 *   - An explicit CRSF failsafe frame (type 0x1C) is received.
 * It clears as soon as a valid RC_CHANNELS_PACKED frame is processed.
 *
 * Thread-safe (mutex protected).
 */
bool crsf_is_failsafe();

/**
 * Convert a raw CRSF channel value (172–1811) to microseconds (1000–2000 µs).
 *
 * Values outside the CRSF raw range are clamped at 1000 or 2000 µs.
 * Formula: us = ((raw - 172) * 1000 / (1811 - 172)) + 1000
 *
 * @param raw  Raw 11-bit CRSF value (nominally 172–1811)
 * @return     Equivalent pulse width in microseconds (1000–2000)
 */
uint16_t crsf_raw_to_us(uint16_t raw);

/**
 * Normalise a microsecond channel value to a signed float in [-1.0, +1.0].
 *
 * Centre (1500 µs) maps to 0.0. Full throw (1000 or 2000 µs) maps to ±1.0.
 * A dead band of ±50 µs around centre is zeroed (|norm| < 0.05 → 0.0).
 *
 * @param us_value  Channel value in microseconds (typically 1000–2000)
 * @return          Normalised value in [-1.0, +1.0] with dead band applied
 */
float crsf_us_to_norm(uint16_t us_value);

/**
 * Returns true when the CRSF UART has been silent for more than 2 ms.
 *
 * The CRSF protocol reserves the inter-packet gap (>2 ms line silence) as
 * the telemetry transmission window. The CRSF telemetry module should check
 * this flag before transmitting a telemetry frame to the ER8 receiver.
 *
 * The flag is reset to false as soon as any byte is received from Serial2,
 * and set to true once 2 ms of silence is detected by the receive task.
 *
 * Note: this flag is a volatile bool — no mutex needed for single-byte access.
 */
bool crsf_telemetry_window_open();
