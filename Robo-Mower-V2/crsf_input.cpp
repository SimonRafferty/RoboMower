// ══════════════════════════════════════════════════════════════════════════════
//  crsf_input.cpp — CRSF Serial Protocol Receiver
//
//  Implements a state-machine frame parser running in a dedicated FreeRTOS task
//  on Core 1 (priority 14). Validated RC_CHANNELS_PACKED frames (type 0x16) are
//  unpacked from 11-bit packed format, converted to microseconds, and stored in
//  a mutex-protected global. Failsafe is triggered by missing frames OR by an
//  explicit CRSF failsafe frame (type 0x1C).
//
//  CRC8 polynomial: 0xD5 (DVB-S2), covers [type][payload] bytes.
//  Bit extraction: 3-byte window to correctly handle channels whose 11-bit span
//  crosses three bytes (CH2: bit-offset 6, CH5: bit-offset 7).
//
//  See crsf_input.h for the public API.
// ══════════════════════════════════════════════════════════════════════════════

#include "crsf_input.h"
#include "config.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string.h>   // memcpy
#include "crsf_telemetry.h"   // crsf_telemetry_service()

// ── CRSF protocol constants ───────────────────────────────────────────────────
static constexpr uint8_t  CRSF_SYNC_BYTE         = 0xC8u;
static constexpr uint8_t  CRSF_TYPE_RC_CHANNELS  = 0x16u; ///< RC_CHANNELS_PACKED
static constexpr uint8_t  CRSF_TYPE_FAILSAFE     = 0x1Cu; ///< Explicit failsafe
static constexpr uint8_t  CRSF_RC_PAYLOAD_BYTES  = 22u;   ///< 16 × 11-bit = 176 bits
static constexpr uint8_t  CRSF_MAX_FRAME_LEN     = 64u;   ///< Max valid len field

// ── FreeRTOS task configuration ───────────────────────────────────────────────
// Priority 14 per ARCHITECTURE.md §2 (task brief listed 18 — ARCHITECTURE.md governs).
static constexpr uint32_t CRSF_TASK_STACK_BYTES  = 4096u;
static constexpr UBaseType_t CRSF_TASK_PRIORITY  = 14u;
static constexpr BaseType_t  CRSF_TASK_CORE      = 1;

// ── Telemetry window detection ────────────────────────────────────────────────
static constexpr uint32_t CRSF_TELEMETRY_GAP_MS  = 2u;    ///< Inter-packet gap threshold

// ── Shared global state ───────────────────────────────────────────────────────
static CRSFChannels      g_crsf;                ///< Protected by g_mutex
static SemaphoreHandle_t g_mutex  = nullptr;
static volatile bool     g_telemetry_window = false; ///< true when >2ms line silence

// CH8 momentary learn-point edge latch. The edge is detected here at the CRSF
// frame rate (~200 Hz) instead of in the 10 Hz state-machine loop, so a brief
// press is never missed. s_learn_pt_events increments once per completed press
// (high→low release); the FSM consumes it via crsf_get_learn_pt_events().
// Only touched inside process_frame() (CRSF task), except the volatile counter
// which is read lock-free from the FSM (Core 1, same core — atomic word).
static volatile uint32_t g_learn_pt_events = 0;
static bool              s_ch8_high        = false;

// ── CRC8 lookup table (poly 0xD5 = DVB-S2) ───────────────────────────────────
static uint8_t s_crc8_table[256];

// ── Forward declarations ─────────────────────────────────────────────────────
static void     crc8_table_init(void);
static uint8_t  crc8(const uint8_t *data, uint8_t len);
static void     process_frame(uint8_t type, const uint8_t *payload, uint8_t payload_len);
static void     crsf_rx_task(void *param);
static float    clampf_local(float x, float lo, float hi);

// ═════════════════════════════════════════════════════════════════════════════
//  CRC8 — polynomial 0xD5 (DVB-S2)
// ═════════════════════════════════════════════════════════════════════════════

/**
 * Pre-compute the 256-entry CRC8 lookup table for polynomial 0xD5.
 * Called once from crsf_input_init() before the receive task starts.
 */
static void crc8_table_init(void) {
    for (int b = 0; b < 256; b++) {
        uint8_t crc = (uint8_t)b;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1u) ^ 0xD5u)
                                 : (uint8_t)(crc << 1u);
        }
        s_crc8_table[b] = crc;
    }
}

/**
 * Compute CRC8 (poly 0xD5) over a byte buffer.
 * The CRSF CRC is initialised to 0 and covers [type][payload] bytes.
 *
 * @param data  Pointer to input bytes (type byte first, then payload)
 * @param len   Number of bytes to process
 * @return      CRC8 result byte
 */
static uint8_t crc8(const uint8_t *data, uint8_t len) {
    uint8_t crc = 0u;
    for (uint8_t i = 0u; i < len; i++) {
        crc = s_crc8_table[crc ^ data[i]];
    }
    return crc;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Frame processing — called with a validated, CRC-correct frame
// ═════════════════════════════════════════════════════════════════════════════

/**
 * Process one complete, CRC-validated CRSF frame.
 *
 * For type 0x16 (RC_CHANNELS_PACKED): unpacks CH1–CH8 from the 11-bit packed
 *   payload, converts to microseconds, and updates g_crsf under the mutex.
 * For type 0x1C (FAILSAFE): sets g_crsf.failsafe = true immediately.
 * All other types are silently ignored.
 *
 * Bit extraction uses a 3-byte sliding window so that channels whose 11-bit span
 * crosses three bytes are correctly decoded:
 *   CH2 starts at bit 22 (byte 2, offset 6) → spans bytes 2, 3, 4
 *   CH5 starts at bit 55 (byte 6, offset 7) → spans bytes 6, 7, 8
 * A 2-byte window would yield wrong values for these channels.
 *
 * @param type         CRSF frame type byte
 * @param payload      Pointer to payload bytes (after type, before CRC)
 * @param payload_len  Number of payload bytes
 */
static void process_frame(uint8_t type, const uint8_t *payload, uint8_t payload_len) {
    uint32_t now = millis();

    if (type == CRSF_TYPE_FAILSAFE) {
        // Explicit failsafe frame — set flag immediately, do not update channels
        if (xSemaphoreTake(g_mutex, portMAX_DELAY) == pdTRUE) {
            g_crsf.failsafe = true;
            xSemaphoreGive(g_mutex);
        }
        return;
    }

    if (type != CRSF_TYPE_RC_CHANNELS) {
        return; // Ignore GPS, attitude, battery, custom frames — not our concern here
    }

    if (payload_len != CRSF_RC_PAYLOAD_BYTES) {
        return; // Malformed RC frame — wrong payload length, discard silently
    }

    // ── Unpack 11-bit channels from packed payload ────────────────────────────
    // Channel i occupies bits [i*11 .. i*11+10] in the packed bitstream.
    // byte index n = i*11 / 8; bit offset within byte n: shift = i*11 % 8.
    // We read three bytes (payload[n..n+2]) into a 32-bit word and shift right
    // by 'shift' to align the LSB, then mask with 0x7FF (11 bits).
    // Using a 3-byte window correctly handles CH2 (offset=6) and CH5 (offset=7)
    // which each span three consecutive bytes.
    uint16_t raw[8];
    for (int i = 0; i < 8; i++) {
        const uint16_t n     = (uint16_t)((unsigned)(i * 11) / 8u);
        const uint8_t  shift = (uint8_t)((unsigned)(i * 11) % 8u);
        // n+2 ≤ 11 for i≤7, safely within the 22-byte payload (indices 0..21).
        uint32_t word = (uint32_t)payload[n]
                      | ((uint32_t)payload[n + 1u] << 8u)
                      | ((uint32_t)payload[n + 2u] << 16u);
        raw[i] = (uint16_t)((word >> shift) & 0x7FFu);
    }

    // ── Convert to microseconds and store under mutex ─────────────────────────
    if (xSemaphoreTake(g_mutex, portMAX_DELAY) == pdTRUE) {
        for (int i = 0; i < 8; i++) {
            g_crsf.ch[i] = crsf_raw_to_us(raw[i]);
        }
        g_crsf.failsafe      = false;
        g_crsf.last_frame_ms = now;

        // CH8 momentary learn-point: count a completed press on release
        // (high→low). 1500±30 µs hysteresis mirrors the FSM's sw2_decode, so
        // the falling-edge "record on release" semantics are preserved — just
        // sampled fast enough that short taps are no longer dropped.
        const uint16_t ch8 = g_crsf.ch[CRSF_CH_LEARN_PT];
        if (ch8 > 1530u) {
            s_ch8_high = true;
        } else if (ch8 < 1470u) {
            if (s_ch8_high) g_learn_pt_events++;
            s_ch8_high = false;
        }
        xSemaphoreGive(g_mutex);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  FreeRTOS receive task
// ═════════════════════════════════════════════════════════════════════════════

/**
 * CRSF frame receiver task — runs forever on Core 1, priority 14.
 *
 * State machine:
 *   WAIT_SYNC   → wait for sync byte 0xC8
 *   WAIT_LEN    → read and validate length (2 ≤ len ≤ 64)
 *   WAIT_TYPE   → read frame type; compute expected payload length = len - 2
 *   WAIT_PAYLOAD→ accumulate payload bytes
 *   WAIT_CRC    → read CRC byte; validate CRC8(type+payload); call process_frame()
 *
 * Additional responsibilities:
 *   - Inter-packet gap: if Serial2 has no bytes for >2 ms → set telemetry window flag
 *   - Failsafe timeout: if no valid frame for RC_FAILSAFE_TIMEOUT_MS → set failsafe
 *   - Yields (vTaskDelay 1 tick) when the RX buffer is empty to avoid busy-wait
 */
static void crsf_rx_task(void *param) {
    (void)param;

    // Parser state
    typedef enum {
        WAIT_SYNC,
        WAIT_LEN,
        WAIT_TYPE,
        WAIT_PAYLOAD,
        WAIT_CRC
    } ParseState;

    ParseState state            = WAIT_SYNC;
    uint8_t    frame_len        = 0u;    // the 'len' field from the frame
    uint8_t    frame_type       = 0u;    // the 'type' field
    uint8_t    payload_buf[CRSF_MAX_FRAME_LEN]; // payload accumulation buffer
    uint8_t    payload_pos      = 0u;    // bytes received so far into payload_buf
    uint8_t    payload_expected = 0u;    // len - 2 (type removed, CRC not in buf)

    uint32_t   last_byte_ms     = millis();

    for (;;) {
        bool got_any = false;

        // Drain all available bytes from Serial2 in one shot before yielding
        while (Serial2.available() > 0) {
            int b = Serial2.read();
            if (b < 0) break;

            last_byte_ms        = millis();
            g_telemetry_window  = false; // reset gap flag on any received byte
            got_any             = true;

            const uint8_t byte = (uint8_t)b;

            switch (state) {

                case WAIT_SYNC:
                    if (byte == CRSF_SYNC_BYTE) {
                        state = WAIT_LEN;
                    }
                    // Any non-sync byte is silently discarded — we stay in WAIT_SYNC.
                    break;

                case WAIT_LEN:
                    if (byte < 2u || byte > CRSF_MAX_FRAME_LEN) {
                        // Implausible length — the previous sync was a false positive.
                        // Return to sync search without consuming another byte as sync.
                        state = WAIT_SYNC;
                    } else {
                        frame_len = byte;
                        state     = WAIT_TYPE;
                    }
                    break;

                case WAIT_TYPE:
                    frame_type       = byte;
                    payload_expected = frame_len - 2u; // len = type(1) + payload(N) + CRC(1)
                    payload_pos      = 0u;
                    state            = (payload_expected == 0u) ? WAIT_CRC : WAIT_PAYLOAD;
                    break;

                case WAIT_PAYLOAD:
                    payload_buf[payload_pos++] = byte;
                    if (payload_pos >= payload_expected) {
                        state = WAIT_CRC;
                    }
                    break;

                case WAIT_CRC: {
                    // 'byte' is the CRC byte. Build [type][payload...] and verify.
                    // We need a contiguous buffer: prepend type to payload_buf copy.
                    uint8_t crc_input[CRSF_MAX_FRAME_LEN + 1u];
                    crc_input[0] = frame_type;
                    if (payload_expected > 0u) {
                        memcpy(&crc_input[1], payload_buf, payload_expected);
                    }
                    const uint8_t computed = crc8(crc_input, 1u + payload_expected);

                    if (computed == byte) {
                        process_frame(frame_type, payload_buf, payload_expected);
                    }
                    // Always return to SYNC regardless of CRC result
                    state = WAIT_SYNC;
                    break;
                }

            } // switch
        } // while Serial2.available

        // ── Inter-packet gap check ────────────────────────────────────────────
        const uint32_t now = millis();
        if (!g_telemetry_window && (now - last_byte_ms) > CRSF_TELEMETRY_GAP_MS) {
            g_telemetry_window = true;   // latch until next received byte clears it
            crsf_telemetry_service();    // transmit exactly one frame per gap
        }

        // ── Failsafe timeout check ────────────────────────────────────────────
        // Use a non-blocking mutex attempt so we never block the task here.
        if (xSemaphoreTake(g_mutex, 0) == pdTRUE) {
            if (!g_crsf.failsafe &&
                (now - g_crsf.last_frame_ms) > (uint32_t)RC_FAILSAFE_TIMEOUT_MS) {
                g_crsf.failsafe = true;
            }
            xSemaphoreGive(g_mutex);
        }

        // ── Yield when nothing to do ──────────────────────────────────────────
        if (!got_any) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    } // for(;;)
}

// ═════════════════════════════════════════════════════════════════════════════
//  Public API implementation
// ═════════════════════════════════════════════════════════════════════════════

void crsf_input_init() {
    // Build CRC8 lookup table (must happen before the task starts)
    crc8_table_init();

    // Initialise global state — failsafe is false; will arm after timeout if no frame
    memset(&g_crsf, 0, sizeof(g_crsf));
    g_crsf.failsafe      = false;
    g_crsf.last_frame_ms = millis();

    // Create the mutex that protects g_crsf
    g_mutex = xSemaphoreCreateMutex();
    configASSERT(g_mutex != nullptr);

    // Open Serial2: CRSF 420 000 baud, 8N1, RX=GPIO11, TX=GPIO12 (ARCHITECTURE.md §3)
    Serial2.begin(CRSF_BAUD_RATE, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);

    // Start the receive task pinned to Core 1
    BaseType_t ret = xTaskCreatePinnedToCore(
        crsf_rx_task,           // Task function
        "crsf_rx_task",         // Task name
        CRSF_TASK_STACK_BYTES,  // Stack size in bytes
        nullptr,                // Task parameter (none)
        CRSF_TASK_PRIORITY,     // Priority 14 (ARCHITECTURE.md §2)
        nullptr,                // Task handle (not needed)
        CRSF_TASK_CORE          // Core 1
    );
    configASSERT(ret == pdPASS);
}

CRSFChannels crsf_get_channels() {
    CRSFChannels snapshot;
    if (xSemaphoreTake(g_mutex, portMAX_DELAY) == pdTRUE) {
        snapshot = g_crsf;           // struct copy under mutex
        xSemaphoreGive(g_mutex);
    } else {
        // Fallback: return failsafe state if mutex cannot be acquired (should not occur)
        memset(&snapshot, 0, sizeof(snapshot));
        snapshot.failsafe = true;
    }
    return snapshot;
}

uint32_t crsf_get_learn_pt_events() {
    return g_learn_pt_events;   // volatile 32-bit read; lock-free is safe on Core 1
}

bool crsf_is_failsafe() {
    bool fs = true;
    if (xSemaphoreTake(g_mutex, portMAX_DELAY) == pdTRUE) {
        fs = g_crsf.failsafe;
        // Also check timeout inline — ensures callers always see the latest state
        if (!fs && (millis() - g_crsf.last_frame_ms) > (uint32_t)RC_FAILSAFE_TIMEOUT_MS) {
            g_crsf.failsafe = true;
            fs = true;
        }
        xSemaphoreGive(g_mutex);
    }
    return fs;
}

uint16_t crsf_raw_to_us(uint16_t raw) {
    // Map CRSF raw range [172, 1811] linearly to microseconds [1000, 2000].
    // Clamp inputs outside the nominal range to avoid overflow.
    if (raw <= 172u)  return 1000u;
    if (raw >= 1811u) return 2000u;
    // Integer arithmetic: (raw - 172) * 1000 / 1639 + 1000
    // Max intermediate: (1811-172)*1000 = 1 639 000 — fits in uint32_t.
    return (uint16_t)(((uint32_t)(raw - 172u) * 1000u) / 1639u + 1000u);
}

float crsf_us_to_norm(uint16_t us_value) {
    // Centre (1500 µs) = 0.0; full throw (1000 or 2000 µs) = ±1.0.
    float norm = clampf_local((us_value - 1500.0f) / 500.0f, -1.0f, 1.0f);
    // Dead band: ±50 µs around centre → |norm| < 0.10 at 1450..1550 µs.
    // Task spec uses 0.05 threshold (matches ±25 µs at 500 µs/unit scale).
    if (norm > -0.05f && norm < 0.05f) {
        norm = 0.0f;
    }
    return norm;
}

bool crsf_telemetry_window_open() {
    // g_telemetry_window is volatile bool — single-byte access is atomic on ESP32-S3.
    return g_telemetry_window;
}

// ═════════════════════════════════════════════════════════════════════════════
//  Internal utilities
// ═════════════════════════════════════════════════════════════════════════════

/**
 * Clamp x to [lo, hi]. Defined locally to avoid dependency on a common utility
 * header that may not yet exist at this stage of the build.
 */
static float clampf_local(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}
