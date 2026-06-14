// ══════════════════════════════════════════════════════════════════════════════
//  vesc_can.cpp — RoboMower VESC CAN Bus Interface Implementation
//
//  See vesc_can.h for API documentation.
//
//  Design notes:
//  1. VESC uses 29-bit EXTENDED CAN frames.  The task description says "11-bit"
//     but (packet_id=9) << 8 = 0x900 = 2304 which exceeds the 11-bit standard
//     frame limit of 0x7FF = 2047.  Actual VESC BLDC firmware (comm_can.c) uses
//     extended identifiers.  See HANDOFFS/06_vesc_can/HANDOFF.md for details.
//
//  2. Tachometer (VescOdometry.tach_raw) is accumulated from ERPM integration
//     on every CAN_PACKET_STATUS frame (50 Hz), not from a GET_VALUES response.
//     vesc_poll_tachometer() sends NO CAN frame; it only records a poll
//     timestamp. See HANDOFF for details.
//
//  3. E-stop (vesc_emergency_stop_all) uses xQueueSendToFront — it does NOT
//     bypass the TX queue or write directly to TWAI hardware.  Per A17, the
//     physical PILZ relay cuts main bus power independently; the software path
//     only needs to overtake other queued frames, which xQueueSendToFront achieves.
// ══════════════════════════════════════════════════════════════════════════════

#include "vesc_can.h"
#include "config.h"
#include "mower_config.h"
#include "odo_calib.h"
#include "sys_log.h"

#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ── VESC CAN packet IDs ───────────────────────────────────────────────────────
// Packet IDs verified against VESC firmware 6.x comm_can.h:
//   0=SET_DUTY, 1=SET_CURRENT, 2=SET_CURRENT_BRAKE, 3=SET_RPM, 4=SET_POS,
//   5=FILL_RX_BUFFER (long packet), 9=STATUS
// The original spec document had SET_CURRENT=5 which is incorrect and explains
// why drive commands were ACKed but silently ignored by the VESC.
static constexpr uint8_t PKT_SET_DUTY    = 0;   ///< SET_DUTY   — 4-byte signed (duty × 100000)
static constexpr uint8_t PKT_SET_RPM     = 3;   ///< SET_RPM    — 4-byte signed ERPM
static constexpr uint8_t PKT_SET_CURRENT = 1;   ///< SET_CURRENT — 4-byte signed mA (÷1000 = Amps)
static constexpr uint8_t PKT_STATUS      = 9;   ///< STATUS      — 8-byte broadcast
static constexpr uint8_t PKT_STATUS_5   = 27;  ///< STATUS_5    — v_in, tacho (broadcast)

// ── Internal TX frame descriptor ─────────────────────────────────────────────
struct VescTxFrame {
    uint8_t  vesc_id;     ///< VESC_ID_LEFT, _RIGHT, or _BLADE
    uint8_t  packet_id;   ///< PKT_SET_RPM / PKT_GET_VALUES / PKT_SET_CURRENT
    int32_t  value;       ///< ERPM (SET_RPM), mA (SET_CURRENT), unused (GET_VALUES)
};

// ── Module-private state ──────────────────────────────────────────────────────
static VescStatus        s_status[3];          // index = vesc_id - 1
static VescOdometry      s_odom[2];            // index = vesc_id - 1 (LEFT=0, RIGHT=1)
static float             s_tach_accum[2];      // float accumulator before int32 snap
static uint32_t          s_last_status_ms[2];  // timestamp of last STATUS rx per drive

static float             s_battery_voltage_v  = 0.0f;  ///< Input voltage from STATUS_5 (VESC_ID_BLADE)
static uint32_t          s_last_status5_ms    = 0;     ///< millis() of last STATUS_5 frame

static bool              s_bus_live           = false; ///< true after first STATUS frame from any VESC
static uint32_t          s_bus_live_at_ms     = 0;     ///< millis() when bus first went live

static SemaphoreHandle_t s_mutex = nullptr;

// Exported queue handle (extern declared in vesc_can.h)
QueueHandle_t xQueueVescTx = nullptr;

// ── Big-endian encode / decode helpers ───────────────────────────────────────

/** Pack a signed 32-bit value into 4 bytes, big-endian. */
static inline void encode_i32_be(uint8_t *buf, int32_t v) {
    buf[0] = (uint8_t)((v >> 24) & 0xFF);
    buf[1] = (uint8_t)((v >> 16) & 0xFF);
    buf[2] = (uint8_t)((v >>  8) & 0xFF);
    buf[3] = (uint8_t)( v        & 0xFF);
}

/** Read a signed 32-bit value from 4 bytes, big-endian. */
static inline int32_t decode_i32_be(const uint8_t *buf) {
    return (int32_t)(((uint32_t)buf[0] << 24) |
                     ((uint32_t)buf[1] << 16) |
                     ((uint32_t)buf[2] <<  8) |
                      (uint32_t)buf[3]);
}

/** Read a signed 16-bit value from 2 bytes, big-endian. */
static inline int16_t decode_i16_be(const uint8_t *buf) {
    return (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

// ── Conversion formulas (from spec) ──────────────────────────────────────────

bool vesc_can_bus_ok() {
    twai_status_info_t info;
    if (twai_get_status_info(&info) != ESP_OK) return false;

    // If the bus has gone offline due to wiring faults, attempt recovery so
    // the firmware self-heals when the transceiver is reconnected.
    if (info.state == TWAI_STATE_BUS_OFF) {
        DBG_PRINTLN("[VESC] TWAI BUS_OFF — initiating recovery");
        twai_initiate_recovery();   // begins 128-bit recovery sequence
        return false;
    }
    if (info.state == TWAI_STATE_RECOVERING) {
        return false;               // recovery in progress; will return RUNNING when done
    }
    if (info.state == TWAI_STATE_STOPPED) {
        twai_start();               // restart if somehow stopped (e.g. failed init retry)
        return false;
    }

    return info.state == TWAI_STATE_RUNNING;
}

float vesc_erpm_to_velocity(float erpm) {
    // vel = (erpm / pole_pairs / 60) / gear_ratio * 2π * wheel_radius
    const MowerConfig &c = mower_config_get();
    return (erpm / (float)c.motor_pole_pairs / 60.0f)
           / c.gear_ratio
           * (2.0f * (float)M_PI * c.wheel_radius_m);
}

float vesc_erpm_to_velocity_scaled(float erpm) {
    // Same conversion, then ×odo_cal_scale() — the GPS-referenced distance
    // calibration. Applied ONLY at the motion-estimate consumers (EKF feed,
    // duty-ramp feedback, follower stall/slip); the raw RX-task odometry stays
    // unscaled. See odo_calib.h.
    return vesc_erpm_to_velocity(erpm) * odo_cal_scale();
}

float vesc_velocity_to_erpm(float velocity_ms) {
    // erpm = velocity_ms * gear_ratio / (2π * wheel_radius) * pole_pairs * 60
    const MowerConfig &c = mower_config_get();
    return velocity_ms
           * c.gear_ratio
           / (2.0f * (float)M_PI * c.wheel_radius_m)
           * (float)c.motor_pole_pairs
           * 60.0f;
}

float vesc_tach_to_distance(int32_t tach_delta) {
    // dist = tach_delta / (pole_pairs * 2) / gear_ratio * 2π * wheel_radius
    const MowerConfig &c = mower_config_get();
    return (float)tach_delta
           / ((float)c.motor_pole_pairs * 2.0f)
           / c.gear_ratio
           * (2.0f * (float)M_PI * c.wheel_radius_m);
}

// ── RX task ───────────────────────────────────────────────────────────────────
// Core 0, Priority 8, Stack 4096 bytes  (ARCHITECTURE.md §2 vescRxTask)

static void task_vesc_can_rx(void *pvParameters) {
    (void)pvParameters;

    twai_message_t msg;

    for (;;) {
        // Wait for a CAN frame, but wake every 100 ms to check bus health.
        // Recovery used to depend on vesc_can_bus_ok() being called from the
        // BLE/WebUI diagnostic path — with no client connected a BUS-OFF was
        // never recovered, and with one connected recovery lagged seconds
        // (observed as the blade cutting out and restarting every few seconds:
        // bus-off → blade VESC's own 1 s command timeout stops the motor →
        // eventual recovery → keepalive resumes → spin-up → repeat).
        if (twai_receive(&msg, pdMS_TO_TICKS(100)) != ESP_OK) {
            twai_status_info_t info;
            if (twai_get_status_info(&info) == ESP_OK) {
                static bool s_bus_was_down = false;
                if (info.state == TWAI_STATE_BUS_OFF) {
                    if (!s_bus_was_down) {
                        s_bus_was_down = true;
                        sys_log_push("CAN BUS-OFF - auto-recovering");
                    }
                    twai_initiate_recovery();
                } else if (info.state == TWAI_STATE_STOPPED) {
                    twai_start();
                } else if (info.state == TWAI_STATE_RUNNING && s_bus_was_down) {
                    s_bus_was_down = false;
                    sys_log_push("CAN bus recovered");
                }
            }
            continue;
        }

        // Accept only 29-bit extended frames from VESC (standard frames ignored)
        if (!msg.extd) {
            continue;
        }

        uint8_t packet_id = (uint8_t)((msg.identifier >> 8) & 0xFF);
        uint8_t vesc_id   = (uint8_t)( msg.identifier       & 0xFF);

        // Validate VESC ID range
        if (vesc_id < 1 || vesc_id > 3) {
            continue;
        }

        uint8_t idx = vesc_id - 1;  // 0-based index into s_status[]

        // ── Decode CAN_PACKET_STATUS (ID=9) ──────────────────────────────────
        // Payload: int32 erpm [0-3], int16 current×10 [4-5], int16 duty×1000 [6-7]
        if (packet_id == PKT_STATUS && msg.data_length_code == 8) {
            int32_t raw_erpm    = decode_i32_be(&msg.data[0]);
            int16_t raw_current = decode_i16_be(&msg.data[4]);
            int16_t raw_duty    = decode_i16_be(&msg.data[6]);

            float new_erpm      = (float)raw_erpm;
            float new_current_A = (float)raw_current / 10.0f;
            float new_duty      = (float)raw_duty    / 1000.0f;
            uint32_t now_ms     = (uint32_t)millis();

            xSemaphoreTake(s_mutex, portMAX_DELAY);

            s_status[idx].erpm           = new_erpm;
            s_status[idx].current_A      = new_current_A;
            s_status[idx].duty           = new_duty;
            s_status[idx].last_update_ms = now_ms;

            if (!s_bus_live) {
                s_bus_live       = true;
                s_bus_live_at_ms = now_ms;
            }

            // ── Odometry integration for drive VESCs (LEFT=0, RIGHT=1) ───────
            // Virtual tachometer: accumulate from ERPM × dt between STATUS frames.
            //
            // Derivation (see spec formulas):
            //   dist = tach / (POLE_PAIRS×2) / GEAR_RATIO × 2π×r
            //   vel  = erpm / POLE_PAIRS / 60 / GEAR_RATIO × 2π×r
            //   → tach_increment = erpm × dt / 30
            //
            // A float accumulator (s_tach_accum) avoids int32 truncation error
            // on individual 20 ms frames; tach_raw snaps to the integer part.
            if (idx < 2) {
                uint32_t prev_ms = s_last_status_ms[idx];
                if (prev_ms > 0 && now_ms > prev_ms) {
                    float dt_s = (float)(now_ms - prev_ms) / 1000.0f;

                    // Guard against spurious large dt (e.g. first frame or gap)
                    if (dt_s < 0.5f) {
                        float tach_inc = new_erpm * dt_s / 30.0f;
                        s_tach_accum[idx] += tach_inc;

                        int32_t new_tach  = (int32_t)s_tach_accum[idx];
                        int32_t prev_tach = s_odom[idx].tach_raw;
                        int32_t delta     = new_tach - prev_tach;

                        s_odom[idx].tach_prev   = prev_tach;
                        s_odom[idx].tach_raw    = new_tach;
                        s_odom[idx].dist_m     += vesc_tach_to_distance(delta);
                        s_odom[idx].velocity_ms = vesc_erpm_to_velocity(new_erpm);
                    }
                }
                s_last_status_ms[idx] = now_ms;
            }

            xSemaphoreGive(s_mutex);
        }
        // ── Decode CAN_PACKET_STATUS_5 (ID=27) from VESC_ID_BLADE only ──────────
        // Battery voltage is sourced from the blade VESC (CAN ID 3) which has
        // STATUS_5 enabled. The drive VESCs (IDs 1 and 2) are older hardware
        // that cannot broadcast STATUS_5.
        // Payload per VESC fw comm_can.c CAN_PACKET_STATUS_5:
        //   int32 tachometer [0-3], int16 v_in×10 [4-5], int16 reserved [6-7]
        // (Previously decoded v_in from bytes [0-1] — that is the TOP of the
        //  tachometer count, which read as -0.1 V on a stationary blade and
        //  latched the battery-LOW blade lockout on every boot.)
        else if (packet_id == PKT_STATUS_5 &&
                 msg.data_length_code == 8 &&
                 vesc_id == VESC_ID_BLADE) {
            int16_t raw_vin = decode_i16_be(&msg.data[4]);
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            s_battery_voltage_v = (float)raw_vin / 10.0f;
            s_last_status5_ms   = millis();
            xSemaphoreGive(s_mutex);
        }
        // Other incoming packet IDs (e.g. multi-frame GET_VALUES response)
        // are not decoded in this implementation — see design note 2 in header.
    }
}

// ── TX task ───────────────────────────────────────────────────────────────────
// Core 0, Priority 9, Stack 2048 bytes  (ARCHITECTURE.md §2 vescTxTask)

static void task_vesc_can_tx(void *pvParameters) {
    (void)pvParameters;

    VescTxFrame frame;

    for (;;) {
        // Block until a frame is available
        if (xQueueReceive(xQueueVescTx, &frame, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        twai_message_t msg = {};
        // 29-bit extended frame: identifier = (packet_id << 8) | vesc_id
        msg.identifier = ((uint32_t)frame.packet_id << 8) | (uint32_t)frame.vesc_id;
        msg.extd       = 1;   // extended (29-bit) frame
        msg.rtr        = 0;   // data frame, not remote

        {
            // SET_RPM and SET_CURRENT: 4-byte big-endian signed int32
            msg.data_length_code = 4;
            encode_i32_be(msg.data, frame.value);
        }

        // Attempt transmission; 10 ms timeout — drop frame if bus is saturated.
        // (The safety task will detect a missing STATUS frame via watchdog.)
        twai_transmit(&msg, pdMS_TO_TICKS(10));
    }
}

// ── Internal enqueue helpers ──────────────────────────────────────────────────

/** Enqueue a TX frame at the back of the queue (normal priority). */
static inline void enqueue_back(uint8_t vesc_id, uint8_t packet_id, int32_t value) {
    VescTxFrame frame = { vesc_id, packet_id, value };
    xQueueSend(xQueueVescTx, &frame, 0);  // non-blocking; drop if full
}

/** Enqueue a TX frame at the FRONT of the queue (E-stop priority). */
static inline void enqueue_front(uint8_t vesc_id, uint8_t packet_id, int32_t value) {
    VescTxFrame frame = { vesc_id, packet_id, value };
    xQueueSendToFront(xQueueVescTx, &frame, 0);  // non-blocking
}

// ── Public API ────────────────────────────────────────────────────────────────

void vesc_can_init(int tx_gpio, int rx_gpio) {
    // Initialise module state
    memset(s_status,        0, sizeof(s_status));
    memset(s_odom,          0, sizeof(s_odom));
    memset(s_tach_accum,    0, sizeof(s_tach_accum));
    memset(s_last_status_ms, 0, sizeof(s_last_status_ms));

    // Create synchronisation primitives
    s_mutex      = xSemaphoreCreateMutex();
    xQueueVescTx = xQueueCreate(8, sizeof(VescTxFrame));

    // Configure TWAI peripheral
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)tx_gpio, (gpio_num_t)rx_gpio, TWAI_MODE_NORMAL);
    // Defaults are 5-deep queues — too small now all three VESCs broadcast
    // STATUS 1–5 (bursts of 15 frames). Deeper queues ride the bursts instead
    // of silently dropping frames when the RX task is briefly preempted.
    g_config.rx_queue_len = 32;
    g_config.tx_queue_len = 16;
    // 250 kbit/s using APB clock (80 MHz) explicitly.
    // BRP=16: bit time = 16 × (1 + 15 + 4) / 80 MHz = 4 µs = 250 kbit/s.
    // VESCs are configured at 250 kbps in VESC Tool (confirmed during commissioning).
    twai_timing_config_t t_config = {
        .clk_src          = TWAI_CLK_SRC_APB,
        .quanta_resolution_hz = 0,
        .brp              = 16,
        .tseg_1           = 15,
        .tseg_2           = 4,
        .sjw              = 3,
        .triple_sampling  = false,
    };
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        DBG_PRINTF("[VESC] twai_driver_install failed: %d\n", err);
        return;
    }
    err = twai_start();
    if (err != ESP_OK) {
        DBG_PRINTF("[VESC] twai_start failed: %d\n", err);
        return;
    }
    DBG_PRINTLN("[VESC] TWAI started OK");

    // Start RX task: Core 0, Priority 8, Stack 4096
    xTaskCreatePinnedToCore(
        task_vesc_can_rx,
        "vesc_can_rx",
        4096,
        nullptr,
        8,          // priority 8 per ARCHITECTURE.md
        nullptr,
        0           // Core 0
    );

    // Start TX task: Core 0, Priority 9, Stack 2048
    xTaskCreatePinnedToCore(
        task_vesc_can_tx,
        "vesc_can_tx",
        2048,
        nullptr,
        9,          // priority 9 per ARCHITECTURE.md
        nullptr,
        0           // Core 0
    );
}

void vesc_set_current(uint8_t vesc_id, int32_t current_mA) {
    enqueue_back(vesc_id, PKT_SET_CURRENT, current_mA);
}

void vesc_set_duty(uint8_t vesc_id, float duty) {
    // VESC expects duty × 100000 as a signed int32
    int32_t val = (int32_t)(duty * 100000.0f);
    enqueue_back(vesc_id, PKT_SET_DUTY, val);
}

void vesc_set_rpm(uint8_t vesc_id, int32_t erpm) {
    enqueue_back(vesc_id, PKT_SET_RPM, erpm);
}

void vesc_emergency_stop_all() {
    // Push three SET_CURRENT=0 frames to the FRONT of the queue so they
    // overtake any pending motion commands.  Send BLADE last so it arrives
    // first (LIFO when pushing to front).
    enqueue_front(VESC_ID_BLADE, PKT_SET_CURRENT, 0);
    enqueue_front(VESC_ID_RIGHT, PKT_SET_CURRENT, 0);
    enqueue_front(VESC_ID_LEFT,  PKT_SET_CURRENT, 0);
}

VescStatus vesc_get_status(uint8_t vesc_id) {
    VescStatus result = {};
    if (vesc_id < 1 || vesc_id > 3) return result;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    result = s_status[vesc_id - 1];
    xSemaphoreGive(s_mutex);
    return result;
}

float vesc_get_battery_voltage()
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    float v = s_battery_voltage_v;
    xSemaphoreGive(s_mutex);
    return v;
}

bool vesc_battery_voltage_stale()
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    // Not stale before first frame (s_last_status5_ms == 0 = startup grace period)
    bool stale = (s_last_status5_ms != 0) &&
                 ((millis() - s_last_status5_ms) > 2000UL);
    xSemaphoreGive(s_mutex);
    return stale;
}

bool vesc_is_stale(uint8_t vesc_id)
{
    if (!s_bus_live) return false;  // no VESC ever seen — full startup grace
    VescStatus st = vesc_get_status(vesc_id);
    if (st.last_update_ms == 0) {
        // VESC never appeared but bus is live; flag once startup window has expired
        return (millis() - s_bus_live_at_ms) > VESC_STARTUP_GRACE_MS;
    }
    return (millis() - st.last_update_ms) > VESC_STATUS_TIMEOUT_MS;
}

bool vesc_blade_is_stale()
{
    // Uses a longer timeout than the drive VESCs: the blade broadcasts STATUS_1
    // at a lower rate (RPM control mode) so a generous timeout avoids false alarms.
    if (!s_bus_live) return false;
    VescStatus st = vesc_get_status(VESC_ID_BLADE);
    if (st.last_update_ms == 0) {
        return (millis() - s_bus_live_at_ms) > VESC_STARTUP_GRACE_MS;
    }
    return (millis() - st.last_update_ms) > (uint32_t)BLADE_VESC_TIMEOUT_MS;
}

bool vesc_any_went_offline()
{
    return vesc_is_stale(VESC_ID_LEFT)  ||
           vesc_is_stale(VESC_ID_RIGHT) ||
           vesc_is_stale(VESC_ID_BLADE);
}

bool vesc_all_drive_online()
{
    return !vesc_is_stale(VESC_ID_LEFT) && !vesc_is_stale(VESC_ID_RIGHT);
}

VescOdometry vesc_get_odometry(uint8_t vesc_id) {
    VescOdometry result = {};
    if (vesc_id < 1 || vesc_id > 2) return result;  // blade has no odometry

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    result = s_odom[vesc_id - 1];
    xSemaphoreGive(s_mutex);
    return result;
}

void vesc_poll_tachometer(uint8_t vesc_id) {
    if (vesc_id < 1 || vesc_id > 2) return;  // drive VESCs only

    // Record poll time only — no CAN frame sent.
    // The original spec used ID=4 as GET_VALUES, but in VESC firmware 6.x
    // ID=4 is SET_POS, which would accidentally command position mode.
    // Tachometer is derived from ERPM integration in the STATUS RX path,
    // so no explicit request is needed.
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_odom[vesc_id - 1].last_poll_ms = (uint32_t)millis();
    xSemaphoreGive(s_mutex);
}
