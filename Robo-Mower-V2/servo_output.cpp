// ══════════════════════════════════════════════════════════════════════════════
//  servo_output.cpp — RoboMower ESP32-S3 Firmware
//  Cut-height servo controller using ESP32 LEDC peripheral.
//
//  The servo is mechanically calibrated: 1000 µs = minimum deck height,
//  2000 µs = maximum deck height.  No software calibration is required.
//
//  Uses the ESP-IDF LEDC API directly (not the Arduino ledcAttach wrappers)
//  to guarantee permanent ownership of Timer 1 / Channel 1.
//  14-bit resolution @ 50 Hz: duty = (pulse_µs × 16383) / 20000.
// ══════════════════════════════════════════════════════════════════════════════

#include "servo_output.h"
#include "config.h"
#include "mower_config.h"
#include <driver/ledc.h>

// ── ESP-IDF LEDC resource assignments ────────────────────────────────────────
#define SERVO_LEDC_SPEED    LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_TIMER    LEDC_TIMER_1
#define SERVO_LEDC_CH       LEDC_CHANNEL_1

// ── Fixed pulse range (mechanically calibrated) ───────────────────────────────
#define SERVO_MIN_US  1000  // minimum deck height
#define SERVO_MAX_US  2000  // maximum deck height

// ── Module state ─────────────────────────────────────────────────────────────
static uint16_t servo_current_pulse_us = 0;
static uint32_t s_servo_last_drive_ms  = 0;     // millis() of last commanded movement
static bool     s_servo_idle           = false; // true = PPM stopped (actuator de-powered)

// ── Helpers ──────────────────────────────────────────────────────────────────

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

#define SERVO_DUTY_MAX  ((1UL << SERVO_LEDC_RES_BITS) - 1UL)   // 16383

static inline uint32_t pulseUsToDuty(uint16_t pulse_us)
{
    return ((uint32_t)pulse_us * SERVO_DUTY_MAX) / 20000UL;
}

static void ledcWritePulse(uint16_t pulse_us)
{
    DBG_PRINTF("[SERVO] pulse=%d us\n", (int)pulse_us);
    ledc_set_duty(SERVO_LEDC_SPEED, SERVO_LEDC_CH, pulseUsToDuty(pulse_us));
    ledc_update_duty(SERVO_LEDC_SPEED, SERVO_LEDC_CH);
    servo_current_pulse_us = pulse_us;
}

static void servo_stop_ppm()
{
    // Duty 0 = constant LOW = no servo pulses → the self-holding actuator de-powers.
    ledc_set_duty(SERVO_LEDC_SPEED, SERVO_LEDC_CH, 0);
    ledc_update_duty(SERVO_LEDC_SPEED, SERVO_LEDC_CH);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public API
// ─────────────────────────────────────────────────────────────────────────────

void servo_output_init()
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = SERVO_LEDC_SPEED,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num       = SERVO_LEDC_TIMER,
        .freq_hz         = (uint32_t)SERVO_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        DBG_PRINTF("[SERVO] ledc_timer_config failed: %d\n", err);
        return;
    }

    // Start at the configured height so the servo doesn't sweep on boot.
    // mower_config_init() is guaranteed to run before servo_output_init().
    uint16_t init_pulse_us = heightToServoPulse(
        (float)mower_config_get().cut_height_max_mm);

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = SERVO_PIN,
        .speed_mode = SERVO_LEDC_SPEED,
        .channel    = SERVO_LEDC_CH,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = SERVO_LEDC_TIMER,
        .duty       = pulseUsToDuty(init_pulse_us),
        .hpoint     = 0,
        .flags      = { .output_invert = 0 },
    };
    err = ledc_channel_config(&ch_cfg);
    if (err != ESP_OK) {
        DBG_PRINTF("[SERVO] ledc_channel_config failed: %d\n", err);
        return;
    }
    servo_current_pulse_us = init_pulse_us;
    s_servo_last_drive_ms = millis();
    s_servo_idle          = false;

    DBG_PRINTF("[SERVO] Ready — GPIO%d, %dHz, 14-bit, range %d–%d µs\n",
                  SERVO_PIN, SERVO_LEDC_FREQ_HZ, SERVO_MIN_US, SERVO_MAX_US);
}

// ─────────────────────────────────────────────────────────────────────────────

uint16_t heightToServoPulse(float height_mm)
{
    float t = (height_mm - (float)CUT_HEIGHT_MIN_MM)
              / (float)(CUT_HEIGHT_MAX_MM - CUT_HEIGHT_MIN_MM);
    t = 1.0f - clampf(t, 0.0f, 1.0f);  // invert: lower height = longer pulse
    return (uint16_t)((float)SERVO_MIN_US + t * (float)(SERVO_MAX_US - SERVO_MIN_US));
}

// ─────────────────────────────────────────────────────────────────────────────

void servo_set_height_mm(float height_mm)
{
    float clamped  = clampf(height_mm, (float)CUT_HEIGHT_MIN_MM, (float)CUT_HEIGHT_MAX_MM);
    uint16_t target_us = heightToServoPulse(clamped);

    // ── Persistent slew state ─────────────────────────────────────────────────
    static uint16_t s_slew_us     = 0;
    static uint16_t s_prev_target = 0;
    static bool     s_backoff     = false;

    if (s_slew_us == 0) {
        s_slew_us     = servo_current_pulse_us;
        s_prev_target = servo_current_pulse_us;
    }

    if (target_us != s_prev_target) {
        if (target_us > s_prev_target) s_backoff = false;
        s_prev_target = target_us;
    }

    // ── Slew step ─────────────────────────────────────────────────────────────
    int32_t step = (int32_t)target_us - (int32_t)s_slew_us;
    bool raising  = (step < 0);
    if (step >  (int32_t)SERVO_SLEW_US_PER_TICK) step =  (int32_t)SERVO_SLEW_US_PER_TICK;
    if (step < -(int32_t)SERVO_SLEW_US_PER_TICK) step = -(int32_t)SERVO_SLEW_US_PER_TICK;
    s_slew_us = (uint16_t)((int32_t)s_slew_us + step);

    if (raising && s_slew_us == target_us) s_backoff = true;

    // ── Backoff at raised end ─────────────────────────────────────────────────
    // Relieves mechanical strain when holding at maximum height.
    uint16_t output_us = s_slew_us;
    if (s_backoff) {
        uint32_t backed = (uint32_t)s_slew_us
                        + (uint32_t)((SERVO_MAX_US - SERVO_MIN_US) * SERVO_BACKOFF_PCT / 100);
        if (backed > SERVO_ABS_MAX_US) backed = SERVO_ABS_MAX_US;
        output_us = (uint16_t)backed;
    }

    // Only (re)drive on an actual movement; ledcWritePulse re-applies the duty, so
    // this also resumes the PPM if it had timed out. When the output is steady, the
    // timeout in servo_output_update() de-powers the actuator to stop it drawing
    // current fighting mechanical stiffness.
    if (output_us != servo_current_pulse_us) {
        ledcWritePulse(output_us);
        s_servo_last_drive_ms = millis();
        s_servo_idle          = false;
    }
}

// ─────────────────────────────────────────────────────────────────────────────

void servo_handle_cal_command(const char *sub_cmd)
{
    if (strncmp(sub_cmd, "TEST", 4) == 0) {
        if (strlen(sub_cmd) < 6) {
            DBG_PRINTLN("[SERVO] Usage: CALHEIGHT TEST <height_mm>");
            return;
        }
        float mm = atof(sub_cmd + 5);
        servo_set_height_mm(mm);
        DBG_PRINTF("[SERVO] Test: %.0f mm → %d µs\n", mm, heightToServoPulse(mm));
    } else {
        DBG_PRINTLN("[SERVO] Usage: CALHEIGHT TEST <mm>");
    }
}

// ─────────────────────────────────────────────────────────────────────────────

void servo_output_update()
{
    if (s_servo_idle) return;                          // already de-powered
    float timeout_s = (float)mower_config_get().cut_height_timeout_s;
    if (timeout_s <= 0.0f) return;                     // 0 = continuous PPM
    if (s_servo_last_drive_ms == 0) return;            // nothing driven yet
    if ((uint32_t)(millis() - s_servo_last_drive_ms) >= (uint32_t)(timeout_s * 1000.0f)) {
        servo_stop_ppm();
        s_servo_idle = true;
        DBG_PRINTF("[SERVO] PPM off after %.1fs hold (actuator self-holds)\n", (double)timeout_s);
    }
}
