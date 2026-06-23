#pragma once
#include <Arduino.h>

// ══════════════════════════════════════════════════════════════════════════════
//  servo_output.h — RoboMower cut-height servo controller
//
//  Hardware: SERVO_PIN (GPIO5), LEDC Timer 1 / Channel 1, 50 Hz, 14-bit.
//  Range:    1000 µs = minimum deck height, 2000 µs = maximum deck height.
//            The servo is mechanically calibrated — no software calibration.
// ══════════════════════════════════════════════════════════════════════════════

// ── LEDC configuration ────────────────────────────────────────────────────────
#define SERVO_LEDC_FREQ_HZ   50   ///< PWM frequency (50 Hz = 20 ms period)
#define SERVO_LEDC_RES_BITS  14   ///< Resolution bits (ESP32-S3 LEDC max)

// ── Absolute clamp ────────────────────────────────────────────────────────────
#define SERVO_ABS_MAX_US  2500   ///< Hard maximum — hardware safety clamp

// ── Slew and backoff ──────────────────────────────────────────────────────────
#define SERVO_SLEW_US_PER_TICK  20  ///< Max pulse-width change per 100 ms tick (µs)
#define SERVO_BACKOFF_PCT        3  ///< Pulse backoff % at max-height end to relieve strain


/**
 * @brief Initialise LEDC peripheral and start at maximum deck height (1000 µs).
 */
void servo_output_init();

/**
 * @brief Move the deck to the requested height (mm), with slew-rate limiting.
 *
 * Maps height_mm linearly from [CUT_HEIGHT_MIN_MM, CUT_HEIGHT_MAX_MM]
 * to [1000, 2000] µs.  Input is clamped before conversion.
 *
 * @param height_mm  Target cut height in mm.
 */
void servo_set_height_mm(float height_mm);

/**
 * @brief Compute the servo pulse width for a given cut height (no side effects).
 *
 * @param height_mm  Target height in mm.
 * @return           Pulse width in microseconds.
 */
uint16_t heightToServoPulse(float height_mm);

/**
 * @brief Handle the CALHEIGHT serial command.
 *
 * Only supports `TEST <mm>` — moves the servo to the specified height
 * to verify mechanical operation.
 *
 * @param sub_cmd  String after "CALHEIGHT " (null-terminated).
 */
void servo_handle_cal_command(const char *sub_cmd);

/**
 * @brief Per-tick housekeeping. After cut_height_timeout_s (MowerConfig) of no
 *  commanded movement, stop the PPM so the self-holding actuator de-powers and
 *  stops drawing current. 0 = never time out (continuous PPM). Call every loop tick.
 */
void servo_output_update();
