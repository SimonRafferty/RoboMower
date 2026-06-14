// ══════════════════════════════════════════════════════════════════════════════
//  cutting_monitor.cpp — RoboMower Blade Load & Cutting Condition Monitor
//
//  Implements 10 Hz cutting condition assessment with:
//    • Circular-buffer rolling average of voltage-compensated blade current
//    • Immediate BLADE_FAULT and CUTTING_STALLED detection
//    • Hysteresis timers for CUTTING_OVERLOADED and OBSTACLE_SUSPECTED
//    • Auto-calibration via P90 of current samples, saved to NVS
//
//  References:
//    ASSUMPTIONS.md A03 (constants), A20 (runtime blade_max_A), A23 (voltage)
//    HANDOFFS/06_vesc_can/HANDOFF.md  (VescStatus.current_A field)
//    HANDOFFS/10_nvs_storage/HANDOFF.md (NVS key "blade_cal")
// ══════════════════════════════════════════════════════════════════════════════

#include "cutting_monitor.h"
#include "config.h"
#include "nvs_storage.h"

#include <Arduino.h>
#include <cmath>
#include <algorithm>   // std::sort
#include <vector>      // std::vector (calibration only — freed after cal)

#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"


// ── Rolling average buffer ────────────────────────────────────────────────────
// 8 samples at 10 Hz = 0.8 s window, covering the 500 ms assessment window
// with margin.  Power-of-2 size makes the modulo a single AND on most targets.
#define ROLLING_BUF_SIZE  8

static float    g_blade_buf[ROLLING_BUF_SIZE];  // circular buffer of compensated current (A)
static int      g_buf_head  = 0;                // next write index
static bool     g_buf_full  = false;            // true after first full revolution
static float    g_rolling_avg = 0.0f;           // current rolling average (A, compensated)

// ── Runtime blade reference current ──────────────────────────────────────────
// Initialised from NVS on boot; updated by auto-calibration.
static float    g_blade_max_A = BLADE_MAX_EXPECTED_CURRENT_A_DEFAULT;

// ── Confirmed cutting status ──────────────────────────────────────────────────
static CuttingStatus g_confirmed_status = CUTTING_NORMAL;

// Spinlock protecting all getters (called from multiple tasks/cores)
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

// ── Hysteresis timers ─────────────────────────────────────────────────────────
static uint32_t g_overload_start_ms  = 0;  // millis() when OVERLOADED was first seen
static bool     g_overload_pending   = false;
static uint32_t g_obstacle_start_ms  = 0;  // millis() when OBSTACLE_SUSPECTED first seen
static bool     g_obstacle_pending   = false;

// ── Auto-calibration state ────────────────────────────────────────────────────
static bool              g_cal_active    = false;
static bool              g_cal_complete  = false;
static uint32_t          g_cal_start_ms  = 0;
static std::vector<float> g_cal_samples;  // temporary; cleared after calibration


// ── Internal helpers ──────────────────────────────────────────────────────────

/** Push one compensated current sample into the circular buffer and recompute
 *  the rolling average.  Must be called on Core 1 only (no locking needed here
 *  as g_rolling_avg is published atomically via the spinlock in the getter). */
static void pushSample(float compensated_A) {
    g_blade_buf[g_buf_head] = compensated_A;
    g_buf_head = (g_buf_head + 1) % ROLLING_BUF_SIZE;
    if (g_buf_head == 0) g_buf_full = true;

    int count = g_buf_full ? ROLLING_BUF_SIZE : g_buf_head;
    float sum = 0.0f;
    for (int i = 0; i < count; i++) sum += g_blade_buf[i];
    g_rolling_avg = (count > 0) ? (sum / count) : 0.0f;
}

/** Determine the raw (pre-hysteresis) cutting status from current sensor data.
 *
 *  blade_current_A is the RAW current (used only for BLADE_FAULT detection,
 *  which must react immediately without rolling-average delay).
 *  g_rolling_avg already holds voltage-compensated current. */
/** millis() when blade_commanded last went false→true; 0 = not commanded.
 *  Used to suppress BLADE_FAULT during physical spin-up: the blade is RPM-
 *  controlled (SET_RPM) and the VESC's internal current-limited RPM PID does a
 *  ~2 s spin-up, so eRPM is legitimately near zero at first. The fault is gated
 *  on BLADE_FAULT_GRACE_MS after arm. Without this grace the fault fired on the
 *  very first commanded tick (eRPM 0 vs full target) and AUTO paused before the
 *  blade ever moved. */
static uint32_t g_blade_cmd_since_ms = 0;

static CuttingStatus assessCuttingCondition(
        float fused_velocity,
        float blade_current_A,
        float blade_actual_erpm,
        float blade_target_erpm,
        bool  blade_commanded)
{
    (void)blade_target_erpm;  // blade is RPM-controlled (SET_RPM); VESC's RPM PID caps current

    // ── BLADE_FAULT: uses raw current (not rolling average) ────────────────
    // Fault = commanded, spin-up grace elapsed, and the motor is drawing no
    // current AND not turning. The old target-eRPM comparison is gone: the
    // VESC's internal current-limited RPM PID handles the SET_RPM spin-up, so
    // there is no firmware RPM target to compare against here.
    if (blade_commanded && g_blade_cmd_since_ms != 0
        && (millis() - g_blade_cmd_since_ms) > BLADE_FAULT_GRACE_MS) {
        if (blade_current_A < (BLADE_LOAD_MIN * BLADE_CURRENT_LIMIT_A)
            && fabsf(blade_actual_erpm) < BLADE_FAULT_MIN_ERPM) {
            return BLADE_FAULT;
        }
    }

    // ── All other conditions: use rolling average of compensated current ───
    // Load is fraction of the fixed VESC current limit (BLADE_CURRENT_LIMIT_A),
    // NOT the auto-calibrated g_blade_max_A — the auto-cal captured idle current
    // as 100% and made everything read overloaded. See config.h (2026-06-13).
    bool  moving = (fabsf(fused_velocity) > STALL_SPEED_THRESHOLD_MS);
    float load   = g_rolling_avg / BLADE_CURRENT_LIMIT_A;

    if ( moving && load < BLADE_LOAD_HIGH)  return CUTTING_NORMAL;
    if ( moving && load >= BLADE_LOAD_HIGH) return CUTTING_OVERLOADED;
    if (!moving && load >= BLADE_LOAD_HIGH) return CUTTING_STALLED;
    if (!moving && load < BLADE_LOAD_HIGH)  return OBSTACLE_SUSPECTED;

    return CUTTING_NORMAL;  // unreachable, but satisfies the compiler
}

/** Update auto-calibration with one raw (uncompensated) current sample.
 *  Voltage compensation is intentionally NOT applied here: calibration runs
 *  at session start when the battery is well-charged, so raw current at that
 *  voltage becomes the reference.  The voltage compensation in assessCutting
 *  then normalises subsequent readings back to this full-charge baseline. */
static void updateCalibration(float blade_current_A) {
    if (!g_cal_active) return;

    g_cal_samples.push_back(blade_current_A);

    uint32_t elapsed = millis() - g_cal_start_ms;
    if (elapsed >= BLADE_CAL_WARMUP_MS && g_cal_samples.size() >= 10) {
        std::sort(g_cal_samples.begin(), g_cal_samples.end());
        // P90: for N sorted samples (0-indexed), the 90th-percentile element
        // is at index floor(N * 0.90). Clamp to [0, N-1] for safety.
        int p90_idx = (int)(g_cal_samples.size() * 0.90f);
        if (p90_idx >= (int)g_cal_samples.size())
            p90_idx = (int)g_cal_samples.size() - 1;
        float measured_max = g_cal_samples[p90_idx];

        if (measured_max > BLADE_CAL_MIN_VALID_A) {
            g_blade_max_A = measured_max;
            nvs_set_float("blade_cal", measured_max);
            DBG_PRINTF("[BLADE CAL] Reference set to %.2f A\n", measured_max);
        } else {
            DBG_PRINTF("[BLADE CAL] %.2f A too low — retaining %.2f A\n",
                          measured_max, g_blade_max_A);
        }

        // Release memory and mark complete
        g_cal_samples.clear();
        g_cal_samples.shrink_to_fit();
        g_cal_active   = false;
        g_cal_complete = true;
    }
}


// ── Public API ────────────────────────────────────────────────────────────────

void cutting_monitor_init() {
    // Load calibrated reference from NVS (falls back to compile-time default)
    g_blade_max_A = nvs_get_float("blade_cal", BLADE_MAX_EXPECTED_CURRENT_A_DEFAULT);
    DBG_PRINTF("[CUTTING_MON] blade_max_A loaded: %.2f A\n", g_blade_max_A);

    // Reset circular buffer
    for (int i = 0; i < ROLLING_BUF_SIZE; i++) g_blade_buf[i] = 0.0f;
    g_buf_head    = 0;
    g_buf_full    = false;
    g_rolling_avg = 0.0f;

    // Reset hysteresis state
    g_overload_start_ms = 0;
    g_overload_pending  = false;
    g_obstacle_start_ms = 0;
    g_obstacle_pending  = false;

    // Reset calibration state
    g_cal_active   = false;
    g_cal_complete = false;
    g_cal_samples.clear();

    portENTER_CRITICAL(&g_mux);
    g_confirmed_status = CUTTING_NORMAL;
    portEXIT_CRITICAL(&g_mux);
}

void cutting_monitor_update(float fused_velocity,
                            float blade_current_A,
                            float blade_actual_erpm,
                            float blade_target_erpm,
                            bool  blade_commanded,
                            float battery_voltage_V)
{
    // Track when the blade was commanded on (for the spin-up fault grace)
    static bool s_prev_commanded = false;
    if (blade_commanded && !s_prev_commanded) {
        g_blade_cmd_since_ms = millis();
    } else if (!blade_commanded) {
        g_blade_cmd_since_ms = 0;
    }
    s_prev_commanded = blade_commanded;

    // ── Step 1: Voltage-compensated current ───────────────────────────────
    // Nominal voltage = 13S full charge = 13 × 4.2V = 54.6V.
    // As the battery drains, the same mechanical load draws proportionally
    // more current; scaling by (nominal/actual) normalises back to the
    // full-charge reference used during auto-calibration.
    // Only apply when battery is above BATTERY_LOW_V (motors stop below that).
    static const float k_nominal_voltage = 54.6f;  // 13S × 4.2V

    float v_scale = 1.0f;
    if (battery_voltage_V > BATTERY_LOW_V && battery_voltage_V < k_nominal_voltage) {
        v_scale = k_nominal_voltage / battery_voltage_V;  // e.g. 1.14 at 48V
    }
    float compensated_current = blade_current_A * v_scale;

    // ── Step 2: Push into rolling buffer ─────────────────────────────────
    pushSample(compensated_current);

    // ── Step 3 & 4: Assess raw (pre-hysteresis) status ───────────────────
    // BLADE_FAULT check uses raw current; all others use g_rolling_avg.
    CuttingStatus raw = assessCuttingCondition(
            fused_velocity,
            blade_current_A,     // raw for BLADE_FAULT detection
            blade_actual_erpm,
            blade_target_erpm,
            blade_commanded);

    // ── Step 5: Apply hysteresis timers ──────────────────────────────────
    uint32_t now = millis();
    CuttingStatus confirmed;

    switch (raw) {
        case BLADE_FAULT:
            // Immediate — no confirmation delay
            confirmed = BLADE_FAULT;
            g_overload_pending = false;
            g_obstacle_pending = false;
            break;

        case CUTTING_STALLED:
            // Immediate — no confirmation delay
            confirmed = CUTTING_STALLED;
            g_overload_pending = false;
            g_obstacle_pending = false;
            break;

        case CUTTING_OVERLOADED:
            // Must persist for BLADE_LOAD_SAMPLE_WINDOW_MS before promoting
            g_obstacle_pending = false;
            if (!g_overload_pending) {
                g_overload_pending  = true;
                g_overload_start_ms = now;
                // While pending, keep previous confirmed status
                confirmed = g_confirmed_status;
            } else if ((now - g_overload_start_ms) >= BLADE_LOAD_SAMPLE_WINDOW_MS) {
                confirmed = CUTTING_OVERLOADED;
            } else {
                confirmed = g_confirmed_status;  // still waiting
            }
            break;

        case OBSTACLE_SUSPECTED:
            // Must persist for OBSTACLE_CONFIRM_MS before promoting
            g_overload_pending = false;
            if (!g_obstacle_pending) {
                g_obstacle_pending  = true;
                g_obstacle_start_ms = now;
                confirmed = g_confirmed_status;
            } else if ((now - g_obstacle_start_ms) >= OBSTACLE_CONFIRM_MS) {
                confirmed = OBSTACLE_SUSPECTED;
            } else {
                confirmed = g_confirmed_status;  // still waiting
            }
            break;

        case CUTTING_NORMAL:
        default:
            // Clear any pending hysteresis
            g_overload_pending = false;
            g_obstacle_pending = false;
            confirmed = CUTTING_NORMAL;
            break;
    }

    // ── Step 6: Auto-calibration (uses raw current, not compensated) ──────
    if (g_cal_active && blade_commanded) {
        updateCalibration(blade_current_A);
    }

    // ── Step 7: Publish atomically ────────────────────────────────────────
    portENTER_CRITICAL(&g_mux);
    g_confirmed_status = confirmed;
    portEXIT_CRITICAL(&g_mux);
}

CuttingStatus cutting_monitor_get_status() {
    CuttingStatus s;
    portENTER_CRITICAL(&g_mux);
    s = g_confirmed_status;
    portEXIT_CRITICAL(&g_mux);
    return s;
}

float cutting_monitor_get_avg_current() {
    // g_rolling_avg holds voltage-compensated current; read it atomically.
    // On ESP32-S3 a 32-bit float read is not guaranteed atomic without a lock.
    portENTER_CRITICAL(&g_mux);
    float avg = g_rolling_avg;
    portEXIT_CRITICAL(&g_mux);
    return avg;
}

float cutting_monitor_get_load_fraction() {
    portENTER_CRITICAL(&g_mux);
    float avg = g_rolling_avg;
    portEXIT_CRITICAL(&g_mux);
    // Fraction of the fixed VESC current limit (see config.h, 2026-06-13). The
    // auto-calibrated g_blade_max_A is no longer used as the load reference — it
    // was capturing idle current as 100%, so idle read ~105%.
    return avg / BLADE_CURRENT_LIMIT_A;
}

void cutting_monitor_start_auto_cal() {
    g_cal_samples.clear();
    g_cal_active   = true;
    g_cal_complete = false;
    g_cal_start_ms = millis();
    DBG_PRINTLN("[BLADE CAL] Auto-calibration started");
}

bool cutting_monitor_is_cal_complete() {
    return g_cal_complete;
}

float cutting_monitor_get_cal_current() {
    // g_blade_max_A may be written by updateCalibration() on Core 1 only;
    // safe to read under spinlock.
    portENTER_CRITICAL(&g_mux);
    float val = g_blade_max_A;
    portEXIT_CRITICAL(&g_mux);
    return val;
}

void cutting_monitor_force_save_cal()
{
    portENTER_CRITICAL(&g_mux);
    float current_cal = g_blade_max_A;
    portEXIT_CRITICAL(&g_mux);

    // Skip if still at the factory default — a real calibration has never run,
    // so writing the default would overwrite a previously saved good value.
    if (current_cal <= BLADE_MAX_EXPECTED_CURRENT_A_DEFAULT) {
        DBG_PRINTLN("[BLADE CAL] force_save_cal: no valid cal yet — skipping write");
        return;
    }

    nvs_set_float("blade_cal", current_cal);
    DBG_PRINTF("[BLADE CAL] force_save_cal: wrote %.2f A to NVS\n", current_cal);
}
