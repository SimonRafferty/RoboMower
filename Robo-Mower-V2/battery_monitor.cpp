// ══════════════════════════════════════════════════════════════════════════════
//  battery_monitor.cpp — RoboMower 48V LiPo battery voltage monitor
//
//  Reads battery voltage from VESC CAN STATUS_5 via vesc_get_battery_voltage(),
//  applies an IIR low-pass filter, and classifies into one of three states.
//
//  No ADC or NVS dependency — VESC measures v_in directly from its power stage.
//
//  See battery_monitor.h and HANDOFFS/28_vesc_battery/HANDOFF.md for details.
// ══════════════════════════════════════════════════════════════════════════════

#include "battery_monitor.h"
#include "config.h"
#include "vesc_can.h"
#include "sys_log.h"
#include <cstdio>


// ─── Module-private state ─────────────────────────────────────────────────────

/** Filtered voltage estimate [V]. volatile for thread-safe single-float read. */
static volatile float g_filtered_v = 0.0f;

/** Current battery state. volatile so safetyTask can read without a mutex. */
static volatile BatteryState g_state = BATTERY_OK;


// ─── Public API ───────────────────────────────────────────────────────────────

void battery_monitor_init() {
    // Seed filtered voltage to 0.0f; first valid STATUS_5 frame populates it.
    g_filtered_v = 0.0f;
    g_state      = BATTERY_OK;

    DBG_PRINTF("[BATTERY] Init: source VESC CAN STATUS_5 (VESC_ID_BLADE, CAN ID 3)\n");
}


void battery_monitor_update() {
    // Log a one-shot warning if STATUS_5 has never been received after the
    // CAN bus is established. Most likely cause: STATUS_5 not enabled on the
    // blade VESC (CAN ID 3) in VESC Tool. Battery will show 0V on RC telemetry.
    {
        static bool s_status5_warned = false;
        if (!s_status5_warned && vesc_get_battery_voltage() == 0.0f
                && !vesc_battery_voltage_stale()) {
            // Bus is alive (no stale) but voltage is still zero after first call
            // — give it 10 s then warn once.
            static uint32_t s_warn_after_ms = 0;
            if (s_warn_after_ms == 0) s_warn_after_ms = millis() + 10000UL;
            if (millis() > s_warn_after_ms) {
                DBG_PRINTLN("[BATTERY] WARNING: no STATUS_5 from blade VESC (ID 3) — "
                            "enable CAN_STATUS_1_2_3_4_5 on blade VESC in VESC Tool");
                s_status5_warned = true;
            }
        }
        if (vesc_get_battery_voltage() > 0.0f) s_status5_warned = true;
    }

    // If the blade VESC (ID 3) has stopped sending STATUS_5 (e.g. after PILZ fires while
    // supercap keeps the ESP32 alive), keep the last known state rather than
    // feeding zeros into the filter. (BUG-3 fix — stale detection)
    if (vesc_battery_voltage_stale()) {
        // CAN bus / VESC offline — voltage is unknown.
        // Assume OK rather than latching LOW on stale data, so the state
        // machine doesn't get trapped in an AUTO↔PAUSED loop.
        if (g_state != BATTERY_OK) {
            DBG_PRINTLN("[BATTERY] VESC stale — resetting to OK (voltage unknown)");
            g_state = BATTERY_OK;
        }
        g_filtered_v = 0.0f;  // reset filter so next real reading re-seeds
        return;
    }

    float raw_v = vesc_get_battery_voltage();

    // Skip update until the first STATUS_5 frame arrives from the VESC.
    // vesc_get_battery_voltage() returns 0.0f before the first frame.
    if (raw_v == 0.0f) {
        g_state = BATTERY_OK;  // no data yet — assume OK
        return;
    }

    // Plausibility gate: a 13S pack lives between ~30 V (destroyed) and ~60 V
    // (above full charge). Anything outside that range is a decode fault or a
    // mis-sourced CAN frame, NOT a real battery state — never feed it into the
    // filter or the LOW latch (a bogus -0.1 V reading locked the blade out on
    // every boot until the STATUS_5 byte-offset bug was found on 2026-06-12).
    if (raw_v < 30.0f || raw_v > 60.0f) {
        static bool s_implausible_warned = false;
        if (!s_implausible_warned) {
            s_implausible_warned = true;
            char line[64];
            snprintf(line, sizeof(line),
                     "BATTERY: implausible %.1fV ignored (decode fault?)", raw_v);
            sys_log_push(line);
        }
        return;  // hold current state; do not poison filter or latch LOW
    }

    if (g_filtered_v < 1.0f) {
        // First valid reading: seed the filter directly to avoid a multi-second
        // warm-up from zero. Threshold 1.0V is below any plausible battery
        // voltage yet above zero (prevents re-seeding on normal readings).
        g_filtered_v = raw_v;
    } else {
        // Exponential moving average (IIR low-pass):
        //   filtered = filtered × (1 − α) + raw × α
        // α = BATTERY_FILTER_ALPHA = 0.10f
        // At 2 Hz: time constant τ = 1 / (2 × 0.10) = 5 seconds.
        // Motor-start voltage sags (≪ 5 s) are attenuated;
        // genuine battery drain (tens of minutes) passes through.
        g_filtered_v = g_filtered_v * (1.0f - BATTERY_FILTER_ALPHA)
                     + raw_v * BATTERY_FILTER_ALPHA;
    }

    // Classify state.
    // BATTERY_LOW is latched: once the battery is critically low the state
    // cannot return to WARNING or OK without a power-cycle. This is
    // notification-only — latching the state does NOT stop the motors
    // (auto-return / blade lockout were removed — operator decision).
    if (g_filtered_v < BATTERY_LOW_V) {
        g_state = BATTERY_LOW;
    } else if (g_state != BATTERY_LOW) {
        // Only update toward less-severe states if not already latched LOW.
        // Hysteresis on WARNING→OK: require voltage to recover by
        // BATTERY_WARN_HYSTERESIS_V above the warn threshold before clearing
        // the warning — prevents flapping when voltage oscillates. (MED-4 fix)
        if (g_filtered_v < BATTERY_WARN_V) {
            g_state = BATTERY_WARNING;
        } else if (g_filtered_v > BATTERY_WARN_V + BATTERY_WARN_HYSTERESIS_V) {
            g_state = BATTERY_OK;
        }
        // else: voltage is between WARN and WARN+HYSTERESIS — hold current state
    }
}


float battery_get_voltage() {
    // Single volatile float read — 32-bit aligned, atomic on Xtensa LX7.
    // No mutex required.
    return g_filtered_v;
}


BatteryState battery_get_state() {
    return g_state;
}


void battery_dump() {
    // Map state enum to a human-readable label
    const char* state_str;
    switch (g_state) {
        case BATTERY_OK:      state_str = "OK";      break;
        case BATTERY_WARNING: state_str = "WARNING"; break;
        case BATTERY_LOW:     state_str = "LOW";     break;
        default:              state_str = "UNKNOWN"; break;
    }

    DBG_PRINTF("[BATTERY] Voltage: %.1fV  State: %s  (source: VESC CAN STATUS_5)\n",
                  (float)g_filtered_v,
                  state_str);
}
