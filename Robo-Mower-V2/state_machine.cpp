// ══════════════════════════════════════════════════════════════════════════════
//  state_machine.cpp — RoboMower Top-Level State Machine
//
//  Implements all 11 robot states, LED patterns (FastLED), serial commands,
//  2 Hz JSON telemetry, and manual RC drive mapping.
//
//  Call state_machine_init() from setup(), then call state_machine_update()
//  from a FreeRTOS task at 10 Hz on Core 1.
//
//  References:
//    Spec:         Robo_Mower_claudecode_prompt_v3.md §"State Machine"
//    Assumptions:  ASSUMPTIONS.md A07, A08, A10, A23
//    Architecture: ARCHITECTURE.md §1
//    Config:       Robo-Mower-V2/config.h
// ══════════════════════════════════════════════════════════════════════════════

#include "state_machine.h"
#include "ble_server.h"

#include <FastLED.h>
#include <Preferences.h>

#include "config.h"
#include "mower_config.h"
#include "crsf_input.h"
#include "vesc_can.h"
#include "servo_output.h"
#include "ekf_localiser.h"
#include "rtk_gps.h"
#include "imu.h"
#include "perimeter.h"
#include "geometry.h"
#include "coverage_planner.h"
#include "node_follower.h"
#include "clipper_offset.h"
#include "odo_calib.h"
#include "cutting_monitor.h"
#include "bog_recovery.h"
#include "retrace.h"
#include "blade_recovery.h"   // Feature 2: RPM-load recovery (hosts STATE_RETRACE)
#include "safety.h"
#include "obstacle_map.h"
#include "battery_monitor.h"
#include "nvs_storage.h"
#include "collision_detect.h"
#include "heading_controller.h"
#include "crsf_telemetry.h"
#include "wheel_duty_ramp.h"
#include "sys_log.h"

// ── Colour constants (0xRRGGBB) ──────────────────────────────────────────────
#define COL_BLUE    0x0000FF
#define COL_GREEN   0x00FF00
#define COL_AMBER   0xFF8C00
#define COL_YELLOW  0xFFFF00
#define COL_ORANGE  0xFF4500
#define COL_RED     0xFF0000
#define COL_CYAN    0x00FFFF
#define COL_WHITE   0xFFFFFF
#define COL_PURPLE  0x800080

// ── FastLED arrays ────────────────────────────────────────────────────────────
// Shared array: index 0 = onboard LED, indices 1..LED_EXTERNAL_COUNT = strip
static CRGB s_leds[1 + LED_EXTERNAL_COUNT];

// ── Forward declarations ──────────────────────────────────────────────────────
static bool mower_inside_perimeter();

// ── Module-level state ────────────────────────────────────────────────────────
static volatile RobotState g_state = STATE_INIT;
static bool     g_state_entry = true;   // true on first tick of a new state
static RobotState g_prev_state = STATE_INIT;  // state before last transition

// Pending state transition target (applied at end of tick)
static RobotState g_next_state = STATE_INIT;
static bool       g_transition_pending = false;

// Event-triggered pause latch: set when a safety event (perimeter breach,
// blade fault) drives the mower into STATE_PAUSED.  The operator must cycle
// the pause switch (activate then deactivate) to acknowledge before the mower
// will accept a resume command.  Operator-initiated pauses leave this clear.
static bool s_pause_event_latch = false;

// Serial line buffer
static char  s_serial_buf[128];
static uint8_t s_serial_len = 0;

// 2 Hz telemetry timer
static uint32_t s_telem_last_ms = 0;

// Battery update divider (10 Hz tick / 5 = 2 Hz)
static uint8_t s_bat_div = 0;

// ── BLE module state ──────────────────────────────────────────────────────────
#define BLE_CMD_MAX_LEN 1024
struct BleCmdSlot { char json[BLE_CMD_MAX_LEN]; };
static QueueHandle_t s_ble_cmd_queue = nullptr;
static bool s_last_armed = false;    // updated each tick; read by is_armed getter
static bool s_prev_armed = false;   // edge detection for armed beep
static uint8_t s_prev_ch4_mode = 0xFF; // edge detection for mode-switch beep (0=manual,1=auto,2=return); 0xFF = uninitialised

volatile bool g_ble_map_pending    = false;
volatile bool g_ble_status_pending = false;
volatile bool g_ble_diag_pending   = false;

// ── AUTO_MOWING state locals ──────────────────────────────────────────────────
static float    s_desired_cut_height_mm = 200.0f;  // overwritten in state_machine_init() from config
static uint32_t s_cross_track_exceed_ms = 0;  // millis() when |XTE| first > 0.5 m
static bool     s_blade_commanded = false;
static float    s_blade_ramp_erpm  = 0.0f;    // target eRPM sent to blade VESC (0 or full target)
// (blade battery lockout removed 2026-06-12 — VESC firmware handles low-voltage
//  power reduction internally; battery LOW is notification-only)
static uint32_t s_session_start_ms = 0;       // millis() when AUTO_MOWING was entered

// ── BLE drive overlay (RC failsafe only) ─────────────────────────────────────
// WebUI drive only activates when the RC TX is off (failsafe active).
// Timeout must be long enough for a slider held at a fixed position — the
// WebUI only sends SET_DRIVE on movement events, not continuously.
#define BLE_DRIVE_TIMEOUT_MS  5000  // 5 s grace after last active control input
static float    s_ble_throttle   = 0.0f;   // [-1, +1] from SET_DRIVE command
static float    s_ble_steering   = 0.0f;   // [-1, +1] from SET_DRIVE command
static uint32_t s_ble_drive_ms   = 0;       // millis() of last SET_DRIVE received
static bool     s_ble_pause      = false;   // BLE pause state (OR'd with CH7 + GPIO)
static bool     s_ble_arm        = false;   // BLE arm/blade state
static bool     s_ble_learn      = false;   // BLE learn mode toggle

// ── "Looking around" animation (IDLE, no perimeter configured) ───────────────
// Drives left ~90°, then right ~180°, then back to centre — a visible prompt
// that the mower has no perimeter and needs to be taught one.
// Runs once per power-on session, not on every IDLE entry.
enum LookPhase : uint8_t {
    LOOK_NOT_STARTED = 0,
    LOOK_LEFT,
    LOOK_PAUSE_1,
    LOOK_RIGHT,
    LOOK_PAUSE_2,
    LOOK_CENTRE,
    LOOK_DONE
};
static LookPhase s_look_phase = LOOK_NOT_STARTED;
static uint32_t  s_look_ms    = 0;
// Duty for animation turns — gentle, not fast.
#define LOOK_DUTY        0.20f
// Time at LOOK_DUTY for approximately 90° of rotation (tune to your chassis).
#define LOOK_TIME_90_MS  2500U
#define LOOK_PAUSE_MS     400U

// ── PAUSED / MOTORS_OFFLINE state locals ──────────────────────────────────────
// Pause is level-based: mower is paused whenever CH7, GPIO switch, or BLE pause is active.
// No edge detection needed — all inputs are treated as latching switches.

/** Returns true when the latching pause switch is in the active (GND/LOW) position,
 *  after the pin has settled for PAUSE_DEBOUNCE_MS.
 *  Only call from the state machine tick (Core 1).
 *  Switch wiring: one terminal to GPIO PAUSE_PIN, other terminal to GND.
 *  Closed (GND) = pause active.  Open (pulled HIGH) = pause inactive. */
static bool pauseSwitchActive() {
    static bool     s_debounced  = false;  // current settled state (true = active/LOW)
    static bool     s_raw_prev   = true;   // previous raw reading
    static uint32_t s_change_ms  = 0;      // millis() when raw reading last changed

    bool raw = !digitalRead(PAUSE_PIN);    // true = LOW = switch closed = pause active

    if (raw != s_raw_prev) {
        s_raw_prev  = raw;
        s_change_ms = millis();
    }

    if ((millis() - s_change_ms) >= PAUSE_DEBOUNCE_MS) {
        s_debounced = raw;
    }

    return s_debounced;
}

// Waypoint list storage for node follower (coverage planner owns master list;
// we keep a local pointer and count per the existing planner API)
static Waypoint   s_wp_buf[1000];   // large static buffer — coverage planner fills via get_next
static int        s_wp_count = 0;
static int        s_wp_index = 0;

// ── LEARN_PERIMETER state locals ──────────────────────────────────────────────
static bool s_ch5_prev = false;   // previous CH5 active state for edge detection
static uint32_t s_learn_pt_seen = 0;  // last consumed CH8 learn-point event count (crsf_get_learn_pt_events)
static bool s_learn_first_point = true;  // true until first CH8 press in learn mode

// ── Return-to-home position (saved at perimeter recording start) ──────────────
// Stored as ENU metres relative to the GPS origin, persisted to NVS.
// Using the actual entry position avoids the ensureCCW() point-reordering issue.
static float s_home_x = 0.0f;
static float s_home_y = 0.0f;

// ── OBSTACLE_AVOID state locals ───────────────────────────────────────────────
static float    s_obstacle_backup_dist_m  = OBSTACLE_BACKUP_DIST_M;  // direction-adjusted
static uint32_t s_obstacle_enter_ms       = 0;      // millis() when OBSTACLE_AVOID was entered
static uint32_t s_obstacle_backup_time_ms = 0;      // derived backup duration (time-based)
static bool     s_obstacle_backup_done = false;
static bool     s_obstacle_reset_done  = false;
static CollisionDirection s_lastCollisionDir = COLLISION_DIR_UNKNOWN;

// ── AUTO_RETURN state locals ───────────────────────────────────────────────────
static Waypoint s_return_wp;  // single waypoint: perimeter.pts[0]

// ── RETRACE state locals ──────────────────────────────────────────────────────
static float s_retrace_strip_end_x = 0.0f;
static float s_retrace_strip_end_y = 0.0f;

// clampf() is provided by geometry.h (included via state_machine.h)

// ── Helper: short state name for logging / telemetry ─────────────────────────
static const char *sm_state_name(RobotState s) {
    static const char *k_names[] = {
        "INIT","IDLE","MANUAL","LEARN","AUTO",
        "RETRACE","BOG","OBS-AVOID","RETURN","PAUSED","MOT-OFF"
    };
    int i = (int)s;
    return (i >= 0 && i < 11) ? k_names[i] : "?";
}

// ── Helper: request state transition ─────────────────────────────────────────
// event_latch: when true, sets s_pause_event_latch so that STATE_PAUSED
// requires an explicit operator acknowledge (pause-switch cycle) before resume.
static void transition_to(RobotState next, bool event_latch = false) {
    // Every transition lands in the BLE-visible system log so re-entry loops
    // (the "beeps repeatedly, does nothing" failure) are diagnosable in the field.
    if (next != g_state) {
        char line[SYS_LOG_MAX_LEN];
        snprintf(line, sizeof(line), "SM %s>%s%s t=%lu",
                 sm_state_name(g_state), sm_state_name(next),
                 (event_latch && next == STATE_PAUSED) ? " latch" : "",
                 (unsigned long)millis());
        sys_log_push(line);
    }
    g_next_state = next;
    g_transition_pending = true;
    if (event_latch && next == STATE_PAUSED) {
        s_pause_event_latch = true;
    }
}

// ── Hysteresis switch decoders ────────────────────────────────────────────────
// RC switch µs values are noisy (± a few counts). Single-threshold comparisons
// cause rapid toggling when the signal hovers near a boundary. These helpers
// use a dead-band (HYST µs either side of the nominal threshold) — the output
// only changes when the value moves convincingly into the new band.

static constexpr uint16_t SW_HYST = 30;  // µs hysteresis half-width

// ── 2-position switch (low / high) with hysteresis ──────────────────────────
// Returns true when value is above (threshold + hyst), false when below
// (threshold − hyst), otherwise holds previous state.
static bool sw2_decode(uint16_t us, uint16_t thresh_us, bool prev) {
    if (us > thresh_us + SW_HYST) return true;
    if (us < thresh_us - SW_HYST) return false;
    return prev;  // inside dead-band — hold
}

// ── 3-position switch (low=0 / mid=1 / high=2) with hysteresis ─────────────
// Two thresholds divide the range into three bands. Each boundary has a
// dead-band of ±SW_HYST where the previous position is retained.
static uint8_t sw3_decode(uint16_t us, uint16_t lo_thresh_us,
                           uint16_t hi_thresh_us, uint8_t prev) {
    // Above the high threshold (with hysteresis)
    if (us > hi_thresh_us + SW_HYST) return 2;
    // Below the low threshold (with hysteresis)
    if (us < lo_thresh_us - SW_HYST) return 0;
    // Clearly between the two thresholds
    if (us > lo_thresh_us + SW_HYST && us < hi_thresh_us - SW_HYST) return 1;
    // Inside a dead-band — hold previous position
    return prev;
}

// Switch thresholds in µs.  Range is 1000–2000 µs.
// 3-position switch positions: ~1000 / ~1500 / ~2000 → midpoints at 1250 & 1750
// 2-position switch positions: ~1000 / ~2000           → midpoint at 1500
static constexpr uint16_t CH4_LO_US = 1250;   // manual(1000) ↔ auto(1500) boundary
static constexpr uint16_t CH4_HI_US = 1750;   // auto(1500) ↔ auto-return(2000) boundary
static constexpr uint16_t CH5_US    = 1500;   // learn off(1000) ↔ on(2000)
static constexpr uint16_t CH6_US    = 1500;   // disarmed(1000) ↔ armed(2000)
static constexpr uint16_t CH7_US    = 1500;   // pause off(1000) ↔ on(2000)
// CH8 (learn-point momentary) is edge-detected in crsf_input at the CRSF frame
// rate, not polled here — see crsf_get_learn_pt_events() and the LEARN handler.

// Latched switch states (initialised to safe defaults)
static uint8_t s_ch4_pos  = 0;      // 0=manual, 1=auto, 2=auto-return
static bool    s_ch5_on   = false;
static bool    s_ch6_on   = false;   // armed
static bool    s_ch7_on   = false;   // pause

// Wrappers matching the original API — now read latched state
static inline bool ch4_is_manual(uint16_t)      { return s_ch4_pos == 0; }
static inline bool ch4_is_auto(uint16_t)        { return s_ch4_pos == 1; }
static inline bool ch4_is_auto_return(uint16_t) { return s_ch4_pos == 2; }
static inline bool ch5_is_learning(uint16_t)    { return s_ch5_on; }
static inline bool ch6_is_armed(uint16_t)       { return s_ch6_on; }
static inline bool ch7_is_pause(uint16_t)       { return s_ch7_on; }


// ══════════════════════════════════════════════════════════════════════════════
//  LED implementation
// ══════════════════════════════════════════════════════════════════════════════

/** Compute PWM brightness [0–255] for a given pattern based on millis(). */
static uint8_t computePatternBrightness(LedPattern pattern) {
    uint32_t t = millis();

    switch (pattern) {
    case LED_SOLID:
        return 255;

    case LED_SLOW_PULSE: {
        // Smooth 0→255→0 over 2000 ms using half-cosine
        uint32_t phase = t % 2000;
        // phase 0..1999 → 0..2π
        float angle = (float)phase / 2000.0f * 2.0f * (float)M_PI;
        float v = (1.0f - cosf(angle)) * 0.5f;  // 0..1
        return (uint8_t)(v * 255.0f);
    }

    case LED_SINGLE_BLINK: {
        // 100 ms on, 900 ms off (1000 ms period)
        uint32_t phase = t % 1000;
        return (phase < 100) ? 255 : 0;
    }

    case LED_DOUBLE_BLINK: {
        // 100 on, 100 off, 100 on, 700 off (1000 ms period)
        uint32_t phase = t % 1000;
        if (phase < 100) return 255;
        if (phase < 200) return 0;
        if (phase < 300) return 255;
        return 0;
    }

    case LED_SLOW_FLASH: {
        // 500 ms on, 500 ms off (1000 ms period)
        uint32_t phase = t % 1000;
        return (phase < 500) ? 255 : 0;
    }

    case LED_FAST_FLASH: {
        // 250 ms on, 250 ms off (500 ms period)
        uint32_t phase = t % 500;
        return (phase < 250) ? 255 : 0;
    }

    case LED_ALTERNATING:
    case LED_THREE_FLASH:
        // Handled by dedicated functions; should not reach here
        return 255;
    }
    return 255;
}

/** Write colour + brightness to all LEDs and call FastLED.show(). */
static void applyLeds(CRGB colour) {
    for (int i = 0; i < 1 + LED_EXTERNAL_COUNT; i++) {
        s_leds[i] = colour;
    }
    FastLED.show();
}

void showLeds(uint32_t rgb, LedPattern pattern) {
    uint8_t brightness = computePatternBrightness(pattern);
    CRGB colour = CRGB((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
    colour.nscale8(brightness);
    applyLeds(colour);
}

static void showLedsWithGps(uint32_t primary_rgb) {
    // 1250 ms cycle: 500ms primary → 250ms off → 250ms GPS state → 250ms off
    GpsMeasurement gps = rtk_gps_get_measurement();
    // GPS_FIX_RTK_FIXED=4, GPS_FIX_RTK_FLOAT=5 — float is numerically higher,
    // so use equality to avoid float being mistaken for fixed.
    uint32_t gps_rgb = (gps.fix_type == GPS_FIX_RTK_FIXED) ? COL_GREEN
                     : (gps.fix_type == GPS_FIX_RTK_FLOAT) ? COL_ORANGE
                     :                                        COL_RED;
    uint32_t phase = millis() % 1250;
    uint32_t rgb = (phase < 500)  ? primary_rgb
                 : (phase < 750)  ? 0u
                 : (phase < 1000) ? gps_rgb
                 :                  0u;
    CRGB c = CRGB((rgb >> 16) & 0xFF, (rgb >> 8) & 0xFF, rgb & 0xFF);
    applyLeds(c);
}

void ledFlashWhite3x() {
    // 3 × (100 ms on, 100 ms off) — blocking but only called on perimeter save
    for (int i = 0; i < 3; i++) {
        applyLeds(CRGB(255, 255, 255));
        vTaskDelay(pdMS_TO_TICKS(100));
        applyLeds(CRGB(0, 0, 0));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    // 200 ms tail-off
    vTaskDelay(pdMS_TO_TICKS(200));
}


// ══════════════════════════════════════════════════════════════════════════════
//  Battery warning LED overlay
// ══════════════════════════════════════════════════════════════════════════════

/** Apply battery warning amber overlay (called after normal state LED update). */
static void applyBatteryWarningOverlay() {
    if (safety_is_battery_warning() &&
        g_state != STATE_MOTORS_OFFLINE) {
        // 200 ms amber flash every 2 s
        uint32_t phase = millis() % 2000;
        if (phase < 200) {
            showLeds(COL_AMBER, LED_SOLID);
        }
    }
}


// ══════════════════════════════════════════════════════════════════════════════
//  Waypoint buffer helpers for node follower
// ══════════════════════════════════════════════════════════════════════════════

/** Drain the coverage planner into s_wp_buf[]. Returns number of waypoints. */
static int load_waypoints_from_planner() {
    int count = 0;
    Waypoint wp;
    while (count < (int)(sizeof(s_wp_buf) / sizeof(s_wp_buf[0])) &&
           coverage_planner_get_next(wp)) {
        s_wp_buf[count++] = wp;
    }
    return count;
}


// ── Plan geometry dump (BLE field diagnostic) ──────────────────────────────────
// Emit a compact summary of the planned path to the system log so it can be read
// over Bluetooth (no serial, no PWA "Plan Test" button needed). Reports counts by
// type, total path length, and — crucially — the largest jump between consecutive
// waypoints, both overall and WITHIN the headland pass. A large headland gap means
// the stored perimeter polygon is sparse, so the follower tracks long straight
// chords instead of hugging the edge — the "drives diagonally across, ignores the
// perimeter, ends in the flower bed" symptom (cross-track stays 0 because the mower
// IS perfectly on the chord). Also shows where waypoint 0 sits relative to the
// mower, to reveal a plan that starts far from home.
static void log_plan_stats(const Waypoint *wps, int n, const Pose2D &ps) {
    if (!wps || n <= 0) { sys_log_push("PLAN empty (0 wp)"); return; }

    int   n_hl = 0, n_mow = 0, n_tr = 0;
    float total_len   = 0.0f;
    float maxgap      = 0.0f;  int maxgap_i    = -1;   // largest gap (any type)
    float maxgap_hl   = 0.0f;  int maxgap_hl_i = -1;   // largest headland-to-headland gap
    int   big_gaps    = 0, big_gaps_hl = 0;            // count of gaps > 1.0 m
    for (int i = 0; i < n; i++) {
        if (wps[i].headland)    n_hl++;
        else if (wps[i].mowing) n_mow++;
        else                    n_tr++;
        if (i > 0) {
            float gx = wps[i].x - wps[i-1].x;
            float gy = wps[i].y - wps[i-1].y;
            float d  = sqrtf(gx*gx + gy*gy);
            total_len += d;
            if (d > maxgap) { maxgap = d; maxgap_i = i; }
            if (d > 1.0f) big_gaps++;
            if (wps[i].headland && wps[i-1].headland) {
                if (d > maxgap_hl) { maxgap_hl = d; maxgap_hl_i = i; }
                if (d > 1.0f) big_gaps_hl++;
            }
        }
    }

    char l[SYS_LOG_MAX_LEN];
    snprintf(l, sizeof(l), "PLAN n=%d hl=%d mow=%d tr=%d len=%.0fm",
             n, n_hl, n_mow, n_tr, total_len);
    sys_log_push(l);
    snprintf(l, sizeof(l), "PLAN maxgap=%.1f@%d big>1m=%d (hl:%.1f@%d x%d)",
             maxgap, maxgap_i, big_gaps, maxgap_hl, maxgap_hl_i, big_gaps_hl);
    sys_log_push(l);
    float d0 = sqrtf((wps[0].x - ps.x) * (wps[0].x - ps.x)
                   + (wps[0].y - ps.y) * (wps[0].y - ps.y));
    snprintf(l, sizeof(l), "PLAN pos=%.1f,%.1f w0=%.1f,%.1f d0=%.1f",
             ps.x, ps.y, wps[0].x, wps[0].y, d0);
    sys_log_push(l);
    if (maxgap_i > 0) {
        snprintf(l, sizeof(l), "PLAN gap@%d %.1f,%.1f->%.1f,%.1f",
                 maxgap_i, wps[maxgap_i-1].x, wps[maxgap_i-1].y,
                 wps[maxgap_i].x, wps[maxgap_i].y);
        sys_log_push(l);
    }
}


// ══════════════════════════════════════════════════════════════════════════════
//  State machine initialisation
// ══════════════════════════════════════════════════════════════════════════════

void state_machine_init() {
    // Configure FastLED — shared array, two strips
    FastLED.addLeds<WS2812, LED_ONBOARD_PIN, GRB>(s_leds, 0, 1);
    FastLED.addLeds<WS2812, LED_EXTERNAL_PIN, GRB>(s_leds, 1, LED_EXTERNAL_COUNT);
    FastLED.setBrightness(128);

    g_state = STATE_INIT;
    g_state_entry = true;
    g_transition_pending = false;

    s_serial_len = 0;
    s_telem_last_ms = 0;
    s_bat_div = 0;
    s_ch5_prev = false;

    s_ble_cmd_queue = xQueueCreate(4, sizeof(BleCmdSlot));

    // Default cut height from runtime config (matches compiled default on first boot)
    s_desired_cut_height_mm = (float)mower_config_get().cut_height_max_mm;

    // Restore return-to-home position from NVS (saved when perimeter recording starts)
    s_home_x = nvs_get_float("home_x", 0.0f);
    s_home_y = nvs_get_float("home_y", 0.0f);

    DBG_PRINTLN("[SM] State machine initialised → STATE_INIT");
}


// ══════════════════════════════════════════════════════════════════════════════
//  Serial command processing
// ══════════════════════════════════════════════════════════════════════════════

/** Trim leading/trailing whitespace in-place; returns pointer to first non-space. */
static const char *trim(const char *s) {
    while (*s == ' ' || *s == '\t') s++;
    return s;
}

/** Case-insensitive prefix match. Returns pointer past the prefix on match, NULL otherwise. */
static const char *cmd_prefix(const char *line, const char *prefix) {
    size_t plen = strlen(prefix);
    if (strncasecmp(line, prefix, plen) == 0) {
        return line + plen;
    }
    return nullptr;
}

void state_machine_handle_serial(const char *raw_cmd) {
    const char *cmd = trim(raw_cmd);

    // ── STATUS ────────────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "STATUS") == 0) {
        Pose2D pose = ekf_get_pose();
        GpsMeasurement gps = rtk_gps_get_measurement();
        float batt = battery_get_voltage();
        BatteryState bs = battery_get_state();
        CuttingStatus cs = cutting_monitor_get_status();

        const char *state_names[] = {
            "INIT","IDLE","MANUAL","LEARN_PERIMETER","AUTO_MOWING",
            "RETRACE","BOG_RECOVERY","OBSTACLE_AVOID","AUTO_RETURN",
            "PAUSED","MOTORS_OFFLINE"
        };
        const char *bat_names[] = {"OK","WARNING","LOW"};
        const char *cut_names[] = {"NORMAL","OVERLOADED","STALLED","OBS_SUSPECTED","BLADE_FAULT"};

        DBG_PRINTF("[STATUS] State=%s  x=%.3f y=%.3f hdg=%.3f  "
                      "fix=%d sats=%d  battV=%.2fV battState=%s  "
                      "bladeLoad=%.2f calA=%.1fA  cutStatus=%s  "
                      "hProg=%.2f sProg=%.2f total=%.2f  unc=%.4fm  "
                      "collBaseline=%.4fg\r\n",
            state_names[(int)g_state],
            pose.x, pose.y, pose.heading,
            (int)gps.fix_type, (int)gps.sat_count,
            batt, bat_names[(int)bs],
            cutting_monitor_get_load_fraction(),
            cutting_monitor_get_cal_current(),
            cut_names[(int)cs],
            coverage_planner_get_headland_progress(),
            coverage_planner_get_strip_progress(),
            coverage_planner_get_total_progress(),
            ekf_get_position_uncertainty(),
            collisionGetBaseline());
        return;
    }

    // ── IMUCALTEST / IMUSAVECAL — BNO055 calibration NVS diagnostics ───────────
    if (strcasecmp(cmd, "IMUCALTEST") == 0) {
        bool ok = imu_nvs_selftest();
        DBG_PRINTF("[IMU] NVS self-test: %s\r\n", ok ? "PASS" : "FAIL");
        return;
    }
    if (strcasecmp(cmd, "IMUSAVECAL") == 0) {
        if (!imu_heading_is_confident()) {
            DBG_PRINTLN("[IMU] save refused — gyro/accel/mag not all 3 (drive slow loops first)");
        } else {
            imu_request_save();
            DBG_PRINTLN("[IMU] calibration save requested — check log for verified result");
        }
        return;
    }

    // ── PERIMETER ─────────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "PERIMETER") == 0) {
        if (!perimeter_is_valid()) {
            DBG_PRINTLN("[PERIM] No valid perimeter stored.");
            return;
        }
        Polygon p = perimeter_get_perimeter();
        DBG_PRINTF("[PERIM] %d points:\r\n", (int)p.pts.size());
        for (size_t i = 0; i < p.pts.size(); i++) {
            DBG_PRINTF("  [%d] x=%.4f y=%.4f\r\n", (int)i, p.pts[i].x, p.pts[i].y);
        }
        return;
    }

    // ── NAVBOUNDARY ───────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "NAVBOUNDARY") == 0) {
        if (!perimeter_is_valid()) {
            DBG_PRINTLN("[NAVBND] No valid perimeter stored.");
            return;
        }
        Polygon p = perimeter_get_nav_boundary();
        DBG_PRINTF("[NAVBND] %d points:\r\n", (int)p.pts.size());
        for (size_t i = 0; i < p.pts.size(); i++) {
            DBG_PRINTF("  [%d] x=%.4f y=%.4f\r\n", (int)i, p.pts[i].x, p.pts[i].y);
        }
        return;
    }

    // ── WORKINGAREA ───────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "WORKINGAREA") == 0) {
        if (!perimeter_is_valid()) {
            DBG_PRINTLN("[WAREA] No valid perimeter stored.");
            return;
        }
        Polygon p = perimeter_get_working_area();
        DBG_PRINTF("[WAREA] %d points:\r\n", (int)p.pts.size());
        for (size_t i = 0; i < p.pts.size(); i++) {
            DBG_PRINTF("  [%d] x=%.4f y=%.4f\r\n", (int)i, p.pts[i].x, p.pts[i].y);
        }
        return;
    }

    // ── PLAN ──────────────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "PLAN") == 0) {
        DBG_PRINTF("[PLAN] wp_count=%d  wp_index=%d  hProg=%.2f sProg=%.2f "
                      "complete=%s  unreachable_zones=%d\r\n",
            s_wp_count, s_wp_index,
            coverage_planner_get_headland_progress(),
            coverage_planner_get_strip_progress(),
            coverage_planner_is_complete() ? "YES" : "NO",
            coverage_planner_get_unreachable_zone_count());
        // Print first/last few waypoints
        int show = (s_wp_count < 10) ? s_wp_count : 10;
        for (int i = 0; i < show; i++) {
            DBG_PRINTF("  wp[%d] x=%.3f y=%.3f hdg=%.3f mow=%d hdl=%d\r\n",
                i, s_wp_buf[i].x, s_wp_buf[i].y, s_wp_buf[i].heading,
                (int)s_wp_buf[i].mowing, (int)s_wp_buf[i].headland);
        }
        if (s_wp_count > 10) {
            DBG_PRINTF("  ... (%d more)\r\n", s_wp_count - 10);
        }
        return;
    }

    // ── GRID ──────────────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "GRID") == 0) {
        grid_dump_ascii();
        return;
    }

    // ── UNREACHABLE ───────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "UNREACHABLE") == 0) {
        coverage_planner_log_unreachable_zones();
        perimeter_log_unreachable_zones();
        return;
    }

    // ── EKFSTATE ──────────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "EKFSTATE") == 0) {
        ekf_dump_state();
        return;
    }

    // ── CALDUMP ───────────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "CALDUMP") == 0) {
        battery_dump();
        DBG_PRINTF("[CALDUMP] Servo range 1000–2000 µs (mechanically calibrated)\r\n");
        DBG_PRINTF("[CALDUMP] Blade cal=%.2fA  cal_done=%s\r\n",
            cutting_monitor_get_cal_current(),
            cutting_monitor_is_cal_complete() ? "YES" : "NO");
        DBG_PRINTF("[CALDUMP] Collision baseline=%.4f g\r\n", collisionGetBaseline());
        return;
    }

    // ── OBSTACLES ─────────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "OBSTACLES") == 0) {
        obstacle_dump();
        return;
    }

    // ── ERRORS ────────────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "ERRORS") == 0) {
        DBG_PRINTLN("[ERRORS] Software E-stop removed — hardware E-stop handled by PILZ relay.");
        return;
    }

    // ── CLEARPERIM ────────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "CLEARPERIM CONFIRM") == 0) {
        if (g_state != STATE_IDLE && g_state != STATE_MANUAL) {
            DBG_PRINTLN("[PERIM] CLEARPERIM rejected — robot must be IDLE or MANUAL.");
            return;
        }
        perimeter_clear();
        DBG_PRINTLN("[PERIM] Perimeter cleared from NVS.");
        return;
    }
    if (strcasecmp(cmd, "CLEARPERIM") == 0) {
        DBG_PRINTLN("[PERIM] Type 'CLEARPERIM CONFIRM' to erase the stored perimeter.");
        return;
    }

    // ── RESETEKF ──────────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "RESETEKF") == 0) {
        ekf_reset_covariance();
        DBG_PRINTLN("[EKF] Covariance reset to high initial values.");
        return;
    }

    // ── CALHEIGHT TEST ────────────────────────────────────────────────────────
    // Servo is mechanically calibrated (1000–2000 µs full range). TEST only.
    if (const char *sub = cmd_prefix(cmd, "CALHEIGHT ")) {
        servo_handle_cal_command(sub);
        return;
    }

    // ── PAUSE ─────────────────────────────────────────────────────────────────
    if (strcasecmp(cmd, "PAUSE") == 0) {
        if (g_state == STATE_AUTO_MOWING) {
            transition_to(STATE_PAUSED);
            DBG_PRINTLN("[SM] PAUSE command — entering STATE_PAUSED.");
        } else if (g_state == STATE_PAUSED) {
            transition_to(STATE_AUTO_MOWING);
            DBG_PRINTLN("[SM] PAUSE command — resuming AUTO_MOWING.");
        } else {
            DBG_PRINTLN("[SM] PAUSE command ignored (only active in AUTO_MOWING or PAUSED).");
        }
        return;
    }

    // ── Unknown command ───────────────────────────────────────────────────────
    DBG_PRINTF("[SM] Unknown command: '%s'\r\n", cmd);
    DBG_PRINTLN("     Commands: STATUS PERIMETER NAVBOUNDARY WORKINGAREA PLAN");
    DBG_PRINTLN("               GRID UNREACHABLE EKFSTATE CALDUMP OBSTACLES ERRORS");
    DBG_PRINTLN("               CLEARPERIM [CONFIRM] RESETEKF CALHEIGHT TEST <mm> PAUSE");
}


// ══════════════════════════════════════════════════════════════════════════════
//  2 Hz JSON telemetry
// ══════════════════════════════════════════════════════════════════════════════

static void emit_telemetry() {
    Pose2D pose     = ekf_get_pose();
    GpsMeasurement gps = rtk_gps_get_measurement();
    VescStatus blade = vesc_get_status(VESC_ID_BLADE);
    float batt      = battery_get_voltage();
    BatteryState bs = battery_get_state();
    CuttingStatus cs = cutting_monitor_get_status();
    float speed     = ekf_get_speed();
    float unc       = ekf_get_position_uncertainty();

    const char *state_names[] = {
        "INIT","IDLE","MANUAL","LEARN_PERIM","AUTO","RETRACE",
        "BOG","OBS_AVOID","RETURN","PAUSED","MOTORS_OFFLINE"
    };
    const char *bat_names[] = {"OK","WARNING","LOW"};
    const char *cut_names[] = {"NORMAL","OVERLOADED","STALLED","OBS_SUSP","BLADE_FAULT"};

    // blade RPM = erpm / pole_pairs
    float blade_rpm = fabsf(blade.erpm) / (float)mower_config_get().blade_motor_pole_pairs;

    DBG_PRINTF("{\"t\":%lu,\"state\":\"%s\","
                  "\"x\":%.3f,\"y\":%.3f,\"hdg\":%.3f,"
                  "\"fix\":%d,\"sat\":%d,\"vel\":%.3f,"
                  "\"hprog\":%.3f,\"sprog\":%.3f,"
                  "\"cutH\":%.0f,"
                  "\"bog\":0,\"obs\":%d,\"unc\":%.4f,"
                  "\"bladeRPM\":%.0f,\"bladeA\":%.2f,\"bladeLoad\":%.3f,"
                  "\"cutStatus\":\"%s\","
                  "\"battV\":%.2f,\"battState\":\"%s\","
                  "\"bladeCmd\":%d,\"bladeLock\":%d,"
                  "\"collBase\":%.3f}\r\n",
        (unsigned long)millis(),
        state_names[(int)g_state],
        pose.x, pose.y, pose.heading,
        (int)gps.fix_type, (int)gps.sat_count, speed,
        coverage_planner_get_headland_progress(),
        coverage_planner_get_strip_progress(),
        s_desired_cut_height_mm,
        obstacle_get_count(),
        unc,
        blade_rpm, blade.current_A, cutting_monitor_get_rpm_load_fraction(),  // RPM-based load (Feature 2)
        cut_names[(int)cs],
        batt, bat_names[(int)bs],
        (int)s_blade_commanded, 0 /* lockout removed 2026-06-12 */,
        collisionGetBaseline());
}


// ══════════════════════════════════════════════════════════════════════════════
//  Manual RC drive helper
// ══════════════════════════════════════════════════════════════════════════════

static float s_desired_heading  = 0.0f;   // heading hold target (rad)
static bool  s_heading_active   = false;  // true once throttle has been applied
static float s_drive_duty_l     = 0.0f;   // rate-limited duty for left wheel
static float s_drive_duty_r     = 0.0f;   // rate-limited duty for right wheel

static void drive_manual(const CRSFChannels &rc, const Pose2D &pose, float dt) {
    float throttle, steering;

    // Rescaled deadband: zeroes the centre region then ramps smoothly to ±1.
    // Applied after crsf_us_to_norm() which already has a ±5% hard zero.
    auto deadband = [](float v) -> float {
        constexpr float db = MANUAL_DEADBAND;
        if (fabsf(v) <= db) return 0.0f;
        return (v > 0.0f ? 1.0f : -1.0f) * (fabsf(v) - db) / (1.0f - db);
    };

    // Exponential curve applied after deadband: softens response around centre
    // while preserving full output at ±1.  Formula: v × ((1−ex) + ex × v²)
    auto expo = [](float v) -> float {
        constexpr float ex = MANUAL_EXPO;
        return v * ((1.0f - ex) + ex * v * v);
    };

    if (rc.failsafe) {
        // RC offline — use BLE drive values if fresh, otherwise coast for safety
        if ((millis() - s_ble_drive_ms) > BLE_DRIVE_TIMEOUT_MS) {
            vesc_set_current(VESC_ID_LEFT,  0.0f);
            vesc_set_current(VESC_ID_RIGHT, 0.0f);
            s_heading_active = false;
            return;
        }
        throttle = s_ble_throttle;
        steering = s_ble_steering;
    } else {
        // RC active: use RC sticks; WebUI drive is disabled while TX is on
        throttle =  expo(deadband(crsf_us_to_norm(rc.ch[CRSF_CH_THROTTLE])));
        steering = -expo(deadband(crsf_us_to_norm(rc.ch[CRSF_CH_STEERING])));
    }

    const MowerConfig &mc = mower_config_get();

    // ── Tank mix: stick → target duty per wheel ──────────────────────────────
    float target_l = mc.manual_max_duty * (throttle - steering);
    float target_r = mc.manual_max_duty * (throttle + steering);

    // ── Heading stabilisation (additive correction for drift only) ────────────
    if (fabsf(throttle) < 0.05f) {
        s_heading_active = false;
    } else {
        if (!s_heading_active) {
            s_desired_heading = pose.heading;
            heading_controller_reset();
            s_heading_active = true;
        }

        float yaw_rate_cmd   = -steering * mc.manual_max_yaw_rate;  // negate: steering sign is inverted vs gz_rads (CW+)
        s_desired_heading    = wrapAngle(s_desired_heading + yaw_rate_cmd * dt);

        float heading_error  = wrapAngle(s_desired_heading - pose.heading);
        float yaw_rate_error = yaw_rate_cmd - imu_get_gz_rads();
        float corr_ms = heading_controller_compute(heading_error, yaw_rate_error,
                                                    mc.heading_kp, mc.heading_kd, dt);
        float corr_duty = corr_ms / mc.max_wheel_speed_ms * mc.manual_max_duty;
        // Yaw rate from a differential drive is (v_left - v_right)/track REGARDLESS
        // of travel direction — backing up does NOT reverse which wheel-speed
        // differential produces a CW yaw. So the heading-hold correction is applied
        // with the SAME sign forward and reverse. (An earlier throttle-sign flip here
        // turned the loop into positive feedback in reverse — the "haywire" symptom.)
        target_l += corr_duty;
        target_r -= corr_duty;
    }

    target_l = clampf(target_l, -1.0f, 1.0f);
    target_r = clampf(target_r, -1.0f, 1.0f);

    // ── Rate-limited ramp — prevents sudden high-torque from lifting the front ─
    const float max_step = MANUAL_DUTY_RAMP_PER_S * dt;
    s_drive_duty_l = clampf(target_l,
                             s_drive_duty_l - max_step,
                             s_drive_duty_l + max_step);
    s_drive_duty_r = clampf(target_r,
                             s_drive_duty_r - max_step,
                             s_drive_duty_r + max_step);

    vesc_set_duty(VESC_ID_LEFT,  s_drive_duty_l);
    vesc_set_duty(VESC_ID_RIGHT, s_drive_duty_r);
}


// ══════════════════════════════════════════════════════════════════════════════
//  BLE command handler
// ══════════════════════════════════════════════════════════════════════════════

/** Extract the value of a JSON string field: "key":"value" → copies value into buf. */
static bool json_get_str(const char *json, const char *key, char *buf, int bufsz) {
    // Build search pattern: "key":"
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\":\"", key);
    const char *p = strstr(json, pattern);
    if (!p) return false;
    p += strlen(pattern);
    int i = 0;
    while (*p && *p != '"' && i < bufsz - 1) buf[i++] = *p++;
    buf[i] = '\0';
    return i > 0;
}

/** Parse a SEND_PERIMETER JSON command (protocol v2: absolute WGS-84 lat/lon +
 *  per-point accuracy). JSON shape:
 *    {"cmd":"SEND_PERIMETER","v":2,"polygon":[[lat,lon,acc],...]}
 *  The firmware OWNS the ENU origin (first vertex); older ENU-offset uploads are
 *  rejected so a PWA/firmware version mismatch can't silently mis-place the
 *  perimeter. Stores the canonical lat/lon perimeter, then re-derives ENU. */
static void handle_send_perimeter(const char *json) {
    // Require protocol v2 — pre-v2 PWAs sent ENU offsets against a PWA-chosen
    // origin (the old "translating between schemes" error source). Fail loudly.
    if (!strstr(json, "\"v\":2")) {
        ble_server_send_ack("SEND_PERIMETER", false, "old PWA - update to v2 (lat/lon)");
        return;
    }

    // Parse polygon array of [lat, lon, acc] triples (acc optional).
    const char *poly_start = strstr(json, "\"polygon\":[");
    if (!poly_start) { ble_server_send_ack("SEND_PERIMETER", false, "no polygon"); return; }
    poly_start += strlen("\"polygon\":[");

    // Transient heap buffers (freed on return) — no permanent static buffer.
    std::vector<double> la, lo;
    std::vector<float>  ac;

    const char *p = poly_start;
    while (*p && (int)la.size() < MAX_PERIMETER_POINTS) {
        while (*p == ' ' || *p == '\t' || *p == ',') p++;
        if (*p == ']') break;            // end of polygon array
        if (*p != '[') { p++; continue; }
        p++;                             // skip '['
        double lat = 0.0, lon = 0.0; float acc = 0.0f;
        int consumed = 0;
        if (sscanf(p, "%lf,%lf,%f%n", &lat, &lon, &acc, &consumed) >= 2) {
            la.push_back(lat);
            lo.push_back(lon);
            ac.push_back(acc > 0.0f ? acc : 0.05f);   // tolerate a missing accuracy field
            p += consumed;
        }
        const char *close = strchr(p, ']');
        if (close) p = close + 1; else break;
    }
    int n = (int)la.size();

    if (n < 4) {
        ble_server_send_ack("SEND_PERIMETER", false, "too few points (need 4+)"); return;
    }

    // Firmware owns the ENU origin = first perimeter vertex. Setting it makes the
    // GPS module convert live fixes into the same ENU frame as the perimeter; the
    // EKF is reset so it re-seeds from GPS in the new frame on the next fix.
    rtk_gps_set_origin(la[0], lo[0]);
    ekf_init();
    DBG_PRINTF("[BLE] SEND_PERIMETER: %d pts, origin (%.7f,%.7f), EKF reset\r\n",
               n, la[0], lo[0]);

    // Build the ENU polygon (origin now set) for validation / auto-clean only.
    Polygon poly;
    poly.pts.reserve(n);
    for (int i = 0; i < n; i++) {
        float e = 0.0f, no = 0.0f;
        rtk_gps_latlon_to_enu(la[i], lo[i], &e, &no);
        poly.pts.push_back({e, no});
    }
    poly.ensureCCW();

    // Auto-clean spurs / self-intersecting loops drawn in the PWA editor.
    bool cleaned = false;
    if (isSelfIntersecting(poly)) {
        auto parts = splitSelfIntersecting(poly);
        if (parts.empty()) {
            ble_server_send_ack("SEND_PERIMETER", false,
                                "polygon self-intersects (could not auto-clean)");
            return;
        }
        int best = 0;
        for (int i = 1; i < (int)parts.size(); i++)
            if (parts[i].area() > parts[best].area()) best = i;
        poly = parts[best];
        poly.ensureCCW();
        cleaned = true;
        DBG_PRINTF("[BLE] SEND_PERIMETER: auto-cleaned self-intersecting polygon -> %d pts\n",
                   (int)poly.pts.size());
    }

    // Persist the canonical absolute lat/lon perimeter (the single source of truth).
    //  • Normal case: store the EXACT PWA lat/lon + per-point accuracy (no ENU
    //    round-trip, so per-corner confidence is preserved).
    //  • Self-intersecting (rare): the cleaned ENU geometry no longer maps 1:1 to
    //    the uploaded points, so fall back to the ENU->latlon round-trip tagged
    //    with the worst-case accuracy across all uploaded points.
    bool ok;
    if (!cleaned) {
        ok = perimeter_store_canonical_latlon(la.data(), lo.data(), ac.data(), n);
    } else {
        float worst = 0.0f;
        for (int i = 0; i < n; i++) if (ac[i] > worst) worst = ac[i];
        ok = perimeter_save_canonical(poly, worst > 0.0f ? worst : 0.05f);
    }
    if (!ok) { ble_server_send_ack("SEND_PERIMETER", false, "nvs save failed"); return; }

    // Re-derive everything (ENU perimeter, nav boundary, working area, breach
    // accuracy) from the canonical store, and re-persist the derived ENU blobs.
    perimeter_init();
    if (!perimeter_is_valid()) {
        ble_server_send_ack("SEND_PERIMETER", false, "perimeter too small for robot");
        return;
    }
    obstacle_map_init(perimeter_get_perimeter());

    // Generate mowing plan so the PWA can display the planned path.
    coverage_planner_plan(
        perimeter_get_perimeter(),
        perimeter_get_nav_boundary(),
        perimeter_get_working_area());

    g_ble_map_pending = true;
    ble_server_send_ack("SEND_PERIMETER", true, "ok");
}

static bool get_best_position(float &out_x, float &out_y);  // defined below handle_ble_command

/** Dispatch a BLE JSON command string to the appropriate handler. */
static void handle_ble_command(const char *json) {
    char cmd[32] = {};
    if (!json_get_str(json, "cmd", cmd, sizeof(cmd))) {
        ble_server_send_ack("?", false, "no cmd field");
        return;
    }

    // ── SET_MODE ──────────────────────────────────────────────────────────────
    if (strcmp(cmd, "SET_MODE") == 0) {
        char mode[16] = {};
        json_get_str(json, "value", mode, sizeof(mode));
        if (strcmp(mode, "AUTO") == 0) {
            if (!perimeter_is_valid()) {
                ble_server_send_ack(cmd, false, "no perimeter");
            } else {
                float px = 0.0f, py = 0.0f;
                if (!get_best_position(px, py)) {
                    ble_server_send_ack(cmd, false, "no GPS fix");
                } else {
                    const Polygon &perim = perimeter_get_perimeter();
                    // distanceToNearestEdge: positive=inside, negative=outside.
                    // Allow 0.5 m outside tolerance for GPS noise near boundary.
                    // The nav boundary breach watchdog is the real safety net during mowing.
                    float d = distanceToNearestEdge(perim, px, py);
                    if (d < -0.5f) {
                        char msg[64];
                        snprintf(msg, sizeof(msg), "outside perim %.1fm (E%.1f N%.1f)",
                                 -d, px, py);
                        ble_server_send_ack(cmd, false, msg);
                    } else {
                        transition_to(STATE_AUTO_MOWING);
                        ble_server_send_ack(cmd, true, "transitioning to AUTO");
                    }
                }
            }
        } else if (strcmp(mode, "MANUAL") == 0) {
            transition_to(STATE_MANUAL);
            ble_server_send_ack(cmd, true, "transitioning to MANUAL");
        } else if (strcmp(mode, "AUTO_RETURN") == 0) {
            transition_to(STATE_AUTO_RETURN);
            ble_server_send_ack(cmd, true, "transitioning to AUTO_RETURN");
        } else {
            ble_server_send_ack(cmd, false, "unknown mode");
        }
        return;
    }

    // ── SET_ARM ───────────────────────────────────────────────────────────────
    if (strcmp(cmd, "SET_ARM") == 0) {
        if (!crsf_is_failsafe()) {
            ble_server_send_ack(cmd, false, "RC active — use CH6");
            return;
        }
        const char *vp = strstr(json, "\"value\":");
        if (vp) {
            int v = 0;
            if (sscanf(vp + 8, "%d", &v) == 1) {
                s_ble_arm = (v != 0);
                ble_server_send_ack(cmd, true, s_ble_arm ? "armed" : "disarmed");
                return;
            }
        }
        ble_server_send_ack(cmd, false, "bad value");
        return;
    }

    // ── SET_CUT_HEIGHT ────────────────────────────────────────────────────────
    if (strcmp(cmd, "SET_CUT_HEIGHT") == 0) {
        const char *vp = strstr(json, "\"value\":");
        if (vp) {
            float mm = 0.0f;
            if (sscanf(vp + 8, "%f", &mm) == 1) {
                mm = clampf(mm, (float)mower_config_get().cut_height_min_mm, (float)mower_config_get().cut_height_max_mm);
                s_desired_cut_height_mm = mm;
                servo_set_height_mm(mm);
                ble_server_send_ack(cmd, true, "ok");
                return;
            }
        }
        ble_server_send_ack(cmd, false, "bad value");
        return;
    }

    // ── PAUSE_TOGGLE ─────────────────────────────────────────────────────────
    // Toggles BLE pause flag — OR'd with CH7 and GPIO pause switch.
    // Actual state transition handled by the common pause check each tick.
    if (strcmp(cmd, "PAUSE_TOGGLE") == 0) {
        s_ble_pause = !s_ble_pause;
        ble_server_send_ack(cmd, true, s_ble_pause ? "BLE pause ON" : "BLE pause OFF");
        return;
    }

    // ── REQUEST_MAP ──────────────────────────────────────────────────────────
    if (strcmp(cmd, "REQUEST_MAP") == 0) {
        g_ble_map_pending = true;
        ble_server_send_ack(cmd, true, "ok");
        return;
    }

    // ── REQUEST_STATUS ────────────────────────────────────────────────────────
    if (strcmp(cmd, "REQUEST_STATUS") == 0) {
        g_ble_status_pending = true;
        ble_server_send_ack(cmd, true, "ok");
        return;
    }

    // ── REQUEST_DIAG ─────────────────────────────────────────────────────────
    if (strcmp(cmd, "REQUEST_DIAG") == 0) {
        g_ble_diag_pending = true;
        ble_server_send_ack(cmd, true, "ok");
        return;
    }

    // ── RECAL_IMU ─────────────────────────────────────────────────────────────
    if (strcmp(cmd, "RECAL_IMU") == 0) {
        imu_recalibrate();
        sys_log_push("IMU: compass recalibration started (drive slow loops in Manual)");
        request_beep(BEEP_CONFIRM);
        return;
    }

    // ── IMUCALTEST — prove the NVS calibration path without recalibrating ──────
    if (strcmp(cmd, "IMUCALTEST") == 0) {
        bool ok = imu_nvs_selftest();
        ble_server_send_ack(cmd, ok, ok ? "NVS self-test PASS" : "NVS self-test FAIL");
        request_beep(ok ? BEEP_CONFIRM : BEEP_WARNING);
        return;
    }

    // ── IMUSAVECAL — save the current calibration now (read-back verified) ─────
    if (strcmp(cmd, "IMUSAVECAL") == 0) {
        if (!imu_heading_is_confident()) {
            ble_server_send_ack(cmd, false, "not fully calibrated — drive slow loops first");
            request_beep(BEEP_WARNING);
            return;
        }
        imu_request_save();   // performed on Core 0; result reported to the log
        sys_log_push("IMU: calibration save requested");
        ble_server_send_ack(cmd, true, "save requested — check log for verified result");
        request_beep(BEEP_CONFIRM);
        return;
    }

    // ── SEND_PERIMETER ────────────────────────────────────────────────────────
    if (strcmp(cmd, "SEND_PERIMETER") == 0) {
        handle_send_perimeter(json);
        return;
    }

    // ── CLEAR_OBSTACLES ───────────────────────────────────────────────────────
    if (strcmp(cmd, "CLEAR_OBSTACLES") == 0) {
        obstacle_map_reset();
        ble_server_send_ack(cmd, true, "obstacles cleared");
        return;
    }

    // ── CLEAR_PERIMETER ───────────────────────────────────────────────────────
    if (strcmp(cmd, "CLEAR_PERIMETER") == 0) {
        perimeter_clear();
        ble_server_send_ack(cmd, true, "perimeter cleared");
        return;
    }

    // ── PLAN_TEST — coverage-planner dry run on the stored polygons ──────────
    // Lets the operator verify (over BLE, in the field) that planning succeeds
    // on the real perimeter and see how many waypoints it produces, without
    // engaging AUTO. Only allowed at rest: it overwrites the planner's state,
    // which a PAUSED→AUTO resume depends on.
    if (strcmp(cmd, "PLAN_TEST") == 0) {
        if (g_state != STATE_IDLE && g_state != STATE_MANUAL) {
            ble_server_send_ack(cmd, false, "stop mower first (IDLE/MANUAL only)");
            return;
        }
        if (!perimeter_is_valid()) {
            ble_server_send_ack(cmd, false, "no perimeter stored");
            return;
        }
        bool ok = coverage_planner_plan(perimeter_get_perimeter(),
                                        perimeter_get_nav_boundary(),
                                        perimeter_get_working_area());
        char msg[64];
        if (ok) {
            snprintf(msg, sizeof(msg), "plan ok: %d waypoints, %d unreachable",
                     coverage_planner_get_waypoint_count(),
                     coverage_planner_get_unreachable_zone_count());
        } else {
            snprintf(msg, sizeof(msg), "plan FAILED (see log)");
        }
        sys_log_push(msg);

        // Dump the planned path geometry to the system log (see log_plan_stats).
        if (ok) {
            const std::vector<Waypoint> &wps = coverage_planner_get_waypoints();
            log_plan_stats(wps.data(), (int)wps.size(), ekf_get_pose());
        }

        ble_server_send_ack(cmd, ok, msg);
        return;
    }

    // ── SET_DRIVE (high-frequency, no ACK) ─────────────────────────────────
    if (strcmp(cmd, "SET_DRIVE") == 0) {
        float thr = 0.0f, str = 0.0f;
        const char *pt = strstr(json, "\"thr\":");
        if (pt) thr = strtof(pt + 6, nullptr);
        const char *ps = strstr(json, "\"str\":");
        if (ps) str = strtof(ps + 6, nullptr);
        s_ble_throttle = clampf(thr, -1.0f, 1.0f);
        s_ble_steering = clampf(str, -1.0f, 1.0f);
        s_ble_drive_ms = millis();
        return;  // no ACK — 10 Hz would flood BLE
    }

    // ── SET_LEARN ────────────────────────────────────────────────────────────
    if (strcmp(cmd, "SET_LEARN") == 0) {
        s_ble_learn = !s_ble_learn;
        if (s_ble_learn && (g_state == STATE_MANUAL || g_state == STATE_IDLE)) {
            transition_to(STATE_LEARN_PERIMETER);
            ble_server_send_ack(cmd, true, "learn mode ON");
        } else if (!s_ble_learn && g_state == STATE_LEARN_PERIMETER) {
            if (s_learn_first_point) {
                DBG_PRINTLN("[LEARN] BLE: No points recorded — exiting.");
                transition_to(STATE_IDLE);
                ble_server_send_ack(cmd, true, "learn OFF (no points)");
            } else {
                char err[64] = "";
                bool ok = perimeter_finish_recording(err);
                if (ok) {
                    GpsOrigin org = rtk_gps_get_origin();
                    if (org.set) {
                        rtk_gps_set_origin(org.lat_deg, org.lon_deg);
                    }
                    safety_set_perimeter(perimeter_get_perimeter());
                    ledFlashWhite3x();
                    ble_server_send_ack(cmd, true, "perimeter saved");
                } else {
                    // Save partial points so they appear on the map for editing
                    perimeter_save_partial();
                    ble_server_send_ack(cmd, false, err);
                }
                transition_to(STATE_IDLE);
            }
        } else {
            s_ble_learn = !s_ble_learn;  // revert toggle
            ble_server_send_ack(cmd, false, "wrong state for learn");
        }
        return;
    }

    // ── STORE_POINT ──────────────────────────────────────────────────────────
    if (strcmp(cmd, "STORE_POINT") == 0) {
        if (g_state != STATE_LEARN_PERIMETER) {
            ble_server_send_ack(cmd, false, "not in learn mode");
            return;
        }
        Pose2D pose = ekf_get_pose();
        GpsMeasurement gps = rtk_gps_get_measurement();
        if (s_learn_first_point) {
            perimeter_start_recording();
            s_learn_first_point = false;
        }
        float px = gps.valid ? gps.enu_east_m  : pose.x;
        float py = gps.valid ? gps.enu_north_m : pose.y;
        if (perimeter_record_point(px, py, rtk_gps_accuracy_m((int)gps.fix_type, gps.hdop), true)) {
            request_beep(BEEP_WARNING);
            char msg[48];
            snprintf(msg, sizeof(msg), "pt %d stored", perimeter_recording_point_count());
            ble_server_send_ack(cmd, true, msg);
        } else {
            ble_server_send_ack(cmd, false, "recording not active");
        }
        return;
    }

    // ── SET_CONFIG ────────────────────────────────────────────────────────────
    if (strcmp(cmd, "SET_CONFIG") == 0) {
        // Reject config changes while mowing — mid-flight geometry changes are unsafe
        if (g_state != STATE_IDLE && g_state != STATE_MANUAL) {
            ble_server_send_ack(cmd, false, "stop mower before changing config");
            return;
        }
        MowerConfig cfg = mower_config_get();  // start from current values
        // Helper lambdas for parsing — fall back to current value if field absent
        auto gf = [&](const char *key, float &dst) {
            const char *p = strstr(json, key);
            if (p) {
                p = strchr(p, ':');
                if (p) { float v = strtof(p + 1, nullptr); if (v != 0.0f || *(p+1) == '0') dst = v; }
            }
        };
        auto gi = [&](const char *key, auto &dst) {
            const char *p = strstr(json, key);
            if (p) {
                p = strchr(p, ':');
                if (p) { long v = strtol(p + 1, nullptr, 10); dst = v; }
            }
        };
        gf("\"footprint_width_m\"",      cfg.footprint_width_m);
        gf("\"footprint_length_m\"",     cfg.footprint_length_m);
        gf("\"track_width_m\"",          cfg.track_width_m);
        gf("\"wheel_radius_m\"",         cfg.wheel_radius_m);
        gi("\"motor_pole_pairs\"",        cfg.motor_pole_pairs);
        gf("\"gear_ratio\"",             cfg.gear_ratio);
        gf("\"antenna_fwd_m\"",          cfg.antenna_fwd_m);
        gf("\"antenna_right_m\"",        cfg.antenna_right_m);
        gf("\"steer_to_cut_m\"",         cfg.steer_to_cut_m);
        gf("\"cut_disc_radius_m\"",      cfg.cut_disc_radius_m);
        gf("\"cut_width_m\"",            cfg.cut_width_m);
        gf("\"strip_overlap_m\"",        cfg.strip_overlap_m);
        gi("\"cut_height_min_mm\"",       cfg.cut_height_min_mm);
        gi("\"cut_height_max_mm\"",       cfg.cut_height_max_mm);
        gi("\"blade_motor_pole_pairs\"",  cfg.blade_motor_pole_pairs);
        gi("\"blade_target_rpm\"",        cfg.blade_target_rpm);
        gf("\"pp_lookahead_base_m\"",    cfg.pp_lookahead_base_m);
        gf("\"pp_lookahead_k\"",         cfg.pp_lookahead_k);
        gf("\"max_mowing_speed_ms\"",    cfg.max_mowing_speed_ms);
        gf("\"headland_speed_ms\"",      cfg.headland_speed_ms);
        gf("\"transit_speed_ms\"",       cfg.transit_speed_ms);
        gf("\"min_creep_speed_ms\"",     cfg.min_creep_speed_ms);
        gf("\"waypoint_arrive_dist_m\"", cfg.waypoint_arrive_dist_m);
        gf("\"max_wheel_speed_ms\"",     cfg.max_wheel_speed_ms);
        gf("\"max_current_a\"",          cfg.max_current_a);
        gf("\"current_ramp_a_per_s\"",   cfg.current_ramp_a_per_s);
        gf("\"uncertainty_margin_m\"",   cfg.uncertainty_margin_m);
        gf("\"tilt_limit_normal_deg\"",  cfg.tilt_limit_normal_deg);
        gf("\"tilt_limit_careful_deg\"", cfg.tilt_limit_careful_deg);
        gf("\"collision_mult_careful\"", cfg.collision_mult_careful);
        gf("\"heading_kp\"",            cfg.heading_kp);
        gf("\"heading_kd\"",            cfg.heading_kd);
        gf("\"manual_max_yaw_rate\"",   cfg.manual_max_yaw_rate);
        gf("\"wheel_pi_kp\"",           cfg.wheel_pi_kp);
        gf("\"wheel_pi_ki\"",           cfg.wheel_pi_ki);
        gf("\"manual_max_duty\"",       cfg.manual_max_duty);
        gf("\"manual_max_speed_ms\"",  cfg.manual_max_speed_ms);
        gf("\"min_turn_radius_m\"",    cfg.min_turn_radius_m);
        gf("\"min_move_duty\"",        cfg.min_move_duty);

        // ── BNO055 calibration restore (carried in the settings file) ───────────
        // The PWA round-trips the saved calibration as "bnocal" (44 hex chars = 22
        // offset bytes) + "bnocalq" (quality 0..9). Restore it to NVS and the live
        // sensor. Independent of the MowerConfig above and of its validation.
        {
            const char *p = strstr(json, "\"bnocal\"");
            if (p && (p = strchr(p, ':')) && (p = strchr(p, '"'))) {
                p++;  // first hex char
                auto hexval = [](char c) -> int {
                    if (c >= '0' && c <= '9') return c - '0';
                    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
                    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
                    return -1;
                };
                uint8_t buf[22];
                int nb = 0;
                while (nb < 22) {
                    int hi = hexval(p[0]);
                    if (hi < 0) break;
                    int lo = hexval(p[1]);
                    if (lo < 0) break;
                    buf[nb++] = (uint8_t)((hi << 4) | lo);
                    p += 2;
                }
                if (nb == 22) {
                    int q = 0;
                    const char *pq = strstr(json, "\"bnocalq\"");
                    if (pq && (pq = strchr(pq, ':'))) q = (int)strtol(pq + 1, nullptr, 10);
                    if (q < 0) q = 0;
                    if (q > 9) q = 9;
                    imu_restore_cal(buf, (uint8_t)q);   // writes NVS + applies live
                }
            }
        }

        // Validate critical geometry constraints before saving
        if (cfg.cut_width_m <= cfg.strip_overlap_m + 0.01f) {
            ble_server_send_ack(cmd, false, "strip_overlap_m must be < cut_width_m");
            return;
        }
        if (cfg.max_mowing_speed_ms <= cfg.min_creep_speed_ms) {
            ble_server_send_ack(cmd, false, "max_mowing_speed_ms must be > min_creep_speed_ms");
            return;
        }
        if (cfg.wheel_radius_m <= 0.0f || cfg.gear_ratio <= 0.0f) {
            ble_server_send_ack(cmd, false, "wheel_radius_m and gear_ratio must be > 0");
            return;
        }
        if (cfg.blade_target_rpm == 0) {
            ble_server_send_ack(cmd, false, "blade_target_rpm must be > 0");
            return;
        }

        mower_config_set(cfg);

        // Re-derive navigation boundaries from new dimensions
        if (perimeter_is_valid()) {
            perimeter_recompute();  // recomputes nav and working area with new insets
            obstacle_map_init(perimeter_get_perimeter());
            if (g_state == STATE_AUTO_MOWING) {
                coverage_planner_plan(perimeter_get_perimeter(),
                                      perimeter_get_nav_boundary(),
                                      perimeter_get_working_area());
            }
        }
        g_ble_status_pending = true;  // send updated config back to PWA
        ble_server_send_ack(cmd, true, "config saved");
        return;
    }

    ble_server_send_ack(cmd, false, "unknown command");
}


// ══════════════════════════════════════════════════════════════════════════════
//  BLE public API
// ══════════════════════════════════════════════════════════════════════════════

void state_machine_enqueue_ble_cmd(const char *json, size_t len) {
    if (!s_ble_cmd_queue || len >= BLE_CMD_MAX_LEN) return;
    BleCmdSlot slot;
    memcpy(slot.json, json, len);
    slot.json[len] = '\0';
    xQueueSend(s_ble_cmd_queue, &slot, 0);  // non-blocking; drop if full
}

float state_machine_get_cut_height_mm() {
    return s_desired_cut_height_mm;
}

bool state_machine_is_armed() {
    return s_last_armed;
}

uint8_t state_machine_get_mode_sw() {
    return s_ch4_pos;
}

bool state_machine_is_paused_sw() {
    return s_ch7_on;
}

bool state_machine_is_learn_sw() {
    return s_ch5_on;
}

bool state_machine_has_perimeter() {
    return perimeter_is_valid();
}

float state_machine_get_perimeter_area_m2() {
    if (!perimeter_is_valid()) return 0.0f;
    return perimeter_get_perimeter().area();
}

void state_machine_ble_disconnected() {
    s_ble_throttle = 0.0f;
    s_ble_steering = 0.0f;
    // Stamp with current time (not 0) so the BLE_DRIVE_TIMEOUT_MS grace window
    // starts from the disconnect moment. Setting 0 would make the grace expire
    // immediately, causing MANUAL→IDLE if RC briefly glitches after BLE drops.
    s_ble_drive_ms = millis();
    s_ble_pause    = false;
    // Preserve the arm state on disconnect so reconnecting the phone
    // does not stop the blade. If RC is active, s_ble_arm is immediately
    // overwritten by s_ch6_on on the next tick anyway. If RC is in failsafe,
    // s_last_armed = s_ble_arm — clearing it here caused the blade to stop
    // the moment the phone reconnected while the TX was off (e.g. while
    // monitoring via WebUI with the TX powered down).
    s_ble_arm      = s_last_armed;
    s_ble_learn    = false;
}

bool state_machine_is_ble_paused()       { return s_ble_pause; }
bool state_machine_is_ble_armed()        { return s_ble_arm; }
bool state_machine_is_blade_commanded()  { return s_blade_commanded; }
// Blade battery lockout removed 2026-06-12 — always false; kept so the BLE
// status/diag JSON shape (and the PWA parsing it) doesn't need a breaking change.
bool state_machine_is_blade_lockout()    { return false; }
uint16_t state_machine_get_ch6_us()      { return crsf_get_channels().ch[CRSF_CH_ARM]; }


/**
 * Get the best available mower position in ENU (m).
 * Prefers GPS steering-centre measurement; falls back to EKF pose if GPS isn't valid.
 * Returns false if no position is available at all.
 */
static bool get_best_position(float &out_x, float &out_y) {
    GpsMeasurement gps_check = rtk_gps_get_measurement();
    if (gps_check.valid) {
        out_x = gps_check.enu_east_m;
        out_y = gps_check.enu_north_m;
        return true;
    }
    if (ekf_is_seeded()) {
        Pose2D pose = ekf_get_pose();
        out_x = pose.x;
        out_y = pose.y;
        return true;
    }
    return false;
}

/**
 * Returns true if the mower is inside (or within 0.5 m of) the perimeter polygon.
 * The 0.5 m outside tolerance handles GPS noise near the boundary; the nav boundary
 * breach watchdog is the real safety net during mowing.
 */
static bool mower_inside_perimeter() {
    float px = 0.0f, py = 0.0f;
    if (!get_best_position(px, py)) return false;
    const Polygon &perim = perimeter_get_perimeter();
    if (perim.pts.size() < 3) return false;
    return distanceToNearestEdge(perim, px, py) > -0.5f;
}

/**
 * Log + beep an AUTO-entry refusal ONCE per CH4-AUTO engagement.
 * Previously the denial beeped every 100 ms tick with no reason given —
 * the operator heard "repeated beeps, does nothing". The reason (with the
 * position actually used for the check) now lands in the BLE system log.
 * The latch is cleared when CH4 leaves the AUTO position.
 */
static bool s_auto_deny_latch = false;

static void deny_auto_entry(const char *who) {
    if (s_auto_deny_latch) return;
    s_auto_deny_latch = true;

    char line[SYS_LOG_MAX_LEN];
    float px = 0.0f, py = 0.0f;
    if (!get_best_position(px, py)) {
        snprintf(line, sizeof(line),
                 "%s: AUTO denied - NO POSITION (GPS invalid, EKF unseeded)", who);
    } else {
        float d = distanceToNearestEdge(perimeter_get_perimeter(), px, py);
        snprintf(line, sizeof(line),
                 "%s: AUTO denied - outside perim %.1fm at E%.1f N%.1f", who, -d, px, py);
    }
    sys_log_push(line);
    request_beep(BEEP_WARNING);
}

// ══════════════════════════════════════════════════════════════════════════════
//  State machine update (10 Hz tick)
// ══════════════════════════════════════════════════════════════════════════════

RobotState state_machine_get_state() {
    return g_state;
}

void state_machine_update() {
    // ── Drain BLE command queue (max 2 per tick to bound latency) ─────────────
    if (s_ble_cmd_queue) {
        BleCmdSlot slot;
        for (int i = 0; i < 2; i++) {
            if (xQueueReceive(s_ble_cmd_queue, &slot, 0) == pdTRUE) {
                handle_ble_command(slot.json);
            } else {
                break;
            }
        }
    }

    // ── Battery update at 2 Hz ────────────────────────────────────────────────
    if (++s_bat_div >= 5) {
        s_bat_div = 0;
        battery_monitor_update();
    }

    // ── 2 Hz JSON telemetry + BLE updates ────────────────────────────────────
    uint32_t now = millis();
    if (now - s_telem_last_ms >= 500) {
        s_telem_last_ms = now;
        emit_telemetry();
        ble_server_update();   // BLE telemetry (throttles internally)
    }

    if (g_ble_map_pending) {
        ble_server_send_map();
        g_ble_map_pending = false;
    }
    if (g_ble_status_pending) {
        ble_server_send_status();
        g_ble_status_pending = false;
    }
    if (g_ble_diag_pending) {
        ble_server_send_diag();
        g_ble_diag_pending = false;
    }

    // ── Compute state machine dt ────────────────────────────────────────────
    static uint32_t s_sm_last_ms = 0;
    float sm_dt;
    {
        uint32_t now_sm = millis();
        sm_dt = (s_sm_last_ms == 0) ? 0.1f : (now_sm - s_sm_last_ms) * 0.001f;
        s_sm_last_ms = now_sm;
    }

    // ── EKF prediction step ───────────────────────────────────────────────────
    // Feed actual VESC wheel velocities so the EKF tracks position between 1 Hz
    // GPS updates. Without this the predicted position never advances, causing
    // GPS innovations to grow with every metre driven and eventually exceed the
    // innovation gate, locking out all GPS corrections.
    {
        static uint32_t s_ekf_last_ms = 0;
        uint32_t now_ekf = millis();
        float dt = (s_ekf_last_ms == 0) ? 0.1f
                                        : (now_ekf - s_ekf_last_ms) * 0.001f;
        s_ekf_last_ms = now_ekf;
        // SCALED velocities: the GPS-referenced distance calibration feeds the EKF
        // (position) and the self-calibrator together.
        float v_left  = vesc_erpm_to_velocity_scaled(vesc_get_status(VESC_ID_LEFT).erpm);
        float v_right = vesc_erpm_to_velocity_scaled(vesc_get_status(VESC_ID_RIGHT).erpm);
        // Heading comes from the BNO055 absolute fusion + GPS-trimmed offset
        // inside ekf_predict() — no gyro/odometry heading term here.
        ekf_predict(v_left, v_right, dt);
        // Self-calibrate distance scale (straights) from GPS in every driving state.
        odo_calib_update(v_left, v_right, dt);
        // Persist the GPS-trimmed heading offset (throttled, NVS).
        ekf_save_heading_offset_if_due();
    }

    // ── Collect RC snapshot ───────────────────────────────────────────────────
    CRSFChannels rc = crsf_get_channels();
    // Keep the RC/BLE grace timer alive while the RC TX is active.
    // Without this, s_ble_drive_ms is only updated by BLE slider commands, so
    // on an RC-only setup it stays at 0 and any RC dropout immediately triggers
    // the MANUAL→IDLE failsafe transition regardless of switch position.
    if (!rc.failsafe) {
        s_ble_drive_ms = millis();
    }
    Pose2D pose     = ekf_get_pose();
    float speed     = ekf_get_speed();
    GpsMeasurement gps = rtk_gps_get_measurement();

    // ── RC link transition logging ────────────────────────────────────────────
    // Diagnoses the boot lockout: shows exactly WHEN valid CRSF frames started
    // (failsafe cleared) relative to boot and to any BLE connect in the log.
    {
        static int8_t s_prev_rc_link = -1;   // -1 = unknown, 0 = down, 1 = up
        int8_t rc_link = crsf_is_failsafe() ? 0 : 1;
        if (rc_link != s_prev_rc_link) {
            s_prev_rc_link = rc_link;
            char line[SYS_LOG_MAX_LEN];
            snprintf(line, sizeof(line), "RC link %s t=%lu",
                     rc_link ? "UP" : "DOWN", (unsigned long)millis());
            sys_log_push(line);
        }
    }

    // ── Decode switches with hysteresis ──────────────────────────────────────
    s_ch4_pos = sw3_decode(rc.ch[CRSF_CH_MODE], CH4_LO_US, CH4_HI_US, s_ch4_pos);
    s_ch5_on  = sw2_decode(rc.ch[CRSF_CH_LEARN], CH5_US, s_ch5_on);
    s_ch6_on  = sw2_decode(rc.ch[CRSF_CH_ARM],   CH6_US, s_ch6_on);
    s_ch7_on  = sw2_decode(rc.ch[CRSF_CH_PAUSE], CH7_US, s_ch7_on);
    // CH8 learn-point is latched in crsf_input (CRSF rate); consumed in LEARN.

    // Re-arm the AUTO-denial beep/log when CH4 leaves the AUTO position
    if (s_ch4_pos != 1) s_auto_deny_latch = false;

    // Update armed state (used by BLE getter and telemetry).
    // RC active: CH6 controls arm; BLE state follows RC.
    // RC failsafe + BLE connected: BLE controls arm.
    // RC failsafe + no BLE: HOLD the last known arm state.
    //   Brief RC dropouts would otherwise clear s_last_armed, set s_blade_commanded=false,
    //   and cause the keepalive to send SET_CURRENT=0 — producing the spike-then-stop
    //   pattern visible in VESC Tool diagnostics. The 5-second MANUAL→IDLE grace
    //   transition handles genuine TX-off events instead.
    if (!crsf_is_failsafe()) {
        s_last_armed = s_ch6_on;
        s_ble_arm = s_ch6_on;  // sync BLE state to RC when RC is active
    } else if (ble_server_is_connected()) {
        s_last_armed = s_ble_arm;
    }
    // else: RC failsafe without BLE — hold s_last_armed at last known RC value

    // Beep on armed edge (disarmed → armed)
    if (s_last_armed && !s_prev_armed) {
        request_beep(BEEP_WARNING);
    }
    s_prev_armed = s_last_armed;

    // Beep on CH4 mode switch change
    if (s_ch4_pos != s_prev_ch4_mode && s_prev_ch4_mode != 0xFF) {
        request_beep(BEEP_CONFIRM);
    }
    s_prev_ch4_mode = s_ch4_pos;

    // CH5 state for learn edge detection (latching switch)
    bool ch5_active = s_ch5_on;

    // CH7 state for pause level detection (latching switch)
    bool ch7_active = s_ch7_on;

    // ── Poll Serial for commands (non-blocking) ───────────────────────────────
    while (DBG_AVAILABLE() > 0) {
        char c = (char)DBG_READ();
        if (c == '\n' || c == '\r') {
            if (s_serial_len > 0) {
                s_serial_buf[s_serial_len] = '\0';
                state_machine_handle_serial(s_serial_buf);
                s_serial_len = 0;
            }
        } else if (s_serial_len < (uint8_t)(sizeof(s_serial_buf) - 1)) {
            s_serial_buf[s_serial_len++] = c;
        }
    }

    // ── Motors offline: always override when safety detects VESC silence ────────
    // PILZ has fired or battery disconnected — do NOT send CAN commands.
    if (safety_is_motors_offline() && g_state != STATE_MOTORS_OFFLINE) {
        transition_to(STATE_MOTORS_OFFLINE);
    }

    // ── Perimeter breach: safety task requested a pause ──────────────────────
    // Applies during AUTO states only (safety task gates this to s_in_auto_mode).
    if (safety_is_pause_requested()) {
        safety_clear_pause_request();
        if (g_state == STATE_AUTO_MOWING  || g_state == STATE_RETRACE ||
            g_state == STATE_BOG_RECOVERY || g_state == STATE_OBSTACLE_AVOID) {
            vesc_emergency_stop_all();
            nvs_log_estop(millis(), pose.x, pose.y, "Perimeter breach");
            DBG_PRINTLN("[SM] Perimeter breach — pausing (event latch set)");
            transition_to(STATE_PAUSED, true);
        }
    }

    // ── RC failsafe: only relevant when operator is driving ──────────────────
    // RC is switched off during autonomous operation by design.
    // Only stop if the operator was actively driving (MANUAL or LEARN_PERIMETER)
    // AND no BLE connection exists (phone may be controlling instead of RC).
    // When BLE is connected, the phone is the control source — stay in MANUAL.
    if (crsf_is_failsafe() && !ble_server_is_connected()
        && (millis() - s_ble_drive_ms) > BLE_DRIVE_TIMEOUT_MS) {
        if (g_state == STATE_MANUAL || g_state == STATE_LEARN_PERIMETER) {
            vesc_set_current(VESC_ID_LEFT,  0);
            vesc_set_current(VESC_ID_RIGHT, 0);
            vesc_set_current(VESC_ID_BLADE, 0);
            s_blade_commanded = false;
            s_blade_ramp_erpm = 0.0f;
            transition_to(STATE_IDLE);
        }
    }


    // ── Battery WARNING/LOW → operator notification only ─────────────────────
    // No automatic action (blade lockout removed 2026-06-12; auto-return also
    // removed 2026-06-12, both operator decisions — auto-return may come back
    // when the mower can self-charge). The VESCs manage low-voltage power
    // reduction internally; the OPERATOR decides whether to select Return.
    // Notification paths:
    //   • TX16S: warning beep on detection + repeat every 60 s, and flags bit
    //     0x20 in MOWER_STATUS drives a flashing banner on the Lua widget
    //   • WebUI: battState in telemetry — PWA toasts/beeps on the transition
    //   • Mower LEDs: amber overlay (applyBatteryWarningOverlay, existing)
    {
        static BatteryState s_prev_bs    = BATTERY_OK;
        static uint32_t     s_batt_beep_ms = 0;
        BatteryState bs = battery_get_state();
        if (bs != s_prev_bs) {
            s_prev_bs = bs;
            if (bs != BATTERY_OK) {
                char line[SYS_LOG_MAX_LEN];
                snprintf(line, sizeof(line), "BATTERY %s %.1fV - consider Return",
                         bs == BATTERY_LOW ? "LOW" : "WARNING", battery_get_voltage());
                sys_log_push(line);
                request_beep(BEEP_WARNING);
                s_batt_beep_ms = millis();
            } else {
                sys_log_push("BATTERY recovered - OK");
            }
        }
        // Audible reminder every 60 s while the warning persists
        if (bs != BATTERY_OK && millis() - s_batt_beep_ms >= 60000UL) {
            s_batt_beep_ms = millis();
            request_beep(BEEP_WARNING);
        }
    }

    // ── CH7 / GPIO / BLE pause → PAUSED from any active state ──────────────
    // Skip state logic this tick so AUTO/MANUAL can't re-command motors.
    bool pause_handled = false;
    if (g_state != STATE_INIT && g_state != STATE_IDLE &&
        g_state != STATE_PAUSED && g_state != STATE_MOTORS_OFFLINE) {
        bool pause_active = ch7_active || pauseSwitchActive() || s_ble_pause;
        if (pause_active) {
            vesc_set_current(VESC_ID_LEFT,  0);
            vesc_set_current(VESC_ID_RIGHT, 0);
            vesc_set_current(VESC_ID_BLADE, 0);
            s_blade_commanded  = false;
            s_blade_ramp_erpm  = 0.0f;
            safety_set_auto_mode(false);
            DBG_PRINTF("[SM] PAUSE — entering STATE_PAUSED (CH7=%d sw=%d ble=%d) from %d\n",
                          (int)ch7_active, (int)pauseSwitchActive(), (int)s_ble_pause, (int)g_state);
            collisionSaveBaselineForced();
            transition_to(STATE_PAUSED);
            pause_handled = true;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  State execute — skipped if common pause already triggered
    // ─────────────────────────────────────────────────────────────────────────
    if (!pause_handled) switch (g_state) {

    // ── STATE_INIT ────────────────────────────────────────────────────────────
    case STATE_INIT: {
        if (g_state_entry) {
            DBG_PRINTLN("[INIT] Waiting for RC, GPS origin, and IMU bias...");
            // Load perimeter and blade calibration from NVS (initialised before SM)
            perimeter_init();
            cutting_monitor_init();
        }

        showLedsWithGps(COL_BLUE);

        // Wait for: control source (RC or BLE).
        // GPS origin is NOT required here — manual drive works without GPS,
        // and AUTO_MOWING already gates on RTK fix before it starts.
        // Use crsf_is_failsafe() directly (timeout-aware, real-time) rather
        // than rc.failsafe (snapshot from before the 2.5 s IMU blocking call).
        bool rc_ready  = !crsf_is_failsafe();
        bool ble_ready = ble_server_is_connected();
        bool gps_ready = rtk_gps_has_origin();  // informational only

        if (g_state_entry) {
            ekf_init();
        }

        if (rc_ready || ble_ready) {
            DBG_PRINTF("[INIT] Prerequisites met (rc=%d ble=%d gps=%d) → STATE_IDLE\n",
                       (int)rc_ready, (int)ble_ready, (int)gps_ready);
            char line[SYS_LOG_MAX_LEN];
            snprintf(line, sizeof(line), "INIT done: rc=%d ble=%d gps=%d",
                     (int)rc_ready, (int)ble_ready, (int)gps_ready);
            sys_log_push(line);
            transition_to(STATE_IDLE);
        }
        break;
    }

    // ── STATE_IDLE ────────────────────────────────────────────────────────────
    case STATE_IDLE: {
        if (g_state_entry) {
            vesc_set_current(VESC_ID_LEFT,  0);
            vesc_set_current(VESC_ID_RIGHT, 0);
            vesc_set_current(VESC_ID_BLADE, 0);
            safety_set_auto_mode(false);
            s_blade_commanded = false;
            s_blade_ramp_erpm = 0.0f;
            s_pause_event_latch = false;  // clear any residual event latch on IDLE entry
            odo_calib_flush();            // persist any pending odometry calibration
            DBG_PRINTLN("[IDLE] Entered IDLE state.");
        }

        // ── Look-around animation (non-blocking) ─────────────────────────────
        // Left turn: left motor backward, right motor forward.
        // Right turn: left motor forward, right motor backward.
        if (s_look_phase != LOOK_NOT_STARTED && s_look_phase != LOOK_DONE) {
            uint32_t elapsed = millis() - s_look_ms;
            bool phase_done  = false;

            switch (s_look_phase) {
            case LOOK_LEFT:
                vesc_set_duty(VESC_ID_LEFT,  -LOOK_DUTY);
                vesc_set_duty(VESC_ID_RIGHT,  LOOK_DUTY);
                phase_done = (elapsed >= LOOK_TIME_90_MS);
                break;
            case LOOK_PAUSE_1:
                vesc_set_duty(VESC_ID_LEFT,  0.0f);
                vesc_set_duty(VESC_ID_RIGHT, 0.0f);
                phase_done = (elapsed >= LOOK_PAUSE_MS);
                break;
            case LOOK_RIGHT:
                // 2× duration: sweeps 180° to end up ~90° right of start
                vesc_set_duty(VESC_ID_LEFT,   LOOK_DUTY);
                vesc_set_duty(VESC_ID_RIGHT, -LOOK_DUTY);
                phase_done = (elapsed >= LOOK_TIME_90_MS * 2U);
                break;
            case LOOK_PAUSE_2:
                vesc_set_duty(VESC_ID_LEFT,  0.0f);
                vesc_set_duty(VESC_ID_RIGHT, 0.0f);
                phase_done = (elapsed >= LOOK_PAUSE_MS);
                break;
            case LOOK_CENTRE:
                // Return to start heading
                vesc_set_duty(VESC_ID_LEFT,  -LOOK_DUTY);
                vesc_set_duty(VESC_ID_RIGHT,  LOOK_DUTY);
                phase_done = (elapsed >= LOOK_TIME_90_MS);
                break;
            default:
                break;
            }

            if (phase_done) {
                s_look_ms = millis();
                s_look_phase = static_cast<LookPhase>(static_cast<uint8_t>(s_look_phase) + 1);
                if (s_look_phase == LOOK_DONE) {
                    vesc_set_duty(VESC_ID_LEFT,  0.0f);
                    vesc_set_duty(VESC_ID_RIGHT, 0.0f);
                    DBG_PRINTLN("[IDLE] Look-around complete");
                }
            }
        }

        showLedsWithGps(COL_BLUE);

        uint16_t ch4 = rc.ch[CRSF_CH_MODE];

        // CH4 == MANUAL → MANUAL, but ONLY when the operator actually moves a
        // stick — not on switch position alone. Gating on position alone made
        // this fight the MANUAL "throttle centred > 2 s → IDLE" auto-idle: with
        // CH4 held in the manual position and the sticks centred, the two flapped
        // MANUAL↔IDLE every ~2 s (1-tick IDLE, ~2.1 s MANUAL — spamming the log
        // and restarting drive each cycle). Requiring a deflected stick makes the
        // auto-idle stable: it rests in IDLE until the operator drives again.
        {
            float thr_in = crsf_us_to_norm(rc.ch[CRSF_CH_THROTTLE]);
            float str_in = crsf_us_to_norm(rc.ch[CRSF_CH_STEERING]);
            bool stick_active = (fabsf(thr_in) >= 0.05f) || (fabsf(str_in) >= 0.05f);
            if (ch4_is_manual(ch4) && !rc.failsafe && stick_active) {
                s_look_phase = LOOK_DONE;   // cancel look-around if still running
                transition_to(STATE_MANUAL);
                break;
            }
        }

        // CH5 activated → LEARN_PERIMETER
        if (ch5_active && !s_ch5_prev) {
            s_look_phase = LOOK_DONE;   // cancel look-around if still running
            transition_to(STATE_LEARN_PERIMETER);
            break;
        }

        // CH4 == AUTO
        if (ch4_is_auto(ch4)) {
            if (perimeter_is_valid()) {
                // GPS fix quality is not gated — uncertainty-aware navigation handles it.
                if (gps.fix_type < GPS_FIX_RTK_FIXED) {
                    DBG_PRINTF("[IDLE] AUTO starting with reduced GPS fix (%d) — "
                               "perimeter margin will be wider\n", (int)gps.fix_type);
                }
                if (mower_inside_perimeter()) {
                    transition_to(STATE_AUTO_MOWING);
                } else {
                    DBG_PRINTLN("[IDLE] AUTO blocked — mower outside perimeter");
                    deny_auto_entry("IDLE");   // beeps + logs reason once per engagement
                }
            } else {
                // No perimeter: trigger look-around once per session to indicate
                // that a perimeter must be taught before AUTO can run.
                if (s_look_phase == LOOK_NOT_STARTED && !safety_is_motors_offline()) {
                    s_look_phase = LOOK_LEFT;
                    s_look_ms    = millis();
                    DBG_PRINTLN("[IDLE] No perimeter — look-around triggered by CH4 AUTO");
                }
            }
            break;
        }
        break;
    }

    // ── STATE_MANUAL ──────────────────────────────────────────────────────────
    case STATE_MANUAL: {
        if (g_state_entry) {
            s_heading_active = false;
            s_drive_duty_l = 0.0f;
            s_drive_duty_r = 0.0f;
            heading_controller_reset();
            DBG_PRINTLN("[MANUAL] Entered MANUAL state.");
        }

        showLedsWithGps(COL_GREEN);

        // Cut height from CH3: only update s_desired_cut_height_mm when the
        // stick physically moves.  A resting CH3 (even at minimum) must not
        // override the WebUI slider — the two sources coexist, with the RC
        // taking over only when the operator deliberately moves the stick.
        {
            static uint16_t s_ch3_height_prev = 0xFFFF;  // 0xFFFF = uninitialised
            uint16_t ch3_raw = rc.ch[CRSF_CH_CUT_HEIGHT];
            if (s_ch3_height_prev == 0xFFFF) {
                s_ch3_height_prev = ch3_raw;  // record resting position, do not apply
            } else {
                uint16_t delta = (ch3_raw > s_ch3_height_prev)
                               ? (ch3_raw - s_ch3_height_prev)
                               : (s_ch3_height_prev - ch3_raw);
                if (delta >= CH3_HEIGHT_MOVE_THRESHOLD) {
                    s_ch3_height_prev = ch3_raw;
                    float ch3_norm    = crsf_us_to_norm(ch3_raw);
                    float height_range = (float)(mower_config_get().cut_height_max_mm
                                               - mower_config_get().cut_height_min_mm);
                    s_desired_cut_height_mm = (float)mower_config_get().cut_height_min_mm
                                           + (ch3_norm + 1.0f) * 0.5f * height_range;
                    s_desired_cut_height_mm = clampf(s_desired_cut_height_mm,
                                                     (float)mower_config_get().cut_height_min_mm,
                                                     (float)mower_config_get().cut_height_max_mm);
                }
            }
        }
        servo_set_height_mm(s_desired_cut_height_mm);

        // Blade control: armed = blade on (ramp up), disarmed = blade off (ramp down)
        if (s_last_armed && !s_blade_commanded) {
            s_blade_commanded = true;
            DBG_PRINTLN("[MANUAL] Armed — blade spinning up");
            sys_log_push("MANUAL: blade ON (duty ramp)");
        } else if (!s_last_armed && s_blade_commanded) {
            s_blade_commanded = false;
            DBG_PRINTLN("[MANUAL] Disarmed — blade spinning down");
            sys_log_push("MANUAL: blade OFF (freewheel)");
        }

        // Update cutting monitor so blade load is reported in MANUAL mode.
        // Previously only updated in AUTO states, so load always read 0 in MANUAL.
        {
            VescStatus blade = vesc_get_status(VESC_ID_BLADE);
            cutting_monitor_update(speed, blade.current_A, blade.erpm,
                                   s_blade_ramp_erpm, s_blade_commanded,
                                   battery_get_voltage());
        }

        // Apply manual drive with heading stabilisation
        drive_manual(rc, pose, sm_dt);

        // Transitions
        uint16_t ch4 = rc.ch[CRSF_CH_MODE];

        // CH4 == AUTO
        if (ch4_is_auto(ch4)) {
            if (perimeter_is_valid()) {
                // GPS fix quality is not gated — uncertainty-aware navigation handles it.
                if (gps.fix_type < GPS_FIX_RTK_FIXED) {
                    DBG_PRINTF("[MANUAL] AUTO starting with reduced GPS fix (%d) — "
                               "perimeter margin will be wider\n", (int)gps.fix_type);
                }
                if (mower_inside_perimeter()) {
                    vesc_set_current(VESC_ID_LEFT,  0);
                    vesc_set_current(VESC_ID_RIGHT, 0);
                    transition_to(STATE_AUTO_MOWING);
                } else {
                    DBG_PRINTLN("[MANUAL] AUTO blocked — mower outside perimeter");
                    deny_auto_entry("MANUAL");  // beeps + logs reason once per engagement
                }
            } else {
                // No perimeter: trigger look-around once per session, then go to IDLE
                // where the animation loop runs.
                if (s_look_phase == LOOK_NOT_STARTED && !safety_is_motors_offline()) {
                    s_look_phase = LOOK_LEFT;
                    s_look_ms    = millis();
                    DBG_PRINTLN("[MANUAL] No perimeter — look-around triggered by CH4 AUTO");
                }
                vesc_set_current(VESC_ID_LEFT,  0);
                vesc_set_current(VESC_ID_RIGHT, 0);
                transition_to(STATE_IDLE);
            }
            break;
        }

        // CH5 activated → LEARN_PERIMETER
        if (ch5_active && !s_ch5_prev) {
            vesc_set_current(VESC_ID_LEFT,  0);
            vesc_set_current(VESC_ID_RIGHT, 0);
            transition_to(STATE_LEARN_PERIMETER);
            break;
        }

        break;
    }

    // ── STATE_LEARN_PERIMETER ─────────────────────────────────────────────────
    case STATE_LEARN_PERIMETER: {
        if (g_state_entry) {
            s_learn_first_point = true;
            // Ignore any CH8 presses made before entering LEARN.
            s_learn_pt_seen = crsf_get_learn_pt_events();
            s_heading_active = false;
            heading_controller_reset();
            vesc_set_current(VESC_ID_BLADE, 0);
            s_blade_commanded = false;
            s_blade_ramp_erpm = 0.0f;

            // Save current GPS position as the return-to-home target.
            // This is persisted so RETURN mode comes back here after any reboot.
            GpsMeasurement gps_home = rtk_gps_get_measurement();
            if (gps_home.valid) {
                s_home_x = gps_home.enu_east_m;
                s_home_y = gps_home.enu_north_m;
                nvs_set_float("home_x", s_home_x);
                nvs_set_float("home_y", s_home_y);
                DBG_PRINTF("[LEARN] Home position saved: (%.3f, %.3f)\n", s_home_x, s_home_y);
            }

            DBG_PRINTLN("[LEARN] Ready. Press CH8 at each corner.");
        }

        showLedsWithGps(COL_ORANGE);

        // Drive with heading stabilisation while recording
        drive_manual(rc, pose, sm_dt);

        // Record a point for each CH8 momentary press. The press edge is latched
        // in crsf_input at the CRSF frame rate, so even a brief tap registers
        // (the old 10 Hz poll dropped short presses). Consume all pending events
        // but record only once per tick — distinct corners are always >1 tick
        // apart, so collapsing here only de-dups physically-impossible doubles.
        uint32_t learn_ev = crsf_get_learn_pt_events();
        if (learn_ev != s_learn_pt_seen) {
            s_learn_pt_seen = learn_ev;
            // First point clears old perimeter and starts fresh recording
            if (s_learn_first_point) {
                perimeter_start_recording();
                s_learn_first_point = false;
            }
            // Use live GPS ENU (steering-centre corrected) rather than EKF pose.
            // force=true bypasses the distance gate — the operator explicitly
            // chose this position by pressing the button.
            float px = gps.valid ? gps.enu_east_m  : pose.x;
            float py = gps.valid ? gps.enu_north_m : pose.y;
            if (perimeter_record_point(px, py, rtk_gps_accuracy_m((int)gps.fix_type, gps.hdop), true)) {
                request_beep(BEEP_WARNING);
                DBG_PRINTF("[LEARN] Point %d stored (%.3f, %.3f)\n",
                              perimeter_recording_point_count(), px, py);
            }
        }

        // CH5 deactivated → attempt to close and save
        if (!ch5_active && s_ch5_prev) {
            if (s_learn_first_point) {
                DBG_PRINTLN("[LEARN] No points recorded — exiting.");
                transition_to(STATE_IDLE);
                break;
            }
            char err[64] = "";
            bool ok = perimeter_finish_recording(err);
            if (ok) {
                // Persist the auto-seeded ENU origin so the stored perimeter
                // coordinates remain valid after a reboot.
                GpsOrigin org = rtk_gps_get_origin();
                if (org.set) {
                    rtk_gps_set_origin(org.lat_deg, org.lon_deg);
                }
                safety_set_perimeter(perimeter_get_perimeter());
                ledFlashWhite3x();
                DBG_PRINTF("[LEARN] Perimeter saved (%d points).\r\n",
                    (int)perimeter_get_perimeter().pts.size());
            } else {
                DBG_PRINTF("[LEARN] Perimeter rejected: %s\r\n", err);
                perimeter_save_partial();
            }
            transition_to(STATE_IDLE);
            break;
        }
        break;
    }

    // ── STATE_AUTO_MOWING ─────────────────────────────────────────────────────
    case STATE_AUTO_MOWING: {
        static uint32_t s_auto_entry_ms = 0;  // time AUTO was entered; used to gate obstacle detection
        static float    s_auto_entry_x  = 0;  // mower position at AUTO entry — segment start
        static float    s_auto_entry_y  = 0;  // for the index-0 pass-through projection
        if (g_state_entry) {
            heading_controller_reset();
            DBG_PRINTLN("[AUTO] Entering AUTO_MOWING — planning path...");
            s_session_start_ms = millis();
            s_auto_entry_ms    = millis();
            s_auto_entry_x     = pose.x;
            s_auto_entry_y     = pose.y;
            // When resuming from PAUSED, the coverage planner already has a
            // valid plan (or was re-generated by MOTORS_OFFLINE resume logic).
            // Skip re-planning; just reload waypoints from where we left off.
            bool resuming = (g_prev_state == STATE_PAUSED);

            if (!resuming) {
                // Plan the full coverage path
                bool plan_ok = coverage_planner_plan(
                    perimeter_get_perimeter(),
                    perimeter_get_nav_boundary(),
                    perimeter_get_working_area());

                if (!plan_ok) {
                    DBG_PRINTLN("[AUTO] Coverage planning failed — returning to IDLE.");
                    sys_log_push("AUTO: coverage plan FAILED -> IDLE");
                    request_beep(BEEP_FAULT);   // audible fault on TX16S
                    transition_to(STATE_IDLE);
                    break;
                }
                // Reset collision baseline settle distance (fresh session only)
                node_follower_reset_session_distance();
            }

            // Load all waypoints into local buffer for node follower
            s_wp_count = load_waypoints_from_planner();
            s_wp_index = 0;
            DBG_PRINTF("[AUTO] Plan ready: %d waypoints (resuming=%d).\r\n",
                          s_wp_count, (int)resuming);

            if (s_wp_count == 0) {
                // Plan succeeded but produced no waypoints — shouldn't normally
                // happen; treat as a planning failure and give audible feedback.
                DBG_PRINTLN("[AUTO] Plan empty — returning to IDLE.");
                sys_log_push("AUTO: plan EMPTY (0 waypoints) -> IDLE");
                request_beep(BEEP_FAULT);
                transition_to(STATE_IDLE);
                break;
            }
            {
                char line[SYS_LOG_MAX_LEN];
                snprintf(line, sizeof(line), "AUTO: plan ok, %d waypoints (resume=%d)",
                         s_wp_count, (int)resuming);
                sys_log_push(line);
            }
            // Dump the planned path geometry to the system log on every AUTO start,
            // so it is readable over BLE without the PWA "Plan Test" button.
            log_plan_stats(s_wp_buf, s_wp_count, pose);
            // Audible confirm: one short beep on TX16S so operator knows mowing has started
            request_beep(BEEP_CONFIRM);

            // Log unreachable zones
            coverage_planner_log_unreachable_zones();

            // Start blade auto-calibration (or restart it after a pause)
            cutting_monitor_start_auto_cal();

            // Enable safety perimeter breach checking
            safety_set_auto_mode(true);

            // Reset cross-track timer
            s_cross_track_exceed_ms = 0;

            node_follower_reset_stall();
            wheel_duty_ramp_reset(); // zero stale duty from manual driving; prevents carry-over reverse
            collisionClear();        // flush stale jolt data from manual driving
        }

        // ── Blade follows arm switch: armed = blade on (ramp up), disarmed = blade off (ramp down)
        if (s_last_armed && !s_blade_commanded) {
            s_blade_commanded = true;
            DBG_PRINTLN("[AUTO] Armed — blade spinning up");
            sys_log_push("AUTO: blade ON (duty ramp)");
        } else if (!s_last_armed && s_blade_commanded) {
            s_blade_commanded = false;
            DBG_PRINTLN("[AUTO] Disarmed — blade spinning down");
            sys_log_push("AUTO: blade OFF (freewheel)");
        }

        // ── Mode-switch override — check FIRST so it is never masked by
        //    tilt/collision/pause checks that break early to PAUSED.
        {
            uint16_t ch4 = rc.ch[CRSF_CH_MODE];
            if (ch4_is_manual(ch4)) {
                vesc_set_current(VESC_ID_LEFT,  0);
                vesc_set_current(VESC_ID_RIGHT, 0);
                vesc_set_current(VESC_ID_BLADE, 0);
                s_blade_commanded = false;
                s_blade_ramp_erpm = 0.0f;
                safety_set_auto_mode(false);
                DBG_PRINTLN("[AUTO] CH4→MANUAL: saving planner state.");
                collisionSaveBaselineForced();
                transition_to(STATE_MANUAL);
                break;
            }
        }

        showLedsWithGps(COL_RED);

        // Update cutting monitor — use ramp ERPM as target so BLADE_FAULT
        // detection is accurate during blade spin-up (VESC-internal ~2 s RPM ramp).
        {
            VescStatus blade = vesc_get_status(VESC_ID_BLADE);
            cutting_monitor_update(speed,
                blade.current_A,
                blade.erpm,
                s_blade_ramp_erpm,
                s_blade_commanded,
                battery_get_voltage());
        }

        // Check cutting status.
        // In debug mode, VESCs may be offline so blade faults, stalls, and
        // overloads are expected — log them but don't change state.
        CuttingStatus cs = cutting_monitor_get_status();

        // Gate obstacle/fault responses for OBSTACLE_DETECT_STARTUP_MS after AUTO
        // entry. At entry the mower is stationary and the blade is spinning up,
        // which produces exactly the OBSTACLE_SUSPECTED / STALLED / OVERLOADED
        // signatures — suppress them until the mower is actually moving.
        // Declared here (outside #if) so it is also in scope for the wheel-stall
        // check further below.
        bool obs_armed = (millis() - s_auto_entry_ms) >= OBSTACLE_DETECT_STARTUP_MS;

        if (cs != CUTTING_NORMAL) {
            DBG_PRINTF("[SM] Cutting status: %d\n", (int)cs);
        }
#if !BENCH_TEST_NO_VESC
        // Blade fault (commanded but no current/rpm) must NOT change the drive mode:
        // the blade is the operator's to control (CH6) and there is no blade<->mode
        // interlock. Warn once per fault episode and keep driving the path; the blade
        // keepalive keeps issuing SET_RPM so a transient fault self-recovers.
        {
            static bool s_blade_fault_warned = false;
            if (cs == BLADE_FAULT) {
                if (!s_blade_fault_warned) {
                    s_blade_fault_warned = true;
                    DBG_PRINTLN("[SM] Blade not cutting (commanded, no rpm) — driving on");
                    sys_log_push("AUTO: blade not cutting (commanded, no rpm) - driving on");
                    request_beep(BEEP_WARNING);
                }
            } else {
                s_blade_fault_warned = false;
            }
        }

        // ── RPM-load recovery trigger (Feature 2) ──────────────────────────────
        // INDEPENDENT of AUTO_FAULT_RESPONSES_ENABLED: enabling this does NOT wake
        // the old current-overload / stall / slip / obstacle detectors. Fires on a
        // SUSTAINED high RPM-based load and hosts the hybrid back-up + height-step-
        // down maneuver in STATE_RETRACE. The blade is LEFT ARMED (deck-only
        // recovery) so it keeps cutting via the keepalive. Default OFF
        // (BLADE_RPM_RECOVERY_ENABLED 0) — verify the Tx load reading first.
        {
            static uint32_t s_rpm_load_high_since_ms = 0;
            float rpm_load = cutting_monitor_get_rpm_load_fraction();
            if (BLADE_RPM_RECOVERY_ENABLED && obs_armed && s_blade_commanded
                && rpm_load >= BLADE_RPM_RECOVERY_LOAD) {
                if (s_rpm_load_high_since_ms == 0) {
                    s_rpm_load_high_since_ms = millis();
                } else if ((millis() - s_rpm_load_high_since_ms) >= BLADE_RPM_RECOVERY_CONFIRM_MS) {
                    s_rpm_load_high_since_ms = 0;
                    char l[SYS_LOG_MAX_LEN];
                    snprintf(l, sizeof(l), "AUTO: rpm-load %.0f%% sustained - blade recovery",
                             (double)(rpm_load * 100.0f));
                    sys_log_push(l);
                    collisionSaveBaselineForced();
                    // Do NOT zero the blade: recovery raises the deck only; the
                    // blade keepalive keeps it spinning so it can re-cut the patch.
                    transition_to(STATE_RETRACE);
                    break;
                }
            } else {
                s_rpm_load_high_since_ms = 0;
            }
        }

        if (AUTO_FAULT_RESPONSES_ENABLED && obs_armed && cs == CUTTING_OVERLOADED) {
            // Save strip end for retrace
            if (s_wp_index < s_wp_count) {
                s_retrace_strip_end_x = s_wp_buf[s_wp_index].x;
                s_retrace_strip_end_y = s_wp_buf[s_wp_index].y;
            } else {
                s_retrace_strip_end_x = pose.x;
                s_retrace_strip_end_y = pose.y;
            }
            vesc_set_current(VESC_ID_BLADE, 0);
            s_blade_commanded = false;
            s_blade_ramp_erpm = 0.0f;
            collisionSaveBaselineForced();
            transition_to(STATE_RETRACE);
            break;
        }
        if (AUTO_FAULT_RESPONSES_ENABLED && obs_armed && cs == CUTTING_STALLED) {
            vesc_set_current(VESC_ID_BLADE, 0);
            s_blade_commanded = false;
            s_blade_ramp_erpm = 0.0f;
            collisionSaveBaselineForced();
            transition_to(STATE_BOG_RECOVERY);
            break;
        }
        // OBSTACLE_SUSPECTED means blade-current low + mower not moving.
        // Only meaningful when blade is actually commanded — the condition is
        // trivially true (blade load = 0) whenever the blade is off.
        if (AUTO_FAULT_RESPONSES_ENABLED && obs_armed && s_blade_commanded && cs == OBSTACLE_SUSPECTED) {
            vesc_set_current(VESC_ID_BLADE, 0);
            s_blade_commanded = false;
            s_blade_ramp_erpm = 0.0f;
            s_lastCollisionDir = COLLISION_DIR_UNKNOWN;
            collisionSaveBaselineForced();
            transition_to(STATE_OBSTACLE_AVOID);
            break;
        }
#endif  // !BENCH_TEST_NO_VESC

        // ── Uncertainty-aware perimeter navigation ─────────────────────────
        // Distance is to the PERIMETER (the steering-centre limit), not the inset
        // nav boundary — the centre may drive up to the perimeter, so speed only
        // ramps down approaching the perimeter itself (not half a robot earlier).
        float uncertainty = ekf_get_position_uncertainty();
        float dist_to_edge = distanceToNearestEdge(perimeter_get_perimeter(),
                                                    pose.x, pose.y);
        float margin = dist_to_edge - uncertainty;
        const MowerConfig &mc_ua = mower_config_get();

        // Speed scale: PURE proximity factor — 1.0 when margin >= threshold,
        // ramps linearly to 0 at margin=0. Do NOT floor it here with a
        // min_creep/max_mowing fraction: that made max_mowing_speed_ms cancel
        // (v = max_mowing × min_creep/max_mowing = min_creep), so the mow-speed
        // setting had no effect near the perimeter — the outer spiral rings, which
        // ARE the perimeter, so margin<=0 there. The absolute min_creep floor is
        // applied downstream in node_follower (after this scale).
        float speed_scale = clampf(margin / mc_ua.uncertainty_margin_m, 0.0f, 1.0f);

        // Collision sensitivity: lower multiplier when margin is low
        bool careful = (margin < mc_ua.uncertainty_margin_m);
        collisionDetectSetMultiplier(careful ? mc_ua.collision_mult_careful
                                             : COLLISION_THRESHOLD_MULTIPLIER);

        // Tilt check: use stricter limit when careful
        float tilt_deg = imu_get_tilt_rad() * (180.0f / M_PI);
        float tilt_limit = careful ? mc_ua.tilt_limit_careful_deg
                                   : mc_ua.tilt_limit_normal_deg;
        if (tilt_deg > tilt_limit) {
            DBG_PRINTF("[AUTO] Tilt %.1f > limit %.1f — pausing\n",
                          tilt_deg, tilt_limit);
            {
                char line[SYS_LOG_MAX_LEN];
                snprintf(line, sizeof(line), "AUTO: tilt %.1f > %.1f deg", tilt_deg, tilt_limit);
                sys_log_push(line);
            }
            vesc_set_current(VESC_ID_LEFT,  0);
            vesc_set_current(VESC_ID_RIGHT, 0);
            vesc_set_current(VESC_ID_BLADE, 0);
            s_blade_commanded = false;
            s_blade_ramp_erpm = 0.0f;
            collisionSaveBaselineForced();
            transition_to(STATE_PAUSED);
            break;
        }

        // Strip truncation: if margin <= 0 and on a mowing STRIP waypoint, skip
        // the rest of that strip. Headland passes are EXEMPT: they are flagged
        // mowing=true but are deliberately driven along the perimeter edge, where
        // margin is naturally <= 0 (especially under RTK-Float uncertainty).
        // Without this exemption the uncertainty logic skipped the entire headland
        // on AUTO entry ("skip strip wp 0..35"), so the mower never followed the
        // perimeter round to home. Only true working strips (headland=false) are
        // truncated here.
        if (margin <= 0.0f && s_wp_index < s_wp_count
            && s_wp_buf[s_wp_index].mowing && !s_wp_buf[s_wp_index].headland) {
            DBG_PRINTF("[AUTO] Margin=%.2f — skipping rest of strip at wp %d\n",
                          margin, s_wp_index);
            int skip_to = s_wp_index;
            while (skip_to < s_wp_count && s_wp_buf[skip_to].mowing
                   && !s_wp_buf[skip_to].headland) {
                skip_to++;
            }
            {
                char line[SYS_LOG_MAX_LEN];
                snprintf(line, sizeof(line),
                         "AUTO margin %.2f<=0 -> skip strip wp %d..%d",
                         margin, s_wp_index, skip_to);
                sys_log_push(line);
            }
            s_wp_index = skip_to;
        }

        // Check plan complete.
        // Use ONLY the local waypoint counter. The global coverage_planner_is_complete()
        // is unusable here: load_waypoints_from_planner() drains the planner via
        // coverage_planner_get_next(), which runs the planner's internal g_wp_index up to
        // the end the instant the plan loads — so is_complete() is true on the very first
        // AUTO tick and used to bounce AUTO straight back to IDLE every tick. s_wp_index
        // (advanced at waypoint-reached, below) is the authoritative progress tracker.
        bool plan_done = (s_wp_index >= s_wp_count);
        if (plan_done) {
            vesc_set_current(VESC_ID_LEFT,  0);
            vesc_set_current(VESC_ID_RIGHT, 0);
            vesc_set_current(VESC_ID_BLADE, 0);
            s_blade_commanded = false;
            s_blade_ramp_erpm = 0.0f;
            safety_set_auto_mode(false);
            collisionSaveBaselineForced();

            uint16_t ch4 = rc.ch[CRSF_CH_MODE];
            if (ch4_is_auto_return(ch4)) {
                transition_to(STATE_AUTO_RETURN);
            } else {
                transition_to(STATE_IDLE);
            }
            break;
        }

        // Execute node follower
        if (s_wp_index < s_wp_count) {
            // Heading comes from the BNO055 (NDOF auto-calibrates continuously) +
            // GPS-trimmed offset. AUTO does NOT gate on calibration status — the
            // fused heading is usable from the start and GPS corrects any bias.
            WheelCmd cmd = node_follower_compute(pose, speed,
                                                 s_wp_buf, s_wp_count, s_wp_index,
                                                 s_desired_cut_height_mm, speed_scale);

            // Per-track velocity cap: ALLOW REVERSE on a track so the tracked
            // vehicle can counter-rotate (tank steer / pivot on the spot) to reach
            // tight corner nodes. Previously clamped to [0, max_v] (forward only),
            // which killed every tight turn — the inner track could never slow
            // below zero, so the minimum turn radius was ~half the track width and
            // sharp nodes were overshot. Now bounded to ±max_v in both directions.
            {
                // Clamp to the mechanical wheel-speed limit, NOT max_mowing_speed_ms.
                // transit_speed_ms (0.30) and headland_speed_ms (0.20) are MEANT to
                // exceed mow speed (0.15); clamping to max_mowing silently capped them
                // to mow speed, so the transit slider did nothing above mow speed.
                // node_follower already scales wheels to <= max_wheel_speed_ms; this is
                // just the outer bound. Keep the +/- so a track can counter-rotate
                // (tank steer / pivot on the spot) to reach tight corner nodes.
                float max_v = mower_config_get().max_wheel_speed_ms;
                cmd.left_ms  = clampf(cmd.left_ms,  -max_v, max_v);
                cmd.right_ms = clampf(cmd.right_ms, -max_v, max_v);
            }
            // Duty-cycle ramp toward desired wheel velocities (ramp is reset on AUTO entry)
            node_follower_to_vesc_duty(cmd);

            // ── AUTO drive diagnostic → PWA log (no field serial access) ───────
            // Throttled to 0.5 Hz so it doesn't flood the 50-entry sys_log ring and
            // bury state-transition / fault entries. Lets the operator confirm from
            // the PWA Diagnostics log that each speed slider now changes the desired
            // wheel speed, that the perimeter slowdown still ramps `scale`, and
            // whether the mower is stuck pivoting (`piv`, a separate heading issue).
            {
                static uint32_t s_drv_log_ms = 0;
                if ((uint32_t)(millis() - s_drv_log_ms) >= 2000u) {
                    s_drv_log_ms = millis();
                    float dl = 0.0f, dr = 0.0f;
                    node_follower_get_last_duty(&dl, &dr);
                    char line[SYS_LOG_MAX_LEN];
                    snprintf(line, sizeof(line),
                             "AUTO drv des=%.2f/%.2f duty=%.2f/%.2f scale=%.2f piv=%d",
                             (double)cmd.left_ms, (double)cmd.right_ms,
                             (double)dl, (double)dr, (double)speed_scale,
                             node_follower_is_pivoting() ? 1 : 0);
                    sys_log_push(line);
                }
            }

            // ── Robust waypoint advancement (reached OR passed) ───────────────
            // Advancing only on a tight physical-arrival window (waypoint_arrive_dist_m,
            // default 0.15 m) is fragile: the steering centre overshoots corners and
            // GPS/EKF noise means the window is essentially never hit, so the index
            // sticks and the node-follower lookahead stays anchored to one waypoint —
            // producing the ~180° U-turn weave. Also clear a waypoint once the mower has
            // driven PAST it (projection onto the incoming segment t >= 1), keeping
            // progress monotonic. Reverse spurs back toward their point (inverted
            // projection sign), so those still advance on arrival only.
            {
                const MowerConfig &mc_adv = mower_config_get();
                const float arrive   = mc_adv.waypoint_arrive_dist_m;
                const float arrive2  = arrive * arrive;
                float lookahead_est  = max(mc_adv.pp_lookahead_base_m,
                                           mc_adv.pp_lookahead_k * speed);
                const float near2    = (lookahead_est + arrive) * (lookahead_est + arrive);

                int advanced = 0;
                while (s_wp_index < s_wp_count && advanced < 4) {
                    const Waypoint &w = s_wp_buf[s_wp_index];
                    float dxr  = w.x - pose.x, dyr = w.y - pose.y;
                    float d2   = dxr * dxr + dyr * dyr;

                    bool reached = (d2 <= arrive2);

                    bool passed = false;
                    if (!reached && !w.reverse) {
                        float ax = (s_wp_index > 0) ? s_wp_buf[s_wp_index - 1].x : s_auto_entry_x;
                        float ay = (s_wp_index > 0) ? s_wp_buf[s_wp_index - 1].y : s_auto_entry_y;
                        float sx = w.x - ax, sy = w.y - ay;
                        float seg2 = sx * sx + sy * sy;
                        if (seg2 > 1e-6f) {
                            float t = ((pose.x - ax) * sx + (pose.y - ay) * sy) / seg2;
                            // Gate on proximity to the segment end so a wildly off-track
                            // pose can't skip ahead — cross-track recovery handles that.
                            passed = (t >= 1.0f) && (d2 <= near2);
                        }
                    }

                    if (reached || passed) { s_wp_index++; advanced++; }
                    else break;
                }
            }

            // ── AUTO nav diagnostics (~1 Hz, surfaced in the PWA System Log) ──
            {
                static uint32_t s_nav_log_ms = 0;
                if ((millis() - s_nav_log_ms) >= 1000 && s_wp_index < s_wp_count) {
                    s_nav_log_ms = millis();
                    const Waypoint &w = s_wp_buf[s_wp_index];
                    float d = sqrtf((w.x - pose.x) * (w.x - pose.x)
                                  + (w.y - pose.y) * (w.y - pose.y));
                    // Bearing to target and own heading, both deg CW-from-North,
                    // so we can tell a circling/heading problem (h sweeps while b
                    // holds) from a plan/arrival problem (h tracks b, no progress).
                    float brg = atan2f(w.x - pose.x, w.y - pose.y) * 57.29578f;
                    if (brg < 0.0f) brg += 360.0f;
                    float hdg = pose.heading * 57.29578f;
                    if (hdg < 0.0f) hdg += 360.0f;
                    char line[SYS_LOG_MAX_LEN];
                    snprintf(line, sizeof(line),
                             "AUTO i=%d/%d pos=%.1f,%.1f h=%.0f tgt=%.1f,%.1f b=%.0f d=%.2f x=%.2f",
                             s_wp_index, s_wp_count, pose.x, pose.y, hdg,
                             w.x, w.y, brg, d,
                             node_follower_get_cross_track_error());
                    sys_log_push(line);
                }
            }

            // Cross-track error check: if |XTE| > 0.5 m for > 2 s → reset to nearest
            float xte = fabsf(node_follower_get_cross_track_error());
            if (xte > 0.5f) {
                if (s_cross_track_exceed_ms == 0) s_cross_track_exceed_ms = millis();
                if (millis() - s_cross_track_exceed_ms > 2000) {
                    DBG_PRINTLN("[AUTO] Cross-track error > 0.5 m for >2s — resetting to nearest wp.");
                    coverage_planner_reset_to_nearest(pose.x, pose.y);
                    s_wp_count = load_waypoints_from_planner();
                    s_wp_index = 0;
                    s_cross_track_exceed_ms = 0;
                    {
                        char line[SYS_LOG_MAX_LEN];
                        snprintf(line, sizeof(line),
                                 "AUTO xte>0.5m 2s -> reset to nearest (%d wp)", s_wp_count);
                        sys_log_push(line);
                    }
                }
            } else {
                s_cross_track_exceed_ms = 0;
            }

            // Wheel stall: commanded to move, VESC ERPM low, AND GPS/EKF confirms
            // robot not moving → hard obstacle or physical stop.
            if (AUTO_FAULT_RESPONSES_ENABLED && obs_armed && node_follower_is_stalled()) {
                DBG_PRINTLN("[AUTO] Wheel stall detected (eRPM + GPS) — entering OBSTACLE_AVOID.");
                sys_log_push("AUTO: wheel stall -> OBS-AVOID");
                vesc_set_current(VESC_ID_BLADE, 0);
                s_blade_commanded = false;
                s_blade_ramp_erpm = 0.0f;
                s_lastCollisionDir = COLLISION_DIR_UNKNOWN;
                collisionSaveBaselineForced();
                transition_to(STATE_OBSTACLE_AVOID);
                break;
            }

            // Wheel slip: wheels spinning but GPS/EKF says robot barely moving →
            // sinking into soft/wet ground. Trigger BOG_RECOVERY early.
#if !BENCH_TEST_NO_VESC
            if (AUTO_FAULT_RESPONSES_ENABLED && obs_armed && node_follower_is_slipping()) {
                DBG_PRINTLN("[AUTO] Wheel slip detected — entering BOG_RECOVERY.");
                sys_log_push("AUTO: wheel slip -> BOG");
                vesc_set_current(VESC_ID_BLADE, 0);
                s_blade_commanded = false;
                s_blade_ramp_erpm = 0.0f;
                collisionSaveBaselineForced();
                transition_to(STATE_BOG_RECOVERY);
                break;
            }
#endif
        }

        // CH7 / GPIO pause is now handled in the common pre-switch check.
        break;
    }

    // ── STATE_RETRACE ─────────────────────────────────────────────────────────
    // Re-purposed (Feature 2, 2026-06-16) to host the RPM-load blade_recovery
    // maneuver (hybrid back-up + height-step-down, blade stays armed). The legacy
    // retrace.cpp/bog_recovery.cpp remain in-tree as gated dead code. The Tx still
    // shows "RETRACE" (telemetry flag 0x08) during this recovery.
    case STATE_RETRACE: {
        if (g_state_entry) {
            DBG_PRINTF("[BLADE-REC] Entering recovery at (%.3f, %.3f) h=%.0fmm\r\n",
                pose.x, pose.y, s_desired_cut_height_mm);
            blade_recovery_enter(pose, perimeter_get_perimeter(),
                                 s_desired_cut_height_mm);
        }

        // Tilt safety: also applies during recovery states
        {
            float tilt_deg = imu_get_tilt_rad() * (180.0f / M_PI);
            float tilt_lim = mower_config_get().tilt_limit_normal_deg;
            if (tilt_deg > tilt_lim) {
                DBG_PRINTF("[BLADE-REC] Tilt %.1f > %.1f — pausing.\n", tilt_deg, tilt_lim);
                blade_recovery_exit();
                transition_to(STATE_PAUSED, true);
                break;
            }
        }

        showLedsWithGps(COL_RED);

        // Blade stays armed during recovery — feed the monitor the real commanded
        // state/target so rpm-load and BLADE_FAULT are meaningful (the keepalive
        // after the switch keeps issuing SET_RPM while s_blade_commanded is true).
        {
            VescStatus blade = vesc_get_status(VESC_ID_BLADE);
            cutting_monitor_update(speed, blade.current_A, blade.erpm,
                                   s_blade_ramp_erpm, s_blade_commanded,
                                   battery_get_voltage());
        }

        CuttingStatus cs = cutting_monitor_get_status();
        BladeRecoveryResult result = blade_recovery_update(pose, speed, cs);

        switch (result) {
        case BLADE_RECOVERY_IN_PROGRESS:
            break;

        case BLADE_RECOVERY_COMPLETE:
        case BLADE_RECOVERY_GAVE_UP:
            blade_recovery_exit();
            DBG_PRINTLN(result == BLADE_RECOVERY_COMPLETE
                ? "[BLADE-REC] Complete — resuming AUTO_MOWING."
                : "[BLADE-REC] Gave up — resuming AUTO_MOWING (patch logged).");
            // The recovery backed up / re-cut, so the old waypoint index is stale —
            // re-plan from the current position and reload waypoints.
            coverage_planner_reset_to_nearest(pose.x, pose.y);
            s_wp_count = load_waypoints_from_planner();
            s_wp_index = 0;
            s_blade_commanded = true;   // already armed — keep it so
            transition_to(STATE_AUTO_MOWING);
            break;

        case BLADE_RECOVERY_BLADE_FAULT:
            DBG_PRINTLN("[SM] Blade fault in recovery — pausing");
#if !BENCH_TEST_NO_VESC
            blade_recovery_exit();
            transition_to(STATE_PAUSED, true);
#endif
            break;
        }
        break;
    }

    // ── STATE_BOG_RECOVERY ────────────────────────────────────────────────────
    case STATE_BOG_RECOVERY: {
        if (g_state_entry) {
            DBG_PRINTF("[BOG] Entering BOG_RECOVERY at (%.3f, %.3f) h=%.0fmm\r\n",
                pose.x, pose.y, s_desired_cut_height_mm);
            bog_recovery_enter(pose, s_desired_cut_height_mm);
        }

        // Tilt safety: also applies during recovery states
        {
            float tilt_deg = imu_get_tilt_rad() * (180.0f / M_PI);
            float tilt_lim = mower_config_get().tilt_limit_normal_deg;
            if (tilt_deg > tilt_lim) {
                DBG_PRINTF("[BOG] Tilt %.1f > %.1f — pausing.\n", tilt_deg, tilt_lim);
                bog_recovery_exit();
                transition_to(STATE_PAUSED, true);
                break;
            }
        }

        showLeds(COL_RED, LED_FAST_FLASH);

        {
            VescStatus blade = vesc_get_status(VESC_ID_BLADE);
            cutting_monitor_update(speed, blade.current_A, blade.erpm,
                                   (float)BLADE_TARGET_ERPM, false, battery_get_voltage());
        }

        CuttingStatus cs = cutting_monitor_get_status();
        BogResult result = bog_recovery_update(pose, speed, cs);

        switch (result) {
        case BOG_IN_PROGRESS:
            break;

        case BOG_RECOVERED:
        case BOG_GAVE_UP:
            bog_recovery_exit();
            DBG_PRINTLN(result == BOG_RECOVERED
                ? "[BOG] Recovered — resuming AUTO_MOWING."
                : "[BOG] Gave up — resuming AUTO_MOWING (obstacle logged).");
            // Reset plan to nearest waypoint
            coverage_planner_reset_to_nearest(pose.x, pose.y);
            s_wp_count = load_waypoints_from_planner();
            s_wp_index = 0;
            s_blade_commanded = true;
            transition_to(STATE_AUTO_MOWING);
            break;

        case BOG_BLADE_FAULT:
            DBG_PRINTLN("[SM] Blade fault in BOG_RECOVERY — pausing");
#if !BENCH_TEST_NO_VESC
            bog_recovery_exit();
            transition_to(STATE_PAUSED, true);
#endif
            break;
        }
        break;
    }

    // ── STATE_OBSTACLE_AVOID ──────────────────────────────────────────────────
    case STATE_OBSTACLE_AVOID: {
        // ── RC override: any non-AUTO mode switch position exits immediately ──
        // This is the critical safety path — the operator must always be able
        // to halt the mower by flipping CH4 to MANUAL/RETURN or via failsafe.
        {
            bool rc_wants_stop = !rc.failsafe && !ch4_is_auto(rc.ch[CRSF_CH_MODE]);
            bool rc_failsafe   =  rc.failsafe && !ble_server_is_connected();
            if (rc_wants_stop || rc_failsafe) {
                vesc_set_current(VESC_ID_LEFT,  0);
                vesc_set_current(VESC_ID_RIGHT, 0);
                vesc_set_current(VESC_ID_BLADE, 0);
                s_blade_commanded = false;
                DBG_PRINTLN("[OBS] RC override — stopping obstacle avoid.");
                transition_to(STATE_IDLE);
                break;
            }
        }

        if (g_state_entry) {
            DBG_PRINTF("[OBS] Obstacle at (%.3f, %.3f) hdg=%.3f dir=%d\r\n",
                pose.x, pose.y, pose.heading, (int)s_lastCollisionDir);

            // 1. Zero drive motors
            vesc_set_current(VESC_ID_LEFT,  0);
            vesc_set_current(VESC_ID_RIGHT, 0);

            // 2. Report obstacle to planner
            coverage_planner_report_obstacle(pose.x, pose.y, pose.heading);

            // 3. Determine backup distance from IMU collision direction (A34/A37)
            if (s_lastCollisionDir == COLLISION_DIR_REVERSE) {
                s_obstacle_backup_dist_m = 0.0f;   // already reversed into something — don't back up
            } else if (s_lastCollisionDir == COLLISION_DIR_LEFT ||
                       s_lastCollisionDir == COLLISION_DIR_RIGHT) {
                s_obstacle_backup_dist_m = 0.15f;  // side hit — small backup to clear
            } else {
                s_obstacle_backup_dist_m = OBSTACLE_BACKUP_DIST_M;  // default forward/oblique
            }

            // Time-based backup: avoids dependency on EKF position accuracy
            s_obstacle_backup_time_ms = (uint32_t)(s_obstacle_backup_dist_m
                                                   / OBSTACLE_BACKUP_SPEED_MS * 1000.0f);
            s_obstacle_enter_ms    = millis();
            s_obstacle_backup_done = false;
            s_obstacle_reset_done  = false;
        }

        showLedsWithGps(COL_RED);

        if (!s_obstacle_backup_done) {
            // Time-based backup: run reverse for a fixed duration derived from
            // s_obstacle_backup_dist_m / OBSTACLE_BACKUP_SPEED_MS. This avoids
            // the EKF position dependency that caused indefinite reverse when
            // the EKF was frozen.
            if ((millis() - s_obstacle_enter_ms) < s_obstacle_backup_time_ms) {
                float I_back = (-OBSTACLE_BACKUP_SPEED_MS / MAX_MOWING_SPEED_MS) * MAX_CURRENT_A;
                int32_t I_mA = (int32_t)(I_back * 1000.0f);
                vesc_set_current(VESC_ID_LEFT,  I_mA);
                vesc_set_current(VESC_ID_RIGHT, I_mA);
            } else {
                vesc_set_current(VESC_ID_LEFT,  0);
                vesc_set_current(VESC_ID_RIGHT, 0);
                s_obstacle_backup_done = true;
            }
        }

        if (s_obstacle_backup_done && !s_obstacle_reset_done) {
            // 4. Reset planner to nearest unvisited waypoint
            coverage_planner_reset_to_nearest(pose.x, pose.y);
            s_wp_count = load_waypoints_from_planner();
            s_wp_index = 0;
            s_obstacle_reset_done = true;
            DBG_PRINTLN("[OBS] Backup complete, plan reset — resuming AUTO_MOWING.");
            s_blade_commanded = true;
            transition_to(STATE_AUTO_MOWING);
        }
        break;
    }

    // ── STATE_AUTO_RETURN ─────────────────────────────────────────────────────
    case STATE_AUTO_RETURN: {
        if (g_state_entry) {
            // Use the saved home position (GPS ENU at perimeter-recording start).
            // perimeter.pts[0] is unreliable: ensureCCW() may reverse the array,
            // and perimeter_close_track() reorders the start/end points.
            if (s_home_x == 0.0f && s_home_y == 0.0f) {
                // No home saved — fall back to first perimeter point
                Polygon perim = perimeter_get_perimeter();
                if (perim.pts.empty()) {
                    transition_to(STATE_IDLE);
                    break;
                }
                s_return_wp.x = perim.pts[0].x;
                s_return_wp.y = perim.pts[0].y;
                DBG_PRINTLN("[RETURN] No home saved — using perimeter.pts[0].");
            } else {
                s_return_wp.x = s_home_x;
                s_return_wp.y = s_home_y;
                DBG_PRINTF("[RETURN] Returning to home (%.3f, %.3f).\n", s_home_x, s_home_y);
            }
            s_return_wp.heading  = 0.0f;
            s_return_wp.mowing   = false;
            s_return_wp.headland = false;

            // Reset duty ramp so we don't carry stale duty from mowing into RETURN.
            wheel_duty_ramp_reset();
            safety_set_auto_mode(false);
        }

        // ── Mode-switch override ──────────────────────────────────────────
        {
            uint16_t ch4 = rc.ch[CRSF_CH_MODE];
            if (ch4_is_manual(ch4)) {
                vesc_set_current(VESC_ID_LEFT,  0);
                vesc_set_current(VESC_ID_RIGHT, 0);
                DBG_PRINTLN("[RETURN] CH4→MANUAL — exiting to IDLE.");
                transition_to(STATE_IDLE);
                break;
            }
            if (ch4_is_auto(ch4)) {
                vesc_set_current(VESC_ID_LEFT,  0);
                vesc_set_current(VESC_ID_RIGHT, 0);
                DBG_PRINTLN("[RETURN] CH4→AUTO — exiting to IDLE.");
                transition_to(STATE_IDLE);
                break;
            }
        }

        showLedsWithGps(COL_RED);

        // Follow path to return waypoint via the node follower
        {
            WheelCmd cmd = node_follower_compute(pose, speed,
                                                 &s_return_wp, 1, 0,
                                                 (float)mower_config_get().cut_height_max_mm);
            node_follower_to_vesc_duty(cmd);
        }

        // Arrived?
        if (node_follower_waypoint_reached(pose, s_return_wp)) {
            vesc_set_current(VESC_ID_LEFT,  0);
            vesc_set_current(VESC_ID_RIGHT, 0);
            DBG_PRINTLN("[RETURN] Arrived at start point → STATE_IDLE.");
            transition_to(STATE_IDLE);
        }
        break;
    }

    // ── STATE_PAUSED ──────────────────────────────────────────────────────────
    case STATE_PAUSED: {
        if (g_state_entry) {
            // Motors were already zeroed before transitioning here.
            servo_set_height_mm(mower_config_get().cut_height_max_mm);
            if (s_pause_event_latch) {
                DBG_PRINTLN("[PAUSED] Safety event pause — cycle pause switch to acknowledge.");
            } else {
                DBG_PRINTF("[PAUSED] Mowing paused (CH7=%d sw=%d). Deactivate both to resume.\n",
                              (int)ch7_active, (int)pauseSwitchActive());
            }
        }

        showLeds(COL_PURPLE, LED_FAST_FLASH);

        // Pause switch takes priority — stay paused while any source is active
        bool pause_active = ch7_active || pauseSwitchActive() || s_ble_pause;

        // Event-latch acknowledgement: operator must cycle the pause switch
        // (activate then deactivate) to clear the latch set by safety events.
        {
            static bool s_pause_was_active = false;
            if (s_pause_event_latch) {
                if (s_pause_was_active && !pause_active) {
                    // Falling edge — operator acknowledged
                    s_pause_event_latch = false;
                    DBG_PRINTLN("[PAUSED] Event latch cleared — resume now available.");
                }
                s_pause_was_active = pause_active;
                // Stay paused until latch is cleared
                break;
            }
            s_pause_was_active = pause_active;
        }

        if (pause_active) {
            break;
        }

        // Pause released — check mode switch to decide where to go
        uint16_t ch4p = rc.ch[CRSF_CH_MODE];
        if (ch4_is_manual(ch4p)) {
            DBG_PRINTLN("[PAUSED] Pause released, CH4=MANUAL — exiting to IDLE.");
            transition_to(STATE_IDLE);
            break;
        }

        {
            // Tilt safety: don't resume if tilt still exceeds limit
            float tilt_deg = imu_get_tilt_rad() * (180.0f / M_PI);
            float tilt_lim = mower_config_get().tilt_limit_normal_deg;
            if (tilt_deg > tilt_lim) {
                static uint32_t s_tilt_log_ms = 0;
                if (millis() - s_tilt_log_ms > 2000) {
                    DBG_PRINTF("[PAUSED] Tilt %.1f > %.1f — staying paused\n",
                                  tilt_deg, tilt_lim);
                    s_tilt_log_ms = millis();
                }
                break;
            }

            if (ch4_is_auto(ch4p)) {
                // Validate that the perimeter is still valid and mower is inside it
                if (!perimeter_is_valid()) {
                    DBG_PRINTLN("[PAUSED] No valid perimeter — returning to IDLE.");
                    transition_to(STATE_IDLE);
                } else if (!mower_inside_perimeter()) {
                    DBG_PRINTLN("[PAUSED] Mower outside perimeter — returning to IDLE.");
                    transition_to(STATE_IDLE);
                } else {
                    DBG_PRINTLN("[PAUSED] Pause released — resuming AUTO_MOWING.");
                    safety_set_auto_mode(true);
                    transition_to(STATE_AUTO_MOWING);
                }
            } else {
                DBG_PRINTLN("[PAUSED] Pause released, mode not AUTO — returning to IDLE.");
                transition_to(STATE_IDLE);
            }
        }
        break;
    }

    // ── STATE_MOTORS_OFFLINE ──────────────────────────────────────────────────
    case STATE_MOTORS_OFFLINE: {
        if (g_state_entry) {
            // Record whether we came from PAUSED (enables battery-swap resume)
            bool was_paused = (g_prev_state == STATE_PAUSED);
            servo_set_height_mm(mower_config_get().cut_height_max_mm);
            cutting_monitor_force_save_cal();
            collisionSaveBaselineForced();
            safety_set_auto_mode(false);
            s_blade_commanded = false;

            // Save session state to NVS "session" namespace
            Preferences prefs;
            prefs.begin("session", false);
            prefs.putBool("was_paused", was_paused);
            if (was_paused) {
                Pose2D p = ekf_get_pose();
                prefs.putFloat("pause_x",     p.x);
                prefs.putFloat("pause_y",     p.y);
                prefs.putFloat("pause_hdg",   p.heading);
                prefs.putUInt("waypoint_idx", (uint32_t)coverage_planner_get_waypoint_index());
                prefs.putFloat("hprog",       coverage_planner_get_headland_progress());
                prefs.putFloat("sprog",       coverage_planner_get_strip_progress());
                prefs.putFloat("cut_height",  s_desired_cut_height_mm);
                prefs.putUInt("session_ms",   millis() - s_session_start_ms);
            }
            prefs.end();

            DBG_PRINTF("[MOTORS_OFFLINE] VESCs lost. Was paused: %d\n", (int)was_paused);
        }

        showLeds(COL_RED, LED_FAST_FLASH);

        // Poll for VESCs returning online (drive VESCs only — blade may lag)
        if (vesc_all_drive_online()) {
            Preferences prefs;
            prefs.begin("session", true);  // read-only
            bool was_paused_nvs = prefs.getBool("was_paused", false);
            prefs.end();

            if (was_paused_nvs) {
                // Regenerate plan and skip to saved waypoint
                bool plan_ok = coverage_planner_plan(
                    perimeter_get_perimeter(),
                    perimeter_get_nav_boundary(),
                    perimeter_get_working_area());

                if (plan_ok) {
                    prefs.begin("session", true);
                    int   idx  = (int)prefs.getUInt("waypoint_idx", 0);
                    float cutH = prefs.getFloat("cut_height", (float)mower_config_get().cut_height_min_mm);
                    prefs.end();
                    coverage_planner_skip_to_waypoint(idx);
                    s_desired_cut_height_mm = cutH;
                    DBG_PRINTF("[MOTORS_OFFLINE] VESCs restored — resuming from waypoint %d\n", idx);
                } else {
                    DBG_PRINTLN("[MOTORS_OFFLINE] Re-plan failed — falling back to IDLE.");
                    was_paused_nvs = false;
                }

                // Clear flag so a subsequent power cycle doesn't spuriously resume
                prefs.begin("session", false);
                prefs.putBool("was_paused", false);
                prefs.end();
            }

            safety_clear_motors_offline();

            if (was_paused_nvs) {
                DBG_PRINTLN("[MOTORS_OFFLINE] VESCs restored — returning to PAUSED.");
                transition_to(STATE_PAUSED);
            } else {
                DBG_PRINTLN("[MOTORS_OFFLINE] VESCs restored — re-arm required.");
                transition_to(STATE_IDLE);
            }
        }
        break;
    }

    } // end switch(g_state)

    // ── Blade VESC keepalive (10 Hz) ─────────────────────────────────────────
    // Blade runs in RPM control (SET_RPM every 100 ms): the VESC's internal
    // current-limited RPM PID gives a smooth ~2 s spin-up. (A firmware-side
    // duty ramp was tried 2026-06-10/12 and reverted — duty steps were jerky
    // and interacted badly with CAN dropouts.) On disarm, command zero current
    // every tick — zero torque, so the blade freewheels down (no regen).
    {
        static uint32_t s_blade_ka_ms = 0;
        if (millis() - s_blade_ka_ms >= 100) {
            s_blade_ka_ms = millis();
            const MowerConfig &mc = mower_config_get();
            // Guard against zero config values (NVS corruption or bad BLE
            // SET_CONFIG) which would produce vesc_set_rpm(blade, 0) — an
            // active stop command.
            uint32_t target_erpm = (mc.blade_target_rpm > 0 && mc.blade_motor_pole_pairs > 0)
                                   ? (uint32_t)mc.blade_target_rpm * (uint32_t)mc.blade_motor_pole_pairs
                                   : (uint32_t)(BLADE_TARGET_RPM * BLADE_MOTOR_POLE_PAIRS);
            s_blade_ramp_erpm = s_blade_commanded ? (float)target_erpm : 0.0f;
            if (s_blade_commanded) {
                vesc_set_rpm(VESC_ID_BLADE, (int32_t)s_blade_ramp_erpm);
            } else {
                vesc_set_current(VESC_ID_BLADE, 0.0f);
            }
        }
    }

    // ── Drive VESC keepalive (2 Hz, idle states only) ─────────────────────────
    // VESCs only send CAN status frames in response to commands. In states where
    // no drive commands are issued, they go quiet and the stale timer trips the
    // motors-offline safety watchdog. Send zero-current every 250 ms to keep the
    // drives reporting. Active states (MANUAL, LEARN_PERIMETER, AUTO_*) send their
    // own commands every 100 ms so this block is skipped there.
    {
        bool drives_idle = (g_state == STATE_INIT   ||
                            g_state == STATE_IDLE   ||
                            g_state == STATE_PAUSED ||
                            g_state == STATE_MOTORS_OFFLINE);
        if (drives_idle) {
            static uint32_t s_drive_ka_ms = 0;
            if (millis() - s_drive_ka_ms >= 250) {
                s_drive_ka_ms = millis();
                vesc_set_current(VESC_ID_LEFT,  0.0f);
                vesc_set_current(VESC_ID_RIGHT, 0.0f);
            }
        }
    }

    // ── Apply battery warning overlay (after state LED) ───────────────────────
    applyBatteryWarningOverlay();

    // ── VESC error override (highest priority LED) ────────────────────────────
    // If ANY VESC that was previously online has gone silent, show red fast flash
    // regardless of state.  Covers drive VESCs and blade VESC.
    // Startup grace built into vesc_any_went_offline() — safe from any state.
    if (vesc_any_went_offline()) {
        showLeds(COL_RED, LED_FAST_FLASH);
    }

    // ── VESC raw RX dump → PWA log — DISABLED (declutters the System Log) ────
    // This 5 s heartbeat of raw TWAI RX state (status age / eRPM / current /
    // battery / bus-ok) was useful while chasing the v4-VESC "is it broadcasting
    // CAN status at all?" question, but it churns the log ring and buries the
    // PLAN / AUTO-nav diagnostics. Re-enable (#if 1) if VESC RX needs auditing.
    #if 0
    {
        static uint32_t s_vesc_dump_ms = 0;
        if (ble_server_is_connected() && millis() - s_vesc_dump_ms >= 5000) {
            s_vesc_dump_ms = millis();
            uint32_t now_d = millis();
            VescStatus vl = vesc_get_status(VESC_ID_LEFT);
            VescStatus vr = vesc_get_status(VESC_ID_RIGHT);
            VescStatus vb = vesc_get_status(VESC_ID_BLADE);
            char line[SYS_LOG_MAX_LEN];
            snprintf(line, sizeof(line),
                "CAN L[a%ld e%.0f] R[a%ld e%.0f] B[a%ld e%.0f i%.1f] V%.1f ok%d",
                vl.last_update_ms ? (long)(now_d - vl.last_update_ms) : -1L, vl.erpm,
                vr.last_update_ms ? (long)(now_d - vr.last_update_ms) : -1L, vr.erpm,
                vb.last_update_ms ? (long)(now_d - vb.last_update_ms) : -1L, vb.erpm,
                vb.current_A,
                vesc_get_battery_voltage(), vesc_can_bus_ok() ? 1 : 0);
            sys_log_push(line);
        }
    }
    #endif

    // ── CRSF telemetry snapshot (2 Hz) ──────────────────────────────────────
    static uint32_t s_crsf_telem_last_ms = 0;
    if (millis() - s_crsf_telem_last_ms >= 500) {
        s_crsf_telem_last_ms = millis();
        static const char *s_mode_names[] = {
            "INIT","IDLE","MANUAL","LEARN","AUTO",
            "RETRACE","BOG","OBS-AVOID","RETURN","PAUSED","MOT-OFF"
        };

        TelemetryData td;
        memset(&td, 0, sizeof(td));

        // GPS
        td.lat_deg         = gps.lat_deg;
        td.lon_deg         = gps.lon_deg;
        td.groundspeed_ms  = speed;
        td.heading_deg     = pose.heading * (180.0f / (float)M_PI);
        td.altitude_m      = 0.0f;
        td.satellites       = gps.sat_count;

        // Battery
        float batt = battery_get_voltage();
        td.battery_voltage_V     = batt;
        td.blade_current_A       = vesc_get_status(VESC_ID_BLADE).current_A;
        td.capacity_mah_used     = 0;
        // 13S LiPo: full ≈ 54.6V (4.2V/cell), low = BATTERY_LOW_V
        const float BATT_FULL = 54.6f;
        float pct = (batt - BATTERY_LOW_V) / (BATT_FULL - BATTERY_LOW_V) * 100.0f;
        td.battery_remaining_pct = (pct < 0) ? 0 : (pct > 100) ? 100 : (uint8_t)pct;

        // Attitude
        td.yaw_rad = pose.heading;

        // Flight mode
        int si = (int)g_state;
        const char *mode = (si >= 0 && si < 11) ? s_mode_names[si] : "?";
        strncpy(td.flight_mode, mode, sizeof(td.flight_mode) - 1);

        // Mower status
        td.state          = (uint8_t)g_state;
        {   static uint8_t s_prev_crsf_state = 0xFF;
            if (td.state != s_prev_crsf_state) {
                DBG_PRINTF("[CRSF] state byte: %d (%s)\n", td.state, mode);
                s_prev_crsf_state = td.state;
            }
        }
        td.hprog          = (uint8_t)(coverage_planner_get_headland_progress() * 100.0f);
        td.sprog          = (uint8_t)(coverage_planner_get_strip_progress()    * 100.0f);
        td.cut_height_mm  = (uint8_t)state_machine_get_cut_height_mm();
        td.blade_load_pct = (uint8_t)(cutting_monitor_get_rpm_load_fraction() * 100.0f);  // RPM-based (Feature 2)
        td.fix_type       = (uint8_t)gps.fix_type;
        td.calib_status   = imu_get_calib_status();
        td.flags          = (s_last_armed && g_state != STATE_PAUSED
                             && g_state != STATE_MOTORS_OFFLINE ? 0x01 : 0)
                          // Bit 0x02 = blade actually commanded — NOT just
                          // "armed while in a running state". Previously this
                          // displayed "Running" while the battery lockout or a
                          // state gate silently kept the blade off.
                          | (s_blade_commanded ? 0x02 : 0)
                          | (g_state == STATE_BOG_RECOVERY ? 0x04 : 0)
                          | (g_state == STATE_RETRACE      ? 0x08 : 0)
                          | (gps.fix_type == GPS_FIX_RTK_FLOAT ? 0x10 : 0)
                          // Bit 0x20: battery WARNING or LOW — drives the
                          // flashing battery banner on the TX16S Lua widget
                          | (battery_get_state() != BATTERY_OK ? 0x20 : 0);
        td.obs_count      = (uint16_t)obstacle_get_count();
        { float unc_cm = ekf_get_position_uncertainty() * 100.0f;
          td.ekf_unc_cm = (unc_cm > 65535.0f) ? 65535u : (uint16_t)unc_cm; }
        td.session_mowed_dm2 = 0;  // TODO: integrate from coverage_planner

        // ── Boot self-test sweep ─────────────────────────────────────────────
        // For the first 8 s of telemetry, override battery / blade load /
        // heading with synthetic ramps (40→55 V, 0→100 %, 0→360°). If the
        // TX16S display sweeps and then freezes, the radio/Lua/sensor path is
        // proven good and the freeze is on the CAN/decoding side. If it never
        // sweeps, the EdgeTX sensors need re-discovery.
        {
            static constexpr uint32_t k_sweep_ms = 8000;
            static uint32_t s_sweep_start_ms = 0;
            if (s_sweep_start_ms == 0) {
                s_sweep_start_ms = millis();
                sys_log_push("TELEM self-test sweep 8s: batt 40-55V load 0-100% hdg 0-360");
            }
            uint32_t sweep_el = millis() - s_sweep_start_ms;
            if (sweep_el < k_sweep_ms) {
                float ph = (float)sweep_el / (float)k_sweep_ms;   // 0 → 1
                td.battery_voltage_V     = 40.0f + 15.0f * ph;
                td.battery_remaining_pct = (uint8_t)(ph * 100.0f);
                td.blade_load_pct        = (uint8_t)(ph * 100.0f);
                td.heading_deg           = ph * 360.0f;
                td.yaw_rad               = (ph * 2.0f - 1.0f) * (float)M_PI;
            }
        }

        crsf_telemetry_update(td);

        // ── Collision baseline capture (detection DISABLED) ───────────────────
        // Detection is gated off (AUTO_FAULT_RESPONSES_ENABLED 0). Log the
        // adaptive baseline + live jolt RMS every 5 s while driving so a real
        // normal-driving/mowing baseline can be observed before re-enabling it.
        {
            static uint32_t s_coll_log_ms = 0;
            bool driving = (g_state == STATE_MANUAL || g_state == STATE_AUTO_MOWING);
            if (driving && (millis() - s_coll_log_ms) > 5000) {
                s_coll_log_ms = millis();
                char cline[SYS_LOG_MAX_LEN];
                snprintf(cline, sizeof(cline), "COLL base=%.3fg jolt=%.3fg",
                         (double)collisionGetBaseline(), (double)collisionGetJoltRms());
                sys_log_push(cline);
            }
        }

        // ── Heading diagnostic (debugging the AUTO heading-flip) ───────────────
        // bno = raw BNO055 fused heading; th = EKF heading (= bno + GPS offset);
        // off = the GPS-trimmed offset (deg). If `th` flips while `bno` stays
        // steady, the GPS offset-trim drifted; if `bno` itself flips, it's the
        // magnetometer (motor/blade interference). Temporary — remove once solved.
        {
            static uint32_t s_hdg_log_ms = 0;
            bool driving = (g_state == STATE_MANUAL || g_state == STATE_AUTO_MOWING);
            if (driving && (millis() - s_hdg_log_ms) > 3000) {
                s_hdg_log_ms = millis();
                int bno_i = ((int)roundf(imu_get_heading_fused() * 57.29578f) % 360 + 360) % 360;
                int th_i  = ((int)roundf(pose.heading           * 57.29578f) % 360 + 360) % 360;
                int off_i = (int)roundf(ekf_get_heading_offset() * 57.29578f);
                char hline[SYS_LOG_MAX_LEN];
                snprintf(hline, sizeof(hline), "HDG bno=%d th=%d off=%d", bno_i, th_i, off_i);
                sys_log_push(hline);
            }
        }
    }

    // ── Update previous-channel states ───────────────────────────────────────
    s_ch5_prev = ch5_active;

    // ── Apply pending state transition ────────────────────────────────────────
    if (g_transition_pending) {
        g_prev_state = g_state;
        g_transition_pending = false;
        g_state = g_next_state;
        g_state_entry = true;
        DBG_PRINTF("[SM] → State: %d\r\n", (int)g_state);
    } else {
        g_state_entry = false;
    }
}
