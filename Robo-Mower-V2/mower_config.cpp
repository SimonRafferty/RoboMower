// ══════════════════════════════════════════════════════════════════════════════
//  mower_config.cpp — Runtime-configurable physical dimensions
//
//  See mower_config.h for full description.
// ══════════════════════════════════════════════════════════════════════════════

#include "mower_config.h"
#include "config.h"
#include <nvs.h>
#include <Arduino.h>

// NVS storage: same "mower" namespace as nvs_storage.cpp to avoid wasting slots.
// Key must be ≤ 15 characters.
static const char *k_nvs_ns  = "mower";
static const char *k_nvs_key = "mow_cfg_v12"; // bumped: + min_move_duty (kickstart). v11 added footprint W×L box + track_width_m

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
static SemaphoreHandle_t s_cfg_mutex = nullptr;

// Compile-time defaults from config.h — used when NVS has no saved config.
static const MowerConfig k_defaults = {
    /* footprint_width_m      */ FOOTPRINT_WIDTH_M,
    /* footprint_length_m     */ FOOTPRINT_LENGTH_M,
    /* track_width_m          */ TRACK_WIDTH_M,
    /* wheel_radius_m         */ WHEEL_RADIUS_M,
    /* motor_pole_pairs       */ (uint8_t)MOTOR_POLE_PAIRS,
    /* gear_ratio             */ GEAR_RATIO,
    /* antenna_fwd_m          */ ANTENNA_OFFSET_FORWARD_M,
    /* antenna_right_m        */ ANTENNA_OFFSET_RIGHT_M,
    /* steer_to_cut_m         */ STEER_CENTRE_TO_CUT_CENTRE_M,
    /* cut_disc_radius_m      */ CUT_DISC_RADIUS_M,
    /* cut_width_m            */ CUT_WIDTH_M,
    /* strip_overlap_m        */ STRIP_OVERLAP_M,
    /* cut_height_min_mm      */ (uint16_t)CUT_HEIGHT_MIN_MM,
    /* cut_height_max_mm      */ (uint16_t)CUT_HEIGHT_MAX_MM,
    /* blade_motor_pole_pairs */ (uint8_t)BLADE_MOTOR_POLE_PAIRS,
    /* blade_target_rpm       */ (uint16_t)BLADE_TARGET_RPM,
    /* pp_lookahead_base_m    */ PURE_PURSUIT_LOOKAHEAD_BASE_M,
    /* pp_lookahead_k         */ PURE_PURSUIT_LOOKAHEAD_K,
    /* max_mowing_speed_ms    */ MAX_MOWING_SPEED_MS,
    /* headland_speed_ms      */ HEADLAND_SPEED_MS,
    /* transit_speed_ms       */ TRANSIT_SPEED_MS,
    /* min_creep_speed_ms     */ MIN_CREEP_SPEED_MS,
    /* waypoint_arrive_dist_m */ WAYPOINT_ARRIVE_DIST_M,
    /* max_wheel_speed_ms     */ MAX_WHEEL_SPEED_MS,
    /* max_current_a          */ MAX_CURRENT_A,
    /* current_ramp_a_per_s   */ CURRENT_RAMP_A_PER_S,
    /* uncertainty_margin_m   */ UNCERTAINTY_MARGIN_M,
    /* tilt_limit_normal_deg  */ TILT_LIMIT_NORMAL_DEG,
    /* tilt_limit_careful_deg */ TILT_LIMIT_CAREFUL_DEG,
    /* collision_mult_careful */ COLLISION_MULT_CAREFUL,
    /* heading_kp             */ HEADING_KP,
    /* heading_kd             */ HEADING_KD,
    /* manual_max_yaw_rate    */ MANUAL_MAX_YAW_RATE,
    /* wheel_pi_kp            */ WHEEL_PI_KP,
    /* wheel_pi_ki            */ WHEEL_PI_KI,
    /* manual_max_duty        */ MANUAL_MAX_DUTY,
    /* manual_max_speed_ms    */ MANUAL_MAX_SPEED_MS,
    /* min_turn_radius_m      */ MIN_TURNING_RADIUS_M,
    /* min_move_duty          */ MIN_MOVE_DUTY,
};

static MowerConfig s_cfg;

// ─────────────────────────────────────────────────────────────────────────────

void mower_config_init() {
    s_cfg_mutex = xSemaphoreCreateMutex();
    configASSERT(s_cfg_mutex != nullptr);

    s_cfg = k_defaults;

    nvs_handle_t h = 0;
    esp_err_t err = nvs_open(k_nvs_ns, NVS_READONLY, &h);
    if (err == ESP_OK) {
        size_t sz = sizeof(MowerConfig);
        err = nvs_get_blob(h, k_nvs_key, &s_cfg, &sz);
        nvs_close(h);
        if (err != ESP_OK || sz != sizeof(MowerConfig)) {
            DBG_PRINTLN("[CFG] No saved config or size mismatch — using defaults");
            s_cfg = k_defaults;
        } else {
            DBG_PRINTLN("[CFG] Mower config loaded from NVS");
        }
    } else {
        DBG_PRINTLN("[CFG] NVS not available — using compiled defaults");
    }

    // Validate critical blade parameters — a prior BLE SET_CONFIG with a blank
    // field stores 0, which causes vesc_set_rpm(blade, 0) on every keepalive tick,
    // permanently stopping the blade motor.  Restore defaults if invalid.
    if (s_cfg.blade_target_rpm == 0) {
        DBG_PRINTLN("[CFG] blade_target_rpm is 0 — restoring default");
        s_cfg.blade_target_rpm = k_defaults.blade_target_rpm;
    }
    if (s_cfg.blade_motor_pole_pairs == 0) {
        DBG_PRINTLN("[CFG] blade_motor_pole_pairs is 0 — restoring default");
        s_cfg.blade_motor_pole_pairs = k_defaults.blade_motor_pole_pairs;
    }
    // Similarly guard drive parameters that would break odometry or speed control
    if (s_cfg.wheel_radius_m <= 0.0f) s_cfg.wheel_radius_m = k_defaults.wheel_radius_m;
    if (s_cfg.gear_ratio       <= 0.0f) s_cfg.gear_ratio      = k_defaults.gear_ratio;
    if (s_cfg.motor_pole_pairs == 0)   s_cfg.motor_pole_pairs  = k_defaults.motor_pole_pairs;

    DBG_PRINTF("[CFG] Blade: %u RPM × %u pole-pairs = %u ERPM\n",
               (unsigned)s_cfg.blade_target_rpm,
               (unsigned)s_cfg.blade_motor_pole_pairs,
               (unsigned)s_cfg.blade_target_rpm * (unsigned)s_cfg.blade_motor_pole_pairs);
}

void mower_config_set(const MowerConfig &cfg) {
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    s_cfg = cfg;
    xSemaphoreGive(s_cfg_mutex);

    nvs_handle_t h = 0;
    esp_err_t err = nvs_open(k_nvs_ns, NVS_READWRITE, &h);
    if (err == ESP_OK) {
        err = nvs_set_blob(h, k_nvs_key, &s_cfg, sizeof(MowerConfig));
        if (err == ESP_OK) {
            nvs_commit(h);
            DBG_PRINTLN("[CFG] Mower config saved to NVS");
        } else {
            DBG_PRINTF("[CFG] NVS write failed: %s\n", esp_err_to_name(err));
        }
        nvs_close(h);
    } else {
        DBG_PRINTF("[CFG] NVS open (rw) failed: %s\n", esp_err_to_name(err));
    }
}

MowerConfig mower_config_get() {
    xSemaphoreTake(s_cfg_mutex, portMAX_DELAY);
    MowerConfig copy = s_cfg;
    xSemaphoreGive(s_cfg_mutex);
    return copy;
}

// ── Derived geometry ──────────────────────────────────────────────────────────
// Each helper takes a snapshot under mutex via mower_config_get().

float mower_config_track_width_m() {
    return mower_config_get().track_width_m;
}

float mower_config_nav_inset_m() {
    MowerConfig mc = mower_config_get();
    // Half the footprint diagonal: the radius the footprint corners sweep when the
    // robot pivots on the spot about the steering centre. Keeping the steering
    // centre this far inside the perimeter guarantees no corner crosses it during a
    // corner pivot. (Using half the WIDTH let a corner swing past — the bug this
    // fixes.) Assumes the steering centre sits ~centrally in the footprint box.
    float half_diag = 0.5f * sqrtf(mc.footprint_width_m  * mc.footprint_width_m +
                                   mc.footprint_length_m * mc.footprint_length_m);
    return half_diag + 0.05f;   // + GPS margin
}

float mower_config_blade_fwd_reach_m() {
    MowerConfig mc = mower_config_get();
    return fmaxf(0.0f, mc.steer_to_cut_m + mc.cut_disc_radius_m);
}

float mower_config_headland_m() {
    MowerConfig mc = mower_config_get();
    float blade_fwd = fmaxf(0.0f, mc.steer_to_cut_m + mc.cut_disc_radius_m);
    // Vestigial under the spiral planner (working area is still derived/displayed).
    // Rear-extent term uses half the footprint length now that robot_rear_m is gone.
    return fmaxf(fmaxf(mc.cut_width_m * 1.5f,
                       mc.footprint_length_m * 0.5f + mc.strip_overlap_m),
                 blade_fwd + mc.strip_overlap_m);
}

float mower_config_strip_step_m() {
    MowerConfig mc = mower_config_get();
    float step = mc.cut_width_m - mc.strip_overlap_m;
    return (step > 0.01f) ? step : 0.01f;  // guard against bad config (A8)
}

float mower_config_min_turn_radius_m() {
    return s_cfg.min_turn_radius_m;
}

float mower_config_spur_min_turn_width_m() {
    return 2.0f * s_cfg.min_turn_radius_m;
}
