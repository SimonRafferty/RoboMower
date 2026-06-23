#pragma once
#include <Arduino.h>

// ══════════════════════════════════════════════════════════════════════════════
//  mower_config.h — Runtime-configurable physical dimensions
//
//  All physical constants that affect mowing geometry, navigation, and
//  odometry are held in a single MowerConfig struct.  The struct is persisted
//  to NVS and can be updated at runtime via the BLE SET_CONFIG command,
//  removing the need to recompile firmware when the chassis is modified.
//
//  Default values are taken from the compile-time #defines in config.h.
//  On first boot (or after NVS erase) the defaults are loaded and written
//  to NVS so subsequent boots use the last-configured values.
//
//  Thread safety:
//    mower_config_get() returns a const reference to a module-private struct.
//    It is safe to call from any task.
//    mower_config_set() MUST only be called from the state machine task
//    (Core 1) via the BLE command queue — never directly from a BLE callback.
// ══════════════════════════════════════════════════════════════════════════════

struct MowerConfig {
    // ── Robot footprint — OVERALL bounding box (outer extents) ────────────────
    // Boundary-clearance only: nav inset = half the diagonal of this box.
    float    footprint_width_m;        // overall outer width [m]
    float    footprint_length_m;       // overall outer length [m]

    // ── Steering track (drivetrain) ───────────────────────────────────────────
    float    track_width_m;            // track centre-to-centre [m] (steering odometry)

    // ── Drive system ──────────────────────────────────────────────────────────
    float    wheel_radius_m;           // driven wheel radius [m]
    uint8_t  motor_pole_pairs;         // drive motor rotor pole PAIRS
    float    gear_ratio;               // gearbox reduction (motor → wheel shaft)

    // ── RTK GPS antenna offset from steering centre ───────────────────────────
    float    antenna_fwd_m;            // offset along heading axis (+ = ahead)
    float    antenna_right_m;          // offset perpendicular (+ = right)

    // ── Blade / cutting geometry ──────────────────────────────────────────────
    float    steer_to_cut_m;           // steering centre → cut disc centre (+ = ahead)
    float    cut_disc_radius_m;        // cutting disc radius [m]
    float    cut_width_m;              // effective cut width per pass [m]
    float    strip_overlap_m;          // overlap between adjacent strips [m]

    // ── Cut height limits ─────────────────────────────────────────────────────
    uint16_t cut_height_min_mm;        // minimum deck height [mm]
    uint16_t cut_height_max_mm;        // maximum deck height [mm]

    // ── Blade motor ───────────────────────────────────────────────────────────
    uint8_t  blade_motor_pole_pairs;   // blade motor rotor pole PAIRS
    uint16_t blade_target_rpm;         // blade target mechanical RPM

    // ── Path following (pure pursuit) ────────────────────────────────────────
    float    pp_lookahead_base_m;      // base lookahead distance at zero speed [m]
    float    pp_lookahead_k;           // lookahead gain: total = base + K * speed [s]
    float    max_mowing_speed_ms;      // normal mowing speed [m/s]
    float    headland_speed_ms;        // headland pass speed [m/s]
    float    transit_speed_ms;         // transition/non-mowing speed [m/s]
    float    min_creep_speed_ms;       // minimum commanded speed [m/s]
    float    waypoint_arrive_dist_m;   // distance to consider waypoint reached [m]
    float    max_wheel_speed_ms;       // maximum wheel surface speed [m/s]
    float    max_current_a;            // drive motor continuous current limit [A]
    float    current_ramp_a_per_s;     // current slew rate limit [A/s]

    // ── Uncertainty-aware navigation ─────────────────────────────────────────
    float    uncertainty_margin_m;        // margin below which caution begins [m]
    float    tilt_limit_normal_deg;       // max tilt angle before pausing (normal) [deg]
    float    tilt_limit_careful_deg;      // max tilt angle before pausing (near edge) [deg]
    float    collision_mult_careful;      // collision threshold multiplier when near edge

    // ── Heading stabilisation ────────────────────────────────────────────────
    float    heading_kp;                  // P gain for heading hold [(m/s)/rad]
    float    heading_kd;                  // D gain for yaw rate damping [(m/s)/(rad/s)]
    float    manual_max_yaw_rate;         // max yaw rate at full stick [rad/s]

    // ── ESP32-side wheel speed PI ─────────────────────────────────────────────
    float    wheel_pi_kp;                 // proportional gain [A/(m/s)]
    float    wheel_pi_ki;                 // integral gain [A/(m/s·s)]

    // ── Manual drive ──────────────────────────────────────────────────────────
    float    manual_max_duty;             // retained for NVS compatibility [0–1]
    float    manual_max_speed_ms;         // max wheel speed at full throttle in manual mode [m/s]

    // ── Turning geometry ─────────────────────────────────────────────────────
    float    min_turn_radius_m;           // minimum path-following turn radius [m]; 0 = tracked (pivot)

    // ── AUTO drive ────────────────────────────────────────────────────────────
    // Appended at the end (keeps the positional NVS layout of all fields above
    // intact). NVS key bumped to mow_cfg_v12 for this addition.
    float    min_move_duty;               // static-friction kickstart: min |duty| when a wheel move is commanded [0–1]; 0 = off

    // ── Coverage geometry ─────────────────────────────────────────────────────
    // Appended (NVS key bumped to mow_cfg_v14). Inset of the spiral's outer ring
    // (mow path) from the perimeter so the body clears corners while pivoting.
    float    turn_margin_m;               // ring-0 inset from perimeter [m]; 0 = ring 0 = perimeter

    // ── AUTO exception-handling layer enables (PWA-switchable) ───────────────
    // Appended (NVS key bumped to mow_cfg_v15). Replaces compile-time #defines.
    bool     resp_collision_en;           // collision → back-up + detour (Plan 3)
    bool     resp_stall_en;               // stall → PAUSE (Plan 4 — node-follower stall)
    bool     resp_tilt_en;                // tilt → reverse-to-clear (Plan 4 opt-in; default off — immediate PAUSE is safer)
    bool     resp_slip_en;                // slip → reverse-retry (Plan 5; default off — noisy under Float)
    bool     resp_blade_slow_en;          // blade-load → drive slowdown (Plan 6a; on by default)
    bool     resp_blade_reverse_en;       // blade-load → reverse-retry (Plan 6b; opt-in)

    // ── Cut-height actuator PPM timeout ──────────────────────────────────────
    // Appended (NVS key bumped to mow_cfg_v16). After this many seconds with no
    // commanded height change, the PPM is stopped so the self-holding actuator
    // de-powers and stops drawing current fighting stiffness. 0 = continuous.
    float    cut_height_timeout_s;         // stop cut-height PPM after this many s idle (0 = continuous)
};


// ── Lifecycle ─────────────────────────────────────────────────────────────────

/**
 * @brief Load config from NVS; apply compile-time defaults if absent.
 *
 * Must be called after nvs_storage_init() and before any module that uses
 * mower_config_get() (i.e. step 1.5 in setup(), after NVS init).
 */
void mower_config_init();

/**
 * @brief Apply a new config: save to NVS and update the in-memory struct.
 *
 * Changes take effect immediately for all subsequent calls to
 * mower_config_get() and the derived geometry helpers.
 * Callers (state_machine.cpp) must re-trigger perimeter recompute and
 * coverage re-plan after calling this.
 *
 * Thread safety: call only from the state machine task.
 */
void mower_config_set(const MowerConfig &cfg);


// ── Accessors ─────────────────────────────────────────────────────────────────

/** Returns a copy of the current configuration (thread-safe, mutex-protected). */
MowerConfig mower_config_get();


// ── Derived geometry (computed from current config) ───────────────────────────

/** Steering track width = track_width_m (track centre-to-centre). */
float mower_config_track_width_m();

/** Navigation exclusion inset: steering centre must stay this far inside the
 *  perimeter so no footprint corner sweeps past it during an on-the-spot pivot.
 *  = 0.5·√(footprint_width² + footprint_length²) + 0.05 m GPS margin. */
float mower_config_nav_inset_m();

/** Headland width: inset from nav boundary to working area. */
float mower_config_headland_m();

/** Strip advance per pass = cut_width_m − strip_overlap_m. */
float mower_config_strip_step_m();

/** Turn margin: inset of the spiral's outer ring (mow path) from the perimeter [m]. */
float mower_config_turn_margin_m();

/** Forward reach of blade beyond steering centre = max(0, steer_to_cut + disc_radius). */
float mower_config_blade_fwd_reach_m();

/** Minimum turning radius (m). 0 = tracked/skid-steer (pivot on spot). */
float mower_config_min_turn_radius_m();

/** Minimum spur width for a U-turn = 2 × min_turn_radius. */
float mower_config_spur_min_turn_width_m();
