#pragma once
#include <Arduino.h>
#include <cmath>

// ══════════════════════════════════════════════════════════════════════════════
//  config.h — RoboMower ESP32-S3 Firmware
//  All pin definitions, physical constants, timing, EKF tuning, derived values.
//  NO implementation logic in this file — only #define constants.
//
//  Source references:
//    Spec: Robo_Mower_claudecode_prompt_v3.md
//    Assumptions: ASSUMPTIONS.md (A02, A03, A05, A23)
//    Pins: ARCHITECTURE.md Section 3
//    Decisions: HANDOFFS/01_architecture/HANDOFF.md
// ══════════════════════════════════════════════════════════════════════════════


// ══════════════════════════════════════════════════════════════════════════════
//  BUILD FLAGS
// ══════════════════════════════════════════════════════════════════════════════

// TEST_MODE: when 1, run geometry_test suite from setup() and halt.
//            Compile as normal (0) for production firmware.
#define TEST_MODE                    0

// DEBUG_SERIAL: when 1, enable USB Serial debug output (bench testing).
//               when 0, all Serial output compiles to no-ops (production / field).
//               Does NOT affect Serial2 (CRSF) or BLE telemetry.
// KEEP THIS 0. There is NO USB serial access while the mower is operating, so
// serial debug cannot validate field behaviour. Route all runtime diagnostics to
// the PWA log via sys_log_push() (surfaced in the STATUS JSON `log` array and the
// PWA Diagnostics tab) instead of Serial.
#define DEBUG_SERIAL                 0

// BENCH_TEST_NO_VESC: set to 1 ONLY when testing on bench without VESCs connected.
// Suppresses cutting-monitor state transitions (blade fault / overload / stall).
// Never set this to 1 in field firmware.
#define BENCH_TEST_NO_VESC           0

#if DEBUG_SERIAL
  #define DBG_BEGIN(baud)   do { Serial.begin(baud); } while(0)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_AVAILABLE()   Serial.available()
  #define DBG_READ()        Serial.read()
#else
  #define DBG_BEGIN(baud)   ((void)0)
  #define DBG_PRINTF(...)   ((void)0)
  #define DBG_PRINTLN(...)  ((void)0)
  #define DBG_PRINT(...)    ((void)0)
  #define DBG_AVAILABLE()   (0)
  #define DBG_READ()        (-1)
#endif

// HEADLAND_FIRST: always 1 — headland passes are mowed before working-area strips.
//   The HEADLAND_FIRST=0 code path is not implemented (different vehicle topology).
#define HEADLAND_FIRST               1


// ══════════════════════════════════════════════════════════════════════════════
//  GPIO PIN ASSIGNMENTS  (from ARCHITECTURE.md Section 3)
//  ESP32-S3 DevKitC-1 N16R8
//  Avoided: GPIO0/3/45/46 (strapping), GPIO19/20 (USB), GPIO35/36/37 (PSRAM)
// ══════════════════════════════════════════════════════════════════════════════

// ── CAN / TWAI (VESC motor controllers) ──────────────────────────────────────
#define CAN_TX_PIN                   2   // TWAI TX → CAN transceiver TXD pin
#define CAN_RX_PIN                   1   // TWAI RX ← CAN transceiver RXD pin
// Note: GPIO1 and GPIO2 are swapped vs. PCB silkscreen — the board routes
// GPIO1 to the module's RXD and GPIO2 to the module's TXD.

// ── Servo (cut height) ───────────────────────────────────────────────────────
#define SERVO_PIN                    5   // LEDC channel 0, 50Hz PWM, 1000–2000μs

// ── Physical pause switch ────────────────────────────────────────────────────
#define PAUSE_PIN                    6   // INPUT_PULLUP, active LOW, latching switch to GND
                                         // Closed (GND) = pause AUTO_MOWING
                                         // Open  (HIGH) = resume AUTO_MOWING
#define PAUSE_DEBOUNCE_MS           50   // settle time (ms) before level change is acted on

// ── External LED strip ───────────────────────────────────────────────────────
// NOTE: changed from spec default (GPIO10) to GPIO7 — GPIO10 is used for GPS Serial1 RX.
//       See HANDOFFS/01_architecture/HANDOFF.md for rationale.
#define LED_EXTERNAL_PIN             7   // WS2812/SK6812 data line (FastLED)

// ── IMU I2C (BNO055) ─────────────────────────────────────────────────────────
#define IMU_SDA_PIN                  8   // Wire I2C SDA — ESP32-S3 Arduino default
#define IMU_SCL_PIN                  9   // Wire I2C SCL — ESP32-S3 Arduino default, 400kHz

// ── GPS UART (DFRobot RTK LoRa) — bidirectional (request/response protocol) ──
#define GPS_RX_PIN                  10   // Serial1 RX ← GPS module TX
#define GPS_TX_PIN                  14   // Serial1 TX → GPS module RX (sends data requests)

// ── CRSF UART (RadioMaster ER8) ──────────────────────────────────────────────
#define CRSF_RX_PIN                 11   // Serial2 RX ← ER8 TX, 420000 baud
#define CRSF_TX_PIN                 12   // Serial2 TX → ER8 RX, telemetry uplink

// ── Onboard NeoPixel ─────────────────────────────────────────────────────────
#define LED_ONBOARD_PIN             48   // Fixed by DevKitC-1 hardware — WS2812 single LED


// ══════════════════════════════════════════════════════════════════════════════
//  SERIAL / CAN / I2C
// ══════════════════════════════════════════════════════════════════════════════

#define GPS_BAUD_RATE           115200   // Serial1 — Quectel LC29H NMEA output
#define CRSF_BAUD_RATE          420000   // Serial2 — CRSF standard baud rate

#define CAN_BAUD_RATE           250000   // TWAI CAN 2.0B, 250 kbit/s (matches VESC Tool setting)

// VESC CAN IDs (1-indexed, as configured in VESC Tool)
#define VESC_ID_LEFT                 1   // Left drive motor VESC
#define VESC_ID_RIGHT                2   // Right drive motor VESC
#define VESC_ID_BLADE                3   // Blade motor VESC (Gtech CLM021)

// IMU I2C address (BNO055 ADR/COM3 low → 0x28; high → 0x29)
#define IMU_I2C_ADDRESS           0x28   // Adafruit BNO055 default I2C address
#define IMU_I2C_CLOCK_HZ          100000 // BNO055 clock-stretch quirk — 100 kHz

// BNO055 sampling task (Core 0)
#define IMU_TASK_HZ               100    // BNO fusion output rate
#define IMU_TASK_PRIORITY         10
#define IMU_TASK_STACK            4096

// BNO055 axis remap to the robot frame (confirmed at bring-up).
// Defaults = chip placement P1 (Adafruit default). surge fwd+, sway right+,
// heave up+, heading CW-from-North+. Adjust REMAP/SIGN after the bring-up check.
#define IMU_AXIS_REMAP_CONFIG     0x24   // Adafruit_BNO055::REMAP_CONFIG_P1
#define IMU_AXIS_REMAP_SIGN       0x00   // Adafruit_BNO055::REMAP_SIGN_P1

// Heading-confidence gate: per-sensor BNO calibration levels (each 0..3) required
// for the absolute heading to be trusted. The BNO055 `sys` aggregate is unreliable
// (often stays 0 even when gyro/accel/mag are all 3, and mag decays when held
// still), so we gate on the three real sensors instead of sys.
#define IMU_CALIB_MAG_MIN         2
#define IMU_CALIB_GYRO_MIN        2
#define IMU_CALIB_ACCEL_MIN       2


// ══════════════════════════════════════════════════════════════════════════════
//  PHYSICAL DIMENSIONS  (all from steering centre = midpoint of front driven axle)
//  Configuration: front-drive differential + rear castors (Config A)
//  All dimensions in metres unless stated.
// ══════════════════════════════════════════════════════════════════════════════

// ── Robot footprint — OVERALL bounding box (outer extents) ───────────────────
// These are the outer dimensions of the robot's footprint and define the
// boundary-clearance exclusion zone. The nav inset is HALF THE DIAGONAL of this
// box (the radius the footprint corners sweep when the robot pivots on the spot)
// — see mower_config_nav_inset_m(). Measure the true outer extents, including
// any blade/guard overhang. (Steering centre assumed ~central in the box.)
#define FOOTPRINT_WIDTH_M         0.60f  // overall width, outer edge to outer edge [m]
#define FOOTPRINT_LENGTH_M        0.70f  // overall length, front-most to rear-most [m]

// ── Steering track (drivetrain) ──────────────────────────────────────────────
// Distance between the two track/wheel CENTRELINES — NOT the overall width.
// This is the kinematic track for differential steering odometry:
//   dHeading = (dRight − dLeft) / TRACK_WIDTH_M.
// For tracks of width w, this is (overall width − w): the steer reference runs
// up the middle of each track. Seeds odo_calib, which then GPS-calibrates it.
#define TRACK_WIDTH_M             0.50f  // track centre to centre [m]
#define WHEEL_HALF_TRACK_M        (TRACK_WIDTH_M / 2.0f)       // [m] — half of track width

// ── RTK antenna offset from steering centre ───────────────────────────────────
// Positive = ahead of/right of steering centre. Negative = behind/left.
// GPS accuracy depends on these values being measured carefully on the actual chassis.
#define ANTENNA_OFFSET_FORWARD_M  -0.20f // antenna offset along heading axis [m]
#define ANTENNA_OFFSET_RIGHT_M     0.00f // antenna offset perpendicular to heading [m]

// ── Blade geometry ────────────────────────────────────────────────────────────
// Signed distance from steering centre to cutting disc centre.
// Negative = blade behind steering centre (typical for front-drive + rear-castor).
// Measure on the actual chassis and update this value.
#define STEER_CENTRE_TO_CUT_CENTRE_M   0.0f  // [m] — positive=ahead, negative=behind
#define CUT_DISC_RADIUS_M              0.21f // [m] — Gtech CLM021 cutting radius (420mm dia / 2)


// ══════════════════════════════════════════════════════════════════════════════
//  DRIVE SYSTEM
// ══════════════════════════════════════════════════════════════════════════════

// Drive motor parameters (same for left and right VESCs)
#define MOTOR_POLE_PAIRS             7       // [count] — rotor magnet pole PAIRS (not poles)
#define GEAR_RATIO                20.0f     // [ratio] — gearbox reduction (motor to wheel)
#define WHEEL_RADIUS_M             0.10f    // [m] — driven wheel radius; measure on chassis

// Minimum turning radius: 0.0 = tracked/skid-steer (pivot on the spot).
// Wheeled differential without reverse on inner wheel: typically TRACK_WIDTH_M * 0.5.
// Pure Pursuit enforces this limit in curvature calculations.
#define MIN_TURNING_RADIUS_M       0.0f  // [m] — 0 = tracked vehicle (counter-rotation pivot)

/** Minimum perpendicular nav-boundary width needed for a U-turn.
 *  Spurs narrower than this get forward-in / reverse-out treatment.
 *  = 2 × MIN_TURNING_RADIUS_M gives one full diameter of clearance. */
#define SPUR_MIN_TURN_WIDTH_M      (2.0f * MIN_TURNING_RADIUS_M)  // [m]

/** Reverse speed while exiting a spur strip (blade ON). */
#define SPUR_REVERSE_SPEED_MS      MAX_MOWING_SPEED_MS  // [m/s]

// Motor current and speed limits
#define MAX_CURRENT_A             15.0f    // [A] — drive motor continuous current limit
#define MAX_WHEEL_SPEED_MS         0.5f    // [m/s] — maximum wheel surface speed
#define CURRENT_RAMP_A_PER_S       3.0f    // [A/s] — gentle current ramp on soft ground


// ══════════════════════════════════════════════════════════════════════════════
//  MOWING PARAMETERS
// ══════════════════════════════════════════════════════════════════════════════

// Strip geometry
#define CUT_WIDTH_M                0.38f   // [m] — effective cut width per pass (≈ blade dia − overlap)
#define STRIP_OVERLAP_M            0.02f   // [m] — additional overlap between adjacent strips

// Cut height range — servo maps linearly between these values (stored in NVS after cal)
// CUT_HEIGHT_MAX_MM also used as safe height for bog recovery passes.
#define CUT_HEIGHT_MIN_MM          35      // [mm] — minimum deck height (measured mechanical limit)
#define CUT_HEIGHT_MAX_MM          90      // [mm] — maximum deck height (measured mechanical limit)


// ══════════════════════════════════════════════════════════════════════════════
//  BLADE VESC PARAMETERS  (CAN ID 3 — Gtech CLM021, 800W / 48V, sensorless FOC)
// ══════════════════════════════════════════════════════════════════════════════

// Commanded speed
#define BLADE_TARGET_RPM          2800     // [RPM] — Gtech CLM021 rated mechanical speed
// 10 detent positions when turning rotor by hand = 10 pole pairs.
#define BLADE_MOTOR_POLE_PAIRS      10     // [count] — measured from rotor detents

// Soft start / stop timing
#define BLADE_SPINUP_TIME_MS      3000     // [ms] — duty ramps 0 → 100 % over this duration
// Suppress BLADE_FAULT for this long after the blade is commanded on, so the
// motor has time to physically spin up before the no-current/no-rpm check applies.
#define BLADE_FAULT_GRACE_MS      4000     // [ms]
// Below this |eRPM| the blade is considered "not turning" for fault detection
// (2800 RPM × 10 pole pairs = 28000 eRPM at speed; 1000 eRPM ≈ stopped).
#define BLADE_FAULT_MIN_ERPM      1000.0f  // [eRPM]

// Cutting load thresholds (fraction of BLADE_MAX_EXPECTED_CURRENT_A)
// BLADE_MAX_EXPECTED_CURRENT_A is a runtime variable initialised from NVS; see A20.
#define BLADE_LOAD_MIN             0.05f   // [fraction] — below this while commanded ON → blade fault
#define BLADE_LOAD_NORMAL          0.40f   // [fraction] — healthy cutting reference (A03)
#define BLADE_LOAD_HIGH            0.75f   // [fraction] — high load threshold, trigger speed reduction
#define BLADE_LOAD_CRITICAL        0.90f   // [fraction] — imminent stall, trigger bog recovery (A03)

// Current reference for load fraction calculation
// Runtime value is loaded from NVS; this is the compile-time default / fallback.
#define BLADE_MAX_EXPECTED_CURRENT_A_DEFAULT   12.0f  // [A] — default before auto-calibration

// Blade motor current LIMIT — must match the "Motor Current Max" set in VESC Tool.
// This is the fixed 100%-load reference for the blade-load display and thresholds
// (2026-06-13). Replaces the auto-calibrated reference, which was capturing the
// idle current (~7 A) as 100% and so reading ~105% at idle. With this fixed 15 A
// reference, idle (~7.5 A) reads ~50% and the HIGH (0.75) threshold sits at 11.25 A.
#define BLADE_CURRENT_LIMIT_A      15.0f   // [A] — VESC blade motor current limit

// Auto-calibration parameters (P90 of blade current during warmup phase)
#define BLADE_CAL_WARMUP_MS        8000    // [ms] — warmup duration before P90 is computed
#define BLADE_CAL_MIN_VALID_A      2.0f    // [A] — minimum current for a valid calibration sample

// Rolling average window for load assessment
#define BLADE_LOAD_SAMPLE_WINDOW_MS  500   // [ms] — rolling window; 500ms masks brief spikes (A03 corrected)

// ── Feature 2 (2026-06-16): RPM-based blade load + load recovery + adaptive speed ─
// The blade VESC is current-limited (SET_RPM PID caps current), so RPM is a better
// load proxy than current: free spin holds the target RPM; under load the RPM droops.
//   rpm_load = clamp((blade_target_rpm - actual_rpm) / blade_target_rpm, 0, 1)
// = 0 % at/above target, 100 % at standstill. This drives the Tx/PWA load display
// (cutting_monitor_get_rpm_load_fraction()). The current-based fraction is retained
// for diagnostics (amps) and the (inert) classifier.
#define BLADE_RPM_LOAD_EMA_ALPHA      0.30f   // EMA smoothing on the displayed/triggering rpm-load

// Load-triggered recovery (hosted in the existing STATE_RETRACE slot). This flag is
// INDEPENDENT of AUTO_FAULT_RESPONSES_ENABLED — enabling it does NOT wake the old
// current-overload / stall / slip / obstacle detectors. Default OFF: field-verify the
// RPM load reading on the Tx before letting it move actuators (user decision).
#define BLADE_RPM_RECOVERY_ENABLED        0       // master gate for the new recovery
#define BLADE_RPM_RECOVERY_LOAD           0.80f   // [fraction] rpm-load that triggers recovery (~560 rpm at 2800)
#define BLADE_RPM_RECOVERY_CONFIRM_MS     1500    // [ms] rpm-load must stay high this long before triggering
#define BLADE_RECOVERY_BACKUP_MAX_M       1.0f    // [m] max reverse distance (perimeter-clipped)
#define BLADE_RECOVERY_MAX_PASSES         BOG_MAX_RETRIES  // re-use the bog step-down retry budget (6)

// Load-adaptive mow speed (built but DISABLED this cycle — user deferred it so it
// cannot obscure other issues while debugging the new metric). Applied at the
// node_follower speed hook, BEFORE the min_creep floor, so it can only slow, never stall.
#define BLADE_LOAD_ADAPTIVE_SPEED_ENABLED 0       // master gate for inverse-load speed
#define BLADE_LOAD_SPEED_KNEE             0.50f   // [fraction] rpm-load above which slowdown begins
#define BLADE_LOAD_SPEED_MIN_FACTOR       0.40f   // [×] floor on the speed factor before min_creep


// ══════════════════════════════════════════════════════════════════════════════
//  RC INPUT — CRSF CHANNEL THRESHOLDS
// ══════════════════════════════════════════════════════════════════════════════

// CRSF raw range: 172 (1000μs) – 1811 (2000μs), centre 992 (1500μs)
// CH4 mode thresholds (raw CRSF units)
// CH4: <496=MANUAL, 496–1316=AUTO, >1316=AUTO+RETURN
// CH5: >1316 = perimeter learning active (2-position switch)
// CH6: <496=DISARMED, >496=ARMED (2-position switch; A02 corrected)

#define CH6_ARMED_THRESHOLD        496     // [CRSF raw] — any "high" switch position = armed (A02)
#define CH3_HEIGHT_MOVE_THRESHOLD   30     // [CRSF raw] — min CH3 change to override WebUI height (~1 mm)
#define CH5_LEARN_THRESHOLD       1316     // [CRSF raw] — above this = perimeter learning active
#define CRSF_CH_PAUSE              6       // CH7 (0-indexed) — pause while active (latching switch)
#define CH7_PAUSE_THRESHOLD        496     // [CRSF raw] — above this = pause active


// ══════════════════════════════════════════════════════════════════════════════
//  NAVIGATION CONSTANTS
// ══════════════════════════════════════════════════════════════════════════════

// Mowing speeds
#define MAX_MOWING_SPEED_MS        0.15f   // [m/s] — normal mowing speed
#define LONG_GRASS_SPEED_MS        0.08f   // [m/s] — reduced speed when grass is tall
#define LONG_GRASS_THRESHOLD_MM    100     // [mm] — cut height above which LONG_GRASS_SPEED is used
#define TRANSIT_SPEED_MS           0.30f   // [m/s] — speed during headland transitions
#define HEADLAND_SPEED_MS          0.20f   // [m/s] — speed during perimeter/headland passes
#define MIN_CREEP_SPEED_MS         0.05f   // [m/s] — minimum commanded speed (below this = stopped)

// Path following
#define WAYPOINT_ARRIVE_DIST_M     0.15f   // [m] — distance to waypoint to consider it reached
#define PURE_PURSUIT_LOOKAHEAD_BASE_M  0.40f  // [m] — base lookahead distance at zero speed
#define PURE_PURSUIT_LOOKAHEAD_K   0.80f   // [s] — lookahead gain: total = base + K * speed
#define PERIMETER_CLOSE_WINDOW    15    // points searched at each end of track for closest-endpoint close

// Pivot-on-the-spot (tank turn). This is a tracked vehicle (MIN_TURNING_RADIUS_M=0):
// when the heading error to the lookahead target is too large for a forward arc,
// the AUTO follower spins in place (one track forward, one reverse) until roughly
// aligned, then resumes forward pursuit. Lets it reach tight corner nodes that a
// forward arc would overshoot. Hysteresis (enter > exit) prevents chatter.
#define PIVOT_ENTER_DEG            55.0f  // [deg] — start pivoting when |heading error| exceeds this
#define PIVOT_EXIT_DEG            12.0f  // [deg] — stop pivoting once |heading error| drops below this
#define PIVOT_WHEEL_MS            0.18f  // [m/s] — per-track tangential speed during a pivot

// Node follower (drive branch): P-controller turns yaw rate from heading error.
#define NODE_HEADING_KP           1.5f   // [1/s] — yaw rate = KP × heading error (rad)
#define NODE_YAW_RATE_MAX         1.2f   // [rad/s] — cap on drive-branch yaw rate

// RTK requirement for perimeter learning.
#define RTK_MIN_FIX_FOR_LEARNING      4    // [fix_type] — 4=RTK fixed required

// ── Heading = BNO absolute fusion + GPS-trimmed offset ────────────────────────
// The BNO055 supplies a tilt-compensated absolute heading at 100 Hz. The only
// correction is a slowly-trimmed offset (mounting + magnetic declination),
// estimated from the GPS travel chord on STRAIGHT, MEASURABLE RTK segments and
// persisted to NVS. No wheel-odometry heading, no gyro-pivot hack, no GPS lock.
//   STRAIGHT_MAX_TURN_RAD : Σ|Δheading| since the reference above this → the
//                           segment wasn't straight; discard the chord, restart.
//   FROM_GPS_MIN_DIST_M   : distance floor for a usable chord.
//   DIST_SIGMA_K          : required chord also ≥ K × GPS sigma (1 cm fixed →
//                           0.3 m; 15 cm float → ~3 m).
//   OFFSET_TRIM_GAIN      : EMA gain folding each chord estimate into the offset.
#define HEADING_STRAIGHT_MAX_TURN_RAD 0.20f   // ~11 deg
#define HEADING_FROM_GPS_MIN_DIST_M   0.30f   // [m]
#define HEADING_GPS_DIST_SIGMA_K      20.0f
#define HEADING_OFFSET_TRIM_GAIN      0.10f   // EMA gain per qualifying chord

// Offset NVS persistence throttle (avoid flash wear; offset drifts very slowly).
#define HEADING_OFFSET_SAVE_MIN_INTERVAL_MS  30000   // [ms]
#define HEADING_OFFSET_SAVE_MIN_CHANGE_RAD   0.0087f // ~0.5 deg

// EKF heading variance when the BNO is healthy (absolute fused heading is good).
#define EKF_HDG_VAR_BNO               0.0012f // rad² (~2 deg 1-σ)

// ── AUTO fault responses — TEMPORARILY DISABLED (2026-06-13) ──────────────────
// The cutting-overload, wheel-stall ("crash"), wheel-slip and obstacle-suspected
// detectors are unreliable (e.g. blade load reads 105% constantly → false
// overload), so they spuriously toggle the blade and bounce AUTO into the
// recovery sub-states (RETRACE/BOG_RECOVERY/OBSTACLE_AVOID) — one of which was
// driving away at full speed on blade-start. While 0, AUTO just follows the
// planned path and the BLADE IS CONTROLLED SOLELY BY THE OPERATOR (RC arm
// switch), exactly like MANUAL. Tilt → PAUSE and perimeter/VESC safety stay
// active. Set back to 1 once the detectors are fixed and recalibrated.
#define AUTO_FAULT_RESPONSES_ENABLED  0

// Obstacle avoidance
#define OBSTACLE_BACKUP_DIST_M        0.40f   // [m] — reverse distance after collision (default; direction-adjusted)
#define OBSTACLE_BACKUP_SPEED_MS      0.10f   // [m/s] — speed during reverse after collision
#define OBSTACLE_CONFIRM_MS            500    // [ms] — time cutting obstacle must persist to be confirmed (A03 corrected)
#define OBSTACLE_DETECT_STARTUP_MS   10000   // [ms] — startup gate: suppress cutting-monitor triggers for this long after AUTO entry

// ── IMU Collision Detection ───────────────────────────────────────────────────
// Replaces the physical bumper sensor (no dedicated bumper GPIO — GPIO 6 is now
// the PAUSE switch, PAUSE_PIN; see ASSUMPTIONS.md A34).
// Collision is detected when a short-duration acceleration spike exceeds
// COLLISION_THRESHOLD_MULTIPLIER × the adaptive baseline jolt level.
#define COLLISION_THRESHOLD_MULTIPLIER  5.0f   // spike must be n× baseline to trigger
#define COLLISION_RISE_TIME_MS          50     // spike rise time < this = collision (ms)
                                               // terrain bumps rise more slowly
#define COLLISION_CONFIRM_MS            20     // spike must persist this long before triggering
#define COLLISION_RMS_WINDOW_SAMPLES    25     // short-window RMS: 25 samples = 250ms at 100Hz (BNO task)

// Adaptive baseline parameters
#define BASELINE_ALPHA                  0.001f // EMA adaptation rate (~1000 samples = ~5s full update)
#define BASELINE_DEFAULT_G              0.15f  // first-run default; tunes itself down in use
#define BASELINE_SETTLE_M               3.0f   // metres driven before baseline updates begin
#define BASELINE_OUTLIER_GATE           3.0f   // reject samples > n× baseline (prevents collision corruption)
#define BASELINE_NVS_SAVE_INTERVAL_S    120    // minimum seconds between NVS writes
#define BASELINE_NVS_MIN_CHANGE         0.02f  // only write NVS if baseline changed by more than this (g)

// Direction detection thresholds (impact angle from chassis forward axis)
#define COLLISION_FORWARD_CONE_DEG      45.0f  // |angle| < this = forward hit
#define COLLISION_SIDE_CONE_DEG         45.0f  // 90° ± this = side hit
#define RETRACE_OVERLAP_M          0.10f   // [m] — overlap when retracing a strip at higher cut height (A03 corrected)

// ── Uncertainty-aware navigation ─────────────────────────────────────────
#define UNCERTAINTY_MARGIN_M          0.50f  // [m] margin below which caution begins
#define TILT_LIMIT_NORMAL_DEG        30.0f   // [deg] max tilt in normal operation
#define TILT_LIMIT_CAREFUL_DEG       15.0f   // [deg] max tilt when margin is low
#define COLLISION_MULT_CAREFUL        3.0f   // collision multiplier when margin is low
#define TILT_EMA_ALPHA               0.016f  // EMA filter for tilt/pitch/roll (~0.6s at the 100Hz IMU task)

// ── Heading stabilisation ────────────────────────────────────────────────────
// Gains in [(m/s)/rad] and [(m/s)/(rad/s)].
// The correction is a velocity differential applied to each wheel's speed setpoint.
#define HEADING_KP                   0.30f  // [(m/s)/rad]
#define HEADING_KD                   0.05f  // [(m/s)/(rad/s)]
#define MANUAL_MAX_YAW_RATE          0.8f   // [rad/s] yaw rate at full steering deflection

// ── ESP32-side wheel speed PI ────────────────────────────────────────────────
// VESC runs in current-control mode; the ESP32 closes the velocity loop.
// Kp: output Amps per m/s of velocity error.
// Ki: output Amps per accumulated (m/s·s) of velocity error.
// Starting values conservative — tune up if response is slow.
#define WHEEL_PI_KP                 10.0f   // [A/(m/s)]
#define WHEEL_PI_KI                  2.0f   // [A/(m/s·s)]

// Maximum ERPM for drive wheels derived from max wheel surface speed.
// erpm = speed_ms × gear_ratio / (2π × wheel_r) × pole_pairs × 60
#define MAX_WHEEL_ERPM  ((float)(MAX_WHEEL_SPEED_MS \
        * GEAR_RATIO \
        / (2.0f * (float)M_PI * WHEEL_RADIUS_M) \
        * MOTOR_POLE_PAIRS \
        * 60.0f))

// Manual drive deadband applied inside drive_manual() after CRSF normalisation.
// Output is rescaled so the response is continuous (no jump at the band edge).
// crsf_us_to_norm() already zeroes ±5%; this adds a larger rescaled band on top.
#define MANUAL_DEADBAND              0.10f  // [fraction 0–1] — 10% each side of centre
#define MANUAL_EXPO                  0.65f  // [0–1] — exponential curve; 0=linear, 1=full cubic
#define MANUAL_MAX_DUTY              0.60f  // [0–1] — duty ceiling for both MANUAL and AUTO_MOWING
#define MANUAL_MAX_SPEED_MS          0.5f   // [m/s] — max wheel speed at full throttle stick in manual mode
#define MANUAL_DUTY_RAMP_PER_S       0.4f   // [duty/s] — max rate of change of wheel duty in manual mode (prevents wheelies)

// AUTO_MOWING duty-cycle ramp: increment per 100 ms tick (10 Hz call rate).
// 0.02 / tick = 0.2 / second = full range in 5 seconds.
// Tune in field: increase if acceleration is too slow; decrease if too jerky.
#define DUTY_RAMP_STEP               0.02f  // [duty / tick]
// eRPM feedback older than this is treated as absent: the duty ramp must NOT
// integrate against a frozen reading (error never closes → duty winds up to
// max → runaway). Falls back to bounded open-loop duty instead.
#define DUTY_FEEDBACK_STALE_MS       500    // [ms]

// Static-friction kickstart (AUTO duty floor). At creep speed the proportional
// open-loop duty (≈0.06) sits below the duty needed to break static friction, so
// the wheels never start. When a real wheel move is commanded, the AUTO duty ramp
// floors |duty| to at least this value in the commanded direction. Runtime-tunable
// as MowerConfig.min_move_duty (PWA). 0 disables the kickstart. Tune in field:
// raise until the mower reliably starts from rest at creep; lower if it lurches.
#define MIN_MOVE_DUTY                0.12f  // [duty 0–1] — break-free duty floor

// Wheel-slip detection: if GPS/EKF speed < wheel_erpm_speed × this threshold,
// the wheels are spinning but the robot is barely moving → bog-in-progress.
// 0.5 = EKF speed must be at least 50% of wheel speed to avoid slip flag.
#define SLIP_RATIO_THRESHOLD         0.5f   // [fraction 0–1]

// ── AGC (Automatic Gain Control) ─────────────────────────────────────────────
#define AGC_WINDOW_TICKS              50    // ~5s at 10Hz state machine rate
#define AGC_OSCILLATION_THRESHOLD    0.3f   // zero-crossings/window above which gain decreases
#define AGC_ERROR_THRESHOLD          0.05f  // [rad] ~3° persistent error above which gain increases
#define AGC_DECAY_RATE               0.95f  // gain multiplier when oscillating
#define AGC_GROW_RATE                1.02f  // gain multiplier when persistent error
#define AGC_GAIN_MIN                 0.2f   // minimum gain multiplier
#define AGC_GAIN_MAX                 2.0f   // maximum gain multiplier

// Perimeter and polygon size constraints
#define MAX_PERIMETER_POINTS        500    // [count] — maximum recorded perimeter waypoints
#define MAX_OBSTACLES                50    // [count] — maximum tracked obstacle positions
#define MIN_ZONE_AREA_M2           0.50f   // [m²] — discard inset sub-polygons smaller than this


// ══════════════════════════════════════════════════════════════════════════════
//  EKF TUNING
//  Process noise covariance Q — scaled by dt in ekf_predict()
//  State: [x(m), y(m), theta(rad), v(m/s)]
// ══════════════════════════════════════════════════════════════════════════════

#define EKF_Q_POS               0.050f    // [m²/s]     — position process noise
#define EKF_Q_HDG               0.010f    // [rad²/s]   — heading process noise
#define EKF_Q_VEL               0.100f    // [(m/s)²/s] — velocity process noise
#define EKF_Q_NO_DR_SCALE       0.001f    // Q multiplier when no gyro/wheel data

// GPS innovation gate is now dynamic (5×sigma, min 0.3 m) computed in
// ekf_update_gps() from fix_type + HDOP + dif_age_s. No static constant needed.

// Covariance clamps — prevent unbounded growth if corrections stop
#define EKF_P_MAX_POS          25.0f    // [m²]      — max position variance (~5 m 1σ)
#define EKF_P_MAX_HDG           1.0f    // [rad²]    — max heading variance (~57°)
#define EKF_P_MAX_VEL           1.0f    // [(m/s)²]  — max velocity variance

// GPS ceiling is applied continuously (in both ekf_predict and ekf_update_gps)
// using the measurement noise R from the last GPS fix — no multiplier needed.


// ══════════════════════════════════════════════════════════════════════════════
//  SAFETY AND TIMING
// ══════════════════════════════════════════════════════════════════════════════

// Watchdog timeouts — exceeding these triggers STATE_PAUSED (or MOTORS_OFFLINE for VESC silence)
#define RC_FAILSAFE_TIMEOUT_MS       500   // [ms] — no valid CRSF frame → failsafe
#define GPS_UPDATE_TIMEOUT_MS       2500   // [ms] — no GPS NMEA sentence → GPS fault
#define VESC_STATUS_TIMEOUT_MS      2000   // [ms] — no VESC STATUS packet → VESC fault
#define BLADE_VESC_TIMEOUT_MS       5000   // [ms] — blade VESC silent this long → stop blade
#define VESC_STARTUP_GRACE_MS       3000   // [ms] — after bus goes live, all VESCs must appear within this window

// Perimeter safety
#define PERIMETER_BREACH_DIST_M    0.80f   // [m] — steering centre outside the PERIMETER by this much → E-stop

// Stall / bog detection
#define STALL_DETECT_TIME_MS        3000   // [ms] — time below stall threshold before bog recovery
#define STALL_SPEED_THRESHOLD_MS   0.03f   // [m/s] — below this (while commanded) = stalled

// Bog recovery — progressive height raise then retry
// 10mm steps over 30–200mm range = up to 17 steps; BOG_MAX_RETRIES limits attempts.
#define BOG_HEIGHT_STEP_MM            10   // [mm] — deck height raised per retry step
#define BOG_MAX_RETRIES                6   // [count] — max height-step attempts before marking as obstacle
#define BOG_PASS_DWELL_MS            300   // [ms] — pause between consecutive bog recovery passes

// LED strip
#define LED_EXTERNAL_COUNT            22   // [count] — number of LEDs in external WS2812 strip


// ══════════════════════════════════════════════════════════════════════════════
//  BATTERY MONITORING  (VESC CAN STATUS_5 — replaces ADC voltage divider)
//  Voltage is read from CAN_PACKET_STATUS_5 broadcast by VESC_ID_LEFT (CAN ID 1).
//  Pack: 13S LiPo nominal (operator must verify and adjust if 14S)
//  See HANDOFFS/28_vesc_battery/HANDOFF.md for migration details.
// ══════════════════════════════════════════════════════════════════════════════

#define BATTERY_WARN_V            45.5f    // [V] — 13S × 3.5V/cell — warn operator, continue mowing
#define BATTERY_LOW_V             42.9f    // [V] — 13S × 3.3V/cell — stop all motors immediately
#define BATTERY_FILTER_ALPHA       0.10f   // [0–1] — IIR filter coefficient for VESC voltage (2Hz call rate → ~5s TC)
// Hysteresis on WARNING→OK: voltage must recover this much above BATTERY_WARN_V
// to clear the warning. Prevents flapping when voltage oscillates around threshold. (MED-4 fix)
#define BATTERY_WARN_HYSTERESIS_V  1.0f   // [V]


// ══════════════════════════════════════════════════════════════════════════════
//  DERIVED / COMPUTED CONSTANTS
//  No magic numbers in .cpp files — all geometry depends on these.
//  All formulas taken directly from spec (Physical Dimensions section).
//
//  Computed values for review (at default physical constants):
//    NAV_EXCLUSION_INSET_M = 0.5·√(0.60² + 0.70²) + 0.05      = 0.51 m
//    BLADE_FORWARD_REACH_M = max(0.0, 0.0 + 0.21)              = 0.21 m
//    HEADLAND_WIDTH_M      = max(max(0.57, 0.37), 0.23)        = 0.57 m
//
//  NOTE: these compile-time macros are not used by the firmware (the runtime
//  mower_config_*() helpers are the live source). They are kept as documentation
//  of the formulas and updated to match the overall-footprint model.
// ══════════════════════════════════════════════════════════════════════════════

// Inset 1: navigation exclusion boundary = HALF THE FOOTPRINT DIAGONAL + GPS margin.
// The steering centre must stay this far inside the perimeter so that no footprint
// corner sweeps past it when the robot pivots on the spot (see mower_config_nav_inset_m()).
#define NAV_EXCLUSION_INSET_M \
    (0.5f * sqrtf(FOOTPRINT_WIDTH_M*FOOTPRINT_WIDTH_M + \
                  FOOTPRINT_LENGTH_M*FOOTPRINT_LENGTH_M) + 0.05f)

// Blade forward reach: distance the cut disc extends ahead of steering centre.
// Zero or positive only — a rearward blade does not project forward into the headland.
#define BLADE_FORWARD_REACH_M \
    (max(0.0f, STEER_CENTRE_TO_CUT_CENTRE_M + CUT_DISC_RADIUS_M))

// Headland width: inset from nav boundary to working area boundary (working area
// is vestigial under the spiral planner but still derived/displayed).
#define HEADLAND_WIDTH_M \
    (max(max(CUT_WIDTH_M * 1.5f, \
             FOOTPRINT_LENGTH_M * 0.5f + STRIP_OVERLAP_M), \
         BLADE_FORWARD_REACH_M + STRIP_OVERLAP_M))

// Blade target ERPM sent to blade VESC via CAN_PACKET_SET_RPM
// ERPM = mechanical RPM × pole pairs
#define BLADE_TARGET_ERPM \
    (BLADE_TARGET_RPM * BLADE_MOTOR_POLE_PAIRS)
