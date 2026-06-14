#pragma once
#include <Arduino.h>

// ══════════════════════════════════════════════════════════════════════════════
//  collision_detect.h — RoboMower IMU-Based Collision Detection
//
//  Replaces the physical bumper sensor (removed; GPIO6 is now the pause switch).
//  Uses BNO055 gravity-removed linear acceleration (fed at 100Hz from the Core 0
//  IMU task, in g) to detect short-duration acceleration spikes from obstacle
//  contact. NOTE: detection is currently DISABLED via AUTO_FAULT_RESPONSES_ENABLED;
//  this module captures a baseline for future re-tuning.
//
//  Detection method:
//    1. Compute short-window RMS variation of compensated acceleration magnitude.
//    2. Maintain an adaptive EMA baseline of normal driving jolt level.
//    3. Trigger when a spike exceeds COLLISION_THRESHOLD_MULTIPLIER × baseline
//       AND has a rise time shorter than COLLISION_RISE_TIME_MS (distinguishes
//       collisions from slow terrain undulations).
//    4. Classify impact direction from accelerometer axis ratios at spike peak.
//
//  Thread safety:
//    collisionDetectUpdate() — called from Core 0 (IMU task) only.
//    All other functions — called from Core 1 (state machine) only.
//    s_collisionFlag is volatile bool: set by Core 0, cleared by Core 1.
//    Single-writer/single-reader volatile is safe without a spinlock.
//
//  References:
//    Spec:        COLLISION_DETECT.md
//    Assumptions: ASSUMPTIONS.md A34–A37
//    Config:      Robo-Mower-V2/config.h (COLLISION_* and BASELINE_* constants)
// ══════════════════════════════════════════════════════════════════════════════


// ── Direction enumeration ─────────────────────────────────────────────────────

/** Direction of detected collision impact, relative to chassis forward axis. */
enum CollisionDirection {
    COLLISION_DIR_FORWARD,       ///< |angle| < COLLISION_FORWARD_CONE_DEG
    COLLISION_DIR_REVERSE,       ///< |angle - 180°| < COLLISION_FORWARD_CONE_DEG
    COLLISION_DIR_LEFT,          ///< strong Y-axis, negative
    COLLISION_DIR_RIGHT,         ///< strong Y-axis, positive
    COLLISION_DIR_OBLIQUE_FL,    ///< front-left
    COLLISION_DIR_OBLIQUE_FR,    ///< front-right
    COLLISION_DIR_TERRAIN,       ///< Z-axis dominant — terrain irregularity, not obstacle
    COLLISION_DIR_UNKNOWN,
};


// ── Public API ────────────────────────────────────────────────────────────────

/**
 * @brief Initialise collision detection. Loads baseline from NVS (Preferences,
 *        namespace "collision", key "baseline_v2"). Call once from setup() after
 *        the IMU is initialised.
 */
void collisionDetectInit();

/**
 * @brief Feed a new IMU sample into the collision detector.
 *        Call at 100Hz from the IMU sampling task (Core 0).
 *        Pass BNO055 linear acceleration in g (gravity already removed by fusion).
 *
 * @param ax  Surge (forward) linear acceleration in g
 * @param ay  Sway (right) linear acceleration in g
 * @param az  Heave (up) linear acceleration in g
 */
void collisionDetectUpdate(float ax, float ay, float az);

/**
 * @brief Allow baseline adaptation during straight driving.
 *        Call from the node-follower / AUTO mowing path when mowing conditions are valid:
 *          - speed > MIN_CREEP_SPEED_MS
 *          - |yaw_rate| < 0.2 rad/s (not turning)
 *          - session distance > BASELINE_SETTLE_M
 *        Do NOT call during turns, bog recovery, or obstacle avoidance.
 *
 * @param allow  true = baseline updates permitted this cycle
 */
void collisionDetectSetBaselineUpdate(bool allow);

/**
 * @brief Override the collision threshold multiplier at runtime.
 *
 * Default is COLLISION_THRESHOLD_MULTIPLIER (5.0). Lower values make
 * detection more sensitive. Call from the state machine task (Core 1) only.
 * The multiplier is applied on Core 0 at the next collisionDetectUpdate().
 *
 * @param mult  New threshold multiplier (spike must exceed baseline × mult)
 */
void collisionDetectSetMultiplier(float mult);

/**
 * @brief Returns true if a collision has been detected since the last
 *        call to collisionClear(). Non-blocking — safe to poll from Core 1.
 */
bool collisionDetected();

/**
 * @brief Returns the direction of the most recent collision.
 *        Valid only when collisionDetected() returns true.
 */
CollisionDirection collisionGetDirection();

/**
 * @brief Clear the collision flag after the state machine has acted on it.
 */
void collisionClear();

/**
 * @brief Save current baseline to NVS if the interval and change threshold
 *        are both met (BASELINE_NVS_SAVE_INTERVAL_S, BASELINE_NVS_MIN_CHANGE).
 *        Call periodically from AUTO_MOWING (e.g. every 10 seconds).
 */
void collisionSaveBaselineIfDue();

/**
 * @brief Force-save the baseline to NVS immediately, regardless of interval
 *        or change threshold. Call on clean exit from AUTO_MOWING to preserve
 *        the session's learned baseline.
 */
void collisionSaveBaselineForced();

/**
 * @brief Returns the current adaptive baseline in g (for STATUS/CALDUMP output).
 */
float collisionGetBaseline();

/**
 * @brief Returns the latest short-window jolt RMS in g — the live signal the
 *        adaptive baseline tracks. Used for baseline-capture logging.
 */
float collisionGetJoltRms();
