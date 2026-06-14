// ══════════════════════════════════════════════════════════════════════════════
//  collision_detect.cpp — RoboMower IMU-Based Collision Detection
//
//  Implements adaptive-baseline jolt detection using BNO055 linear acceleration
//  (gravity already removed by fusion, in g). Core 0 calls collisionDetectUpdate()
//  at 100Hz. NOTE: detection is currently DISABLED via AUTO_FAULT_RESPONSES_ENABLED;
//  this module captures a baseline for future re-tuning.
//  Core 1 (state machine) reads collisionDetected() / collisionGetDirection().
//
//  References:
//    Spec:        COLLISION_DETECT.md Step 3
//    Assumptions: ASSUMPTIONS.md A34–A37
//    Config:      Robo-Mower-V2/config.h
// ══════════════════════════════════════════════════════════════════════════════

#include "collision_detect.h"
#include "config.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>


// ── NVS namespace and key ─────────────────────────────────────────────────────
static const char* NVS_NS  = "collision";
// "baseline_v2": fresh key for the BNO055 (gravity-removed linear accel in g).
// The old "baseline" value (raw-accel-minus-1g from the previous IMU) has different
// noise characteristics and is intentionally abandoned so the detector re-learns
// from BASELINE_DEFAULT_G.
static const char* NVS_KEY = "baseline_v2";


// ── Internal state ────────────────────────────────────────────────────────────

// Spinlock protecting s_baseline and s_savedBaseline.
// Core 0 (IMU task) writes s_baseline at 100 Hz; Core 1 reads it in
// collisionGetBaseline() and collisionSaveBaselineIfDue(). (HIGH-3 fix)
static portMUX_TYPE s_baseline_mux = portMUX_INITIALIZER_UNLOCKED;

static float    s_baseline       = BASELINE_DEFAULT_G;
static float    s_savedBaseline  = BASELINE_DEFAULT_G;  // value at last NVS write
static volatile float s_jolt_rms = 0.0f;  // latest short-window jolt RMS (g), for capture/diagnostics

// Circular buffer for short-window RMS computation
static float    s_rmsWindow[COLLISION_RMS_WINDOW_SAMPLES];
static int      s_rmsIdx         = 0;
static float    s_rmsSum         = 0.0f;   // sum of mag² in window
static float    s_magSum         = 0.0f;   // sum of mag in window

static bool     s_baselineAllowed = false;
static volatile float s_multiplier  = COLLISION_THRESHOLD_MULTIPLIER;

// Collision flag and direction — set by Core 0, read/cleared by Core 1
static volatile bool             s_collisionFlag = false;
static volatile CollisionDirection s_direction   = COLLISION_DIR_UNKNOWN;

// Spike rise-time tracking
static float    s_prevMag        = 0.0f;
static float    s_spikeStartMag  = 0.0f;
static uint32_t s_spikeStartMs   = 0;
static bool     s_inSpike        = false;

// Axis capture at spike peak
static float    s_peakAx = 0.0f;
static float    s_peakAy = 0.0f;
static float    s_peakAz = 0.0f;
static float    s_peakMag = 0.0f;

// NVS timing
static uint32_t s_lastNvsSaveMs = 0;


// ── Private helpers ───────────────────────────────────────────────────────────

/**
 * Classify impact direction from chassis-frame accelerations at spike peak.
 *
 * ax = forward, ay = left, az = up (all gravity-compensated).
 * Returns TERRAIN if Z-axis dominates (> 2× horizontal magnitude).
 * Otherwise classifies by atan2 angle from forward axis.
 */
static CollisionDirection computeDirection(float ax, float ay, float az) {
    float horiz = sqrtf(ax * ax + ay * ay);

    // Z dominant → terrain irregularity, not an obstacle
    if (az > horiz * 2.0f) {
        return COLLISION_DIR_TERRAIN;
    }

    // angle_deg: 0°=forward, 90°=left, −90°=right, ±180°=reverse
    float angle_deg = atan2f(ay, ax) * 180.0f / (float)M_PI;

    float fwd_cone  = COLLISION_FORWARD_CONE_DEG;
    float side_cone = COLLISION_SIDE_CONE_DEG;

    if (fabsf(angle_deg) < fwd_cone) {
        return COLLISION_DIR_FORWARD;
    }
    if (fabsf(angle_deg) > (180.0f - fwd_cone)) {
        return COLLISION_DIR_REVERSE;
    }
    if (angle_deg > (90.0f - side_cone) && angle_deg < (90.0f + side_cone)) {
        return COLLISION_DIR_LEFT;
    }
    if (angle_deg < -(90.0f - side_cone) && angle_deg > -(90.0f + side_cone)) {
        return COLLISION_DIR_RIGHT;
    }
    // Oblique quadrants
    if (angle_deg > 0.0f) {
        return (angle_deg < 90.0f) ? COLLISION_DIR_OBLIQUE_FL : COLLISION_DIR_LEFT;
    }
    return (angle_deg > -90.0f) ? COLLISION_DIR_OBLIQUE_FR : COLLISION_DIR_RIGHT;
}


// ── Public API ────────────────────────────────────────────────────────────────

void collisionDetectInit() {
    // Load baseline from NVS
    Preferences prefs;
    prefs.begin(NVS_NS, /*readOnly=*/true);
    s_baseline      = prefs.getFloat(NVS_KEY, BASELINE_DEFAULT_G);
    prefs.end();
    s_savedBaseline = s_baseline;

    // Initialise RMS buffer to zeros
    for (int i = 0; i < COLLISION_RMS_WINDOW_SAMPLES; i++) {
        s_rmsWindow[i] = 0.0f;
    }
    s_rmsIdx  = 0;
    s_rmsSum  = 0.0f;
    s_magSum  = 0.0f;

    // Reset spike state
    s_inSpike      = false;
    s_prevMag      = 0.0f;
    s_spikeStartMag = 0.0f;
    s_spikeStartMs = 0;
    s_peakMag      = 0.0f;
    s_peakAx = s_peakAy = s_peakAz = 0.0f;

    s_collisionFlag   = false;
    s_direction       = COLLISION_DIR_UNKNOWN;
    s_baselineAllowed = false;
    s_lastNvsSaveMs   = 0;

    DBG_PRINTF("[COLLISION] Baseline loaded: %.4f g\n", s_baseline);
}


void collisionDetectUpdate(float ax, float ay, float az) {
    // ── 1. Magnitude of gravity-compensated acceleration ──────────────────────
    float mag = sqrtf(ax * ax + ay * ay + az * az);

    // ── 2. Update circular RMS buffer ─────────────────────────────────────────
    // Remove oldest sample from running sums
    float old_mag  = s_rmsWindow[s_rmsIdx];
    s_rmsSum      -= old_mag * old_mag;
    s_magSum      -= old_mag;

    // Add new sample
    s_rmsWindow[s_rmsIdx] = mag;
    s_rmsSum             += mag * mag;
    s_magSum             += mag;
    s_rmsIdx              = (s_rmsIdx + 1) % COLLISION_RMS_WINDOW_SAMPLES;

    // ── 3. Short-window jolt RMS (standard deviation of magnitude) ────────────
    float mean_mag  = s_magSum  / (float)COLLISION_RMS_WINDOW_SAMPLES;
    float mean_mag2 = s_rmsSum  / (float)COLLISION_RMS_WINDOW_SAMPLES;
    float variance  = mean_mag2 - mean_mag * mean_mag;
    if (variance < 0.0f) variance = 0.0f;  // guard floating point rounding
    float jolt_rms  = sqrtf(variance);
    s_jolt_rms = jolt_rms;   // publish for baseline-capture logging (Core 1 reads)

    // ── 4. Baseline adaptation (only during straight mowing) ──────────────────
    if (s_baselineAllowed && jolt_rms < s_baseline * BASELINE_OUTLIER_GATE) {
        taskENTER_CRITICAL(&s_baseline_mux);
        s_baseline = s_baseline * (1.0f - BASELINE_ALPHA)
                   + jolt_rms  * BASELINE_ALPHA;
        taskEXIT_CRITICAL(&s_baseline_mux);
    }

    // ── 5. Spike detection with rise-time gate ────────────────────────────────
    taskENTER_CRITICAL(&s_baseline_mux);
    float bl   = s_baseline;
    float mult = s_multiplier;
    taskEXIT_CRITICAL(&s_baseline_mux);
    float threshold = bl * mult;

    if (!s_inSpike && mag > threshold) {
        // Spike start
        s_inSpike       = true;
        s_spikeStartMs  = millis();
        s_spikeStartMag = s_prevMag;
        s_peakMag       = mag;
        s_peakAx = ax;  s_peakAy = ay;  s_peakAz = az;

    } else if (s_inSpike) {
        // Track peak while in spike
        if (mag > s_peakMag) {
            s_peakMag = mag;
            s_peakAx = ax;  s_peakAy = ay;  s_peakAz = az;
        }

        uint32_t elapsed_ms = millis() - s_spikeStartMs;
        float    rise       = s_peakMag - s_spikeStartMag;

        if (elapsed_ms > COLLISION_CONFIRM_MS && rise > 0.0f) {
            // Fast rise = collision; slow rise = terrain undulation
            bool fast_rise = (elapsed_ms < (uint32_t)COLLISION_RISE_TIME_MS);

            if (!s_collisionFlag && fast_rise) {
                CollisionDirection dir = computeDirection(s_peakAx, s_peakAy, s_peakAz);
                // Write direction BEFORE raising flag — Core 1 reads flag first,
                // then direction; this ordering ensures direction is never stale. (BUG-2 fix)
                s_direction     = dir;
                s_collisionFlag = true;
                DBG_PRINTF("[COLLISION] Detected: dir=%d mag=%.3fg "
                              "baseline=%.4fg rise=%lums\n",
                              (int)dir, s_peakMag, s_baseline,
                              (unsigned long)elapsed_ms);
            }
        }

        // Spike over when magnitude drops below threshold
        if (mag < threshold) {
            s_inSpike = false;
        }
    }

    s_prevMag = mag;
}


void collisionDetectSetBaselineUpdate(bool allow) {
    s_baselineAllowed = allow;
}

void collisionDetectSetMultiplier(float mult) {
    taskENTER_CRITICAL(&s_baseline_mux);
    s_multiplier = mult;
    taskEXIT_CRITICAL(&s_baseline_mux);
}


bool collisionDetected() {
    return s_collisionFlag;
}


CollisionDirection collisionGetDirection() {
    return (CollisionDirection)s_direction;
}


void collisionClear() {
    s_collisionFlag = false;
    s_direction     = COLLISION_DIR_UNKNOWN;
}


void collisionSaveBaselineIfDue() {
    uint32_t now = millis();
    bool time_ok = (now - s_lastNvsSaveMs) > ((uint32_t)BASELINE_NVS_SAVE_INTERVAL_S * 1000UL);

    // Copy baseline under spinlock to get a consistent snapshot (HIGH-3 fix)
    taskENTER_CRITICAL(&s_baseline_mux);
    float bl       = s_baseline;
    float saved_bl = s_savedBaseline;
    taskEXIT_CRITICAL(&s_baseline_mux);

    bool change_ok = fabsf(bl - saved_bl) > BASELINE_NVS_MIN_CHANGE;

    if (time_ok && change_ok) {
        Preferences prefs;
        prefs.begin(NVS_NS, /*readOnly=*/false);
        prefs.putFloat(NVS_KEY, bl);
        prefs.end();
        taskENTER_CRITICAL(&s_baseline_mux);
        s_savedBaseline = bl;
        taskEXIT_CRITICAL(&s_baseline_mux);
        s_lastNvsSaveMs = now;
        DBG_PRINTF("[COLLISION] Baseline saved to NVS: %.4f g\n", bl);
    }
}


void collisionSaveBaselineForced() {
    // Copy baseline under spinlock (HIGH-3 fix)
    taskENTER_CRITICAL(&s_baseline_mux);
    float bl = s_baseline;
    taskEXIT_CRITICAL(&s_baseline_mux);

    Preferences prefs;
    prefs.begin(NVS_NS, /*readOnly=*/false);
    prefs.putFloat(NVS_KEY, bl);
    prefs.end();

    taskENTER_CRITICAL(&s_baseline_mux);
    s_savedBaseline = bl;
    taskEXIT_CRITICAL(&s_baseline_mux);
    s_lastNvsSaveMs = millis();
    DBG_PRINTF("[COLLISION] Baseline force-saved to NVS: %.4f g\n", bl);
}


float collisionGetBaseline() {
    taskENTER_CRITICAL(&s_baseline_mux);
    float bl = s_baseline;
    taskEXIT_CRITICAL(&s_baseline_mux);
    return bl;
}

float collisionGetJoltRms() {
    return s_jolt_rms;  // single volatile float — atomic-enough for diagnostics
}
