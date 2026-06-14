#pragma once
#include <Arduino.h>
#include "geometry.h"  // for wrapAngle()

// ══════════════════════════════════════════════════════════════════════════════
//  ekf_localiser.h — Extended Kalman Filter for 2D pose estimation
//
//  State vector: [x (m), y (m), θ (rad), v (m/s)]
//    x, y  : ENU east and north from GPS origin
//    θ     : Heading, 0 = North/+Y, clockwise positive, wrapped to ±π
//    v     : Forward speed (m/s)
//
//  Sensor fusion:
//    ekf_predict()          — Differential wheel odometry    @10 Hz   Core 1
//    ekf_update_gps()       — RTK GPS position + heading     @1 Hz    Core 0
//    ekf_get_pose()         — Read-only access               any rate Core 1
//
//  Thread safety: FreeRTOS mutex (g_ekf_mutex) protects all state access.
//    Core 0 writes, Core 1 reads.
// ══════════════════════════════════════════════════════════════════════════════


// ─────────────────────────────────────────────────────────────────────────────
//  Pose2D  — 2D pose of the robot steering centre in ENU coordinates
// ─────────────────────────────────────────────────────────────────────────────

/** 2D pose of the robot steering centre in ENU coordinates */
struct Pose2D {
    float x;        ///< ENU east (m), from GPS origin
    float y;        ///< ENU north (m), from GPS origin
    float heading;  ///< Heading (rad, 0=North/+Y, CW positive, wrapped ±π)
};


// ─────────────────────────────────────────────────────────────────────────────
//  Mat4  — minimal 4×4 float matrix for EKF covariance propagation
//
//  All operations use float — ESP32-S3 has hardware FPU for single precision.
//  No external matrix library (no Eigen, no arm_math).
// ─────────────────────────────────────────────────────────────────────────────

/** Minimal 4×4 float matrix. Default constructor initialises to identity. */
struct Mat4 {
    float m[4][4];

    /** Construct identity matrix */
    Mat4();

    /** Matrix multiply: (this) × b */
    Mat4 operator*(const Mat4 &b) const;

    /** Element-wise add: (this) + b */
    Mat4 operator+(const Mat4 &b) const;

    /** Transpose */
    Mat4 transpose() const;

    /**
     * @brief 4×4 matrix inverse via cofactor / adjugate method.
     *
     * Computes the 16 cofactors (each a signed 3×3 determinant), forms the
     * adjugate as the transposed cofactor matrix, and divides by the
     * determinant. Returns identity if the determinant is < 1e-9 (singular
     * guard — in practice the EKF P matrix is always positive-definite).
     *
     * This function exists to satisfy the module specification. The EKF update
     * path uses mat2_inverse() for the 2×2 innovation covariance S, which is
     * both cheaper and more numerically stable.
     */
    Mat4 inverse() const;

    /** Element access (mutable) */
    float& at(int r, int c) { return m[r][c]; }

    /** Element access (const) */
    const float& at(int r, int c) const { return m[r][c]; }
};


// ─────────────────────────────────────────────────────────────────────────────
//  Mat2  — 2×2 float matrix for Kalman gain (innovation covariance S)
// ─────────────────────────────────────────────────────────────────────────────

/** 2×2 float matrix, used for the 2×2 innovation covariance S in GPS update */
struct Mat2 {
    float m[2][2];
};

/**
 * @brief 2×2 matrix inverse.
 *
 * Uses the standard scalar formula: inv = (1/det) * [[d,-b],[-c,a]].
 * Returns identity if det < 1e-9 (degenerate guard).
 *
 * @param mat  Input 2×2 matrix.
 * @return Inverse of mat, or identity if singular.
 */
Mat2 mat2_inverse(const Mat2 &mat);


// ─────────────────────────────────────────────────────────────────────────────
//  EKF public API
// ─────────────────────────────────────────────────────────────────────────────

/**
 * @brief Initialise EKF state to origin with high initial covariance.
 *
 * Sets state [x, y, θ, v] = [0, 0, 0, 0] and P = diag(1.0, 1.0, 0.5, 0.1).
 * Creates the FreeRTOS mutex on first call.
 * Call from setup() after IMU bias is collected and GPS origin is established.
 */
void ekf_init();

/**
 * @brief EKF prediction step. Call at 10 Hz from Core 1.
 *
 * Heading is taken directly from the BNO055 absolute fusion plus the
 * GPS-trimmed offset (imu_get_heading_fused() + offset) — no integration. If the
 * BNO has faulted (imu_is_fault()), heading is held (AUTO pauses elsewhere).
 * Position dead-reckons: dx = v·sinθ·dt, dy = v·cosθ·dt.
 *
 * @param v_left   Left wheel surface velocity (m/s).
 * @param v_right  Right wheel surface velocity (m/s).
 * @param dt       Time step (seconds).
 */
void ekf_predict(float v_left, float v_right, float dt);

/**
 * @brief EKF GPS position (and optional heading) update. Call at 1 Hz from Core 0.
 *
 * Computes GPS measurement noise sigma continuously from fix_type, HDOP, and
 * correction age (dif_age_s). Innovation gate is dynamic (5×sigma, min 5 m) so
 * the filter follows GPS closely at high accuracy and relaxes on degraded fixes.
 *
 * @param gps_east    GPS antenna ENU east (m) — antenna position, not steering centre.
 * @param gps_north   GPS antenna ENU north (m).
 * @param fix_type    NMEA GGA fix quality: 4=RTK fixed, 5=RTK float, 1=autonomous.
 * @param hdop        Horizontal dilution of precision from GGA (≥1.0).
 * @param dif_age_s   Seconds since last RTCM/NTRIP correction was applied.
 *                    Increases sigma when corrections are stale.
 * @param heading_rad Current EKF heading (rad), used for antenna offset rotation.
 */
void ekf_update_gps(float gps_east, float gps_north, int fix_type,
                     float hdop, float dif_age_s, float heading_rad);


/**
 * @brief Get current pose estimate. Thread-safe (acquires mutex).
 *
 * Safe to call from Core 1. Returns a copy — the returned struct is not live.
 *
 * @return Copy of current [x, y, heading] state.
 */
Pose2D ekf_get_pose();

/**
 * @brief Get current forward speed estimate (m/s). Thread-safe (acquires mutex).
 *
 * @return Current speed in m/s (negative = reversing).
 */
float ekf_get_speed();

/**
 * @brief Get current heading estimate (rad). Thread-safe (acquires mutex).
 *
 * Convenience accessor used by rtk_gps.cpp to apply the antenna offset before
 * calling ekf_update_gps(). Returns 0.0f safely before ekf_init() is called
 * (mutex is nullptr; the function guards against this).
 *
 * @return Current heading in rad (0=North/+Y, CW positive, wrapped ±π).
 */
float ekf_get_heading();

/**
 * @brief Get scalar position uncertainty (m). Thread-safe (acquires mutex).
 *
 * Computed as sqrt(P[0][0] + P[1][1]) — RMS position error
 * (sqrt of sum of x and y variances).
 *
 * @return Position uncertainty in metres.
 */
float ekf_get_position_uncertainty();

/**
 * @brief Reset covariance matrix to high initial values. Thread-safe.
 *
 * Sets P = diag(1.0, 1.0, 0.5, 0.1) without resetting the state estimate.
 * Call after the robot has been repositioned externally (e.g., lifted and
 * moved by the operator) so the filter rapidly accepts new GPS measurements.
 */
void ekf_reset_covariance();

/**
 * @brief True once a GPS heading-offset trim has been applied at least once
 *        since boot (or RESETEKF) — i.e. the BNO heading has been GPS-referenced.
 *
 * Heading itself comes from the BNO055 absolute fusion + GPS-trimmed offset; this
 * flag only records that the offset has been GPS-trimmed at least once. Currently
 * unused by callers (the old AUTO bootstrap that consumed it was removed) — kept
 * for diagnostics. Thread-safe: single bool — atomic read.
 */
bool ekf_heading_is_established();

/**
 * @brief Latest GPS-derived FRONT-FACING heading event (for odo_calib).
 *
 * Each clean straight-RTK heading fix publishes a monotonically increasing seq
 * together with the front-facing travel heading (reverse-corrected) and the
 * steering-centre ENU position at that fix. A consumer detects a new event by a
 * changed seq.
 *
 * @param[out] seq    Monotonic event counter (0 = none yet).
 * @param[out] theta  Front-facing heading (rad, 0=North, CW+, ±π).
 * @param[out] east   Steering-centre ENU east at the fix (m).
 * @param[out] north  Steering-centre ENU north at the fix (m).
 * @return true if at least one event has been published.
 */
bool ekf_get_gps_heading_event(uint32_t *seq, float *theta,
                               float *east, float *north);

/**
 * @brief Heading 1-sigma uncertainty (rad) = sqrt(P[2][2]). Thread-safe.
 */
float ekf_get_heading_uncertainty();

/** Current GPS-trimmed heading offset (rad) = heading − BNO_fused. Thread-safe. */
float ekf_get_heading_offset();

/** Persist the heading offset to NVS if it has changed enough and the throttle
 *  interval has elapsed. Call from the 10 Hz hook (Core 1). */
void ekf_save_heading_offset_if_due();

/**
 * @brief Returns true once the EKF has received its first valid GPS fix.
 *
 * Before this, the EKF position is (0, 0) which maps to the GPS ENU origin
 * and must NOT be displayed as the mower's position. After seeding, the EKF
 * tracks the actual rover position in ENU space.
 *
 * Thread-safe: s_gps_seeded is a single bool — atomic read, no mutex needed.
 */
bool ekf_is_seeded();

/**
 * @brief Dump EKF state vector and full covariance matrix to Serial.
 *
 * Output format:
 *   === EKF State ===
 *   x=%.4f m  y=%.4f m  θ=%.4f rad  v=%.4f m/s
 *   P:
 *   [ row0 ]
 *   [ row1 ]
 *   [ row2 ]
 *   [ row3 ]
 */
void ekf_dump_state();
