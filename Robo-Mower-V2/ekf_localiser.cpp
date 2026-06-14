// ══════════════════════════════════════════════════════════════════════════════
//  ekf_localiser.cpp — Extended Kalman Filter, 2D pose estimation
//
//  State: [x(m), y(m), θ(rad), v(m/s)]  — ENU, heading CW from North, wrapped ±π
//
//  Two update paths:
//    ekf_predict()          10 Hz  differential wheel odometry (gyro Z only during
//                                  on-the-spot pivots) + wheel speed (Core 1)
//    ekf_update_gps()      ~1 Hz   RTK GPS position ± heading (Core 0, GPS task)
//
//  All state access is protected by g_ekf_mutex (FreeRTOS mutex).
//  Written from BOTH cores (predict on Core 1, GPS update on Core 0); read via
//  ekf_get_pose() / ekf_get_speed().
//
//  Matrix arithmetic uses the local Mat4 / Mat2 types — no Eigen.
//  All arithmetic is float (ESP32-S3 hardware FPU for float).
// ══════════════════════════════════════════════════════════════════════════════

#include "ekf_localiser.h"
#include "config.h"
#include "mower_config.h"
#include "geometry.h"        // for clampf(), wrapAngle()
#include "heading_fusion.h"  // offset gate + wrap-safe EMA + compose
#include "imu.h"             // imu_get_heading_fused(), imu_is_fault()
#include <Preferences.h>     // NVS persistence of the heading offset

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cmath>
#include <cstring>

static const char* EKF_NVS_NS      = "imu";
static const char* EKF_NVS_KEY_OFF = "hdgoff";

// ─────────────────────────────────────────────────────────────────────────────
//  EKF state (protected by g_ekf_mutex)
// ─────────────────────────────────────────────────────────────────────────────

static float s_x     = 0.0f;   ///< ENU east (m)
static float s_y     = 0.0f;   ///< ENU north (m)
static float s_theta = 0.0f;   ///< Heading (rad, CW from North)
static float s_v     = 0.0f;   ///< Forward speed (m/s)
static float s_P[4][4];        ///< 4×4 state covariance matrix

/** FreeRTOS mutex protecting s_x, s_y, s_theta, s_v, s_P */
static SemaphoreHandle_t g_ekf_mutex = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
//  GPS heading tracking (written and read only from Core 0 with mutex held)
// ─────────────────────────────────────────────────────────────────────────────

/** Steering-centre position of the last GPS fix used to initialise/update heading */
static float s_prev_hdg_east  = 0.0f;
static float s_prev_hdg_north = 0.0f;
/** True once the first RTK-quality fix (fix_type >= 4) has been stored */
static bool  s_prev_hdg_valid = false;
/** Total |heading change| (rad) accumulated since the heading reference above was
 *  set. Summed in ekf_predict() from the per-tick yaw rate; reset whenever the
 *  reference moves. Used to tell whether the travel since the reference was
 *  STRAIGHT — only then is the GPS chord a valid absolute heading to lock onto. */
static float s_hdg_turn_accum = 0.0f;

/** GPS-trimmed heading offset: s_theta = imu_get_heading_fused() + s_hdg_offset.
 *  Loaded from NVS at ekf_init(); trimmed in ekf_update_gps(); persisted by
 *  ekf_save_heading_offset_if_due(). Survives RESETEKF (mounting/declination). */
static float s_hdg_offset      = 0.0f;
/** Previous BNO heading, for per-tick turn accumulation. */
static float s_prev_bno_hdg    = 0.0f;
static bool  s_prev_bno_valid  = false;
/** Last value/time persisted to NVS (throttle). */
static float s_hdg_offset_saved = 0.0f;
static uint32_t s_hdg_offset_save_ms = 0;

/** False until the first valid GPS update has been used to seed the EKF position.
 *  On boot the EKF starts at (0,0) which may be many metres from the actual ENU
 *  position if an NVS origin is already set. The innovation gate is bypassed for
 *  this one cold-start update so the EKF teleports to the GPS position rather than
 *  rejecting every update until the user physically returns to the origin point. */
static bool  s_gps_seeded = false;

/** Per-axis position variance from the last GPS fix (= sigma²).
 *  Set by ekf_update_gps(); held constant in ekf_predict() so reported
 *  uncertainty tracks GPS accuracy rather than growing between fixes. */
static float s_gps_p_ceil = EKF_P_MAX_POS;

// ─────────────────────────────────────────────────────────────────────────────
//  GPS heading-event publication (for AUTO bootstrap + odo_calib)
// ─────────────────────────────────────────────────────────────────────────────

/** True once a GPS heading correction has been applied at least once since boot
 *  (or RESETEKF). Heading is then carried by odometry through pivots. */
static bool     s_heading_established = false;

/** Monotonic counter + the latest GPS-derived FRONT-FACING travel heading and
 *  the steering-centre ENU position at that fix. Consumed by odo_calib to detect
 *  new straight/turn intervals. The heading is reverse-corrected (front-facing). */
static uint32_t s_gps_hdg_seq   = 0;
static float    s_gps_hdg_theta = 0.0f;
static float    s_gps_hdg_east  = 0.0f;
static float    s_gps_hdg_north = 0.0f;


// ══════════════════════════════════════════════════════════════════════════════
//  Mat4 implementation
// ══════════════════════════════════════════════════════════════════════════════

Mat4::Mat4() {
    memset(m, 0, sizeof(m));
    m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.0f;
}

Mat4 Mat4::operator*(const Mat4 &b) const {
    Mat4 r;
    memset(r.m, 0, sizeof(r.m));  // zero before accumulation (constructor sets identity)
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                r.m[i][j] += m[i][k] * b.m[k][j];
    return r;
}

Mat4 Mat4::operator+(const Mat4 &b) const {
    Mat4 r;
    memset(r.m, 0, sizeof(r.m));
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            r.m[i][j] = m[i][j] + b.m[i][j];
    return r;
}

Mat4 Mat4::transpose() const {
    Mat4 r;
    memset(r.m, 0, sizeof(r.m));
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            r.m[i][j] = m[j][i];
    return r;
}

/** Helper: 3×3 determinant from 9 individual floats (row-major order) */
static float det3(float a00, float a01, float a02,
                  float a10, float a11, float a12,
                  float a20, float a21, float a22) {
    return a00 * (a11*a22 - a12*a21)
         - a01 * (a10*a22 - a12*a20)
         + a02 * (a10*a21 - a11*a20);
}

Mat4 Mat4::inverse() const {
    // Build cofactor matrix C[i][j] = (-1)^{i+j} * det(3×3 minor omitting row i, col j)
    float C[4][4];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            // Extract the 3×3 minor by removing row i and column j
            float sub[3][3];
            int ri = 0;
            for (int r = 0; r < 4; r++) {
                if (r == i) continue;
                int ci = 0;
                for (int c = 0; c < 4; c++) {
                    if (c == j) continue;
                    sub[ri][ci++] = m[r][c];
                }
                ri++;
            }
            float sign = ((i + j) % 2 == 0) ? 1.0f : -1.0f;
            C[i][j] = sign * det3(sub[0][0], sub[0][1], sub[0][2],
                                  sub[1][0], sub[1][1], sub[1][2],
                                  sub[2][0], sub[2][1], sub[2][2]);
        }
    }

    // Determinant via first-row expansion
    float det = 0.0f;
    for (int j = 0; j < 4; j++)
        det += m[0][j] * C[0][j];

    // Guard against singular matrix (P should always be positive-definite)
    if (fabsf(det) < 1e-9f) {
        return Mat4();  // identity as safe fallback
    }

    // Inverse = adjugate / det  (adjugate = transpose of cofactor matrix)
    Mat4 inv;
    memset(inv.m, 0, sizeof(inv.m));
    float inv_det = 1.0f / det;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            inv.m[i][j] = C[j][i] * inv_det;  // note: C[j][i] = adjugate[i][j]

    return inv;
}


// ══════════════════════════════════════════════════════════════════════════════
//  Mat2 implementation
// ══════════════════════════════════════════════════════════════════════════════

Mat2 mat2_inverse(const Mat2 &mat) {
    float det = mat.m[0][0] * mat.m[1][1] - mat.m[0][1] * mat.m[1][0];
    Mat2 inv;
    if (fabsf(det) < 1e-9f) {
        // Degenerate: return identity (should not occur with valid GPS noise R > 0)
        inv.m[0][0] = 1.0f; inv.m[0][1] = 0.0f;
        inv.m[1][0] = 0.0f; inv.m[1][1] = 1.0f;
        return inv;
    }
    float inv_det = 1.0f / det;
    inv.m[0][0] =  mat.m[1][1] * inv_det;
    inv.m[0][1] = -mat.m[0][1] * inv_det;
    inv.m[1][0] = -mat.m[1][0] * inv_det;
    inv.m[1][1] =  mat.m[0][0] * inv_det;
    return inv;
}


// ══════════════════════════════════════════════════════════════════════════════
//  Internal helpers
// ══════════════════════════════════════════════════════════════════════════════

/** Set P = diag(1.0, 1.0, 0.5, 0.1) — caller must hold mutex */
static void reset_covariance_locked() {
    memset(s_P, 0, sizeof(s_P));
    s_P[0][0] = 1.0f;   // x variance (m²)
    s_P[1][1] = 1.0f;   // y variance (m²)
    s_P[2][2] = 0.5f;   // heading variance (rad²)
    s_P[3][3] = 0.1f;   // velocity variance ((m/s)²)
}


// ══════════════════════════════════════════════════════════════════════════════
//  EKF public API
// ══════════════════════════════════════════════════════════════════════════════

void ekf_init() {
    if (g_ekf_mutex == nullptr) {
        g_ekf_mutex = xSemaphoreCreateMutex();
    }

    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);

    s_x     = 0.0f;
    s_y     = 0.0f;
    s_theta = 0.0f;
    s_v     = 0.0f;
    reset_covariance_locked();

    s_prev_hdg_valid = false;
    s_prev_hdg_east  = 0.0f;
    s_prev_hdg_north = 0.0f;
    s_hdg_turn_accum = 0.0f;
    s_gps_seeded     = false;

    // Load the persisted heading offset (mounting + declination). NOT reset by
    // RESETEKF — it is a physical constant, only re-trimmed by GPS.
    {
        Preferences p;
        p.begin(EKF_NVS_NS, /*readOnly=*/true);
        s_hdg_offset = p.getFloat(EKF_NVS_KEY_OFF, 0.0f);
        p.end();
        s_hdg_offset_saved = s_hdg_offset;
        s_prev_bno_valid   = false;
    }

    s_heading_established = false;
    s_gps_hdg_seq         = 0;

    xSemaphoreGive(g_ekf_mutex);
}

// ─────────────────────────────────────────────────────────────────────────────

bool ekf_is_seeded() {
    return s_gps_seeded;  // atomic bool read — no mutex needed
}

// ─────────────────────────────────────────────────────────────────────────────

void ekf_predict(float v_left, float v_right, float dt) {
    float v = 0.5f * (v_left + v_right);

    // Read BNO heading outside the EKF mutex (driver uses its own spinlock).
    float bno      = imu_get_heading_fused();
    bool  bno_ok   = !imu_is_fault();

    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);

    if (!s_gps_seeded) {           // position (0,0) meaningless until first GPS
        xSemaphoreGive(g_ekf_mutex);
        return;
    }

    // ── Heading from BNO absolute fusion + GPS-trimmed offset ────────────────
    if (bno_ok) {
        if (s_prev_bno_valid) {
            s_hdg_turn_accum += fabsf(wrapAngle(bno - s_prev_bno_hdg));
        }
        s_prev_bno_hdg   = bno;
        s_prev_bno_valid = true;
        s_theta = heading_compose(bno, s_hdg_offset);
    } else {
        // BNO faulted — hold s_theta, do not accrue turn (AUTO pauses). Drop the
        // previous-sample reference so turn-accum restarts cleanly on recovery
        // (avoids a spurious lump measured across the fault gap).
        s_prev_bno_valid = false;
    }

    // ── Position dead-reckoning ──────────────────────────────────────────────
    float theta = s_theta;
    float sth   = sinf(theta);
    float cth   = cosf(theta);
    s_x += v * sth * dt;
    s_y += v * cth * dt;
    s_v  = v;

    // ── Covariance ───────────────────────────────────────────────────────────
    float q_scale = (fabsf(v) > 0.01f) ? 1.0f : EKF_Q_NO_DR_SCALE;
    if (bno_ok) {
        s_P[2][2] = EKF_HDG_VAR_BNO;   // absolute heading is good
    } else {
        s_P[2][2] = fminf(s_P[2][2] + EKF_Q_HDG * dt, EKF_P_MAX_HDG);
    }
    s_P[3][3] = fminf(s_P[3][3] + EKF_Q_VEL * dt * q_scale, EKF_P_MAX_VEL);
    if (!isfinite(s_P[2][2]) || s_P[2][2] < 0.0f) s_P[2][2] = EKF_P_MAX_HDG;
    if (!isfinite(s_P[3][3]) || s_P[3][3] < 0.0f) s_P[3][3] = EKF_P_MAX_VEL;

    xSemaphoreGive(g_ekf_mutex);
}

// ─────────────────────────────────────────────────────────────────────────────

void ekf_update_gps(float gps_east, float gps_north, int fix_type,
                     float hdop, float dif_age_s, float heading_rad) {
    // ── Transform antenna position to steering-centre position ───────────────
    // The GPS antenna is offset from the steering centre:
    //   sc = antenna - forward_offset * heading_unit - right_offset * right_unit
    // heading unit (CW from North): (sin θ, cos θ) in ENU (east, north)
    // right unit: (cos θ, -sin θ) in ENU
    const MowerConfig &mc = mower_config_get();
    float sc_east  = gps_east  - mc.antenna_fwd_m * sinf(heading_rad)
                                - mc.antenna_right_m * cosf(heading_rad);
    float sc_north = gps_north - mc.antenna_fwd_m * cosf(heading_rad)
                               + mc.antenna_right_m * sinf(heading_rad);

    // ── Measurement noise R (diagonal, 2×2) ─────────────────────────────────
    // sigma is computed continuously from three inputs:
    //   fix_type  — baseline noise floor per fix quality
    //   HDOP      — satellite geometry (1.0 ideal, higher = worse)
    //   dif_age_s — stale NTRIP correction penalty (+5% per second beyond 5 s)
    // This avoids binary jumps between RTK fixed and float; float at HDOP=1
    // and fresh corrections gives ~5 cm accuracy, which is nearly as good as fixed.
    // Base values match the PWA gpsAccLabel formula: acc = base × HDOP × √2
    // so that ekf_get_position_uncertainty() and the WebUI display the same number.
    float sigma_base;
    switch (fix_type) {
        case 4:  sigma_base = 0.01f;  break;  // RTK fixed  ~1 cm
        case 5:  sigma_base = 0.15f;  break;  // RTK float  ~15 cm
        case 2:  sigma_base = 0.50f;  break;  // DGPS       ~50 cm
        default: sigma_base = 1.00f;  break;  // GPS/none   ~100 cm
    }
    float sigma = sigma_base * fmaxf(hdop, 1.0f) * 1.41421356f;  // ×√2 = 2D RMS
    if (fix_type >= 4 && dif_age_s > 5.0f) {
        sigma *= (1.0f + (dif_age_s - 5.0f) * 0.05f);
    }
    sigma = clampf(sigma, 0.005f, 5.0f);
    float r_val = sigma * sigma;

    // BNO heading read OUTSIDE the EKF mutex (the driver uses its own spinlock).
    // Used below for the heading-offset trim; on a gate-enforced STRAIGHT segment
    // (turn < HEADING_STRAIGHT_MAX_TURN_RAD ≈ 11°) this end-of-segment sample is a
    // bounded approximation of the chord-average BNO heading.
    float bno_hdg = imu_get_heading_fused();

    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);

    // ── Innovation gate ───────────────────────────────────────────────────────
    // 5 m minimum: catches genuine multipath / NTRIP jumps while allowing the
    // EKF to recover if dead-reckoning has drifted several metres from GPS
    // (e.g. after a cold-start with a stale heading, or a long GPS outage).
    float innov_e    = sc_east  - s_x;
    float innov_n    = sc_north - s_y;
    float innov_norm = sqrtf(innov_e*innov_e + innov_n*innov_n);
    float gate_m     = fmaxf(5.0f * sigma, 5.0f);

    if (!s_gps_seeded) {
        s_gps_seeded = true;
        DBG_PRINTF("[EKF] GPS cold-start seed: (%.2f, %.2f) m  sigma=%.3fm\n",
                   sc_east, sc_north, sigma);
        // Fall through — cold start bypasses gate, snaps below
    } else if (innov_norm > gate_m) {
        DBG_PRINTF("[EKF] GPS innovation %.2fm > gate %.2fm (sigma=%.3fm) — skipping\n",
                      innov_norm, gate_m, sigma);
        xSemaphoreGive(g_ekf_mutex);
        return;
    }

    // ── Position snap ─────────────────────────────────────────────────────────
    // GPS provides absolute position at 1 Hz; wheel odometry dead-reckons
    // between fixes.  Snap directly rather than blending — GPS is more accurate
    // than odometry over any interval that matters for a lawnmower.
    s_x = sc_east;
    s_y = sc_north;

    // ── GPS ceiling: force position covariance to GPS accuracy ───────────────
    // Zero all cross-terms involving position so ekf_predict() cannot build
    // them beyond the position diagonal (which would violate PSD).
    s_gps_p_ceil  = r_val;
    s_P[0][0] = r_val;  s_P[1][1] = r_val;
    s_P[0][1] = s_P[1][0] = 0.0f;
    s_P[0][2] = s_P[2][0] = 0.0f;
    s_P[0][3] = s_P[3][0] = 0.0f;
    s_P[1][2] = s_P[2][1] = 0.0f;
    s_P[1][3] = s_P[3][1] = 0.0f;

    // ── GPS heading-offset trim ──────────────────────────────────────────────
    // GPS travel direction is the only absolute heading truth. On a STRAIGHT,
    // MEASURABLE RTK segment the chord IS the heading, so trim the offset toward
    // (chord − BNO heading) by a slow EMA. Otherwise hold the offset (robust to
    // GPS outages). No blend into s_theta — heading comes from the BNO.
    if (fix_type >= 4) {
        if (s_prev_hdg_valid) {
            float dE   = sc_east  - s_prev_hdg_east;
            float dN   = sc_north - s_prev_hdg_north;
            float dist = sqrtf(dE*dE + dN*dN);
            bool turned = (s_hdg_turn_accum > HEADING_STRAIGHT_MAX_TURN_RAD);

            if (turned) {
                // Pivot/corner since the reference — restart the straight segment.
                s_prev_hdg_east  = sc_east;
                s_prev_hdg_north = sc_north;
                s_hdg_turn_accum = 0.0f;
            } else if (heading_offset_segment_qualifies(
                           s_hdg_turn_accum, dist, sigma,
                           HEADING_STRAIGHT_MAX_TURN_RAD,
                           HEADING_FROM_GPS_MIN_DIST_M,
                           HEADING_GPS_DIST_SIGMA_K)) {
                float z_hdg = atan2f(dE, dN);   // travel dir, 0=N, CW+
                if (isfinite(z_hdg)) {
                    // Reverse correction: chassis FRONT is 180° opposite when
                    // reversing (s_v < 0).
                    if (s_v < -0.03f) z_hdg = wrapAngle(z_hdg + (float)M_PI);

                    s_hdg_offset = heading_offset_ema(s_hdg_offset, z_hdg, bno_hdg,
                                                      HEADING_OFFSET_TRIM_GAIN);
                    s_heading_established = true;

                    // Publish FRONT-facing heading event for odo_calib.
                    s_gps_hdg_seq++;
                    s_gps_hdg_theta = z_hdg;
                    s_gps_hdg_east  = sc_east;
                    s_gps_hdg_north = sc_north;
                }
                s_prev_hdg_east  = sc_east;
                s_prev_hdg_north = sc_north;
                s_hdg_turn_accum = 0.0f;
            }
            // else: straight but too short — hold reference, keep accumulating.
        } else {
            s_prev_hdg_east  = sc_east;
            s_prev_hdg_north = sc_north;
            s_prev_hdg_valid = true;
            s_hdg_turn_accum = 0.0f;
        }
    }

    xSemaphoreGive(g_ekf_mutex);
}

// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────

Pose2D ekf_get_pose() {
    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);
    Pose2D pose{ s_x, s_y, s_theta };
    xSemaphoreGive(g_ekf_mutex);
    return pose;
}

float ekf_get_speed() {
    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);
    float v = s_v;
    xSemaphoreGive(g_ekf_mutex);
    return v;
}

float ekf_get_heading() {
    // Guard for calls before ekf_init() (e.g. from rtk_gps.cpp during early boot)
    if (g_ekf_mutex == nullptr) return 0.0f;
    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);
    float theta = s_theta;
    xSemaphoreGive(g_ekf_mutex);
    return theta;
}

float ekf_get_position_uncertainty() {
    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);
    float p_ceil = s_gps_p_ceil;
    xSemaphoreGive(g_ekf_mutex);
    // Return GPS sigma directly — position IS the GPS snap, so EKF accuracy
    // equals GPS accuracy.  s_gps_p_ceil = sigma², so sqrt gives sigma (metres).
    return sqrtf(p_ceil > 0.0f ? p_ceil : 1.0f);
}

bool ekf_heading_is_established() {
    return s_heading_established;  // single bool — atomic read, no mutex needed
}

bool ekf_get_gps_heading_event(uint32_t *seq, float *theta,
                               float *east, float *north) {
    if (g_ekf_mutex == nullptr) return false;
    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);
    uint32_t s = s_gps_hdg_seq;
    if (seq)   *seq   = s;
    if (theta) *theta = s_gps_hdg_theta;
    if (east)  *east  = s_gps_hdg_east;
    if (north) *north = s_gps_hdg_north;
    xSemaphoreGive(g_ekf_mutex);
    return (s > 0);  // false until the first heading event has been published
}

float ekf_get_heading_uncertainty() {
    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);
    float p = s_P[2][2];
    xSemaphoreGive(g_ekf_mutex);
    return sqrtf(p > 0.0f ? p : 0.0f);
}

float ekf_get_heading_offset() {
    if (g_ekf_mutex == nullptr) return 0.0f;
    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);
    float o = s_hdg_offset;
    xSemaphoreGive(g_ekf_mutex);
    return o;
}

void ekf_save_heading_offset_if_due() {
    // Called only from the single 10 Hz EKF hook (Core 1), so the throttle
    // timestamp s_hdg_offset_save_ms needs no mutex; s_hdg_offset itself is
    // copied under the mutex below.
    if (g_ekf_mutex == nullptr) return;
    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);
    float off   = s_hdg_offset;
    float saved = s_hdg_offset_saved;
    xSemaphoreGive(g_ekf_mutex);

    uint32_t now = millis();
    bool time_ok   = (now - s_hdg_offset_save_ms) > HEADING_OFFSET_SAVE_MIN_INTERVAL_MS;
    bool change_ok = fabsf(wrapAngle(off - saved)) > HEADING_OFFSET_SAVE_MIN_CHANGE_RAD;
    if (time_ok && change_ok) {
        Preferences p;
        p.begin(EKF_NVS_NS, /*readOnly=*/false);
        p.putFloat(EKF_NVS_KEY_OFF, off);
        p.end();
        s_hdg_offset_save_ms = now;
        xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);
        s_hdg_offset_saved = off;
        xSemaphoreGive(g_ekf_mutex);
        DBG_PRINTF("[EKF] heading offset saved: %.4f rad\n", off);
    }
}

void ekf_reset_covariance() {
    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);
    reset_covariance_locked();
    // Force a fresh GPS heading establishment after an external reposition
    // (RESETEKF): drop the established flag and the stale heading reference.
    s_heading_established = false;
    s_prev_hdg_valid      = false;
    xSemaphoreGive(g_ekf_mutex);
}

void ekf_dump_state() {
    // Copy state under mutex; print outside to minimise lock hold time
    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);
    float x     = s_x;
    float y     = s_y;
    float theta = s_theta;
    float v     = s_v;
    float P[4][4];
    memcpy(P, s_P, sizeof(P));
    xSemaphoreGive(g_ekf_mutex);

    DBG_PRINTLN("=== EKF State ===");
    DBG_PRINTF("  x=%.4f m  y=%.4f m  theta=%.4f rad  v=%.4f m/s\n",
                  x, y, theta, v);
    DBG_PRINTLN("  P:");
    for (int i = 0; i < 4; i++) {
        DBG_PRINTF("    [ %10.6f  %10.6f  %10.6f  %10.6f ]\n",
                      P[i][0], P[i][1], P[i][2], P[i][3]);
    }
    DBG_PRINTLN("=================");
}
