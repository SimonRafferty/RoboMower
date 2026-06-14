// ══════════════════════════════════════════════════════════════════════════════
//  odo_calib.cpp — GPS-referenced drive-odometry self-calibration
//
//  See odo_calib.h for the public API and the observability argument.
//
//  Mechanism (event-driven, runs on Core 1 at 10 Hz):
//    • Every tick we integrate, since the last GPS heading event:
//        d_odo  = Σ |½(vL+vR)| · dt        — odometry path length (scaled m)
//        s_diff = Σ  (vL − vR)  · dt        — differential integral (scaled m)
//    • The EKF emits a "heading event" (monotonic seq + travel heading + ENU
//      position) every time it gets a clean straight RTK segment > 0.30 m.
//      When a NEW event arrives we look at the interval just completed:
//        Δθ = wrap(θ_now − θ_prev),  d_gps = |ENU_now − ENU_prev|
//        – nearly straight (|Δθ|<10°): accumulate d_gps / d_odo; once ≥ 3 m,
//          ratio = d_gps/d_odo → nudge `scale` (d_odo is already ×scale, so the
//          ratio self-references: it converges to 1 as scale → truth).
//        – a real turn  (|Δθ|>25°): track = s_diff/Δθ  (straights add ~0 to
//          s_diff; the pivot dominates) → nudge `track_m`.
//        – 10–25°: ambiguous, discard.
//    • Each nudge is EMA-smoothed and clamped to ≤2 %/update and to hard bounds.
//      NVS writes are rate-limited; a CAL line is pushed to the system log.
// ══════════════════════════════════════════════════════════════════════════════

#include "odo_calib.h"
#include "config.h"      // TRACK_WIDTH_M default
#include "ekf_localiser.h"
#include "mower_config.h"
#include "geometry.h"    // wrapAngle(), clampf()
#include "sys_log.h"
#include <Preferences.h>
#include <math.h>

// ── NVS ───────────────────────────────────────────────────────────────────────
static const char *NVS_NS        = "odocal";
static const char *NVS_KEY_SCALE = "scale";
static const char *NVS_KEY_TRACK = "track";

// ── Tunables ──────────────────────────────────────────────────────────────────
static const float SCALE_MIN        = 0.50f;   // hard bounds on the distance scale
static const float SCALE_MAX        = 2.00f;
static const float TRACK_MIN_FRAC   = 0.50f;   // track bounds as a fraction of nominal
static const float TRACK_MAX_FRAC   = 1.50f;
static const float STRAIGHT_MAX_RAD = 10.0f * (float)M_PI / 180.0f;  // "straight" interval
static const float TURN_MIN_RAD     = 25.0f * (float)M_PI / 180.0f;  // "turn" interval
static const float DIST_TARGET_M    = 3.0f;    // straight distance to accumulate per scale update
static const float EMA_ALPHA        = 0.25f;   // smoothing gain
static const float MAX_STEP_FRAC    = 0.02f;   // ≤2 % change per update
static const float INTERVAL_MAX_M   = 10.0f;   // discard intervals with more odometry than this (stale prev)
static const float RATIO_OUTLIER_LO = 0.30f;   // discard d_gps/d_odo outside this band
static const float RATIO_OUTLIER_HI = 3.00f;
static const uint32_t NVS_SAVE_MS   = 20000;   // min interval between NVS writes
static const float NVS_MIN_CHANGE   = 0.005f;  // 0.5 % relative change to bother saving
static const uint32_t REPORT_MS     = 30000;   // min interval between CAL log lines

// ── Calibrated values (Core 1 owned; 32-bit aligned float reads are atomic) ───
static volatile float s_scale   = 1.0f;
static volatile float s_track_m = TRACK_WIDTH_M;
static float s_nominal_track    = TRACK_WIDTH_M;

// ── Per-interval accumulators (since last GPS heading event) ──────────────────
static double s_acc_dodo  = 0.0;   // odometry path length (m)
static double s_acc_sdiff = 0.0;   // differential integral Σ(vL−vR)·dt (m)

// ── Straight-run accumulators (across consecutive straight intervals) ─────────
static double s_straight_dgps = 0.0;
static double s_straight_dodo = 0.0;

// ── GPS heading-event tracking ────────────────────────────────────────────────
static uint32_t s_last_seq    = 0;      // last EKF event seq processed
static bool     s_have_prev   = false;  // a previous event is on record
static float    s_prev_theta  = 0.0f;
static float    s_prev_east   = 0.0f;
static float    s_prev_north  = 0.0f;

// ── Bookkeeping ───────────────────────────────────────────────────────────────
static uint32_t s_last_save_ms   = 0;
static float    s_saved_scale    = 1.0f;
static float    s_saved_track    = TRACK_WIDTH_M;
static uint32_t s_last_report_ms = 0;
static bool     s_dirty          = false;   // unsaved change pending


static void reset_accumulators() {
    s_acc_dodo  = 0.0;
    s_acc_sdiff = 0.0;
}

static void reset_straight() {
    s_straight_dgps = 0.0;
    s_straight_dodo = 0.0;
}


void odo_calib_init() {
    s_nominal_track = mower_config_track_width_m();

    Preferences prefs;
    prefs.begin(NVS_NS, /*readOnly=*/true);
    float scale = prefs.getFloat(NVS_KEY_SCALE, 1.0f);
    float track = prefs.getFloat(NVS_KEY_TRACK, s_nominal_track);
    prefs.end();

    s_scale   = clampf(scale, SCALE_MIN, SCALE_MAX);
    s_track_m = clampf(track, s_nominal_track * TRACK_MIN_FRAC,
                              s_nominal_track * TRACK_MAX_FRAC);
    s_saved_scale = s_scale;
    s_saved_track = s_track_m;

    reset_accumulators();
    reset_straight();
    s_last_seq  = 0;
    s_have_prev = false;
    s_dirty     = false;

    char line[SYS_LOG_MAX_LEN];
    snprintf(line, sizeof(line), "ODOCAL load x%.3f track %.0fmm (nom %.0f)",
             (double)s_scale, (double)(s_track_m * 1000.0f),
             (double)(s_nominal_track * 1000.0f));
    sys_log_push(line);
}


// Push a human-readable calibration line to the system log (PWA), throttled.
static void report(bool force) {
    uint32_t now = millis();
    if (!force && (now - s_last_report_ms) < REPORT_MS) return;
    s_last_report_ms = now;

    float wheel_r = mower_config_get().wheel_radius_m;       // nominal radius (m)
    float eff_d_mm = 2.0f * wheel_r * s_scale * 1000.0f;     // implied effective Ø
    char line[SYS_LOG_MAX_LEN];
    snprintf(line, sizeof(line), "CAL scale %.3f wheelD %.0fmm track %.0fmm(nom%.0f)",
             (double)s_scale, (double)eff_d_mm,
             (double)(s_track_m * 1000.0f), (double)(s_nominal_track * 1000.0f));
    sys_log_push(line);
}


static void maybe_save() {
    uint32_t now = millis();
    if (!s_dirty) return;
    if ((now - s_last_save_ms) < NVS_SAVE_MS) return;

    bool scale_changed = fabsf(s_scale   - s_saved_scale) > (NVS_MIN_CHANGE * s_saved_scale);
    bool track_changed = fabsf(s_track_m - s_saved_track) > (NVS_MIN_CHANGE * s_saved_track);
    if (!scale_changed && !track_changed) { s_dirty = false; return; }

    Preferences prefs;
    prefs.begin(NVS_NS, /*readOnly=*/false);
    prefs.putFloat(NVS_KEY_SCALE, s_scale);
    prefs.putFloat(NVS_KEY_TRACK, s_track_m);
    prefs.end();

    s_saved_scale  = s_scale;
    s_saved_track  = s_track_m;
    s_last_save_ms = now;
    s_dirty        = false;
}


void odo_calib_flush() {
    if (!s_dirty) return;
    bool scale_changed = fabsf(s_scale   - s_saved_scale) > (NVS_MIN_CHANGE * s_saved_scale);
    bool track_changed = fabsf(s_track_m - s_saved_track) > (NVS_MIN_CHANGE * s_saved_track);
    if (!scale_changed && !track_changed) { s_dirty = false; return; }

    Preferences prefs;
    prefs.begin(NVS_NS, /*readOnly=*/false);
    prefs.putFloat(NVS_KEY_SCALE, s_scale);
    prefs.putFloat(NVS_KEY_TRACK, s_track_m);
    prefs.end();

    s_saved_scale  = s_scale;
    s_saved_track  = s_track_m;
    s_last_save_ms = millis();
    s_dirty        = false;
}


// Apply an EMA nudge toward `target`, limited to ≤MAX_STEP_FRAC of current.
static float nudge(float current, float target) {
    float step = EMA_ALPHA * (target - current);
    float lim  = MAX_STEP_FRAC * current;
    return current + clampf(step, -lim, lim);
}


// Process one completed interval between two GPS heading events.
static void process_interval(float theta_now, float east_now, float north_now) {
    float dtheta = wrapAngle(theta_now - s_prev_theta);
    float dE     = east_now  - s_prev_east;
    float dN     = north_now - s_prev_north;
    float d_gps  = sqrtf(dE * dE + dN * dN);
    float d_odo  = (float)s_acc_dodo;
    float adth   = fabsf(dtheta);

    // Guard against a stale previous reference (long GPS gap) producing a huge,
    // meaningless interval.
    if (d_odo > INTERVAL_MAX_M) { reset_straight(); return; }

    if (adth < STRAIGHT_MAX_RAD) {
        // ── Straight interval → distance scale ────────────────────────────────
        // Sanity: GPS chord and odometry distance must roughly agree.
        if (d_odo > 0.05f && d_gps > 0.05f) {
            float ratio = d_gps / d_odo;
            if (ratio > RATIO_OUTLIER_LO && ratio < RATIO_OUTLIER_HI) {
                s_straight_dgps += d_gps;
                s_straight_dodo += d_odo;
                if (s_straight_dodo >= DIST_TARGET_M) {
                    float run_ratio = (float)(s_straight_dgps / s_straight_dodo);
                    // d_odo is already ×scale, so multiply the scale by the ratio.
                    float target = s_scale * run_ratio;
                    s_scale = clampf(nudge(s_scale, target), SCALE_MIN, SCALE_MAX);
                    s_dirty = true;
                    reset_straight();
                    report(false);
                }
            } else {
                reset_straight();   // outlier — drop the straight run
            }
        }
    } else if (adth > TURN_MIN_RAD) {
        // ── Turn interval → track width ───────────────────────────────────────
        // track = Σ(vL−vR)·dt / Δθ. Both numerator and Δθ are CW-positive.
        float track_est = (float)s_acc_sdiff / dtheta;
        float lo = s_nominal_track * TRACK_MIN_FRAC;
        float hi = s_nominal_track * TRACK_MAX_FRAC;
        if (isfinite(track_est) && track_est > lo && track_est < hi) {
            s_track_m = clampf(nudge(s_track_m, track_est), lo, hi);
            s_dirty = true;
            report(false);
        }
        reset_straight();   // a turn breaks any straight run in progress
    } else {
        // Ambiguous heading change — not clean enough to attribute.
        reset_straight();
    }
}


void odo_calib_update(float v_left_scaled, float v_right_scaled, float dt) {
    if (dt <= 0.0f || dt > 1.0f) return;   // ignore absurd dt (first tick / hiccup)

    // Integrate this tick into the per-interval accumulators.
    float v_fwd  = 0.5f * (v_left_scaled + v_right_scaled);
    s_acc_dodo  += (double)fabsf(v_fwd) * dt;
    s_acc_sdiff += (double)(v_left_scaled - v_right_scaled) * dt;

    // Poll the EKF for a new GPS heading event.
    uint32_t seq; float theta, east, north;
    if (ekf_get_gps_heading_event(&seq, &theta, &east, &north) && seq != s_last_seq) {
        if (s_have_prev) {
            process_interval(theta, east, north);
        }
        // Advance the reference to this event and start a fresh interval.
        s_prev_theta = theta;
        s_prev_east  = east;
        s_prev_north = north;
        s_have_prev  = true;
        s_last_seq   = seq;
        reset_accumulators();
    }

    maybe_save();
}


float odo_cal_scale()   { return s_scale; }
float odo_cal_track_m() { return s_track_m; }
