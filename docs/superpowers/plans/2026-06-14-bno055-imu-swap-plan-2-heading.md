# BNO055 IMU Swap — Plan 2: Heading Rewrite

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make EKF heading `s_theta = BNO_fused_heading + GPS-trimmed offset`, deleting the wheel-odometry heading path, the gyro-during-pivot exception, the hard GPS-lock, and the AUTO bootstrap; gate AUTO on `imu_heading_is_confident()` with continuous monitoring.

**Architecture:** A new pure, host-tested `heading_fusion` module holds the offset-trim math (straight-segment gate, wrapped-angle EMA, heading compose). `ekf_predict()` sets heading directly from the BNO each tick (no integration); `ekf_update_gps()` slowly trims the offset from the GPS travel chord on straight RTK segments and persists it to NVS. The state machine replaces the 2 m-clearance + creep bootstrap with a heading-confidence gate that also pauses AUTO if confidence is lost.

**Tech Stack:** ESP32-S3 / Arduino, FreeRTOS mutex, ESP32 `Preferences` (NVS ns `imu`, key `hdgoff`), Zig clang for host tests, `arduino-cli` for compile.

**Depends on:** Plan 1 (`imu.h`: `imu_get_heading_fused`, `imu_is_fault`, `imu_heading_is_confident`).

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `Robo-Mower-V2/heading_fusion.h` | Create | Pure offset-trim API (no Arduino deps). |
| `Robo-Mower-V2/heading_fusion.cpp` | Create | Pure offset-trim implementation. |
| `host_test/heading_fusion_test.cpp` | Create | Host unit tests for the pure module. |
| `Robo-Mower-V2/config.h` | Modify | Replace heading/bootstrap/gyro-pivot constants with offset-trim constants. |
| `Robo-Mower-V2/ekf_localiser.h` | Modify | `ekf_predict` signature; add offset getter + persister; doc. |
| `Robo-Mower-V2/ekf_localiser.cpp` | Modify | BNO-primary heading, offset trim, NVS persist; delete odometry/gyro/lock. |
| `Robo-Mower-V2/state_machine.cpp` | Modify | New `ekf_predict` call; persist hook; confidence gate replaces bootstrap. |

---

## Task 1: Pure `heading_fusion` module + failing host test

**Files:**
- Create: `Robo-Mower-V2/heading_fusion.h`
- Create: `host_test/heading_fusion_test.cpp`

- [ ] **Step 1: Write `heading_fusion.h`**

Create `Robo-Mower-V2/heading_fusion.h` with exactly:
```cpp
#pragma once
// ══════════════════════════════════════════════════════════════════════════════
//  heading_fusion.h — pure helpers for BNO-heading + GPS-trimmed offset
//
//  No Arduino / FreeRTOS dependencies — host-testable. Angles in radians,
//  wrapped to ±π. Thresholds are passed in (no config.h dependency) so the
//  module is trivially unit-testable.
// ══════════════════════════════════════════════════════════════════════════════

/** True if the segment since the heading reference was STRAIGHT and MEASURABLE,
 *  i.e. the GPS travel chord is a valid absolute heading to trim the offset to.
 *  @param turn_accum_rad   Σ|Δheading| since the reference (rad).
 *  @param dist_m           Travel distance of the chord (m).
 *  @param sigma_m          GPS 1-σ position noise (m).
 *  @param straight_max_turn_rad  Max allowed turn to call it straight.
 *  @param min_dist_m       Distance floor.
 *  @param dist_sigma_k     Required dist also ≥ k·sigma. */
bool heading_offset_segment_qualifies(float turn_accum_rad, float dist_m,
                                      float sigma_m, float straight_max_turn_rad,
                                      float min_dist_m, float dist_sigma_k);

/** EMA-update the heading offset toward (z_hdg − bno_hdg), wrap-safe.
 *  @return new offset (rad, ±π). */
float heading_offset_ema(float offset, float z_hdg, float bno_hdg, float gain);

/** Compose absolute heading from BNO heading + offset (rad, ±π). */
float heading_compose(float bno_hdg, float offset);
```

- [ ] **Step 2: Write the host test**

Create `host_test/heading_fusion_test.cpp` with exactly:
```cpp
// Host unit tests for heading_fusion.cpp. Build with Zig clang (see run cmd).
// Exit code = number of failures.
#include "heading_fusion.h"
#include "geometry.h"   // wrapAngle()
#include <cstdio>
#include <cmath>

static int g_fail = 0;
static void check(bool cond, const char* name) {
    if (!cond) { printf("FAIL: %s\n", name); g_fail++; }
    else       { printf("ok:   %s\n", name); }
}
static bool near_rad(float a, float b, float tol) {
    return fabsf(wrapAngle(a - b)) < tol;
}

int main() {
    const float MAXTURN = 0.20f, MINDIST = 0.30f, K = 20.0f;

    // Gate: a turn beyond the straight limit disqualifies regardless of distance.
    check(!heading_offset_segment_qualifies(0.30f, 5.0f, 0.01f, MAXTURN, MINDIST, K),
          "turned segment disqualified");
    // Gate: straight + far enough (RTK fixed, sigma 1 cm → thresh 0.30 m).
    check(heading_offset_segment_qualifies(0.05f, 0.50f, 0.01f, MAXTURN, MINDIST, K),
          "straight+far qualifies");
    // Gate: straight but too short.
    check(!heading_offset_segment_qualifies(0.05f, 0.20f, 0.01f, MAXTURN, MINDIST, K),
          "straight+short disqualified");
    // Gate: float fix (sigma 0.15 m → thresh = 3.0 m), 1 m is too short.
    check(!heading_offset_segment_qualifies(0.05f, 1.0f, 0.15f, MAXTURN, MINDIST, K),
          "float fix needs 3 m");

    // compose wraps.
    check(near_rad(heading_compose(3.0f, 0.5f), wrapAngle(3.5f), 1e-4f),
          "compose wraps");

    // EMA converges offset so compose(bno, offset) → z_hdg.
    // True heading z = 1.00 rad; BNO reads 0.70 rad → offset should approach 0.30.
    float off = 0.0f;
    for (int i = 0; i < 200; i++) off = heading_offset_ema(off, 1.00f, 0.70f, 0.1f);
    check(near_rad(off, 0.30f, 1e-3f), "EMA converges to offset");
    check(near_rad(heading_compose(0.70f, off), 1.00f, 1e-3f),
          "composed heading matches truth");

    // EMA wrap-safe near ±π: z = +3.10, bno = -3.10 → small positive offset.
    float off2 = 0.0f;
    for (int i = 0; i < 200; i++) off2 = heading_offset_ema(off2, 3.10f, -3.10f, 0.1f);
    check(near_rad(heading_compose(-3.10f, off2), 3.10f, 1e-3f),
          "EMA wrap-safe across ±pi");

    printf("\n%d failure(s)\n", g_fail);
    return g_fail;
}
```

- [ ] **Step 3: Build the test and confirm it FAILS to link (no implementation yet)**

Run:
```powershell
$zig = (Get-ChildItem "$env:LOCALAPPDATA\Microsoft\WinGet\Packages\zig.zig_*\zig-*\zig.exe").FullName
cd host_test
& $zig c++ -std=c++17 -O1 "-I../Robo-Mower-V2" -o hftest.exe `
  heading_fusion_test.cpp ../Robo-Mower-V2/heading_fusion.cpp geometry.cpp
```
Expected: **fails** — `../Robo-Mower-V2/heading_fusion.cpp` does not exist yet (file-not-found / link error). This confirms the test targets unimplemented code.

- [ ] **Step 4: Commit**

```bash
git add Robo-Mower-V2/heading_fusion.h host_test/heading_fusion_test.cpp
git commit -m "heading_fusion: pure offset-trim API + failing host test"
```

---

## Task 2: Implement `heading_fusion.cpp` and pass the test

**Files:**
- Create: `Robo-Mower-V2/heading_fusion.cpp`

- [ ] **Step 1: Write `heading_fusion.cpp`**

Create `Robo-Mower-V2/heading_fusion.cpp` with exactly:
```cpp
#include "heading_fusion.h"
#include "geometry.h"   // wrapAngle()
#include <cmath>

bool heading_offset_segment_qualifies(float turn_accum_rad, float dist_m,
                                      float sigma_m, float straight_max_turn_rad,
                                      float min_dist_m, float dist_sigma_k) {
    if (turn_accum_rad > straight_max_turn_rad) return false;   // not straight
    float thresh = fmaxf(min_dist_m, dist_sigma_k * sigma_m);   // measurable
    return dist_m > thresh;
}

float heading_offset_ema(float offset, float z_hdg, float bno_hdg, float gain) {
    float inst = wrapAngle(z_hdg - bno_hdg);            // instantaneous offset
    return wrapAngle(offset + gain * wrapAngle(inst - offset));  // wrap-safe EMA
}

float heading_compose(float bno_hdg, float offset) {
    return wrapAngle(bno_hdg + offset);
}
```

- [ ] **Step 2: Build and run — expect PASS**

Run:
```powershell
$zig = (Get-ChildItem "$env:LOCALAPPDATA\Microsoft\WinGet\Packages\zig.zig_*\zig-*\zig.exe").FullName
cd host_test
& $zig c++ -std=c++17 -O1 "-I../Robo-Mower-V2" -o hftest.exe `
  heading_fusion_test.cpp ../Robo-Mower-V2/heading_fusion.cpp geometry.cpp
.\hftest.exe
echo "exit=$LASTEXITCODE"
```
Expected: all lines `ok:`, final `0 failure(s)`, `exit=0`.

- [ ] **Step 3: Commit**

```bash
git add Robo-Mower-V2/heading_fusion.cpp
git commit -m "heading_fusion: implement offset gate + wrap-safe EMA + compose (tests pass)"
```

---

## Task 3: Replace heading constants in `config.h`

**Files:**
- Modify: `Robo-Mower-V2/config.h` (heading block, ~lines 285–326)

- [ ] **Step 1: Replace the gyro-pivot + bootstrap + GPS-lock block**

In `Robo-Mower-V2/config.h`, replace the entire block from the comment
`// Gyro-assisted heading during pivots ...` (≈ line 285) down to and including
`#define HEADING_GPS_DIST_SIGMA_K      20.0f` (≈ line 326) with:
```cpp
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
```

> This deletes `GYRO_HEADING_MAX_V_MS`, `GYRO_HEADING_MIN_OMEGA`,
> `AUTO_BOOTSTRAP_PERIM_MIN_M`, `AUTO_BOOTSTRAP_SPEED_MS`, `AUTO_BOOTSTRAP_MAX_MS`,
> and `HEADING_GPS_LOCK_GAIN`. `RTK_MIN_FIX_FOR_LEARNING`, `NODE_HEADING_KP`,
> `NODE_YAW_RATE_MAX`, `HEADING_STRAIGHT_MAX_TURN_RAD`, `HEADING_FROM_GPS_MIN_DIST_M`,
> `HEADING_GPS_DIST_SIGMA_K` are preserved (values unchanged).

- [ ] **Step 2: Verify the deleted names are gone**

```powershell
Select-String -Path Robo-Mower-V2\config.h -Pattern 'GYRO_HEADING_|AUTO_BOOTSTRAP_|HEADING_GPS_LOCK_GAIN'
```
Expected: **no matches.**

- [ ] **Step 3: Commit**

```bash
git add Robo-Mower-V2/config.h
git commit -m "config: replace gyro-pivot/bootstrap/GPS-lock with offset-trim constants"
```

---

## Task 4: Update `ekf_localiser.h`

**Files:**
- Modify: `Robo-Mower-V2/ekf_localiser.h`

- [ ] **Step 1: Change the `ekf_predict` declaration**

Replace:
```cpp
void ekf_predict(float v_left, float v_right, float gyro_rate_cw, float dt);
```
with:
```cpp
void ekf_predict(float v_left, float v_right, float dt);
```
And replace its doc comment (the block starting `@brief EKF prediction step`) with:
```cpp
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
```

- [ ] **Step 2: Add the offset getter + persister declarations**

After the `ekf_get_heading_uncertainty()` declaration, add:
```cpp
/** Current GPS-trimmed heading offset (rad) = heading − BNO_fused. Thread-safe. */
float ekf_get_heading_offset();

/** Persist the heading offset to NVS if it has changed enough and the throttle
 *  interval has elapsed. Call from the 10 Hz hook (Core 1). */
void ekf_save_heading_offset_if_due();
```

- [ ] **Step 3: Commit**

```bash
git add Robo-Mower-V2/ekf_localiser.h
git commit -m "ekf: header — drop gyro arg from ekf_predict, add offset getter/persister"
```

---

## Task 5: Rewrite `ekf_localiser.cpp` heading path

**Files:**
- Modify: `Robo-Mower-V2/ekf_localiser.cpp`

- [ ] **Step 1: Update includes and add offset state**

Replace:
```cpp
#include "odo_calib.h"  // for odo_cal_track_m() — calibrated kinematic track
#include "geometry.h"   // for clampf(), wrapAngle()
```
with:
```cpp
#include "geometry.h"        // for clampf(), wrapAngle()
#include "heading_fusion.h"  // offset gate + wrap-safe EMA + compose
#include "imu.h"             // imu_get_heading_fused(), imu_is_fault()
#include <Preferences.h>     // NVS persistence of the heading offset
```
Then, in the "GPS heading tracking" static block (after `s_hdg_turn_accum`), add:
```cpp
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
```
And add the NVS namespace/key constants near the top of the file (after the includes):
```cpp
static const char* EKF_NVS_NS      = "imu";
static const char* EKF_NVS_KEY_OFF = "hdgoff";
```

- [ ] **Step 2: Load the offset in `ekf_init()`**

In `ekf_init()`, after `s_gps_seeded = false;` and before `s_heading_established = false;`, insert:
```cpp
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
```

- [ ] **Step 3: Replace the body of `ekf_predict()`**

Replace the entire `ekf_predict(...)` function (from its signature through its closing brace, ≈ lines 253–324) with:
```cpp
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
    }
    // else: BNO faulted — hold s_theta, do not accrue turn (AUTO pauses).

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
```

- [ ] **Step 4: Replace the GPS heading-lock block in `ekf_update_gps()`**

In `ekf_update_gps()`, replace the block that begins at the comment
`// ── GPS heading update ───` and runs through the end of the
`if (fix_type >= 4) { ... }` statement (≈ lines 405–464) with:
```cpp
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

                    float bno = imu_get_heading_fused();
                    s_hdg_offset = heading_offset_ema(s_hdg_offset, z_hdg, bno,
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
```

- [ ] **Step 5: Add the offset getter + throttled persister**

After `ekf_get_heading_uncertainty()` (≈ line 527), add:
```cpp
float ekf_get_heading_offset() {
    if (g_ekf_mutex == nullptr) return 0.0f;
    xSemaphoreTake(g_ekf_mutex, portMAX_DELAY);
    float o = s_hdg_offset;
    xSemaphoreGive(g_ekf_mutex);
    return o;
}

void ekf_save_heading_offset_if_due() {
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
```
> `millis()` and `DBG_PRINTF` come transitively via `config.h`/Arduino already
> included by `ekf_localiser.cpp`. If `millis` is unresolved, add `#include <Arduino.h>`.

- [ ] **Step 6: Commit**

```bash
git add Robo-Mower-V2/ekf_localiser.cpp
git commit -m "ekf: BNO-primary heading + GPS offset trim; delete odometry/gyro/lock heading"
```

---

## Task 6: Update the EKF call site in `state_machine.cpp`

**Files:**
- Modify: `Robo-Mower-V2/state_machine.cpp` (EKF hook, ≈ lines 1540–1553)

- [ ] **Step 1: Drop the gyro read, change the call, add the persist hook**

Replace:
```cpp
        float v_left  = vesc_erpm_to_velocity_scaled(vesc_get_status(VESC_ID_LEFT).erpm);
        float v_right = vesc_erpm_to_velocity_scaled(vesc_get_status(VESC_ID_RIGHT).erpm);
        // Gyro Z (bias-corrected, CW+) — used by the EKF for heading ONLY during
        // on-the-spot pivots, where wheel-odometry heading over-rotates (scrub)
        // and GPS cannot correct it. Odometry+GPS remain the source otherwise.
        float gyro_cw = imu_get_gz_rads();
        ekf_predict(v_left, v_right, gyro_cw, dt);
        // Self-calibrate scale (straights) + track width (turns) from GPS, in
        // every state where the wheels turn (manual and auto).
        odo_calib_update(v_left, v_right, dt);
```
with:
```cpp
        float v_left  = vesc_erpm_to_velocity_scaled(vesc_get_status(VESC_ID_LEFT).erpm);
        float v_right = vesc_erpm_to_velocity_scaled(vesc_get_status(VESC_ID_RIGHT).erpm);
        // Heading comes from the BNO055 absolute fusion + GPS-trimmed offset
        // inside ekf_predict() — no gyro/odometry heading term here.
        ekf_predict(v_left, v_right, dt);
        // Self-calibrate distance scale (straights) from GPS in every driving state.
        odo_calib_update(v_left, v_right, dt);
        // Persist the GPS-trimmed heading offset (throttled, NVS).
        ekf_save_heading_offset_if_due();
```

- [ ] **Step 2: Commit**

```bash
git add Robo-Mower-V2/state_machine.cpp
git commit -m "sm: ekf_predict without gyro; persist heading offset each EKF tick"
```

---

## Task 7: Replace the AUTO bootstrap with a heading-confidence gate

**Files:**
- Modify: `Robo-Mower-V2/state_machine.cpp` (STATE_AUTO_MOWING, ≈ lines 2108–2126 and 2409–2468)

- [ ] **Step 1: Remove the bootstrap statics at AUTO entry**

In `case STATE_AUTO_MOWING:` remove the now-unused statics and their entry
assignments. Delete:
```cpp
        static bool     s_in_bootstrap  = false; // true during the start phase (2 m gate + heading establish)
        static bool     s_auto_resumed  = false; // true when this AUTO run resumed from PAUSED
```
and in the `if (g_state_entry)` block delete:
```cpp
            // Run the start-phase checks (2 m perimeter gate + heading bootstrap)
            // on every entry. A resume from PAUSED skips the 2 m gate so a mow
            // paused near the edge (e.g. mid headland pass) can resume.
            s_in_bootstrap     = true;
            s_auto_resumed     = (g_prev_state == STATE_PAUSED);
```
> Keep the separate `bool resuming = (g_prev_state == STATE_PAUSED);` line below —
> it gates re-planning and is unrelated to the bootstrap.

- [ ] **Step 2: Replace the start-phase block with the confidence gate**

Replace the entire `if (s_in_bootstrap) { ... }` block (≈ lines 2409–2468,
from the comment `// ── Start phase: 2 m perimeter gate + heading bootstrap ──`
through the closing `}` and the `sys_log_push("AUTO: start checks ok ...")`)
with:
```cpp
            // ── Heading-confidence gate (replaces 2 m clearance + creep) ──────
            // AUTO needs a trustworthy absolute heading. The BNO055 provides it
            // at 100 Hz; we only require its magnetometer to be calibrated.
            // Checked every tick: if confidence is lost mid-mow, PAUSE rather
            // than degrade. The operator recalibrates by driving slow loops in
            // MANUAL (prompted on the TX16S widget and PWA).
            if (!imu_heading_is_confident()) {
                vesc_set_current(VESC_ID_LEFT,  0);
                vesc_set_current(VESC_ID_RIGHT, 0);
                sys_log_push("AUTO: heading not confident (mag) -> PAUSE; drive loops in Manual");
                request_beep(BEEP_WARNING);
                transition_to(STATE_PAUSED);
                break;
            }
```

- [ ] **Step 3: Verify the removed symbols are gone**

```powershell
Select-String -Path Robo-Mower-V2\state_machine.cpp -Pattern 's_in_bootstrap|s_auto_resumed|AUTO_BOOTSTRAP|ekf_heading_is_established'
```
Expected: **no matches** (the bootstrap is fully removed; `ekf_heading_is_established()` is no longer called from the state machine).

- [ ] **Step 4: Commit**

```bash
git add Robo-Mower-V2/state_machine.cpp
git commit -m "sm: AUTO heading-confidence gate replaces 2m-clearance + creep bootstrap"
```

---

## Task 8: Compile the full firmware

**Files:** none (verification)

- [ ] **Step 1: Compile**

Run:
```powershell
$cli = "C:\Users\simon\Downloads\arduino-cli_1.5.0_Windows_64bit\arduino-cli.exe"
& $cli compile --fqbn esp32:esp32:esp32s3 `
  --board-options "PartitionScheme=huge_app,PSRAM=opi" Robo-Mower-V2
```
Expected: **no errors.** Likely fix points: a leftover `gyro_cw`/4-arg `ekf_predict`
call, a stale `AUTO_BOOTSTRAP_*`/`GYRO_HEADING_*` reference, or `heading_fusion.h`
not found (it lives in the sketch root, compiled automatically).

- [ ] **Step 2: Re-run the host test (still green)**

```powershell
$zig = (Get-ChildItem "$env:LOCALAPPDATA\Microsoft\WinGet\Packages\zig.zig_*\zig-*\zig.exe").FullName
cd host_test
& $zig c++ -std=c++17 -O1 "-I../Robo-Mower-V2" -o hftest.exe `
  heading_fusion_test.cpp ../Robo-Mower-V2/heading_fusion.cpp geometry.cpp
.\hftest.exe; echo "exit=$LASTEXITCODE"
```
Expected: `0 failure(s)`, `exit=0`.

- [ ] **Step 3: [ON-HARDWARE — DEFERRED] Verify with the BNO wired (Plan 2 bring-up)**

- Drive a straight RTK-fixed run: the EKF heading offset converges so `hdg`
  (telemetry) matches the GPS travel direction; the value persists across reboot.
- Pivot on the spot: heading tracks the BNO smoothly (no hunt — the gyro-pivot
  path is gone).
- Start AUTO with the mag uncalibrated → PAUSE with the "drive loops" log; after
  calibrating, AUTO starts with no creep.

- [ ] **Step 4: Commit (marker)**

No code change unless Step 1/2 required fixes (commit those under their task).

---

## Self-Review (against the spec)

- **§5.1 single source of truth (`s_theta`):** Task 5 Step 3. ✓
- **§5.2 BNO + offset, no integration; position DR:** Task 5 Step 3. ✓
- **§5.3 offset trim on straight RTK + NVS persist:** Tasks 1–2 (math), 5 Steps 1/2/4/5, 6. ✓
- **§5.4 delete gyro-pivot, GPS-lock, bootstrap, odometry-heading + `gyro_rate_cw`:** Tasks 3, 4, 5, 6, 7. ✓
- **§5.6 BNO fault holds heading (no odometry); AUTO pause:** Task 5 Step 3 (hold) + Task 7 (confidence/fault → pause). ✓
- **§7 confidence gate + continuous monitor:** Task 7. ✓
- **odo_calib heading events still published:** Task 5 Step 4 (`s_gps_hdg_*`). ✓

**Type/name consistency:** `ekf_predict(float,float,float)` (Tasks 4, 5, 6); `heading_offset_segment_qualifies` / `heading_offset_ema` / `heading_compose` identical across Tasks 1, 2, 5; NVS ns `imu` key `hdgoff` matches Plan 1's reserved key.

**Carried to later plans:** MANUAL needs no edit (Task: none — `pose.heading` and `imu_get_gz_rads()` are already BNO-sourced, per spec §5.5). Telemetry of `imu_heading_is_confident()` to the operator (Lua + PWA) is Plan 3. `BNO fault → MANUAL drops heading-assist` is inherent (heading-hold uses `pose.heading`/`gz`, which simply stop updating; the operator stays in control) — no code change required, noted for Plan 3 verification.
