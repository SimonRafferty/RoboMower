# BNO055 IMU Swap — Design Spec

**Date:** 2026-06-14
**Status:** Approved (design) — ready for implementation plan
**Scope owner:** Simon Rafferty

## 1. Summary

Replace the 6-axis BMI270 IMU with a 9-axis **Bosch BNO055** (on-chip sensor
fusion) to obtain a continuous, tilt-compensated **absolute heading** at
standstill, during on-the-spot pivots, and under patchy tree cover — the gap
that GPS travel-heading cannot fill. The BNO055 becomes the *only* IMU; there is
no PCB space for both, and all trace of the BMI270 is removed from code and docs
as if it never existed.

The swap also lets us **delete** the fragile AUTO heading machinery (odometry
heading integration, the gyro-during-pivot exception, the hard GPS-lock-on-
straights blend, and the AUTO bootstrap straight-creep) and replace it with a
single clean rule:

```
heading = BNO_fused_heading + offset
```

where `offset` is slowly trimmed from the GPS travel-direction on straight RTK
segments and persisted to NVS.

The part is **in hand but not yet wired**. All code and tooling is written now;
on-hardware verification (I²C comms, axis signs, magnetometer distortion,
collision baseline) happens when it is connected.

## 2. Goals / Non-Goals

### Goals
- Drop-in BNO055 driver behind the **existing `imu_*()` API** so the main
  consumers (collision, safety/tilt, node_follower) need no change. The EKF,
  telemetry, and BLE diagnostics get small, targeted edits (heading production,
  calibration-status field, dropping the removed `imu_get_heading/bias` reads).
- BNO-primary absolute heading with a GPS-trimmed offset, wired into the EKF for
  **both AUTO and MANUAL** operation.
- Rewrite the collision-detection framework for the new sensor, but leave it
  **disabled** until a real normal-driving/mowing acceleration baseline exists.
- Replace the AUTO-start "≥2 m perimeter clearance + creep-to-establish-heading"
  gate with a **heading-confidence** gate based on BNO calibration.
- Surface BNO calibration status to the TX16S and guide the operator through a
  one-time magnetometer calibration drive.
- **Total BMI270 purge** across code, comments, `CLAUDE.md`, and `README.md`.

### Non-Goals (explicitly out of scope — do not touch)
- **Coverage / spiral path planner** — it works well; left untouched. The BNO
  swap does not intersect it.
- **Dual-antenna RTK heading** — rejected earlier (cost, little benefit under
  tree cover).
- **Re-enabling AUTO fault responses** (`AUTO_FAULT_RESPONSES_ENABLED` stays 0).
  Collision is rewritten but remains disabled.
- **IMUPLUS / accelerometer-only heading fallback — deliberately rejected.**
  Accelerometer/gyro-only heading proved unreliable; that is the reason for the
  magnetometer. If the magnetometer is untrustworthy, the design **requires a
  recalibration or a hardware fix**, never an automatic downgrade to a worse
  source. Likewise, **no wheel-odometry heading fallback** (it was the original
  ~90°-off AUTO heading problem).

## 3. Build / verification phasing

One spec, built in order. "Verifiable when" notes what can only be confirmed on
hardware once the BNO055 is wired.

| Phase | Work | Verifiable |
|---|---|---|
| 0 | Scaffolding: device-neutral `imu.h` API, `imu_bno055.cpp` skeleton, `config.h` switches. Compiles with the BMI270 gone. | Offline (compile) |
| 1 | Driver bring-up: I²C @100 kHz, comms, axis-remap confirm, tilt/accel/gyro/heading getters, NVS calibration profile. | When wired |
| 2 | Mag-distortion **validation**: log BNO heading vs GPS travel-heading under varying drive/blade load → confirm trustworthy; if not, **fix the install** (re-mount / move from interference). No software fallback. | When wired + driving |
| 3 | Heading rewrite: `heading = BNO + GPS-trimmed offset` into EKF; delete gyro-pivot, GPS-lock, bootstrap creep, **and the odometry-heading path**; re-gate AUTO start. | When wired + RTK |
| 4 | Collision re-tune: BNO linear-accel source, reset baseline, baseline-capture logging. **Stays disabled.** | When wired |

**Phases 0 and 3 (plus the driver implementation they depend on) and the
collision rewrite are all written in this effort.** Phases 1 and 2 are the
hardware-verification activities for code written now.

## 4. Driver and API

### 4.1 File structure
- New **`imu.h`** — the stable, device-neutral `imu_*()` API (the contract all
  consumers already use).
- New **`imu_bno055.cpp`** — implementation.
- Delete **`imu_bmi270.h` / `imu_bmi270.cpp`** (recoverable from git history).
- Rename the one device-specific symbol `imu_bmi270_init()` → **`imu_init()`**
  (only call site: `Robo-Mower-V2.ino`).
- Update `#include "imu_bmi270.h"` → `#include "imu.h"` in: `ble_server.cpp`,
  `node_follower.cpp`, `safety.cpp`, `safety.h` (comment), `state_machine.cpp`,
  and the `.ino`.

### 4.2 Library
- **`Adafruit_BNO055`** + **`Adafruit_Unified_Sensor`** (Library Manager).
- Remove `SparkFun BMI270 Arduino Library` from the required-library list.

### 4.3 Preserved API (signatures unchanged)
`imu_init()` (renamed), `imu_get_gz_rads()`, `imu_get_tilt_rad()`,
`imu_get_pitch_rad()`, `imu_get_roll_rad()`, `imu_get_accel(ax,ay,az)`,
`imu_is_present()`, `imu_is_fault()`.

### 4.4 New / changed API
- `float imu_get_heading_fused()` — absolute, tilt-compensated heading
  (rad, ±π, CW-from-North), from BNO fusion. **New.**
- `bool imu_heading_is_confident()` — true when the absolute heading is
  trustworthy (see §7). **New.**
- `uint8_t imu_get_calib_status()` — packed BNO calibration (sys/gyro/accel/mag,
  2 bits each) for telemetry/diagnostics. **New.**
- Removed: `imu_collect_bias()` (BNO self-calibrates), `imu_get_heading()` and
  `imu_set_heading()` (the integrated-yaw getter/setter were vestigial — heading
  now lives in the EKF as `s_theta`), `imu_get_bias()`.
- **`ble_server.cpp` diagnostics block** (currently reads `imu_get_heading`,
  `imu_get_gz_rads`, `imu_get_bias`): update to source heading from
  `imu_get_heading_fused()` / the EKF, drop the `bias` field, and add the
  calibration-status field. Small, contained edit.

### 4.5 Sensor / bus configuration
- I²C **100 kHz** (BNO clock-stretch quirk). The BNO is the only device on the
  bus (SDA=GPIO8, SCL=GPIO9), so the lower clock is global and harmless.
- **Axis remap** to robot frame (surge fwd+, sway right+, heave up+, heading
  CW-from-North+) via the BNO `AXIS_MAP_CONFIG`/`AXIS_MAP_SIGN` registers, set
  once at init. Default remap values are written with a documented bring-up
  check procedure; exact signs are confirmed when wired.
- **Fusion mode: NDOF only** (full 9-axis, mag-referenced absolute heading).
  IMUPLUS is deliberately **not** used — see Non-Goals. The mag-distortion test
  (Phase 2) validates NDOF rather than selecting between modes; a bad result is a
  hardware problem to fix, not a mode to fall back to.
- Sampling task on **Core 0 at 100 Hz** (BNO fusion output rate), replacing the
  200 Hz BMI270 task. Thread safety unchanged (portMUX spinlock around the
  published volatiles).
- `imu_get_tilt_rad/pitch/roll` derived from the **BNO Euler/quaternion**
  output (tilt-compensated) rather than raw-accel `atan2` — strictly better for
  the tilt safety check.

## 5. Heading architecture (AUTO + MANUAL)

### 5.1 Single source of truth
Heading remains the EKF state **`s_theta`**, read by every consumer through the
existing EKF heading getter. Nothing downstream changes. Only the *production*
of `s_theta` changes.

### 5.2 Normal operation (BNO healthy)
Each 10 Hz EKF tick:
```
s_theta = wrapAngle(imu_get_heading_fused() + s_hdg_offset)
```
No heading integration. Position still dead-reckons between GPS fixes using this
heading and wheel velocity.

### 5.3 Offset trim (`s_hdg_offset`)
Reuses the existing straight-RTK detector almost verbatim:
- On a clean **straight, measurable** RTK segment (`s_hdg_turn_accum <
  HEADING_STRAIGHT_MAX_TURN_RAD` and travel `> max(min_dist, k·σ)`), the GPS
  travel chord `z_hdg` is the true heading (reverse-corrected when `s_v < 0`,
  exactly as today).
- Instantaneous estimate `= wrapAngle(z_hdg − imu_get_heading_fused())`, folded
  into `s_hdg_offset` by a **slow EMA** (`HEADING_OFFSET_TRIM_GAIN`, ≈0.1).
- Robust to GPS outages — the offset simply holds.
- `s_hdg_offset` **persisted to NVS** so absolute heading is correct at boot.

### 5.4 Deleted (not gated — removed entirely)
- The gyro-during-pivot exception in `ekf_predict` (`in_place_turn`,
  `GYRO_HEADING_MAX_V_MS`, `GYRO_HEADING_MIN_OMEGA`).
- The hard GPS-lock-on-straights blend (`HEADING_GPS_LOCK_GAIN` applied to
  `s_theta`), replaced by §5.3's gentle offset trim.
- The AUTO bootstrap straight-creep heading establishment.
- Associated now-dead `config.h` constants.
- `ekf_predict`'s `gyro_rate_cw` parameter (the only consumer was the pivot
  exception) and the gyro argument at its `state_machine.cpp` call-site.
- **The wheel-odometry heading path** (`omega_odo = (vL−vR)/track` driving
  `s_theta`). The BNO supplies heading at 100 Hz independent of GPS, so there is
  no scenario where odometry heading is needed. `odo_calib`'s distance `scale`
  is retained for position dead-reckoning between GPS fixes; its kinematic
  `track_m` calibration is no longer heading-critical (kept only as a harmless
  diagnostic / for future use).

### 5.5 MANUAL operation
**No structural change.** `drive_manual` + `heading_controller` (PD + AGC) keep
working as-is; the heading hold fundamentally works. Both of its inputs are
re-wired *for free*:
- `pose.heading` ← EKF `s_theta` ← now BNO + offset.
- `yaw_rate_error` uses `imu_get_gz_rads()` ← now the BNO gyro-Z (API preserved).

### 5.6 Failure handling (no heading degradation — stop and require a fix)
There is **no fallback to a lesser heading source**. If trustworthy heading is
lost, autonomous operation stops.
- **BNO chip fault** (`imu_is_fault()`): in **AUTO**, treat as a safety fault →
  **PAUSE** (VESCs powered, CAN stop sent), surfaced to the operator. In
  **MANUAL**, disable the heading-hold assist and fall to plain open-loop
  tank-mix — the operator remains in direct control. Heading is never
  reconstructed from odometry or accelerometer.
- **Heading confidence lost during AUTO** (`imu_heading_is_confident()` goes
  false — e.g. a magnetometer-distortion event): **PAUSE** and prompt
  recalibration. This is the §7 gate, monitored continuously, not only at start.
- `odo_calib` continues to consume the published straight-RTK heading events for
  its distance `scale` self-cal (now sourced from BNO-quality headings).

## 6. Calibration and NVS

Namespace `imu` (old `gyrobias` key purged):
- **`bnocal`** — 22-byte BNO calibration profile blob. Loaded at boot (skips
  re-learning the magnetometer); auto-saved when calibration reaches full and the
  profile changes.
- **`hdgoff`** — float `s_hdg_offset`, persisted (§5.3).

The 2.5 s boot gyro-bias collection (`imu_collect_bias`) is removed — the BNO
self-calibrates.

## 7. Heading-confidence gate and AUTO start

`imu_heading_is_confident()` (NDOF): true when the BNO **system +
magnetometer** calibration are good (e.g. both ≥ 2).

**AUTO start** (`STATE_AUTO_MOWING` entry):
- The **≥2 m perimeter-clearance** requirement and the **straight-creep** are
  both **removed**.
- AUTO starts immediately when `imu_heading_is_confident()`.
- Otherwise PAUSE with an operator message: *drive slow loops in MANUAL to
  calibrate the magnetometer*.

**During AUTO:** the same condition is monitored continuously; if it goes false
(§5.6), AUTO pauses and prompts recalibration rather than degrading.

## 8. Magnetometer calibration UX

The BNO magnetometer calibrates only after seeing varied headings. A ground
robot can't perform a hand-held 3D figure-8; the on-ground equivalent is
**driving a few slow loops/circles** (sweeping heading through 360°). Because the
calibration profile is persisted to NVS, this is a **one-time** setup that
re-converges almost immediately on later boots.

- **Telemetry:** add BNO calibration status (at minimum system + magnetometer,
  0–3 each) to the `MOWER_STATUS` (0x80) CRSF frame. Extend the payload by one
  byte (or pack into spare flag bits); the Lua already tolerates variable-length
  payloads — update it for the new field.
- **TX16S Lua widget:** when `imu_heading_is_confident()` is false, display e.g.
  `MAG CAL n/3 — drive slow loops (MANUAL)`; clear once confident. Same condition
  that gates AUTO start.

## 9. Collision framework rewrite (left disabled)

- Feed `collisionDetectUpdate()` from BNO **linear acceleration** (gravity
  removed by fusion — cleaner than the old flat-ground `az − 1g` approximation),
  converted to **g** so the existing magnitude-jolt algorithm and the `*_G`
  constants remain valid with minimal churn.
- **Reset the NVS collision baseline** (`collision/baseline`) — units and noise
  characteristics differ from the BMI270.
- Add **baseline-capture logging** (visible over BLE/serial) so a real
  normal-driving/mowing baseline can be observed.
- Detection stays **off** (already gated by `AUTO_FAULT_RESPONSES_ENABLED 0`).
  Do **not** enable until a trustworthy baseline is established.

## 10. BMI270 purge

Remove every BMI270 reference as if it never existed:
- Delete `imu_bmi270.h/.cpp`.
- Update all `#include`s and the `imu_init()` call site (§4.1).
- **`CLAUDE.md`:** remove the *"Planned: BNO055 IMU swap"* section; rewrite all
  IMU references to describe the BNO055 as the present sensor; update the module
  overview, the NVS Key Registry (`imu` namespace → `bnocal`, `hdgoff`), the
  required-library list, the I²C clock (100 kHz), and the heading/EKF sections to
  match §5. Remove now-deleted heading constants from any documented lists. Watch
  the 40 k-char budget — net effect should shrink the file.
- **`README.md`:** update IMU mentions and any wiring/tuning notes.
- All in-code comments referencing the BMI270, gyro bias collection, or the old
  heading machinery.

## 11. Testing strategy

### Offline (now)
- **Host-test the offset-trim math**: straight-segment detector + EMA
  convergence to a known offset; reverse-correction when `s_v < 0`; behaviour
  across GPS outages (offset holds). Follow the existing `host_test/` pattern.
- Full project **compiles** with the BMI270 removed.

### On-hardware (when wired), in order
1. I²C comms @100 kHz; `imu_is_present()` true.
2. Axis-sign confirmation (tilt the robot; check surge/sway/heave + yaw signs).
3. Tilt/pitch/roll sanity vs the safety tilt limit.
4. **Mag-distortion validation** (§4.5, Phase 2): confirm NDOF heading holds
   under drive/blade load. A bad result → fix the install; do **not** ship a
   software fallback.
5. BNO heading vs GPS travel-heading on straight RTK runs (offset converges).
6. AUTO heading correct at start, through pivots, and under tree cover.
7. Collision baseline capture during normal driving/mowing.

## 12. Risks and rollback

- **Axis signs wrong** → caught at bring-up (step 2); fix the remap constants.
- **Magnetometer distortion from motors/blade** → validated in Phase 2 and
  expected to be a non-issue (a phone magnetometer in the same mounting spot
  reads good heading). Mounting is ~150 mm from the motors, and distance — not
  the 304 stainless — is what helps. **If distortion is found, the fix is
  hardware** (re-mount / shield routing / move interference), per the no-fallback
  decision; there is no software degradation path.
- **First-ever-boot heading unknown** → the confidence gate refuses AUTO start
  until the magnetometer is calibrated (operator drives slow loops); the cal
  profile and `hdgoff` then persist to NVS for subsequent boots.
- **Rollback:** the BMI270 is recoverable from git history, but because the purge
  is intentional, rollback means `git revert`, not a config flag. Acceptable —
  the BNO055 is in hand and this is the committed direction.

## 13. Per-CLAUDE.md backup note

Before the implementation touches source, create a dated full-source backup under
`Backups/2026-06-14/` per the project backup policy (Claude Code checkpoints do
not survive context compaction).
