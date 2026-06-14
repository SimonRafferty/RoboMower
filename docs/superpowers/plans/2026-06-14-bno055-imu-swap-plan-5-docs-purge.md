# BNO055 IMU Swap — Plan 5: Documentation & Comment Purge

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove every BMI270 / old-heading reference from `CLAUDE.md`, `README.md`, and remaining code comments, and document the BNO055 as the present IMU (NDOF absolute heading + GPS-trimmed offset, no odometry/gyro-pivot/GPS-lock/bootstrap) — as if the BMI270 never existed.

**Architecture:** Pure documentation/comment edits. No behaviour change, nothing to compile (a final compile confirms no comment edit broke a line). Net effect should *shrink* `CLAUDE.md` (the large "Planned: BNO055 IMU swap" section is removed), keeping it under the 40 k-char budget. `collision_detect.*` is excluded — Plan 4 already purged it.

**Tech Stack:** Markdown docs; `arduino-cli` (final compile sanity).

**Depends on:** Plans 1–4 (the code they describe must already exist).

**Pre-flight (run first to see the full surface):**
```powershell
Select-String -Path CLAUDE.md,README.md,Robo-Mower-V2\*.h,Robo-Mower-V2\*.cpp,Robo-Mower-V2\*.ino `
  -Pattern 'BMI270|bmi270|gyrobias|GYRO_HEADING|AUTO_BOOTSTRAP|HEADING_GPS_LOCK|imu_collect_bias|imu_get_heading\b|imu_set_heading|imu_get_bias'
```
Keep this list; Task 9 re-runs it and must come back empty.

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `CLAUDE.md` | Modify | Remove BMI270/swap; document BNO055 as present; update NVS/library/GPIO/EKF/heading. |
| `README.md` | Modify | IMU bullet, library line, heading-source paragraph. |
| `Robo-Mower-V2/*.{h,cpp,ino}` | Modify | Any remaining BMI270/old-heading comment tokens (excl. `collision_detect.*`). |

---

## Task 1: CLAUDE.md — overview, library, GPIO

**Files:** Modify `CLAUDE.md`

- [ ] **Step 1: Project Overview**

Replace:
```
VESC motor controllers over CAN bus, a BMI270 IMU (**being replaced by a BNO055** — see *Planned: BNO055 IMU swap*), CRSF radio (RadioMaster TX16S + ER8 receiver), and a **concentric inward spiral** coverage planner (replaced boustrophedon strips 2026-06-13) built on the vendored Clipper2 polygon-offset library.
```
with:
```
VESC motor controllers over CAN bus, a Bosch **BNO055** 9-axis IMU (on-chip NDOF fusion → tilt-compensated absolute heading), CRSF radio (RadioMaster TX16S + ER8 receiver), and a **concentric inward spiral** coverage planner (replaced boustrophedon strips 2026-06-13) built on the vendored Clipper2 polygon-offset library.
```

- [ ] **Step 2: Required libraries (Build section)**

Replace:
```
Required libraries (Library Manager): `SparkFun BMI270 Arduino Library` (IMU — until the BNO055 swap, then e.g. `Adafruit_BNO055`) and `FastLED`.
```
with:
```
Required libraries (Library Manager): `Adafruit BNO055` (+ `Adafruit Unified Sensor`) (IMU) and `FastLED`.
```

- [ ] **Step 3: GPIO pin assignments line**

Replace:
```
IMU I2C SDA/SCL GPIO8/9 (BNO055 will share this bus at 100 kHz)
```
with:
```
IMU I2C SDA/SCL GPIO8/9 (BNO055 @ 100 kHz)
```

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "docs(claude): BNO055 in overview/library/GPIO"
```

---

## Task 2: CLAUDE.md — remove the "Planned: BNO055 IMU swap" section

**Files:** Modify `CLAUDE.md`

- [ ] **Step 1: Delete the whole section**

Delete everything from the heading line:
```
## Planned: BNO055 IMU swap (replaces the BMI270)
```
up to (but **not** including) the next top-level heading:
```
## Architecture
```
(The entire "Decision (2026-06-14) … Order: …" block is removed — it is now implemented.)

- [ ] **Step 2: Remove the "Read first" pointer if it references the swap**

Near the top, the blockquote `> **Read first:** *Known Issues & TODO* below — much changed on 2026-06-13/14, be wary of stale assumptions.` may stay (still true). Leave it.

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md
git commit -m "docs(claude): remove implemented BNO055-swap planning section"
```

---

## Task 3: CLAUDE.md — module overview lines

**Files:** Modify `CLAUDE.md` (Architecture › Module overview)

- [ ] **Step 1: IMU module line**

Replace:
```
imu_bmi270.h/.cpp   — BMI270 I2C @200 Hz; gyro bias; feeds EKF + collision + tilt. **Being replaced by a BNO055 — see Planned swap; imu_*() API kept.**
```
with:
```
imu.h / imu_bno055.cpp — BNO055 I2C @100 Hz (NDOF fusion); tilt-compensated absolute heading + tilt/pitch/roll + linear accel; feeds EKF + collision + tilt. Calibration profile + heading offset persisted to NVS.
```

- [ ] **Step 2: EKF module line**

Replace:
```
ekf_localiser.h/.cpp — 4-state EKF [x, y, θ, v]; GPS + differential-wheel-odometry fusion (gyro NOT used for heading); reverse-aware GPS heading; publishes heading events for odo_calib; ENU frame
```
with:
```
ekf_localiser.h/.cpp — 4-state EKF [x, y, θ, v]; position from GPS + wheel-odometry dead-reckoning; heading = BNO055 absolute fusion + GPS-trimmed offset; publishes heading events for odo_calib; ENU frame
```

- [ ] **Step 3: heading_fusion module line (new)**

Immediately after the `geometry.h/.cpp` line in the module overview, add:
```
heading_fusion.h/.cpp — pure (host-testable) helpers: GPS straight-segment gate, wrap-safe offset EMA, heading compose
```

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "docs(claude): module overview — imu_bno055 / heading_fusion / EKF heading"
```

---

## Task 4: CLAUDE.md — EKF localiser architecture section

**Files:** Modify `CLAUDE.md` (Architecture › EKF localiser)

- [ ] **Step 1: Replace the prediction + heading-lock prose**

In the "### EKF localiser" section, replace the three bullets describing
`ekf_predict` (differential wheel-odometry + pivot exception), the
`ekf_update_gps` heading lock, and the "Heading LOCK to GPS (2026-06-13)" bullet
— i.e. from the bullet starting `**\`ekf_predict(...)\`**` through the bullet
ending `…left heading 90° off over 10 m at creep.` — with:
```
- **`ekf_predict(v_left, v_right, dt)`** @10 Hz — heading is set directly from the **BNO055 absolute fusion + a GPS-trimmed offset** (`imu_get_heading_fused() + s_hdg_offset`); no integration. Position dead-reckons (`dx=v·sinθ·dt`, `dy=v·cosθ·dt`) between GPS fixes. If the BNO faults (`imu_is_fault()`), heading is **held** (AUTO pauses elsewhere) — there is no odometry/gyro heading fallback.
- **`ekf_update_gps()`** @~1 Hz — position **snaps** to GPS (innovation gate `max(5σ, 5 m)`, cold-start bypass). On a **straight, measurable** RTK segment (`s_hdg_turn_accum < HEADING_STRAIGHT_MAX_TURN_RAD` AND travel `> max(HEADING_FROM_GPS_MIN_DIST_M, HEADING_GPS_DIST_SIGMA_K·σ)`) the GPS travel chord trims the **heading offset** by a slow EMA (`HEADING_OFFSET_TRIM_GAIN`), reverse-corrected when reversing. The offset is persisted to NVS (`imu`/`hdgoff`), so absolute heading is correct at boot and robust to GPS outages. The offset-trim math is the pure `heading_fusion` module (host-tested).
- Reported uncertainty = last GPS σ; heading variance is fixed small (`EKF_HDG_VAR_BNO`) while the BNO is healthy.
```

- [ ] **Step 2: Commit**

```bash
git add CLAUDE.md
git commit -m "docs(claude): EKF section — BNO heading + GPS offset trim"
```

---

## Task 5: CLAUDE.md — Known Issues & TODO

**Files:** Modify `CLAUDE.md` (Known Issues & TODO)

- [ ] **Step 1: Replace the heading subsection**

Replace the entire `**Heading: the main open problem (reason for the BNO055 swap).**`
block (its three bullets about AUTO heading being wrong, gyro-pivot hunt, and the
GPS heading LOCK) with:
```
**Heading: BNO055 absolute fusion + GPS-trimmed offset.** Heading comes from the BNO055 (NDOF, tilt-compensated) plus an offset slowly trimmed from the GPS travel chord on straight RTK runs and persisted to NVS. No wheel-odometry heading, no gyro-pivot hack, no GPS-lock blend, no AUTO bootstrap creep. AUTO requires `imu_heading_is_confident()` (BNO sys+mag calibration) and pauses if confidence is lost — there is **no** fallback to a lesser heading source (recalibrate or fix the mounting instead). **Unverified on hardware** until the BNO is wired (see the implementation plans under `docs/superpowers/plans/`).
```

- [ ] **Step 2: Fix the "Pivot/turn tuning" line**

Replace:
```
**Pivot/turn tuning** (`PIVOT_*`, `GYRO_HEADING_*`, `NODE_*` in config.h) is still rough — revisit after the BNO055 gives a trustworthy heading.
```
with:
```
**Pivot/turn tuning** (`PIVOT_*`, `NODE_*` in config.h) is still rough — revisit on hardware now that the BNO055 supplies a trustworthy heading.
```

- [ ] **Step 3: Fix the collision-detection note**

Replace:
```
- **Collision detection does NOT work** — needs a full re-tune (to be redone with the BNO055 accelerometer, so no loss).
```
with:
```
- **Collision detection is DISABLED** — re-seated on the BNO055 linear accelerometer (fresh NVS baseline `baseline_v2`, capture logging via `COLL base=… jolt=…`) but left off pending a real normal-driving baseline; re-enable when a trustworthy threshold is known.
```

- [ ] **Step 4: Update the section date header**

Change `## Known Issues & TODO (2026-06-14)` — leave the date; it remains current.

- [ ] **Step 5: Commit**

```bash
git add CLAUDE.md
git commit -m "docs(claude): Known Issues — BNO heading present, collision disabled"
```

---

## Task 6: CLAUDE.md — Project Map (EKF detail, AUTO start, pivot)

**Files:** Modify `CLAUDE.md` (Project Map › EKF localiser / Odometry self-calibration)

- [ ] **Step 1: EKF localiser detail bullets**

In "### EKF localiser (detail …)", replace the bullets covering Prediction
(differential wheel-odometry + pivot exception), GPS heading LOCK, reverse-aware
heading, and heading establishment with:
```
- Prediction (`ekf_predict()`): heading = `imu_get_heading_fused() + s_hdg_offset` each tick (no integration); position dead-reckons. BNO fault → heading held (no fallback).
- GPS update (`ekf_update_gps()`): position snap + innovation gate; on a straight, measurable RTK segment the travel chord trims `s_hdg_offset` (slow EMA, reverse-corrected), persisted to NVS (`imu`/`hdgoff`).
- **Heading confidence:** `imu_heading_is_confident()` (BNO sys+mag calib) gates AUTO start and is monitored continuously; loss → PAUSE.
- **Heading events:** each clean straight-RTK trim publishes a front-facing event (`ekf_get_gps_heading_event()`) consumed by odo_calib. Reset on RESETEKF.
```

- [ ] **Step 2: AUTO heading bootstrap → confidence gate**

In "### Odometry self-calibration … + AUTO heading bootstrap", replace the
`**AUTO-start bootstrap**` bullet (the ≥2 m clearance + creep + 8 s timeout text)
with:
```
- **AUTO start (heading-confidence gate):** AUTO starts immediately when `imu_heading_is_confident()` (BNO sys+mag calibration). The old ≥2 m perimeter-clearance gate and the straight-creep heading bootstrap are **removed**. If not confident, AUTO PAUSEs and the operator drives slow loops in MANUAL to calibrate the magnetometer (prompted on the TX16S widget and the PWA "Recalibrate compass" button → `RECAL_IMU`). The confidence check is also re-evaluated every tick during AUTO.
```

- [ ] **Step 3: Motor-commands / pivot mentions**

Search the Project Map for the pivot-exception description under "Motor commands"
or "Prediction" and any remaining `GYRO_HEADING_*` / "gyro Z … during … pivot"
text; remove those clauses (heading no longer has a pivot exception). Verify:
```powershell
Select-String -Path CLAUDE.md -Pattern 'GYRO_HEADING|pivot exception|over-rotates|scrub'
```
Expected: no IMU-heading pivot references remain (spiral/cutting "scrub" unrelated — none expected).

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "docs(claude): Project Map — confidence gate, offset trim, no pivot hack"
```

---

## Task 7: CLAUDE.md — NVS Key Registry

**Files:** Modify `CLAUDE.md` (NVS Key Registry)

- [ ] **Step 1: `imu` namespace**

Replace:
```
Namespace `"imu"`: `gyrobias` — float; Z-axis gyro offset (rad/s).
```
with:
```
Namespace `"imu"`: `bnocal` — 22-byte BNO055 calibration profile blob (auto-saved when fully calibrated); `hdgoff` — float; GPS-trimmed heading offset (rad), `heading = BNO_fused + hdgoff`.
```

- [ ] **Step 2: `collision` namespace**

Replace:
```
Namespace `"collision"`: `baseline` — float; adaptive collision baseline (g).
```
with:
```
Namespace `"collision"`: `baseline_v2` — float; adaptive collision baseline (g) for the BNO linear-accel feed (the old `baseline` key from the BMI270 is abandoned).
```

- [ ] **Step 3: Commit**

```bash
git add CLAUDE.md
git commit -m "docs(claude): NVS registry — bnocal/hdgoff, collision baseline_v2"
```

---

## Task 8: README.md

**Files:** Modify `README.md`

- [ ] **Step 1: Heading-source paragraph (line ~25)**

Replace:
```
Position comes from an RTK GPS (Quectel LC29H) fused with wheel odometry.  I tried using the IMU gyro for heading, but the ground is far too bumpy - the readings were nonsense.  The machine barely slips on grass, so differential wheel odometry between GPS fixes works much better.
```
with:
```
Position comes from an RTK GPS (Quectel LC29H); between fixes it dead-reckons on wheel odometry.  Heading comes from a 9-axis BNO055 (on-chip fusion) as a tilt-compensated absolute compass, with a small offset continuously trimmed from the GPS travel direction on straight runs.  I tried integrating a bare gyro for heading first, but the ground is far too bumpy - the readings drifted badly; the magnetometer-based absolute heading is far steadier.
```

- [ ] **Step 2: Hardware IMU bullet (line ~33)**

Replace:
```
* SparkFun BMI270 IMU - used for tilt safety and collision detection (the latter currently being re-tuned).
```
with:
```
* Bosch BNO055 9-axis IMU - on-chip sensor fusion gives an absolute heading; also used for tilt safety and collision detection (the latter currently disabled pending re-tune).
```

- [ ] **Step 3: Libraries line (line ~62)**

Replace:
```
Libraries: SparkFun BMI270 and FastLED.
```
with:
```
Libraries: Adafruit BNO055 (+ Adafruit Unified Sensor) and FastLED.
```

- [ ] **Step 4: Commit**

```bash
git add README.md
git commit -m "docs(readme): BNO055 IMU, absolute heading, library list"
```

---

## Task 9: Code-comment straggler sweep

**Files:** Modify `Robo-Mower-V2/*.{h,cpp,ino}` as found (exclude `collision_detect.*`)

- [ ] **Step 1: Find remaining tokens**

```powershell
Select-String -Path Robo-Mower-V2\*.h,Robo-Mower-V2\*.cpp,Robo-Mower-V2\*.ino `
  -Pattern 'BMI270|bmi270|gyrobias|GYRO_HEADING|AUTO_BOOTSTRAP|HEADING_GPS_LOCK|imu_collect_bias|imu_get_heading\b|imu_set_heading|imu_get_bias'
```

- [ ] **Step 2: Fix each hit**

For every match (a stray comment after Plans 1–4), edit per this rule:
- A BMI270/gyro-bias mention → describe the BNO055 (NDOF fusion, absolute heading) or delete the clause.
- A removed-constant/-function mention (`GYRO_HEADING_*`, `AUTO_BOOTSTRAP_*`, `HEADING_GPS_LOCK`, `imu_collect_bias`, `imu_get_heading`/`set_heading`/`get_bias`) → delete the clause; it no longer exists.
`collision_detect.h/.cpp` were already purged in Plan 4 — do not touch them here.

- [ ] **Step 3: Verify clean**

Re-run the Step 1 command. Expected: **no matches** across code (and re-run the
pre-flight command over `CLAUDE.md`/`README.md` too — also empty).

- [ ] **Step 4: Commit**

```bash
git add Robo-Mower-V2
git commit -m "docs(code): purge remaining BMI270 / old-heading comments"
```

---

## Task 10: Final verification

**Files:** none (verification)

- [ ] **Step 1: Repo-wide token check**

```powershell
Select-String -Path CLAUDE.md,README.md,Robo-Mower-V2\*.h,Robo-Mower-V2\*.cpp,Robo-Mower-V2\*.ino `
  -Pattern 'BMI270|bmi270'
```
Expected: **no matches** anywhere.

- [ ] **Step 2: CLAUDE.md size budget**

```powershell
(Get-Item CLAUDE.md).Length
```
Expected: **< 40000** bytes (the removed planning section should leave headroom).
If over, tighten the EKF/Known-Issues prose further.

- [ ] **Step 3: Compile sanity (comments only — must still build)**

```powershell
$cli = "C:\Users\simon\Downloads\arduino-cli_1.5.0_Windows_64bit\arduino-cli.exe"
& $cli compile --fqbn esp32:esp32:esp32s3 `
  --board-options "PartitionScheme=huge_app,PSRAM=opi" Robo-Mower-V2
```
Expected: **no errors** (a comment edit must not have eaten a code line).

- [ ] **Step 4: Commit (marker)**

No code change unless Step 3 required a fix.

---

## Self-Review (against the spec)

- **§10 delete `imu_bmi270.h/.cpp`:** done in Plan 1; Task 9/10 confirm no token remains.
- **§10 CLAUDE.md — remove "Planned" section; BNO present; module overview; NVS registry; library; I²C 100 kHz; heading/EKF sections:** Tasks 1–7. ✓
- **§10 README.md:** Task 8. ✓
- **§10 all code comments referencing BMI270 / gyro-bias / old heading:** Task 9 (+ Plan 4 for `collision_detect.*`). ✓
- **40 k CLAUDE.md budget (memory `claude-md-size-limit`):** Task 10 Step 2. ✓

**Consistency:** NVS keys quoted here (`bnocal`, `hdgoff`, `baseline_v2`) match Plans 1/2/4; the library name matches Plan 1; the heading description matches Plan 2; collision wording matches Plan 4.

**Carried forward:** none — this is the final plan. After it, the only remaining work is the on-hardware verification steps flagged **[ON-HARDWARE — DEFERRED]** across Plans 1–4 (run when the BNO055 is wired).
