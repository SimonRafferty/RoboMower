# BNO055 IMU Swap — Plan 4: Collision Framework Rewrite (left disabled)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Re-seat collision detection on the BNO055 — gravity-removed linear acceleration in g (already wired in Plan 1), a 100 Hz-correct RMS window, and a *fresh* NVS baseline — and add baseline-capture logging so a real normal-driving/mowing baseline can be observed. Detection stays **disabled** (`AUTO_FAULT_RESPONSES_ENABLED 0`); do not re-enable.

**Architecture:** The detector algorithm is unchanged (adaptive-baseline magnitude-jolt). Plan 1 already feeds it `collisionDetectUpdate(ax,ay,az)` with BNO linear acceleration in g. This plan: forces a fresh baseline by renaming the NVS key (the BMI270 value's units/noise differ), resizes the RMS window for the 100 Hz task, exposes the live jolt RMS, and logs `baseline`/`jolt` periodically while driving. This plan is the sole editor of `collision_detect.h/.cpp`, including their BMI270 comment purge.

**Tech Stack:** ESP32-S3 / Arduino, ESP32 `Preferences` (NVS ns `collision`), `sys_log` ring, `arduino-cli`.

**Depends on:** Plan 1 (driver feeds BNO linear-accel-in-g to `collisionDetectUpdate`).

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `Robo-Mower-V2/collision_detect.h` | Modify | New `collisionGetJoltRms()`; doc to BNO/100 Hz/linear-accel; purge BMI270. |
| `Robo-Mower-V2/collision_detect.cpp` | Modify | Fresh NVS key; store + expose jolt RMS; purge BMI270 comments. |
| `Robo-Mower-V2/config.h` | Modify | RMS window 50→25 (250 ms at 100 Hz). |
| `Robo-Mower-V2/state_machine.cpp` | Modify | Periodic baseline-capture log while driving. |

---

## Task 1: Reset baseline + expose jolt RMS (collision_detect)

**Files:**
- Modify: `Robo-Mower-V2/collision_detect.cpp`
- Modify: `Robo-Mower-V2/collision_detect.h`

- [ ] **Step 1: Force a fresh baseline via a new NVS key**

In `Robo-Mower-V2/collision_detect.cpp`, change:
```cpp
static const char* NVS_KEY = "baseline";
```
to:
```cpp
// "baseline_v2": fresh key for the BNO055 (gravity-removed linear accel in g).
// The old "baseline" value (BMI270 raw-accel-minus-1g) has different noise and is
// intentionally abandoned, so the detector re-learns from BASELINE_DEFAULT_G.
static const char* NVS_KEY = "baseline_v2";
```

- [ ] **Step 2: Add a jolt-RMS state variable and store it**

In `collision_detect.cpp`, after the line `static float    s_savedBaseline  = BASELINE_DEFAULT_G;  // value at last NVS write`
add:
```cpp
static volatile float s_jolt_rms = 0.0f;  // latest short-window jolt RMS (g), for capture/diagnostics
```
Then in `collisionDetectUpdate()`, immediately after the line that computes
`float jolt_rms  = sqrtf(variance);` add:
```cpp
    s_jolt_rms = jolt_rms;   // publish for baseline-capture logging (Core 1 reads)
```

- [ ] **Step 3: Add the getter**

At the end of `collision_detect.cpp` (after `collisionGetBaseline()`), add:
```cpp
float collisionGetJoltRms() {
    return s_jolt_rms;  // single volatile float — atomic-enough for diagnostics
}
```

- [ ] **Step 4: Declare the getter and refresh the docs (purge BMI270)**

In `Robo-Mower-V2/collision_detect.h`:

(a) Replace the file-header lines:
```cpp
//  Replaces the physical bumper sensor (removed; GPIO6 is now the pause switch).
//  Uses BMI270 accelerometer data (fed at 200Hz from Core 0 IMU task) to detect
//  short-duration acceleration spikes that indicate obstacle contact.
```
with:
```cpp
//  Replaces the physical bumper sensor (removed; GPIO6 is now the pause switch).
//  Uses BNO055 gravity-removed linear acceleration (fed at 100Hz from the Core 0
//  IMU task, in g) to detect short-duration acceleration spikes from obstacle
//  contact. NOTE: detection is currently DISABLED via AUTO_FAULT_RESPONSES_ENABLED;
//  this module captures a baseline for future re-tuning.
```

(b) Replace the `collisionDetectUpdate` doc comment:
```cpp
/**
 * @brief Feed a new IMU sample into the collision detector.
 *        Call at 200Hz from the IMU sampling task (Core 0).
 *        Pass gravity-compensated accelerometer values in g (subtract 1g from Z).
 *
 * @param ax  Chassis X (forward) acceleration in g, gravity-compensated
 * @param ay  Chassis Y (left) acceleration in g, gravity-compensated
 * @param az  Chassis Z (up) acceleration in g, gravity-compensated (1g removed)
 */
```
with:
```cpp
/**
 * @brief Feed a new IMU sample into the collision detector.
 *        Call at 100Hz from the IMU sampling task (Core 0).
 *        Pass BNO055 linear acceleration in g (gravity already removed by fusion).
 *
 * @param ax  Surge (forward) linear acceleration in g
 * @param ay  Sway (right) linear acceleration in g
 * @param az  Heave (up) linear acceleration in g
 */
```

(c) Before the closing of the public API section (after the `collisionGetBaseline()`
declaration), add:
```cpp
/**
 * @brief Returns the latest short-window jolt RMS in g — the live signal the
 *        adaptive baseline tracks. Used for baseline-capture logging.
 */
float collisionGetJoltRms();
```

(d) In the same header, change the `Config:` reference line
`//    Config:      Robo-Mower-V2/config.h (COLLISION_* and BASELINE_* constants)`
— leave as-is (no BMI270 token). Confirm no `BMI270` tokens remain:
```powershell
Select-String -Path Robo-Mower-V2\collision_detect.h,Robo-Mower-V2\collision_detect.cpp -Pattern 'BMI270|200Hz|200 Hz|subtract 1g'
```
Expected: **no matches.**

- [ ] **Step 5: Commit**

```bash
git add Robo-Mower-V2/collision_detect.h Robo-Mower-V2/collision_detect.cpp
git commit -m "collision: BNO linear-accel source, fresh NVS baseline, expose jolt RMS"
```

---

## Task 2: Resize the RMS window for 100 Hz

**Files:**
- Modify: `Robo-Mower-V2/config.h` (~line 354)

- [ ] **Step 1: Halve the window sample count**

Replace:
```cpp
#define COLLISION_RMS_WINDOW_SAMPLES    50     // short-window RMS: 50 samples = 250ms at 200Hz
```
with:
```cpp
#define COLLISION_RMS_WINDOW_SAMPLES    25     // short-window RMS: 25 samples = 250ms at 100Hz (BNO task)
```

- [ ] **Step 2: Commit**

```bash
git add Robo-Mower-V2/config.h
git commit -m "config: collision RMS window 25 samples (250ms at the 100Hz BNO rate)"
```

---

## Task 3: Baseline-capture logging while driving

**Files:**
- Modify: `Robo-Mower-V2/state_machine.cpp` (telemetry block, after `crsf_telemetry_update(td);` ~line 3229)

- [ ] **Step 1: Add the periodic capture log**

In `Robo-Mower-V2/state_machine.cpp`, immediately after:
```cpp
        crsf_telemetry_update(td);
```
add:
```cpp
        // ── Collision baseline capture (detection DISABLED) ───────────────────
        // Detection is gated off (AUTO_FAULT_RESPONSES_ENABLED 0). Log the
        // adaptive baseline + live jolt RMS every 5 s while driving so a real
        // normal-driving/mowing baseline can be observed before re-enabling it.
        {
            static uint32_t s_coll_log_ms = 0;
            bool driving = (g_state == STATE_MANUAL || g_state == STATE_AUTO_MOWING);
            if (driving && (millis() - s_coll_log_ms) > 5000) {
                s_coll_log_ms = millis();
                char cline[SYS_LOG_MAX_LEN];
                snprintf(cline, sizeof(cline), "COLL base=%.3fg jolt=%.3fg",
                         (double)collisionGetBaseline(), (double)collisionGetJoltRms());
                sys_log_push(cline);
            }
        }
```

- [ ] **Step 2: Commit**

```bash
git add Robo-Mower-V2/state_machine.cpp
git commit -m "sm: log collision baseline + jolt RMS every 5s while driving (capture)"
```

---

## Task 4: Confirm detection stays disabled

**Files:** none (verification)

- [ ] **Step 1: Confirm the gate is still off**

```powershell
Select-String -Path Robo-Mower-V2\config.h -Pattern 'AUTO_FAULT_RESPONSES_ENABLED'
```
Expected: `#define AUTO_FAULT_RESPONSES_ENABLED  0`. Do **not** change it.

- [ ] **Step 2: Confirm collision still only triggers behind the gate**

```powershell
Select-String -Path Robo-Mower-V2\state_machine.cpp -Pattern 'collisionDetected\(\)'
```
Expected: every `collisionDetected()` use sits behind `AUTO_FAULT_RESPONSES_ENABLED`
(and `obs_armed`) — unchanged from before this plan. No edits required.

---

## Task 5: Compile

**Files:** none (verification)

- [ ] **Step 1: Compile**

Run:
```powershell
$cli = "C:\Users\simon\Downloads\arduino-cli_1.5.0_Windows_64bit\arduino-cli.exe"
& $cli compile --fqbn esp32:esp32:esp32s3 `
  --board-options "PartitionScheme=huge_app,PSRAM=opi" Robo-Mower-V2
```
Expected: **no errors.** Likely fix points: `collisionGetJoltRms()` declared in the
header before use; `SYS_LOG_MAX_LEN` already in scope in `state_machine.cpp` (it is,
used by other `sys_log_push` call sites).

- [ ] **Step 2: [ON-HARDWARE — DEFERRED] Observe the baseline**

Drive normally/mow with the BNO wired: the system log (PWA Dashboard / serial)
shows `COLL base=… jolt=…` every 5 s. Record a representative normal-driving
baseline; this informs the threshold when collision is eventually re-enabled.

- [ ] **Step 3: Commit (marker)**

No code change unless Step 1 required fixes (commit those under their task).

---

## Self-Review (against the spec)

- **§9 feed BNO linear acceleration (gravity removed) in g:** Plan 1 (driver) + Task 1 docs. ✓
- **§9 reset the NVS collision baseline:** Task 1 Step 1 (new key `baseline_v2`). ✓
- **§9 baseline-capture logging (visible over BLE/serial):** Task 1 (jolt getter) + Task 3 (sys_log). ✓
- **§9 detection stays off (`AUTO_FAULT_RESPONSES_ENABLED 0`):** Task 4. ✓
- **100 Hz timing correctness:** Task 2 (RMS window). ✓
- **BMI270 purge in collision files:** Task 1 Step 4. ✓ (Plan 5 excludes `collision_detect.*`.)

**Type/name consistency:** `collisionGetJoltRms()` declared (Task 1d) and defined (Task 1c) identically; NVS ns `collision` key `baseline_v2` (CLAUDE.md NVS registry updated in Plan 5).

**Carried to later plans:** CLAUDE.md NVS-registry note for `baseline_v2` and the "collision re-tune" status text are Plan 5.
