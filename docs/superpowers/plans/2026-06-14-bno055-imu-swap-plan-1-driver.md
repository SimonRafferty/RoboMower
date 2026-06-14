# BNO055 IMU Swap — Plan 1: Driver Foundation & API

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the BMI270 driver with a BNO055 driver behind a device-neutral `imu.h` API, so the firmware compiles and runs with the BNO055 supplying tilt, gyro-Z, accel (and a new fused absolute heading + calibration status), while the existing EKF heading path keeps working unchanged.

**Architecture:** A new `imu.h` declares the stable `imu_*()` contract every consumer already uses. `imu_bno055.cpp` implements it against `Adafruit_BNO055` (NDOF mode, I²C @100 kHz, hardware axis-remap to the robot frame), runs a 100 Hz Core-0 sampling task, persists the 22-byte calibration profile + a heading offset to NVS (Preferences, namespace `imu`), and feeds gravity-removed linear acceleration (in g) to the collision detector. The BMI270 driver and all references to it are deleted.

**Tech Stack:** ESP32-S3 / Arduino, FreeRTOS, `Adafruit_BNO055` + `Adafruit_Unified_Sensor`, ESP32 `Preferences` (NVS), `arduino-cli` for compile verification.

**Scope note:** The part is **in hand but not wired**. The executable check for every task is **compilation** (`arduino-cli`). Steps that require the physical sensor (I²C comms, axis-sign confirmation) are marked **[ON-HARDWARE — DEFERRED]** and are verified later under Plan 2's bring-up; they are written now with sensible defaults and a documented check.

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `Robo-Mower-V2/imu.h` | Create | Device-neutral `imu_*()` API (the stable contract). |
| `Robo-Mower-V2/imu_bno055.cpp` | Create | BNO055 driver: init, axis remap, 100 Hz task, getters, NVS cal, collision feed. |
| `Robo-Mower-V2/imu_bmi270.h` | Delete | Old API header (superseded by `imu.h`). |
| `Robo-Mower-V2/imu_bmi270.cpp` | Delete | Old driver. |
| `Robo-Mower-V2/config.h` | Modify | BNO055 I²C address/clock, calib thresholds, axis-remap defaults; remove BMI270-only notes. |
| `Robo-Mower-V2/Robo-Mower-V2.ino` | Modify | `#include "imu.h"`, `imu_bmi270_init()` → `imu_init()`. |
| `Robo-Mower-V2/ble_server.cpp` | Modify | `#include "imu.h"`; diagnostics block uses fused heading + calib (drops removed getters). |
| `Robo-Mower-V2/node_follower.cpp` | Modify | `#include "imu.h"`. |
| `Robo-Mower-V2/state_machine.cpp` | Modify | `#include "imu.h"`; remove `imu_collect_bias()` call. |
| `Robo-Mower-V2/safety.cpp` | Modify | `#include "imu.h"`. |
| `Robo-Mower-V2/safety.h` | Modify | Comment reference `imu_bmi270_init()` → `imu_init()`. |

**Preserved API (unchanged signatures):** `imu_get_gz_rads`, `imu_get_tilt_rad`, `imu_get_pitch_rad`, `imu_get_roll_rad`, `imu_get_accel`, `imu_is_present`, `imu_is_fault`.
**New:** `imu_init` (rename), `imu_get_heading_fused`, `imu_heading_is_confident`, `imu_get_calib_status`, `imu_recalibrate`.
**Removed:** `imu_bmi270_init`, `imu_collect_bias`, `imu_get_heading`, `imu_set_heading`, `imu_get_bias`.

> `imu_recalibrate()` and `imu_heading_is_confident()` are declared and implemented here (the driver owns calibration); they are *consumed* by Plans 2–3. Declaring them now keeps the API complete and avoids a second header edit.

---

## Task 1: Install library and add BNO055 config

**Files:**
- Modify: `Robo-Mower-V2/config.h` (IMU section, ~lines 84–114)

- [ ] **Step 1: Install the Adafruit BNO055 library**

Run:
```powershell
$cli = "C:\Users\simon\Downloads\arduino-cli_1.5.0_Windows_64bit\arduino-cli.exe"
& $cli lib install "Adafruit BNO055"
```
Expected: installs `Adafruit BNO055` and its dependency `Adafruit Unified Sensor` (and `Adafruit BusIO`). Output ends with `Installed`.

- [ ] **Step 2: Replace the BMI270 I²C address line and add BNO055 constants**

In `Robo-Mower-V2/config.h`, find:
```cpp
// IMU I2C address (BMI270 SA0=VCC → 0x69)
#define IMU_I2C_ADDRESS           0x69   // SparkFun BMI270 I2C address (SDO/SA0=VCC)
```
Replace with:
```cpp
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

// Heading-confidence gate: BNO calibration levels (each 0..3) required for the
// absolute heading to be trusted (NDOF: system + magnetometer).
#define IMU_CALIB_SYS_MIN         2
#define IMU_CALIB_MAG_MIN         2
```

- [ ] **Step 3: Confirm `TILT_EMA_ALPHA` still exists for the driver**

Run:
```powershell
Select-String -Path Robo-Mower-V2\config.h -Pattern "TILT_EMA_ALPHA"
```
Expected: one match (the driver reuses it as a light smoothing factor). If absent, add `#define TILT_EMA_ALPHA 0.1f` near the other IMU constants.

- [ ] **Step 4: Commit**

```bash
git add Robo-Mower-V2/config.h
git commit -m "config: BNO055 I2C/calib/axis-remap constants (replaces BMI270 0x69)"
```

---

## Task 2: Create the device-neutral `imu.h` API

**Files:**
- Create: `Robo-Mower-V2/imu.h`

- [ ] **Step 1: Write `imu.h`**

Create `Robo-Mower-V2/imu.h` with exactly:
```cpp
#pragma once
#include <Arduino.h>
#include <stdint.h>

// ══════════════════════════════════════════════════════════════════════════════
//  imu.h — RoboMower IMU interface (device-neutral)
//
//  Implemented by imu_bno055.cpp against a Bosch BNO055 (9-axis, on-chip fusion)
//  on I2C Wire (SDA=GPIO8, SCL=GPIO9) at 100 kHz. A 100 Hz FreeRTOS task on
//  Core 0 publishes fused heading, gyro Z, tilt/pitch/roll, and acceleration.
//
//  Units:
//    - Heading / pitch / roll / tilt: radians (heading wrapped ±π, CW from North)
//    - Gyro rate: rad/s, CW-positive
//    - Accel (imu_get_accel): g, gravity-removed (BNO linear acceleration)
// ══════════════════════════════════════════════════════════════════════════════

/** Initialise the BNO055 (NDOF, axis remap, NVS calibration) and start the
 *  100 Hz Core-0 sampling task. @return true on success. */
bool imu_init();

/** Fused, tilt-compensated absolute heading (rad, ±π, CW from North). */
float imu_get_heading_fused();

/** Gyro Z yaw rate, CW-positive (rad/s). */
float imu_get_gz_rads();

/** EMA-filtered tilt angle from vertical (rad; 0 = flat). */
float imu_get_tilt_rad();

/** Pitch (rad, nose-up positive). */
float imu_get_pitch_rad();

/** Roll (rad, right-side-down positive). */
float imu_get_roll_rad();

/** Gravity-removed acceleration in g (surge fwd+, sway right+, heave up+). */
void imu_get_accel(float *ax, float *ay, float *az);

/** True if the BNO055 was initialised and the task is running. */
bool imu_is_present();

/** True if I2C contact has been lost for > 200 ms. */
bool imu_is_fault();

/** Packed BNO calibration status: bits [7:6]=sys [5:4]=gyro [3:2]=accel [1:0]=mag,
 *  each 0..3. */
uint8_t imu_get_calib_status();

/** True when the absolute heading is trustworthy (NDOF: sys ≥ IMU_CALIB_SYS_MIN
 *  AND mag ≥ IMU_CALIB_MAG_MIN). */
bool imu_heading_is_confident();

/** Force a fresh magnetometer recalibration: discard the stored NVS profile and
 *  relearn from motion. The new profile is auto-saved once fully calibrated. */
void imu_recalibrate();
```

- [ ] **Step 2: Commit**

```bash
git add Robo-Mower-V2/imu.h
git commit -m "imu: add device-neutral imu.h API (BNO055 contract)"
```

---

## Task 3: Implement `imu_bno055.cpp`

**Files:**
- Create: `Robo-Mower-V2/imu_bno055.cpp`

- [ ] **Step 1: Write `imu_bno055.cpp`**

Create `Robo-Mower-V2/imu_bno055.cpp` with exactly:
```cpp
// ══════════════════════════════════════════════════════════════════════════════
//  imu_bno055.cpp — RoboMower BNO055 IMU implementation
//
//  NDOF mode (9-axis on-chip fusion). I2C @100 kHz. 100 Hz Core-0 task publishes
//  fused absolute heading, gyro Z (CW+), tilt/pitch/roll (from Euler), and
//  gravity-removed linear acceleration (in g) — the last fed to the collision
//  detector. The 22-byte calibration profile and a placeholder heading offset
//  are persisted to NVS (namespace "imu").
//
//  Axis remap (IMU_AXIS_REMAP_*) puts the sensor in the robot frame, so no
//  manual per-axis sign flips are needed here. The gyro sign is negated once to
//  convert the BNO right-hand-rule (CCW+) yaw rate to the firmware CW+ convention.
//  All signs/zero are confirmed at bring-up (Plan 2).
// ══════════════════════════════════════════════════════════════════════════════

#include "imu.h"
#include "config.h"
#include "geometry.h"          // wrapAngle()
#include "collision_detect.h"  // collisionDetectUpdate()

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

// ── NVS ───────────────────────────────────────────────────────────────────────
static const char* NVS_NAMESPACE = "imu";
static const char* NVS_KEY_CAL   = "bnocal";   // 22-byte BNO offsets blob
static const char* NVS_KEY_HDG   = "hdgoff";   // float heading offset (used in Plan 2)

static const float G_PER_MS2 = 1.0f / 9.80665f;  // m/s² → g for collision feed

// ── Module state ──────────────────────────────────────────────────────────────
static Adafruit_BNO055  s_bno(-1, IMU_I2C_ADDRESS, &Wire);
static portMUX_TYPE     s_spinlock      = portMUX_INITIALIZER_UNLOCKED;
static volatile float   s_heading       = 0.0f;   // rad, ±π, CW from North
static volatile float   s_gz_rads       = 0.0f;   // rad/s, CW+
static volatile float   s_tilt_rad      = 0.0f;
static volatile float   s_pitch_rad     = 0.0f;
static volatile float   s_roll_rad      = 0.0f;
static volatile float   s_accel_x       = 0.0f;   // g, surge fwd+
static volatile float   s_accel_y       = 0.0f;   // g, sway right+
static volatile float   s_accel_z       = 0.0f;   // g, heave up+
static volatile uint8_t s_calib         = 0;      // packed sys/gyro/accel/mag
static volatile bool    s_imu_fault     = false;
static volatile bool    s_initialized   = false;
static volatile bool    s_force_recal   = false;  // imu_recalibrate() request
static volatile bool    s_profile_saved = false;  // a good profile is persisted
static volatile uint32_t s_last_valid_ms = 0;

static TaskHandle_t     s_task_handle   = nullptr;

static void imu_task(void* pv);

// ── NVS helpers ───────────────────────────────────────────────────────────────
static bool load_cal_from_nvs(uint8_t out[22]) {
    Preferences p;
    p.begin(NVS_NAMESPACE, /*readOnly=*/true);
    size_t n = p.getBytes(NVS_KEY_CAL, out, 22);
    p.end();
    return n == 22;
}

static void save_cal_to_nvs(const uint8_t buf[22]) {
    Preferences p;
    p.begin(NVS_NAMESPACE, /*readOnly=*/false);
    p.putBytes(NVS_KEY_CAL, buf, 22);
    p.end();
}

static void clear_cal_in_nvs() {
    Preferences p;
    p.begin(NVS_NAMESPACE, /*readOnly=*/false);
    p.remove(NVS_KEY_CAL);
    p.end();
}

// ══════════════════════════════════════════════════════════════════════════════
//  Public API
// ══════════════════════════════════════════════════════════════════════════════

bool imu_init() {
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(IMU_I2C_CLOCK_HZ);  // 100 kHz — BNO clock-stretch quirk

    // Retry begin() to tolerate transient power-up I2C noise.
    bool ok = false;
    for (int attempt = 1; attempt <= 5; attempt++) {
        if (s_bno.begin(OPERATION_MODE_NDOF)) { ok = true; break; }
        DBG_PRINTF("[IMU] BNO055 begin() attempt %d/5 failed\n", attempt);
        delay(100);
    }
    if (!ok) {
        DBG_PRINTLN("[IMU] BNO055 not detected");
        return false;
    }

    s_bno.setExtCrystalUse(true);

    // Axis remap to the robot frame (confirmed at bring-up).
    s_bno.setAxisRemap((Adafruit_BNO055::adafruit_bno055_axis_remap_config_t)IMU_AXIS_REMAP_CONFIG);
    s_bno.setAxisSign((Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t)IMU_AXIS_REMAP_SIGN);

    // Restore a saved calibration profile if present (skips re-learning the mag).
    uint8_t cal[22];
    if (load_cal_from_nvs(cal)) {
        s_bno.setSensorOffsets(cal);
        s_profile_saved = true;
        DBG_PRINTLN("[IMU] BNO055 calibration profile restored from NVS");
    } else {
        DBG_PRINTLN("[IMU] No stored BNO055 calibration — drive loops to calibrate");
    }

    s_last_valid_ms = millis();
    s_initialized   = true;

    BaseType_t rc = xTaskCreatePinnedToCore(
        imu_task, "imu_task", IMU_TASK_STACK, nullptr,
        IMU_TASK_PRIORITY, &s_task_handle, 0 /*Core 0*/);
    if (rc != pdPASS) {
        DBG_PRINTLN("[IMU] task creation failed");
        s_initialized = false;
        return false;
    }
    DBG_PRINTF("[IMU] BNO055 initialised — %d Hz task on Core 0\n", IMU_TASK_HZ);
    return true;
}

float imu_get_heading_fused() {
    float h; taskENTER_CRITICAL(&s_spinlock); h = s_heading; taskEXIT_CRITICAL(&s_spinlock); return h;
}
float imu_get_gz_rads() {
    float g; taskENTER_CRITICAL(&s_spinlock); g = s_gz_rads; taskEXIT_CRITICAL(&s_spinlock); return g;
}
float imu_get_tilt_rad() {
    float t; taskENTER_CRITICAL(&s_spinlock); t = s_tilt_rad; taskEXIT_CRITICAL(&s_spinlock); return t;
}
float imu_get_pitch_rad() {
    float p; taskENTER_CRITICAL(&s_spinlock); p = s_pitch_rad; taskEXIT_CRITICAL(&s_spinlock); return p;
}
float imu_get_roll_rad() {
    float r; taskENTER_CRITICAL(&s_spinlock); r = s_roll_rad; taskEXIT_CRITICAL(&s_spinlock); return r;
}
void imu_get_accel(float *ax, float *ay, float *az) {
    taskENTER_CRITICAL(&s_spinlock);
    *ax = s_accel_x; *ay = s_accel_y; *az = s_accel_z;
    taskEXIT_CRITICAL(&s_spinlock);
}
bool imu_is_present()  { return s_initialized; }
bool imu_is_fault()    { return s_imu_fault; }
uint8_t imu_get_calib_status() { return s_calib; }

bool imu_heading_is_confident() {
    uint8_t c = s_calib;
    uint8_t sys = (c >> 6) & 0x03;
    uint8_t mag =  c       & 0x03;
    return (sys >= IMU_CALIB_SYS_MIN) && (mag >= IMU_CALIB_MAG_MIN);
}

void imu_recalibrate() {
    clear_cal_in_nvs();
    s_profile_saved = false;
    s_force_recal   = true;   // task re-enters NDOF to drop the live profile
}

// ══════════════════════════════════════════════════════════════════════════════
//  100 Hz sampling task (Core 0)
// ══════════════════════════════════════════════════════════════════════════════
static void imu_task(void* pv) {
    TickType_t wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / IMU_TASK_HZ);

    while (true) {
        vTaskDelayUntil(&wake, period);

        // Handle a recalibration request: re-init NDOF (drops the live profile).
        if (s_force_recal) {
            s_force_recal = false;
            s_bno.setMode(OPERATION_MODE_CONFIG);
            delay(25);
            s_bno.setMode(OPERATION_MODE_NDOF);
            DBG_PRINTLN("[IMU] recalibration started — drive slow loops");
        }

        sensors_event_t ev;
        bool ok = s_bno.getEvent(&ev, Adafruit_BNO055::VECTOR_EULER);
        if (ok) {
            // Euler: x = heading (deg, 0..360), y = roll, z = pitch.
            float hdg_rad = wrapAngle(ev.orientation.x * (float)DEG_TO_RAD);

            imu::Vector<3> gyro = s_bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);   // deg/s
            imu::Vector<3> lin  = s_bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); // m/s²

            // BNO right-hand-rule yaw (CCW+) → firmware CW+ : negate.
            float gz = -(float)gyro.z() * (float)DEG_TO_RAD;

            // Euler pitch/roll already in the robot frame via axis remap.
            float pitch = ev.orientation.z * (float)DEG_TO_RAD;
            float roll  = ev.orientation.y * (float)DEG_TO_RAD;
            float tilt  = acosf(cosf(pitch) * cosf(roll));  // angle from vertical

            // Linear accel (gravity removed) → g, robot frame from axis remap.
            float ax = (float)lin.x() * G_PER_MS2;
            float ay = (float)lin.y() * G_PER_MS2;
            float az = (float)lin.z() * G_PER_MS2;

            uint8_t sys, gy, ac, mg;
            s_bno.getCalibration(&sys, &gy, &ac, &mg);
            uint8_t packed = (uint8_t)((sys << 6) | (gy << 4) | (ac << 2) | mg);

            const float a = TILT_EMA_ALPHA;
            taskENTER_CRITICAL(&s_spinlock);
            s_heading   = hdg_rad;
            s_gz_rads   = gz;
            s_pitch_rad = s_pitch_rad * (1.0f - a) + pitch * a;
            s_roll_rad  = s_roll_rad  * (1.0f - a) + roll  * a;
            s_tilt_rad  = s_tilt_rad  * (1.0f - a) + tilt  * a;
            s_accel_x   = ax; s_accel_y = ay; s_accel_z = az;
            s_calib     = packed;
            taskEXIT_CRITICAL(&s_spinlock);

            // Feed gravity-removed acceleration (g) to the collision detector.
            collisionDetectUpdate(ax, ay, az);

            // Auto-save the calibration profile once fully calibrated and changed.
            if (!s_profile_saved && s_bno.isFullyCalibrated()) {
                uint8_t buf[22];
                if (s_bno.getSensorOffsets(buf)) {
                    save_cal_to_nvs(buf);
                    s_profile_saved = true;
                    DBG_PRINTLN("[IMU] BNO055 calibration profile saved to NVS");
                }
            }

            s_last_valid_ms = millis();
            s_imu_fault = false;
        } else if ((millis() - s_last_valid_ms) > 200UL) {
            s_imu_fault = true;
        }
    }
}
```

- [ ] **Step 2: Commit**

```bash
git add Robo-Mower-V2/imu_bno055.cpp
git commit -m "imu: BNO055 driver (NDOF, 100Hz task, NVS cal, collision feed)"
```

---

## Task 4: Delete BMI270 and rewire includes / init

**Files:**
- Delete: `Robo-Mower-V2/imu_bmi270.h`, `Robo-Mower-V2/imu_bmi270.cpp`
- Modify: `Robo-Mower-V2/Robo-Mower-V2.ino`, `node_follower.cpp`, `safety.cpp`, `safety.h`, `state_machine.cpp`

- [ ] **Step 1: Delete the BMI270 driver files**

```bash
git rm Robo-Mower-V2/imu_bmi270.h Robo-Mower-V2/imu_bmi270.cpp
```

- [ ] **Step 2: Repoint all includes**

In each of `Robo-Mower-V2/Robo-Mower-V2.ino`, `node_follower.cpp`, `safety.cpp`, `ble_server.cpp`, `state_machine.cpp`, replace:
```cpp
#include "imu_bmi270.h"
```
with:
```cpp
#include "imu.h"
```
Verify none remain:
```powershell
Select-String -Path Robo-Mower-V2\*.cpp,Robo-Mower-V2\*.ino,Robo-Mower-V2\*.h -Pattern 'imu_bmi270'
```
Expected after edits: only `safety.h` (a comment) still matches — fixed in Step 4.

- [ ] **Step 3: Rename the init call**

In `Robo-Mower-V2/Robo-Mower-V2.ino`, replace:
```cpp
    if (!imu_bmi270_init()) {
        sys_log_push("IMU: BMI270 not detected — check I2C wiring");
    }
```
with:
```cpp
    if (!imu_init()) {
        sys_log_push("IMU: BNO055 not detected — check I2C wiring");
    }
```

- [ ] **Step 4: Fix the `safety.h` comment**

In `Robo-Mower-V2/safety.h`, replace `imu_bmi270_init()` with `imu_init()` in the comment near line 42.

- [ ] **Step 5: Remove the gyro-bias collection call**

`imu_collect_bias()` no longer exists (the BNO self-calibrates). In `Robo-Mower-V2/state_machine.cpp` find (≈ line 1761):
```cpp
            imu_collect_bias();
```
Remove that line and any now-empty enclosing `if`/comment that referenced gyro-bias collection. If it sits inside an INIT-state block whose only purpose was bias collection, remove the block; otherwise just delete the single call. Verify:
```powershell
Select-String -Path Robo-Mower-V2\*.cpp -Pattern 'imu_collect_bias|imu_get_bias|imu_get_heading\b|imu_set_heading'
```
Expected: the only remaining matches are in `ble_server.cpp` (fixed in Task 5). No matches for `imu_collect_bias`.

- [ ] **Step 6: Commit**

```bash
git add -A Robo-Mower-V2
git commit -m "imu: delete BMI270 driver, repoint includes to imu.h, imu_init()"
```

---

## Task 5: Update BLE diagnostics for the new getters

**Files:**
- Modify: `Robo-Mower-V2/ble_server.cpp` (`build_diag_json()`, ~lines 653–711)

- [ ] **Step 1: Replace the IMU read block**

In `Robo-Mower-V2/ble_server.cpp` `build_diag_json()`, replace:
```cpp
    // IMU
    float imu_hdg   = imu_get_heading();
    float imu_gz    = imu_get_gz_rads();
    float imu_bias  = imu_get_bias();
    float imu_pitch = imu_get_pitch_rad() * (180.0f / M_PI);
    float imu_roll  = imu_get_roll_rad()  * (180.0f / M_PI);
    float imu_ax, imu_ay, imu_az;
    imu_get_accel(&imu_ax, &imu_ay, &imu_az);
    bool  imu_ok    = imu_is_present() && !imu_is_fault();
```
with:
```cpp
    // IMU
    float imu_hdg   = imu_get_heading_fused();
    float imu_gz    = imu_get_gz_rads();
    uint8_t imu_cal = imu_get_calib_status();   // packed sys/gyro/accel/mag
    float imu_pitch = imu_get_pitch_rad() * (180.0f / M_PI);
    float imu_roll  = imu_get_roll_rad()  * (180.0f / M_PI);
    float imu_ax, imu_ay, imu_az;
    imu_get_accel(&imu_ax, &imu_ay, &imu_az);
    bool  imu_ok    = imu_is_present() && !imu_is_fault();
```

- [ ] **Step 2: Replace the IMU JSON field and its argument**

Find the IMU object in the `snprintf` format string:
```cpp
        "\"imu\":{\"ok\":%d,\"hdg\":%.4f,\"gz\":%.4f,\"bias\":%.5f,"
        "\"pitch\":%.1f,\"roll\":%.1f,"
```
Replace `\"bias\":%.5f` with `\"cal\":%d`:
```cpp
        "\"imu\":{\"ok\":%d,\"hdg\":%.4f,\"gz\":%.4f,\"cal\":%d,"
        "\"pitch\":%.1f,\"roll\":%.1f,"
```
Then locate the matching argument list (the arguments are positional, in the same order as the format). Find where `imu_ok, imu_hdg, imu_gz, imu_bias,` are passed and change `imu_bias` to `imu_cal`:
```cpp
        imu_ok, imu_hdg, imu_gz, (int)imu_cal,
```
(If the existing argument grouping differs, change only the single `imu_bias` token to `(int)imu_cal` — do not reorder the others.)

- [ ] **Step 3: Verify no removed getters remain**

```powershell
Select-String -Path Robo-Mower-V2\*.cpp -Pattern 'imu_get_heading\b|imu_get_bias|imu_set_heading|imu_collect_bias'
```
Expected: **no matches.**

- [ ] **Step 4: Commit**

```bash
git add Robo-Mower-V2/ble_server.cpp
git commit -m "ble: diagnostics use BNO fused heading + calib (drop removed getters)"
```

---

## Task 6: Compile the full firmware

**Files:** none (verification task)

- [ ] **Step 1: Compile with the required board options**

Run:
```powershell
$cli = "C:\Users\simon\Downloads\arduino-cli_1.5.0_Windows_64bit\arduino-cli.exe"
& $cli compile --fqbn esp32:esp32:esp32s3 `
  --board-options "PartitionScheme=huge_app,PSRAM=opi" Robo-Mower-V2
```
Expected: `Sketch uses … bytes` with **no errors**. Resolve any compile error before continuing (common: a stray `imu_bmi270` reference, a missing `#include "imu.h"`, or an axis-remap enum cast typo).

- [ ] **Step 2: Confirm the BMI270 is fully gone**

```powershell
Select-String -Path Robo-Mower-V2\*.cpp,Robo-Mower-V2\*.h,Robo-Mower-V2\*.ino -Pattern 'bmi270|BMI270'
```
Expected: **no matches** (docs purge of CLAUDE.md/README is Plan 5; code must be clean now).

- [ ] **Step 3: [ON-HARDWARE — DEFERRED] Bring-up checks (Plan 2)**

When the BNO055 is wired, these confirm the defaults written above:
- `imu_is_present()` true; serial shows `BNO055 initialised`.
- Tilt the robot: surge (nose-down +), sway (right-down +), heave signs correct; if not, adjust `IMU_AXIS_REMAP_*`.
- Yaw the robot clockwise (viewed from above): `imu_get_gz_rads()` > 0 and `imu_get_heading_fused()` increases; if reversed, flip the gyro sign / heading sign.
- Drive slow loops: `imu_get_calib_status()` mag/sys climb to 3; profile auto-saves to NVS.

- [ ] **Step 4: Commit (no-op marker)**

No code change in this task. If Steps 1–2 required fixes, they were committed in their own tasks. Proceed to Plan 2.

---

## Self-Review (against the spec)

- **Spec §4.1/§4.2 (file structure, library):** Tasks 1–4. ✓
- **Spec §4.3/§4.4 (preserved + new + removed API):** Tasks 2, 3, 5. ✓
- **Spec §4.5 (I²C 100 kHz, axis remap, NDOF, 100 Hz task, Euler tilt):** Tasks 1, 3. ✓ (NDOF-only; no IMUPLUS — per Non-Goals.)
- **Spec §6 (NVS `bnocal`/`hdgoff`, no gyro-bias collection):** Task 3 (`hdgoff` key reserved for Plan 2), Task 4 Step 5. ✓
- **Spec §7 (`imu_heading_is_confident`):** Task 3. ✓ (consumed by Plan 2.)
- **Spec §8.1 (`imu_recalibrate` / `RECAL_IMU` driver side):** Task 3. ✓ (BLE command wiring is Plan 3.)
- **Spec §9 (collision fed BNO linear-accel in g):** Task 3 (`collisionDetectUpdate(ax,ay,az)`). ✓ (baseline reset + capture logging is Plan 4.)
- **Spec §10 (BMI270 code purge):** Tasks 4, 6 Step 2. ✓ (CLAUDE.md/README purge is Plan 5.)

**Carried to later plans (intentionally not in Plan 1):** heading EKF rewrite + `hdgoff` use + bootstrap removal (Plan 2); telemetry calib byte + Lua + PWA + `RECAL_IMU` BLE dispatch (Plan 3); collision baseline reset + capture logging (Plan 4); CLAUDE.md/README/comment purge (Plan 5).
