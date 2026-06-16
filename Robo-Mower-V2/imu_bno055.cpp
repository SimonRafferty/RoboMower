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
#include "sys_log.h"           // operator-visible calibration-save confirmation

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#include <cstring>

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
// The status flags below are a deliberately lock-free cross-core handshake: each
// is a single aligned byte/word (atomic on this target), so `volatile` visibility
// is sufficient and a spinlock would only add cost. Core 1 sets s_force_recal /
// s_profile_saved (imu_recalibrate); Core 0 (imu_task) clears / reads them.
static volatile bool    s_imu_fault     = false;
static volatile bool    s_initialized   = false;
static volatile bool    s_force_recal   = false;  // imu_recalibrate() request
static volatile bool    s_save_request  = false;  // imu_request_save() — performed on Core 0
static volatile bool    s_profile_saved = false;  // a good profile is persisted
static volatile bool    s_profile_loaded_boot = false;  // a profile was restored at boot
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

// Persist the 22-byte profile AND verify it read back identically from a freshly
// opened handle. The old code assumed the write stuck (the reported bug: "saved"
// logged but absent on reboot). The read-back tells us definitively whether the
// blob reached flash, so a silent write failure can no longer masquerade as
// success — s_profile_saved is only set when this returns true.
static bool save_cal_to_nvs(const uint8_t buf[22]) {
    {
        Preferences p;
        if (!p.begin(NVS_NAMESPACE, /*readOnly=*/false)) {
            DBG_PRINTLN("[IMU] cal save: NVS begin(rw) failed");
            return false;
        }
        size_t w = p.putBytes(NVS_KEY_CAL, buf, 22);
        p.end();   // Preferences::end() flushes/commits the namespace
        if (w != 22) {
            DBG_PRINTF("[IMU] cal save: putBytes wrote %u/22 bytes\n", (unsigned)w);
            return false;
        }
    }
    uint8_t check[22] = {0};
    Preferences p;
    p.begin(NVS_NAMESPACE, /*readOnly=*/true);
    size_t r = p.getBytes(NVS_KEY_CAL, check, 22);
    p.end();
    if (r != 22 || memcmp(check, buf, 22) != 0) {
        DBG_PRINTF("[IMU] cal save: read-back FAILED (r=%u)\n", (unsigned)r);
        return false;
    }
    DBG_PRINTLN("[IMU] cal save: read-back verified OK");
    return true;
}

static void clear_cal_in_nvs() {
    Preferences p;
    p.begin(NVS_NAMESPACE, /*readOnly=*/false);
    p.remove(NVS_KEY_CAL);
    p.end();
}

// Read the BNO055's 22 calibration-offset registers DIRECTLY (in CONFIG mode),
// bypassing Adafruit's getSensorOffsets() — that one only returns data when
// isFullyCalibrated() is true, which in NDOF requires sys==3. A magnetically-noisy
// machine never reaches sys==3, so the profile would otherwise never save. The
// CONFIG↔NDOF switch restarts fusion (live calib status briefly drops then
// re-validates from these offsets). Called once per session when gyro/accel/mag=3.
static bool read_offsets_raw(uint8_t out[22]) {
    s_bno.setMode(OPERATION_MODE_CONFIG);
    delay(25);
    bool ok = false;
    Wire.beginTransmission((uint8_t)IMU_I2C_ADDRESS);
    Wire.write((uint8_t)0x55);  // ACCEL_OFFSET_X_LSB_ADDR — first of 22 offset regs
    if (Wire.endTransmission() == 0) {
        uint8_t got = Wire.requestFrom((uint8_t)IMU_I2C_ADDRESS, (uint8_t)22);
        uint8_t i = 0;
        while (i < 22 && Wire.available()) out[i++] = Wire.read();
        ok = (got == 22 && i == 22);
    }
    s_bno.setMode(OPERATION_MODE_NDOF);
    delay(20);
    return ok;
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
        s_profile_saved       = true;
        s_profile_loaded_boot = true;
        // Log a checksum so consecutive boots can be compared from the BLE log:
        // a changing/zero sum across boots means the profile is not persisting.
        uint16_t sum = 0;
        for (int i = 0; i < 22; i++) sum += cal[i];
        DBG_PRINTF("[IMU] BNO055 calibration profile restored from NVS (chk=0x%04X)\n", sum);
        sys_log_push("IMU: BNO055 calibration profile restored from NVS");
    } else {
        DBG_PRINTLN("[IMU] No stored BNO055 calibration — drive loops to calibrate");
        sys_log_push("IMU: no stored BNO055 calibration (heading starts relative)");
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
    uint8_t gyro  = (c >> 4) & 0x03;
    uint8_t accel = (c >> 2) & 0x03;
    uint8_t mag   =  c       & 0x03;
    // The BNO055 `sys` aggregate is unreliable (often 0 even when the three real
    // sensors are fully calibrated), so gate on mag/gyro/accel directly.
    return (mag   >= IMU_CALIB_MAG_MIN)
        && (gyro  >= IMU_CALIB_GYRO_MIN)
        && (accel >= IMU_CALIB_ACCEL_MIN);
}

void imu_recalibrate() {
    clear_cal_in_nvs();
    s_profile_saved = false;
    s_force_recal   = true;   // task re-enters NDOF to drop the live profile
}

bool imu_profile_loaded() {
    return s_profile_loaded_boot;
}

void imu_request_save() {
    s_save_request = true;    // performed on Core 0 in imu_task (I2C-safe)
}

// Prove the NVS path can store and read back 22 bytes in the "imu" namespace
// WITHOUT touching the BNO055 or requiring a physical recalibration (the sensor
// must be unbolted to recal — a "ball ache"). Writes a known pattern to a scratch
// key, reads it back from a fresh handle, removes it, and reports PASS/FAIL.
// Pure NVS, so safe to call from Core 1 (NVS has its own internal locking).
bool imu_nvs_selftest() {
    uint8_t pat[22], chk[22] = {0};
    for (int i = 0; i < 22; i++) pat[i] = (uint8_t)(0xA5 ^ i);

    {
        Preferences p;
        if (!p.begin(NVS_NAMESPACE, /*readOnly=*/false)) {
            sys_log_push("IMU: NVS self-test FAIL (begin rw)");
            return false;
        }
        size_t w = p.putBytes("caltst", pat, 22);
        p.end();
        if (w != 22) {
            DBG_PRINTF("[IMU] NVS self-test: wrote %u/22\n", (unsigned)w);
            sys_log_push("IMU: NVS self-test FAIL (write)");
            return false;
        }
    }

    bool ok;
    {
        Preferences p;
        p.begin(NVS_NAMESPACE, /*readOnly=*/true);
        size_t r = p.getBytes("caltst", chk, 22);
        p.end();
        ok = (r == 22 && memcmp(chk, pat, 22) == 0);
    }
    {
        Preferences p;
        p.begin(NVS_NAMESPACE, /*readOnly=*/false);
        p.remove("caltst");
        p.end();
    }

    DBG_PRINTF("[IMU] NVS self-test: %s\n", ok ? "PASS" : "FAIL");
    sys_log_push(ok ? "IMU: NVS self-test PASS" : "IMU: NVS self-test FAIL");
    return ok;
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
        // The 25 ms mode-switch briefly stalls publishing — acceptable for this
        // operator-triggered one-shot (s_last_valid_ms is refreshed on the next
        // good sample, so it does not trip the fault timer).
        if (s_force_recal) {
            s_force_recal = false;
            s_bno.setMode(OPERATION_MODE_CONFIG);
            delay(25);
            s_bno.setMode(OPERATION_MODE_NDOF);
            DBG_PRINTLN("[IMU] recalibration started — drive slow loops");
        }

        // Operator-triggered "save calibration now" (imu_request_save). MUST run
        // here on Core 0 — read_offsets_raw() drives the I2C bus, which this task
        // owns; doing it from the Core-1 command handler would corrupt the bus.
        if (s_save_request) {
            s_save_request = false;
            uint8_t c = s_calib;
            uint8_t gy = (c >> 4) & 0x03, ac = (c >> 2) & 0x03, mg = c & 0x03;
            if (gy == 3 && ac == 3 && mg == 3) {
                uint8_t buf[22];
                if (read_offsets_raw(buf) && save_cal_to_nvs(buf)) {
                    s_profile_saved = true;
                    sys_log_push("IMU: calibration saved + verified (manual)");
                } else {
                    sys_log_push("IMU: manual calibration save FAILED");
                }
            } else {
                sys_log_push("IMU: manual save refused — gyro/accel/mag not all 3");
            }
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

            // Euler pitch/roll in the robot frame via axis remap. Negated so the
            // sign matches the imu.h convention (pitch nose-up +, roll right-side-
            // down +); confirmed on the bench 2026-06-15 (BNO P1 default reads the
            // opposite). Tilt below is sign-independent (acos of cos·cos), so this
            // only affects the reported/displayed attitude sign.
            float pitch = -ev.orientation.z * (float)DEG_TO_RAD;
            float roll  = -ev.orientation.y * (float)DEG_TO_RAD;
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

            // Auto-save the calibration profile once the three real sensors are
            // fully calibrated. NOT s_bno.isFullyCalibrated() — that also requires
            // sys==3, which the BNO055 often never reaches, so the profile would
            // otherwise never persist (forcing a re-calibrate every boot).
            if (!s_profile_saved && gy == 3 && ac == 3 && mg == 3) {
                uint8_t buf[22];
                // Only set s_profile_saved when the read-back actually verified —
                // otherwise leave it false so the next fully-calibrated sample
                // retries instead of silently giving up (the reported failure).
                if (read_offsets_raw(buf) && save_cal_to_nvs(buf)) {
                    s_profile_saved = true;
                    sys_log_push("IMU: BNO055 calibration saved + verified (restores on reboot)");
                    DBG_PRINTLN("[IMU] BNO055 calibration profile saved to NVS");
                } else {
                    sys_log_push("IMU: BNO055 calibration save FAILED — will retry");
                }
            }

            s_last_valid_ms = millis();
            s_imu_fault = false;
        } else if ((millis() - s_last_valid_ms) > 200UL) {
            s_imu_fault = true;
        }
    }
}
