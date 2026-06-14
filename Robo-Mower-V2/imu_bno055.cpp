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
