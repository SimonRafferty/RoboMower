// ══════════════════════════════════════════════════════════════════════════════
//  imu_bmi270.cpp — RoboMower BMI270 IMU Implementation
//
//  Sampling task runs on Core 0 at 200Hz via vTaskDelayUntil (5ms period).
//  Heading is integrated using wrapAngle() from geometry.h after every step.
//  Thread safety: portMUX spinlock (very short critical sections at 200Hz).
//
//  Notes (see BLOCKERS.md B01, B02):
//    - Task priority 10 / stack 4096 per ARCHITECTURE.md (overrides agent prompt
//      which stated priority 22 / stack 3072 — see BLOCKERS.md B01).
//    - SparkFun BMI270 library returns gyro data in deg/s; converted to rad/s
//      before integration and bias storage (see BLOCKERS.md B02).
// ══════════════════════════════════════════════════════════════════════════════

#include "imu_bmi270.h"
#include "config.h"
#include "geometry.h"
#include "collision_detect.h"

#include <SparkFun_BMI270_Arduino_Library.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

// ── NVS namespace and key ─────────────────────────────────────────────────────
static const char* NVS_NAMESPACE   = "imu";
static const char* NVS_KEY_BIAS    = "gyrobias";

// ── Module state ──────────────────────────────────────────────────────────────
static BMI270           s_imu;
static portMUX_TYPE     s_spinlock       = portMUX_INITIALIZER_UNLOCKED;
static volatile float   s_heading        = 0.0f;   // radians, ±π
static volatile float   s_gz_rads        = 0.0f;   // rad/s, bias-corrected
static volatile float   s_gyro_bias      = 0.0f;   // rad/s, loaded from NVS
static volatile float   s_tilt_rad       = 0.0f;   // EMA-filtered tilt angle (rad)
static volatile float   s_pitch_rad      = 0.0f;   // EMA-filtered pitch (rad, nose-up positive)
static volatile float   s_roll_rad       = 0.0f;   // EMA-filtered roll (rad, right-down positive)
static volatile float   s_accel_x        = 0.0f;   // EMA-filtered surge (g, forward positive)
static volatile float   s_accel_y        = 0.0f;   // EMA-filtered sway (g, right positive)
static volatile float   s_accel_z        = 0.0f;   // EMA-filtered heave (g, up positive)
static volatile bool    s_imu_fault      = false;
static volatile uint32_t s_last_valid_ms = 0;
static volatile bool    s_initialized    = false;

static TaskHandle_t     s_task_handle    = nullptr;

// ── Forward declarations ──────────────────────────────────────────────────────
static void imu_task(void* pvParameters);

// ── Private helpers ───────────────────────────────────────────────────────────

/** Apply BMI270 gyro configuration: 200Hz ODR, NORMAL_MODE BW, ±250dps range. */
static bool apply_gyro_config() {
    // Read current (default) config — also captures the library's conversion scalar
    bmi2_sens_config orig;
    orig.type = BMI2_GYRO;
    int8_t result = s_imu.getConfig(&orig);
    if (result != BMI2_OK) {
        DBG_PRINTF("[IMU] getConfig() failed: %d\n", result);
        return false;
    }

    // Try our preferred config
    bmi2_sens_config config = orig;
    config.cfg.gyr.odr          = BMI2_GYR_ODR_200HZ;
    config.cfg.gyr.bwp          = BMI2_GYR_NORMAL_MODE;
    config.cfg.gyr.range        = BMI2_GYR_RANGE_250;
    config.cfg.gyr.ois_range    = BMI2_GYR_OIS_250;
    config.cfg.gyr.noise_perf   = BMI2_POWER_OPT_MODE;
    config.cfg.gyr.filter_perf  = BMI2_POWER_OPT_MODE;

    result = s_imu.setConfig(config);
    if (result != BMI2_OK) {
        DBG_PRINTF("[IMU] setConfig() error: %d — restoring default scalar\n", result);
        // setConfig overwrites the library's deg/s scalar BEFORE the Bosch API
        // rejects the config. Re-set the original config to fix the scalar.
        s_imu.setConfig(orig);
        return false;
    }
    return true;
}

/** Load gyro bias (rad/s) from NVS; returns 0.0f if not stored. */
static float load_bias_from_nvs() {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, /*readOnly=*/true);
    float bias = prefs.getFloat(NVS_KEY_BIAS, 0.0f);
    prefs.end();
    return bias;
}

/** Store gyro bias (rad/s) to NVS. */
static void save_bias_to_nvs(float bias_rads) {
    Preferences prefs;
    prefs.begin(NVS_NAMESPACE, /*readOnly=*/false);
    prefs.putFloat(NVS_KEY_BIAS, bias_rads);
    prefs.end();
}

// ══════════════════════════════════════════════════════════════════════════════
//  Public API
// ══════════════════════════════════════════════════════════════════════════════

bool imu_bmi270_init() {
    // Initialise I2C bus
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);  // 400kHz — spec Section 3

    // Begin BMI270 at configured I2C address.
    // Retry up to 5 times with 100 ms delay to tolerate transient I2C noise
    // during power-up (MED-5 fix — single-shot was failing on EMI-noisy bench).
    int8_t result = BMI2_E_NULL_PTR;
    for (int attempt = 1; attempt <= 5; attempt++) {
        result = s_imu.beginI2C(IMU_I2C_ADDRESS, Wire);
        if (result == BMI2_OK) break;
        DBG_PRINTF("[IMU] begin() attempt %d/5 failed: %d\n", attempt, result);
        delay(100);
    }
    if (result != BMI2_OK) {
        DBG_PRINTF("[IMU] begin() failed after 5 attempts: %d\n", result);
        return false;
    }

    // Configure gyro: 200Hz ODR, NORMAL_MODE bandwidth, ±250dps range
    if (!apply_gyro_config()) {
        DBG_PRINTLN("[IMU] gyro setConfig() failed — continuing with defaults");
    }

    // Configure accelerometer: 200Hz ODR to match sampling task, AVG4 filter
    {
        int8_t r = s_imu.setAccelODR(BMI2_ACC_ODR_200HZ);
        if (r != BMI2_OK) DBG_PRINTF("[IMU] setAccelODR failed: %d\n", r);
        r = s_imu.setAccelFilterBandwidth(BMI2_ACC_NORMAL_AVG4);
        if (r != BMI2_OK) DBG_PRINTF("[IMU] setAccelFilterBW failed: %d\n", r);
    }

    // Load bias from NVS (defaults to 0.0f if not yet calibrated)
    s_gyro_bias = load_bias_from_nvs();
    DBG_PRINTF("[IMU] Loaded gyro bias: %.6f rad/s\n", s_gyro_bias);

    s_last_valid_ms = millis();
    s_initialized   = true;

    // Start 200Hz sampling task on Core 0
    // Priority 10, stack 4096 per ARCHITECTURE.md Section 2
    BaseType_t rc = xTaskCreatePinnedToCore(
        imu_task,
        "imu_task",
        4096,           // stack bytes — ARCHITECTURE.md
        nullptr,
        10,             // priority — ARCHITECTURE.md (see BLOCKERS.md B01)
        &s_task_handle,
        0               // Core 0
    );

    if (rc != pdPASS) {
        DBG_PRINTLN("[IMU] Task creation failed");
        s_initialized = false;
        return false;
    }

    DBG_PRINTLN("[IMU] Initialised — 200Hz task running on Core 0");
    return true;
}

bool imu_collect_bias() {
    if (!s_initialized || s_task_handle == nullptr) {
        DBG_PRINTLN("[IMU] imu_collect_bias: not initialised");
        return false;
    }

    // Suspend sampling task to take exclusive I2C access
    vTaskSuspend(s_task_handle);

    DBG_PRINTLN("[IMU] Collecting gyro bias — robot must be stationary (2.5s)...");

    float sum_dps = 0.0f;
    int   count   = 0;

    for (int i = 0; i < 500; i++) {
        // Poll until a valid sample arrives (handles possible I2C latency)
        uint32_t deadline = millis() + 50;  // 50ms timeout per sample
        while (s_imu.getSensorData() != BMI2_OK) {
            if (millis() > deadline) {
                DBG_PRINTF("[IMU] imu_collect_bias: sample %d timed out\n", i);
                vTaskResume(s_task_handle);
                return false;
            }
            vTaskDelay(1);
        }
        sum_dps += s_imu.data.gyroZ;  // deg/s from library (see BLOCKERS.md B02)
        count++;
        vTaskDelay(pdMS_TO_TICKS(5));  // ~200Hz sample spacing
    }

    // Convert mean bias from deg/s to rad/s before storage
    float bias_rads = (sum_dps / (float)count) * (M_PI / 180.0f);

    // Atomic update of live bias
    taskENTER_CRITICAL(&s_spinlock);
    s_gyro_bias = bias_rads;
    taskEXIT_CRITICAL(&s_spinlock);

    // Persist to NVS
    save_bias_to_nvs(bias_rads);

    DBG_PRINTF("[IMU] Gyro bias calibrated: %.6f rad/s (%.4f deg/s) from %d samples\n",
                  bias_rads, bias_rads * (180.0f / M_PI), count);

    vTaskResume(s_task_handle);
    return true;
}

float imu_get_heading() {
    float h;
    taskENTER_CRITICAL(&s_spinlock);
    h = s_heading;
    taskEXIT_CRITICAL(&s_spinlock);
    return h;
}

float imu_get_gz_rads() {
    float gz;
    taskENTER_CRITICAL(&s_spinlock);
    gz = s_gz_rads;
    taskEXIT_CRITICAL(&s_spinlock);
    return gz;
}

float imu_get_bias() {
    // Bias is only written at init and during imu_collect_bias (with spinlock).
    // Reading the volatile directly is safe for diagnostic purposes.
    return s_gyro_bias;
}

bool imu_is_present() {
    return s_initialized;
}

bool imu_is_fault() {
    return s_imu_fault;
}

float imu_get_tilt_rad() {
    float t;
    taskENTER_CRITICAL(&s_spinlock);
    t = s_tilt_rad;
    taskEXIT_CRITICAL(&s_spinlock);
    return t;
}

float imu_get_pitch_rad() {
    float p;
    taskENTER_CRITICAL(&s_spinlock);
    p = s_pitch_rad;
    taskEXIT_CRITICAL(&s_spinlock);
    return p;
}

float imu_get_roll_rad() {
    float r;
    taskENTER_CRITICAL(&s_spinlock);
    r = s_roll_rad;
    taskEXIT_CRITICAL(&s_spinlock);
    return r;
}

void imu_get_accel(float *ax, float *ay, float *az) {
    taskENTER_CRITICAL(&s_spinlock);
    *ax = s_accel_x;
    *ay = s_accel_y;
    *az = s_accel_z;
    taskEXIT_CRITICAL(&s_spinlock);
}

void imu_set_heading(float heading_rad) {
    taskENTER_CRITICAL(&s_spinlock);
    s_heading = wrapAngle(heading_rad);
    taskEXIT_CRITICAL(&s_spinlock);
}

// ══════════════════════════════════════════════════════════════════════════════
//  FreeRTOS sampling task  (Core 0, 200Hz)
// ══════════════════════════════════════════════════════════════════════════════

static void imu_task(void* pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(5);  // 200Hz period

    uint32_t last_us = micros();

    while (true) {
        // Wait until the next 5ms tick (200Hz)
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        // Compute dt since last execution (micros gives ~1μs resolution)
        uint32_t now_us = micros();
        float dt = (float)(now_us - last_us) * 1e-6f;
        last_us = now_us;

        if (s_imu.getSensorData() == BMI2_OK) {
            // Apply axis corrections for board mounting orientation:
            //   Chip X = forward (ok),  Y = left (invert),  Z = up (ok)
            //   Chip gyroZ = CCW+ (invert for firmware CW+ convention)
            float gz_rads = -(s_imu.data.gyroZ * (M_PI / 180.0f) - s_gyro_bias);
            float ax =  s_imu.data.accelX;
            float ay = -s_imu.data.accelY;
            float az =  s_imu.data.accelZ;

            // Integrate heading and wrap to ±π; update published gz
            taskENTER_CRITICAL(&s_spinlock);
            s_heading  = wrapAngle(s_heading + gz_rads * dt);
            s_gz_rads  = gz_rads;
            taskEXIT_CRITICAL(&s_spinlock);

            // Compute EMA-filtered tilt, pitch, roll from accelerometer
            {
                float raw_tilt  = atan2f(sqrtf(ax * ax + ay * ay), az);
                float raw_pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
                float raw_roll  = atan2f(ay, az);
                float a = TILT_EMA_ALPHA;
                taskENTER_CRITICAL(&s_spinlock);
                s_tilt_rad  = s_tilt_rad  * (1.0f - a) + raw_tilt  * a;
                s_pitch_rad = s_pitch_rad * (1.0f - a) + raw_pitch * a;
                s_roll_rad  = s_roll_rad  * (1.0f - a) + raw_roll  * a;
                taskEXIT_CRITICAL(&s_spinlock);
            }

            // EMA-filtered gravity-compensated acceleration (g) for diagnostics.
            // To verify signs: TILT the board and hold (not tap).
            //   Tilt nose down → positive surge; tilt right down → positive sway
            {
                const float ad = 0.05f;  // faster than tilt EMA for readability
                float surge = -ax;           // forward = +ve (chip X is backward)
                float sway  =  ay;           // right   = +ve
                float heave = az - 1.0f;     // up      = +ve, gravity removed

                taskENTER_CRITICAL(&s_spinlock);
                s_accel_x = s_accel_x * (1.0f - ad) + surge * ad;
                s_accel_y = s_accel_y * (1.0f - ad) + sway  * ad;
                s_accel_z = s_accel_z * (1.0f - ad) + heave * ad;
                taskEXIT_CRITICAL(&s_spinlock);
            }

            // Feed gravity-compensated acceleration to collision detector.
            // Uses corrected axes; subtract 1g from Z (flat-ground approximation).
            collisionDetectUpdate(ax, ay, az - 1.0f);

            s_last_valid_ms = millis();
            s_imu_fault     = false;
        } else {
            // No data — check if fault threshold exceeded
            if ((millis() - s_last_valid_ms) > 200UL) {
                s_imu_fault = true;
            }
        }
    }
}
