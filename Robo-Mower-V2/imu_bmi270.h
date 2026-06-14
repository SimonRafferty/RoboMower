#pragma once
#include <Arduino.h>
#include <stdint.h>

// ══════════════════════════════════════════════════════════════════════════════
//  imu_bmi270.h — RoboMower BMI270 IMU Interface
//
//  Manages the SparkFun BMI270 gyroscope via I2C Wire (SDA=GPIO8, SCL=GPIO9).
//  Runs a 200Hz FreeRTOS sampling task on Core 0 that integrates gyro Z into
//  a yaw heading, protected by a portMUX spinlock for thread safety.
//
//  Units throughout:
//    - Heading: radians, wrapped to ±π via wrapAngle()
//    - Gyro rate: rad/s (converted from library's deg/s output)
//    - Bias: rad/s
//
//  Dependencies:
//    - SparkFun_BMI270_Arduino_Library (Library Manager)
//    - geometry.h (for wrapAngle())
//    - config.h   (IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_ADDRESS)
//    - Preferences (ESP32 NVS, bias key "gyrobias")
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Initialise BMI270 via I2C Wire at address 0x69 (IMU_I2C_ADDRESS, SDO/SA0=VCC).
 *
 * Configures:
 *   - ODR = 200Hz  (BMI2_GYR_ODR_200HZ)
 *   - Bandwidth = NORMAL_MODE  (BMI2_GYR_NORMAL_MODE, ~40Hz effective BW)
 *   - Range = ±250 dps  (BMI2_GYR_RANGE_250)
 *
 * Loads gyro bias from NVS key "gyrobias" if present; defaults to 0.0f.
 * Does NOT collect a new bias — bias collection is done separately in INIT
 * state via imu_collect_bias().
 *
 * Starts the FreeRTOS "imu_task" on Core 0, Priority 10, Stack 4096 bytes.
 *
 * @return true  on success (BMI270 responded and task created)
 * @return false if BMI270 begin() failed or task could not be created
 */
bool imu_bmi270_init();

/**
 * @brief Collect gyro bias calibration.
 *
 * Suspends the sampling task, samples 500 readings from the BMI270, computes
 * the mean Z-rate, converts to rad/s, and stores it to NVS key "gyrobias".
 * Resumes the sampling task afterwards.
 *
 * Blocks the calling task for approximately 2.5 seconds (500 samples × 5ms).
 * The robot MUST be stationary during this call. Call from INIT state only.
 *
 * @return true  on success
 * @return false if the IMU is not initialised or sampling failed
 */
bool imu_collect_bias();

/**
 * @brief Get the current integrated yaw heading.
 *
 * Returns the heading in radians, wrapped to ±π by wrapAngle() after every
 * integration step. Thread-safe (spinlock protected).
 *
 * @return float  heading in radians [−π, +π]
 */
float imu_get_heading();

/**
 * @brief Get the current gyro Z rate (bias-corrected).
 *
 * Returns the instantaneous bias-corrected angular rate about the Z axis,
 * converted from deg/s to rad/s. Thread-safe (spinlock protected).
 *
 * @return float  angular rate in rad/s
 */
float imu_get_gz_rads();

/**
 * @brief Get the gyro bias value currently in use.
 *
 * Returns the bias loaded from NVS (or collected during imu_collect_bias()).
 * Value is in rad/s.
 *
 * @return float  gyro bias in rad/s
 */
float imu_get_bias();

/**
 * @brief Returns true if the BMI270 was successfully initialised.
 *
 * Returns false if imu_bmi270_init() failed (device not detected or task
 * could not be created). Distinct from imu_is_fault(), which only fires
 * after the sampling task has started and then loses I2C contact.
 *
 * @return true   IMU was found and task is running
 * @return false  IMU not present or init failed
 */
bool imu_is_present();

/**
 * @brief Query IMU fault status.
 *
 * Returns true if I2C communication with the BMI270 has been continuously
 * failing for more than 200ms.
 *
 * @return true   IMU I2C fault (no valid data for > 200ms)
 * @return false  IMU operating normally
 */
bool imu_is_fault();

/**
 * @brief Get EMA-filtered tilt angle from vertical.
 *
 * Computed from BMI270 accelerometer data at 200Hz using
 * atan2(sqrt(ax²+ay²), az). EMA-filtered with α=TILT_EMA_ALPHA
 * (~0.3s time constant) to reject single-sample noise.
 *
 * @return float  tilt angle in radians (0 = flat, π/2 = on side)
 */
float imu_get_tilt_rad();

/** Get EMA-filtered pitch angle (radians). Nose-up = positive. */
float imu_get_pitch_rad();

/** Get EMA-filtered roll angle (radians). Right-side-down = positive. */
float imu_get_roll_rad();

/** Get EMA-filtered accelerometer axes in g (corrected for mounting).
 *  Surge (ax): forward = +ve. Sway (ay): right = +ve. Heave (az): up = +ve. */
void imu_get_accel(float *ax, float *ay, float *az);

/**
 * @brief Reset the integrated heading to a specific value.
 *
 * Called by the EKF localiser when a GPS-derived heading update is available
 * and sufficiently reliable. Acquires the spinlock before writing.
 *
 * @param heading_rad  New heading in radians (will be wrapped to ±π)
 */
void imu_set_heading(float heading_rad);
