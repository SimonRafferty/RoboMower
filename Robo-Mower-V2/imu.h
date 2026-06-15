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

/** True when the absolute heading is trustworthy: BNO mag/gyro/accel each at or
 *  above their IMU_CALIB_*_MIN thresholds. The unreliable `sys` aggregate is NOT
 *  used (it often never reaches 3 even when the three sensors are fully calibrated). */
bool imu_heading_is_confident();

/** Force a fresh magnetometer recalibration: discard the stored NVS profile and
 *  relearn from motion. The new profile is auto-saved once fully calibrated. */
void imu_recalibrate();
