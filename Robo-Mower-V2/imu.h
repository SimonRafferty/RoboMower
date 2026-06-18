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

/** True if a calibration profile was successfully restored from NVS at boot.
 *  When false, the BNO055 heading starts relative — the EKF then starts its
 *  GPS-trimmed heading offset at 0 rather than trusting a stale saved offset. */
bool imu_profile_loaded();

/** Request an immediate save of the current calibration (performed on Core 0 to
 *  keep the I2C bus single-owner). Verifies the write by read-back; reports the
 *  result to the system log. Refused unless mag AND gyro are fully (3) calibrated
 *  (accel is a bonus). Mostly redundant now that the firmware auto-saves the best
 *  calibration continuously, but kept for an explicit operator save. */
void imu_request_save();

/** Read the calibration profile persisted in NVS (22 offset bytes + quality 0..9).
 *  NVS-only (no BNO/I2C access), safe from any core. Used to export the calibration
 *  in the PWA settings file so it survives an NVS wipe / new board.
 *  @return true if a saved profile exists (out/quality filled), false otherwise. */
bool imu_get_saved_cal(uint8_t out[22], uint8_t *quality);

/** Restore a calibration profile (22 offset bytes + quality) from the settings
 *  file: persist it to NVS (read-back verified) and apply it to the live BNO055
 *  on Core 0. @return true if the NVS write verified. */
bool imu_restore_cal(const uint8_t buf[22], uint8_t quality);

/** Re-apply the calibration profile already saved in NVS to the live BNO055,
 *  without a reboot (operator "Apply saved calibration"). The BNO055's reported
 *  calibration STATUS reads low right after an offset restore until it re-validates
 *  with motion — the offsets and absolute heading are applied regardless. Logs
 *  "applied" / "no saved calibration to apply". No-op if no profile is stored. */
void imu_reapply_saved_cal();

/** Self-test the NVS calibration path WITHOUT touching the BNO055 or needing a
 *  physical recalibration: writes a known 22-byte pattern, reads it back from a
 *  fresh handle, and reports PASS/FAIL to the system log. @return true on PASS. */
bool imu_nvs_selftest();
