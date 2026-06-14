#pragma once
#include <Arduino.h>
#include <stdint.h>

// ══════════════════════════════════════════════════════════════════════════════
//  odo_calib.h — GPS-referenced drive-odometry self-calibration
//
//  Continuously calibrates two odometry constants against RTK GPS, in BOTH
//  manual and auto driving (any time the wheels turn and GPS is RTK quality):
//
//    1. scale   — dimensionless distance multiplier applied to wheel velocity.
//                 Corrects ticks-per-metre / effective wheel diameter. Default 1.0.
//    2. track_m — kinematic track width (m) used for differential heading (omega).
//                 The effective track shifts as the machine leans or rides a track
//                 edge, so it is calibrated separately from the nominal steering
//                 track. Default = nominal track_width_m.
//
//  The two are SEPARATELY OBSERVABLE: distance scale from straight runs (where the
//  wheels turn equally, so track width is irrelevant), track width from turns
//  (where the GPS heading change reveals the true omega-per-differential).
//
//  Storage: its OWN NVS namespace "odocal" (keys "scale", "track"), kept apart
//  from the mow_cfg blob so the operator's manually entered config is never
//  disturbed. Writes are rate-limited.
//
//  Threading: odo_calib_update() runs on Core 1 (10 Hz EKF hook in state_machine).
//  The accessors odo_cal_scale() / odo_cal_track_m() are read on Core 1 only
//  (vesc_erpm_to_velocity_scaled, ekf_predict omega, node_follower, bog_recovery).
// ══════════════════════════════════════════════════════════════════════════════


/** Load scale / track_m from NVS (defaults: 1.0 and nominal track_width_m).
 *  MUST be called after mower_config_init() so the nominal track is available. */
void odo_calib_init();

/** Feed one 10 Hz odometry tick. vL/vR are the SCALED wheel velocities (m/s,
 *  signed) already multiplied by odo_cal_scale(); dt in seconds. Polls the EKF's
 *  GPS heading-event stream and updates the calibration when a clean straight or
 *  turn interval completes. Saves to NVS at most every ~20 s when changed. */
void odo_calib_update(float v_left_scaled, float v_right_scaled, float dt);

/** Force any pending calibration change to NVS (call on exit from driving, e.g.
 *  STATE_IDLE entry). No-op if nothing changed since the last save. */
void odo_calib_flush();

/** Current distance multiplier (applied to raw wheel velocity). */
float odo_cal_scale();

/** Current calibrated kinematic track width (m). */
float odo_cal_track_m();
