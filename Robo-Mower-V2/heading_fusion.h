#pragma once
// ══════════════════════════════════════════════════════════════════════════════
//  heading_fusion.h — pure helpers for BNO-heading + GPS-trimmed offset
//
//  No Arduino / FreeRTOS dependencies — host-testable. Angles in radians,
//  wrapped to ±π. Thresholds are passed in (no config.h dependency) so the
//  module is trivially unit-testable.
// ══════════════════════════════════════════════════════════════════════════════

/** True if the segment since the heading reference was STRAIGHT and MEASURABLE,
 *  i.e. the GPS travel chord is a valid absolute heading to trim the offset to.
 *  @param turn_accum_rad   Σ|Δheading| since the reference (rad).
 *  @param dist_m           Travel distance of the chord (m).
 *  @param sigma_m          GPS 1-σ position noise (m).
 *  @param straight_max_turn_rad  Max allowed turn to call it straight.
 *  @param min_dist_m       Distance floor.
 *  @param dist_sigma_k     Required dist also ≥ k·sigma. */
bool heading_offset_segment_qualifies(float turn_accum_rad, float dist_m,
                                      float sigma_m, float straight_max_turn_rad,
                                      float min_dist_m, float dist_sigma_k);

/** EMA-update the heading offset toward (z_hdg − bno_hdg), wrap-safe.
 *  @return new offset (rad, ±π). */
float heading_offset_ema(float offset, float z_hdg, float bno_hdg, float gain);

/** Compose absolute heading from BNO heading + offset (rad, ±π). */
float heading_compose(float bno_hdg, float offset);
