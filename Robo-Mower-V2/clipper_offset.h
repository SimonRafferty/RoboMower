#pragma once
#include "geometry.h"
#include <vector>

// ══════════════════════════════════════════════════════════════════════════════
//  clipper_offset.h — robust polygon inset via the Clipper2 library
//
//  The hand-rolled insetPolygon() spikes at concave (reflex) corners and inverts
//  past the local inradius, because it has no boolean-union cleanup. Clipper2
//  (the same offsetting engine libslic3r/OpenMower use) does the cleanup, so it
//  insets arbitrary concave gardens — notches, splits, over-shrink — robustly.
//
//  This wraps Clipper2's InflatePaths() with round joins. Coordinates are scaled
//  to Clipper's int64 grid (0.1 mm) and back.
// ══════════════════════════════════════════════════════════════════════════════

/**
 * @brief Inset a polygon inward by inset_m metres using Clipper2 (round joins).
 *
 * Robust for concave shapes: notches round off instead of spiking, the shape
 * splits into multiple sub-polygons where it pinches, and an over-shrink
 * collapses cleanly to an empty result (no inverted/garbage polygons).
 *
 * @param poly     Input polygon (>= 3 vertices). Winding is normalised internally.
 * @param inset_m  Inward inset distance in metres (> 0).
 * @param sharp_corners  false (default) = round joins (concave notches round off,
 *                 safest for arbitrary gardens). true = MITER joins (limit 2.0):
 *                 keeps CONVEX corners SHARP — single turn-point nodes — so the
 *                 node follower pivots once per corner instead of multiple times
 *                 through a bevelled arc. Use for the spiral mow path, which must
 *                 match the operator's sparse pivot-at-corner recording. Acute and
 *                 reflex corners are still squared off at the miter limit.
 * @return Vector of solid (CCW) sub-polygons. Empty if fully collapsed. Caller
 *         applies any minimum-area filtering (e.g. MIN_ZONE_AREA_M2).
 */
std::vector<Polygon> offsetPolygonClipper(const Polygon &poly, float inset_m,
                                          bool sharp_corners = false);

/**
 * @brief Inset a polygon, returning only the single LARGEST solid sub-polygon.
 *
 * Drop-in robust replacement for the hand-rolled insetPolygon() at the
 * nav-boundary / working-area derivation sites: same contract (one Polygon, or
 * empty if the inset collapses), but Clipper2's round-join offset so concave
 * notches round off cleanly instead of spiking inward.
 *
 * @param poly     Input polygon (>= 3 vertices).
 * @param inset_m  Inward inset distance in metres (> 0).
 * @return Largest resulting sub-polygon (CCW), or empty if fully collapsed.
 */
Polygon insetPolygonClipper(const Polygon &poly, float inset_m);
