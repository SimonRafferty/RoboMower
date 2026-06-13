#pragma once
#include <vector>
#include <utility>
#include <cmath>
#include <cstdint>
#include <algorithm>

// ══════════════════════════════════════════════════════════════════════════════
//  geometry.h — RoboMower polygon mathematics
//
//  All coordinates are in ENU (East-North-Up) metres, 2D (x=East, y=North).
//  All angles are in radians. CCW winding = positive area (standard maths).
//  Float precision throughout — ESP32-S3 has hardware FPU for float.
//
//  Epsilon values:
//    1e-4f — vertex coincidence check (0.1 mm)
//    1e-6f — parallel-edge / denominator guard
// ══════════════════════════════════════════════════════════════════════════════

/** 2D point in ENU metres (x=East, y=North) */
struct Point {
    float x;
    float y;
    Point() : x(0.0f), y(0.0f) {}
    Point(float x_, float y_) : x(x_), y(y_) {}
};

/** 2D polygon with (x,y) vertices in ENU metres */
struct Polygon {
    std::vector<Point> pts;

    /** Signed area via shoelace formula. Positive = CCW winding. */
    float area() const;

    /** True if vertices are in clockwise order */
    bool isClockwise() const;

    /** Ensure counter-clockwise winding (reverse if needed) */
    void ensureCCW();
};

/**
 * @brief Inset a polygon inward by inset_m metres.
 *
 * For each edge, computes an inward-offset parallel edge moved by inset_m
 * along the inward normal. Adjacent offset-edge intersections become the new
 * vertices. Convex corners use the intersection directly; concave corners
 * (interior angle > 180°) insert an arc fillet of radius inset_m sampled at
 * ARC_INTERVAL_M spacing. Duplicate vertices (< 1e-4 m apart) are removed.
 * If the result self-intersects, splitSelfIntersecting() is called and the
 * largest valid sub-polygon is returned. Returns an empty Polygon (pts.empty())
 * if the result has fewer than 3 vertices or zero area.
 *
 * @param poly     Input polygon (will be made CCW internally).
 * @param inset_m  Inset distance in metres (positive = inward).
 * @return Inset polygon, CCW winding, or empty if fully collapsed.
 */
Polygon insetPolygon(const Polygon &poly, float inset_m);

/**
 * @brief Inset a polygon, returning ALL resulting sub-polygons.
 *
 * Same algorithm as insetPolygon(), but when the inset creates a
 * self-intersecting polygon that splits into multiple regions (e.g. an
 * L-shape with a narrow neck), ALL valid sub-polygons are returned
 * instead of just the largest. Each sub-polygon has CCW winding.
 *
 * @param poly     Input polygon (will be made CCW internally).
 * @param inset_m  Inset distance in metres (positive = inward).
 * @return Vector of valid sub-polygons, or empty if fully collapsed.
 */
std::vector<Polygon> insetPolygonMulti(const Polygon &poly, float inset_m);

/**
 * @brief Ray-casting point-in-polygon test.
 *
 * Casts a horizontal ray from (px, py) and counts edge crossings.
 * Points on an edge are treated as inside (returns true).
 *
 * @param poly  Polygon to test against.
 * @param px    Point x coordinate.
 * @param py    Point y coordinate.
 * @return true if inside or on boundary, false if outside.
 */
bool isInsidePolygon(const Polygon &poly, float px, float py);

/**
 * @brief Signed distance from point (px,py) to nearest edge of poly.
 *
 * Computes the minimum perpendicular distance from the point to any edge
 * segment. Sign: positive if inside the polygon, negative if outside.
 * Uses isInsidePolygon() to determine sign.
 *
 * @param poly  Polygon to measure against.
 * @param px    Point x coordinate.
 * @param py    Point y coordinate.
 * @return Signed distance in metres.
 */
float distanceToNearestEdge(const Polygon &poly, float px, float py);

/**
 * @brief Convex hull via Graham scan.
 *
 * Algorithm:
 *   1. Find lowest-y (then lowest-x) point as pivot.
 *   2. Sort remaining points by polar angle from pivot, breaking ties by distance.
 *   3. Process with a stack, keeping only left-turn vertices.
 *
 * @param poly  Input point cloud (polygon vertices used as input points).
 * @return CCW convex hull polygon.
 */
Polygon convexHull(const Polygon &poly);

/**
 * @brief Split a self-intersecting polygon into non-self-intersecting sub-polygons.
 *
 * Finds all self-intersection points between non-adjacent edges, splits the
 * polygon at each crossing, and returns sub-polygons with signed area >=
 * MIN_ZONE_AREA_M2 (from config.h). Sub-polygons with insufficient area are
 * discarded. Result polygons are ensured CCW.
 *
 * @param poly  Potentially self-intersecting polygon.
 * @return Vector of valid (area >= MIN_ZONE_AREA_M2) non-self-intersecting sub-polygons.
 */
std::vector<Polygon> splitSelfIntersecting(const Polygon &poly);

/**
 * @brief Check if polygon self-intersects.
 *
 * Tests all non-adjacent edge pairs for intersection using the cross-product
 * segment-intersection test. O(n²) — acceptable for the polygon sizes used
 * in this application (≤ 500 + arc vertices).
 *
 * @param poly  Polygon to test.
 * @return true if any two non-adjacent edges intersect.
 */
bool isSelfIntersecting(const Polygon &poly);

/**
 * @brief Sample a circular arc at arc_interval_m spacing.
 *
 * Generates equally-spaced points along the arc from start_angle to end_angle
 * (both in radians, CCW positive). The angular step is computed so that the
 * chord length approximates arc_interval_m. Always includes the endpoint.
 * Handles total arc angle > 2π (wraps around).
 *
 * @param cx              Arc centre x coordinate.
 * @param cy              Arc centre y coordinate.
 * @param r               Arc radius in metres.
 * @param start_angle     Start angle in radians.
 * @param end_angle       End angle in radians (CCW from start).
 * @param arc_interval_m  Maximum spacing between consecutive points in metres.
 * @return Vector of (x,y) points along the arc.
 */
std::vector<Point> sampleArc(float cx, float cy, float r,
                             float start_angle, float end_angle,
                             float arc_interval_m);

/**
 * @brief Wrap angle to [-PI, PI] using atan2(sin, cos) method.
 *
 * @param angle_rad  Input angle in radians (any range).
 * @return Equivalent angle in [-PI, PI].
 */
float wrapAngle(float angle_rad);

/**
 * @brief Clamp value to [lo, hi].
 *
 * @param v   Value to clamp.
 * @param lo  Lower bound.
 * @param hi  Upper bound.
 * @return Clamped value.
 */
float clampf(float v, float lo, float hi);

/**
 * @brief Smooth convex corners of a CCW polygon to a given turn radius.
 *
 * For each convex corner (left turn in CCW traversal), replaces the sharp
 * vertex with arc sample points of the given radius. The arc is inscribed
 * inside the original polygon so that the smoothed path never extends
 * outside the original boundary. Concave corners (right turn in CCW
 * traversal) are kept as-is since the mower can pivot there.
 *
 * If the tangent distance at a corner exceeds half the length of either
 * adjacent edge, the corner is kept sharp (not enough room to inscribe
 * the arc).
 *
 * @param poly              Input polygon (will be made CCW internally).
 * @param min_turn_radius   Minimum turn radius in metres.
 * @return Smoothed polygon with CCW winding, or original if < 3 vertices.
 */
Polygon smoothPolygonCorners(const Polygon &poly, float min_turn_radius);
