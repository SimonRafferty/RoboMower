// ══════════════════════════════════════════════════════════════════════════════
//  geometry_test.cpp — Unit tests for geometry.h / geometry.cpp
//
//  Tests are self-contained: no external framework. Results printed to Serial.
//  Build with TEST_MODE defined; call geometry_test_runAll() from setup().
//
//  Expected results at time of authoring:
//    Test 1 (Convex inset square)       — PASS
//    Test 2 (Concave L-shape inset)     — PASS
//    Test 3 (Narrow notch closure)      — FAIL (see note below)
//    Test 4 (Point-in-polygon)          — PASS
//    Test 5 (Self-intersection detect)  — PASS
//
//  NOTE on Test 3: The parallel-edge inset algorithm moves notch walls in the
//  direction of the polygon's inward normal (away from the notch centreline).
//  A narrow notch therefore WIDENS during inset rather than collapsing. The
//  test documents the intended specification for future implementation review.
// ══════════════════════════════════════════════════════════════════════════════

#include "geometry_test.h"
#include "geometry.h"
#include "config.h"
#include <Arduino.h>

// ── Helpers ──────────────────────────────────────────────────────────────────

static bool approxEqual(float a, float b, float tol = 0.005f)
{
    return fabsf(a - b) <= tol;
}

/** Search result polygon for a vertex within tolerance of (x, y). */
static bool findVertex(const Polygon &poly, float x, float y, float tol = 0.001f)
{
    for (const auto &p : poly.pts) {
        if (approxEqual(p.x,  x, tol) &&
            approxEqual(p.y, y, tol)) {
            return true;
        }
    }
    return false;
}

// ── Individual test functions ─────────────────────────────────────────────────

/**
 * Test 1 — Convex polygon inset (unit square).
 *
 * Input:  (0,0),(1,0),(1,1),(0,1) — unit square (CCW).
 * Inset:  0.1 m.
 * Expected: (0.1,0.1),(0.9,0.1),(0.9,0.9),(0.1,0.9). Area = 0.64.
 */
static int test_convex_inset_square()
{
    static const char *name = "Test 1 (Convex inset square)";
    int fails = 0;

    Polygon sq;
    sq.pts = {{0.0f,0.0f},{1.0f,0.0f},{1.0f,1.0f},{0.0f,1.0f}};
    Polygon result = insetPolygon(sq, 0.1f);

    if (result.pts.empty()) {
        DBG_PRINTF("[GTEST] FAIL %s: result polygon is empty\n", name);
        DBG_PRINTF("[GTEST] %s: FAIL\n", name);
        return 1;
    }

    // Vertex count: inset of a convex square produces exactly 4 vertices
    int nv = static_cast<int>(result.pts.size());
    if (nv != 4) {
        DBG_PRINTF("[GTEST] FAIL %s: expected 4 vertices, got %d\n", name, nv);
        fails++;
    }

    // Area ≈ 0.64 (0.8 × 0.8), tolerance ±0.001
    float area = fabsf(result.area());
    if (!approxEqual(area, 0.64f, 0.001f)) {
        DBG_PRINTF("[GTEST] FAIL %s: area expected 0.64 got %.5f\n", name, area);
        fails++;
    }

    // All four expected vertices present, tolerance ±0.001 per coordinate
    float expected[4][2] = {
        {0.1f, 0.1f}, {0.9f, 0.1f}, {0.9f, 0.9f}, {0.1f, 0.9f}
    };
    for (int e = 0; e < 4; ++e) {
        if (!findVertex(result, expected[e][0], expected[e][1], 0.001f)) {
            DBG_PRINTF("[GTEST] FAIL %s: vertex %d expected (%.2f,%.2f) not found\n",
                          name, e, expected[e][0], expected[e][1]);
            fails++;
        }
    }

    DBG_PRINTF("[GTEST] %s: %s\n", name, fails == 0 ? "PASS" : "FAIL");
    return fails > 0 ? 1 : 0;
}

/**
 * Test 2 — Concave polygon inset (L-shape).
 *
 * Input:  (0,0),(2,0),(2,1),(1,1),(1,2),(0,2) — CCW L-shape, area = 3.0.
 * Inset:  0.1 m.
 * Expected: result area < 3.0; >= 6 vertices (arc fillet at concave corner);
 *           inner corner near (0.1,0.1); all vertices inside original.
 */
static int test_concave_Lshape_inset()
{
    static const char *name = "Test 2 (Concave L-shape inset)";
    int fails = 0;

    Polygon lshape;
    lshape.pts = {
        {0.0f,0.0f},{2.0f,0.0f},{2.0f,1.0f},
        {1.0f,1.0f},{1.0f,2.0f},{0.0f,2.0f}
    };
    float inputArea = fabsf(lshape.area());  // = 3.0

    Polygon result = insetPolygon(lshape, 0.1f);

    if (result.pts.empty()) {
        DBG_PRINTF("[GTEST] FAIL %s: result polygon is empty\n", name);
        DBG_PRINTF("[GTEST] %s: FAIL\n", name);
        return 1;
    }

    // Result area must be smaller than input
    float resultArea = fabsf(result.area());
    if (resultArea >= inputArea) {
        DBG_PRINTF("[GTEST] FAIL %s: result area %.4f not < input area %.4f\n",
                      name, resultArea, inputArea);
        fails++;
    }

    // Inner corner at approximately (0.1, 0.1) — the inset of corner (0,0)
    if (!findVertex(result, 0.1f, 0.1f, 0.005f)) {
        DBG_PRINTF("[GTEST] FAIL %s: inner corner near (0.10,0.10) not found\n", name);
        fails++;
    }

    // At least 6 vertices: 5 convex corners + arc fillet points for concave corner at (1,1)
    int nv = static_cast<int>(result.pts.size());
    if (nv < 6) {
        DBG_PRINTF("[GTEST] FAIL %s: expected >= 6 vertices, got %d\n", name, nv);
        fails++;
    }

    // All result vertices must be inside the original L-shape
    for (int i = 0; i < nv; ++i) {
        float x = result.pts[i].x;
        float y = result.pts[i].y;
        if (!isInsidePolygon(lshape, x, y)) {
            DBG_PRINTF("[GTEST] FAIL %s: vertex %d (%.4f,%.4f) lies outside L-shape\n",
                          name, i, x, y);
            fails++;
        }
    }

    DBG_PRINTF("[GTEST] %s: %s\n", name, fails == 0 ? "PASS" : "FAIL");
    return fails > 0 ? 1 : 0;
}

/**
 * Test 3 — Narrow notch closure.
 *
 * Input:  Rectangle (0,0)-(4,3) with a narrow notch in the top edge.
 *         Notch: x = 1.95 to 2.05 (width 0.1 m), y = 2.5 to 3.0 (depth 0.5 m).
 * Inset:  0.15 m (2 × 0.15 = 0.30 > notch width 0.10).
 * Expected: notch collapses; result is near-convex; vertex count <= input count.
 *
 * NOTE: With the parallel-edge inset algorithm used, the notch walls move
 * away from each other (their inward normals point into the main polygon body,
 * not toward the notch centreline). The notch therefore widens rather than
 * collapsing. This test documents the intended behaviour; it is expected to
 * fail against the current implementation and serves as a regression marker.
 */
static int test_narrow_notch_closure()
{
    static const char *name = "Test 3 (Narrow notch closure)";
    int fails = 0;

    // Rectangle (0,0)-(4,3) with narrow notch (0.1 m wide, 0.5 m deep) in top edge
    Polygon notchPoly;
    notchPoly.pts = {
        {0.0f,  0.0f}, {4.0f,  0.0f}, {4.0f,  3.0f},
        {2.05f, 3.0f}, {2.05f, 2.5f}, {1.95f, 2.5f}, {1.95f, 3.0f},
        {0.0f,  3.0f}
    };
    int inputVertexCount = static_cast<int>(notchPoly.pts.size());  // 8

    Polygon result = insetPolygon(notchPoly, 0.15f);

    if (result.pts.empty()) {
        DBG_PRINTF("[GTEST] FAIL %s: result polygon is empty\n", name);
        DBG_PRINTF("[GTEST] %s: FAIL\n", name);
        return 1;
    }

    // Result must be a valid (non-self-intersecting) polygon
    if (isSelfIntersecting(result)) {
        DBG_PRINTF("[GTEST] FAIL %s: result polygon is self-intersecting\n", name);
        fails++;
    }

    // Notch-collapsed result should have no more vertices than the input
    // (4 rectangular corners <= 8 input vertices)
    int resultVertexCount = static_cast<int>(result.pts.size());
    if (resultVertexCount > inputVertexCount) {
        DBG_PRINTF("[GTEST] FAIL %s: notch not collapsed — got %d vertices (expected <= %d)\n",
                      name, resultVertexCount, inputVertexCount);
        fails++;
    }

    DBG_PRINTF("[GTEST] %s: %s\n", name, fails == 0 ? "PASS" : "FAIL");
    return fails > 0 ? 1 : 0;
}

/**
 * Test 4 — Point-in-polygon (ray-casting).
 *
 * Polygon: square (0,0) to (2,2).
 * Tests inside, outside, on-edge, corner, and clearly-outside points.
 */
static int test_point_in_polygon()
{
    static const char *name = "Test 4 (Point-in-polygon)";
    int fails = 0;

    Polygon sq;
    sq.pts = {{0.0f,0.0f},{2.0f,0.0f},{2.0f,2.0f},{0.0f,2.0f}};

    struct TC { float x; float y; bool expect; const char *desc; };
    static const TC cases[] = {
        { 1.0f,  1.0f, true,  "interior centre"  },
        { 3.0f,  1.0f, false, "outside right"    },
        { 0.0f,  1.0f, true,  "on left edge"     },
        { 0.0f,  0.0f, true,  "bottom-left corner"},
        {-0.1f,  1.0f, false, "outside left"     },
        { 1.0f,  2.1f, false, "above top edge"   },
    };

    for (const auto &tc : cases) {
        bool got = isInsidePolygon(sq, tc.x, tc.y);
        if (got != tc.expect) {
            DBG_PRINTF("[GTEST] FAIL %s: (%.1f,%.1f) [%s] expected %s, got %s\n",
                          name, tc.x, tc.y, tc.desc,
                          tc.expect ? "inside" : "outside",
                          got       ? "inside" : "outside");
            fails++;
        }
    }

    DBG_PRINTF("[GTEST] %s: %s\n", name, fails == 0 ? "PASS" : "FAIL");
    return fails > 0 ? 1 : 0;
}

/**
 * Test 5 — Self-intersecting polygon detection.
 *
 * Tests three polygons: a simple square (false), a bow-tie (true),
 * and a triangle (false — n < 4, cannot self-intersect).
 */
static int test_self_intersection_detect()
{
    static const char *name = "Test 5 (Self-intersection detect)";
    int fails = 0;

    // Simple square — should NOT self-intersect
    Polygon sq;
    sq.pts = {{0.0f,0.0f},{2.0f,0.0f},{2.0f,2.0f},{0.0f,2.0f}};
    if (isSelfIntersecting(sq)) {
        DBG_PRINTF("[GTEST] FAIL %s: simple square incorrectly flagged as self-intersecting\n",
                      name);
        fails++;
    }

    // Bow-tie: edges (0,0)-(2,2) and (2,0)-(0,2) cross — MUST self-intersect
    Polygon bowTie;
    bowTie.pts = {{0.0f,0.0f},{2.0f,2.0f},{2.0f,0.0f},{0.0f,2.0f}};
    if (!isSelfIntersecting(bowTie)) {
        DBG_PRINTF("[GTEST] FAIL %s: bow-tie not detected as self-intersecting\n", name);
        fails++;
    }

    // Triangle — fewer than 4 vertices, cannot self-intersect
    Polygon tri;
    tri.pts = {{0.0f,0.0f},{1.0f,0.0f},{0.5f,1.0f}};
    if (isSelfIntersecting(tri)) {
        DBG_PRINTF("[GTEST] FAIL %s: triangle incorrectly flagged as self-intersecting\n",
                      name);
        fails++;
    }

    DBG_PRINTF("[GTEST] %s: %s\n", name, fails == 0 ? "PASS" : "FAIL");
    return fails > 0 ? 1 : 0;
}

// ── Public entry point ────────────────────────────────────────────────────────

int geometry_test_runAll()
{
    int failures = 0;
    failures += test_convex_inset_square();
    failures += test_concave_Lshape_inset();
    failures += test_narrow_notch_closure();
    failures += test_point_in_polygon();
    failures += test_self_intersection_detect();
    DBG_PRINTF("[GTEST] ===== %d/5 tests passed =====\n", 5 - failures);
    return failures;
}
