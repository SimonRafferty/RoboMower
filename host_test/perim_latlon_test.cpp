// ══════════════════════════════════════════════════════════════════════════════
//  perim_latlon_test.cpp — host test for the lat/lon perimeter storage (Item 3)
//
//  Validates, offline (no hardware), the pure math that makes the absolute
//  lat/lon perimeter correct and origin-independent:
//    1. lat/lon → ENU → lat/lon round-trips to sub-cm.
//    2. The relative vector between two points is ~origin-independent (the
//       property that fixes the power-up RTK-vs-map mismatch).
//    3. The confidence-aware breach margin formula.
//    4. The "perim2" blob serialise/deserialise + CRC32 round-trip and
//       corruption detection.
//
//  These replicate the firmware formulas (rtk_gps.cpp / nvs_storage.cpp /
//  safety.cpp). Keep them in sync if those change.
//
//  Build & run (exit code = number of failures):
//    zig c++ -std=c++17 -O1 -o perimtest.exe perim_latlon_test.cpp
//    ./perimtest.exe
// ══════════════════════════════════════════════════════════════════════════════

#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static int g_failures = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("FAIL: %s\n", (msg)); ++g_failures; } \
    else         { printf("ok  : %s\n", (msg)); } \
} while (0)

// ── Firmware formula replicas ────────────────────────────────────────────────
static constexpr double K = 111319.5;  // METRES_PER_DEG_LAT (rtk_gps.cpp)

static void latlon_to_enu(double lat, double lon, double lat0, double lon0,
                          float *e, float *n) {
    *e = (float)((lon - lon0) * cos(lat0 * M_PI / 180.0) * K);
    *n = (float)((lat - lat0) * K);
}
static void enu_to_latlon(float e, float n, double lat0, double lon0,
                          double *lat, double *lon) {
    *lat = lat0 + (double)n / K;
    *lon = lon0 + (double)e / (K * cos(lat0 * M_PI / 180.0));
}

// Confidence-aware breach threshold (safety.cpp).
static float eff_breach_dist(float breach_dist, float acc) {
    float d = breach_dist - acc;
    if (d < 0.10f) d = 0.10f;
    return d;
}

// CRC32 (nvs_storage.cpp crc32_compute, poly 0xEDB88320).
static uint32_t crc32_compute(const uint8_t *data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : (crc >> 1);
    }
    return crc ^ 0xFFFFFFFFu;
}

// perim2 blob: [u32 count][ (double lat, double lon, float acc) x N ][u32 crc]
static std::vector<uint8_t> ll_serialise(const std::vector<double> &lat,
                                         const std::vector<double> &lon,
                                         const std::vector<float>  &acc) {
    uint32_t n = (uint32_t)lat.size();
    size_t data = 4 + (size_t)n * 20;
    std::vector<uint8_t> b(data + 4);
    memcpy(b.data(), &n, 4);
    size_t off = 4;
    for (uint32_t i = 0; i < n; i++) {
        memcpy(b.data() + off, &lat[i], 8); off += 8;
        memcpy(b.data() + off, &lon[i], 8); off += 8;
        memcpy(b.data() + off, &acc[i], 4); off += 4;
    }
    uint32_t crc = crc32_compute(b.data(), data);
    memcpy(b.data() + data, &crc, 4);
    return b;
}
static int ll_deserialise(const std::vector<uint8_t> &b,
                          std::vector<double> &lat, std::vector<double> &lon,
                          std::vector<float> &acc) {
    if (b.size() < 4 + 20 + 4) return 0;
    uint32_t n; memcpy(&n, b.data(), 4);
    size_t data = 4 + (size_t)n * 20;
    if (b.size() != data + 4) return 0;
    uint32_t stored; memcpy(&stored, b.data() + data, 4);
    if (crc32_compute(b.data(), data) != stored) return 0;
    lat.resize(n); lon.resize(n); acc.resize(n);
    size_t off = 4;
    for (uint32_t i = 0; i < n; i++) {
        memcpy(&lat[i], b.data() + off, 8); off += 8;
        memcpy(&lon[i], b.data() + off, 8); off += 8;
        memcpy(&acc[i], b.data() + off, 4); off += 4;
    }
    return (int)n;
}

int main() {
    // ── 1. Round-trip lat/lon → ENU → lat/lon (sub-cm) ───────────────────────
    {
        double lat0 = 51.5000000, lon0 = -0.1000000;          // London-ish origin
        double lat = 51.5004500, lon = -0.0993000;            // ~50 m NE
        float e, n;
        latlon_to_enu(lat, lon, lat0, lon0, &e, &n);
        double rlat, rlon;
        enu_to_latlon(e, n, lat0, lon0, &rlat, &rlon);
        // 1e-7 deg ≈ 1.1 cm — float ENU precision limit.
        CHECK(fabs(rlat - lat) < 1e-7 && fabs(rlon - lon) < 1e-7,
              "lat/lon -> ENU -> lat/lon round-trips to sub-cm");
        // Sanity: ~50 m north-east.
        CHECK(n > 45.0f && n < 55.0f, "northing ~50 m for 0.00045 deg lat");
        CHECK(e  > 40.0f && e  < 55.0f, "easting plausible for 0.0007 deg lon");
    }

    // ── 2. Relative geometry is ~origin-independent ──────────────────────────
    // A perimeter point P and a live position Q, both absolute lat/lon. The
    // vector (P - Q) in ENU must be ~identical whether computed against origin A
    // or a shifted origin B — this is exactly why a lat/lon perimeter cannot
    // drift relative to the live fix when the origin changes between boots.
    {
        double Plat = 51.5005000, Plon = -0.0990000;
        double Qlat = 51.5002000, Qlon = -0.0995000;
        double Alat = 51.5000000, Alon = -0.1000000;          // origin A
        double Blat = 51.5000900, Blon = -0.0999100;          // origin B (~10 m away)

        float Pe_a, Pn_a, Qe_a, Qn_a, Pe_b, Pn_b, Qe_b, Qn_b;
        latlon_to_enu(Plat, Plon, Alat, Alon, &Pe_a, &Pn_a);
        latlon_to_enu(Qlat, Qlon, Alat, Alon, &Qe_a, &Qn_a);
        latlon_to_enu(Plat, Plon, Blat, Blon, &Pe_b, &Pn_b);
        latlon_to_enu(Qlat, Qlon, Blat, Blon, &Qe_b, &Qn_b);

        float de_a = Pe_a - Qe_a, dn_a = Pn_a - Qn_a;
        float de_b = Pe_b - Qe_b, dn_b = Pn_b - Qn_b;
        float drift = sqrtf((de_a - de_b) * (de_a - de_b) +
                            (dn_a - dn_b) * (dn_a - dn_b));
        printf("     relative-vector drift between origins = %.4f m\n", drift);
        CHECK(drift < 0.005f, "P-Q relative vector is origin-independent (<5 mm)");
    }

    // ── 3. Confidence-aware breach margin ────────────────────────────────────
    {
        const float B = 0.80f;  // PERIMETER_BREACH_DIST_M
        CHECK(fabs(eff_breach_dist(B, 0.00f) - 0.80f) < 1e-6, "acc 0    -> 0.80 m");
        CHECK(fabs(eff_breach_dist(B, 0.03f) - 0.77f) < 1e-6, "acc 0.03 -> 0.77 m (RTK fixed ~unchanged)");
        CHECK(fabs(eff_breach_dist(B, 0.50f) - 0.30f) < 1e-6, "acc 0.50 -> 0.30 m (RTK float: tighter)");
        CHECK(fabs(eff_breach_dist(B, 5.00f) - 0.10f) < 1e-6, "acc 5.0  -> 0.10 m floor (no fix)");
    }

    // ── 4. perim2 blob round-trip + corruption detection ─────────────────────
    {
        std::vector<double> lat = {51.5001, 51.5002, 51.5003, 51.5001};
        std::vector<double> lon = {-0.1001, -0.1000, -0.1002, -0.1001};
        std::vector<float>  acc = {0.03f, 0.03f, 0.50f, 0.03f};
        auto blob = ll_serialise(lat, lon, acc);

        std::vector<double> rlat, rlon; std::vector<float> racc;
        int n = ll_deserialise(blob, rlat, rlon, racc);
        bool same = (n == 4);
        for (int i = 0; same && i < n; i++)
            same = (rlat[i] == lat[i] && rlon[i] == lon[i] && racc[i] == acc[i]);
        CHECK(same, "perim2 blob serialise/deserialise round-trips exactly");

        // Worst-case accuracy used by the breach margin.
        float accmax = 0.0f;
        for (float a : racc) if (a > accmax) accmax = a;
        CHECK(fabs(accmax - 0.50f) < 1e-6, "worst-case accuracy = max over points (0.50 m)");

        // Flip one payload byte → CRC must reject.
        auto bad = blob; bad[10] ^= 0xFF;
        std::vector<double> blat, blon; std::vector<float> bacc;
        CHECK(ll_deserialise(bad, blat, blon, bacc) == 0, "corrupt blob rejected by CRC32");
    }

    printf("\n%d failure(s)\n", g_failures);
    return g_failures;
}
