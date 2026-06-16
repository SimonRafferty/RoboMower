// ══════════════════════════════════════════════════════════════════════════════
//  nvs_storage.cpp — RoboMower NVS storage implementation
//
//  Design notes
//  ────────────
//  • All blobs (polygons, e-stop log, GPS origin) are stored via the ESP-IDF
//    nvs.h low-level API so that arbitrary-length blobs can be written atomically.
//  • All scalar calibration values (floats, uint16s) are stored via the Arduino
//    Preferences library which wraps NVS scalars with a convenient typed API.
//  • Both APIs share the same NVS namespace "mower".
//  • The Preferences handle (g_prefs) is opened once in nvs_storage_init() and
//    left open for the lifetime of the application (safe — NVS allows multiple
//    concurrent handles on the same namespace).
//
//  Polygon blob format
//  ───────────────────
//  [ uint32_t count ][ float x0 ][ float y0 ][ float x1 ][ float y1 ]...[ uint32_t crc32 ]
//  Total = 4 + count*8 + 4 bytes.
//  CRC32 covers bytes 0 .. (blob_size - 5) inclusive (everything before crc field).
//
//  E-stop log format
//  ─────────────────
//  NVS key "estops"      : blob, 20 × 44 = 880 bytes (20 × EStopEvent)
//  NVS key "estop_idx"   : uint16 in Preferences, circular write index (0–19)
//  NVS keys "estop_crc_N": uint32 in Preferences (N = 0..19), CRC32 per slot
//  Write atomicity: blob is written BEFORE the CRC key. A power cut between
//  the two writes leaves the entry detectable as corrupt (stale/absent CRC).
//
//  GPS origin format
//  ─────────────────
//  NVS key "gpsorigin": blob, sizeof(GpsOriginNvs) = 24 bytes.
//  CRC32 embedded in struct, computed over first 16 bytes (lat + lon).
//
//  Namespace conflict note (see BLOCKERS.md B03)
//  ─────────────────────────────────────────────
//  Task prompt specified namespace "robomower". ARCHITECTURE.md Section 6
//  specifies "mower". This file uses "mower" to match the authoritative registry.
// ══════════════════════════════════════════════════════════════════════════════

#include "nvs_storage.h"
#include "config.h"
#include <Preferences.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ── Constants ─────────────────────────────────────────────────────────────────

static const char* NVS_NAMESPACE   = "mower";      ///< Shared by Preferences and nvs.h calls
static const int   ESTOP_MAX       = 20;            ///< Circular log capacity
static const int   ESTOP_BLOB_SIZE = ESTOP_MAX * sizeof(EStopEvent);  ///< 880 bytes

// NVS blob keys
static const char* KEY_PERIM       = "perim";    ///< legacy ENU-float perimeter (migrated to perim2)
static const char* KEY_PERIM_LL    = "perim2";   ///< canonical lat/lon + per-point accuracy perimeter
static const char* KEY_NAVPOLY     = "navpoly";
static const char* KEY_WORKPOLY    = "workpoly";
static const char* KEY_GPSORIGIN   = "gpsorigin";
static const char* KEY_ESTOPS      = "estops";

// Preferences scalar keys
static const char* PKEY_ESTOP_IDX  = "estop_idx";  ///< uint16 circular index

// ── Module state ──────────────────────────────────────────────────────────────

static Preferences g_prefs;

/** Boot-time CRC validation results for each blob.
 *  false = absent (first boot) OR present but CRC failed. */
static bool g_valid_perim    = false;
static bool g_valid_navpoly  = false;
static bool g_valid_workpoly = false;
static bool g_valid_gpsorigin = false;

// ── CRC32 (IEEE 802.3 / Ethernet polynomial) ─────────────────────────────────

/**
 * @brief Compute CRC32 over a byte buffer.
 *
 * Uses the standard IEEE 802.3 reflected polynomial (0xEDB88320).
 * Produces the same values as zlib crc32() and Ethernet FCS.
 */
static uint32_t crc32_compute(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else         crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}

// ── NVS handle helpers ────────────────────────────────────────────────────────

/** Open a read-write NVS handle in NVS_NAMESPACE. Returns 0 on failure. */
static nvs_handle_t nvs_open_rw()
{
    nvs_handle_t h = 0;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        DBG_PRINTF("[NVS] nvs_open failed: %s\n", esp_err_to_name(err));
        return 0;
    }
    return h;
}

/** Open a read-only NVS handle in NVS_NAMESPACE. Returns 0 on failure. */
static nvs_handle_t nvs_open_ro()
{
    nvs_handle_t h = 0;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        // NVS_READONLY fails with ESP_ERR_NVS_NOT_FOUND if namespace is empty
        // (first boot). That is not an error condition worth printing.
        return 0;
    }
    return h;
}

// ── Polygon serialisation helpers ─────────────────────────────────────────────

/**
 * @brief Serialise a Polygon to a heap-allocated byte buffer.
 *
 * Format: [uint32_t count][float x0][float y0]...[float xN][float yN][uint32_t crc32]
 * Caller must free() the returned buffer.
 *
 * @param poly      Polygon to serialise.
 * @param out_size  Set to the number of bytes in the returned buffer.
 * @return Pointer to allocated buffer, or nullptr on failure.
 */
static uint8_t *poly_serialise(const Polygon &poly, size_t *out_size)
{
    uint32_t count = (uint32_t)poly.pts.size();
    if (count == 0) return nullptr;

    // Enforce size limit on save — deserialise already checks this, but a save
    // with >500 points would write >4008 bytes and may exceed an NVS page. (HIGH-2 fix)
    if (count > 500) {
        DBG_PRINTF("[NVS] ERROR: polygon has %u points, exceeds 500-point limit — save rejected\n",
                      (unsigned)count);
        return nullptr;
    }

    // Layout: 4 bytes count + count*8 bytes coord pairs + 4 bytes CRC
    size_t data_size = 4 + (size_t)count * 8;
    size_t total     = data_size + 4;

    uint8_t *buf = (uint8_t *)malloc(total);
    if (!buf) {
        DBG_PRINTLN("[NVS] poly_serialise: malloc failed");
        return nullptr;
    }

    // Write count (little-endian native)
    memcpy(buf, &count, 4);

    // Write interleaved x, y pairs
    size_t off = 4;
    for (uint32_t i = 0; i < count; i++) {
        float x = poly.pts[i].x;
        float y = poly.pts[i].y;
        memcpy(buf + off,     &x, 4);
        memcpy(buf + off + 4, &y, 4);
        off += 8;
    }

    // Append CRC32 of everything before this field
    uint32_t crc = crc32_compute(buf, data_size);
    memcpy(buf + data_size, &crc, 4);

    *out_size = total;
    return buf;
}

/**
 * @brief Deserialise a Polygon from a byte buffer, verifying CRC32.
 *
 * @param buf   Buffer in the format written by poly_serialise().
 * @param size  Buffer size in bytes.
 * @return Decoded Polygon, or empty Polygon if size < 8, count mismatch,
 *         or CRC fails.
 */
static Polygon poly_deserialise(const uint8_t *buf, size_t size)
{
    Polygon result;
    if (!buf || size < 8) return result;   // need at least count + 1 point + crc

    uint32_t count;
    memcpy(&count, buf, 4);

    // Expected size: 4 + count*8 + 4
    size_t expected = 4 + (size_t)count * 8 + 4;
    if (size != expected) {
        DBG_PRINTF("[NVS] poly_deserialise: size mismatch (got %u, want %u)\n",
                      (unsigned)size, (unsigned)expected);
        return result;
    }
    if (count == 0 || count > 500) {
        DBG_PRINTF("[NVS] poly_deserialise: invalid count %u\n", (unsigned)count);
        return result;
    }

    // Verify CRC (covers all bytes before the last 4)
    size_t data_size = 4 + (size_t)count * 8;
    uint32_t computed = crc32_compute(buf, data_size);
    uint32_t stored;
    memcpy(&stored, buf + data_size, 4);

    if (computed != stored) {
        DBG_PRINTF("[NVS] poly_deserialise: CRC mismatch (0x%08X vs 0x%08X)\n",
                      (unsigned)computed, (unsigned)stored);
        return result;
    }

    // Decode points
    result.pts.reserve(count);
    size_t off = 4;
    for (uint32_t i = 0; i < count; i++) {
        float x, y;
        memcpy(&x, buf + off,     4);
        memcpy(&y, buf + off + 4, 4);
        result.pts.emplace_back(x, y);
        off += 8;
    }
    return result;
}

// ── Internal polygon save/load (shared by all three polygon keys) ─────────────

/**
 * @brief Write a polygon blob to NVS, updating validity flag on success.
 * @param key        NVS key (e.g. "perim", "navpoly", "workpoly").
 * @param poly       Polygon to store.
 * @param valid_flag Pointer to the module-level validity flag to update.
 * @return true on success.
 */
static bool poly_save_impl(const char *key, const Polygon &poly, bool *valid_flag)
{
    if (poly.pts.empty()) {
        DBG_PRINTF("[NVS] poly_save_impl(%s): empty polygon rejected\n", key);
        return false;
    }

    size_t   blob_size;
    uint8_t *blob = poly_serialise(poly, &blob_size);
    if (!blob) return false;

    nvs_handle_t h = nvs_open_rw();
    if (!h) { free(blob); return false; }

    esp_err_t err = nvs_set_blob(h, key, blob, blob_size);
    free(blob);

    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err != ESP_OK) {
        DBG_PRINTF("[NVS] poly_save_impl(%s): write error: %s\n",
                      key, esp_err_to_name(err));
        *valid_flag = false;
        return false;
    }

    *valid_flag = true;
    return true;
}

/**
 * @brief Read and CRC-verify a polygon blob from NVS, updating validity flag.
 * @param key        NVS key.
 * @param valid_flag Pointer to the module-level validity flag to update.
 * @return Decoded polygon, or empty polygon on failure.
 */
static Polygon poly_load_impl(const char *key, bool *valid_flag)
{
    Polygon result;

    nvs_handle_t h = nvs_open_ro();
    if (!h) {
        *valid_flag = false;
        return result;
    }

    // Query blob size
    size_t    blob_size = 0;
    esp_err_t err = nvs_get_blob(h, key, nullptr, &blob_size);
    if (err == ESP_ERR_NVS_NOT_FOUND || blob_size == 0) {
        // Absent on first boot — not an error
        nvs_close(h);
        *valid_flag = false;
        return result;
    }
    if (err != ESP_OK) {
        DBG_PRINTF("[NVS] poly_load_impl(%s): size query error: %s\n",
                      key, esp_err_to_name(err));
        nvs_close(h);
        *valid_flag = false;
        return result;
    }

    uint8_t *buf = (uint8_t *)malloc(blob_size);
    if (!buf) {
        DBG_PRINTF("[NVS] poly_load_impl(%s): malloc failed\n", key);
        nvs_close(h);
        *valid_flag = false;
        return result;
    }

    err = nvs_get_blob(h, key, buf, &blob_size);
    nvs_close(h);

    if (err != ESP_OK) {
        DBG_PRINTF("[NVS] poly_load_impl(%s): read error: %s\n",
                      key, esp_err_to_name(err));
        free(buf);
        *valid_flag = false;
        return result;
    }

    result = poly_deserialise(buf, blob_size);
    free(buf);

    *valid_flag = !result.pts.empty();

    if (!*valid_flag) {
        DBG_PRINTF("[NVS] poly_load_impl(%s): CRC failed or corrupt blob\n", key);
    }
    return result;
}

// ── Module initialisation ─────────────────────────────────────────────────────

bool nvs_storage_init()
{
    // NVS flash is typically initialised by the ESP32 Arduino framework before
    // setup() is called. Calling nvs_flash_init() again is safe — if it was
    // already done it returns ESP_ERR_NVS_NO_FREE_PAGES or similar, which we
    // handle by erasing (only if NVS is otherwise unusable).
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        DBG_PRINTLN("[NVS] NVS partition full or version mismatch — erasing");
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Open Preferences handle for scalars (stays open for application lifetime)
    if (!g_prefs.begin(NVS_NAMESPACE, false)) {
        DBG_PRINTLN("[NVS] WARNING: Preferences.begin() failed");
        // Non-fatal — scalar reads will return defaults
    }

    // ── Validate polygon blobs ───────────────────────────────────────────────
    bool all_ok = true;

    {   // Perimeter
        Polygon p = poly_load_impl(KEY_PERIM, &g_valid_perim);
        if (!p.pts.empty()) {
            // CRC passed
        } else if (g_valid_perim == false) {
            // Either absent (OK) or CRC failed (logged inside poly_load_impl)
        }
        if (g_valid_perim == false) {
            // Check whether the key actually exists (absent vs corrupt)
            nvs_handle_t h = nvs_open_ro();
            if (h) {
                size_t sz = 0;
                esp_err_t e = nvs_get_blob(h, KEY_PERIM, nullptr, &sz);
                nvs_close(h);
                if (e == ESP_OK && sz > 0) {
                    // Key exists but CRC failed
                    DBG_PRINTLN("[NVS] WARN: perim blob present but CRC FAILED — AUTO mode blocked");
                    all_ok = false;
                }
                // If ESP_ERR_NVS_NOT_FOUND: absent on first boot, not an error
            }
        }
    }

    {   // Nav boundary
        Polygon p = poly_load_impl(KEY_NAVPOLY, &g_valid_navpoly);
        (void)p;
        if (g_valid_navpoly == false) {
            nvs_handle_t h = nvs_open_ro();
            if (h) {
                size_t sz = 0;
                esp_err_t e = nvs_get_blob(h, KEY_NAVPOLY, nullptr, &sz);
                nvs_close(h);
                if (e == ESP_OK && sz > 0) {
                    DBG_PRINTLN("[NVS] WARN: navpoly blob present but CRC FAILED");
                    all_ok = false;
                }
            }
        }
    }

    {   // Working area
        Polygon p = poly_load_impl(KEY_WORKPOLY, &g_valid_workpoly);
        (void)p;
        if (g_valid_workpoly == false) {
            nvs_handle_t h = nvs_open_ro();
            if (h) {
                size_t sz = 0;
                esp_err_t e = nvs_get_blob(h, KEY_WORKPOLY, nullptr, &sz);
                nvs_close(h);
                if (e == ESP_OK && sz > 0) {
                    DBG_PRINTLN("[NVS] WARN: workpoly blob present but CRC FAILED");
                    all_ok = false;
                }
            }
        }
    }

    // ── Validate GPS origin ──────────────────────────────────────────────────
    {
        nvs_handle_t h = nvs_open_ro();
        if (h) {
            GpsOriginNvs origin;
            size_t sz = sizeof(origin);
            esp_err_t e = nvs_get_blob(h, KEY_GPSORIGIN, &origin, &sz);
            nvs_close(h);

            if (e == ESP_ERR_NVS_NOT_FOUND) {
                g_valid_gpsorigin = false;  // absent — normal on first boot
            } else if (e != ESP_OK || sz != sizeof(GpsOriginNvs)) {
                DBG_PRINTLN("[NVS] WARN: gpsorigin blob read error or wrong size");
                g_valid_gpsorigin = false;
                all_ok = false;
            } else {
                // Verify CRC over lat_deg + lon_deg (16 bytes)
                uint32_t computed = crc32_compute((const uint8_t *)&origin, 16);
                if (computed != origin.crc32) {
                    DBG_PRINTF("[NVS] WARN: gpsorigin CRC FAILED (0x%08X vs 0x%08X)\n",
                                  (unsigned)computed, (unsigned)origin.crc32);
                    g_valid_gpsorigin = false;
                    all_ok = false;
                } else {
                    g_valid_gpsorigin = true;
                }
            }
        } else {
            g_valid_gpsorigin = false;  // namespace empty (first boot)
        }
    }

    // ── Validate E-stop log ─────────────────────────────────────────────────-
    // Individual entry validation happens on read (nvs_get_estop_log). Here we
    // just verify the blob size is correct if it exists.
    {
        nvs_handle_t h = nvs_open_ro();
        if (h) {
            size_t sz = 0;
            esp_err_t e = nvs_get_blob(h, KEY_ESTOPS, nullptr, &sz);
            nvs_close(h);
            if (e == ESP_OK && sz != (size_t)ESTOP_BLOB_SIZE) {
                DBG_PRINTF("[NVS] WARN: estops blob size wrong (%u vs %d) — log cleared\n",
                              (unsigned)sz, ESTOP_BLOB_SIZE);
                // Corrupt blob size — erase it to prevent undefined read behaviour
                nvs_handle_t hw = nvs_open_rw();
                if (hw) {
                    nvs_erase_key(hw, KEY_ESTOPS);
                    nvs_commit(hw);
                    nvs_close(hw);
                }
                g_prefs.putUShort(PKEY_ESTOP_IDX, 0);
                all_ok = false;
            }
        }
    }

    DBG_PRINTF("[NVS] init: perim=%d navpoly=%d workpoly=%d gpsorigin=%d all_ok=%d\n",
                  g_valid_perim, g_valid_navpoly, g_valid_workpoly, g_valid_gpsorigin, all_ok);

    return all_ok;
}

// ── Perimeter polygon API ─────────────────────────────────────────────────────

bool nvs_save_perimeter(const Polygon &poly)
{
    return poly_save_impl(KEY_PERIM, poly, &g_valid_perim);
}

Polygon nvs_load_perimeter()
{
    return poly_load_impl(KEY_PERIM, &g_valid_perim);
}

bool nvs_save_nav_boundary(const Polygon &poly)
{
    return poly_save_impl(KEY_NAVPOLY, poly, &g_valid_navpoly);
}

Polygon nvs_load_nav_boundary()
{
    return poly_load_impl(KEY_NAVPOLY, &g_valid_navpoly);
}

bool nvs_save_working_area(const Polygon &poly)
{
    return poly_save_impl(KEY_WORKPOLY, poly, &g_valid_workpoly);
}

Polygon nvs_load_working_area()
{
    return poly_load_impl(KEY_WORKPOLY, &g_valid_workpoly);
}

bool nvs_has_valid_perimeter()
{
    return g_valid_perim;
}

// ── Canonical perimeter: absolute lat/lon + per-point accuracy ────────────────
// Blob: [uint32 count][ (double lat, double lon, float acc) × count ][uint32 crc32]
// Origin-independent — re-derived to ENU on boot against the current origin.
static constexpr size_t LL_PT_BYTES = sizeof(double) * 2 + sizeof(float);  // 20

bool nvs_save_perimeter_ll(const double *lat, const double *lon,
                           const float *acc, int count)
{
    if (!lat || !lon || !acc || count < 3 || count > 500) return false;

    const size_t data_size = 4 + (size_t)count * LL_PT_BYTES;
    const size_t blob_size = data_size + 4;
    uint8_t *blob = (uint8_t *)malloc(blob_size);
    if (!blob) return false;

    uint32_t c = (uint32_t)count;
    memcpy(blob, &c, 4);
    size_t off = 4;
    for (int i = 0; i < count; i++) {
        memcpy(blob + off, &lat[i], 8); off += 8;
        memcpy(blob + off, &lon[i], 8); off += 8;
        memcpy(blob + off, &acc[i], 4); off += 4;
    }
    uint32_t crc = crc32_compute(blob, data_size);
    memcpy(blob + data_size, &crc, 4);

    nvs_handle_t h = nvs_open_rw();
    if (!h) { free(blob); return false; }
    esp_err_t err = nvs_set_blob(h, KEY_PERIM_LL, blob, blob_size);
    free(blob);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) {
        DBG_PRINTF("[NVS] save_perimeter_ll: write error: %s\n", esp_err_to_name(err));
        return false;
    }
    return true;
}

int nvs_load_perimeter_ll(double *lat, double *lon, float *acc, int max_count)
{
    if (!lat || !lon || !acc || max_count <= 0) return 0;

    nvs_handle_t h = nvs_open_ro();
    if (!h) return 0;

    size_t    blob_size = 0;
    esp_err_t err = nvs_get_blob(h, KEY_PERIM_LL, nullptr, &blob_size);
    if (err != ESP_OK || blob_size < 4 + LL_PT_BYTES + 4) { nvs_close(h); return 0; }

    uint8_t *buf = (uint8_t *)malloc(blob_size);
    if (!buf) { nvs_close(h); return 0; }
    err = nvs_get_blob(h, KEY_PERIM_LL, buf, &blob_size);
    nvs_close(h);
    if (err != ESP_OK) { free(buf); return 0; }

    uint32_t count = 0;
    memcpy(&count, buf, 4);
    if (count < 3 || count > 500) { free(buf); return 0; }

    const size_t data_size = 4 + (size_t)count * LL_PT_BYTES;
    if (blob_size != data_size + 4) { free(buf); return 0; }

    uint32_t computed = crc32_compute(buf, data_size);
    uint32_t stored = 0;
    memcpy(&stored, buf + data_size, 4);
    if (computed != stored) {
        DBG_PRINTF("[NVS] load_perimeter_ll: CRC mismatch (0x%08X vs 0x%08X)\n",
                      (unsigned)computed, (unsigned)stored);
        free(buf);
        return 0;
    }

    int n = (int)count;
    if (n > max_count) n = max_count;
    size_t off = 4;
    for (int i = 0; i < n; i++) {
        memcpy(&lat[i], buf + off, 8); off += 8;
        memcpy(&lon[i], buf + off, 8); off += 8;
        memcpy(&acc[i], buf + off, 4); off += 4;
    }
    free(buf);
    return n;
}

bool nvs_has_perimeter_ll()
{
    nvs_handle_t h = nvs_open_ro();
    if (!h) return false;
    size_t sz = 0;
    esp_err_t err = nvs_get_blob(h, KEY_PERIM_LL, nullptr, &sz);
    nvs_close(h);
    return (err == ESP_OK && sz >= 4 + LL_PT_BYTES + 4);
}

void nvs_clear_perimeter()
{
    nvs_handle_t h = nvs_open_rw();
    if (h) {
        nvs_erase_key(h, KEY_PERIM);
        nvs_erase_key(h, KEY_PERIM_LL);
        nvs_erase_key(h, KEY_NAVPOLY);
        nvs_erase_key(h, KEY_WORKPOLY);
        nvs_commit(h);
        nvs_close(h);
    }
    g_valid_perim    = false;
    g_valid_navpoly  = false;
    g_valid_workpoly = false;
    DBG_PRINTLN("[NVS] Perimeter and derived polygons erased");
}

// ── Scalar calibration API (Preferences) ─────────────────────────────────────

void nvs_set_float(const char *key, float value)
{
    g_prefs.putFloat(key, value);
}

float nvs_get_float(const char *key, float default_val)
{
    return g_prefs.getFloat(key, default_val);
}

void nvs_set_uint16(const char *key, uint16_t value)
{
    g_prefs.putUShort(key, value);
}

uint16_t nvs_get_uint16(const char *key, uint16_t default_val)
{
    return g_prefs.getUShort(key, default_val);
}

// ── E-stop circular log ───────────────────────────────────────────────────────

void nvs_log_estop(uint32_t timestamp_ms, float x, float y, const char *reason)
{
    // ── Step 1: load existing blob (or initialise blank) ──────────────────────
    uint8_t blob[ESTOP_BLOB_SIZE];
    memset(blob, 0, sizeof(blob));

    nvs_handle_t h = nvs_open_rw();
    if (!h) {
        DBG_PRINTLN("[NVS] nvs_log_estop: failed to open NVS");
        return;
    }

    size_t    sz  = sizeof(blob);
    esp_err_t err = nvs_get_blob(h, KEY_ESTOPS, blob, &sz);
    // If not found (first write) or wrong size: start fresh with zeroed buffer
    if (err == ESP_ERR_NVS_NOT_FOUND || sz != (size_t)ESTOP_BLOB_SIZE) {
        memset(blob, 0, sizeof(blob));
    }

    // ── Step 2: build the new entry ───────────────────────────────────────────
    uint16_t idx = g_prefs.getUShort(PKEY_ESTOP_IDX, 0);
    if (idx >= ESTOP_MAX) idx = 0;  // guard against corrupt index

    EStopEvent entry;
    entry.timestamp_ms = timestamp_ms;
    entry.x            = x;
    entry.y            = y;
    memset(entry.reason, 0, sizeof(entry.reason));
    if (reason) {
        strncpy(entry.reason, reason, sizeof(entry.reason) - 1);
    }

    // Copy entry into blob at slot idx
    memcpy(blob + idx * sizeof(EStopEvent), &entry, sizeof(EStopEvent));

    // ── Step 3: compute CRC32 of the 44-byte entry ────────────────────────────
    uint32_t entry_crc = crc32_compute((const uint8_t *)&entry, sizeof(EStopEvent));

    // ── Step 4: write updated blob ────────────────────────────────────────────
    err = nvs_set_blob(h, KEY_ESTOPS, blob, sizeof(blob));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err != ESP_OK) {
        DBG_PRINTF("[NVS] nvs_log_estop: blob write failed: %s\n",
                      esp_err_to_name(err));
        return;  // Do NOT advance index if blob write failed
    }

    // ── Step 5: write per-slot CRC via Preferences ────────────────────────────
    char crc_key[16];
    snprintf(crc_key, sizeof(crc_key), "estop_crc_%d", (int)idx);
    g_prefs.putUInt(crc_key, entry_crc);

    // ── Step 6: advance circular index ────────────────────────────────────────
    uint16_t next_idx = (uint16_t)((idx + 1) % ESTOP_MAX);
    g_prefs.putUShort(PKEY_ESTOP_IDX, next_idx);

    DBG_PRINTF("[NVS] E-stop logged at slot %d: ts=%u x=%.2f y=%.2f reason=%s\n",
                  (int)idx, (unsigned)timestamp_ms, x, y, entry.reason);
}

int nvs_get_estop_log(EStopEvent *out, int max_count)
{
    if (!out || max_count <= 0) return 0;

    // Load the blob
    uint8_t blob[ESTOP_BLOB_SIZE];
    memset(blob, 0, sizeof(blob));

    nvs_handle_t h = nvs_open_ro();
    if (h) {
        size_t sz = sizeof(blob);
        esp_err_t err = nvs_get_blob(h, KEY_ESTOPS, blob, &sz);
        nvs_close(h);
        if (err != ESP_OK || sz != (size_t)ESTOP_BLOB_SIZE) {
            // Blob absent or corrupt — return zero entries
            return 0;
        }
    } else {
        return 0;
    }

    // Read oldest-first: start at current write index (= oldest slot)
    uint16_t write_idx = g_prefs.getUShort(PKEY_ESTOP_IDX, 0);
    if (write_idx >= ESTOP_MAX) write_idx = 0;

    int count = 0;
    for (int i = 0; i < ESTOP_MAX && count < max_count; i++) {
        int slot = (write_idx + i) % ESTOP_MAX;

        // Retrieve stored CRC for this slot
        char crc_key[16];
        snprintf(crc_key, sizeof(crc_key), "estop_crc_%d", slot);
        uint32_t stored_crc = g_prefs.getUInt(crc_key, 0xFFFFFFFF);

        // Compute CRC of the 44 bytes in this slot
        const uint8_t *entry_bytes = blob + slot * sizeof(EStopEvent);
        uint32_t computed_crc = crc32_compute(entry_bytes, sizeof(EStopEvent));

        if (computed_crc != stored_crc) {
            // Corrupt or never-written slot — skip silently
            continue;
        }

        memcpy(&out[count], entry_bytes, sizeof(EStopEvent));
        count++;
    }
    return count;
}

void nvs_clear_estop_log()
{
    // Write a zeroed blob to overwrite all entries
    uint8_t blank[ESTOP_BLOB_SIZE];
    memset(blank, 0, sizeof(blank));

    nvs_handle_t h = nvs_open_rw();
    if (h) {
        nvs_set_blob(h, KEY_ESTOPS, blank, sizeof(blank));
        nvs_commit(h);
        nvs_close(h);
    }

    // Reset write index
    g_prefs.putUShort(PKEY_ESTOP_IDX, 0);

    // Zero all per-slot CRC keys so every slot reads as invalid
    char crc_key[16];
    for (int i = 0; i < ESTOP_MAX; i++) {
        snprintf(crc_key, sizeof(crc_key), "estop_crc_%d", i);
        g_prefs.putUInt(crc_key, 0);
        // CRC32 of 44 zero-bytes is non-zero (≈ 0xACD157E4), so storing 0
        // guarantees a mismatch on read → all slots correctly skipped.
    }

    DBG_PRINTLN("[NVS] E-stop log cleared");
}

// ── GPS origin ────────────────────────────────────────────────────────────────

bool nvs_save_gps_origin(double lat_deg, double lon_deg)
{
    GpsOriginNvs origin;
    origin.lat_deg = lat_deg;
    origin.lon_deg = lon_deg;
    // CRC covers first 16 bytes: lat_deg (8) + lon_deg (8)
    origin.crc32   = crc32_compute((const uint8_t *)&origin, 16);

    nvs_handle_t h = nvs_open_rw();
    if (!h) return false;

    esp_err_t err = nvs_set_blob(h, KEY_GPSORIGIN, &origin, sizeof(origin));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);

    if (err != ESP_OK) {
        DBG_PRINTF("[NVS] nvs_save_gps_origin: write error: %s\n",
                      esp_err_to_name(err));
        g_valid_gpsorigin = false;
        return false;
    }

    g_valid_gpsorigin = true;
    DBG_PRINTF("[NVS] GPS origin saved: %.7f, %.7f\n", lat_deg, lon_deg);
    return true;
}

bool nvs_load_gps_origin(double *lat_deg, double *lon_deg)
{
    if (!lat_deg || !lon_deg) return false;

    nvs_handle_t h = nvs_open_ro();
    if (!h) {
        g_valid_gpsorigin = false;
        return false;
    }

    GpsOriginNvs origin;
    size_t    sz  = sizeof(origin);
    esp_err_t err = nvs_get_blob(h, KEY_GPSORIGIN, &origin, &sz);
    nvs_close(h);

    if (err == ESP_ERR_NVS_NOT_FOUND) {
        g_valid_gpsorigin = false;
        return false;
    }
    if (err != ESP_OK || sz != sizeof(GpsOriginNvs)) {
        DBG_PRINTF("[NVS] nvs_load_gps_origin: read error: %s\n",
                      esp_err_to_name(err));
        g_valid_gpsorigin = false;
        return false;
    }

    // Verify CRC over lat + lon (16 bytes)
    uint32_t computed = crc32_compute((const uint8_t *)&origin, 16);
    if (computed != origin.crc32) {
        DBG_PRINTF("[NVS] nvs_load_gps_origin: CRC FAILED (0x%08X vs 0x%08X)\n",
                      (unsigned)computed, (unsigned)origin.crc32);
        g_valid_gpsorigin = false;
        return false;
    }

    *lat_deg = origin.lat_deg;
    *lon_deg = origin.lon_deg;
    g_valid_gpsorigin = true;
    return true;
}

bool nvs_has_gps_origin()
{
    return g_valid_gpsorigin;
}
