#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "geometry.h"

// ══════════════════════════════════════════════════════════════════════════════
//  nvs_storage.h — RoboMower NVS (Non-Volatile Storage) API
//
//  All persistent state goes through this module:
//    • Polygon blobs (perimeter, nav boundary, working area) — nvs.h blob API
//    • E-stop circular log (20 entries × 44 bytes)           — nvs.h blob API
//    • GPS origin (lat/lon + CRC32)                          — nvs.h blob API
//    • Scalar calibration values (floats, uint16s)           — Preferences API
//
//  Every blob is integrity-checked with a software CRC32 (IEEE 802.3 / Ethernet
//  polynomial). On boot nvs_storage_init() validates all stored blobs and sets
//  internal validity flags. nvs_has_valid_perimeter() returns false until a
//  perimeter has been stored AND passed its CRC check.
//
//  NVS namespace: "mower"  (see ARCHITECTURE.md Section 6)
//
//  Thread safety: callers are responsible for serialising access. These
//  functions do NOT acquire any FreeRTOS mutex internally.
//
//  References:
//    ARCHITECTURE.md  — Section 6 (NVS key registry)
//    ASSUMPTIONS.md   — A04 (EStopEvent), A05 (GPS origin)
// ══════════════════════════════════════════════════════════════════════════════

// ── E-stop log entry ─────────────────────────────────────────────────────────

/**
 * @brief Single E-stop event record (44 bytes).
 *
 * CRC32 is stored separately in NVS as key "estop_crc_N" (where N is the
 * circular buffer slot index 0–19). It is NOT embedded in this struct so that
 * a power-cut between the blob write and the CRC write can be detected on
 * read (missing/mismatching CRC → entry treated as corrupt and skipped).
 *
 * See ASSUMPTIONS.md A04 for the rationale behind the two-key approach.
 */
struct EStopEvent {
    uint32_t timestamp_ms;  ///< millis() at time of E-stop trigger
    float    x;             ///< EKF x position at E-stop (metres, ENU East)
    float    y;             ///< EKF y position at E-stop (metres, ENU North)
    char     reason[32];    ///< Human-readable reason string (null-terminated)
};
static_assert(sizeof(EStopEvent) == 44, "EStopEvent must be 44 bytes");

// ── GPS origin ────────────────────────────────────────────────────────────────

/**
 * @brief GPS origin blob stored in NVS (24 bytes total).
 *
 * Records the WGS84 position of the perimeter origin (first vertex).
 * On boot: if absent or CRC fails, the ENU origin is unset — a perimeter
 * must be uploaded before ENU conversion is active.
 */
struct GpsOriginNvs {
    double   lat_deg;   ///< Origin latitude  (degrees WGS84)
    double   lon_deg;   ///< Origin longitude (degrees WGS84)
    uint32_t crc32;     ///< CRC32 of lat_deg + lon_deg (covers first 16 bytes)
    uint32_t _pad;      ///< Padding to maintain 8-byte alignment (double members)
};
// Actual size: 8 + 8 + 4 + 4 = 24 bytes (compiler aligns to double boundary)
static_assert(sizeof(GpsOriginNvs) == 24, "GpsOriginNvs must be 24 bytes");

// ── Module initialisation ─────────────────────────────────────────────────────

/**
 * @brief Initialise NVS flash and validate all stored blobs.
 *
 * Must be called once from setup() before any other nvs_storage function.
 * Calls nvs_flash_init(), opens the Preferences namespace, then loads each
 * known blob and verifies its CRC32.
 *
 * If a blob is ABSENT (first boot): the corresponding validity flag is set
 * false but no warning is printed — this is normal.
 *
 * If a blob is PRESENT but CRC fails: the validity flag is set false and a
 * Serial warning is printed. The corrupt blob is NOT erased automatically;
 * the caller's module must decide whether to re-record.
 *
 * @return true  All blobs that are present have valid CRC32 checksums.
 * @return false One or more stored blobs failed CRC validation.
 */
bool nvs_storage_init();

// ── Perimeter polygons ────────────────────────────────────────────────────────

/**
 * @brief Save the outer perimeter polygon to NVS with CRC32 integrity check.
 *
 * Serialises the polygon as:
 *   [uint32_t count][float x0][float y0]...[float xN][float yN][uint32_t crc32]
 * Maximum 500 points → maximum blob size 4008 bytes.
 * Updates the internal validity flag on success.
 *
 * @param poly  Polygon to persist (must have at least 3 vertices).
 * @return true  Blob written and committed to flash.
 * @return false poly is empty, serialisation failed, or NVS write error.
 */
bool nvs_save_perimeter(const Polygon &poly);

/**
 * @brief Load the outer perimeter polygon from NVS.
 *
 * Verifies CRC32 before returning data. Updates the internal validity flag.
 *
 * @return Loaded polygon on success. Empty Polygon (pts.empty() == true) if
 *         key is absent, blob is corrupt, or CRC fails.
 */
Polygon nvs_load_perimeter();

/**
 * @brief Save the navigation boundary polygon to NVS with CRC32.
 *
 * The nav boundary is the perimeter inset by NAV_EXCLUSION_INSET_M.
 *
 * @param poly  Pre-computed nav boundary polygon.
 * @return true on success.
 */
bool nvs_save_nav_boundary(const Polygon &poly);

/**
 * @brief Load the navigation boundary polygon from NVS.
 * @return Loaded polygon, or empty Polygon if absent or CRC fails.
 */
Polygon nvs_load_nav_boundary();

/**
 * @brief Save the working area polygon to NVS with CRC32.
 *
 * The working area is the nav boundary inset by HEADLAND_WIDTH_M.
 *
 * @param poly  Pre-computed working area polygon.
 * @return true on success.
 */
bool nvs_save_working_area(const Polygon &poly);

/**
 * @brief Load the working area polygon from NVS.
 * @return Loaded polygon, or empty Polygon if absent or CRC fails.
 */
Polygon nvs_load_working_area();

/**
 * @brief Check whether a valid perimeter polygon is available.
 *
 * Returns true only if a perimeter was stored AND its CRC32 passed
 * validation (either at boot via nvs_storage_init() or after a successful
 * nvs_save_perimeter() call in the current session).
 *
 * This flag gates entry to AUTO mode — the state machine must not allow
 * autonomous mowing unless nvs_has_valid_perimeter() returns true.
 *
 * @return true  Valid perimeter is in NVS.
 * @return false No perimeter stored, or stored perimeter failed CRC check.
 */
bool nvs_has_valid_perimeter();

/**
 * @brief Erase the perimeter and all derived polygons (nav boundary, working
 *        area) from NVS. Resets all three validity flags to false.
 *
 * Called when the operator initiates a new LEARN_PERIMETER session.
 */
void nvs_clear_perimeter();

// ── Scalar configuration (via Preferences API) ───────────────────────────────

/**
 * @brief Store a float value under the given key in the "mower" namespace.
 *
 * Used by: cutting_monitor.cpp ("blade_cal"), state_machine.cpp ("home_x"/"home_y").
 * (imu_bmi270.cpp stores its gyro bias via its OWN "imu" Preferences namespace,
 *  key "gyrobias" — NOT this function. There is no battery cal key: battery
 *  voltage comes from the VESC STATUS_5 packet, no stored float.)
 *
 * @param key    NVS key (≤ 15 chars).
 * @param value  Value to store.
 */
void nvs_set_float(const char *key, float value);

/**
 * @brief Retrieve a float value from the "mower" namespace.
 *
 * @param key          NVS key (≤ 15 chars).
 * @param default_val  Returned if key is absent.
 * @return Stored value, or default_val if key not found.
 */
float nvs_get_float(const char *key, float default_val);

/**
 * @brief Store a uint16_t value under the given key in the "mower" namespace.
 *
 * Not currently used by any module (the servo cut-height calibration that
 * formerly stored "servo_min"/"servo_max" now uses fixed #define values in
 * servo_output.cpp). Retained as a generic helper.
 *
 * @param key    NVS key (≤ 15 chars).
 * @param value  Value to store.
 */
void nvs_set_uint16(const char *key, uint16_t value);

/**
 * @brief Retrieve a uint16_t value from the "mower" namespace.
 *
 * @param key          NVS key (≤ 15 chars).
 * @param default_val  Returned if key is absent.
 * @return Stored value, or default_val if key not found.
 */
uint16_t nvs_get_uint16(const char *key, uint16_t default_val);

// ── E-stop circular log ───────────────────────────────────────────────────────

/**
 * @brief Append an E-stop event to the circular log.
 *
 * Implements the two-key write sequence from ASSUMPTIONS.md A04:
 *   1. Load 880-byte "estops" blob (or create zeroed on first write).
 *   2. Overwrite the slot at estop_idx with the new EStopEvent.
 *   3. Compute CRC32 of the 44-byte entry.
 *   4. Write updated blob back to "estops".
 *   5. Write CRC32 to Preferences key "estop_crc_N" (N = estop_idx).
 *   6. Advance estop_idx = (estop_idx + 1) % 20, persist to "estop_idx".
 *
 * A power cut between steps 4 and 5 leaves the blob written but the CRC
 * absent/stale → the entry is detected as corrupt and skipped on read.
 *
 * The log holds a maximum of 20 entries. When full, the oldest entry is
 * overwritten (circular buffer).
 *
 * @param timestamp_ms  millis() value at E-stop.
 * @param x             EKF x position (metres).
 * @param y             EKF y position (metres).
 * @param reason        Reason string (truncated to 31 chars + null if longer).
 */
void nvs_log_estop(uint32_t timestamp_ms, float x, float y, const char *reason);

/**
 * @brief Read all valid E-stop log entries, oldest first.
 *
 * Entries whose CRC32 does not match the stored "estop_crc_N" key are
 * silently skipped (corrupt or incomplete write — see A04).
 *
 * @param out        Output buffer. Must hold at least max_count entries.
 * @param max_count  Maximum entries to return.
 * @return Number of valid entries written to out (0 if log is empty or all
 *         entries are corrupt).
 */
int nvs_get_estop_log(EStopEvent *out, int max_count);

/**
 * @brief Erase the entire E-stop log.
 *
 * Writes a zeroed 880-byte blob, resets estop_idx to 0, and zeroes all
 * "estop_crc_N" keys so that every slot reads as invalid.
 */
void nvs_clear_estop_log();

// ── GPS origin ────────────────────────────────────────────────────────────────

/**
 * @brief Persist the ENU coordinate origin to NVS.
 *
 * Computes CRC32 over the 16 bytes of (lat_deg, lon_deg) and stores the
 * full GpsOriginNvs struct (24 bytes) as a blob under key "gpsorigin".
 * Updates the internal GPS origin validity flag on success.
 *
 * @param lat_deg  Origin latitude in degrees.
 * @param lon_deg  Origin longitude in degrees.
 * @return true on success.
 */
bool nvs_save_gps_origin(double lat_deg, double lon_deg);

/**
 * @brief Load the ENU coordinate origin from NVS.
 *
 * Verifies the embedded CRC32 before returning values.
 *
 * @param lat_deg  Output: latitude in degrees.
 * @param lon_deg  Output: longitude in degrees.
 * @return true  Origin loaded and CRC passed.
 * @return false Key absent or CRC mismatch (outputs are not modified).
 */
bool nvs_load_gps_origin(double *lat_deg, double *lon_deg);

/**
 * @brief Check whether a valid GPS origin is stored in NVS.
 *
 * Returns the internal validity flag set by nvs_storage_init() or
 * nvs_save_gps_origin(). Does NOT re-read NVS on every call.
 *
 * @return true  A valid (CRC-verified) GPS origin is stored.
 * @return false No origin stored, or stored origin failed CRC check.
 */
bool nvs_has_gps_origin();
