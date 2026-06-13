// ══════════════════════════════════════════════════════════════════════════════
//  rtk_gps.cpp — RoboMower RTK GPS Module
//
//  Responsibilities:
//    • Serial1 bidirectional UART to DFRobot RTK LoRa GPS module (RX=10, TX=14)
//    • Uses DFRobot_RTK_LoRa library — request/response protocol (not NMEA stream)
//    • Latitude/longitude polled via getLat()/getLon(), quality via getQuality()
//    • Flat-earth ENU conversion relative to perimeter origin (first vertex)
//    • Antenna-to-steering-centre offset transform (uses EKF heading)
//    • ENU origin set only by perimeter upload; persisted to NVS across reboots
//    • FreeRTOS poll task (Core 0, priority 6, stack 4096 — per ARCHITECTURE.md)
//
//  Requires: DFRobot_RTK_LoRa library — install via Arduino Library Manager.
//  See HANDOFFS/07_rtk_gps/HANDOFF.md for design notes.
// ══════════════════════════════════════════════════════════════════════════════

#include "rtk_gps.h"
#include "config.h"
#include "mower_config.h"
#include "sys_log.h"

#include <DFRobot_RTK_LoRa.h>

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "nvs_flash.h"
#include "nvs.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ── Forward declaration ───────────────────────────────────────────────────────
#include "ekf_localiser.h"  // for ekf_get_pose()


// ── FreeRTOS task parameters ──────────────────────────────────────────────────
// NOTE: Agent 07 task spec stated priority=15, stack=3072. However, ARCHITECTURE.md
// Section 2 specifies priority=6, stack=4096 for gpsTask, consistent with the
// deliberate priority rationale (GPS at 1Hz is lowest-urgency sensor).
// ARCHITECTURE.md values are used here. See BLOCKERS.md entry B03.
#define GPS_TASK_PRIORITY  6
#define GPS_TASK_STACK     4096

// ── NVS namespace / key ───────────────────────────────────────────────────────
#define GPS_NVS_NAMESPACE  "gps"
#define GPS_NVS_KEY        "gpsorigin"

// Poll interval — matches DFRobot example's delay(300) cadence.
// getDataFlush() is checked at this rate; data is only read when it returns true.
#define GPS_POLL_INTERVAL_MS  300


// ── Internal NVS origin struct (ASSUMPTIONS.md A05) ──────────────────────────
// Stored as a single blob under GPS_NVS_NAMESPACE / GPS_NVS_KEY.
// CRC32 is computed over the first 16 bytes (originLat + originLon).
struct GpsNvsOrigin {
    double   originLat;  // degrees
    double   originLon;  // degrees
    uint32_t crc32;      // CRC32 over {originLat, originLon}
};


// ── DFRobot RTK LoRa GPS instance ────────────────────────────────────────────
// For ESP32, pass rx/tx pins to the constructor — the library calls
// Serial1.begin() internally inside begin(). Do NOT call Serial1.begin() separately.
static DFRobot_RTK_LoRa_UART s_gps(&Serial1, GPS_BAUD_RATE, GPS_RX_PIN, GPS_TX_PIN);

// ── Module state ──────────────────────────────────────────────────────────────
static GpsMeasurement    s_measurement       = {};   // latest polled measurement
static GpsOrigin         s_origin            = {};   // current ENU origin
static uint32_t          s_last_valid_ms     = 0;    // millis() of last valid poll
static SemaphoreHandle_t s_mutex             = nullptr;
static GpsFix            s_last_logged_fix   = GPS_FIX_NONE;  // for fix-change log events



// ── CRC32 (nibble-LUT, no large static table) ─────────────────────────────────
static uint32_t crc32_compute(const void* data, size_t len) {
    // Standard CRC32 half-byte look-up table (reflected polynomial 0xEDB88320)
    static const uint32_t lut[16] = {
        0x00000000UL, 0x1DB71064UL, 0x3B6E20C8UL, 0x26D930ACUL,
        0x76DC4190UL, 0x6B6B51F4UL, 0x4DB26158UL, 0x5005713CUL,
        0xEDB88320UL, 0xF00F9344UL, 0xD6D6A3E8UL, 0xCB61B38CUL,
        0x9B64C2B0UL, 0x86D3D2D4UL, 0xA00AE278UL, 0xBDBDF21CUL
    };
    uint32_t crc = 0xFFFFFFFFUL;
    const uint8_t* p = reinterpret_cast<const uint8_t*>(data);
    for (size_t i = 0; i < len; ++i) {
        crc = (crc >> 4) ^ lut[(crc ^  p[i])       & 0x0FU];
        crc = (crc >> 4) ^ lut[(crc ^ (p[i] >> 4)) & 0x0FU];
    }
    return crc ^ 0xFFFFFFFFUL;
}


// ── NVS origin persistence ────────────────────────────────────────────────────

/** Save origin to NVS with CRC32. Silently ignores NVS errors. */
static void nvs_origin_save(double lat, double lon) {
    GpsNvsOrigin obj;
    obj.originLat = lat;
    obj.originLon = lon;
    obj.crc32     = crc32_compute(&obj, 2 * sizeof(double));

    nvs_handle_t h;
    if (nvs_open(GPS_NVS_NAMESPACE, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_blob(h, GPS_NVS_KEY, &obj, sizeof(obj));
        nvs_commit(h);
        nvs_close(h);
    }
}

/** Load and CRC-validate origin from NVS. Returns true on success. */
static bool nvs_origin_load(GpsNvsOrigin* out) {
    nvs_handle_t h;
    if (nvs_open(GPS_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;

    size_t    sz  = sizeof(GpsNvsOrigin);
    GpsNvsOrigin tmp;
    esp_err_t err = nvs_get_blob(h, GPS_NVS_KEY, &tmp, &sz);
    nvs_close(h);

    if (err != ESP_OK || sz != sizeof(GpsNvsOrigin)) return false;

    uint32_t expected = crc32_compute(&tmp, 2 * sizeof(double));
    if (expected != tmp.crc32) return false;

    *out = tmp;
    return true;
}


// ── Antenna-to-steering-centre offset transform ───────────────────────────────
// Converts raw antenna ENU position to steering-centre ENU given EKF heading.
// Heading convention: 0 = North (+Y), clockwise positive (radians).
// Formula from spec:
//   sc_east  = ant_east  - FORWARD * sin(θ) - RIGHT * cos(θ)
//   sc_north = ant_north - FORWARD * cos(θ) + RIGHT * sin(θ)
static void antenna_to_steering_centre(float ant_east,  float ant_north,
                                        float heading_rad,
                                        float* sc_east, float* sc_north) {
    float s = sinf(heading_rad);
    float c = cosf(heading_rad);
    const MowerConfig &mc = mower_config_get();
    *sc_east  = ant_east  - mc.antenna_fwd_m * s
                           - mc.antenna_right_m * c;
    *sc_north = ant_north - mc.antenna_fwd_m * c
                          + mc.antenna_right_m * s;
}


// ── FreeRTOS GPS poll task ────────────────────────────────────────────────────
// Polls the DFRobot RTK LoRa module at GPS_POLL_INTERVAL_MS.
// Each call to getLat()/getLon()/getQuality() issues a request to the module
// and waits for its response — this is handled entirely inside the library.
static void gps_poll_task_fn(void* /*arg*/) {
    for (;;) {
        // ── Wait for fresh data from module ───────────────────────────────
        // getDataFlush() returns true when the module has a new data frame ready.
        // Only read the getters once per flush to avoid stale/partial reads.
        if (!s_gps.getDataFlush()) {
            vTaskDelay(pdMS_TO_TICKS(GPS_POLL_INTERVAL_MS));
            continue;
        }

        // getLat()/getLon() return sLonLat_t structs.
        //   .latitudeDegree / .lonitudeDegree  = decimal degrees (what we need)
        //   .latitude       / .lonitude        = raw DDMM.MMMMM format
        // NB: 'lonitude' is a typo in the DFRobot library (missing 'g').
        int             quality   = s_gps.getQuality();
        sLonLat_t       latData   = s_gps.getLat();
        sLonLat_t       lonData   = s_gps.getLon();
        double          lat_deg   = latData.latitudeDegree;
        double          lon_deg   = lonData.lonitudeDegree;

        // Apply hemisphere sign — the DFRobot library returns positive
        // values; the NMEA direction field tells us the actual hemisphere.
        if (lonData.lonDirection == 'W') lon_deg = -lon_deg;
        if (latData.latDirection == 'S') lat_deg = -lat_deg;
        uint8_t         sat_count = s_gps.getNumSatUsed();
        float           hdop      = static_cast<float>(s_gps.getHdop());
        float           dif_age   = static_cast<float>(s_gps.getDifTime());

        // Clamp quality to valid GGA fix-type range (0–5); treat out-of-range as no-fix
        if (quality < 0 || quality > 5) quality = 0;
        GpsFix fix_type = static_cast<GpsFix>(quality);

        uint32_t now = static_cast<uint32_t>(millis());

        // ── Snapshot current origin state (set only by perimeter upload) ──
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        GpsOrigin origin = s_origin;
        xSemaphoreGive(s_mutex);

        // ── Auto-seed origin from first RTK fix if no perimeter has been stored ──
        // Allows EKF to converge during manual/test driving before any perimeter
        // is uploaded. Not persisted to NVS — a perimeter upload will override it.
        if (!origin.set && fix_type >= GPS_FIX_RTK_FIXED) {
            xSemaphoreTake(s_mutex, portMAX_DELAY);
            if (!s_origin.set) {
                s_origin.lat_deg = lat_deg;
                s_origin.lon_deg = lon_deg;
                s_origin.set     = true;
                DBG_PRINTF("[GPS] Auto-seeded ENU origin from RTK fix: %.8f, %.8f\n",
                           (double)lat_deg, (double)lon_deg);
            }
            origin = s_origin;
            xSemaphoreGive(s_mutex);
        }

        // ── Compute ENU + steering-centre offset ──────────────────────────
        float sc_east  = 0.0f;
        float sc_north = 0.0f;
        bool  valid    = false;

        if (origin.set && fix_type != GPS_FIX_NONE) {
            // Flat-earth ENU from antenna lat/lon
            float ant_east = static_cast<float>(
                (lon_deg - origin.lon_deg)
                * cos(origin.lat_deg * M_PI / 180.0)
                * 111319.5);
            float ant_north = static_cast<float>(
                (lat_deg - origin.lat_deg) * 111319.5);

            // Apply physical antenna offset to get steering-centre ENU
            // Uses current EKF heading; 0.0f is safe before EKF initialises
            float heading = ekf_get_pose().heading;
            antenna_to_steering_centre(ant_east, ant_north,
                                        heading,
                                        &sc_east, &sc_north);
            valid = true;

            // Feed antenna ENU position directly to the EKF.
            // ekf_update_gps() applies the antenna-to-steering-centre transform
            // internally using the current EKF heading, so pass ant_* not sc_*.
            bool was_seeded = ekf_is_seeded();
            ekf_update_gps(ant_east, ant_north, (int)fix_type, hdop, dif_age, heading);
            // When the EKF gets its first valid position, push a fresh map so
            // the PWA receives a non-zero pos.x/y (not the boot-time origin 0,0).
            if (!was_seeded && ekf_is_seeded()) {
                extern volatile bool g_ble_map_pending;
                g_ble_map_pending = true;
                DBG_PRINTLN("[GPS] EKF seeded — triggering BLE map push");
            }
        }

        // ── Update shared measurement (under mutex) ────────────────────────
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_measurement.lat_deg      = lat_deg;
        s_measurement.lon_deg      = lon_deg;
        s_measurement.enu_east_m   = sc_east;
        s_measurement.enu_north_m  = sc_north;
        s_measurement.fix_type     = fix_type;
        s_measurement.hdop         = hdop;
        s_measurement.sat_count    = sat_count;
        s_measurement.dif_age_s    = dif_age;
        s_measurement.timestamp_ms = now;
        s_measurement.valid        = valid;
        s_last_valid_ms            = now;
        xSemaphoreGive(s_mutex);

        // ── Log fix-type transitions ───────────────────────────────────────
        if (fix_type != s_last_logged_fix) {
            static const char * const k_fix_names[] = {
                "No fix", "GPS", "DGPS", "?", "RTK Fixed", "RTK Float"
            };
            const char *prev_name = k_fix_names[(int)s_last_logged_fix < 6 ? (int)s_last_logged_fix : 3];
            const char *new_name  = k_fix_names[(int)fix_type            < 6 ? (int)fix_type            : 3];
            char lbuf[80];
            snprintf(lbuf, sizeof(lbuf),
                "GPS: %s → %s (%u sats, hdop=%.1f)",
                prev_name, new_name, (unsigned)sat_count, (double)hdop);
            sys_log_push(lbuf);
            s_last_logged_fix = fix_type;
        }

        // No extra sleep after a successful read — the readReg calls above
        // each block up to TIME_OUT (200ms), so a full read cycle already
        // takes several hundred milliseconds. The loop-top delay handles pacing.
        vTaskDelay(pdMS_TO_TICKS(GPS_POLL_INTERVAL_MS));
    }
}


// ── Public API ────────────────────────────────────────────────────────────────

void rtk_gps_init() {
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    // Initialise DFRobot GPS library (matches example setup() pattern).
    // The library calls Serial1.begin(baud, rx, tx) internally for ESP32.
    // Retry up to 10 times then continue — the poll task will keep retrying.
    {
        int attempts = 0;
        while (!s_gps.begin()) {
            DBG_PRINTLN("[GPS] DFRobot_RTK_LoRa begin() failed — retrying...");
            delay(1000);
            if (++attempts >= 10) {
                DBG_PRINTLN("[GPS] WARNING: begin() did not confirm — continuing anyway");
                break;
            }
        }
        if (attempts < 10) {
            DBG_PRINTLN("[GPS] DFRobot RTK LoRa initialised");
        }
    }

    // Set module type to LoRa once (mirrors example — no blocking confirmation loop).
    // setModule() is a no-op if already in the correct mode; includes its own delay.
    s_gps.setModule(module_lora);
    DBG_PRINTLN("[GPS] LoRa module mode set");

    // Arm last-valid timer so timeout triggers after GPS_UPDATE_TIMEOUT_MS
    // even if the module has not responded yet
    s_last_valid_ms = static_cast<uint32_t>(millis());

    // Restore perimeter origin from NVS (saved by handle_send_perimeter).
    // Without this, ENU conversion is disabled until a perimeter is uploaded.
    {
        GpsNvsOrigin stored;
        if (nvs_origin_load(&stored)) {
            s_origin.lat_deg = stored.originLat;
            s_origin.lon_deg = stored.originLon;
            s_origin.set     = true;
            DBG_PRINTF("[GPS] Perimeter origin restored from NVS: %.7f, %.7f\n",
                          stored.originLat, stored.originLon);
        } else {
            DBG_PRINTLN("[GPS] No perimeter origin in NVS — upload perimeter to enable ENU");
        }
    }

    // Launch poll task on Core 0 (ARCHITECTURE.md Section 2)
    xTaskCreatePinnedToCore(
        gps_poll_task_fn,   // Task function
        "gps_poll_task",    // Task name
        GPS_TASK_STACK,     // Stack (bytes)
        nullptr,            // No parameters
        GPS_TASK_PRIORITY,  // Priority 6 — lowest-urgency sensor (1 Hz)
        nullptr,            // Task handle not required
        0                   // Core 0
    );
}

GpsMeasurement rtk_gps_get_measurement() {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    GpsMeasurement m = s_measurement;
    xSemaphoreGive(s_mutex);
    return m;
}

GpsOrigin rtk_gps_get_origin() {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    GpsOrigin o = s_origin;
    xSemaphoreGive(s_mutex);
    return o;
}

bool rtk_gps_has_origin() {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool has = s_origin.set;
    xSemaphoreGive(s_mutex);
    return has;
}

bool rtk_gps_is_timeout() {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    uint32_t last = s_last_valid_ms;
    xSemaphoreGive(s_mutex);
    // Returns false until the first sentence arrives (s_last_valid_ms is set
    // to millis() in init, so timeout naturally triggers after the interval)
    return (static_cast<uint32_t>(millis()) - last) > GPS_UPDATE_TIMEOUT_MS;
}

float rtk_gps_get_measurement_noise() {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    GpsFix fix = s_measurement.fix_type;
    xSemaphoreGive(s_mutex);

    switch (fix) {
        case GPS_FIX_RTK_FIXED: return 0.05f;   // ~5 cm
        case GPS_FIX_RTK_FLOAT: return 0.30f;   // ~30 cm
        default:                return 1.00f;   // autonomous / DGPS / no fix
    }
}

void rtk_gps_set_origin(double lat_deg, double lon_deg) {
    // Persist to NVS so origin survives reboot
    nvs_origin_save(lat_deg, lon_deg);

    // Adopt immediately in-memory so all ENU conversions use the new origin
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_origin.lat_deg = lat_deg;
    s_origin.lon_deg = lon_deg;
    s_origin.set     = true;
    xSemaphoreGive(s_mutex);

    DBG_PRINTF("[GPS] Perimeter origin set: %.7f, %.7f\n", lat_deg, lon_deg);

    // Re-push map so PWA immediately gets the updated origin
    extern volatile bool g_ble_map_pending;
    g_ble_map_pending = true;
}
