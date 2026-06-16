#pragma once
#include <Arduino.h>
#include <stdint.h>

// ══════════════════════════════════════════════════════════════════════════════
//  rtk_gps.h — RoboMower RTK GPS Module
//  DFRobot RTK LoRa GPS module via the DFRobot_RTK_LoRa library (request/response,
//  not an NMEA stream) over UART Serial1 (GPS_RX_PIN, 115200 baud). Polls position
//  (getLat()/getLon()) and quality (getQuality()), converts to local ENU
//  coordinates, and reports RTK fix quality.
//
//  ENU origin = first perimeter vertex (set by SEND_PERIMETER command).
//  No origin is established from GPS fixes — a perimeter must be uploaded
//  before ENU conversion is active.
//
//  All ENU positions refer to the STEERING CENTRE (antenna offset applied).
//  Thread-safe: all public functions are protected by an internal mutex.
//
//  Dependencies:
//    config.h        — GPS_RX_PIN, GPS_BAUD_RATE, GPS_UPDATE_TIMEOUT_MS
//    MowerConfig     — antenna_fwd_m, antenna_right_m (runtime antenna offsets)
//    ekf_localiser   — ekf_get_pose() (forward-declared in .cpp)
//    NVS             — ESP-IDF nvs.h, namespace "gps", key "gpsorigin"
// ══════════════════════════════════════════════════════════════════════════════


/** RTK fix quality codes (from $GNGGA fix_type field). */
enum GpsFix {
    GPS_FIX_NONE      = 0,  ///< No fix
    GPS_FIX_AUTO      = 1,  ///< Autonomous GPS (~1 m accuracy)
    GPS_FIX_DGPS      = 2,  ///< DGPS
    GPS_FIX_RTK_FIXED = 4,  ///< RTK fixed (~5 cm accuracy)
    GPS_FIX_RTK_FLOAT = 5   ///< RTK float (~30 cm accuracy)
};


/**
 * @brief A single GPS measurement.
 *
 * All ENU values are relative to the established origin and refer to the
 * STEERING CENTRE after applying the antenna physical offset.
 * Check @c valid before using any fields.
 */
struct GpsMeasurement {
    double   lat_deg;       ///< Latitude  (decimal degrees, WGS-84)
    double   lon_deg;       ///< Longitude (decimal degrees, WGS-84)
    float    enu_east_m;    ///< ENU east  of origin at steering centre [m]
    float    enu_north_m;   ///< ENU north of origin at steering centre [m]
    GpsFix   fix_type;      ///< Fix quality code from $GNGGA field 6
    float    hdop;          ///< Horizontal dilution of precision
    uint8_t  sat_count;     ///< Number of satellites used
    float    dif_age_s;     ///< Seconds since last RTK/DGPS correction received (getDifTime())
    uint32_t timestamp_ms;  ///< millis() at which the NMEA sentence was parsed
    bool     valid;         ///< True if origin is set and fix_type != GPS_FIX_NONE
};


/**
 * @brief ENU origin (first perimeter vertex, set by SEND_PERIMETER).
 *
 * All ENU coordinates are relative to this point.
 * @c set is false until a perimeter has been uploaded.
 */
struct GpsOrigin {
    double lat_deg;  ///< Origin latitude  (decimal degrees)
    double lon_deg;  ///< Origin longitude (decimal degrees)
    bool   set;      ///< True once a perimeter has been uploaded
};


/**
 * @brief Initialise the GPS module.
 *
 * Configures Serial1 at GPS_BAUD_RATE on GPS_RX_PIN (receive-only).
 * Restores the perimeter origin from NVS if present (saved by a prior
 * SEND_PERIMETER). Starts the FreeRTOS parse task on Core 0.
 *
 * Must be called after nvs_flash_init() in the main sketch setup().
 */
void rtk_gps_init();


/**
 * @brief Get the latest GPS measurement. Thread-safe.
 *
 * Returns a snapshot of the most recently parsed $GNGGA sentence with ENU
 * position transformed to the steering centre. Returns @c valid=false if no
 * fix has been received or the ENU origin has not yet been established.
 */
GpsMeasurement rtk_gps_get_measurement();


/**
 * @brief Get the current ENU origin (perimeter first vertex). Thread-safe.
 *
 * @return GpsOrigin with @c set=false until a perimeter is uploaded.
 */
GpsOrigin rtk_gps_get_origin();


/**
 * @brief Returns true if a perimeter origin has been set. Thread-safe.
 *
 * Equivalent to @c rtk_gps_get_origin().set but avoids a full struct copy.
 */
bool rtk_gps_has_origin();


/**
 * @brief Returns true if no valid NMEA sentence has been received within
 *        GPS_UPDATE_TIMEOUT_MS milliseconds. Thread-safe.
 *
 * Returns false if no sentence has ever been received (startup grace period).
 */
bool rtk_gps_is_timeout();


/**
 * @brief EKF measurement noise variance for the current fix type.
 *
 * Returns the diagonal variance (m²) to use for the EKF R matrix:
 *   - RTK fixed  → 0.05 m²
 *   - RTK float  → 0.30 m²
 *   - Autonomous / DGPS / no fix → 1.00 m²
 *
 * Thread-safe.
 */
float rtk_gps_get_measurement_noise();


/**
 * @brief Set the ENU origin to a given lat/lon (perimeter first vertex). Thread-safe.
 *
 * Adopts @p lat_deg / @p lon_deg as the ENU reference frame immediately and
 * saves it to NVS ("gps"/"gpsorigin") so it persists across reboots.
 * Called by handle_send_perimeter() — this is the ONLY way the origin is set.
 *
 * @param lat_deg  Latitude  of perimeter first vertex (decimal degrees, WGS-84).
 * @param lon_deg  Longitude of perimeter first vertex (decimal degrees, WGS-84).
 */
void rtk_gps_set_origin(double lat_deg, double lon_deg);


/**
 * @brief Convert absolute WGS-84 lat/lon to local ENU metres. Thread-safe.
 *
 * Uses the current origin and the SAME WGS-84 local-tangent-plane projection as
 * live GPS fixes, so perimeter points stored as lat/lon re-derive into the
 * identical ENU frame the mower navigates in. Returns false (outputs untouched)
 * if no origin is set.
 */
bool rtk_gps_latlon_to_enu(double lat_deg, double lon_deg, float *east_m, float *north_m);

/**
 * @brief Convert local ENU metres to absolute WGS-84 lat/lon. Thread-safe.
 *
 * Inverse of rtk_gps_latlon_to_enu(); used to migrate a legacy ENU perimeter to
 * lat/lon storage. Returns false (outputs untouched) if no origin is set.
 */
bool rtk_gps_enu_to_latlon(float east_m, float north_m, double *lat_deg, double *lon_deg);
