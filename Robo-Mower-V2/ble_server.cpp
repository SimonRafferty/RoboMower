// ══════════════════════════════════════════════════════════════════════════════
//  ble_server.cpp — RoboMower BLE GATT server implementation
//
//  Four GATT characteristics:
//    TELEM   — 2 Hz / 1 Hz JSON telemetry notify
//    MAP     — fragmented map JSON (perimeter, nav boundary, mowed cells, etc.)
//    CMD     — write-with-response; commands forwarded to state machine queue
//    STATUS  — status JSON on request
//
//  Large commands from the phone use a BINARY fragment protocol:
//    [0x01][frag_index][frag_total][data...]  (raw payload, no JSON escaping)
//  Fragments are reassembled before dispatch.
//
//  References:
//    ble_server.h — public API
//    state_machine.h — enqueue + getter functions
//    obstacle_map.h  — grid_enumerate_mowed + dimension getters
// ══════════════════════════════════════════════════════════════════════════════

#include "ble_server.h"
#include "state_machine.h"
#include "ekf_localiser.h"
#include "rtk_gps.h"
#include "vesc_can.h"
#include "battery_monitor.h"
#include "cutting_monitor.h"
#include "obstacle_map.h"
#include "perimeter.h"
#include "geometry.h"
#include "nvs_storage.h"
#include "coverage_planner.h"
#include "collision_detect.h"
#include "config.h"
#include "mower_config.h"
#include "sys_log.h"
#include "imu.h"
#include "crsf_input.h"
#include "node_follower.h"
#include "heading_controller.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <vector>
#include <utility>

// ── Module state ─────────────────────────────────────────────────────────────

static BLEServer         *s_pServer  = nullptr;
static BLECharacteristic *s_pTelem   = nullptr;
static BLECharacteristic *s_pMap     = nullptr;
static BLECharacteristic *s_pCmd     = nullptr;
static BLECharacteristic *s_pStatus  = nullptr;

static volatile bool s_connected     = false;
static uint16_t s_negotiated_mtu     = 23;  // BLE minimum; updated after connection
static bool     s_mtu_dirty          = false;

static uint32_t s_last_telem_ms      = 0;

// Fragment reassembly buffer for incoming fragmented commands
#define CMD_FRAG_MAX_N    8
#define CMD_FRAG_MAX_LEN  512
static char     s_cmd_frag_buf[CMD_FRAG_MAX_N * CMD_FRAG_MAX_LEN + 1];
static char     s_cmd_frags[CMD_FRAG_MAX_N][CMD_FRAG_MAX_LEN];
static int      s_cmd_frag_n   = 0;
static int      s_cmd_frag_got = 0;


// ── Fragment reassembly ───────────────────────────────────────────────────────

/** Parse and accumulate a binary fragment; dispatch when all fragments arrived.
 *  Binary format: [0x01][f uint8][n uint8][raw JSON bytes...]
 *  No JSON escaping needed — raw payload is passed through as-is. */
static void handle_cmd_fragment(const uint8_t *data, size_t len) {
    // data[0] == 0x01 already confirmed by caller
    if (len < 4) return;                 // need at least header + 1 byte data
    int f = (int)data[1];
    int n = (int)data[2];
    if (f < 0 || f >= CMD_FRAG_MAX_N || n <= 0 || n > CMD_FRAG_MAX_N) return;

    size_t chunk_len = len - 3;
    if (chunk_len >= (size_t)CMD_FRAG_MAX_LEN) chunk_len = CMD_FRAG_MAX_LEN - 1;
    memcpy(s_cmd_frags[f], data + 3, chunk_len);
    s_cmd_frags[f][chunk_len] = '\0';

    if (s_cmd_frag_n == 0) s_cmd_frag_got = 0;
    s_cmd_frag_n = n;
    s_cmd_frag_got++;

    if (s_cmd_frag_got == n) {
        size_t pos = 0;
        for (int i = 0; i < n && pos < sizeof(s_cmd_frag_buf) - 1; i++) {
            size_t sl = strlen(s_cmd_frags[i]);
            if (pos + sl >= sizeof(s_cmd_frag_buf) - 1) sl = sizeof(s_cmd_frag_buf) - 1 - pos;
            memcpy(s_cmd_frag_buf + pos, s_cmd_frags[i], sl);
            pos += sl;
        }
        s_cmd_frag_buf[pos] = '\0';
        state_machine_enqueue_ble_cmd(s_cmd_frag_buf, pos);
        s_cmd_frag_got = 0;
        s_cmd_frag_n   = 0;
    }
}


// ── BLE callbacks ─────────────────────────────────────────────────────────────

class ServerCallbacks : public BLEServerCallbacks {
    // Override the parameter-less onConnect/onDisconnect overloads — these are the
    // "Common public declarations" present and invoked on EVERY core version and BOTH
    // BLE backends (Bluedroid calls them, and so does NimBLE, which the ESP32-S3 build
    // selects from core 3.1+). The old Bluedroid-only onConnect(…, esp_ble_gatts_cb_param_t*)
    // overload doesn't exist under NimBLE, so using it broke the build on newer cores.
    // We only used `param` to set the (never-read, now removed) s_conn_id, so nothing is lost.
    void onConnect(BLEServer *pServer) override {
        s_connected  = true;
        s_mtu_dirty  = true;   // read actual negotiated MTU next tick
        // NOTE: do NOT call BLEDevice::setMTU() here — that would overwrite the
        // value the BLE stack sets via ESP_GATTS_MTU_EVT with our local preference,
        // making getMTU() return 512 regardless of what was actually negotiated.
        // setMTU(512) is already called once in ble_server_init() as our preference.
        // Push map + status to the new client automatically (deferred to Core 1 tick)
        g_ble_map_pending    = true;
        g_ble_status_pending = true;
        DBG_PRINTLN("[BLE] Client connected — map + status queued");
        {   // Timestamped marker for correlating with RC-link events in the log
            char line[48];
            snprintf(line, sizeof(line), "BLE connected t=%lu", (unsigned long)millis());
            sys_log_push(line);
        }
    }
    void onDisconnect(BLEServer *pServer) override {
        s_connected = false;
        state_machine_ble_disconnected();  // clear drive/arm/pause/learn
        BLEDevice::startAdvertising();
        DBG_PRINTLN("[BLE] Client disconnected — restarting advertising");
        {
            char line[48];
            snprintf(line, sizeof(line), "BLE disconnected t=%lu", (unsigned long)millis());
            sys_log_push(line);
        }
    }
};

class CmdCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) override {
        String val = pChar->getValue();
        if (val.length() == 0) return;
        const uint8_t *data = (const uint8_t *)val.c_str();
        size_t len = val.length();
        // Binary fragment: first byte is 0x01 (SOH).
        // Regular JSON always starts with '{' (0x7B) — no ambiguity.
        if (len >= 4 && data[0] == 0x01) {
            handle_cmd_fragment(data, len);
        } else {
            state_machine_enqueue_ble_cmd((const char *)data, len);
        }
    }
};


// ── Fragmented notify helper ──────────────────────────────────────────────────

/** Send @p payload on @p pChar, fragmenting if it exceeds available MTU data bytes.
 *
 *  Fragment format (binary, no JSON escaping needed):
 *    [0x01][f uint8][n uint8][raw payload bytes...]
 *  Byte 0 = 0x01 (SOH) distinguishes fragments from plain JSON (which starts with '{' = 0x7B).
 *  Plain notifications (fits in one packet) are sent as raw JSON with no header.
 */
static void ble_notify_fragmented(BLECharacteristic *pChar, const String &payload) {
    if (!s_connected) return;

    if (s_mtu_dirty) {
        uint16_t m = BLEDevice::getMTU();
        // Clamp to sane range.  getMTU() returns the value set via setMTU() which
        // is updated by ESP_GATTS_MTU_EVT with the actual negotiated value.
        // If it still shows 512 (our preference, not yet exchanged) be conservative.
        if (m < 23)  m = 23;
        if (m > 512) m = 512;
        s_negotiated_mtu = m;
        s_mtu_dirty = false;
        DBG_PRINTF("[BLE] MTU=%u  mtu_data=%u\n", m, (unsigned)(m - 3));
    }

    // ATT header is 3 bytes; usable payload per packet
    size_t mtu_data = (size_t)(s_negotiated_mtu - 3);
    if (mtu_data < 20) mtu_data = 20;

    const uint8_t *data  = (const uint8_t *)payload.c_str();
    size_t         total = payload.length();

    if (total <= mtu_data) {
        // Fits in one packet — send as plain JSON (no header byte)
        pChar->setValue(const_cast<uint8_t *>(data), total);
        pChar->notify();
        return;
    }

    // Fragmented: 3-byte binary header [0x01][f][n] + chunk data
    size_t chunk = (mtu_data > 3) ? mtu_data - 3 : 4;
    int    n     = (int)((total + chunk - 1) / chunk);
    if (n > 255) n = 255;

    for (int f = 0; f < n; f++) {
        size_t offset = (size_t)f * chunk;
        size_t sz     = chunk;
        if (offset + sz > total) sz = total - offset;

        // Build fragment in a single buffer: [0x01][f][n][data...]
        static uint8_t frag[520];
        frag[0] = 0x01;
        frag[1] = (uint8_t)f;
        frag[2] = (uint8_t)n;
        if (sz > sizeof(frag) - 3) sz = sizeof(frag) - 3;
        memcpy(frag + 3, data + offset, sz);

        pChar->setValue(frag, 3 + sz);
        pChar->notify();
        // Give BLE stack time to drain the notification buffer
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


// ── JSON builders ─────────────────────────────────────────────────────────────

/** State name strings — must match RobotState enum order. */
static const char *k_state_names[] = {
    "INIT","IDLE","MANUAL","LEARN_PERIM","AUTO","RETRACE",
    "BOG","OBS_AVOID","NUDGE","PAUSED","MOTORS_OFFLINE"
};

static String build_telemetry_json() {
    Pose2D         pose  = ekf_get_pose();
    GpsMeasurement gps   = rtk_gps_get_measurement();
    VescStatus     blade = vesc_get_status(VESC_ID_BLADE);
    float          batt  = battery_get_voltage();
    BatteryState   bs    = battery_get_state();
    CuttingStatus  cs    = cutting_monitor_get_status();
    float          speed = ekf_get_speed();
    float          unc   = ekf_get_position_uncertainty();

    const char *bat_names[] = {"OK","WARNING","LOW"};
    const char *cut_names[] = {"NORMAL","OVERLOADED","STALLED","OBS_SUSP","BLADE_FAULT"};

    RobotState st     = state_machine_get_state();
    const char *sname = ((int)st >= 0 && (int)st < 11) ? k_state_names[(int)st] : "?";

    float blade_rpm = fabsf(blade.erpm) / (float)mower_config_get().blade_motor_pole_pairs;

    // Module health (1 = ok, 0 = fault/offline/timeout)
    uint32_t now_ms = millis();
    int mod_imu  = (imu_is_present() && !imu_is_fault())                        ? 1 : 0;
    uint8_t imu_cal  = imu_get_calib_status();
    int     cal_sys  = (imu_cal >> 6) & 0x03;
    int     cal_mag  =  imu_cal       & 0x03;
    bool    hdg_conf = imu_heading_is_confident();
    int mod_gps  = rtk_gps_is_timeout()                                         ? 0 : 1;
    int mod_rc   = crsf_is_failsafe()                                           ? 0 : 1;
    int mod_can  = vesc_can_bus_ok()                                            ? 1 : 0;
    int mod_vl   = (now_ms - vesc_get_status(VESC_ID_LEFT ).last_update_ms > VESC_STATUS_TIMEOUT_MS) ? 0 : 1;
    int mod_vr   = (now_ms - vesc_get_status(VESC_ID_RIGHT).last_update_ms > VESC_STATUS_TIMEOUT_MS) ? 0 : 1;
    int mod_vb   = (now_ms - vesc_get_status(VESC_ID_BLADE).last_update_ms > VESC_STATUS_TIMEOUT_MS) ? 0 : 1;
    // On first boot last_update_ms == 0 which would show 'now_ms > timeout'; suppress until
    // the state machine has been running long enough that a real absence is meaningful.
    if (now_ms < 5000) { mod_vl = mod_vr = mod_vb = 1; }

    // EKF steering-centre position as absolute lat/lon — the firmware owns the
    // projection (WGS-84 local tangent plane), so the PWA plots the live arrow
    // directly from posLat/posLon with no origin re-projection (fixes the
    // map/mower offset + the arrow only moving on a manual map fetch).
    double pos_lat = 0.0, pos_lon = 0.0;
    rtk_gps_enu_to_latlon(pose.x, pose.y, &pos_lat, &pos_lon);

    // RAW GPS/RTK steering-centre as lat/lon (same frame as the EKF arrow), so the
    // PWA can draw a second arrow and the operator can see how far the raw fix
    // differs from the fused EKF position — even when the fix reports Fixed. Uses
    // the GPS steering-centre ENU (antenna offset already applied), not the antenna
    // lat/lon, so any divergence is the EKF smoothing/lag, not the antenna offset.
    double gps_lat = 0.0, gps_lon = 0.0;
    if (gps.valid) rtk_gps_enu_to_latlon(gps.enu_east_m, gps.enu_north_m,
                                         &gps_lat, &gps_lon);
    // GPS-based heading = last straight-run travel chord (0=N, CW+, rad). Holds the
    // last value between qualifying straight segments (255/stale is fine — it's a
    // diagnostic). 0 until the first event.
    float gps_hdg = 0.0f;
    { float gth = 0.0f; if (ekf_get_gps_heading_event(nullptr, &gth, nullptr, nullptr)) gps_hdg = gth; }

    char buf[928];   // headroom for IMU calibration + posLat/posLon + raw-GPS fields
    int len = snprintf(buf, sizeof(buf),
        "{\"t\":%lu,\"state\":\"%s\","
        "\"x\":%.3f,\"y\":%.3f,\"hdg\":%.3f,"
        "\"posLat\":%.7f,\"posLon\":%.7f,"
        "\"gpsLat\":%.7f,\"gpsLon\":%.7f,\"gpsHdg\":%.3f,"
        "\"lat\":%.7f,\"lon\":%.7f,"
        "\"fix\":%d,\"sat\":%d,\"hdop\":%.2f,\"difAge\":%.1f,\"vel\":%.3f,"
        "\"hprog\":%.3f,\"sprog\":%.3f,"
        "\"cutH\":%.0f,"
        "\"obs\":%d,\"unc\":%.4f,"
        "\"bladeRPM\":%.0f,\"bladeA\":%.2f,\"bladeLoad\":%.3f,"
        "\"cutStatus\":\"%s\","
        "\"battV\":%.2f,\"battState\":\"%s\","
        "\"collBase\":%.3f,"
        "\"tilt\":%.1f,"
        "\"armed\":%s,\"modeSw\":%d,\"pauseSw\":%s,\"learnSw\":%s,"
        "\"blePause\":%s,\"bleArm\":%s,"
        "\"bladeCmd\":%s,\"bladeLock\":%s,"
        "\"ch6Us\":%d,"
        "\"imuCalSys\":%d,\"imuCalMag\":%d,\"headingConfident\":%s,"
        "\"mod\":{\"imu\":%d,\"gps\":%d,\"rc\":%d,\"can\":%d,\"vL\":%d,\"vR\":%d,\"vB\":%d}}",
        (unsigned long)millis(), sname,
        pose.x, pose.y, pose.heading,
        pos_lat, pos_lon,
        gps_lat, gps_lon, gps_hdg,
        gps.lat_deg, gps.lon_deg,
        (int)gps.fix_type, (int)gps.sat_count, gps.hdop, gps.dif_age_s, speed,
        coverage_planner_get_headland_progress(),
        coverage_planner_get_strip_progress(),
        state_machine_get_cut_height_mm(),
        obstacle_get_count(), unc,
        blade_rpm, blade.current_A, cutting_monitor_get_rpm_load_fraction(),  // RPM-based load (Feature 2)
        cut_names[(int)cs],
        batt, bat_names[(int)bs],
        collisionGetBaseline(),
        imu_get_tilt_rad() * (180.0f / M_PI),
        state_machine_is_armed() ? "true" : "false",
        (int)state_machine_get_mode_sw(),
        state_machine_is_paused_sw() ? "true" : "false",
        state_machine_is_learn_sw()  ? "true" : "false",
        state_machine_is_ble_paused()         ? "true" : "false",
        state_machine_is_ble_armed()          ? "true" : "false",
        state_machine_is_blade_commanded()    ? "true" : "false",
        state_machine_is_blade_lockout()      ? "true" : "false",
        (int)state_machine_get_ch6_us(),
        cal_sys, cal_mag, hdg_conf ? "true" : "false",
        mod_imu, mod_gps, mod_rc, mod_can, mod_vl, mod_vr, mod_vb);

    if (len > 0 && len < (int)sizeof(buf)) return String(buf);
    return String("{}");
}

/** Append an ENU polygon as a JSON array of absolute WGS-84 lat/lon: [[lat,lon],...]
 *  The mower is the single owner of the projection (WGS-84 local tangent plane),
 *  so the PWA consumes lat/lon directly with no scheme translation. */
static void append_polygon_ll(String &out, const Polygon &poly) {
    out += '[';
    for (size_t i = 0; i < poly.pts.size(); i++) {
        if (i) out += ',';
        double la = 0.0, lo = 0.0;
        rtk_gps_enu_to_latlon(poly.pts[i].x, poly.pts[i].y, &la, &lo);
        char pt[40];
        snprintf(pt, sizeof(pt), "[%.7f,%.7f]", la, lo);
        out += pt;
    }
    out += ']';
}

/** Append the canonical perimeter as [[lat,lon,acc],...] — exact stored WGS-84
 *  coordinates with per-point GPS confidence (no ENU round-trip), so the PWA map
 *  carries the same per-corner confidence the mower holds. Falls back to the ENU
 *  perimeter (converted to lat/lon, worst-case accuracy) if no canonical store. */
static void append_perimeter_ll(String &out, const Polygon &fallback_enu) {
    int n = perimeter_canonical_count();
    if (n >= 3) {
        out += '[';
        for (int i = 0; i < n; i++) {
            if (i) out += ',';
            double la = 0.0, lo = 0.0; float ac = 0.05f;
            perimeter_canonical_point(i, &la, &lo, &ac);
            char pt[56];
            snprintf(pt, sizeof(pt), "[%.7f,%.7f,%.2f]", la, lo, (double)ac);
            out += pt;
        }
        out += ']';
        return;
    }
    // Fallback: convert the ENU perimeter, tag with the worst-case accuracy.
    float acc = perimeter_get_accuracy_m();
    out += '[';
    for (size_t i = 0; i < fallback_enu.pts.size(); i++) {
        if (i) out += ',';
        double fla = 0.0, flo = 0.0;
        rtk_gps_enu_to_latlon(fallback_enu.pts[i].x, fallback_enu.pts[i].y, &fla, &flo);
        char pt[56];
        snprintf(pt, sizeof(pt), "[%.7f,%.7f,%.2f]", fla, flo, (double)acc);
        out += pt;
    }
    out += ']';
}

static String build_map_json() {
    GpsOrigin origin = rtk_gps_get_origin();
    Polygon   perim  = nvs_load_perimeter();
    Polygon   nav    = nvs_load_nav_boundary();
    Polygon   work   = nvs_load_working_area();

    // Run the planner so the PWA gets the current mowing path.
    // Uses the mower's current EKF position for the transit path.
    if (perim.pts.size() >= 3 && nav.pts.size() >= 3 && work.pts.size() >= 3) {
        coverage_planner_plan(perim, nav, work);
    }

    String out;
    out.reserve(2048);
    out += "{\"type\":\"map\",\"v\":2,";

    // GPS origin
    char orig_buf[80];
    snprintf(orig_buf, sizeof(orig_buf),
        "\"origin\":{\"lat\":%.7f,\"lon\":%.7f},",
        origin.lat_deg, origin.lon_deg);
    out += orig_buf;

    // Grid metadata
    {
        char grid_buf[128];
        snprintf(grid_buf, sizeof(grid_buf),
            "\"grid\":{\"cols\":%d,\"rows\":%d,\"cell_m\":%.4f,"
            "\"ox\":%.3f,\"oy\":%.3f},",
            grid_get_cols(), grid_get_rows(), grid_get_cell_size_m(),
            grid_get_origin_x(), grid_get_origin_y());
        out += grid_buf;
    }

    // Polygons — emitted as absolute WGS-84 lat/lon (single coordinate scheme).
    // Perimeter carries per-point accuracy [[lat,lon,acc],...]; the rest [[lat,lon],...].
    out += "\"perimeter\":";    append_perimeter_ll(out, perim); out += ',';
    out += "\"nav_boundary\":"; append_polygon_ll(out, nav);     out += ',';
    out += "\"working_area\":"; append_polygon_ll(out, work);    out += ',';

    // In-progress recording points (visible on map during LEARN)
    Polygon rec = perimeter_get_recording_points();
    out += "\"recording\":"; append_polygon_ll(out, rec); out += ',';

    // Mowed cells
    std::vector<std::pair<uint8_t,uint8_t>> mowed;
    grid_enumerate_mowed(mowed);
    out += "\"mowed_cells\":[";
    for (size_t i = 0; i < mowed.size(); i++) {
        if (i) out += ',';
        char cell[16];
        snprintf(cell, sizeof(cell), "[%u,%u]", mowed[i].first, mowed[i].second);
        out += cell;
    }
    out += "],";

    // Obstacles (positions from obstacle records)
    out += "\"obstacles\":[";
    int n_obs = obstacle_get_count();
    // No direct obstacle position iterator in API; use count only for now
    // (positions are stored in the ring buffer but not publicly exposed per-record)
    (void)n_obs;
    out += "],";

    // Current mower position — emitted as absolute lat/lon. Prefer GPS (same frame
    // as the perimeter); fall back to the EKF pose. Heading is the real-time EKF
    // heading (BNO fusion).
    {
        float ex = 0.0f, ny = 0.0f, hdg = 0.0f;
        bool have = false;
        GpsMeasurement gps_pos = rtk_gps_get_measurement();
        if (gps_pos.valid) {
            ex = gps_pos.enu_east_m; ny = gps_pos.enu_north_m;
            hdg = ekf_is_seeded() ? ekf_get_pose().heading : 0.0f;
            have = true;
        } else if (ekf_is_seeded()) {
            Pose2D p = ekf_get_pose();
            ex = p.x; ny = p.y; hdg = p.heading;
            have = true;
        }
        if (have) {
            double la = 0.0, lo = 0.0;
            if (rtk_gps_enu_to_latlon(ex, ny, &la, &lo)) {
                char pos_buf[88];
                snprintf(pos_buf, sizeof(pos_buf),
                    "\"pos\":{\"lat\":%.7f,\"lon\":%.7f,\"hdg\":%.3f},",
                    la, lo, hdg);
                out += pos_buf;
            }
        }
    }

    // Planned mowing path — segmented: array of segments, each segment is a
    // continuous run of mowing=true waypoints.  Breaks at transit gaps so the
    // PWA renders separate polylines (no false cross-perimeter connections).
    // Only include when perimeter is valid — otherwise stale waypoints linger.
    const std::vector<Waypoint> &wps = coverage_planner_get_waypoints();
    if (!wps.empty() && perim.pts.size() >= 3) {
        out += "\"mow_path\":[";
        bool first_seg = true;
        bool in_seg = false;
        for (const Waypoint &wp : wps) {
            if (wp.mowing) {
                if (!in_seg) {
                    if (!first_seg) out += ',';
                    out += '[';
                    first_seg = false;
                    in_seg = true;
                } else {
                    out += ',';
                }
                double la = 0.0, lo = 0.0;
                rtk_gps_enu_to_latlon(wp.x, wp.y, &la, &lo);
                char pt[40];
                snprintf(pt, sizeof(pt), "[%.7f,%.7f]", la, lo);
                out += pt;
            } else {
                if (in_seg) {
                    out += ']';
                    in_seg = false;
                }
            }
        }
        if (in_seg) out += ']';
        out += "],";
    }

    // Transit path: mower → nearest perimeter vertex → follow perimeter to vertex 0
    if (ekf_is_seeded() && perim.pts.size() >= 3) {
        Pose2D pose = ekf_get_pose();
        int n = (int)perim.pts.size();

        // Find nearest perimeter vertex to mower
        int nearest = 0;
        float best_d2 = 1e18f;
        for (int i = 0; i < n; i++) {
            float dx = perim.pts[i].x - pose.x;
            float dy = perim.pts[i].y - pose.y;
            float d2 = dx * dx + dy * dy;
            if (d2 < best_d2) { best_d2 = d2; nearest = i; }
        }

        // Walk perimeter from nearest to vertex 0, choosing shorter direction
        // CW walk: nearest, nearest-1, nearest-2, ..., 0
        // CCW walk: nearest, nearest+1, nearest+2, ..., n-1, 0
        int cw_steps  = nearest;           // nearest → 0 going backwards
        int ccw_steps = n - nearest;       // nearest → 0 going forwards (wrapping)

        out += "\"transit_path\":[";
        char pt[44];
        double la = 0.0, lo = 0.0;
        // Start with mower position (lat/lon)
        rtk_gps_enu_to_latlon(pose.x, pose.y, &la, &lo);
        snprintf(pt, sizeof(pt), "[%.7f,%.7f]", la, lo);
        out += pt;

        if (cw_steps <= ccw_steps) {
            // Walk backwards: nearest, nearest-1, ..., 0
            for (int i = nearest; i >= 0; i--) {
                rtk_gps_enu_to_latlon(perim.pts[i].x, perim.pts[i].y, &la, &lo);
                snprintf(pt, sizeof(pt), ",[%.7f,%.7f]", la, lo);
                out += pt;
            }
        } else {
            // Walk forwards: nearest, nearest+1, ..., n-1, 0
            for (int i = nearest; i < n; i++) {
                rtk_gps_enu_to_latlon(perim.pts[i].x, perim.pts[i].y, &la, &lo);
                snprintf(pt, sizeof(pt), ",[%.7f,%.7f]", la, lo);
                out += pt;
            }
            // Close to vertex 0
            rtk_gps_enu_to_latlon(perim.pts[0].x, perim.pts[0].y, &la, &lo);
            snprintf(pt, sizeof(pt), ",[%.7f,%.7f]", la, lo);
            out += pt;
        }
        out += "],";
    }

    // Strip trailing comma before closing brace
    if (out.length() > 0 && out[out.length()-1] == ',')
        out.remove(out.length()-1);

    out += '}';
    return out;
}

static String build_status_json() {
    RobotState st     = state_machine_get_state();
    const char *sname = ((int)st >= 0 && (int)st < 11) ? k_state_names[(int)st] : "?";

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"status\","
        "\"state\":\"%s\","
        "\"armed\":%s,"
        "\"hasPerim\":%s,"
        "\"perimArea\":%.1f,"
        "\"cutH\":%.0f,"
        "\"battV\":%.2f,"
        "\"obs\":%d,",
        sname,
        state_machine_is_armed()     ? "true" : "false",
        state_machine_has_perimeter()? "true" : "false",
        state_machine_get_perimeter_area_m2(),
        state_machine_get_cut_height_mm(),
        battery_get_voltage(),
        obstacle_get_count());

    String out = String(buf);

    // Append mower config object
    const MowerConfig &mc = mower_config_get();

    // BNO055 calibration profile (22 offset bytes as 44 hex chars + quality 0..9),
    // carried in the config so the PWA can back it up to the settings file and
    // restore it after an NVS wipe / new board. Empty when no profile is saved.
    char calfld[80]; calfld[0] = '\0';
    {
        uint8_t cb[22], cq = 0;
        if (imu_get_saved_cal(cb, &cq)) {
            char hex[45];
            for (int i = 0; i < 22; i++) snprintf(hex + i * 2, 3, "%02x", cb[i]);
            snprintf(calfld, sizeof(calfld),
                     ",\"bnocal\":\"%s\",\"bnocalq\":%u", hex, (unsigned)cq);
        }
    }

    static char cbuf[1700];  // static: BSS segment, not stack — build_status_json() is called from one BLE task only (incl. bnocal hex + resp_* bools)
    snprintf(cbuf, sizeof(cbuf),
        "\"cfg\":{"
        "\"footprint_width_m\":%.3f,\"footprint_length_m\":%.3f,"
        "\"track_width_m\":%.3f,"
        "\"wheel_radius_m\":%.4f,\"motor_pole_pairs\":%d,\"gear_ratio\":%.2f,"
        "\"antenna_fwd_m\":%.3f,\"antenna_right_m\":%.3f,"
        "\"steer_to_cut_m\":%.3f,\"cut_disc_radius_m\":%.3f,"
        "\"cut_width_m\":%.3f,\"strip_overlap_m\":%.3f,"
        "\"cut_height_min_mm\":%d,\"cut_height_max_mm\":%d,"
        "\"blade_motor_pole_pairs\":%d,\"blade_target_rpm\":%d,"
        "\"pp_lookahead_base_m\":%.3f,\"pp_lookahead_k\":%.3f,"
        "\"max_mowing_speed_ms\":%.3f,\"headland_speed_ms\":%.3f,"
        "\"transit_speed_ms\":%.3f,\"min_creep_speed_ms\":%.3f,"
        "\"waypoint_arrive_dist_m\":%.3f,\"max_wheel_speed_ms\":%.3f,"
        "\"max_current_a\":%.1f,\"current_ramp_a_per_s\":%.1f,"
        "\"uncertainty_margin_m\":%.2f,\"tilt_limit_normal_deg\":%.1f,"
        "\"tilt_limit_careful_deg\":%.1f,\"collision_mult_careful\":%.1f,"
        "\"heading_kp\":%.2f,\"heading_kd\":%.2f,\"manual_max_yaw_rate\":%.2f,"
        "\"wheel_pi_kp\":%.2f,\"wheel_pi_ki\":%.2f,"
        "\"manual_max_duty\":%.2f,\"manual_max_speed_ms\":%.3f,"
        "\"min_turn_radius_m\":%.3f,\"min_move_duty\":%.3f,\"turn_margin_m\":%.3f,"
        "\"resp_collision_en\":%s,\"resp_stall_en\":%s,"
        "\"resp_tilt_en\":%s,\"resp_slip_en\":%s,"
        "\"resp_blade_slow_en\":%s,\"resp_blade_reverse_en\":%s,"
        "\"cut_height_timeout_s\":%.1f%s},",
        mc.footprint_width_m, mc.footprint_length_m,
        mc.track_width_m,
        mc.wheel_radius_m, (int)mc.motor_pole_pairs, mc.gear_ratio,
        mc.antenna_fwd_m, mc.antenna_right_m,
        mc.steer_to_cut_m, mc.cut_disc_radius_m,
        mc.cut_width_m, mc.strip_overlap_m,
        (int)mc.cut_height_min_mm, (int)mc.cut_height_max_mm,
        (int)mc.blade_motor_pole_pairs, (int)mc.blade_target_rpm,
        mc.pp_lookahead_base_m, mc.pp_lookahead_k,
        mc.max_mowing_speed_ms, mc.headland_speed_ms,
        mc.transit_speed_ms, mc.min_creep_speed_ms,
        mc.waypoint_arrive_dist_m, mc.max_wheel_speed_ms,
        mc.max_current_a, mc.current_ramp_a_per_s,
        mc.uncertainty_margin_m, mc.tilt_limit_normal_deg,
        mc.tilt_limit_careful_deg, mc.collision_mult_careful,
        mc.heading_kp, mc.heading_kd, mc.manual_max_yaw_rate,
        mc.wheel_pi_kp, mc.wheel_pi_ki,
        mc.manual_max_duty, mc.manual_max_speed_ms,
        mc.min_turn_radius_m, mc.min_move_duty, mc.turn_margin_m,
        mc.resp_collision_en   ? "true" : "false",
        mc.resp_stall_en       ? "true" : "false",
        mc.resp_tilt_en        ? "true" : "false",
        mc.resp_slip_en        ? "true" : "false",
        mc.resp_blade_slow_en  ? "true" : "false",
        mc.resp_blade_reverse_en ? "true" : "false",
        mc.cut_height_timeout_s,
        calfld);
    out += cbuf;

    // Append system log array (last N messages, newest last)
    out += "\"log\":[";
    int n = sys_log_count();
    for (int i = 0; i < n; i++) {
        if (i) out += ',';
        out += '\"';
        // Escape any quotes in the message
        const char *msg = sys_log_get(i);
        for (const char *c = msg; *c; c++) {
            if (*c == '"')  out += "\\\"";
            else if (*c == '\\') out += "\\\\";
            else out += *c;
        }
        out += '\"';
    }
    out += "]}";
    return out;
}


// ── Public API ────────────────────────────────────────────────────────────────

void ble_server_init() {
    BLEDevice::init("RoboMower");
    BLEDevice::setMTU(512);

    s_pServer = BLEDevice::createServer();
    s_pServer->setCallbacks(new ServerCallbacks());

    BLEService *pSvc = s_pServer->createService(BLE_SERVICE_UUID);

    // Telemetry: NOTIFY only
    s_pTelem = pSvc->createCharacteristic(BLE_CHAR_TELEM_UUID,
        BLECharacteristic::PROPERTY_NOTIFY);
    s_pTelem->addDescriptor(new BLE2902());

    // Map: NOTIFY + READ
    s_pMap = pSvc->createCharacteristic(BLE_CHAR_MAP_UUID,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    s_pMap->addDescriptor(new BLE2902());

    // Command: WRITE with response (ACKs go via Status characteristic)
    s_pCmd = pSvc->createCharacteristic(BLE_CHAR_CMD_UUID,
        BLECharacteristic::PROPERTY_WRITE);
    s_pCmd->setCallbacks(new CmdCallbacks());

    // Status: READ + NOTIFY
    s_pStatus = pSvc->createCharacteristic(BLE_CHAR_STATUS_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    s_pStatus->addDescriptor(new BLE2902());

    pSvc->start();

    BLEAdvertising *pAdv = BLEDevice::getAdvertising();
    pAdv->addServiceUUID(BLE_SERVICE_UUID);
    pAdv->setScanResponse(true);
    pAdv->setMinPreferred(0x06);
    BLEDevice::startAdvertising();

    DBG_PRINTLN("[BLE] Advertising as 'RoboMower'");
}

void ble_server_update() {
    if (!s_connected) return;

    // Telemetry rate: 1 Hz idle, 2 Hz active
    RobotState st = state_machine_get_state();
    bool idle = (st == STATE_INIT || st == STATE_IDLE || st == STATE_MOTORS_OFFLINE);
    uint32_t interval = idle ? 1000 : 500;

    uint32_t now = millis();
    if (now - s_last_telem_ms >= interval) {
        s_last_telem_ms = now;
        String telem = build_telemetry_json();
        ble_notify_fragmented(s_pTelem, telem);
    }
}

void ble_server_send_map() {
    if (!s_connected || !s_pMap) return;
    String map = build_map_json();
    ble_notify_fragmented(s_pMap, map);
}

void ble_server_send_status() {
    if (!s_connected || !s_pStatus) return;
    String status = build_status_json();
    ble_notify_fragmented(s_pStatus, status);
}

// ── Diagnostics JSON ─────────────────────────────────────────────────────────

static String build_diag_json() {
    // IMU
    float imu_hdg   = imu_get_heading_fused();
    float imu_gz    = imu_get_gz_rads();
    uint8_t imu_cal = imu_get_calib_status();   // packed sys/gyro/accel/mag
    float imu_pitch = imu_get_pitch_rad() * (180.0f / M_PI);
    float imu_roll  = imu_get_roll_rad()  * (180.0f / M_PI);
    float imu_ax, imu_ay, imu_az;
    imu_get_accel(&imu_ax, &imu_ay, &imu_az);
    bool  imu_ok    = imu_is_present() && !imu_is_fault();

    // GPS
    GpsMeasurement gps = rtk_gps_get_measurement();

    // RC
    CRSFChannels rc = crsf_get_channels();

    // VESCs — feedback
    VescStatus vL = vesc_get_status(VESC_ID_LEFT);
    VescStatus vR = vesc_get_status(VESC_ID_RIGHT);
    VescStatus vB = vesc_get_status(VESC_ID_BLADE);
    VescOdometry oL = vesc_get_odometry(VESC_ID_LEFT);
    VescOdometry oR = vesc_get_odometry(VESC_ID_RIGHT);

    // Blade mechanical RPM from eRPM
    const MowerConfig &mc_d = mower_config_get();
    float blade_rpm = (mc_d.blade_motor_pole_pairs > 0)
                    ? vB.erpm / (float)mc_d.blade_motor_pole_pairs : vB.erpm;

    // VESCs — commanded duty (replacing old current readback)
    float cmd_left_duty = 0, cmd_right_duty = 0;
    node_follower_get_last_duty(&cmd_left_duty, &cmd_right_duty);

    // EKF
    Pose2D pose = ekf_get_pose();
    float  spd  = ekf_get_speed();
    float  unc  = ekf_get_position_uncertainty();

    // Path follower & cutting
    float xte    = node_follower_get_cross_track_error();
    float cut_avg = cutting_monitor_get_avg_current();
    float cut_lf  = cutting_monitor_get_load_fraction();

    // System
    float batt   = vesc_get_battery_voltage();
    bool  can_ok = vesc_can_bus_ok();
    uint32_t now = millis();

    // Uncertainty-aware navigation
    float tilt = imu_get_tilt_rad() * (180.0f / M_PI);
    float margin_d = distanceToNearestEdge(perimeter_get_nav_boundary(),
                                            pose.x, pose.y) - unc;

    char buf[1280];
    snprintf(buf, sizeof(buf),
        "{\"type\":\"diag\","
        "\"imu\":{\"ok\":%d,\"hdg\":%.4f,\"gz\":%.4f,\"cal\":%d,"
        "\"pitch\":%.1f,\"roll\":%.1f,"
        "\"surge\":%.3f,\"sway\":%.3f,\"heave\":%.3f},"
        "\"gps\":{\"v\":%d,\"fix\":%d,\"sat\":%d,\"hdop\":%.2f,"
        "\"lat\":%.7f,\"lon\":%.7f,\"e\":%.3f,\"n\":%.3f,"
        "\"age\":%.1f,\"tmo\":%d},"
        "\"rc\":{\"ch\":[%u,%u,%u,%u,%u,%u,%u,%u],\"fs\":%d,\"age\":%lu},"
        "\"vL\":{\"rpm\":%.0f,\"A\":%.2f,\"d\":%.3f,\"dt\":%lu,"
        "\"dist\":%.2f,\"vel\":%.3f,\"duty\":%.3f},"
        "\"vR\":{\"rpm\":%.0f,\"A\":%.2f,\"d\":%.3f,\"dt\":%lu,"
        "\"dist\":%.2f,\"vel\":%.3f,\"duty\":%.3f},"
        "\"vB\":{\"rpm\":%.0f,\"erpm\":%.0f,\"A\":%.2f,\"d\":%.3f,\"dt\":%lu,"
        "\"cmd\":%d,\"lock\":%d},"
        "\"ekf\":{\"x\":%.3f,\"y\":%.3f,\"h\":%.4f,\"v\":%.3f,\"u\":%.4f},"
        "\"xte\":%.3f,\"cutA\":%.2f,\"cutLF\":%.3f,"
        "\"battV\":%.2f,\"can\":%d,"
        "\"margin\":%.2f,\"tilt\":%.1f,\"hdgGain\":%.2f,"
        "\"t\":%lu}",
        // IMU
        imu_ok ? 1 : 0, imu_hdg, imu_gz, (int)imu_cal, imu_pitch, imu_roll,
        imu_ax, imu_ay, imu_az,
        // GPS
        gps.valid ? 1 : 0, (int)gps.fix_type, (int)gps.sat_count, gps.hdop,
        gps.lat_deg, gps.lon_deg, gps.enu_east_m, gps.enu_north_m,
        gps.dif_age_s, rtk_gps_is_timeout() ? 1 : 0,
        // RC
        rc.ch[0], rc.ch[1], rc.ch[2], rc.ch[3],
        rc.ch[4], rc.ch[5], rc.ch[6], rc.ch[7],
        rc.failsafe ? 1 : 0, (unsigned long)(now - rc.last_frame_ms),
        // VESC Left
        vL.erpm, vL.current_A, vL.duty, (unsigned long)(now - vL.last_update_ms),
        oL.dist_m, oL.velocity_ms, cmd_left_duty,
        // VESC Right
        vR.erpm, vR.current_A, vR.duty, (unsigned long)(now - vR.last_update_ms),
        oR.dist_m, oR.velocity_ms, cmd_right_duty,
        // VESC Blade
        blade_rpm, vB.erpm, vB.current_A, vB.duty, (unsigned long)(now - vB.last_update_ms),
        state_machine_is_blade_commanded() ? 1 : 0,
        state_machine_is_blade_lockout()   ? 1 : 0,
        // EKF
        pose.x, pose.y, pose.heading, spd, unc,
        // PP, cutting, system
        xte, cut_avg, cut_lf,
        batt, can_ok ? 1 : 0,
        // Uncertainty navigation + heading stabilisation
        margin_d, tilt, heading_controller_get_gain(),
        (unsigned long)now);

    return String(buf);
}

void ble_server_send_diag() {
    if (!s_connected || !s_pStatus) return;
    String diag = build_diag_json();
    ble_notify_fragmented(s_pStatus, diag);
}

void ble_server_send_ack(const char *cmd, bool ok, const char *msg) {
    if (!s_connected || !s_pStatus) return;
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
        "{\"type\":\"ack\",\"cmd\":\"%s\",\"ok\":%s,\"msg\":\"%s\"}",
        cmd, ok ? "true" : "false", msg);
    if (len > 0 && len < (int)sizeof(buf)) {
        s_pStatus->setValue((uint8_t *)buf, (size_t)len);
        s_pStatus->notify();
    }
}

bool ble_server_is_connected() {
    return s_connected;
}
