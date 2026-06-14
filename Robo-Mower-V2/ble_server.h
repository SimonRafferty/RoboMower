#pragma once
#include <Arduino.h>

// ══════════════════════════════════════════════════════════════════════════════
//  ble_server.h — RoboMower BLE GATT server
//
//  Exposes four GATT characteristics over a single BLE service:
//    TELEM   — NOTIFY only; 2 Hz JSON telemetry (active) / 1 Hz (idle)
//    MAP     — NOTIFY + READ; full map JSON on request or perimeter change
//    CMD     — WRITE with response; JSON commands from phone → ESP32
//    STATUS  — READ + NOTIFY; status JSON on request
//
//  Large CMD payloads (e.g. SEND_PERIMETER) use a BINARY fragment protocol:
//    [0x01][frag_index][frag_total][data...]  (raw payload, no JSON escaping)
//  Fragments are reassembled before dispatch.
//
//  Lifecycle:
//    ble_server_init()   — call once from setup() after state_machine_init()
//    ble_server_update() — call from state_machine_update() every 100 ms tick
//
//  Thread safety:
//    BLE callbacks run on Core 0. Commands are forwarded to Core 1 via
//    state_machine_enqueue_ble_cmd(). Notify calls in ble_server_update() /
//    ble_server_send_map() etc. run on Core 1 (called from state_machine).
//
//  References:
//    Plan:    rosy-inventing-star.md §A5, §A6
//    config.h BLE_SERVICE_UUID etc.
// ══════════════════════════════════════════════════════════════════════════════

// ── Service and characteristic UUIDs ────────────────────────────────────────
#define BLE_SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_TELEM_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_CHAR_MAP_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define BLE_CHAR_CMD_UUID     "beb5483e-36e1-4688-b7f5-ea07361b26aa"
#define BLE_CHAR_STATUS_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26ab"

// ── Lifecycle ────────────────────────────────────────────────────────────────

/** Initialise the BLE GATT server and start advertising as "RoboMower".
 *  Must be called once from setup() after state_machine_init(). */
void ble_server_init();

/** Throttled telemetry sender — call every state machine tick (100 ms).
 *  Sends telemetry at 2 Hz when active, 1 Hz when idle.
 *  No-op when no client is connected. */
void ble_server_update();

/** Send the full map JSON via the MAP characteristic (fragmented if needed).
 *  Call when g_ble_map_pending is set. No-op if not connected. */
void ble_server_send_map();

/** Send a status JSON snapshot via the STATUS characteristic.
 *  Call when g_ble_status_pending is set. No-op if not connected. */
void ble_server_send_status();

/** Send a diagnostics JSON snapshot via the STATUS characteristic.
 *  Call when g_ble_diag_pending is set. No-op if not connected. */
void ble_server_send_diag();

/** Send a command acknowledgement on the CMD characteristic.
 *  @param cmd  Command name that was processed.
 *  @param ok   true = success, false = failure.
 *  @param msg  Short human-readable message (max ~60 chars). */
void ble_server_send_ack(const char *cmd, bool ok, const char *msg);

/** Return true if a BLE central is currently connected. */
bool ble_server_is_connected();
