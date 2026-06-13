// ══════════════════════════════════════════════════════════════════════════════
//  Robo-Mower-V2.ino — RoboMower ESP32-S3 Firmware v3
//  Arduino entry point: setup() and loop() only.
//  All subsystem logic is in the accompanying .cpp / .h files.
//
//  See README.md for full setup, calibration, and serial command reference.
// ══════════════════════════════════════════════════════════════════════════════
//
//  ── BOARD ──────────────────────────────────────────────────────────────────
//
//  Board:            ESP32S3 Dev Module
//                    (Espressif ESP32-S3-WROOM-1, N16R8 — 16MB flash, 8MB OPI PSRAM)
//  Arduino core:     esp32 by Espressif Systems v3.x
//  Partition scheme: Huge APP (3MB No OTA / 1MB SPIFFS)  ← REQUIRED
//                    Firmware exceeds the default 1.2MB partition.
//  PSRAM:            OPI PSRAM  (required for N16R8; set to Disabled if no PSRAM)
//  USB Mode:         Hardware CDC and JTAG
//  Upload speed:     921600 (default)
//  CLI FQBN:         esp32:esp32:esp32s3
//
//  ── REQUIRED LIBRARIES (install via Library Manager) ──────────────────────
//
//  SparkFun BMI270 Arduino Library   SparkFun Electronics   IMU (I2C, 200Hz)
//  FastLED                           Daniel Garcia          WS2812 LED control
//
//  ── BUILT-IN (ESP32 Arduino core — do NOT install separately) ─────────────
//
//  driver/twai.h          CAN/TWAI peripheral (VESC communication)
//  Preferences.h          NVS key-value storage (calibration, session state)
//  nvs.h / nvs_flash.h    NVS blob storage (perimeter, obstacle map)
//  HardwareSerial         UART1 = GPS, UART2 = CRSF RC receiver
//
//  ── COMPILE-TIME OPTIONS (config.h) ───────────────────────────────────────
//
//  TEST_MODE   0   Normal operation.
//              1   Runs geometry unit tests from setup() then halts.
//                  Use to verify geometry routines after any planner change.
//
//  ── PIN CONNECTIONS ────────────────────────────────────────────────────────
//
//  GPIO  1   TWAI TX   →  SN65HVD230 TXD pin
//              CAN transceiver shared by all 3 VESCs (500 kbit/s)
//              Twisted-pair CANH/CANL, 120Ω termination at each end of bus
//
//  GPIO  2   TWAI RX   ←  SN65HVD230 RXD pin
//
//  GPIO  4   (unused)  —  formerly battery ADC voltage divider
//              Battery voltage now read from VESC CAN STATUS_5 (no hardware needed)
//
//  GPIO  5   LEDC PWM  →  Cut-height servo signal wire
//              50 Hz, 1000–2000 µs pulse width, LEDC channel 0
//              Servo power: 5V rail; signal logic: 3.3V (direct connection OK)
//
//  GPIO  6   Pause switch  ←  Latching switch to GND (SPST)
//              INPUT_PULLUP, active LOW, 50ms settle debounce
//              Switch CLOSED (GND) → pause AUTO_MOWING, all state preserved
//              Switch OPEN         → resume mowing from same position
//              CH7 on RC transmitter provides independent pause (either input pauses)
//
//  GPIO  7   WS2812 data  →  External LED strip data-in
//              FastLED, SK6812/WS2812B, 8 LEDs (LED_EXTERNAL_COUNT)
//              Power: 5V rail; data: 3.3V logic (direct connection OK for most strips)
//
//  GPIO  8   I2C SDA   ↔  SparkFun BMI270 (SEN-22398) SDI pin
//              Wire.h, 400 kHz; 4.7kΩ pull-up to 3.3V on each line
//
//  GPIO  9   I2C SCL   →  SparkFun BMI270 (SEN-22398) SCK pin
//
//              BMI270 wiring summary:
//                ESP32 3.3V  →  VDD, VDDIO
//                ESP32 GND   →  GND, SA0  (SA0=GND → I2C address 0x68)
//
//  GPIO 10   Serial1 RX  ←  Quectel LC29H RTK GPS module TX pin
//              115200 baud, NMEA 0183 ($GNGGA sentences)
//              LC29H supply: check module board (typically 3.3V or 5V)
//              RTK corrections supplied externally via NTRIP (not this firmware)
//
//  GPIO 14   Serial1 TX  →  Quectel LC29H RTK GPS module RX pin
//              115200 baud, data requests (DFRobot RTK LoRa library)
//
//  GPIO 11   Serial2 RX  ←  RadioMaster ER8 CRSF receiver TX pin
//              420000 baud, CRSF protocol, RC channels inbound
//
//  GPIO 12   Serial2 TX  →  RadioMaster ER8 CRSF receiver RX pin
//              420000 baud, CRSF telemetry uplink (GPS position, battery,
//              attitude, custom mower status frame 0x80)
//
//              ER8 wiring summary:
//                5V rail  →  ER8 power (receiver requires 5V)
//                ESP32 GND  →  ER8 GND
//
//  GPIO 48   WS2812 data  →  Onboard NeoPixel (DevKitC-1 fixed location)
//              FastLED, single LED, mirrors external strip pattern
//              No external wiring needed
//
//  ── CAN BUS — VESC MOTOR CONTROLLERS ──────────────────────────────────────
//
//  All three VESCs share one CAN bus (daisy-chain, 120Ω at each end):
//
//  CAN ID 1  Left drive VESC    Differential drive, left wheel
//  CAN ID 2  Right drive VESC   Differential drive, right wheel
//  CAN ID 3  Blade VESC         Gtech CLM021 cutting deck (800W / 48V)
//
//  The left drive VESC (ID 1) must have STATUS_5 (CAN packet ID 27) enabled
//  in VESC Tool — this is the source of battery voltage telemetry.
//  See README.md §VESC Tool Configuration for required settings.
//
//  ── POWER RAILS ────────────────────────────────────────────────────────────
//
//  48V battery  →  PILZ safety relay  →  all three VESCs
//  5V rail      →  ESP32-S3 DevKitC-1 (USB or 5V pin), ER8 receiver,
//                  cut-height servo power
//  3.3V (ESP32 internal LDO)  →  BMI270, SN65HVD230, LED data lines
//
//  Supercapacitor backup (separate supply):
//    Powers ESP32 + ER8 + RTK GPS for several minutes after PILZ fires.
//    Enables clean NVS save and battery-swap mid-session without losing position.
//
//  PILZ safety relay: hardware E-stop only — no ESP32 GPIO connection.
//  When PILZ fires, VESCs go silent on CAN → firmware enters STATE_MOTORS_OFFLINE.
//
// ══════════════════════════════════════════════════════════════════════════════

// ── System includes ──────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <driver/twai.h>
#include <FastLED.h>
#include <vector>
#include <algorithm>
#include <cmath>

// SparkFun BMI270 library (install via Library Manager)
#include <SparkFun_BMI270_Arduino_Library.h>

// ── Local module includes (in dependency order) ──────────────
#include "config.h"
#include "mower_config.h"
#include "geometry.h"
#include "geometry_test.h"
#include "nvs_storage.h"
#include "crsf_input.h"
#include "crsf_telemetry.h"
#include "vesc_can.h"
#include "rtk_gps.h"
#include "imu_bmi270.h"
#include "collision_detect.h"
#include "servo_output.h"
#include "obstacle_map.h"
#include "ekf_localiser.h"
#include "perimeter.h"
#include "cutting_monitor.h"
#include "coverage_planner.h"
#include "pure_pursuit.h"
#include "bog_recovery.h"
#include "retrace.h"
#include "battery_monitor.h"
#include "safety.h"
#include "state_machine.h"
#include "ble_server.h"
#include "sys_log.h"

// ─────────────────────────────────────────────────────────────

void setup() {
    DBG_BEGIN(115200);
    delay(500);  // Allow USB CDC to connect
    DBG_PRINTLN("[BOOT] RoboMower v3 starting...");

#if TEST_MODE
    // ── TEST MODE: run geometry unit tests and halt ──────────
    DBG_PRINTLN("[TEST] Running geometry unit tests...");
    int failures = geometry_test_runAll();
    DBG_PRINTF("[TEST] Done: %d failures\n", failures);
    DBG_PRINTLN("[TEST] Halting. Reflash without TEST_MODE for normal operation.");
    while (true) { delay(1000); }
#endif

    // ── Normal boot sequence ─────────────────────────────────

    // 1. NVS storage (must be first — validates all blobs)
    if (!nvs_storage_init()) {
        sys_log_push("NVS: one or more stored blobs failed CRC — data may be corrupt");
    }

    // 1b. Runtime mower config (after NVS, before any module that uses dimensions)
    mower_config_init();

    // 2. Hardware peripherals
    pinMode(PAUSE_PIN, INPUT_PULLUP);   // physical pause switch, GPIO6, active LOW
    servo_output_init();
    vesc_can_init(CAN_TX_PIN, CAN_RX_PIN);  // from config.h
    if (!imu_bmi270_init()) {
        sys_log_push("IMU: BMI270 not detected — check I2C wiring");
    }
    collisionDetectInit();     // loads collision baseline from NVS
    rtk_gps_init();            // starts GPS parse task on Core 0
    // Telemetry MUST init before the CRSF RX task starts: the task calls
    // crsf_telemetry_service() on the first inter-packet gap (within ~3 ms of
    // creation, at priority 14 — preempting the rest of setup() on this core),
    // and that uses the mutex crsf_telemetry_init() creates. Starting the task
    // first races a null-mutex xSemaphoreTake → configASSERT abort at boot.
    crsf_telemetry_init();
    crsf_input_init();         // starts CRSF RX task on Core 1

    // 3. Algorithm modules
    ekf_init();
    perimeter_init();          // loads from NVS

    // The perimeter origin (first vertex lat/lon) is stored in NVS alongside
    // the perimeter and restored in rtk_gps_init(). Both are set atomically
    // by handle_send_perimeter(), so no separate consistency check is needed.

    obstacle_map_init(perimeter_get_perimeter());  // init grid
    coverage_planner_init();
    pure_pursuit_init();
    cutting_monitor_init();

    // 4. Battery monitor (before safety — safety checks battery state)
    battery_monitor_init();    // configures ADC, loads NVS cal offset

    // 5. Safety (must start before state machine)
    safety_init();             // starts safety watchdog task on Core 1

    // 6. State machine (starts from INIT state)
    state_machine_init();

    // 7. BLE GATT server (after state machine so getters are valid)
    ble_server_init();

    DBG_PRINTLN("[BOOT] Setup complete.");
}

void loop() {
    // Core 1 main loop — runs state machine at 10Hz
    static uint32_t last_sm_ms = 0;
    uint32_t now = millis();

    if (now - last_sm_ms >= 100) {  // 10Hz
        last_sm_ms = now;
        state_machine_update();
    }

    // Handle serial commands (non-blocking check)
    static char serial_buf[64];
    static int  serial_idx = 0;
    while (DBG_AVAILABLE()) {
        char c = DBG_READ();
        if (c == '\n' || c == '\r') {
            if (serial_idx > 0) {
                serial_buf[serial_idx] = '\0';
                state_machine_handle_serial(serial_buf);
                serial_idx = 0;
            }
        } else if (serial_idx < 63) {
            serial_buf[serial_idx++] = c;
        }
    }

    // Yield to avoid starving Core 1 tasks
    vTaskDelay(1);
}
