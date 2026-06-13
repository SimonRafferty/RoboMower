# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3 firmware for an autonomous RTK GPS lawnmower robot. Uses VESC motor controllers over CAN bus, a BMI270 IMU, CRSF radio (RadioMaster TX16S + ER8 receiver), and a boustrophedon coverage planner with headland passes.

**Board:** ESP32S3 Dev Module — specifically the **N16R8 variant** (16 MB flash, 8 MB OPI PSRAM). Generic ESP32-S3 boards are **not compatible** (blank flash requires Flash Download Tool or JTAG to recover — do not flash wrong firmware).

Companion files: `robomower-pwa/robomower.html` (single-file PWA, Web Bluetooth + Leaflet), `TX16S Telemetry LUA script/main.lua` (EdgeTX Lua widget for TX16S, reads MOWER_STATUS 0x80 telemetry frame).

## Build & Upload

### Arduino IDE 2.x (preferred)

Open `Robo-Mower-V2/Robo-Mower-V2.ino`. Required board settings:

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| Partition Scheme | **Huge APP (3MB No OTA / 1MB SPIFFS)** ← REQUIRED |
| PSRAM | OPI PSRAM |
| USB Mode | Hardware CDC and JTAG |

Required libraries (install via Library Manager):
- `SparkFun BMI270 Arduino Library` by SparkFun Electronics
- `FastLED` by Daniel Garcia

### arduino-cli (safe to use — previously caused BSOD but was updated 2026-04-09)

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  --board-options "PartitionScheme=huge_app,PSRAM=opi" Robo-Mower-V2/

arduino-cli upload -p COM44 --fqbn esp32:esp32:esp32s3 Robo-Mower-V2/
```

**Always upload to COM44 after a successful compile.**

### Build-time switches (`config.h:22–34`)

- `TEST_MODE 1` — runs the geometry unit-test suite from `setup()` and halts (pass/fail on USB Serial at 115200). Reset to 0 for normal operation.
- `DEBUG_SERIAL 1` — enables USB Serial debug output, serial commands, and the 2 Hz JSON telemetry. **At 0 (production default) all USB Serial I/O compiles to no-ops** — serial commands will not respond. Does not affect CRSF or BLE.
- `BENCH_TEST_NO_VESC 1` — suppresses cutting-monitor fault transitions (blade fault / overload / stall) for bench testing without VESCs. Never 1 in field firmware.

### Geometry unit tests — host build (no hardware needed)

`host_test/` compiles `geometry.cpp` + `geometry_test.cpp` for Windows using Zig's bundled clang (`winget install zig.zig`), plus extra concave-perimeter / chained-inset scenarios in `host_test/main.cpp`:

```powershell
cd host_test
& "$env:LOCALAPPDATA\Microsoft\WinGet\Packages\zig.zig_*\zig-*\zig.exe" c++ -std=c++17 -O1 -o geomtest.exe main.cpp geometry.cpp geometry_test.cpp
.\geomtest.exe   # exit code = number of failures
```

`host_test/geometry.cpp` / `geometry_test.cpp` are COPIES of the Robo-Mower-V2 sources with `config.h` swapped for `host_config.h` — re-copy them after editing the originals:
`(Get-Content Robo-Mower-V2\geometry.cpp -Raw) -replace '#include "config.h"', '#include "host_config.h"' | Set-Content host_test\geometry.cpp -Encoding utf8` (same for `geometry_test.cpp`, which also drops `#include <Arduino.h>`).

## Repository Notes

- **Remote:** `https://github.com/SimonRafferty/RoboMower`. The `Robo_Mower_2/` tree is its **own standalone git repo**, pushed there. Note it is physically **nested inside** the home-directory `MAKER-Code` monorepo (`C:/Users/simon/.git`) — when working on the mower, the nested `.git` in `Robo_Mower_2/` is the one that matters. Don't auto-push; push only when the user asks.
- The working directory is `Robo_Mower_2/`; the Arduino sketch is in `Robo-Mower-V2/`.
- Documentation: `telemetry.md` (CRSF frame payload layouts), `manual.md` (user-facing operating guide), `README.md` (full ops manual with wiring and tuning).

## Architecture

### Module overview (roughly in dependency order)

```
config.h            — All #define constants: pins, physical dims, tuning, derived values; DEBUG_SERIAL=0 by default
mower_config.h/.cpp — Runtime-configurable MowerConfig struct; BLE SET_CONFIG updates this
geometry.h/.cpp     — 2D polygon maths (inset, hull, intersect); geometry_test.h/.cpp for unit tests
nvs_storage.h/.cpp  — All NVS persistence (Preferences + blob API); CRC32 on every blob
crsf_input.h/.cpp   — CRSF RC frame parse (420 kbaud, Serial2); runs FreeRTOS task Core 1
crsf_telemetry.h/.cpp — 5-frame telemetry rotation to TX16S (GPS, Battery, Attitude, FlightMode, MOWER_STATUS 0x80)
vesc_can.h/.cpp     — TWAI/CAN driver; SET_DUTY / SET_CURRENT / SET_RPM commands; STATUS_1–5 reception
rtk_gps.h/.cpp      — Quectel LC29H NMEA parse (115200, Serial1); ENU origin management; Core 0 task
imu_bmi270.h/.cpp   — BMI270 I2C at 200 Hz; gyro bias collection; feeds EKF + collision detect
collision_detect.h/.cpp — Adaptive-baseline jolt detector; direction (fwd/side/rear) classification
servo_output.h/.cpp — Cut-height servo; LEDC PWM; fixed mechanical calibration 1000–2000 µs (#defines, no NVS cal)
obstacle_map.h/.cpp — Grid of detected obstacle positions; routes around them
ekf_localiser.h/.cpp — 4-state EKF [x, y, θ, v]; GPS + odometry + gyro fusion; ENU frame
perimeter.h/.cpp    — Polygon record/save/load; derives nav boundary and working area
cutting_monitor.h/.cpp — Blade current monitoring; load fractions; CUTTING_STALLED / OVERLOADED / OBSTACLE_SUSPECTED
coverage_planner.h/.cpp — Boustrophedon strip planner; headland passes; optimal angle selection
pure_pursuit.h/.cpp — Path-following; lookahead scaling; cross-track error recovery
heading_controller.h/.cpp — Gyro-based heading stabilisation; PD correction used by manual drive
wheel_duty_ramp.h/.cpp — AUTO drive control: fixed duty step (0.02/tick at 10 Hz) toward target speed via SET_DUTY
wheel_speed_pi.h/.cpp — LEGACY: old PI current loop, superseded by wheel_duty_ramp; compiled but no longer called
bog_recovery.h/.cpp — Progressive height-raise recovery when stalled; eRPM bogdown detection
retrace.h/.cpp      — Strip re-trace at max height when overloaded
battery_monitor.h/.cpp — Battery voltage from VESC CAN STATUS_5 (ID 1); warn/low thresholds
safety.h/.cpp       — VESC-silence, blade-stale, perimeter breach watchdogs; FreeRTOS task Core 1
state_machine.h/.cpp — Top-level 11-state FSM at 10 Hz; manual drive; BLE command handler; LEDs; serial commands; 2 Hz JSON telemetry
ble_server.h/.cpp   — BLE GATT server (4 characteristics: TELEM, MAP, CMD, STATUS); fragment reassembly; queues JSON to Core 1
sys_log.h/.cpp      — Circular in-memory log for diagnostic messages
```

### FreeRTOS core assignment

| Core | Tasks |
|------|-------|
| Core 0 | `rtk_gps` task, `ble_server` task, VESC CAN RX (TWAI driver) |
| Core 1 | `loop()` → `state_machine_update()` at 10 Hz, `crsf_input` task, `safety` watchdog task |

All shared state is accessed via `volatile` atomics or `state_machine_get_*()` getters — never directly from the other core. EKF state is protected by a FreeRTOS mutex (Core 0 writes, Core 1 reads).

### State machine (11 states)

`STATE_INIT` → `STATE_IDLE` → `STATE_MANUAL` / `STATE_LEARN_PERIMETER` / `STATE_AUTO_MOWING`

Recovery sub-states: `STATE_RETRACE`, `STATE_BOG_RECOVERY`, `STATE_OBSTACLE_AVOID`

Completion: `STATE_AUTO_RETURN` → `STATE_IDLE`

Operator pause: `STATE_PAUSED` (CH7 switch, GPIO6 latching switch, BLE pause, serial `PAUSE`, or software safety fault — perimeter breach, blade fault, tilt limit all trigger this state; VESCs still powered, CAN stop sent)

Hardware fault: `STATE_MOTORS_OFFLINE` (PILZ E-stop fired — VESC power cut; ESP32 runs on supercap)

Other behaviours wired into the FSM:
- RC failsafe (no CRSF, no BLE, no fresh BLE drive) only acts while operator-driving: MANUAL/LEARN → IDLE (`state_machine.cpp:1491`). RC is deliberately off during autonomous operation.
- Battery WARNING/LOW → **operator notification only**: TX warning beep (repeated every 60 s) + MOWER_STATUS flags bit 0x20 (flashing banner on the Lua widget) + PWA toast/beep + amber LED overlay. **No blade lockout and no auto-return** (both removed 2026-06-12, operator decisions — the VESCs handle low-voltage power reduction internally; the operator selects Return. Auto-return may be reinstated when the mower can self-charge — future project).
- IDLE entry runs a non-blocking "look-around" turret animation (`LOOK_*` phases, `state_machine.cpp:1599`).

### Three nested polygons

All derived from the user-taught perimeter and stored in NVS:
1. **Perimeter** — as recorded (outermost blade edge)
2. **Nav boundary** — inset by `NAV_EXCLUSION_INSET_M`; steering centre must stay inside
3. **Working area** — inset further by `HEADLAND_WIDTH_M`; where boustrophedon strips are planned

### EKF localiser

4-state: `[x(m), y(m), θ(rad), v(m/s)]` in local ENU frame (origin set by perimeter upload, else auto-seeded at first RTK-fixed fix; persisted in NVS).
- `ekf_predict(v_left, v_right, dt)` at 10 Hz from `state_machine_update()` — heading from **differential wheel odometry** `(v_right−v_left)/track_width`. **Deliberate decision (June 2026): the gyro is NOT used for heading** — the ground is too bumpy and the machine has near-zero wheel slip, so odometry is the better source. This requires live drive-VESC eRPM frames.
- `ekf_update_gps()` at ~1 Hz — position **snaps** to GPS; heading corrected from GPS travel direction when RTK-quality and enough distance accumulated. Innovation gate = `max(5σ, 5 m)`; bypassed for the first fix after boot (cold-start seed).
- Reported uncertainty = last GPS σ.

### VESC motor control

Drive VESCs (CAN ID 1, 2) are commanded by **duty cycle** (`vesc_set_duty()` → PKT_SET_DUTY); stops are commanded with `vesc_set_current(id, 0)`.

**Hardware note:** the drive VESCs are HW **v4** (older firmware, no per-message CAN-status selection — verify "Send CAN Status" is enabled in VESC Tool or they broadcast nothing); the blade VESC is HW **v6** with status 1–5 enabled. Closed-loop drive control, odometry heading, and slip/bog detection all require drive STATUS_1 (eRPM) frames.

- **Manual drive** (`state_machine.cpp` `drive_manual`): tank-mix of expo/deadband sticks → per-wheel duty, with a gyro heading-stabilisation PD overlay (`heading_controller_compute()`) and a slew-rate limit (`MANUAL_DUTY_RAMP_PER_S`) to stop wheelies. Fully open-loop — works without VESC telemetry. BLE/WebUI drive sliders are only honoured while the RC TX is off (failsafe active); RC always wins when present.
- **AUTO drive** (`wheel_duty_ramp.cpp`, called from `pure_pursuit.cpp`): steps each wheel's duty by a fixed `DUTY_RAMP_STEP` (0.02) per 10 Hz tick toward the target m/s, using live VESC eRPM as feedback; clamped to `manual_max_duty`. **Staleness guard:** if a drive VESC's status was never seen or is older than `DUTY_FEEDBACK_STALE_MS` (500 ms), the ramp does NOT integrate (a frozen 0 reading would wind duty to max — runaway); it falls back to bounded open-loop duty and logs once. The old PI-current loop (`wheel_speed_pi.h/.cpp`) still compiles but has no callers.
- **Blade** (CAN ID 3): **RPM control** — keepalive sends `SET_RPM` (target eRPM = `blade_target_rpm × pole_pairs`) every 100 ms; the VESC's internal current-limited RPM PID gives a smooth ~2 s spin-up. A firmware duty ramp was tried and **reverted 2026-06-12** (jerky steps, bad interaction with CAN dropouts) — don't reintroduce it. Disarm = zero-current command (freewheel, no regen). BLADE_FAULT requires `BLADE_FAULT_GRACE_MS` (4 s) after arm before it can fire (`cutting_monitor.cpp`). Telemetry flags bit 0x02 reports the real `s_blade_commanded` state.

### BLE ↔ PWA protocol

BLE server (`ble_server.cpp`) runs on Core 0 and only transports bytes: commands arrive on the `CMD` characteristic as JSON (large payloads fragmented as `{"f":<i>,"n":<total>,"d":"<chunk>"}`), are reassembled, and queued via `state_machine_enqueue_ble_cmd()`. **Command parsing/dispatch happens on Core 1 in `state_machine.cpp`** (`handle_ble_command()`).

The PWA (`robomower-pwa/robomower.html`) is a single self-contained HTML file with four tabs: Dashboard (live status + config panel), Map (Leaflet + leaflet-geoman perimeter editing), Controls (drive sliders, mode buttons, height), Diagnostics (real-time sensor data). Web Bluetooth API — Chrome on Android only.

### CRSF telemetry rotation

Five frames in rotation: `FLIGHT_MODE (0x1E)` → `GPS (0x02)` → `BATTERY (0x08)` → `MOWER_STATUS (0x80)` → `ATTITUDE (0x14)` (`crsf_telemetry.cpp:53` `s_frame_order[]`). Data snapshot at 2 Hz from state machine; frames dispatched by CRSF RX task on inter-packet gaps (~100 Hz opportunity). See `telemetry.md` for full payload layouts.

### Physical constants and tuning

All in `config.h` (compile-time `#define`) and mirrored in `mower_config.h` (runtime `MowerConfig` struct updatable via BLE). The runtime struct takes precedence after first boot.

Key derived constants (computed in `config.h` and `mower_config.cpp`):
- `NAV_EXCLUSION_INSET_M = max(robot_left, robot_right, wheel_half_track) + 0.05`
- `HEADLAND_WIDTH_M = max(CUT_WIDTH_M×1.5, ROBOT_REAR_M + STRIP_OVERLAP_M, blade_fwd_reach + STRIP_OVERLAP_M)`

## Project Map — Where to Find What

Quick-lookup reference by function/topic. Line numbers are approximate; function names are stable.

### GPIO pin assignments
All in `config.h` (~lines 57–100). Key pins: CAN TX/RX GPIO2/1 · servo GPIO5 · pause switch GPIO6 · external LED strip GPIO7 · IMU I2C SDA/SCL GPIO8/9 · GPS Serial1 RX/TX GPIO10/14 · CRSF Serial2 RX/TX GPIO11/12 · onboard NeoPixel GPIO48.

### State transitions
- `state_machine_update()` entry: `state_machine.cpp:1334`; main FSM switch: `:1548` (state cases `:1551–2680`)
- Transition helper: `state_machine.cpp:211` `transition_to(next, event_latch)`
- Pre-FSM safety overrides (~`:1500–1650`): VESC silence → MOTORS_OFFLINE · perimeter breach → PAUSED · RC failsafe → IDLE · battery LOW → auto-return (no blade lockout) · CH7/GPIO6/BLE pause → PAUSED
- In AUTO_MOWING: blade fault → PAUSED (latched) `:2025` · CUTTING_OVERLOADED → RETRACE `:2032` · CUTTING_STALLED → BOG_RECOVERY `:2048` · OBSTACLE_SUSPECTED → OBSTACLE_AVOID `:2059` · tilt → PAUSED `:2092` · collision → OBSTACLE_AVOID `:2198` · wheel slip → BOG_RECOVERY `:2211`
- Fault responses are gated for `OBSTACLE_DETECT_STARTUP_MS` after AUTO entry (`obs_armed`, `:2019`) — blade spinup looks like a stall.

### RC channel mapping
`crsf_input.h:24–32` — CH1=steer, CH2=throttle, CH3=cut height, CH4=mode (manual/auto/return), CH5=learn perimeter, CH6=arm/blade, CH7=pause (`CRSF_CH_PAUSE` in `config.h:241`), CH8=momentary record point. Switch decode with hysteresis: `state_machine.cpp:1410–1417`. `ch[]` values are in **microseconds** (1000–2000), not raw CRSF units.

### Custom MOWER_STATUS telemetry (CRSF 0x80)
`crsf_telemetry.cpp:292` `sendMowerStatus()` — **19-byte** payload: state, progress, cut height, blade load, fix type, flags (+beep bits 7:6), obstacle count, EKF uncertainty, mowed area, **battery V×100 and heading deg×10 (offsets 15–18, added 2026-06-10 so the Lua widget does not depend on EdgeTX `RxBt`/`Yaw` sensor discovery)**. The Lua accepts both 15- and 19-byte payloads.

### EKF localiser
- Prediction (gyro + wheel odometry model, 200 Hz): `ekf_localiser.cpp:226` `ekf_predict()`
- GPS update + innovation gate (max(5σ, 5 m), cold-start bypass): `ekf_localiser.cpp:274` `ekf_update_gps()`; gate check at `:312–331`
- ENU origin lock: `rtk_gps.cpp` `rtk_gps_set_origin()` — called at first RTK-fixed GPS fix and when perimeter is received via BLE

### Motor commands
- Drive duty: `vesc_can.cpp` `vesc_set_duty(id, duty)` → CAN PKT_SET_DUTY; stop via `vesc_set_current(id, 0)`
- Blade SET_RPM: `vesc_can.cpp` `vesc_set_rpm(id, erpm)` → CAN PKT_SET_RPM
- AUTO duty ramp: `wheel_duty_ramp.cpp:20` `wheel_duty_ramp_compute()` — step = `DUTY_RAMP_STEP` (0.02)/tick, called from `pure_pursuit.cpp:411`
- Manual heading PD correction: `heading_controller.cpp:31` `heading_controller_compute()`, applied in `drive_manual` at `state_machine.cpp:775`
- Blade spinup ramp (3 s): `s_blade_ramp_erpm` interpolation in MANUAL `state_machine.cpp:~1737` and AUTO handlers, gated by `s_last_armed`

### Safety watchdogs (`safety.cpp`, Core 1, 50 ms period)
- Both drive VESCs stale > 2 s → STATE_MOTORS_OFFLINE: `safety.cpp:~190–215`
- Blade VESC stale (separate, non-fatal for drive): `safety.cpp:217`
- Perimeter breach > `PERIMETER_BREACH_DIST_M` outside nav boundary → pause request: `safety.cpp:246`
- Tilt limit (normal vs careful, from MowerConfig) checked in AUTO: `state_machine.cpp:2088`
- Uncertainty-aware navigation (speed scaling, careful collision multiplier, strip truncation): `state_machine.cpp:2070–2110`

### Perimeter recording
- Sample GPS point (0.2 m distance gate): `perimeter.cpp:184` `perimeter_record_point()`
- Auto-close polygon on LEARN exit: `perimeter.cpp:221` `perimeter_close_track()`
- Recompute nav boundary + working area insets: `perimeter.cpp:448` `perimeter_recompute()`

### Coverage planning (`coverage_planner.cpp`)
- Entry point (optimal angle, headland passes, boustrophedon strips): `coverage_planner_plan()` `:346`
- Each strip region gets a **region-outline pass before its strips** (added 2026-06-12, Slic3r-style innermost perimeter) — seals the wedge gaps that parallel strips leave along boundary edges not parallel to the mow direction
- Unreachable zones (area < 0.5 m²): detected during planning, logged, skipped; accessible via `UNREACHABLE` serial command
- Waypoint drain: `coverage_planner_get_next()` `:767`; resume near position: `coverage_planner_reset_to_nearest(x, y)` `:796`

### Recovery modes
- Stall vs overload classification: `cutting_monitor.cpp:83` `assessCuttingCondition()`
  - STALLED (not moving, load high) → BOG_RECOVERY; OVERLOADED (moving, load high) → RETRACE; OBSTACLE_SUSPECTED (blade commanded, load low, not moving) → OBSTACLE_AVOID
- Bog recovery state machine: `bog_recovery.cpp:141` `bog_recovery_update()` (dwell → lower deck → forward pass, up to 6 retries)
- eRPM bogdown / wheel slip: `pure_pursuit.cpp:397` `pure_pursuit_is_slipping()` → BOG_RECOVERY at `state_machine.cpp:2211`
- IMU collision spike: `collision_detect.cpp` `collisionDetected()` → OBSTACLE_AVOID at `state_machine.cpp:2198`

### BLE command handling
- Command dispatch: `state_machine.cpp:915` `handle_ble_command()` — routes on `"cmd"` JSON field (runs on Core 1; `ble_server.cpp` only reassembles fragments and queues)
- SET_CONFIG (update MowerConfig via BLE): `state_machine.cpp:1123`
- SEND_PERIMETER (origin + polygon from PWA): `state_machine.cpp:822` `handle_send_perimeter()`
- MAP characteristic assembly (perimeter, nav boundary, working area, obstacles): `ble_server.cpp` `ble_server_send_map()`
- TELEM characteristic (periodic sensor snapshot): `ble_server.cpp` `ble_server_update()`

### NVS read/write locations
- Perimeter / nav boundary / working area blobs: `nvs_storage.cpp` `nvs_save_perimeter()` / `nvs_save_nav_boundary()` / `nvs_save_working_area()`
- MowerConfig blob (`mow_cfg`): `mower_config.cpp` `mower_config_set()`
- Blade calibration float (`blade_cal`): `cutting_monitor.cpp:134`
- Adaptive collision baseline float (`coll_base`): `collision_detect.cpp`, saved every 120 s if changed > 0.02 g
- GPS ENU origin (`gpsorigin`): `nvs_save_gps_origin()` / `nvs_load_gps_origin()`

### Blade load calibration
`cutting_monitor.cpp:117` `updateCalibration()` — P90 percentile of blade current samples collected over first 3 s of AUTO_MOWING. Minimum valid: 2.0 A. Voltage-compensated against nominal 54.6 V (13S × 4.2 V).

### 2 Hz JSON telemetry (USB Serial, requires DEBUG_SERIAL=1)
`state_machine.cpp:663` `emit_telemetry()` — fields: `t, state, x, y, hdg, fix, sat, vel, hprog, sprog, cutH, obs, unc, bladeRPM, bladeA, bladeLoad, cutStatus, battV, battState, collBase`.

### LED patterns
Pattern enum: `state_machine.h:55`. Per-state colours in `state_machine.cpp`: INIT/IDLE=blue · MANUAL=green · LEARN=orange · AUTO/RETRACE/BOG=red · PAUSED=yellow. GPS overlay (RTK-fixed=green, float=orange, other=red): `state_machine.cpp:348` `showLedsWithGps()`. Battery warning amber flash every 2 s: `state_machine.cpp:383` `applyBatteryWarningOverlay()`. Onboard LED = GPIO48 `s_leds[0]`; external strip = GPIO7 `s_leds[1+]`.

## NVS Key Registry

**Warning:** Changing the NVS namespace or renaming keys discards all saved config the user has manually entered (perimeter, calibration, robot dimensions). Always warn the user before bumping the `mow_cfg` NVS key or clearing the `mower` namespace.

Key NVS keys (namespace `"mower"`):
- `perimeter`, `nav_bound`, `work_area` — polygon blobs with CRC32
- `gpsorigin` — ENU origin (lat/lon + CRC32)
- `estops` — circular E-stop log (20 × 44 bytes)
- `gyro_bias` — float; Z-axis gyro offset (rad/s)
- `blade_cal` — float; auto-calibrated blade reference current (A)
- `mow_cfg` — MowerConfig blob (all runtime physical constants)
- `coll_base` — float; adaptive collision baseline (g)

(The old `servo_min_us` / `servo_max_us` keys are gone — the cut-height servo now uses fixed 1000–2000 µs `#define` calibration in `servo_output.cpp`.)

## Serial Commands (115200 baud, USB — only active when `DEBUG_SERIAL 1`)

`STATUS`, `PERIMETER`, `NAVBOUNDARY`, `WORKINGAREA`, `PLAN`, `GRID`, `UNREACHABLE`, `EKFSTATE`, `CALDUMP`, `OBSTACLES`, `ERRORS`, `CLEARPERIM [CONFIRM]`, `RESETEKF`, `CALHEIGHT TEST <mm>`, `PAUSE`

Handler: `state_machine.cpp` `state_machine_handle_serial()` (~line 466). Firmware also outputs 2 Hz JSON telemetry automatically.

## VESC CAN IDs

| ID | Motor | Control mode |
|----|-------|-------------|
| 1 | Left drive | Duty (SET_DUTY); stop via SET_CURRENT 0 |
| 2 | Right drive | Duty (SET_DUTY); stop via SET_CURRENT 0 |
| 3 | Blade (Gtech CLM021) | RPM (SET_RPM) |

CAN baud: **250 kbit/s** (`CAN_BAUD_RATE` in `config.h`). Note: README says 500 kbit/s but `config.h` is authoritative.

Battery voltage is read from the **blade VESC (ID 3) STATUS_5** packet (changed 2026-06-07 from drive VESC ID 1 — the v4 drives may not broadcast STATUS_5). No ADC hardware needed.

### Field diagnostics (BLE — no bench/serial needed)
- **Boot telemetry self-test**: first 8 s after boot, the TX16S battery/blade-load/heading sweep synthetic ramps (40→55 V, 0→100 %, 0→360°). Sweep visible = radio/Lua path good; frozen afterwards = CAN/decoding side.
- **System log over BLE**: `sys_log` ring (30 entries) is included in the STATUS JSON (`log` array) and shown in the PWA. State transitions, AUTO denial reasons (with position + distance), plan results, blade on/off, lockout, fault triggers, and a 5-second `CAN L[a.. e..] R[..] B[..] V.. ok..` raw VESC RX dump (a = ms since last STATUS frame, a-1 = never received) all land there.
- **PWA Diagnostics tab**: per-VESC `dt` (status age), eRPM, current, plus `vB.cmd` (blade commanded; `vB.lock` is retained in the JSON but always 0 — lockout removed). "Plan Test" button = coverage-planner dry run (BLE `PLAN_TEST` command); "Refresh Log" re-fetches STATUS.
