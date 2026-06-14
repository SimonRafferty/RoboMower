# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ESP32-S3 firmware for an autonomous RTK GPS lawnmower robot. VESC motor controllers over CAN bus, a Bosch **BNO055** 9-axis IMU (on-chip NDOF fusion → tilt-compensated absolute heading), CRSF radio (RadioMaster TX16S + ER8 receiver), and a **concentric inward spiral** coverage planner (replaced boustrophedon strips 2026-06-13) built on the vendored Clipper2 polygon-offset library.

> **Read first:** *Known Issues & TODO* below — much changed on 2026-06-13/14, be wary of stale assumptions.

**Board:** ESP32S3 Dev Module, **N16R8 variant** (16 MB flash, 8 MB OPI PSRAM). Generic ESP32-S3 boards are **not compatible** (a blank flash needs Flash Download Tool or JTAG to recover — do not flash the wrong firmware).

Companion files: `robomower-pwa/robomower.html` (single-file PWA, Web Bluetooth + Leaflet); `TX16S Telemetry LUA script/main.lua` (EdgeTX widget reading the MOWER_STATUS 0x80 frame).

## Build & Upload

### Arduino IDE 2.x (preferred)

Open `Robo-Mower-V2/Robo-Mower-V2.ino`. Required board settings:

| Setting | Value |
|---------|-------|
| Board | ESP32S3 Dev Module |
| Partition Scheme | **Huge APP (3MB No OTA / 1MB SPIFFS)** ← REQUIRED |
| PSRAM | OPI PSRAM |
| USB Mode | Hardware CDC and JTAG |

Required libraries (Library Manager): `Adafruit BNO055` (+ `Adafruit Unified Sensor`) (IMU) and `FastLED`.

**Clipper2 is vendored in-tree** at `Robo-Mower-V2/src/clipper2/` (Boost license) — Arduino compiles the sketch's `src/` subtree automatically; no install needed.

### arduino-cli (safe to use — caused a BSOD once but was updated 2026-04-09)

Binary is **not on PATH**: `C:\Users\simon\Downloads\arduino-cli_1.5.0_Windows_64bit\arduino-cli.exe`.

```powershell
$cli = "C:\Users\simon\Downloads\arduino-cli_1.5.0_Windows_64bit\arduino-cli.exe"
& $cli compile --fqbn esp32:esp32:esp32s3 `
  --board-options "PartitionScheme=huge_app,PSRAM=opi" Robo-Mower-V2
& $cli upload -p COM47 --fqbn esp32:esp32:esp32s3 Robo-Mower-V2
```

**Upload port is COM47** (the mower; present only when powered + plugged in — check `[System.IO.Ports.SerialPort]::GetPortNames()` first). Don't blindly flash COM3/15/20 (other devices). Build size ≈ 46 % flash / 48 % RAM (Clipper2 added ~270 KB).

### Build-time switches (`config.h:22–34`)

- `TEST_MODE 1` — runs the geometry unit-test suite from `setup()` then halts (pass/fail on USB Serial @115200). 0 for normal operation.
- `DEBUG_SERIAL 1` — USB Serial debug output, serial commands, 2 Hz JSON telemetry. **At 0 (production default) all USB Serial I/O is a no-op** — serial commands won't respond. Does not affect CRSF or BLE.
- `BENCH_TEST_NO_VESC 1` — suppresses cutting-monitor fault transitions for bench testing without VESCs. Never 1 in field firmware.

### Geometry unit tests — host build (no hardware)

`host_test/` compiles `geometry.cpp` + `geometry_test.cpp` for Windows via Zig's bundled clang (`winget install zig.zig`), plus extra concave / chained-inset scenarios in `host_test/main.cpp`:

```powershell
cd host_test
& "$env:LOCALAPPDATA\Microsoft\WinGet\Packages\zig.zig_*\zig-*\zig.exe" c++ -std=c++17 -O1 -o geomtest.exe main.cpp geometry.cpp geometry_test.cpp
.\geomtest.exe   # exit code = number of failures
```

`host_test/geometry.cpp` / `geometry_test.cpp` are COPIES of the Robo-Mower-V2 sources with `config.h` swapped for `host_config.h` — re-copy after editing the originals:
`(Get-Content Robo-Mower-V2\geometry.cpp -Raw) -replace '#include "config.h"', '#include "host_config.h"' | Set-Content host_test\geometry.cpp -Encoding utf8` (same for `geometry_test.cpp`, which also drops `#include <Arduino.h>`).

**Spiral/Clipper host tests** (compile the in-tree Clipper2 + the real wrapper, no copies):
```powershell
$cl = "../Robo-Mower-V2/src/clipper2"
& $zig c++ -std=c++17 -O1 "-I../Robo-Mower-V2" "-I$cl" -o wrap.exe `
  clipper_wrapper_test.cpp ../Robo-Mower-V2/clipper_offset.cpp geometry.cpp `
  "$cl/clipper.engine.cpp" "$cl/clipper.offset.cpp" "$cl/clipper.rectclip.cpp"
.\wrap.exe
```
- `clipper_spiral_test.cpp` — direct `InflatePaths` spiral on the deep-notch garden (no spikes/inversion).
- `clipper_wrapper_test.cpp` — real `offsetPolygonClipper` wrapper + from-perimeter spiral → reaches centre, in-bounds.
- `clipper_multiregion_test.cpp` — dumbbell/pinch shape → both lobes covered (the "missed arm" fix).
- `spiral_test.cpp` / `spiral_dump.cpp` — older hand-rolled-inset probes (kept for reference).

## Repository Notes

- **Remote:** `https://github.com/SimonRafferty/RoboMower`. `Robo_Mower_2/` is its **own standalone git repo**, but is physically **nested inside** the home-directory `MAKER-Code` monorepo (`C:/Users/simon/.git`) — the nested `.git` in `Robo_Mower_2/` is the one that matters. Don't auto-push; push only when asked.
- Working directory is `Robo_Mower_2/`; the Arduino sketch is in `Robo-Mower-V2/`.
- Docs: `telemetry.md` (CRSF payload layouts), `manual.md` (operating guide), `README.md` (full ops manual: wiring + tuning).

## Known Issues & TODO (2026-06-14)

**Coverage planning (spiral): working.** Covers concave gardens, pinched side-arms, uniform spacing perimeter-inward, ring bridges, centre plunge. Residual: a tiny uncut patch at each area's centre (single centroid plunge) — could upgrade to a short cross if needed.

**Heading: BNO055 absolute fusion + GPS-trimmed offset.** Heading comes from the BNO055 (NDOF, tilt-compensated) plus an offset slowly trimmed from the GPS travel chord on straight RTK runs and persisted to NVS. No wheel-odometry heading, no gyro-pivot hack, no GPS-lock blend, no AUTO bootstrap creep. AUTO requires `imu_heading_is_confident()` (BNO sys+mag calibration) and pauses if confidence is lost — there is **no** fallback to a lesser heading source (recalibrate or fix the mounting instead). **Unverified on hardware** until the BNO is wired (see the implementation plans under `docs/superpowers/plans/`).

**Disabled / vestigial:**
- **AUTO fault responses OFF** (`AUTO_FAULT_RESPONSES_ENABLED 0`): overload→RETRACE, stall→BOG_RECOVERY, obstacle→OBSTACLE_AVOID, follower-stall/slip all gated off. Blade is **RC-only** in AUTO. Only **tilt** and **collision** remain (plus always-on perimeter-breach / VESC-silence). Re-enable when detectors are trustworthy.
- **Collision detection is DISABLED** — re-seated on the BNO055 linear accelerometer (fresh NVS baseline `baseline_v2`, capture logging via `COLL base=… jolt=…`) but left off pending a real normal-driving baseline; re-enable when a trustworthy threshold is known.
- **Blade load %** is display-only (fixed `BLADE_CURRENT_LIMIT_A` reference); auto-cal still writes NVS `blade_cal` but its value is unused.
- **Nav boundary / working area** still derived/stored/displayed but **unused for navigation** (the perimeter is the centre-limit; spiral & breach use it directly).
- **Battery auto-return** removed (operator decision; revisit when the mower can self-charge).

**Latent traps:**
- `perimeter_close_track()` uses a point-COUNT window (dense-perimeter assumption) — could drop a corner on a sparse perimeter. Suspect it if a recorded perimeter loses a corner.
- After the `mow_cfg` v10→v11 bump, **saved MowerConfig reset to defaults** — operator must re-enter footprint W×L, track width, and tuning (perimeter & odocal survive in their own keys).
- `BLADE_CURRENT_LIMIT_A` (config.h) must match the VESC Tool "Motor Current Max".

**Pivot/turn tuning** (`PIVOT_*`, `NODE_*` in config.h) is still rough — revisit on hardware now that the BNO055 supplies a trustworthy heading.

## Architecture

### Module overview (roughly dependency order)

```
config.h            — All #define constants: pins, physical dims, tuning, derived values; DEBUG_SERIAL=0 by default
mower_config.h/.cpp — Runtime-configurable MowerConfig struct; BLE SET_CONFIG updates this
geometry.h/.cpp     — 2D polygon maths (hull, intersect, point-in-poly); hand-rolled inset kept for unit tests only
heading_fusion.h/.cpp — pure (host-testable) helpers: GPS straight-segment gate, wrap-safe offset EMA, heading compose
clipper_offset.h/.cpp — robust polygon inset via vendored Clipper2 (src/clipper2/); spiral rings + nav/working-area insets
src/clipper2/       — vendored Clipper2 library (Boost license; the offset engine libslic3r/OpenMower use)
nvs_storage.h/.cpp  — All NVS persistence (Preferences + blob API); CRC32 on every blob
crsf_input.h/.cpp   — CRSF RC frame parse (420 kbaud, Serial2); FreeRTOS task on Core 1
crsf_telemetry.h/.cpp — 5-frame telemetry rotation to TX16S (FlightMode, GPS, Battery, MOWER_STATUS 0x80, Attitude)
vesc_can.h/.cpp     — TWAI/CAN driver; SET_DUTY / SET_CURRENT / SET_RPM; STATUS_1–5 reception
rtk_gps.h/.cpp      — DFRobot RTK LoRa GPS via DFRobot_RTK_LoRa library (request/response, Serial1); ENU origin management; Core 0 task
imu.h / imu_bno055.cpp — BNO055 I2C @100 Hz (NDOF fusion); tilt-compensated absolute heading + tilt/pitch/roll + linear accel; feeds EKF + collision + tilt. Calibration profile + heading offset persisted to NVS.
collision_detect.h/.cpp — Adaptive-baseline jolt detector; direction (fwd/side/rear) classification
servo_output.h/.cpp — Cut-height servo; LEDC PWM; fixed 1000–2000 µs calibration (#defines, no NVS cal)
obstacle_map.h/.cpp — Grid of detected obstacle positions; routes around them
ekf_localiser.h/.cpp — 4-state EKF [x, y, θ, v]; position from GPS + wheel-odometry dead-reckoning; heading = BNO055 absolute fusion + GPS-trimmed offset; publishes heading events for odo_calib; ENU frame
perimeter.h/.cpp    — Polygon record/save/load; derives nav boundary and working area
cutting_monitor.h/.cpp — Blade current monitoring; load fractions; CUTTING_STALLED / OVERLOADED / OBSTACLE_SUSPECTED
coverage_planner.h/.cpp — Concentric inward spiral planner (replaced boustrophedon 2026-06-13); insets the perimeter ring-by-ring to the centre
node_follower.h/.cpp — Node-to-node follower (replaced pure_pursuit 2026-06-13): drives to the sparse node, pivots on the spot to aim at the next, P-controller drive arc; defines `WheelCmd`
odo_calib.h/.cpp    — GPS-referenced self-cal of distance `scale` + kinematic `track_m` (own NVS ns "odocal"); manual+auto; feeds EKF/odometry
heading_controller.h/.cpp — Gyro-based heading stabilisation; PD correction used by manual drive
wheel_duty_ramp.h/.cpp — AUTO drive control: fixed duty step (0.02/tick @10 Hz) toward target speed via SET_DUTY
bog_recovery.h/.cpp — Progressive height-raise recovery when stalled; eRPM bogdown detection
retrace.h/.cpp      — Strip re-trace at max height when overloaded
battery_monitor.h/.cpp — Battery voltage from VESC CAN STATUS_5 (blade VESC, ID 3); warn/low thresholds
safety.h/.cpp       — VESC-silence, blade-stale, perimeter breach watchdogs; FreeRTOS task on Core 1
state_machine.h/.cpp — Top-level 11-state FSM @10 Hz; manual drive; BLE command handler; LEDs; serial commands; 2 Hz JSON telemetry
ble_server.h/.cpp   — BLE GATT server (4 chars: TELEM, MAP, CMD, STATUS); fragment reassembly; queues JSON to Core 1
sys_log.h/.cpp      — Circular in-memory log for diagnostic messages
```

### FreeRTOS core assignment

| Core | Tasks |
|------|-------|
| Core 0 | `rtk_gps` task, `ble_server` task, VESC CAN RX (TWAI driver) |
| Core 1 | `loop()` → `state_machine_update()` @10 Hz, `crsf_input` task, `safety` watchdog task |

Shared state is accessed via `volatile` atomics or `state_machine_get_*()` getters — never directly from the other core. EKF state is protected by a FreeRTOS mutex (Core 0 writes, Core 1 reads).

### State machine (11 states)

`STATE_INIT` → `STATE_IDLE` → `STATE_MANUAL` / `STATE_LEARN_PERIMETER` / `STATE_AUTO_MOWING`

Recovery sub-states: `STATE_RETRACE`, `STATE_BOG_RECOVERY`, `STATE_OBSTACLE_AVOID`. Completion: `STATE_AUTO_RETURN` → `STATE_IDLE`.

Operator pause: `STATE_PAUSED` (CH7 switch, GPIO6 latching switch, BLE pause, serial `PAUSE`, or a software safety fault — **perimeter breach** and **tilt limit** trigger it; VESCs stay powered, CAN stop sent). **Blade fault no longer pauses** (blade is RC-only — it warns and drives on).

Hardware fault: `STATE_MOTORS_OFFLINE` (PILZ E-stop fired — VESC power cut; ESP32 runs on supercap).

Other FSM behaviours:
- RC failsafe (no CRSF, no BLE, no fresh BLE drive) only acts while operator-driving: MANUAL/LEARN → IDLE (`state_machine.cpp:1491`). RC is deliberately off during autonomous operation.
- Battery WARNING/LOW → **operator notification only**: TX warning beep (every 60 s) + MOWER_STATUS flags bit 0x20 (flashing banner on the Lua widget) + PWA toast/beep + amber LED overlay. **No blade lockout and no auto-return** (both removed 2026-06-12 — the VESCs handle low-voltage power reduction internally; the operator selects Return).
- IDLE entry runs a non-blocking "look-around" turret animation (`LOOK_*` phases, `state_machine.cpp:1599`).

### Perimeter = STEERING-CENTRE limit (2026-06-13)

The recorded perimeter is the path of the robot's **steering centre** driven to its maximum extent (body against the physical boundary). So the physical fence sits ~one robot **diagonal-radius OUTSIDE** the perimeter — that margin is baked into the recording, and **the steering centre may drive right up to the perimeter.** Consequences: the spiral's outer ring (ring 0) IS the perimeter (no nav-inset subtracted — that would double-count the robot size); breach is measured directly against the perimeter; speed-scaling distance is to the perimeter.

The legacy **Nav boundary** (perimeter inset by `NAV_EXCLUSION_INSET_M`) and **Working area** (inset by `HEADLAND_WIDTH_M`) are still derived/stored/displayed but **no longer used for navigation** (vestigial under the spiral planner).

### EKF localiser

4-state `[x(m), y(m), θ(rad), v(m/s)]` in local ENU (origin set by perimeter upload, else auto-seeded at the first RTK-fixed fix; persisted in NVS).
- **`ekf_predict(v_left, v_right, dt)`** @10 Hz — heading is set directly from the **BNO055 absolute fusion + a GPS-trimmed offset** (`imu_get_heading_fused() + s_hdg_offset`); no integration. Position dead-reckons (`dx=v·sinθ·dt`, `dy=v·cosθ·dt`) between GPS fixes. If the BNO faults (`imu_is_fault()`), heading is **held** (AUTO pauses elsewhere) — there is no odometry/gyro heading fallback.
- **`ekf_update_gps()`** @~1 Hz — position **snaps** to GPS (innovation gate `max(5σ, 5 m)`, cold-start bypass). On a **straight, measurable** RTK segment (`s_hdg_turn_accum < HEADING_STRAIGHT_MAX_TURN_RAD` AND travel `> max(HEADING_FROM_GPS_MIN_DIST_M, HEADING_GPS_DIST_SIGMA_K·σ)`) the GPS travel chord trims the **heading offset** by a slow EMA (`HEADING_OFFSET_TRIM_GAIN`), reverse-corrected when reversing. The offset is persisted to NVS (`imu`/`hdgoff`), so absolute heading is correct at boot and robust to GPS outages. The offset-trim math is the pure `heading_fusion` module (host-tested).
- Reported uncertainty = last GPS σ; heading variance is fixed small (`EKF_HDG_VAR_BNO`) while the BNO is healthy.

### VESC motor control

Drive VESCs (CAN ID 1, 2) are commanded by **duty cycle** (`vesc_set_duty()` → PKT_SET_DUTY); stops use `vesc_set_current(id, 0)`.

**Hardware note:** drive VESCs are HW **v4** (older firmware, no per-message CAN-status selection — verify "Send CAN Status" is enabled in VESC Tool or they broadcast nothing); the blade VESC is HW **v6** with status 1–5 enabled. Closed-loop drive, odometry heading, and slip/bog detection all need drive STATUS_1 (eRPM) frames.

- **Manual drive** (`state_machine.cpp` `drive_manual`): tank-mix of expo/deadband sticks → per-wheel duty, with a gyro heading-stabilisation PD overlay (`heading_controller_compute()`) and a slew-rate limit (`MANUAL_DUTY_RAMP_PER_S`). Fully open-loop — works without VESC telemetry. BLE/WebUI drive sliders are honoured only while the RC TX is off (failsafe); RC always wins when present.
- **AUTO drive** (`wheel_duty_ramp.cpp`, called from `node_follower.cpp`): steps each wheel's duty by `DUTY_RAMP_STEP` (0.02) per 10 Hz tick toward the target m/s, using live VESC eRPM (scaled by odo_calib via `vesc_erpm_to_velocity_scaled()`) as feedback; clamped to `manual_max_duty`. **Staleness guard:** if a drive VESC's status was never seen or is older than `DUTY_FEEDBACK_STALE_MS` (500 ms), the ramp does NOT integrate (a frozen 0 reading would wind duty to max — runaway); it falls back to bounded open-loop duty and logs once. (The legacy `wheel_speed_pi` PI-current loop was deleted 2026-06-13.)
- **Blade** (CAN ID 3): **RPM control** — keepalive sends `SET_RPM` (target eRPM = `blade_target_rpm × pole_pairs`) every 100 ms; the VESC's internal current-limited RPM PID gives a smooth ~2 s spin-up. A firmware duty ramp was tried and **reverted 2026-06-12** (jerky steps, bad with CAN dropouts) — don't reintroduce it. Disarm = zero-current (freewheel, no regen). BLADE_FAULT requires `BLADE_FAULT_GRACE_MS` (4 s) after arm before it can fire (`cutting_monitor.cpp`). Telemetry flags bit 0x02 reports the real `s_blade_commanded` state.

### BLE ↔ PWA protocol

The BLE server (`ble_server.cpp`, Core 0) only transports bytes: commands arrive on the `CMD` characteristic as JSON (large payloads fragmented as `{"f":<i>,"n":<total>,"d":"<chunk>"}`), are reassembled, and queued via `state_machine_enqueue_ble_cmd()`. **Parsing/dispatch happens on Core 1 in `state_machine.cpp`** (`handle_ble_command()`).

The PWA (`robomower-pwa/robomower.html`) is a single self-contained HTML file with four tabs: Dashboard (status + config), Map (Leaflet + leaflet-geoman perimeter editing), Controls (drive sliders, mode buttons, height), Diagnostics (live sensor data). Web Bluetooth — Chrome on Android only.

### CRSF telemetry rotation

Five frames: `FLIGHT_MODE (0x1E)` → `GPS (0x02)` → `BATTERY (0x08)` → `MOWER_STATUS (0x80)` → `ATTITUDE (0x14)` (`crsf_telemetry.cpp:53` `s_frame_order[]`). Snapshot @2 Hz from the state machine; frames dispatched by the CRSF RX task on inter-packet gaps. See `telemetry.md` for payload layouts.

### Physical constants and tuning

All in `config.h` (compile-time `#define`) and mirrored in `mower_config.h` (runtime `MowerConfig`, updatable via BLE). The runtime struct takes precedence after first boot.

**Robot dimensions model (2026-06-13):** the footprint is an **overall bounding box** — `footprint_width_m` × `footprint_length_m` — kept *separate* from the **steering track** `track_width_m` (track centre-to-centre, for odometry). For tracks of width *w*, `track_width = overall_width − w`. The old per-edge offsets (`robot_front/rear/left/right`) and `chassis_length_m` were **removed** (NVS `mow_cfg` v10→**v11**; saved MowerConfig resets to defaults on first boot — perimeter & odocal survive in their own keys). `chassis_width_m` → renamed `track_width_m`.

Key derived constants (`config.h` + `mower_config.cpp`):
- `NAV_EXCLUSION_INSET_M = 0.5·√(footprint_width² + footprint_length²) + 0.05` — **half the footprint diagonal** (the radius the corners sweep when pivoting on the spot), not half the width. `mower_config_nav_inset_m()`.
- `HEADLAND_WIDTH_M = max(CUT_WIDTH_M×1.5, footprint_length×0.5 + STRIP_OVERLAP_M, blade_fwd_reach + STRIP_OVERLAP_M)` — vestigial under the spiral planner.

## Project Map — Where to Find What

Quick-lookup by topic. Line numbers are approximate; function names are stable.

### GPIO pin assignments
All in `config.h` (~lines 57–100). Key pins: CAN TX/RX GPIO2/1 · servo GPIO5 · pause switch GPIO6 · external LED strip GPIO7 · IMU I2C SDA/SCL GPIO8/9 (BNO055 will share this bus at 100 kHz) · GPS Serial1 RX/TX GPIO10/14 · CRSF Serial2 RX/TX GPIO11/12 · onboard NeoPixel GPIO48.

### State transitions
- `state_machine_update()` entry: `state_machine.cpp:1334`; main FSM switch `:1548` (cases `:1551–2680`); transition helper `:211` `transition_to(next, event_latch)`.
- Pre-FSM safety overrides (~`:1500–1650`): VESC silence → MOTORS_OFFLINE · perimeter breach → PAUSED · RC failsafe → IDLE (operator-driving states only) · CH7/GPIO6/BLE pause → PAUSED. (Battery LOW is **notification-only**.)
- In AUTO_MOWING: **tilt → PAUSED** and **collision → OBSTACLE_AVOID** are the only active fault responses. **BLADE_FAULT just warns and drives on**. OVERLOADED→RETRACE, STALLED→BOG_RECOVERY, OBSTACLE_SUSPECTED→OBSTACLE_AVOID, follower-stall→OBSTACLE_AVOID, follower-slip→BOG_RECOVERY are **all GATED OFF** by `AUTO_FAULT_RESPONSES_ENABLED 0` (each `if (AUTO_FAULT_RESPONSES_ENABLED && obs_armed && …)`).
- The (gated) fault responses are additionally armed only after `OBSTACLE_DETECT_STARTUP_MS` from AUTO entry (`obs_armed`) — blade spin-up looks like a stall.

### RC channel mapping
`crsf_input.h:24–32` — CH1=steer, CH2=throttle, CH3=cut height, CH4=mode (manual/auto/return), CH5=learn perimeter, CH6=arm/blade, CH7=pause (`CRSF_CH_PAUSE` in `config.h:254`), CH8=momentary record point. Switch decode with hysteresis: `state_machine.cpp:1410–1417`. `ch[]` values are in **microseconds** (1000–2000), not raw CRSF units.

### Custom MOWER_STATUS telemetry (CRSF 0x80)
`crsf_telemetry.cpp:292` `sendMowerStatus()` — **19-byte** payload: state, progress, cut height, blade load, fix type, flags (+beep bits 7:6), obstacle count, EKF uncertainty, mowed area, **battery V×100 and heading deg×10 (offsets 15–18, added 2026-06-10 so the Lua widget doesn't depend on EdgeTX `RxBt`/`Yaw` sensor discovery)**. The Lua accepts both 15- and 19-byte payloads.

### EKF localiser (detail in *Architecture › EKF localiser*)
- Prediction (`ekf_predict()`): heading = `imu_get_heading_fused() + s_hdg_offset` each tick (no integration); position dead-reckons. BNO fault → heading held (no fallback).
- GPS update (`ekf_update_gps()`): position snap + innovation gate; on a straight, measurable RTK segment the travel chord trims `s_hdg_offset` (slow EMA, reverse-corrected), persisted to NVS (`imu`/`hdgoff`).
- **Heading confidence:** `imu_heading_is_confident()` (BNO sys+mag calib) gates AUTO start and is monitored continuously; loss → PAUSE.
- **Heading events:** each clean straight-RTK trim publishes a front-facing event (`ekf_get_gps_heading_event()`) consumed by odo_calib. Reset on RESETEKF.
- ENU origin lock: `rtk_gps.cpp` `rtk_gps_set_origin()` — at the first RTK-fixed fix and on perimeter upload via BLE.

### Odometry self-calibration (`odo_calib.cpp`)
- `odo_calib_update()` ticks from the 10 Hz EKF hook in ALL states. Distance `scale` from straight RTK runs (GPS chord / odometry); kinematic `track_m` from turns (Σ(vL−vR)·dt / Δθ_gps). EMA, ≤2 %/update, RTK-only. Stored in its **own NVS ns "odocal"** (keys `scale`, `track`); a `CAL …` line is logged to the PWA. Applied via `vesc_erpm_to_velocity_scaled()` and `odo_cal_track_m()`.
- **AUTO start (heading-confidence gate):** AUTO starts immediately when `imu_heading_is_confident()` (BNO sys+mag calibration). The old ≥2 m perimeter-clearance gate and the straight-creep heading bootstrap are **removed**. If not confident, AUTO PAUSEs and the operator drives slow loops in MANUAL to calibrate the magnetometer (prompted on the TX16S widget and the PWA "Recalibrate compass" button → `RECAL_IMU`). The confidence check is also re-evaluated every tick during AUTO.

### Motor commands
- Drive duty: `vesc_can.cpp` `vesc_set_duty(id, duty)` → PKT_SET_DUTY; stop via `vesc_set_current(id, 0)`.
- Blade SET_RPM: `vesc_can.cpp` `vesc_set_rpm(id, erpm)` → PKT_SET_RPM.
- AUTO duty ramp: `wheel_duty_ramp.cpp:20` `wheel_duty_ramp_compute()` — step `DUTY_RAMP_STEP` (0.02)/tick, called from `node_follower.cpp`.
- Manual heading PD correction: `heading_controller.cpp:31` `heading_controller_compute()`, applied in `drive_manual` at `state_machine.cpp:775`.
- Blade spinup ramp (3 s): `s_blade_ramp_erpm` interpolation in MANUAL `state_machine.cpp:~1737` and AUTO handlers, gated by `s_last_armed`.

### Safety watchdogs (`safety.cpp`, Core 1, 50 ms period)
- Both drive VESCs stale > 2 s → STATE_MOTORS_OFFLINE: `safety.cpp:~190–215`.
- Blade VESC stale (separate, non-fatal for drive): `safety.cpp:217`.
- Perimeter breach > `PERIMETER_BREACH_DIST_M` outside the **PERIMETER** (the steering-centre limit; `safety_set_perimeter()`, not the nav boundary) → hard-stop + pause request: `safety.cpp` (~`:246`).
- Tilt limit (normal vs careful, from MowerConfig) checked in AUTO: `state_machine.cpp:2088`.
- Uncertainty-aware navigation (speed scaling, careful collision multiplier, strip truncation): `state_machine.cpp:2070–2110`.

### Perimeter recording — SPARSE (turn-point nodes, ~10 points)
- **Sparse:** in LEARN mode the operator presses **CH8 at each corner** and one node is recorded on the falling edge (`state_machine.cpp:2072`) via `perimeter_record_point(..., force=true)`, which **bypasses** the 0.2 m distance gate. Edges between corners are straight; nothing densifies them. (The `RECORD_DIST_M = 0.2` gate in `perimeter.cpp` only applies to non-forced calls, which LEARN doesn't use.)
- `perimeter_close_track()` (`perimeter.cpp:221`) uses a point-COUNT window (`PERIMETER_CLOSE_WINDOW`, tuned for dense points) to find the closest start/end pair — a latent dense assumption. For a sparse perimeter the window clamps to 3 nodes; works in practice but could in theory discard a real corner. Watch this if a recorded perimeter loses a corner.
- Recompute nav boundary + working area insets: `perimeter.cpp:448` `perimeter_recompute()`.

### Coverage planning (`coverage_planner.cpp`) — CONCENTRIC SPIRAL (2026-06-13)
- **Replaced the boustrophedon strip planner**, which never planned a *sparse* perimeter cleanly and produced corrupt fan-shaped plans (`PathError.jpg`). All its machinery (scan-line strips, headland/region-outline passes, spur reverses, U-turn transitions, optimal-angle hull scan) is **gone**.
- **The perimeter is SPARSE** (~10 corner nodes joined by straight edges, recorded one at a time via CH8; not densified). The spiral is native to this: each ring is the polygon's sparse vertices; the node follower drives straight edge-to-edge and pivots at corners. Long straight runs between corners are expected, not a fault.
- **Offsetting is done by Clipper2, NOT the hand-rolled `insetPolygon`.** The custom inset spikes at concave (reflex) corners and inverts past the local inradius (no union cleanup), so deep insets of a notched garden produced garbage. Clipper2 does the union cleanup → robust on arbitrary concave gardens. Wrapper: `clipper_offset.h/.cpp` (`offsetPolygonClipper` → all solid sub-polygons; `insetPolygonClipper` → largest single region). Vendored under `Robo-Mower-V2/src/clipper2/` (Boost; flat includes; Arduino compiles `src/` recursively).
- `coverage_planner_plan()`: **ring 0 = the perimeter itself** (the steering-centre limit — NO nav-inset subtracted, so spacing is uniform). **ring k>0 = the perimeter inset by k·step** (`CUT_WIDTH_M − STRIP_OVERLAP_M` = 0.36 m) via `offsetPolygonClipper`, **keeping ALL sub-regions** so a garden that pinches into separate areas (e.g. a neck to a side arm) is fully covered — each area gets its own spiral (`PathError3.jpg`). Rings continue to **`SPIRAL_RING_MIN_AREA_M2` (0.04 m²)** so the spiral reaches the centre. Round-joins are RDP-simplified (8 cm); over-shrink returns empty (loop ends). Caps: `MAX_PLAN_WP` (960) / `MAX_RINGS` (400). All waypoints `mowing=true, headland=true`; no clamp needed (every vertex is inside by construction). `g_headland_wp_end_idx = total`.
- **Breach detection is against the PERIMETER** (`safety.cpp`, `safety_set_perimeter(perimeter_get_perimeter())`): breach when `distanceToNearestEdge(perimeter, pos) < -PERIMETER_BREACH_DIST_M`. One polygon, so it covers all regions incl. pinched-off arms. Speed-scaling `margin` (`state_machine.cpp`) also measures to the perimeter.
- **Nav boundary & working area** are inset via Clipper (`insetPolygonClipper`, largest region) in `perimeter.cpp` / `state_machine.cpp` SEND_PERIMETER so the displayed boundary doesn't spike at notches. Hand-rolled `geometry.cpp` `insetPolygon`/`insetPolygonMulti` remain only for unit tests.
- Validated offline: `host_test/clipper_spiral_test.cpp`, `clipper_wrapper_test.cpp`, `clipper_multiregion_test.cpp` (see *Build › Spiral/Clipper host tests*).
- Waypoint drain: `coverage_planner_get_next()`; resume near position: `coverage_planner_reset_to_nearest(x, y)`. Unreachable-zone diagnostics retained but unused.

### Recovery modes
- **TEMPORARILY DISABLED (2026-06-13):** all AUTO fault responses are gated off by `AUTO_FAULT_RESPONSES_ENABLED 0` (config.h) — see *Known Issues* and *State transitions*. RETRACE/BOG_RECOVERY/OBSTACLE_AVOID are currently unreachable; the detectors fired false recoveries (and a full-speed takeoff via the recovery sub-states). Blade is entirely RC-controlled in AUTO; only **tilt** and **collision** remain (plus always-on perimeter-breach/VESC-silence). Re-enable by setting the flag to 1 once detectors are trustworthy.
- Stall vs overload classification: `cutting_monitor.cpp:83` `assessCuttingCondition()` — STALLED (not moving, load high) → BOG_RECOVERY; OVERLOADED (moving, load high) → RETRACE; OBSTACLE_SUSPECTED (blade commanded, load low, not moving) → OBSTACLE_AVOID.
- Bog recovery: `bog_recovery.cpp:141` `bog_recovery_update()` (dwell → lower deck → forward pass, up to 6 retries).
- eRPM bogdown / wheel slip: `node_follower.cpp` `node_follower_is_slipping()` → BOG_RECOVERY.
- IMU collision spike: `collision_detect.cpp` `collisionDetected()` → OBSTACLE_AVOID at `state_machine.cpp:2198`.

### BLE command handling
- Dispatch: `state_machine.cpp:915` `handle_ble_command()` — routes on the `"cmd"` JSON field (Core 1; `ble_server.cpp` only reassembles fragments and queues).
- SET_CONFIG (update MowerConfig): `state_machine.cpp:1123`.
- SEND_PERIMETER (origin + polygon from PWA): `state_machine.cpp:822` `handle_send_perimeter()`.
- MAP characteristic assembly (perimeter, nav boundary, working area, obstacles): `ble_server.cpp` `ble_server_send_map()`.
- TELEM characteristic (periodic sensor snapshot): `ble_server.cpp` `ble_server_update()`.

### NVS read/write locations
- Perimeter / nav boundary / working area blobs: `nvs_storage.cpp` `nvs_save_perimeter()` / `nvs_save_nav_boundary()` / `nvs_save_working_area()`.
- MowerConfig blob (`mow_cfg`): `mower_config.cpp` `mower_config_set()`.
- Blade calibration float (`blade_cal`): `cutting_monitor.cpp:134`.
- Adaptive collision baseline float (namespace `collision`, key `baseline`): `collision_detect.cpp`, saved every 120 s if changed > 0.02 g.
- GPS ENU origin (`gpsorigin`): `nvs_save_gps_origin()` / `nvs_load_gps_origin()`.

### Blade load calibration
**Load reference (2026-06-13): blade load % = `compensated_current / BLADE_CURRENT_LIMIT_A` (fixed 15 A = VESC motor-current limit, config.h)** — `cutting_monitor.cpp` `cutting_monitor_get_load_fraction()` and the `load` in `assessCuttingCondition()`. Previously it divided by the auto-calibrated `g_blade_max_A`, whose P90 captured the **idle** current (~7 A) as 100 %, so idle read ~105 % and falsely crossed the 0.75 HIGH threshold. With 15 A, idle (~7.5 A) reads ~50 % and HIGH sits at 11.25 A. `BLADE_CURRENT_LIMIT_A` **must match VESC Tool "Motor Current Max".**

The auto-calibration still runs and writes the `blade_cal` NVS float, but `g_blade_max_A` is **no longer used** for the load fraction or thresholds (vestigial until the detectors are re-enabled). `cutting_monitor.cpp:117` `updateCalibration()` — P90 of blade-current samples over the first 3 s of AUTO_MOWING; min valid 2.0 A; voltage-compensated against nominal 54.6 V (13S × 4.2 V).

### 2 Hz JSON telemetry (USB Serial, requires DEBUG_SERIAL=1)
`state_machine.cpp:663` `emit_telemetry()` — fields: `t, state, x, y, hdg, fix, sat, vel, hprog, sprog, cutH, obs, unc, bladeRPM, bladeA, bladeLoad, cutStatus, battV, battState, collBase`.

### LED patterns
Pattern enum: `state_machine.h:55`. Per-state colours (`state_machine.cpp`): INIT/IDLE=blue · MANUAL=green · LEARN=orange · AUTO/RETRACE/BOG=red · PAUSED=yellow. GPS overlay (RTK-fixed=green, float=orange, other=red): `:348` `showLedsWithGps()`. Battery warning amber flash every 2 s: `:383` `applyBatteryWarningOverlay()`. Onboard LED = GPIO48 `s_leds[0]`; external strip = GPIO7 `s_leds[1+]`.

## NVS Key Registry

**Warning:** changing the NVS namespace or renaming keys discards all saved config the user entered (perimeter, calibration, robot dimensions). Always warn the user before bumping the `mow_cfg` key or clearing the `mower` namespace.

Namespace `"mower"` (these are the literal NVS key strings):
- `perim`, `navpoly`, `workpoly` — polygon blobs with CRC32 (perimeter / nav boundary / working area)
- `gpsorigin` — ENU origin (lat/lon + CRC32; `GpsOriginNvs` is 24 bytes)
- `estops` — circular E-stop log (20 × 44 bytes)
- `blade_cal` — float; auto-calibrated blade reference current (A)
- `mow_cfg_v11` — MowerConfig blob. **Bumped v10→v11 on 2026-06-13** (footprint W×L box + `track_width_m`, removed `robot_*`/`chassis_length_m`); the `sizeof` guard rejects the old blob, so saved MowerConfig resets to defaults — operator re-enters dimensions/tuning.

Namespace `"imu"`: `bnocal` — 22-byte BNO055 calibration profile blob (auto-saved when fully calibrated); `hdgoff` — float; GPS-trimmed heading offset (rad), `heading = BNO_fused + hdgoff`.
Namespace `"collision"`: `baseline_v2` — float; adaptive collision baseline (g) for the BNO linear-accel feed (the old `baseline` key from the previous IMU is abandoned).

Namespace `"odocal"` (kept apart from `mow_cfg` so manual config is untouched):
- `scale` — float; GPS-calibrated distance multiplier on wheel odometry (default 1.0)
- `track` — float; GPS-calibrated kinematic track width (m, default = nominal `track_width_m`)

(The old `servo_min_us` / `servo_max_us` keys are gone — the cut-height servo uses fixed 1000–2000 µs `#define` calibration in `servo_output.cpp`.)

## Serial Commands (115200 baud, USB — only when `DEBUG_SERIAL 1`)

`STATUS`, `PERIMETER`, `NAVBOUNDARY`, `WORKINGAREA`, `PLAN`, `GRID`, `UNREACHABLE`, `EKFSTATE`, `CALDUMP`, `OBSTACLES`, `ERRORS`, `CLEARPERIM [CONFIRM]`, `RESETEKF`, `CALHEIGHT TEST <mm>`, `PAUSE`. Handler: `state_machine.cpp` `state_machine_handle_serial()` (~line 466). Firmware also emits 2 Hz JSON telemetry automatically.

## VESC CAN IDs

| ID | Motor | Control mode |
|----|-------|-------------|
| 1 | Left drive | Duty (SET_DUTY); stop via SET_CURRENT 0 |
| 2 | Right drive | Duty (SET_DUTY); stop via SET_CURRENT 0 |
| 3 | Blade (Gtech CLM021) | RPM (SET_RPM) |

CAN baud: **250 kbit/s** (`CAN_BAUD_RATE` in `config.h`). Battery voltage is read from the **blade VESC (ID 3) STATUS_5** packet (changed 2026-06-07 from drive VESC ID 1, which on HW v4 may not broadcast STATUS_5). No ADC hardware needed.

### Field diagnostics (BLE — no bench/serial needed)
- **Boot telemetry self-test:** first 8 s after boot, the TX16S battery/blade-load/heading sweep synthetic ramps (40→55 V, 0→100 %, 0→360°). Sweep visible = radio/Lua path good; frozen afterwards = CAN/decoding side.
- **System log over BLE:** the `sys_log` ring (30 entries) is included in the STATUS JSON (`log` array) and shown in the PWA — state transitions, AUTO denial reasons (with position + distance), plan results, blade on/off, fault triggers, and a 5 s `CAN L[a.. e..] R[..] B[..] V.. ok..` raw VESC RX dump (a = ms since last STATUS frame; a-1 = never received).
- **PWA Diagnostics tab:** per-VESC `dt` (status age), eRPM, current, plus `vB.cmd` (blade commanded; `vB.lock` retained in JSON but always 0). "Plan Test" = coverage-planner dry run (BLE `PLAN_TEST`); "Refresh Log" re-fetches STATUS.
