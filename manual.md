# RoboMower — User Manual

## Contents

1. [System Overview](#1-system-overview)
2. [RC Transmitter Layout](#2-rc-transmitter-layout)
3. [WebUI (Phone App)](#3-webui-phone-app)
4. [First Boot Checklist](#4-first-boot-checklist)
5. [Teaching the Perimeter](#5-teaching-the-perimeter)
6. [Starting a Mow](#6-starting-a-mow)
7. [Manual Driving](#7-manual-driving)
8. [Pause and Resume](#8-pause-and-resume)
9. [Emergency Stop](#9-emergency-stop)
10. [Cut Height](#10-cut-height)
11. [Battery](#11-battery)
12. [LED Status Reference](#12-led-status-reference)
13. [Configuration (WebUI)](#13-configuration-webui)
14. [Serial Debug Commands](#14-serial-debug-commands)
15. [Troubleshooting](#15-troubleshooting)

---

## 1. System Overview

RoboMower is a fully autonomous lawnmower controlled by an ESP32-S3 with RTK GPS positioning. It can be operated using a RadioMaster TX16S radio transmitter or a phone via Bluetooth (BLE).

**Key rule: RC transmitter and WebUI are mutually exclusive.** When the transmitter is switched on, the WebUI drive controls are locked out. Switch the transmitter off to use the phone.

**Hardware summary**

| Component | Role |
|-----------|------|
| ESP32-S3 DevKitC-1 (N16R8) | Main controller |
| Quectel LC29H + NTRIP | RTK GPS positioning (~5–10 cm accuracy) |
| RadioMaster TX16S + ER8 | RC control and telemetry |
| 3× VESCs (IDs 1, 2, 3) | Left drive, right drive, blade motor |
| PILZ safety relay | Hardware E-stop (cuts 48V bus) |
| 48V LiPo battery | Main power |
| Supercapacitor backup | Keeps ESP32 running for minutes after PILZ fires |

---

## 2. RC Transmitter Layout

All controls assume a RadioMaster TX16S in Mode 2 (throttle on the left stick). Assign switches as below — the exact physical switch is your choice as long as the channel numbers match.

| RC Channel | Function | Switch type | Position mapping |
|-----------|----------|-------------|-----------------|
| **CH1** | Steering | Left stick L/R | Left = turn left, Right = turn right |
| **CH2** | Throttle | Right stick U/D | Up = forward, Down = reverse |
| **CH3** | Cut height | Knob or slider | Low = minimum height, High = maximum height |
| **CH4** | Mode | 3-position switch | **Down** = MANUAL · **Centre** = AUTO · **Up** = AUTO+RETURN |
| **CH5** | Learn (perimeter record) | 2-position switch | Up (active) = recording on, Down = off |
| **CH6** | Arm (blade enable) | 2-position switch | Up = ARMED (blade allowed), Down = DISARMED |
| **CH7** | Pause / Soft E-stop | Momentary or latching button | Press/hold = pause all motion |
| **CH8** | Store point | Momentary button | One press = record current GPS position as a perimeter waypoint |

### CH4 Mode switch in detail

| CH4 position | State machine mode | What happens |
|--------------|--------------------|-------------|
| Down | MANUAL | Operator drives with sticks; blade runs if CH6 armed |
| Centre | AUTO | Autonomous mowing (requires valid perimeter) |
| Up | AUTO + RETURN | Stops mowing and navigates back to the start point |

### Telemetry on the TX16S

The mower transmits five CRSF telemetry frames back to the transmitter:
- **Flight mode** — shows current state name (INIT, IDLE, MANUAL, AUTO, etc.)
- **GPS** — position and speed (visible on EdgeTX GPS screen)
- **Battery** — voltage and estimated remaining %
- **Attitude** — heading
- **Mower Status (0x80)** — custom frame with headland/strip progress, blade load, fix quality, obstacles, area mowed. Read with a Lua widget.

---

## 3. WebUI (Phone App)

The WebUI is a single HTML file (`robomower-pwa/robomower.html`). Open it in **Chrome on Android** — other browsers do not support Web Bluetooth.

> The phone controls only activate when the RC transmitter is **switched off** (failsafe active). A yellow banner at the top of the Controls tab says "RC Transmitter Active" and locks the buttons when the transmitter is on.

### Connecting

1. Open `robomower.html` in Chrome on Android.
2. Make sure the mower is powered on and the LED is showing a stable pattern (not INIT flashing).
3. Tap **Connect** in the top-right corner.
4. Select **RoboMower** from the Bluetooth device list.
5. The dot next to the Connect button turns green and shows **Connected**.

### Tabs

| Tab | Purpose |
|-----|---------|
| **Dashboard** | Live status cards: state, GPS fix, battery, blade load, position uncertainty, session area. Also the Configuration panel (collapsed by default). |
| **Map** | Leaflet map showing the perimeter, planned mowing strips, GPS position, and coverage progress. Load/save/edit perimeter here. |
| **Controls** | All operational buttons: mode, arm, learn, drive sliders, cut height. |
| **Diagnostics** | Detailed real-time data from all sensors and subsystems. Only polls while the tab is open. |

### Controls tab — buttons

| Button | Function | Notes |
|--------|----------|-------|
| **Manual** | Switch to MANUAL state | RC must be off |
| **Auto** | Switch to AUTO_MOWING state | Requires valid perimeter; mower must be inside it |
| **Return** | Switch to AUTO_RETURN | Navigates to perimeter start point |
| **Learn** | Toggle perimeter learning on/off | See §5 |
| **Store Pt** | Record current GPS position as a waypoint | Only active during Learn mode |
| **Start Blade / Stop Blade** | Toggle blade arm | RC must be off. Only works in MANUAL or IDLE |
| **Pause / Resume** | Toggle BLE pause | Equivalent to pressing the CH7 button |

### Drive sliders

Two sliders replace the RC sticks when the transmitter is off:

- **Forward/Reverse slider** (left) — push up for forward, down for reverse. Returns to zero when released.
- **Left/Right slider** (right) — steer left or right. Returns to zero when released.

Sliders send commands continuously while held. The mower coasts to a stop within 2 seconds if the connection drops.

### Map tab — perimeter tools

| Button | Function |
|--------|---------|
| **Load from Mower** | Download the perimeter currently stored on the mower |
| **Send to Mower** | Upload the perimeter shown on the map to the mower |
| **Load File** | Load a perimeter JSON file from your phone |
| **Save File** | Save the current map perimeter as a JSON file |
| **Centre on GPS** | Pan the map to the mower's last known position |
| **Satellite** | Toggle between street map and satellite imagery |

You can draw and edit the perimeter directly on the map using the leaflet-geoman draw tools (pencil icon on the map). After editing, tap **Send to Mower** to upload it.

---

## 4. First Boot Checklist

### Before power-on

- [ ] VESCs configured: CAN IDs set (Left=1, Right=2, Blade=3), baud 500 kbit/s, status broadcasts enabled. See README §6.
- [ ] NTRIP client running on a nearby device and corrections reaching the LC29H.
- [ ] 48V battery connected. PILZ relay in RUN state (green light).
- [ ] RC transmitter on, CH4 = MANUAL, CH6 = DISARMED, CH7 = not pressed.

### Power-on sequence

1. **Power on the ESP32** (supercapacitor supply or direct 5V).
2. The LED pulses **blue** slowly — this is the **INIT** state.
3. **Place the mower on flat ground and hold it completely still** for ~3 seconds while the gyro bias is collected. The LED shows blue throughout.
4. The mower waits for:
   - A valid RC signal (ER8 receiver linked to TX16S), AND
   - A GPS ENU origin (either loaded from NVS from a previous session, or the GPS must acquire a fix)
5. Once both are ready the LED changes to a slow **blue single blink** = **IDLE**.
6. On first use there is no stored origin. The GPS will auto-seed from the first RTK-quality fix. This may take 1–5 minutes if the NTRIP is slow to provide corrections.

---

## 5. Teaching the Perimeter

The perimeter defines the outermost boundary of the garden — the blade edge should follow this line exactly. Navigate the **blade edge** (not the chassis edge, not the antenna) along the desired outer cut line.

There are three ways to define the perimeter:

### Method A — Drive with RC (recommended for first setup)

1. CH4 = **MANUAL**, CH6 = **DISARMED**.
2. Flip **CH5** to active. The LED changes to fast green/amber flash = **LEARN**.
3. Drive slowly around the garden boundary. Points are recorded every 0.2 m of travel.
   - **Green fast flash** = RTK fixed (best accuracy ~2 cm)
   - **Amber fast flash** = RTK float (still good, ~5–15 cm)
4. When you return within 3 m of the start point the polygon closes automatically.
5. Flip **CH5** off to save. The LED flashes **white 3×** to confirm success, then returns to IDLE.

If the save fails (too few points, self-intersection, polygon too small), the LED returns to its previous pattern without the white flash. Check the serial output for the reason.

### Method B — Store individual points with RC + CH8

For precise corners or when driving continuously is difficult:

1. Enter LEARN mode as above (CH5 active).
2. Position the mower at each corner, then press the **CH8** momentary button to store that point.
3. When finished, flip CH5 off to close and save.

### Method C — Draw on the WebUI map

1. Open the WebUI Map tab.
2. Use the draw tools to trace the perimeter directly on the satellite image.
3. Tap **Send to Mower** to upload. The mower derives the nav boundary and working area automatically.

You can also combine methods: record a rough perimeter by driving, tap **Load from Mower** in the WebUI, edit individual waypoints on the map to fix inaccurate sections, then **Send to Mower** again.

### After saving

The firmware automatically computes:
- **Navigation boundary** — inset from perimeter (keeps chassis inside)
- **Working area** — inset further (where mowing strips are planned)
- **Mowing plan** — boustrophedon strips at the optimal angle

The WebUI map shows all three layers after a successful upload or load.

---

## 6. Starting a Mow

### Via RC

1. Mower in **IDLE** state (blue single blink).
2. Confirm GPS is green (RTK fix). If amber, the mow can still start but boundary accuracy will be slightly lower near the perimeter.
3. Set **CH4 = AUTO**.
4. Set **CH6 = ARMED** (blade on).
5. The mower enters AUTO_MOWING: LED = **orange double blink**.

The mowing plan runs headland passes first (perimeter loops), then boustrophedon strips covering the interior.

### Via WebUI

1. Turn off the RC transmitter (the "RC Transmitter Active" banner should disappear).
2. In the Controls tab, tap **Start Blade** to arm the blade.
3. Tap **Auto** to start mowing.

### During mowing

- **Orange double blink** = mowing, all well.
- **Yellow double blink** = RTK degraded to float — navigation continues but with wider perimeter margin.
- The Dashboard shows headland progress (%), strip progress (%), blade load, and EKF position uncertainty.
- The Map tab shows the planned route and which strips have been mowed.

### Changing cut height mid-mow

- **RC**: Rotate the CH3 knob. The servo moves immediately; the new height is used for subsequent strips.
- **WebUI**: Drag the Cut Height slider in the Controls tab.

### Completion

When all strips are finished, the mower enters **AUTO_RETURN** and navigates back to the perimeter start point, then enters **IDLE**. The blade stops automatically.

You can also command a manual return at any time:
- **RC**: Flip CH4 to UP (AUTO+RETURN)
- **WebUI**: Tap **Return**

---

## 7. Manual Driving

### Via RC

1. CH4 = **MANUAL**. The LED turns solid **green**.
2. Drive with left stick (steering) and right stick (throttle).
3. CH6 = ARMED to run the blade. CH6 = DISARMED to stop it.

Heading stabilisation is active: if you drive straight without steering input, the mower holds its heading using the gyro.

### Via WebUI (RC must be off)

1. Ensure the transmitter is off.
2. In the Controls tab, tap **Manual**.
3. Use the Forward/Reverse and Left/Right sliders to drive.
4. Tap **Start Blade** to arm the blade if needed.

The sliders return to zero when released, bringing the mower to a gradual stop.

---

## 8. Pause and Resume

Pause can be triggered from any active state and holds everything in place — position, plan progress, and blade state are all preserved.

### Pausing

| Method | Action |
|--------|--------|
| RC CH7 button | Press/hold the pause button |
| Physical switch | Flip the GPIO6 latching switch on the chassis to GND |
| WebUI | Tap the **Pause** button in the Controls tab |

All three sources are OR'd: any one of them pauses the mower. To resume, all active sources must be cleared.

The LED shows **purple fast flash** while paused.

### Resuming — operator-initiated pause

1. Release the CH7 button, flip the GPIO6 switch open, or tap **Pause** again in the WebUI.
2. Set CH4 = AUTO (or tap **Auto** in WebUI).
3. The mower verifies the perimeter is still valid and the mower is inside it, then resumes from exactly where it stopped.

### Resuming — event-triggered pause

If the mower paused itself due to a safety event (perimeter breach, blade fault, tilt exceeded), an **event latch** is set and the mower will not resume automatically even when all pause inputs are cleared. This requires a deliberate acknowledgement:

1. **Activate the pause input** — press CH7 or tap **Pause** in the WebUI (pause goes active).
2. **Deactivate the pause input** — release CH7 or tap **Pause** again (pause goes inactive).
3. The latch clears on this falling edge and the serial output shows `Event latch cleared`.
4. Now set CH4 = AUTO or tap **Auto** to resume normally.

The LED pattern changes from fast flash back to slow flash once the latch is cleared, indicating it is now safe to resume.

---

## 9. Emergency Stop

### Hardware E-stop (PILZ)

The red mushroom button on the mower chassis cuts the 48V bus via the PILZ safety relay. This immediately and physically removes power from all three VESCs. The ESP32 continues running on the supercapacitor for several minutes.

- **Press the mushroom button** to cut power.
- The firmware detects VESC CAN silence and enters **STATE_MOTORS_OFFLINE**: LED = red/amber alternating.
- All session state is saved to NVS (position, waypoint index, cut height, mowed area).

**To recover:**
1. Check that the hazard has been resolved.
2. Unlock the PILZ button (twist or pull to release, depending on model).
3. The VESCs come back online. The ESP32 detects their CAN STATUS frames.
4. The firmware restores the saved session and offers to resume from the waypoint where it left off.
5. Set CH4 = AUTO (or tap **Auto**) to resume mowing.

> **The PILZ is hardware-only. There is no software command to trigger it.**

### Soft pause / stop via RC

The **CH7 pause button** is the recommended soft stop during normal operation:
- Press CH7 to pause immediately (drives and blade stop, position held).
- Release CH7 and set CH4 = AUTO to resume.

### What happens on RC failsafe (transmitter signal lost)

If the transmitter signal is lost for >500 ms while in MANUAL or LEARN mode:
- Drive motors stop immediately.
- State transitions to IDLE.
- The WebUI can take over if connected.

In AUTO_MOWING, RC failsafe causes an immediate stop only if the WebUI is also not connected (both control sources gone).

---

## 10. Cut Height

The cutting deck is raised and lowered by a servo. The range must be calibrated once (see §4 first boot).

**Height range:** 35 mm (min) to 90 mm (max) — adjust `cut_height_min_mm` and `cut_height_max_mm` in the WebUI Config panel if your deck has a different range.

| Method | How |
|--------|-----|
| RC | Rotate the CH3 knob. The height changes continuously as you turn. Minimum change threshold is ~1 mm to filter noise. |
| WebUI | Drag the **Cut Height** slider in the Controls tab. Shows height in mm. |
| Serial | `CALHEIGHT TEST <mm>` (e.g. `CALHEIGHT TEST 50`) moves the servo to verify the height. |

During AUTO_MOWING, the firmware automatically raises the deck if bog recovery or retrace is triggered, and lowers it again step-by-step during recovery.

---

## 11. Battery

### Voltage monitoring

Battery voltage is read directly from the left drive VESC (CAN ID 1) via STATUS_5 packets — no external voltage divider hardware is needed. The pack is 13S LiPo (nominal 48 V).

| Voltage | State | Action |
|---------|-------|--------|
| > 45.5 V | OK | Normal operation |
| 42.9–45.5 V | WARNING | Amber LED overlay on the current state. Mowing continues. |
| < 42.9 V | LOW | Blade motor stops immediately (locked out until power cycle). Drive motors continue if mowing — mower returns to start, then stops. |

The low-voltage blade lockout is permanent until the mower is power-cycled. It cannot be reset by software. This prevents accidental restart on a deeply discharged pack.

### Mid-session battery swap

The supercapacitor keeps the ESP32 running for several minutes after the PILZ fires. This allows a battery swap without losing the session:

1. Press the PILZ mushroom button (LED goes to red/amber alternating).
2. Swap the battery.
3. Restore the PILZ relay.
4. VESCs come back on — the ESP32 detects this and recovers the session automatically.
5. Set CH4 = AUTO to resume mowing.

---

## 12. LED Status Reference

Both the onboard NeoPixel (GPIO 48) and the external LED strip (GPIO 7) show the same colour and pattern.

| State | Colour | Pattern | Meaning |
|-------|--------|---------|---------|
| INIT | Blue | Slow pulse | Collecting gyro bias; waiting for RC + GPS |
| IDLE | Blue | Single blink (100ms on, 900ms off) | Ready, disarmed, no active task |
| MANUAL | Green | Solid | Operator driving |
| LEARN (RTK fixed) | Green | Fast flash (100ms on/off) | Recording perimeter — good accuracy |
| LEARN (RTK float/autonomous) | Amber | Fast flash | Recording perimeter — reduced accuracy |
| AUTO_MOWING (RTK fixed) | Orange | Double blink | Mowing, all well |
| AUTO_MOWING (RTK float) | Yellow | Double blink | Mowing, GPS slightly degraded |
| AUTO_RETURN | Cyan/Blue | Alternating (500ms each) | Returning to start point |
| RETRACE | Yellow | Fast flash | Blade overloaded — retrace in progress |
| BOG_RECOVERY | Yellow | Slow pulse | Mower stalled/slipping — recovering |
| OBSTACLE_AVOID | Orange | Fast flash | IMU collision detected — avoiding |
| PAUSED | Purple | Fast flash | Mowing paused; waiting for resume |
| MOTORS_OFFLINE | Red/Amber | Alternating (500ms each) | PILZ fired or battery disconnected |
| Battery WARNING | Amber | Occasional flash (overlay) | Voltage below 45.5V; overlaid on current state |

**Quick guide:**
- **Orange double blink** = working normally — all is well
- **Purple fast flash** = paused; resume when ready
- **Yellow any pattern** = recoverable condition in progress; no immediate action needed
- **Red/Amber alternating** = PILZ fired; check hardware

---

## 13. Configuration (WebUI)

The Configuration panel is in the **Dashboard** tab (collapsed by default). Tap the header to expand. Configuration can only be changed when the mower is in **IDLE** or **MANUAL** state — the firmware rejects changes during mowing.

After editing, tap **Save Config** to send the new values. The firmware validates the inputs and sends back an error message if anything is out of range.

### Settings groups

**Drive control**

| Field | Default | Notes |
|-------|---------|-------|
| Max duty | 0.60 | Duty cycle ceiling for both AUTO and MANUAL (0–1) |
| Max yaw rate | 0.8 rad/s | Steering rate at full stick deflection |
| Heading Kp / Kd | 0.30 / 0.05 | Gyro heading stabilisation gains |
| Manual max speed | 0.5 m/s | Maximum speed at full throttle in MANUAL |
| Min turn radius | 0.0 m | 0 = tracked vehicle (can pivot on spot) |

**Robot footprint** (from steering centre)

Measure from the midpoint of the front axle:

| Field | Default | Measure |
|-------|---------|---------|
| Front | 0.50 m | Steering centre → front chassis edge |
| Rear | 0.20 m | Steering centre → rear chassis edge |
| Left | 0.30 m | Steering centre → left chassis edge |
| Right | 0.30 m | Steering centre → right chassis edge |

> Getting these wrong affects safety. If Left/Right are too small, the chassis may overhang the perimeter.

**Chassis / drive**

| Field | Notes |
|-------|-------|
| Chassis width | Distance between driven wheel contact patches (= track width) |
| Chassis length | Wheelbase front to rear |
| Wheel radius | Radius of driven wheels — must be measured precisely for odometry |
| Motor pole pairs | Rotor magnetic pole PAIRS (not total poles). Default: 7 |
| Gear ratio | Gearbox reduction from motor shaft to wheel. Default: 20:1 |

**GPS antenna offset**

Offset of the RTK antenna from the steering centre. Positive forward = antenna is ahead of the axle.

| Field | Default |
|-------|---------|
| Forward offset | −0.20 m (antenna behind axle) |
| Right offset | 0.00 m (centred) |

**Cutting geometry**

| Field | Default | Notes |
|-------|---------|-------|
| Steer to cut | 0.0 m | Signed distance: steering centre → blade disc centre |
| Blade radius | 0.21 m | Radius of cutting disc |
| Cut width | 0.38 m | Effective cut width per pass |
| Strip overlap | 0.02 m | Overlap between adjacent strips (must be < cut width) |
| Blade pole pairs | 10 | Blade motor pole pairs — measured from rotor detents |
| Blade RPM | 2800 | Target blade mechanical RPM |

**Cut height limits**

| Field | Default |
|-------|---------|
| Min mm | 35 mm |
| Max mm | 90 mm |

**Path following**

| Field | Default | Notes |
|-------|---------|-------|
| Lookahead base | 0.40 m | Minimum lookahead distance at zero speed |
| Lookahead K | 0.80 s | Lookahead scales with speed: total = base + K × speed |
| Mow speed | 0.15 m/s | Forward speed during strip mowing |
| Headland speed | 0.20 m/s | Speed during perimeter passes |
| Transit speed | 0.30 m/s | Speed when not mowing (repositioning) |

---

## 14. Serial Debug Commands

Connect via USB at **115200 baud**. The mower outputs 2 Hz JSON telemetry automatically.

| Command | Description |
|---------|-------------|
| `STATUS` | Full state: pose, heading, fix quality, speed, progress, blade load, calibration |
| `PERIMETER` | Dump recorded perimeter polygon vertices |
| `NAVBOUNDARY` | Dump navigation boundary polygon |
| `WORKINGAREA` | Dump working area polygon |
| `PLAN` | Dump the current waypoint plan |
| `GRID` | ASCII map of headland coverage progress |
| `UNREACHABLE` | List zones too narrow for the robot to enter |
| `EKFSTATE` | Full EKF state vector and 4×4 covariance matrix |
| `CALDUMP` | Gyro bias, blade calibration current, servo calibration |
| `OBSTACLES` | All tracked obstacle positions with retry count |
| `ERRORS` | Last 20 software-triggered pause events (perimeter breach, blade fault, etc.) |
| `CLEARPERIM CONFIRM` | Erase the stored perimeter from NVS (requires the word CONFIRM) |
| `RESETEKF` | Reset EKF covariance matrix (does not reset position) |
| `CALHEIGHT TEST <mm>` | Move servo to a specific height for verification |
| `PAUSE` | Toggle pause when connected to serial (same as CH7) |

### Automatic 2 Hz JSON telemetry

```json
{"t":123456,"state":"AUTO","x":3.21,"y":7.84,"hdg":1.57,
 "fix":4,"sat":12,"vel":0.43,"hprog":0.42,"sprog":0.71,
 "cutH":35,"bog":0,"obs":2,"unc":0.04,
 "bladeRPM":2795,"bladeA":8.3,"bladeLoad":0.69,"cutStatus":"NORMAL",
 "battV":50.4,"battState":"OK","collBase":0.082}
```

---

## 15. Troubleshooting

### Mower stays in INIT

- Check that the ER8 receiver is linked to the TX16S and the transmitter is on.
- Check that the GPS module has a clear sky view. INIT waits for a GPS ENU origin.
- On first use the origin is auto-seeded from the first RTK-quality fix. This can take several minutes if the NTRIP connection is slow or there is little sky visibility.
- Check USB serial output for `[BOOT]` messages.

### LED shows red/amber alternating immediately

The PILZ has fired or the VESCs lost power before the CAN bus was established. Check:
- Is the PILZ relay in RUN state?
- Are the VESCs powered from the 48V battery?
- Are the CAN bus wires connected and terminated at both ends?

### Mower enters PAUSED immediately after starting AUTO

The safety perimeter breach check triggered on entry. Possible causes:
- The mower is physically outside the recorded perimeter boundary.
- The GPS position is inaccurate (large EKF uncertainty) and the computed position appears outside the nav boundary.
- The perimeter was drawn/recorded in a different GPS session and the ENU origin has shifted.

Check `EKFSTATE` for position uncertainty and `STATUS` for GPS fix quality.

### WebUI "RC Transmitter Active" banner will not clear

The transmitter is on or the CRSF receiver is still linked. Power off the TX16S completely — switching to failsafe channel positions is not enough. The firmware detects the active link, not just the channel positions.

### Mower won't start blade in AUTO (blade not spinning)

- CH6 must be in the ARMED position (or **Start Blade** tapped in WebUI).
- Check `STATUS` output: `armed: true` must appear.
- Check battery voltage — blade is locked out below 42.9V.

### Mower goes into BOG_RECOVERY constantly

The slip detection is triggering: the wheels are spinning but GPS/EKF says the robot is not moving. Possible causes:
- Wet or very soft ground — the wheels are digging in rather than propelling the mower.
- Raise the cut height and approach affected areas more slowly.
- Check `SLIP_RATIO_THRESHOLD` in Config — increasing it makes the slip detection less sensitive.

### GPS position jumps during mowing

The NTRIP RTK correction has changed base station or lost packets. The EKF innovation gate prevents large single-cycle jumps from being applied instantly — the position will converge smoothly over 2–3 GPS updates. If jumps are frequent, check your NTRIP connection quality.

### Perimeter save fails after driving

Common reasons (check serial output):
- **Fewer than 6 points**: drove too fast — slow down or use CH8 to store individual points.
- **Self-intersecting polygon**: the driven path crossed itself. Start again or edit in the WebUI.
- **Area less than 5 m²**: the perimeter is too small.
- **No viable robot fit**: after insetting the navigation boundary, nothing remains — garden is too narrow for the robot.

### Blade fault / mower pauses unexpectedly with event latch

The blade current dropped to near zero while the blade was commanded on, or the blade failed to reach target RPM. Possible causes:
- Object lodged in the cutting deck — inspect and clear.
- Blade belt/spindle failure.
- Blade VESC fault — check VESC Tool for motor faults.

After clearing the cause, cycle the CH7 pause button to acknowledge and resume.
