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
| **CH4** | Mode | 3-position switch | **Down** = MANUAL · **Centre** = AUTO · **Up** = NUDGE (hand sticks back, AUTO held) |
| **CH5** | Learn (perimeter record) | 2-position switch | Up (active) = recording on, Down = off |
| **CH6** | Arm (blade enable) | 2-position switch | Up = ARMED (blade allowed), Down = DISARMED |
| **CH7** | Pause / Soft E-stop | Momentary or latching button | Press/hold = pause all motion |
| **CH8** | Store point | Momentary button | One press = record current GPS position as a perimeter waypoint |

### CH4 Mode switch in detail

| CH4 position | State machine mode | What happens |
|--------------|--------------------|-------------|
| Down | MANUAL | Operator drives with sticks; blade runs if CH6 armed |
| Centre | AUTO | Autonomous mowing (requires valid perimeter + GPS-established heading) |
| Up | NUDGE | AUTO is **held** — you drive the tracks with the sticks to nudge the mower (off an obstacle, or to correct a drift) **without ending the run**. Flip back to Centre and it resumes the mow exactly where it left off. The blade still follows CH6. (This replaced the old "return to start", which wasn't used.) |

> **Accidental mode switch:** if you flick AUTO → MANUAL by mistake and put it back to AUTO within ~4 seconds, it resumes the cycle where it left off rather than starting over. After 4 s, MANUAL → AUTO is treated as a fresh start.

### Telemetry on the TX16S

The mower transmits five CRSF telemetry frames back to the transmitter:
- **Flight mode** — shows current state name (INIT, IDLE, MANUAL, AUTO, etc.)
- **GPS** — position and speed (visible on EdgeTX GPS screen)
- **Battery** — voltage and estimated remaining %
- **Attitude** — heading
- **Mower Status (0x80)** — custom frame with mowing progress, blade load, fix quality, AUTO-deny reason, next-waypoint bearing, area mowed. Read with a Lua widget.

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
| **Auto** | Switch to AUTO_MOWING state | Requires valid perimeter + GPS-established heading; mower must be inside it |
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

You can draw and edit the perimeter directly on the map using the leaflet-geoman draw tools (pencil icon on the map). After editing, tap **Send to Mower** to upload it. Only the perimeter points are editable — the cut path and the GPS accuracy circles are display-only.

The map shows **two arrows**: a **red** one for the mower's fused (EKF) position and heading, and a **black** one for the raw GPS/RTK position and travel heading. When the fix is good they sit on top of each other; if the black one wanders off, that's the raw GPS being noisy (e.g. RTK "Float" under trees) and the red one is holding position on odometry — which is the intended behaviour. Each recorded perimeter corner also carries a small confidence circle sized to the GPS accuracy when it was taught.

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
2. Position the mower at each corner, then press the **CH8** momentary button to store that point. **The transmitter beeps once for each point recorded** — wait for the beep before moving on, so you don't double-record or miss a corner. Corners can be as little as 0.1 m apart.
3. When finished, flip CH5 off to close and save.

Each corner is tagged with the GPS accuracy at the moment you pressed CH8, so it's worth getting an **RTK-Fixed** solution (green LED) at each corner. A corner taught under tree cover in "Float" is automatically pulled slightly inward (toward the garden centre, by its uncertainty) so the mower keeps clear of the boundary there — the rest of the perimeter is unaffected.

### Method C — Draw on the WebUI map

1. Open the WebUI Map tab.
2. Use the draw tools to trace the perimeter directly on the satellite image.
3. Tap **Send to Mower** to upload. The mower derives the nav boundary and working area automatically.

You can also combine methods: record a rough perimeter by driving, tap **Load from Mower** in the WebUI, edit individual waypoints on the map to fix inaccurate sections, then **Send to Mower** again.

### After saving

The firmware automatically computes the **mowing plan**: a concentric inward spiral that follows the perimeter, then steps inward one cut-width per ring to the centre. If the garden pinches into separate areas, each is spiralled in turn. (A navigation boundary and working area are also derived and shown, but the spiral and the safety boundary work directly from the perimeter.)

The WebUI map shows the perimeter and the planned path after a successful upload or load.

---

## 6. Starting a Mow

### Via RC

1. Mower in **IDLE** state (blue single blink).
2. **Establish heading first.** The mower starts up with no absolute heading — it learns it from the GPS travel direction. Get an **RTK-Fixed** solution (green LED) and drive a short straight run in MANUAL; the TX compass swings to true North and then stays locked (you won't need to do it again unless you reset). AUTO will refuse to start until the heading is established (the TX banners the reason).
3. Set **CH4 = AUTO**.
4. Set **CH6 = ARMED** (blade on).
5. The mower enters AUTO_MOWING: LED = **orange double blink**.

The plan follows the perimeter, then spirals inward to the centre. If part of the garden is under tree cover and the fix drops to "Float", the mower keeps going on dead-reckoning (it does **not** chase the noisy Float position) and snaps back to GPS when a clean Fixed returns. To reach a node behind it (e.g. across a bridge between rings), it reverses rather than swinging a wide arc.

### Via WebUI

1. Turn off the RC transmitter (the "RC Transmitter Active" banner should disappear).
2. In the Controls tab, tap **Start Blade** to arm the blade.
3. Tap **Auto** to start mowing.

### During mowing

- **Orange double blink** = mowing, all well.
- **Yellow double blink** = RTK degraded to float — navigation continues on dead-reckoning until Fixed returns.
- The Dashboard shows progress (%), blade load, and EKF position uncertainty.
- The Map tab shows the planned route and which areas have been mowed, plus the red (EKF) and black (raw GPS) position arrows.

### Changing cut height mid-mow

- **RC**: Rotate the CH3 knob. The servo moves immediately; the new height is used for subsequent strips.
- **WebUI**: Drag the Cut Height slider in the Controls tab.

### Completion

When the spiral reaches the centre and the plan is finished, the mower stops, disarms the blade, and returns to **IDLE**. (There is no autonomous "drive back to start" — collect it from where it finishes, or drive it back in MANUAL.)

### Nudging mid-mow

If the mower needs a hand — too close to an obstacle, or a dead-reckoning drift to correct — flip **CH4 to UP (NUDGE)**. AUTO is held and you drive the tracks with the sticks. Flip back to **AUTO (Centre)** and it resumes the mow exactly where it left off (the plan and heading are preserved; the GPS keeps tracking your nudge, so it picks up correctly). The blade stays under CH6 throughout.

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
3. The mower checks whether it **moved** while paused (it compares position before and after). If it didn't move, it **resumes from exactly where it stopped**. If it was carried somewhere (it moved significantly), it **restarts the cycle**, since its dead-reckoned position can no longer be trusted. The heading is kept either way — no need to re-establish it.

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

Battery voltage is read from the **blade VESC (CAN ID 3)** via STATUS_5 packets (bytes 4–5) — no external voltage divider hardware is needed. The pack is 13S LiPo (nominal 48 V).

| Voltage | State | Action |
|---------|-------|--------|
| > 45.5 V | OK | Normal operation |
| 42.9–45.5 V | WARNING | Notification only: TX warning beep (repeats), flashing battery banner on the widget/phone, amber LED overlay. Mowing continues. |
| < 42.9 V | LOW | Notification only (as above, more urgent). The VESCs reduce power internally as the pack sags; **the decision to stop or finish the run is left to you.** |

There is deliberately **no** automatic blade lockout or drive-home on low battery — the VESCs manage the sagging voltage themselves, and a sudden autonomous stop/return mid-mow was more nuisance than help. Watch the warnings and bring it in when you're ready.

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
| NUDGE | Cyan | Solid (with GPS overlay) | AUTO held; you're driving the tracks to nudge it |
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
| Strip overlap | 0.02 m | Overlap between adjacent spiral rings (must be < cut width); ring spacing = cut width − overlap |
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
| Mow speed | 0.15 m/s | Forward speed while mowing |
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

### AUTO won't start (nothing happens when CH4 = AUTO)

AUTO is gated and the TX banner shows the reason for ~2 seconds. Common ones:
- **Heading not set** — drive a short straight run with an RTK-Fixed solution so the heading establishes (the TX compass swings to North), then try again.
- **GPS fix not RTK** — wait for at least a Float (preferably Fixed) solution.
- **Outside perimeter** — the mower's position is outside the recorded boundary.
- **No perimeter** — teach one first (§5).

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
- The blade ramps up over ~2 seconds (the VESC's current limit does the soft start) — give it a moment.

### Recovery modes (BOG / RETRACE / obstacle-avoid)

These are built but **currently disabled** — the detectors produced too many false triggers, so for now the blade is operator-controlled (CH6) in AUTO and the only automatic responses left are tilt and the perimeter-breach/VESC-silence safeties. So you should not see BOG_RECOVERY, RETRACE or OBSTACLE_AVOID in normal use; they'll be re-enabled once the detectors are trustworthy.

### GPS position jumps during mowing

The mower only snaps its position to the GPS when it has a clean **RTK-Fixed** solution; on Float (e.g. under trees) it ignores the noisy fix and dead-reckons on odometry, so the on-map position should stay smooth and only correct when Fixed returns. Watch the two map arrows: if the **black** (raw GPS) arrow jumps about while the **red** (fused) one holds steady, that's working as intended. If the *red* arrow drifts a long way before a Fixed pulls it back, you spent a long stretch in Float — improving sky view / antenna gain (more Fixed) is the fix.

### Perimeter save fails after driving

Common reasons (check serial output):
- **Fewer than 6 points**: drove too fast — slow down or use CH8 to store individual points.
- **Self-intersecting polygon**: the driven path crossed itself. Start again or edit in the WebUI.
- **Area less than 5 m²**: the perimeter is too small.
- **No viable robot fit**: after insetting the navigation boundary, nothing remains — garden is too narrow for the robot.

### Blade warning (load high / not reaching RPM)

In AUTO the blade is operator-controlled and a blade fault **warns but does not pause** the mower (it keeps driving). If the blade load reads high or it isn't reaching target RPM:
- Object lodged in the cutting deck — inspect and clear.
- Blade belt/spindle failure.
- Blade VESC fault — check VESC Tool for motor faults.

(The event-latched PAUSE still applies to the *active* safeties: perimeter breach and tilt.)

After clearing the cause, cycle the CH7 pause button to acknowledge and resume.
