# BNO055 IMU Swap — Plan 3: Calibration UX

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Surface BNO055 calibration status to the operator and let them launch a magnetometer recalibration: extend `MOWER_STATUS` with a calibration byte, add calib fields to the BLE STATUS JSON, add a `RECAL_IMU` command, show a "drive slow loops" prompt on the TX16S Lua widget and a PWA banner with a "Recalibrate compass" button.

**Architecture:** The state machine snapshot carries `imu_get_calib_status()`; `sendMowerStatus()` appends it as payload byte 20; the Lua widget decodes it and flashes a prompt when sys/mag calibration is below the confidence threshold. The PWA reads `imuCalSys`/`imuCalMag`/`headingConfident` from the STATUS JSON, shows a top banner + button, and the button sends `RECAL_IMU`, which calls `imu_recalibrate()` (Plan 1).

**Tech Stack:** ESP32-S3 / Arduino, CRSF custom frame 0x80, BLE GATT JSON, EdgeTX Lua, vanilla-JS PWA (Web Bluetooth). `arduino-cli` for firmware compile; Lua/PWA verified on-device/in-browser (deferred).

**Depends on:** Plan 1 (`imu_get_calib_status`, `imu_heading_is_confident`, `imu_recalibrate`).

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `Robo-Mower-V2/crsf_telemetry.h` | Modify | Add `calib_status` to `TelemetryData`; doc the 20-byte payload. |
| `Robo-Mower-V2/crsf_telemetry.cpp` | Modify | Append calib byte to the 0x80 frame. |
| `Robo-Mower-V2/state_machine.cpp` | Modify | Populate `td.calib_status`; add `RECAL_IMU` BLE handler. |
| `Robo-Mower-V2/ble_server.cpp` | Modify | Add `imuCalSys`/`imuCalMag`/`headingConfident` to STATUS JSON. |
| `TX16S Telemetry LUA script/main.lua` | Modify | Decode calib byte; flash "MAG CAL n/3" prompt. |
| `robomower-pwa/robomower.html` | Modify | `RECAL_IMU` command, top banner + button, status wiring. |

---

## Task 1: Add the calibration byte to MOWER_STATUS

**Files:**
- Modify: `Robo-Mower-V2/crsf_telemetry.h`
- Modify: `Robo-Mower-V2/crsf_telemetry.cpp`
- Modify: `Robo-Mower-V2/state_machine.cpp`

- [ ] **Step 1: Add the field to `TelemetryData`**

In `Robo-Mower-V2/crsf_telemetry.h`, in `struct TelemetryData`, change the mower-status
section header comment `// ── Mower status (frame 0x80, custom 19-byte payload) ──`
to `// ── Mower status (frame 0x80, custom 20-byte payload) ──`, and immediately
after the `uint32_t session_mowed_dm2;` line add:
```cpp
    uint8_t  calib_status;     ///< BNO055 calibration: bits 7:6 sys, 5:4 gyro,
                               ///< 3:2 accel, 1:0 mag (each 0–3). 0x80 byte 19.
```

- [ ] **Step 2: Append the byte in `sendMowerStatus()`**

In `Robo-Mower-V2/crsf_telemetry.cpp`, in the doc comment for `sendMowerStatus`,
after the `[18] (19) ... heading LSB` lines add:
```cpp
 *   [19] (20) uint8_t  calib            sys<<6 | gyro<<4 | accel<<2 | mag (each 0–3)
```
Then change:
```cpp
    uint8_t payload[19];
```
to:
```cpp
    uint8_t payload[20];
```
and immediately before `sendFrame(0x80, payload, sizeof(payload));` add:
```cpp
    payload[19] = d.calib_status;   // BNO055 calibration (sys/gyro/accel/mag)
```
(`sizeof(payload)` now yields 20, so the frame length updates automatically.)

- [ ] **Step 3: Populate the snapshot**

In `Robo-Mower-V2/state_machine.cpp`, in the telemetry snapshot block, immediately
after:
```cpp
        td.fix_type       = (uint8_t)gps.fix_type;
```
add:
```cpp
        td.calib_status   = imu_get_calib_status();
```

- [ ] **Step 4: Commit**

```bash
git add Robo-Mower-V2/crsf_telemetry.h Robo-Mower-V2/crsf_telemetry.cpp Robo-Mower-V2/state_machine.cpp
git commit -m "telemetry: add BNO calibration byte to MOWER_STATUS (0x80, now 20 B)"
```

---

## Task 2: Add calibration fields to the BLE STATUS JSON

**Files:**
- Modify: `Robo-Mower-V2/ble_server.cpp` (`build_status_json()`, ~lines 250–307)

- [ ] **Step 1: Compute the calib values**

In `build_status_json()`, immediately after:
```cpp
    int mod_imu  = (imu_is_present() && !imu_is_fault())                        ? 1 : 0;
```
add:
```cpp
    uint8_t imu_cal  = imu_get_calib_status();
    int     cal_sys  = (imu_cal >> 6) & 0x03;
    int     cal_mag  =  imu_cal       & 0x03;
    bool    hdg_conf = imu_heading_is_confident();
```

- [ ] **Step 2: Enlarge the buffer**

Change:
```cpp
    char buf[700];
```
to:
```cpp
    char buf[768];   // headroom for IMU calibration fields
```

- [ ] **Step 3: Add the JSON fields and arguments**

In the `snprintf` format string, find the final line:
```cpp
        "\"mod\":{\"imu\":%d,\"gps\":%d,\"rc\":%d,\"can\":%d,\"vL\":%d,\"vR\":%d,\"vB\":%d}}",
```
and replace it with:
```cpp
        "\"imuCalSys\":%d,\"imuCalMag\":%d,\"headingConfident\":%s,"
        "\"mod\":{\"imu\":%d,\"gps\":%d,\"rc\":%d,\"can\":%d,\"vL\":%d,\"vR\":%d,\"vB\":%d}}",
```
Then in the argument list, find:
```cpp
        mod_imu, mod_gps, mod_rc, mod_can, mod_vl, mod_vr, mod_vb);
```
and replace it with:
```cpp
        cal_sys, cal_mag, hdg_conf ? "true" : "false",
        mod_imu, mod_gps, mod_rc, mod_can, mod_vl, mod_vr, mod_vb);
```

- [ ] **Step 4: Commit**

```bash
git add Robo-Mower-V2/ble_server.cpp
git commit -m "ble: STATUS JSON adds imuCalSys/imuCalMag/headingConfident"
```

---

## Task 3: Add the `RECAL_IMU` BLE command

**Files:**
- Modify: `Robo-Mower-V2/state_machine.cpp` (`handle_ble_command()`, ~line 1113)

- [ ] **Step 1: Add the handler**

In `handle_ble_command()`, immediately before:
```cpp
    if (strcmp(cmd, "SEND_PERIMETER") == 0) {
```
insert:
```cpp
    if (strcmp(cmd, "RECAL_IMU") == 0) {
        imu_recalibrate();
        sys_log_push("IMU: compass recalibration started (drive slow loops in Manual)");
        request_beep(BEEP_CONFIRM);
        return;
    }
```

- [ ] **Step 2: Commit**

```bash
git add Robo-Mower-V2/state_machine.cpp
git commit -m "sm: RECAL_IMU BLE command triggers imu_recalibrate()"
```

---

## Task 4: TX16S Lua — decode calib byte and prompt

**Files:**
- Modify: `TX16S Telemetry LUA script/main.lua`

- [ ] **Step 1: Fix the header comment**

Change the line:
```lua
-- ║    0x80  MOWER_STATUS — 13-byte custom payload (see spec)            ║
```
to:
```lua
-- ║    0x80  MOWER_STATUS — 20-byte custom payload (see spec)            ║
```

- [ ] **Step 2: Add calib fields to widget state**

In `create()`, in the returned table, after the line `direct_decode = false,  -- true once extended 0x80 payload (>=19 B) seen`
add:
```lua
        calib         = 0,      -- packed BNO calibration byte (0x80 offset 19)
        have_calib    = false,  -- true once a >=20 B payload has been seen
```

- [ ] **Step 3: Decode the calib byte in `parse_telemetry`**

In `parse_telemetry`, immediately after the `if #data >= 19 then ... end` block
(the one that sets `widget.voltage` / `widget.heading` / `widget.direct_decode`),
add:
```lua
            -- Calibration byte (firmware >= 2026-06-14): BNO sys/gyro/accel/mag.
            -- data[20] = offset 19: bits 7:6 sys, 5:4 gyro, 3:2 accel, 1:0 mag.
            if #data >= 20 then
                widget.calib      = data[20] or 0
                widget.have_calib = true
            end
```

- [ ] **Step 4: Draw the prompt in `refresh`**

In `refresh`, immediately after `draw_dividers()` and before the battery-warning
banner block (`if bit32.band(flags, 0x20) ~= 0 then`), add:
```lua
    -- Magnetometer-calibration prompt: when the BNO heading is not yet
    -- trustworthy (sys < 2 or mag < 2), flash a banner telling the operator to
    -- drive slow loops in MANUAL. One-time after first setup (profile persists).
    if widget.have_calib then
        local cal     = widget.calib or 0
        local cal_sys = bit32.band(bit32.rshift(cal, 6), 0x03)
        local cal_mag = bit32.band(cal, 0x03)
        if not (cal_sys >= 2 and cal_mag >= 2) then
            if math.floor(getTime() / 50) % 2 == 0 then
                lcd.drawFilledRectangle(0, HDR_H + 1, SCR_W, 22, C_AMBER)
                lcd.drawText(SCR_W / 2, HDR_H + 3,
                    string.format("MAG CAL %d/3  -  DRIVE SLOW LOOPS (MANUAL)", cal_mag),
                    FXS + FCENT + lcd.RGB(0, 0, 0))
            end
        end
    end
```

- [ ] **Step 5: Commit**

```bash
git add "TX16S Telemetry LUA script/main.lua"
git commit -m "lua: decode BNO calib byte; flash MAG CAL prompt when not confident"
```

- [ ] **Step 6: [ON-HARDWARE — DEFERRED] Verify on the TX16S**

With the BNO wired and uncalibrated: the banner flashes `MAG CAL n/3`; driving
slow loops raises `n`; the banner clears at sys≥2 & mag≥2.

---

## Task 5: PWA — banner, button, and command

**Files:**
- Modify: `robomower-pwa/robomower.html`

- [ ] **Step 1: Add the command factory**

In the `Commands` object, replace:
```javascript
  clearPerimeter:()     => ({ cmd: 'CLEAR_PERIMETER' }),
  sendPerimeter: (origin, polygon) => ({ cmd: 'SEND_PERIMETER', origin, polygon })
```
with:
```javascript
  clearPerimeter:()     => ({ cmd: 'CLEAR_PERIMETER' }),
  recalibrateImu:()     => ({ cmd: 'RECAL_IMU' }),
  sendPerimeter: (origin, polygon) => ({ cmd: 'SEND_PERIMETER', origin, polygon })
```

- [ ] **Step 2: Add the banner CSS**

Immediately after the `#rc-gating-banner .rc-icon { font-size:2rem; }` rule, add:
```css
/* Magnetometer-calibration warning bar (fixed, top) */
#mag-cal-banner {
  position:fixed; top:0; left:0; right:0; z-index:60;
  display:none; align-items:center; justify-content:center; gap:12px;
  background:var(--amber); color:#000; padding:8px 12px;
  font-size:0.9rem; font-weight:700; text-align:center;
}
#mag-cal-banner button {
  background:#000; color:var(--amber); border:none; border-radius:6px;
  padding:6px 12px; font-weight:700; cursor:pointer;
}
```

- [ ] **Step 3: Add the banner markup**

Immediately after `<!-- ACK toast --> <div class="ack-toast" id="ack-toast"></div>`
(i.e. before the `<!-- No-BLE banner -->` comment), add:
```html
<!-- Magnetometer calibration warning -->
<div id="mag-cal-banner">
  <span>&#x1F9ED; Compass not calibrated (<span id="mag-cal-level">sys 0/3 mag 0/3</span>) — drive slow loops in Manual</span>
  <button id="btn-recal-imu" type="button">Recalibrate compass</button>
</div>
```

- [ ] **Step 4: Toggle the banner from telemetry**

In the `telem.onChange(d => { ... })` handler, immediately after the battery-warning
block (after the line `window._prevBattState = d.battState;` and its closing `}`),
and before `document.getElementById('dash-grid').innerHTML = buildDashboard(telem);`,
add:
```javascript
    // Magnetometer calibration banner: prompt recal when heading not trustworthy.
    const magBanner = document.getElementById('mag-cal-banner');
    if (magBanner) {
      const showMag = ble.connected && d.headingConfident === false;
      magBanner.style.display = showMag ? 'flex' : 'none';
      if (showMag) {
        const lvl = document.getElementById('mag-cal-level');
        if (lvl) lvl.textContent = `sys ${d.imuCalSys ?? 0}/3  mag ${d.imuCalMag ?? 0}/3`;
      }
    }
```

- [ ] **Step 5: Wire the button**

Immediately after the `btn-refresh-log` click handler (the block ending at the
`});` after `Commands.requestStatus()`), add:
```javascript
  document.getElementById('btn-recal-imu')?.addEventListener('click', async () => {
    if (!ble.connected) { showToast('Not connected', false); return; }
    try {
      await ble.sendCommand(Commands.recalibrateImu());
      showToast('Compass recalibration started — drive slow loops in Manual', true);
    } catch (e) { showToast(e.message, false); }
  });
```

- [ ] **Step 6: Sanity-check the HTML loads**

Open `robomower-pwa/robomower.html` in a desktop browser. Expected: page renders
with no console errors; the mag banner is hidden (no connection / `headingConfident`
undefined). Full behaviour is verified on-hardware (deferred).

- [ ] **Step 7: Commit**

```bash
git add robomower-pwa/robomower.html
git commit -m "pwa: mag-cal banner + Recalibrate compass button (RECAL_IMU)"
```

---

## Task 6: Compile the firmware

**Files:** none (verification)

- [ ] **Step 1: Compile**

Run:
```powershell
$cli = "C:\Users\simon\Downloads\arduino-cli_1.5.0_Windows_64bit\arduino-cli.exe"
& $cli compile --fqbn esp32:esp32:esp32s3 `
  --board-options "PartitionScheme=huge_app,PSRAM=opi" Robo-Mower-V2
```
Expected: **no errors.** Likely fix points: `td.calib_status` set before
`crsf_telemetry_update`, the STATUS `snprintf` arg count matching the format,
`imu.h` already included in `state_machine.cpp` and `ble_server.cpp` (Plan 1).

- [ ] **Step 2: Commit (marker)**

No code change unless Step 1 required fixes (commit those under their task).

---

## Self-Review (against the spec)

- **§8 telemetry calib status in `MOWER_STATUS`:** Task 1. ✓
- **§8 Lua widget prompt ("MAG CAL n/3 — drive slow loops"):** Task 4. ✓ (cleared at sys≥2 & mag≥2, matching `imu_heading_is_confident`.)
- **§8.1 PWA warning banner + "Recalibrate compass" button:** Task 5. ✓
- **§8.1 `RECAL_IMU` BLE command → `imu_recalibrate()`:** Task 3 (+ Plan 1 driver). ✓
- **§8.1 PWA telemetry fields (`imuCalSys`, `imuCalMag`, `headingConfident`):** Task 2 (firmware) + Task 5 (consumer). ✓
- **§8.1 RC-side is warning + progress only (no trigger control):** Task 4 (display only — no RC channel added). ✓
- **§8 calibration is one-time (profile persists):** copy text reflects this; persistence itself is Plan 1 (`bnocal`). ✓

**Type/name consistency:** JSON keys `imuCalSys`/`imuCalMag`/`headingConfident` identical in producer (Task 2) and consumer (Task 5); command string `RECAL_IMU` identical in PWA (Task 5), dispatch (Task 3), and Plan 1's `imu_recalibrate`; calib bit-packing `sys<<6|gyro<<4|accel<<2|mag` identical in driver (Plan 1), STATUS JSON (Task 2), and Lua (Task 4).

**Carried to later plans:** none — calibration UX is self-contained here.
