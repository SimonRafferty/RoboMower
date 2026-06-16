# CRSF Telemetry Reference

ESP32-S3 → ER8 receiver → RadioMaster TX16S

## Overview

Five CRSF frame types are transmitted in a fixed rotation, one frame per
inter-packet gap (~2 ms silence between incoming RC frames at 100 Hz).
Data is snapshot at **2 Hz** by the state machine; frames are dispatched by
the CRSF receive task whenever a gap is detected.

**Rotation order:** FLIGHT_MODE → GPS → BATTERY → MOWER_STATUS → ATTITUDE → repeat

**Baud rate:** 420 000 (standard CRSF)

**Serial:** `Serial2` (ESP32-S3 UART2)

---

## Frame Format (all frames)

```
[sync] [len] [type] [payload ...] [CRC8]
```

| Byte   | Description |
|--------|-------------|
| `0xC8` | Sync byte (CRSF standard) |
| `len`  | `payload_len + 2` (counts type + CRC, excludes sync & len) |
| `type` | Frame type identifier |
| payload | Type-specific, see below |
| CRC8   | Polynomial `0xD5` (DVB-S2), computed over `type + payload` only |

---

## 1. FLIGHT_MODE — `0x1E`

Null-terminated ASCII string. Max 16 bytes (15 printable chars + `\0`).

**Payload (variable length, 2–16 bytes):**

| Offset | Type | Description |
|--------|------|-------------|
| 0..N-1 | char[] | ASCII state name |
| N | `\0` | Null terminator (always present) |

**Possible values:**
`INIT`, `IDLE`, `MANUAL`, `LEARN`, `AUTO`, `RETRACE`, `BOG`, `OBS-AVOID`, `RETURN`, `PAUSED`, `MOT-OFF`

EdgeTX displays this natively on the main screen.

---

## 2. GPS — `0x02`

**Payload: 15 bytes, big-endian**

| Offset | Type | Unit | Description |
|--------|------|------|-------------|
| 0–3 | int32 | degrees × 10⁷ | Latitude (WGS-84, EKF-fused) |
| 4–7 | int32 | degrees × 10⁷ | Longitude (WGS-84, EKF-fused) |
| 8–9 | uint16 | km/h × 100 | Ground speed (from EKF velocity × 3.6) |
| 10–11 | uint16 | degrees × 100 | Heading [0, 36000) |
| 12–13 | uint16 | metres + 1000 | Altitude (offset to stay unsigned; 0 m → 1000) |
| 14 | uint8 | count | Satellites used |

**Decoding in Lua:**
```lua
lat = payload_i32(0) / 1e7
lon = payload_i32(4) / 1e7
speed_kmh = payload_u16(8) / 100
heading_deg = payload_u16(10) / 100
alt_m = payload_u16(12) - 1000
sats = payload[14]
```

EdgeTX recognises this frame natively (GPS sensor).

---

## 3. BATTERY — `0x08`

**Payload: 8 bytes, big-endian**

| Offset | Type | Unit | Description |
|--------|------|------|-------------|
| 0–1 | uint16 | V × 100 (centi-volts) | Battery voltage |
| 2–3 | uint16 | A × 100 (centi-amps) | Blade motor current |
| 4–6 | uint24 | mAh | Capacity used (3 bytes, MSB first) |
| 7 | uint8 | % | Battery remaining (0–100) |

**Battery:** 13S LiPo — full ≈ 54.6 V, low = 42.9 V (`BATTERY_LOW_V` in config.h)

**Decoding in Lua:**
```lua
voltage = payload_u16(0) / 100       -- volts
current = payload_u16(2) / 100       -- amps
cap_mah = (payload[4] << 16) | (payload[5] << 8) | payload[6]
remaining = payload[7]               -- percent
```

EdgeTX recognises this frame natively (battery sensor).

---

## 4. MOWER_STATUS — `0x80` (custom)

**Not an EdgeTX-standard frame.** Must be parsed by a Lua widget via
`crossfireTelemetryPop()`. Standard CRSF frame types (GPS, Battery, Attitude,
FlightMode) are consumed by EdgeTX internally and are **not** passed to Lua —
all data needed by the widget must be in this frame.

**Payload: 19 bytes** (15 before firmware 2026-06-10; the Lua widget accepts both)

| Offset | Type | Unit | Description |
|--------|------|------|-------------|
| 0 | uint8 | enum | Robot state (see below) |
| 1 | uint8 | % | Headland progress (0–100) |
| 2 | uint8 | % | Strip progress (0–100) |
| 3 | uint8 | mm | Cut height |
| 4 | uint8 | % | Blade load (0–100) |
| 5 | uint8 | enum | GPS fix type (see below) |
| 6 | uint8 | bitfield | Status flags (see below) |
| 7–8 | uint16 | count | Obstacles detected (big-endian) |
| 9–10 | uint16 | cm | EKF position uncertainty (big-endian) |
| 11–14 | uint32 | dm² | Session mowed area (big-endian) |
| 15–16 | uint16 | V×100 | Battery voltage (big-endian) — duplicated here so the widget does not depend on the EdgeTX `RxBt` sensor |
| 17–18 | uint16 | deg×10 | Heading 0–359.9° (big-endian) — duplicated here so the widget does not depend on the EdgeTX `Yaw` sensor |

**State values (offset 0):**

| Value | Name | Description |
|-------|------|-------------|
| 0 | INIT | Boot / hardware init |
| 1 | IDLE | Armed, awaiting command |
| 2 | MANUAL | RC driving |
| 3 | LEARN | Recording perimeter |
| 4 | AUTO | Autonomous mowing |
| 5 | RETRACE | Overload recovery |
| 6 | BOG | Stall recovery |
| 7 | OBS-AVOID | Obstacle avoidance |
| 8 | RETURN | Returning to start |
| 9 | PAUSED | Mowing paused |
| 10 | MOT-OFF | Motors offline |

**Fix type values (offset 5):**

| Value | Meaning |
|-------|---------|
| 0 | No fix |
| 1 | GPS (autonomous) |
| 2 | DGPS |
| 4 | RTK fixed |
| 5 | RTK float |

**Flags bitfield (offset 6):**

| Bit | Meaning |
|-----|---------|
| 0 | Armed |
| 1 | Blade on |
| 2 | Bog recovery active |
| 3 | Retrace active |
| 4 | RTK float (degraded) |
| 5 | Battery WARNING/LOW — Lua shows flashing battery banner (was "obstacle near", never implemented) |
| 7:6 | Beep request (2-bit field, see below) |

**Beep request (flags bits 7:6) — one-shot, auto-cleared after transmission:**

| Bits 7:6 | Value | Meaning |
|----------|-------|---------|
| `00` | 0 | No beep |
| `01` | 1 | Confirm (short tone) |
| `10` | 2 | Warning (attention) |
| `11` | 3 | Fault (critical) |

**Decoding in Lua:**
```lua
local state_names = {
    [0]="INIT", [1]="IDLE", [2]="MANUAL", [3]="LEARN", [4]="AUTO",
    [5]="RETRACE", [6]="BOG", [7]="OBS-AVOID", [8]="RETURN",
    [9]="PAUSED", [10]="MOT-OFF"
}

state       = payload[0]
state_name  = state_names[state] or "?"
hprog       = payload[1]
sprog       = payload[2]
cut_mm      = payload[3]
blade_load  = payload[4]   -- 0–100 %; RPM-based since 2026-06-16 (Feature 2): 0 % at/above target rpm, 100 % at standstill
fix         = payload[5]
flags       = payload[6]
obs_count   = (payload[7] << 8) | payload[8]
ekf_unc_cm  = (payload[9] << 8) | payload[10]
mowed_dm2   = (payload[11] << 24) | (payload[12] << 16) | (payload[13] << 8) | payload[14]
mowed_m2    = mowed_dm2 / 100    -- convert dm² to m²

armed       = bit32.band(flags, 0x01) ~= 0
blade_on    = bit32.band(flags, 0x02) ~= 0
bog         = bit32.band(flags, 0x04) ~= 0
retrace     = bit32.band(flags, 0x08) ~= 0
rtk_float   = bit32.band(flags, 0x10) ~= 0
obs_near    = bit32.band(flags, 0x20) ~= 0
beep        = bit32.rshift(bit32.band(flags, 0xC0), 6)  -- 0=none 1=confirm 2=warn 3=fault
```

---

## 5. ATTITUDE — `0x14`

**Payload: 6 bytes, big-endian, signed**

| Offset | Type | Unit | Description |
|--------|------|------|-------------|
| 0–1 | int16 | radians × 10000 | Pitch (always 0 — mower is flat) |
| 2–3 | int16 | radians × 10000 | Roll (always 0 — mower is flat) |
| 4–5 | int16 | radians × 10000 | Yaw (EKF heading, −π to +π) |

**Decoding in Lua:**
```lua
pitch_rad = payload_i16(0) / 10000
roll_rad  = payload_i16(2) / 10000
yaw_rad   = payload_i16(4) / 10000
heading_deg = math.deg(yaw_rad)    -- convert to degrees if needed
```

EdgeTX recognises this frame natively.

---

## CRC8 Implementation

Polynomial `0xD5` (DVB-S2). Input: `type` byte + all payload bytes. Sync and length bytes are excluded.

```lua
function crsf_crc8(data)
    local crc = 0
    for i = 1, #data do
        crc = bit32.bxor(crc, data:byte(i))
        for _ = 1, 8 do
            if bit32.band(crc, 0x80) ~= 0 then
                crc = bit32.bxor(bit32.lshift(crc, 1), 0xD5)
            else
                crc = bit32.lshift(crc, 1)
            end
            crc = bit32.band(crc, 0xFF)
        end
    end
    return crc
end
```

---

## Timing

- Data snapshot: updated at **2 Hz** (every 500 ms) from state machine
- Frame dispatch: one frame per CRSF inter-packet gap (~100 Hz opportunity)
- Rotation of 5 frames → each type sent ~20 times/sec (far exceeds 2 Hz data rate)
- Longest frame (GPS, 19 bytes total) takes ~360 µs at 420 kbaud — fits within the ~8 ms gap

---

## Source Files

| File | Purpose |
|------|---------|
| `crsf_telemetry.h` | TelemetryData struct, public API |
| `crsf_telemetry.cpp` | Frame builders, CRC, rotation logic |
| `crsf_input.cpp` | CRSF receive + telemetry window detection |
| `state_machine.cpp` | Populates TelemetryData at 2 Hz |
| `config.h` | Battery voltage thresholds |
