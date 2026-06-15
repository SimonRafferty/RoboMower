-- ╔══════════════════════════════════════════════════════════════════════╗
-- ║  MowerHUD  —  EdgeTX Lua Widget  —  RadioMaster TX16S               ║
-- ║  Autonomous Lawnmower Telemetry Display  (480 × 272 colour screen)   ║
-- ║                                                                      ║
-- ║  INSTALLATION                                                        ║
-- ║    /WIDGETS/MowerHUD/main.lua  on the SD card root                   ║
-- ║    Assign as a full-screen widget in EdgeTX model setup              ║
-- ║    Requires EdgeTX 2.7 or later                                      ║
-- ║                                                                      ║
-- ║  CRSF TELEMETRY                                                      ║
-- ║    0x80  MOWER_STATUS — 20-byte custom payload (see spec)            ║
-- ║    GPS / Battery / Attitude / FlightMode are consumed by EdgeTX      ║
-- ║    internally and do NOT arrive via crossfireTelemetryPop().         ║
-- ║    Heading and speed are read via getValue() (EdgeTX native sensors).║
-- ║                                                                      ║
-- ║  CALIBRATION NOTES                                                   ║
-- ║    GSpd  Verify units in your sensor list (km/h vs knots).           ║
-- ║    Battery  13S LiPo assumed (42.9 V empty to 54.6 V full).         ║
-- ╚══════════════════════════════════════════════════════════════════════╝

local name    = "MowerHUD"
local options = {}
local bor     = bit32.bor  -- Lua 5.2 compatible bitwise OR

-- ═══════════════════════════════════════════════════════════════════════
--  FONT / FLAG CONSTANTS
--  Correct EdgeTX names — available since EdgeTX 2.3 on all color radios.
--  SMLSIZE (~12px) is the smallest named font.
--  0 (no flag)    = default font (~16px).
--  MIDSIZE        = mid-size font (~21px) — used for the state badge.
--  CENTER / RIGHT / SOLID are standard since EdgeTX 2.3.
-- ═══════════════════════════════════════════════════════════════════════

local FXXS   = SMLSIZE   -- ~12px: tiny labels, section headers
local FXS    = SMLSIZE   -- ~12px: same — no named size between SMLSIZE and default
local FS     = 0         -- default (~16px): badge text, data values
local FCENT  = CENTER    -- centre-justify (color displays, EdgeTX 2.3+)
local FRGHT  = RIGHT     -- right-justify  (EdgeTX 2.3+)
local FSOLID = SOLID     -- solid line     (EdgeTX 2.3+)

-- lcd.drawFilledTriangle was added in EdgeTX 2.7.
-- Guard against it being nil; fall back to three outline lines.
local function filled_tri(x1, y1, x2, y2, x3, y3, col)
    if lcd.drawFilledTriangle then
        lcd.drawFilledTriangle(x1, y1, x2, y2, x3, y3, col)
    else
        lcd.drawLine(x1, y1, x2, y2, FSOLID, col)
        lcd.drawLine(x2, y2, x3, y3, FSOLID, col)
        lcd.drawLine(x3, y3, x1, y1, FSOLID, col)
    end
end

-- ═══════════════════════════════════════════════════════════════════════
--  LAYOUT  — pure arithmetic, safe at module level (no lcd calls here)
-- ═══════════════════════════════════════════════════════════════════════

local SCR_W  = 480
local SCR_H  = 272
local HDR_H  = 30
local VESC_H = 28
local BODY_Y = HDR_H + 1
local VESC_Y = SCR_H - VESC_H
local BODY_H = VESC_Y - BODY_Y

local L_W = 107
local C_W = 266
local R_W = 107

local CX = L_W + math.floor(C_W / 2)
local CY = BODY_Y + math.floor(BODY_H / 2)

local OUTER_R    = 86
local INNER_R    = 78
local LABEL_R    = 95
local MW         = 22       -- mower rect width px  = 500 mm physical
local MH         = 26       -- mower rect height px = 600 mm physical
-- Scale: MW px = 500 mm physical  →  1 mm = MW/500 px ≈ 0.044 px/mm
-- EKF uncertainty (real measured value in mm from MOWER_STATUS offsets 9-10)
-- is converted to screen pixels using this factor, so the circle is true-to-scale
-- relative to the mower rectangle.
-- At <50 mm the circle (~2 px radius) is too small to read clearly, so
-- sub-50 mm accuracy is shown as a fixed 100 mm-diameter filled white dot.
local MM_TO_PX   = MW / 500.0
local MAX_VEL    = 1.0

-- ═══════════════════════════════════════════════════════════════════════
--  COLOUR TABLES
--  Declared nil here; populated inside create() via init_colors().
--  lcd.RGB() must NOT be called at module load time — it is not available
--  until after EdgeTX has initialised the display and called create().
-- ═══════════════════════════════════════════════════════════════════════

local C_BG, C_RING, C_BORDER, C_DIM
local C_WHITE, C_CYAN, C_AMBER, C_RED, C_GREEN, C_ORANGE
local STATE_COL, STATE_DEF
local FIX_COL,   FIX_DEF
local VESC_COL

local function init_colors()
    -- Base palette. RGB565 quantises aggressively below ~32/64/32 —
    -- everything darker looks identical to black on screen.
    -- All "dark" colours are therefore lifted to be clearly visible.
    C_BG     = lcd.RGB( 18,  44,  18)   -- panel background: visible dark green
    C_RING   = lcd.RGB( 32,  72,  32)   -- compass ring band: slightly lighter than BG
    C_BORDER = lcd.RGB( 56, 112,  56)   -- dividers / rules: clearly visible
    C_DIM    = lcd.RGB(104, 140, 128)   -- section headers / dim labels
    C_WHITE  = lcd.RGB(232, 232, 232)
    C_CYAN   = lcd.RGB( 96, 205, 255)
    C_AMBER  = lcd.RGB(255, 176,  32)
    C_RED    = lcd.RGB(255,  64,  64)
    C_GREEN  = lcd.RGB( 64, 232, 128)
    C_ORANGE = lcd.RGB(255, 136,   0)

    -- State badge backgrounds — lifted so they read as coloured, not black
    STATE_COL = {
        INIT          = { lcd.RGB( 48, 80, 48),  lcd.RGB(168,200,168) },
        IDLE          = { lcd.RGB( 40, 56,104),  lcd.RGB(138,180,216) },
        MANUAL        = { lcd.RGB( 32, 72, 96),  lcd.RGB( 96,205,255) },
        LEARN         = { lcd.RGB( 72, 40,120),  lcd.RGB(192,138,224) },
        AUTO          = { lcd.RGB( 32, 72, 32),  lcd.RGB(232,232,232) },
        RETRACE       = { lcd.RGB( 32, 64, 96),  lcd.RGB( 96,168,248) },
        BOG           = { lcd.RGB(104, 56,  8),  lcd.RGB(255,176, 32) },
        ["OBS-AVOID"] = { lcd.RGB(104, 32,  8),  lcd.RGB(255,104, 64) },
        RETURN        = { lcd.RGB( 32, 56,104),  lcd.RGB( 96,160,255) },
        PAUSED        = { lcd.RGB( 96, 72,  8),  lcd.RGB(255,215, 64) },
        ["MOT-OFF"]   = { lcd.RGB( 56, 56, 56),  lcd.RGB(144,144,144) },
    }
    STATE_DEF = { lcd.RGB(56, 56, 56), lcd.RGB(144, 144, 144) }

    -- Fix badge backgrounds — clearly distinguishable from C_BG
    FIX_COL = {
        [0] = { lcd.RGB( 96,  0,  0), lcd.RGB(255, 80, 80), "NO FIX"    },
        [1] = { lcd.RGB( 96, 56,  0), lcd.RGB(255,168, 48), "GPS ONLY"  },
        [2] = { lcd.RGB( 80, 48, 16), lcd.RGB(224,192,112), "DGPS"      },
        [4] = { lcd.RGB(  8, 64, 32), lcd.RGB( 80,240,160), "RTK FIXED" },
        [5] = { lcd.RGB( 96, 56,  0), lcd.RGB(255,176, 32), "RTK FLOAT" },
    }
    FIX_DEF = { lcd.RGB(96, 0, 0), lcd.RGB(255, 80, 80), "UNKNOWN" }

    -- VESC panel backgrounds — must differ from C_BG (dark green)
    VESC_COL = {
        FWD   = { lcd.RGB( 24, 56, 24), lcd.RGB(232,232,232), lcd.RGB( 80,232,144), "FWD",   "RUNNING" },
        REV   = { lcd.RGB( 56, 32, 16), lcd.RGB(255,176, 32), lcd.RGB(255,144, 64), "REV",   "REVERSE" },
        IDLE  = { lcd.RGB( 24, 40, 80), lcd.RGB(160,200,232), lcd.RGB( 96,160,208), "IDLE",  "STANDBY" },
        FAULT = { lcd.RGB( 80,  0,  0), lcd.RGB(255, 80, 80), lcd.RGB(255, 48, 48), "FAULT", "! CHECK" },
    }
end

-- ═══════════════════════════════════════════════════════════════════════
--  AUDIO
--  playTone(freq_hz, duration_ms, pause_ms) — built into EdgeTX.
--  Guard against it being absent in unusual builds.
--  Beep types match bits 7:6 of the MOWER_STATUS flags byte:
--    1 = confirm  (mode accepted, operation started)
--    2 = warning  (bog, obstacle, RTK degraded)
--    3 = fault    (VESC fault, CAN loss)
-- ═══════════════════════════════════════════════════════════════════════

local function play_beep(beep_type)
    if not playTone then return end
    if     beep_type == 1 then   -- Confirm: clean single pip
        playTone(1200, 120, 0)
    elseif beep_type == 2 then   -- Warning: descending double
        playTone( 900, 100, 40)
        playTone( 650, 180, 0)
    elseif beep_type == 3 then   -- Fault: three short harsh pulses
        playTone( 440, 80, 40)
        playTone( 440, 80, 40)
        playTone( 440, 180, 0)
    end
end
-- ═══════════════════════════════════════════════════════════════════════

local function rot_xy(cx, cy, r, deg)
    local rad = math.rad(deg)
    return math.floor(cx + r * math.sin(rad) + 0.5),
           math.floor(cy - r * math.cos(rad) + 0.5)
end

local function clamp(v, lo, hi)
    return math.max(lo, math.min(hi, v))
end

local function batt_col(pct)
    if pct < 20 then return C_RED
    elseif pct < 40 then return C_AMBER
    else return lcd.RGB(96, 168, 112) end
end

local function blade_col(pct)
    if pct > 85 then return C_RED
    elseif pct > 65 then return C_AMBER
    else return lcd.RGB(96, 120, 144) end
end

local function uncert_col(mm)
    if mm <  50 then return lcd.RGB(255, 255, 255) end
    if mm < 100 then return lcd.RGB( 40, 210, 100) end
    if mm < 300 then return C_AMBER end
    return C_RED
end

local function draw_bar(x, y, w, h, pct, col)
    lcd.drawFilledRectangle(x, y, w, h, lcd.RGB(18, 44, 18))
    lcd.drawRectangle(x, y, w, h, C_BORDER)
    local fw = math.floor(w * clamp(pct, 0, 100) / 100 + 0.5)
    if fw > 0 then lcd.drawFilledRectangle(x, y, fw, h, col) end
end

local function draw_sh(x, y, w, label)
    lcd.drawText(x, y, label, bor(FXXS, C_DIM))
    lcd.drawLine(x, y + 13, x + w, y + 13, FSOLID, C_BORDER)
end

-- ═══════════════════════════════════════════════════════════════════════
--  SWITCH INFERENCE
--  Shows what the mower understood, confirming commands were received.
--  SA Pause : PAUSED -> PAUSE,  else -> RUN
--  SB Mode  : MANUAL -> MANUAL, RETURN -> RETURN, else -> AUTO
--  SC Learn : LEARN  -> LEARN,  else -> OFF
--
--  Note: using explicit if/elseif instead of (cond and 0 or x) because
--  Lua treats 0 as truthy but the ternary idiom fails when the true-
--  branch value is 0 (falsy in boolean context for the or chain).
-- ═══════════════════════════════════════════════════════════════════════

local function infer_switches(state_str)
    local sa, sb, sc

    if state_str == "PAUSED" then sa = 1 else sa = 0 end

    if     state_str == "MANUAL" then sb = 0
    elseif state_str == "RETURN" then sb = 2
    else                              sb = 1
    end

    if state_str == "LEARN" then sc = 1 else sc = 0 end

    return sa, sb, sc
end

-- ═══════════════════════════════════════════════════════════════════════
--  COMPASS / MOWER GRAPHIC
-- ═══════════════════════════════════════════════════════════════════════

local function draw_compass(heading, speed_kmh, ekf_mm, fix_type, is_bog, is_collision)

    -- Ring: fill outer disc, punch out inner disc with background colour
    lcd.drawFilledCircle(CX, CY, OUTER_R,     C_RING)
    lcd.drawFilledCircle(CX, CY, INNER_R - 1, C_BG)
    lcd.drawCircle(CX, CY, OUTER_R, lcd.RGB(64, 104, 80))
    lcd.drawCircle(CX, CY, INNER_R, lcd.RGB(48,  96, 56))

    -- Tick marks rotated by heading
    for a = 0, 350, 10 do
        local sa       = a - heading
        local is_card  = (a % 90 == 0)
        local is_major = (a % 30 == 0)
        local r0       = OUTER_R - (is_card and 9 or (is_major and 7 or 4))
        local x0, y0   = rot_xy(CX, CY, r0,          sa)
        local x1, y1   = rot_xy(CX, CY, OUTER_R - 1, sa)
        local tcol
        if is_card then
            tcol = lcd.RGB(88, 136, 104)
        elseif is_major then
            tcol = lcd.RGB(64, 104,  80)
        else
            tcol = lcd.RGB(48,  88,  64)
        end
        lcd.drawLine(x0, y0, x1, y1, FSOLID, tcol)
    end

    -- Cardinal labels (positions rotate, text stays upright)
    local cards = {
        {   0, "N", lcd.RGB(232, 64, 64), FXS  },
        {  90, "E", lcd.RGB(154,176,184), FXXS },
        { 180, "S", lcd.RGB(154,176,184), FXXS },
        { 270, "W", lcd.RGB(154,176,184), FXXS },
    }
    for _, c in ipairs(cards) do
        local lx, ly = rot_xy(CX, CY, LABEL_R, c[1] - heading)
        local hw = (c[4] == FXS) and 4 or 3
        lcd.drawText(lx - hw, ly - 5, c[2], bor(c[4], c[3]))
    end

    -- North arrow: small filled triangle inside ring
    local ax, ay = rot_xy(CX, CY, INNER_R - 8,  -heading)
    local bx, by = rot_xy(CX, CY, INNER_R - 18, -heading - 8)
    local dx, dy = rot_xy(CX, CY, INNER_R - 18, -heading + 8)
    filled_tri(ax, ay, bx, by, dx, dy, lcd.RGB(208, 48, 48))

    -- Velocity line (always vertical, mower heading = up)
    local line_len = math.floor(clamp(speed_kmh / MAX_VEL, 0, 1) * INNER_R + 0.5)
    if line_len > 3 then
        lcd.drawLine(CX, CY, CX, CY - line_len, FSOLID, C_CYAN)
        lcd.drawLine(CX - 3, CY - line_len + 6, CX, CY - line_len, FSOLID, C_CYAN)
        lcd.drawLine(CX + 3, CY - line_len + 6, CX, CY - line_len, FSOLID, C_CYAN)
    end

    -- Mower rect and uncertainty circle
    local mx = CX - math.floor(MW / 2)
    local my = CY - math.floor(MH / 2)

    lcd.drawFilledRectangle(mx, my, MW, MH, lcd.RGB(32, 72, 32))

    -- Uncertainty circle — scaled to mower physical dimensions.
    -- radius_px = ekf_mm * MM_TO_PX  (MW px = 500 mm)
    -- Sub-50 mm: show a fixed 100 mm-diameter filled white dot (≈4 px radius)
    --            rather than an invisible 2 px ring.
    local ucol = uncert_col(ekf_mm)
    if ekf_mm < 50 then
        -- Excellent accuracy: solid white filled circle, 100 mm diameter at scale
        -- = 50 mm radius * MM_TO_PX ≈ 2 px, clamped to minimum 4 px to be visible
        local dot_r = math.max(4, math.floor(50 * MM_TO_PX + 0.5))
        lcd.drawFilledCircle(CX, CY, dot_r, lcd.RGB(255, 255, 255))
    else
        local ur = clamp(
            math.floor(ekf_mm * MM_TO_PX + 0.5),
            3, INNER_R - 5)
        lcd.drawCircle(CX, CY, ur,     ucol)
        lcd.drawCircle(CX, CY, ur - 1, ucol)
    end

    local border_col
    if     is_collision then border_col = lcd.RGB(255, 32, 32)
    elseif is_bog       then border_col = C_ORANGE
    elseif fix_type == 4 then border_col = C_GREEN
    elseif fix_type == 5 then border_col = C_AMBER
    else                      border_col = C_RED
    end

    lcd.drawRectangle(mx, my, MW, MH, border_col)
    filled_tri(CX, my - 4, CX - 3, my, CX + 3, my, border_col)
    lcd.drawFilledCircle(CX, CY, 2, lcd.RGB(64, 104, 72))
end

-- ═══════════════════════════════════════════════════════════════════════
--  HEADER
-- ═══════════════════════════════════════════════════════════════════════

local function draw_header(state_str, armed, blade_on, tx_voltage)
    lcd.drawFilledRectangle(0, 0, SCR_W, HDR_H, lcd.RGB(18, 44, 18))
    lcd.drawLine(0, HDR_H, SCR_W, HDR_H, FSOLID, C_BORDER)

    local sc = STATE_COL[state_str] or STATE_DEF
    lcd.drawFilledRectangle(4, 4, 118, 22, sc[1])
    lcd.drawText(63, 3, state_str, bor(FS, FCENT, sc[2]))

    -- TX battery indicator — centred in the gap between state badge and ARMED badge
    -- RadioMaster TX16S: 2S LiPo, 8.4 V full, 6.6 V empty
    local TX_FULL  = 8.4
    local TX_EMPTY = 6.6
    local tx_pct   = clamp(math.floor((tx_voltage - TX_EMPTY) / (TX_FULL - TX_EMPTY) * 100 + 0.5), 0, 100)
    local tx_col
    if     tx_pct >= 50 then tx_col = lcd.RGB(80, 200, 112)   -- green: >7.5 V
    elseif tx_pct >= 25 then tx_col = C_AMBER                 -- amber: 7.05–7.5 V
    else                     tx_col = C_RED                   -- red:   <7.05 V
    end
    local tx_str = string.format("%.1fV", tx_voltage)
    -- Layout: "TX" label | bar (56px) | voltage  — centred at x≈223 in the gap
    lcd.drawText(169, 7,  "TX",    bor(FXXS, C_DIM))
    draw_bar(187, 11, 56, 8, tx_pct, tx_col)
    lcd.drawText(249, 7,  tx_str, bor(FXXS, tx_col))

    local arm_bg = armed    and lcd.RGB(24, 64, 88) or lcd.RGB(48, 48, 48)
    local arm_fg = armed    and C_CYAN              or lcd.RGB(96, 96, 96)
    lcd.drawFilledRectangle(322, 5, 72, 20, arm_bg)
    lcd.drawText(358, 7, "ARMED", bor(FXS, FCENT, arm_fg))

    local bld_bg = blade_on and lcd.RGB(80, 48,  0) or lcd.RGB(48, 48, 48)
    local bld_fg = blade_on and C_AMBER             or lcd.RGB(96, 96, 96)
    lcd.drawFilledRectangle(400, 5, 72, 20, bld_bg)
    lcd.drawText(436, 7, "BLADE", bor(FXS, FCENT, bld_fg))
end

-- ═══════════════════════════════════════════════════════════════════════
--  LEFT PANEL
-- ═══════════════════════════════════════════════════════════════════════

local function draw_left_panel(fix_type, ekf_mm, batt_pct, blade_pct, cut_mm)
    local px = 4
    local bw = L_W - 8
    local rx = L_W - 4

    -- GPS / RTK ──────────────────────────────────────────────────────────
    draw_sh(px, BODY_Y + 4, bw, "GPS / RTK")
    local fc = FIX_COL[fix_type] or FIX_DEF
    lcd.drawFilledRectangle(px, BODY_Y + 20, bw, 14, fc[1])
    lcd.drawText(px + 3, BODY_Y + 18, fc[3], bor(FXXS, fc[2]))
    lcd.drawText(px,  BODY_Y + 38, "ACC:", bor(FXXS, C_DIM))
    local ekf_str = "+-" .. tostring(ekf_mm) .. "mm"
    lcd.drawText(rx, BODY_Y + 38, ekf_str, bor(FXXS, FRGHT, uncert_col(ekf_mm)))

    -- POWER ───────────────────────────────────────────────────────────────
    draw_sh(px, BODY_Y + 62, bw, "POWER")
    lcd.drawText(px, BODY_Y + 80, "BATTERY", bor(FXXS, C_DIM))
    local bfg = (batt_pct < 20) and C_RED or C_WHITE
    lcd.drawText(rx, BODY_Y + 80, tostring(batt_pct) .. "%", bor(FXXS, FRGHT, bfg))
    draw_bar(px, BODY_Y + 98, bw, 10, batt_pct, batt_col(batt_pct))

    -- BLADE ───────────────────────────────────────────────────────────────
    draw_sh(px, BODY_Y + 118, bw, "BLADE")
    lcd.drawText(px, BODY_Y + 136, "LOAD", bor(FXXS, C_DIM))
    local lfg = (blade_pct > 85) and C_RED or C_WHITE
    lcd.drawText(rx, BODY_Y + 136, tostring(blade_pct) .. "%", bor(FXXS, FRGHT, lfg))
    draw_bar(px, BODY_Y + 154, bw, 10, blade_pct, blade_col(blade_pct))

    lcd.drawText(px, BODY_Y + 176, "CUT", bor(FXXS, C_DIM))
    lcd.drawText(rx, BODY_Y + 176, tostring(cut_mm) .. "mm", bor(FXXS, FRGHT, C_WHITE))
    local cut_pct = (cut_mm - 20) * 100 / 80
    draw_bar(px, BODY_Y + 194, bw, 10, cut_pct, lcd.RGB(96, 112, 128))
end

-- ═══════════════════════════════════════════════════════════════════════
--  RIGHT PANEL
-- ═══════════════════════════════════════════════════════════════════════

local function draw_right_panel(state_str)
    local px = L_W + C_W + 4
    local bw = R_W - 8
    local sa, sb, sc_val = infer_switches(state_str)

    draw_sh(px, BODY_Y + 4, bw, "SW CONFIRM")

    local function draw_switch(y0, code, func_name, names, active)
        lcd.drawText(px,      y0, code,      bor(FXXS, C_DIM))
        lcd.drawText(px + 18, y0, func_name, bor(FXXS, C_DIM))
        for i, label in ipairs(names) do
            local ly = y0 + 14 + (i - 1) * 16
            if (i - 1) == active then
                lcd.drawFilledRectangle(px, ly, bw, 15, lcd.RGB(32, 64, 88))
                lcd.drawFilledCircle(px + 5, ly + 7, 2, C_CYAN)
                lcd.drawText(px + 12, ly - 1, label, bor(FXXS, C_WHITE))
            else
                lcd.drawFilledCircle(px + 5, ly + 7, 2, lcd.RGB(56, 96, 56))
                lcd.drawText(px + 12, ly - 1, label, bor(FXXS, lcd.RGB(96, 128, 96)))
            end
        end
    end

    draw_switch(BODY_Y + 22,  "SA", "PAUSE", {"RUN",    "PAUSE"},          sa)
    draw_switch(BODY_Y + 84,  "SB", "MODE",  {"MANUAL", "AUTO", "RETURN"}, sb)
    draw_switch(BODY_Y + 162, "SC", "LEARN", {"OFF",    "LEARN"},          sc_val)
end

-- ═══════════════════════════════════════════════════════════════════════
--  VESC STATUS BAR
-- ═══════════════════════════════════════════════════════════════════════

local function vesc_state(is_blade, state_str, blade_on, is_bog)
    if is_blade then
        return blade_on and VESC_COL.FWD or VESC_COL.IDLE
    end
    if state_str == "MOT-OFF" or state_str == "INIT"
    or state_str == "IDLE"    or state_str == "PAUSED" then
        return VESC_COL.IDLE
    end
    if is_bog then return VESC_COL.FAULT end
    return VESC_COL.FWD
end

local function draw_vesc_bar(state_str, blade_on, is_bog)
    lcd.drawLine(0, VESC_Y, SCR_W, VESC_Y, FSOLID, C_BORDER)
    local panels = {
        { vx = 0,   label = "L DRIVE", is_blade = false },
        { vx = 160, label = "R DRIVE", is_blade = false },
        { vx = 320, label = "BLADE",   is_blade = true  },
    }
    for _, p in ipairs(panels) do
        local vc = vesc_state(p.is_blade, state_str, blade_on, is_bog)
        lcd.drawFilledRectangle(p.vx, VESC_Y + 1, 160, VESC_H - 1, vc[1])
        if p.vx > 0 then
            lcd.drawLine(p.vx, VESC_Y, p.vx, SCR_H, FSOLID, C_BORDER)
        end
        lcd.drawText(p.vx + 4,   VESC_Y + 2,  p.label, bor(FXXS, C_DIM))
        lcd.drawText(p.vx + 4,   VESC_Y + 15, vc[4],   bor(FXXS, vc[2]))
        lcd.drawText(p.vx + 156, VESC_Y + 15, vc[5],   bor(FXXS, FRGHT, vc[3]))
    end
end

local function draw_dividers()
    lcd.drawLine(L_W,       BODY_Y, L_W,       VESC_Y, FSOLID, C_BORDER)
    lcd.drawLine(L_W + C_W, BODY_Y, L_W + C_W, VESC_Y, FSOLID, C_BORDER)
end

-- ═══════════════════════════════════════════════════════════════════════
--  WIDGET LIFECYCLE
-- ═══════════════════════════════════════════════════════════════════════

local function create(zone, opts)
    init_colors()   -- Safe: lcd.RGB() only called here, after display init
    return {
        cut_mm     = 35,
        blade_load = 0,
        fix_type   = 0,
        flags      = 0,
        prev_flags = 0,   -- used to detect rising edge on beep bits 7:6
        direct_decode = false,  -- true once extended 0x80 payload (>=19 B) seen
        calib         = 0,      -- packed BNO calibration byte (0x80 offset 19)
        have_calib    = false,  -- true once a >=20 B payload has been seen
        state_str  = "INIT",
        ekf_mm     = 9999,  -- EKF position uncertainty mm, from MOWER_STATUS offsets 9-10
        voltage    = 0.0,
        tx_voltage = 0.0,   -- transmitter battery (getValue 'tx-voltage')
        heading    = 0,
        speed_kmh  = 0.0,
    }
end

local function update(widget, opts)
end

-- State enum → name lookup (matches MOWER_STATUS offset 0 values)
local STATE_NAMES = {
    [0]  = "INIT",
    [1]  = "IDLE",
    [2]  = "MANUAL",
    [3]  = "LEARN",
    [4]  = "AUTO",
    [5]  = "RETRACE",
    [6]  = "BOG",
    [7]  = "OBS-AVOID",
    [8]  = "RETURN",
    [9]  = "PAUSED",
    [10] = "MOT-OFF",
}

local function parse_telemetry(widget)
    local cmd, data = crossfireTelemetryPop()
    while cmd do
        -- 0x1E FLIGHT_MODE is consumed by EdgeTX internally — never arrives here.
        -- State is now carried in MOWER_STATUS offset 0 as a uint8 enum.

        if cmd == 0x80 and data and #data >= 15 then
            -- MOWER_STATUS payload (Lua data[] is 1-based, spec offsets are 0-based)
            -- data[1]       = offset 0  : state enum
            -- data[2]       = offset 1  : headland progress %  (not displayed)
            -- data[3]       = offset 2  : strip progress %     (not displayed)
            -- data[4]       = offset 3  : cut height mm
            -- data[5]       = offset 4  : blade load %
            -- data[6]       = offset 5  : GPS fix type
            -- data[7]       = offset 6  : status flags
            -- data[8-9]     = offset 7-8: obstacle count       (not displayed)
            -- data[10-11]   = offset 9-10: EKF uncertainty cm  (big-endian uint16)
            -- data[12-15]   = offset 11-14: mowed area dm²     (not displayed)
            local state_enum = data[1] or 0
            widget.state_str  = STATE_NAMES[state_enum] or "?"
            widget.cut_mm     = data[4] or widget.cut_mm
            widget.blade_load = data[5] or 0
            widget.fix_type   = data[6] or 0
            widget.flags      = data[7] or 0
            -- EKF uncertainty: uint16 big-endian in cm, convert to mm for display
            local ekf_cm = bit32.bor(bit32.lshift(data[10] or 0, 8), data[11] or 0)
            widget.ekf_mm = ekf_cm * 10

            -- Extended payload (firmware >= 2026-06-10): battery voltage and
            -- heading carried directly in this frame, so the display works even
            -- when the EdgeTX RxBt / Yaw sensors are missing or stale.
            -- data[16-17] = offset 15-16: battery V ×100 (uint16 BE)
            -- data[18-19] = offset 17-18: heading deg ×10 (uint16 BE)
            if #data >= 19 then
                widget.voltage = bit32.bor(bit32.lshift(data[16] or 0, 8), data[17] or 0) / 100
                widget.heading = bit32.bor(bit32.lshift(data[18] or 0, 8), data[19] or 0) / 10
                widget.direct_decode = true
            end
            -- Calibration byte (firmware >= 2026-06-14): BNO sys/gyro/accel/mag.
            -- data[20] = offset 19: bits 7:6 sys, 5:4 gyro, 3:2 accel, 1:0 mag.
            if #data >= 20 then
                widget.calib      = data[20] or 0
                widget.have_calib = true
            end
        end
        cmd, data = crossfireTelemetryPop()
    end
end

local function background(widget)
    parse_telemetry(widget)
end

local function refresh(widget, event, touchState)
    parse_telemetry(widget)

    -- Fallback only: with extended-payload firmware (>= 2026-06-10) battery and
    -- heading come straight from the 0x80 frame in parse_telemetry above.
    if not widget.direct_decode then
        local v = getValue('RxBt')
        if type(v) == "number" then widget.voltage    = v end
    end

    -- TX battery voltage. Source name 'tx-voltage' is standard in EdgeTX.
    -- If this reads 0 constantly, check System > Hardware > Battery Calibration
    -- and confirm the source name via getFieldInfo('tx-voltage') on your build.
    local txv = getValue('tx-voltage')
    if type(txv) == "number" then widget.tx_voltage = txv end

    -- Heading from CRSF Attitude frame yaw, exposed by EdgeTX as "Yaw".
    -- EdgeTX returns the value already converted to radians (not the raw ×10000 int16).
    -- Convert to degrees and wrap to [0, 360).
    -- Fallback only — see direct decode from MOWER_STATUS above.
    if not widget.direct_decode then
        local yaw_raw = getValue('Yaw')
        if type(yaw_raw) == "number" then
            local hdg = math.deg(yaw_raw)
            if hdg < 0 then hdg = hdg + 360 end
            widget.heading = hdg
        end
    end

    local spd = getValue('GSpd')
    if type(spd) == "number" then widget.speed_kmh = spd end

    local batt_pct = clamp(
        math.floor((widget.voltage - 42.9) / (54.6 - 42.9) * 100 + 0.5),
        0, 100)

    local ft     = widget.fix_type
    local ekf_mm = widget.ekf_mm   -- real value from telemetry, in mm

    local flags    = widget.flags
    local armed    = bit32.band(flags, 0x01) ~= 0
    local blade_on = bit32.band(flags, 0x02) ~= 0
    local is_bog   = bit32.band(flags, 0x04) ~= 0 or widget.state_str == "BOG"
    local is_coll  = widget.state_str == "OBS-AVOID"

    -- Beep request: bits 7:6 of flags (0=none, 1=confirm, 2=warning, 3=fault)
    -- Trigger only on rising edge so a sustained flag doesn't loop.
    local beep_now = bit32.band(bit32.rshift(flags, 6), 0x03)
    local beep_was = bit32.band(bit32.rshift(widget.prev_flags, 6), 0x03)
    if beep_now ~= 0 and beep_now ~= beep_was then
        play_beep(beep_now)
    end
    widget.prev_flags = flags

    lcd.drawFilledRectangle(0, 0, SCR_W, SCR_H, C_BG)
    draw_header(widget.state_str, armed, blade_on, widget.tx_voltage)
    draw_left_panel(ft, ekf_mm, batt_pct, widget.blade_load, widget.cut_mm)
    draw_compass(widget.heading, widget.speed_kmh, ekf_mm, ft, is_bog, is_coll)
    draw_right_panel(widget.state_str)
    draw_vesc_bar(widget.state_str, blade_on, is_bog)
    draw_dividers()

    -- Battery warning banner (MOWER_STATUS flags bit 0x20): the firmware
    -- raises this on BATTERY WARNING/LOW. Flash ~1 Hz over the bottom strip.
    -- No automatic return — the operator chooses; firmware repeats a warning
    -- beep every 60 s via the beep bits while the condition persists.
    if bit32.band(flags, 0x20) ~= 0 then
        if math.floor(getTime() / 50) % 2 == 0 then   -- getTime() = 10 ms ticks
            lcd.drawFilledRectangle(0, SCR_H - 22, SCR_W, 22, C_RED)
            lcd.drawText(SCR_W / 2, SCR_H - 20,
                string.format("BATTERY LOW  %.1fV  -  RETURN?", widget.voltage or 0),
                FXS + FCENT + C_WHITE)
        end
    end
end

return {
    name       = name,
    options    = options,
    create     = create,
    update     = update,
    background = background,
    refresh    = refresh,
}
