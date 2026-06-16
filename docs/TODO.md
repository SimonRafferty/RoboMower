# Active TODO / Investigation Items

Work-tickets and open investigations. Steady-state facts about how the firmware
*currently* behaves live in `CLAUDE.md` (Known Issues → *Disabled / vestigial*
and *Latent traps*); this file is for things that still need doing.

Last updated: 2026-06-16.

Implementation plan / design: `C:\Users\simon\.claude\plans\nested-rolling-sifakis.md`.
Pre-change source snapshot: `Backups/2026-06-16/`.

---

## Pending verification (implemented 2026-06-16, not yet hardware/field-tested)

### 1. User Setting Save (PWA) — DONE (browser test only)
Config-only "Save to File" / "Load from File" buttons in the PWA config tab
(`robomower-pwa/robomower.html`). Exports the 38 MowerConfig fields keyed by name
to `robomower-config.json`; import fills missing keys from `CFG_DEFAULTS`
(mirrors `config.h`) and ignores unknown keys, then the operator presses
"Save to Mower". No firmware change (the firmware already parses config by name).
**Verify:** in Chrome/Android — export, hand-edit (remove a key, add junk), import,
confirm the toast counts and that "Save to Mower" applies.

### 2. Learn-point latency — DONE (needs bench test)
CH8 press is now edge-latched in `crsf_input.cpp` at the CRSF frame rate (~200 Hz)
into a counter the 10 Hz FSM drains (`crsf_get_learn_pt_events()`), so brief taps
are no longer dropped. **Verify:** bench, `DEBUG_SERIAL 1`, enter LEARN, rapid
double-taps of CH8 → every tap logs `[LEARN] Point N stored` + a beep.

### 3. GPS / Map inconsistency — DONE (needs field test)
Perimeter is now stored canonically as absolute WGS-84 lat/lon + per-point
accuracy (NVS `perim2`) and re-derived to ENU on boot against the current origin,
so it cannot drift relative to the live fix (root fix for the power-up mismatch).
Legacy ENU perimeters auto-migrate on first boot. Breach margin is confidence-aware
(`safety.cpp`: keep-out widened by perimeter accuracy, floored at 0.10 m) so a
low-confidence perimeter is treated more cautiously — the mower must never exceed
the perimeter. The BLE/PWA protocol is unchanged (firmware ENU is now
origin-consistent). Math validated offline: `host_test/perim_latlon_test.cpp`
(0 failures; origin-shift drift 0.1 mm). **Verify in field:** record a perimeter,
reboot, confirm the map perimeter and live RTK dot align; confirm an existing
perimeter migrates without loss; confirm breach widens for a low-confidence point.

### 4. BNO055 calibration not persisting to NVS — DONE (needs hardware test)
Save is now **read-back verified** (`save_cal_to_nvs` re-opens and `memcmp`s);
`s_profile_saved` is only set on a verified write, else it retries. New
diagnostics avoid blind recalibration of the chassis-mounted sensor:
- `IMUCALTEST` (serial / PWA "Test cal save") — proves the NVS path stores+reads
  22 bytes **without** recalibrating.
- `IMUSAVECAL` (serial / PWA "Save cal now") — saves the live calibration on Core 0
  (I2C-safe), verified, reported to the log; refused unless fully calibrated.
- Boot logs whether `bnocal` loaded + a checksum (compare across boots).
- EKF boot guard (`ekf_localiser.cpp`): if no cal profile loaded, start the heading
  offset at 0 (BNO heading is relative) instead of trusting a stale saved offset;
  per the chosen design the offset is still persisted once cal works.
**Verify on hardware:** run `IMUCALTEST` → expect PASS (no recal). Then one
supervised recal → reboot → confirm `bnocal` loads (boot checksum stable) and the
heading reads absolute (agrees with GPS on a straight RTK run).

---

## Still open / follow-ups

- **GPS-heading-disagreement warning:** the EKF still trims a GPS heading offset
  (the "apply a GPS offset" half of the heading note) and the boot guard handles a
  lost profile, but an explicit "BNO disagrees with GPS → cal likely lost" warning
  was deferred — a naive magnitude check false-positives on legitimate mounting
  offset. Revisit with an offset-instability heuristic if needed.
- **Per-point accuracy fidelity:** the breach uses the worst-case (max) accuracy
  across perimeter points (functionally identical for the keep-out), and the
  canonical store currently tags all points with that value. True per-point
  accuracy could be preserved if a future need justifies the polygon-cleaning
  re-index work.
