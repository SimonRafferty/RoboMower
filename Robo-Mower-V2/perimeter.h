#pragma once
#include <Arduino.h>
#include "geometry.h"
#include "nvs_storage.h"

// ══════════════════════════════════════════════════════════════════════════════
//  perimeter.h — RoboMower perimeter recording and polygon management
//
//  Manages three progressively inset polygons derived from the manually driven
//  boundary:
//
//    PERIMETER       — raw recorded boundary (outermost cut line)
//    NAV_BOUNDARY    — inset by NAV_EXCLUSION_INSET_M (steering centre must
//                      remain inside; used by safety and path controller)
//    WORKING_AREA    — nav boundary inset by HEADLAND_WIDTH_M (strip targets)
//
//  Recording is SPARSE and independent of fix quality (fix_type is ignored):
//  in LEARN mode the operator presses the record button at each corner and a
//  single node is committed on that press (force=true, which bypasses the
//  0.2 m distance gate — that gate applies only to non-forced calls, which
//  LEARN does not use). A maximum of MAX_PERIMETER_POINTS waypoints may be
//  recorded.
//
//  On successful perimeter_finish_recording() all three polygons are saved to
//  NVS. On boot, perimeter_init() re-loads them and reports unreachable zones.
//
//  References:
//    Spec: Robo_Mower_claudecode_prompt_v3.md — "Perimeter Recording" section
//    config.h: NAV_EXCLUSION_INSET_M, HEADLAND_WIDTH_M, MAX_PERIMETER_POINTS,
//              PERIMETER_CLOSE_WINDOW, RTK_MIN_FIX_FOR_LEARNING
//    HANDOFFS/13_perimeter/HANDOFF.md
// ══════════════════════════════════════════════════════════════════════════════


// ── Module lifecycle ─────────────────────────────────────────────────────────

/**
 * @brief Initialise the perimeter module.
 *
 * Attempts to load all three polygons (perimeter, nav boundary, working area)
 * from NVS. If all three load successfully, sets the internal valid flag and
 * calls perimeter_log_unreachable_zones() to print corner regions that the
 * robot cannot reach. Safe to call before NVS storage is initialised — this
 * function relies only on the nvs_load_* / nvs_has_valid_perimeter() API.
 */
void perimeter_init();


// ── Recording API ─────────────────────────────────────────────────────────────

/**
 * @brief Begin recording a new perimeter polygon.
 *
 * Clears any previously accumulated recording buffer and resets the distance
 * accumulator. Any previously recorded-but-unsaved polygon is discarded.
 * Does NOT clear the polygons already stored in NVS — call perimeter_clear()
 * first if you want to erase the old perimeter before starting.
 */
void perimeter_start_recording();

/**
 * @brief Offer a new GPS position to the recording buffer.
 *
 * @param x         Steering centre ENU east coordinate (metres).
 * @param y         Steering centre ENU north coordinate (metres).
 * @param fix_type  Current GPS fix type (0=none … 4=RTK fixed).
 * @param force     If true, bypass the 0.2 m distance gate (use for explicit
 *                  button-press recording where the user chooses each point).
 * @return true  A new waypoint was appended; false otherwise.
 */
bool perimeter_record_point(float x, float y, int fix_type, bool force = false);

/**
 * @brief Finish recording: close, validate, derive polygons, save to NVS.
 *
 * Validation checks (in order):
 *   1. point_count >= 3
 *   2. Polygon area >= 5.0 m² (shoelace formula via Polygon::area())
 *   3. Polygon is not self-intersecting (isSelfIntersecting())
 *   4. After NAV_EXCLUSION_INSET_M inset, a valid (non-empty) polygon remains
 *
 * On success:
 *   - Polygon wound CCW (ensureCCW())
 *   - nav_boundary = insetPolygonClipper(perimeter, mower_config_nav_inset_m())
 *   - working_area = insetPolygonClipper(nav_boundary, mower_config_headland_m())
 *   - All three saved to NVS via nvs_save_perimeter / _nav_boundary / _working_area
 *   - Internal valid flag set; recording state cleared
 *
 * @param[out] error_msg  On failure, filled with a null-terminated reason
 *                        string (maximum 64 bytes including the null terminator).
 *                        Unchanged on success.
 * @return true  Validation passed and polygons saved.
 * @return false Validation failed; reason written to error_msg.
 */
bool perimeter_finish_recording(char *error_msg);

/**
 * @brief Abort recording without saving.
 *
 * Clears the recording buffer and resets all recording state. No NVS changes.
 */
void perimeter_abort_recording();

/**
 * @brief Save partial recording (< 3 points) to NVS for map display.
 *
 * Saves whatever points are in the recording buffer to NVS as a raw perimeter.
 * Does NOT set the perimeter as valid for navigation — only for map display
 * and later editing via the WebUI map editor.  Clears recording state.
 */
void perimeter_save_partial();

/**
 * @brief Query whether a recording session is currently active.
 * @return true if perimeter_start_recording() has been called and
 *         perimeter_finish_recording() or perimeter_abort_recording() has not.
 */
bool perimeter_is_recording();

/**
 * @brief Return the number of waypoints accumulated in the current recording.
 * @return Point count (0 if not recording or nothing recorded yet).
 */
int perimeter_recording_point_count();

/**
 * @brief Return the in-progress recording points as a Polygon.
 * @return Polygon with recorded points (empty if not recording or no points yet).
 */
Polygon perimeter_get_recording_points();


// ── Stored polygon access ─────────────────────────────────────────────────────

/**
 * @brief Query whether a valid perimeter set is available.
 *
 * Returns true if polygons were loaded successfully from NVS on boot OR if
 * perimeter_finish_recording() succeeded in this session.
 *
 * @return true  All three polygons are available for use.
 */
bool perimeter_is_valid();

/**
 * @brief Return the raw perimeter polygon (outermost cut boundary as recorded).
 *
 * This is the polygon the user drove the blade edge along. It is the outermost
 * of the three polygons. Returns an empty Polygon if !perimeter_is_valid().
 *
 * @return Perimeter polygon (CCW winding).
 */
Polygon perimeter_get_perimeter();

/**
 * @brief Return the navigation boundary polygon.
 *
 * Inset from the raw perimeter by NAV_EXCLUSION_INSET_M. The robot's steering
 * centre must stay inside this polygon at all times. Used by the safety module
 * for perimeter breach detection and by the path controller as a hard constraint.
 *
 * @return Nav boundary polygon (CCW winding), or empty if !perimeter_is_valid().
 */
Polygon perimeter_get_nav_boundary();

/**
 * @brief Return the working area polygon.
 *
 * Inset from the nav boundary by HEADLAND_WIDTH_M. Vestigial under the spiral
 * planner — still derived/displayed but not used for navigation. Returns an
 * empty Polygon if !perimeter_is_valid().
 *
 * @return Working area polygon (CCW winding), or empty if !perimeter_is_valid().
 */
Polygon perimeter_get_working_area();

/**
 * @brief Erase all stored perimeter data from NVS and clear in-memory state.
 *
 * Calls nvs_clear_perimeter() and resets the internal valid flag plus all
 * three in-memory polygons. Should be called before starting a new learning
 * session if a valid perimeter is already stored.
 */
void perimeter_clear();

/**
 * @brief Recompute nav boundary and working area from the saved raw perimeter.
 *
 * Uses the current mower_config inset values, re-saves derived polygons to NVS,
 * and updates in-memory state. Call after mower_config_set() to apply new
 * chassis dimensions to an existing perimeter without re-learning it.
 *
 * Does nothing if no valid perimeter is stored in NVS.
 */
void perimeter_recompute();

/**
 * @brief Log unreachable zones to Serial.
 *
 * Scans the interior of the perimeter at a coarse 1 m grid. Grid cells that
 * lie inside the perimeter polygon but outside the nav boundary polygon are
 * "unreachable" — the robot's chassis will clip the boundary before the
 * steering centre reaches them.
 *
 * Nearby unreachable cells are grouped into clusters (radius 2 m). For each
 * cluster the approximate centre and area are printed:
 *   [PERIM] Unreachable zone ~(x, y), approx area X m²
 *
 * No-op if !perimeter_is_valid() or the nav boundary covers the full interior.
 */
void perimeter_log_unreachable_zones();
