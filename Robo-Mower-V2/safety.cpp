// ══════════════════════════════════════════════════════════════════════════════
//  safety.cpp — RoboMower Safety Watchdog and E-stop Module
//
//  See safety.h for full module description and thread-safety notes.
//
//  PHYSICAL E-STOP NOTE (ASSUMPTIONS.md A17):
//    The ESP32 has NO GPIO connected to the hardware mushroom button.
//    The PILZ safety relay cuts main bus power entirely independently.
//    Software E-stop only needs to perform a controlled VESC stop via the
//    normal CAN TX queue.  No portENTER_CRITICAL / hardware queue bypass
//    is used or required.
// ══════════════════════════════════════════════════════════════════════════════

#include "safety.h"
#include "config.h"
#include "crsf_input.h"
#include "vesc_can.h"
#include "imu.h"
#include "ekf_localiser.h"
#include "battery_monitor.h"
#include "rtk_gps.h"
#include "geometry.h"
#include "mower_config.h"   // mower_config_nav_inset_m() for the breach threshold
#include "perimeter.h"      // perimeter_get_accuracy_m() for confidence-aware breach

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ── Task parameters ───────────────────────────────────────────────────────────
// Priority 20 per ARCHITECTURE.md §2 (spec says 24, ARCHITECTURE.md is
// authoritative per ASSUMPTIONS.md A24).
// Stack 4096 bytes per ARCHITECTURE.md §2 (spec says 3072).
// Period 50 ms = 20 Hz.
#define SAFETY_TASK_PRIORITY    20
#define SAFETY_TASK_STACK       4096
#define SAFETY_TASK_PERIOD_MS   50

// ── Module-private state ──────────────────────────────────────────────────────

/** True while any VESC CAN STATUS frame is overdue (PILZ fired or battery disconnected).
 *  Set by safety_task; cleared by safety_clear_motors_offline() from state machine.
 *  When true, all other watchdog checks are suppressed — the VESCs are unpowered. */
static volatile bool s_motors_offline = false;

/** True when battery voltage < BATTERY_WARN_V (overlay flag for LED pattern). */
static volatile bool s_battery_warning = false;

/** True when the safety task should check the perimeter boundary. */
static volatile bool s_in_auto_mode = false;

/** Perimeter polygon for breach detection (covers all regions incl. side arms). */
static Polygon s_perimeter;

/** True once safety_set_perimeter() has been called with a valid polygon. */
static volatile bool s_perimeter_set = false;

/** Mutex protecting s_perimeter (Polygon contains std::vector — not spinlock-safe). */
static SemaphoreHandle_t s_nav_mutex = nullptr;

/** Suppresses repeated GPS-timeout log messages once printed. */
static volatile bool s_gps_timeout_warned = false;

/** Set by safety_task on perimeter breach; polled by state_machine_update(). */
static volatile bool s_pause_requested = false;

/** Set when the blade VESC has been silent for BLADE_VESC_TIMEOUT_MS.
 *  Polled by state_machine_update() to stop the blade only — does NOT
 *  trigger MOTORS_OFFLINE or block any other controls. */
static volatile bool s_blade_vesc_stale = false;

// ── Forward declarations ──────────────────────────────────────────────────────
static void safety_task(void *pv);

// ─────────────────────────────────────────────────────────────────────────────
//  Public API — initialisation
// ─────────────────────────────────────────────────────────────────────────────

void safety_init()
{
    s_nav_mutex = xSemaphoreCreateMutex();
    configASSERT(s_nav_mutex != nullptr);

    xTaskCreatePinnedToCore(
        safety_task,
        "safety_task",
        SAFETY_TASK_STACK,
        nullptr,
        SAFETY_TASK_PRIORITY,
        nullptr,
        1   // Core 1
    );
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public API — arm status
// ─────────────────────────────────────────────────────────────────────────────

bool safety_is_armed()
{
    CRSFChannels ch = crsf_get_channels();
    // ch.ch[] stores values in microseconds (1000–2000 µs).
    // CH6_ARMED_THRESHOLD is in raw CRSF units (496), so convert first.
    // See HANDOFFS/05_crsf_input/HANDOFF.md §1.
    return (!ch.failsafe) &&
           (ch.ch[CRSF_CH_ARM] > crsf_raw_to_us(CH6_ARMED_THRESHOLD));
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public API — perimeter and mode control
// ─────────────────────────────────────────────────────────────────────────────

void safety_set_perimeter(const Polygon &perimeter)
{
    if (xSemaphoreTake(s_nav_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_perimeter = perimeter;
        s_perimeter_set = (perimeter.pts.size() >= 3);
        xSemaphoreGive(s_nav_mutex);
    }
}

void safety_set_auto_mode(bool in_auto)
{
    // volatile bool write — atomic on 32-bit Xtensa LX7
    s_in_auto_mode = in_auto;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public API — battery warning
// ─────────────────────────────────────────────────────────────────────────────

bool safety_is_battery_warning()
{
    return s_battery_warning;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Public API — motors-offline detection
// ─────────────────────────────────────────────────────────────────────────────

bool safety_is_motors_offline()
{
    return s_motors_offline;
}

void safety_clear_motors_offline()
{
    s_motors_offline = false;
}

bool safety_is_pause_requested()
{
    return s_pause_requested;
}

void safety_clear_pause_request()
{
    s_pause_requested = false;
}

bool safety_is_blade_vesc_stale()
{
    return s_blade_vesc_stale;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────────────────────────────────────

// vesc_is_stale() is now the public function in vesc_can.cpp — no local duplicate needed.

// ─────────────────────────────────────────────────────────────────────────────
//  Safety watchdog task (Core 1, priority 20, 50 ms period)
// ─────────────────────────────────────────────────────────────────────────────

static void safety_task(void * /*pv*/)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAFETY_TASK_PERIOD_MS));

        // ── 1. VESC CAN silence → MOTORS_OFFLINE (not EMERGENCY_STOP) ───────────
        // When the PILZ fires or the battery is disconnected, all VESCs lose
        // power and stop transmitting STATUS frames.  This is NOT a firmware
        // fault — sending CAN stop commands to unpowered VESCs is meaningless.
        // Signal MOTORS_OFFLINE and skip all other watchdog checks until the
        // VESCs return.  (ESTOP_VESC_LOST / ESTOP_BLADE_VESC_LOST are kept in
        // the enum for backwards-compatibility but are no longer triggered here.)
        {
            // MOTORS_OFFLINE = PILZ fired or battery disconnected.
            // Conditions (all must be true):
            //   (a) bus has been live (s_bus_live set by first STATUS_1 from any VESC)
            //   (b) BOTH drive VESCs have been seen at least once (last_update_ms != 0)
            //   (c) both are now stale
            // Requiring (b) prevents a false trigger at boot where the blade VESC
            // sends STATUS_1 first (setting s_bus_live) before the drive VESCs have
            // broadcast their first frame. Without (b), after STARTUP_GRACE_MS the
            // drives (last_update_ms==0) look stale and MOT-OFF fires immediately.
            VescStatus l_st = vesc_get_status(VESC_ID_LEFT);
            VescStatus r_st = vesc_get_status(VESC_ID_RIGHT);
            bool drives_were_seen = (l_st.last_update_ms != 0) &&
                                    (r_st.last_update_ms != 0);
            bool drives_stale     = drives_were_seen &&
                                    vesc_is_stale(VESC_ID_LEFT) &&
                                    vesc_is_stale(VESC_ID_RIGHT);
            if (drives_stale) {
                if (!s_motors_offline) {
                    s_motors_offline = true;
                    DBG_PRINTLN("[SAFETY] VESC CAN timeout → MOTORS_OFFLINE");
                }
                // While motors are offline, all other watchdog checks are
                // irrelevant (VESCs are unpowered; perimeter breach cannot
                // be acted on; RC failsafe / blade fault don't matter).
                continue;
            }
        }

        // ── 1b. Blade VESC stale — stop blade, do NOT affect navigation ─────────
        // The blade VESC (RPM control mode) may broadcast STATUS_1 at a lower
        // rate than the drives.  If it goes silent for BLADE_VESC_TIMEOUT_MS,
        // flag it so the state machine stops the blade motor.  This is NOT a
        // MOTORS_OFFLINE event — drives remain fully operational.
        {
            bool blade_stale_now = vesc_blade_is_stale();
            if (blade_stale_now != s_blade_vesc_stale) {
                s_blade_vesc_stale = blade_stale_now;
                if (blade_stale_now) {
                    DBG_PRINTLN("[SAFETY] Blade VESC stale — flagging blade stop");
                } else {
                    DBG_PRINTLN("[SAFETY] Blade VESC back online");
                }
            }
        }

        // ── 2. IMU fault ──────────────────────────────────────────────────────
        // Log once; the EKF continues with GPS-only navigation.
        static bool s_imu_fault_warned = false;
        if (imu_is_fault()) {
            if (!s_imu_fault_warned) {
                s_imu_fault_warned = true;
                DBG_PRINTLN("[SAFETY] IMU fault — EKF running GPS-only");
            }
        } else {
            s_imu_fault_warned = false;
        }

        // -- 4. Perimeter breach (AUTO states only) ----------------------------
        // The recorded PERIMETER is the steering centre's maximum extent (driven
        // body-against-boundary), so the centre is meant to stay inside it. Breach
        // when the centre goes more than PERIMETER_BREACH_DIST_M OUTSIDE the
        // perimeter, i.e. distance-to-perimeter-edge < -PERIMETER_BREACH_DIST_M.
        // (One polygon, so it covers all regions incl. pinched-off side arms.)
        if (s_in_auto_mode && s_perimeter_set) {
            bool  checked   = false;
            float dist      = 0.0f;     // EKF steering-centre signed distance to edge
            float gps_dist  = 0.0f;     // raw RTK steering-centre signed distance to edge
            bool  gps_valid = false;
            // Read the GPS measurement BEFORE taking the nav mutex (rtk_gps has its own
            // lock). enu_east_m/enu_north_m are the STEERING CENTRE — same frame as the
            // perimeter (the steering-centre limit).
            GpsMeasurement gmeas = rtk_gps_get_measurement();
            if (xSemaphoreTake(s_nav_mutex, 0) == pdTRUE) {
                Pose2D pose = ekf_get_pose();
                dist = distanceToNearestEdge(s_perimeter, pose.x, pose.y);
                if (gmeas.valid) {
                    gps_dist  = distanceToNearestEdge(s_perimeter, gmeas.enu_east_m, gmeas.enu_north_m);
                    gps_valid = true;
                }
                checked = true;
                xSemaphoreGive(s_nav_mutex);
            }
            // Confidence-aware margin: a perimeter recorded with low-confidence
            // GPS fixes is less trustworthy, so shrink the allowed overshoot by
            // its accuracy estimate — the mower breaches (stops) sooner and must
            // never exceed the perimeter. Floored at 0.10 m so a high-confidence
            // perimeter keeps essentially the full PERIMETER_BREACH_DIST_M
            // tolerance and the threshold can never collapse to zero.
            float eff_breach_dist = PERIMETER_BREACH_DIST_M - perimeter_get_accuracy_m();
            if (eff_breach_dist < 0.10f) eff_breach_dist = 0.10f;
            const float breach_thresh = -eff_breach_dist;
            static bool s_perim_breach_warned = false;
            // Trip on the EKF position always; trip on the RAW RTK position ONLY when
            // it is RTK-FIXED. Float fixes can be ~2 m off under tree cover, so a raw
            // Float position would false-trip the breach and stop AUTO. In Float the
            // EKF (dead-reckoned from odometry + Fixed snaps) is the trustworthy
            // signal; raw GPS contributes only when Fixed (a lagging-EKF backstop).
            bool gps_fixed = gps_valid && (gmeas.fix_type == GPS_FIX_RTK_FIXED);
            bool breached = (checked   && dist     < breach_thresh) ||
                            (gps_fixed && gps_dist < breach_thresh);
            if (breached) {
                // Hard-stop motors every 50 ms tick while breach persists.
                // Emergency frames go to the front of the VESC TX queue so they
                // arrive before any motion command the state machine may have queued.
                vesc_emergency_stop_all();
                if (!s_perim_breach_warned) {
                    s_perim_breach_warned = true;
                    DBG_PRINTLN("[SAFETY] Perimeter breach — hard stop + requesting pause");
                    s_pause_requested = true;
                }
            } else {
                s_perim_breach_warned = false;
            }
        }

        // ── 5. Battery state ──────────────────────────────────────────────────
        BatteryState bat = battery_get_state();

        if (bat == BATTERY_LOW || bat == BATTERY_WARNING) {
            // Warning: set flag for state machine LED overlay — do NOT E-stop.
            if (!s_battery_warning) {
                s_battery_warning = true;
                DBG_PRINTLN("[SAFETY] Battery warning: voltage < BATTERY_WARN_V");
            }
        } else {
            // Battery recovered above warning level (e.g., measurement noise
            // or a filter transient) — clear the flag.
            s_battery_warning = false;
        }

        // ── 7. GPS timeout ────────────────────────────────────────────────────
        // GPS timeout does NOT trigger an E-stop.  Warn once and suppress
        // repeat messages until GPS recovers.
        if (rtk_gps_is_timeout()) {
            if (!s_gps_timeout_warned) {
                s_gps_timeout_warned = true;
                DBG_PRINTLN("[SAFETY] GPS timeout — navigation uncertainty will increase");
                // Note: setting GPS weight to 0 in the EKF is not possible via
                // the current public ekf_localiser API.  The GPS task naturally
                // stops calling ekf_update_gps() when no new fixes arrive,
                // which has the same practical effect.  Documented in HANDOFF.
            }
        } else {
            // GPS recovered — allow warning to be re-issued if it drops again
            s_gps_timeout_warned = false;
        }
    }
}
