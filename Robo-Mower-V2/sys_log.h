#pragma once
#include <Arduino.h>

// ══════════════════════════════════════════════════════════════════════════════
//  sys_log.h — System event log ring buffer
//
//  Stores the last SYS_LOG_MAX_ENTRIES text messages (e.g. boot errors,
//  module faults) for retrieval via BLE STATUS JSON.
//  Also echoes every message to Serial.
//
//  Not thread-safe — call only from Core 1 (setup / state machine).
// ══════════════════════════════════════════════════════════════════════════════

#define SYS_LOG_MAX_ENTRIES  30
#define SYS_LOG_MAX_LEN      80

/** Push a message into the ring buffer (oldest entry is overwritten when full).
 *  Also prints to Serial with a [LOG] prefix.
 *  Thread-safe — may be called from any task or core. */
void sys_log_push(const char *msg);

/** Number of entries currently stored (0 – SYS_LOG_MAX_ENTRIES). */
int sys_log_count();

/** Get entry at @p index where 0 = oldest, count-1 = newest.
 *  Returns "" if @p index is out of range. */
const char *sys_log_get(int index);
