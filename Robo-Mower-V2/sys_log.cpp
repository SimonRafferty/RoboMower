#include "sys_log.h"
#include "config.h"
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

static char         s_buf[SYS_LOG_MAX_ENTRIES][SYS_LOG_MAX_LEN];
static int          s_head  = 0;   // next write slot
static int          s_count = 0;   // entries stored (0 – MAX)
static portMUX_TYPE s_mux   = portMUX_INITIALIZER_UNLOCKED;

void sys_log_push(const char *msg) {
    portENTER_CRITICAL(&s_mux);
    snprintf(s_buf[s_head], SYS_LOG_MAX_LEN, "%s", msg);
    s_head = (s_head + 1) % SYS_LOG_MAX_ENTRIES;
    if (s_count < SYS_LOG_MAX_ENTRIES) s_count++;
    portEXIT_CRITICAL(&s_mux);
    DBG_PRINTF("[LOG] %s\n", msg);
}

int sys_log_count() {
    return s_count;
}

const char *sys_log_get(int index) {
    if (index < 0 || index >= s_count) return "";
    // oldest entry is at s_head when buffer is full, 0 otherwise
    int oldest = (s_count < SYS_LOG_MAX_ENTRIES) ? 0 : s_head;
    int slot   = (oldest + index) % SYS_LOG_MAX_ENTRIES;
    return s_buf[slot];
}
