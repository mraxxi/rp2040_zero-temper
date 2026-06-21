#include "telemetry.h"
#include "common.h"
#include "config.h"
#include <Arduino.h>

// ─── State Name Helper ────────────────────────────────────────────────────────

static const char *state_name(SystemState s) {
    switch (s) {
        case STATE_IDLE:             return "IDLE";
        case STATE_PREHEAT:          return "PREHEAT";
        case STATE_PREHEAT_RUNNING:  return "PREHEAT_RUN";
        case STATE_BGA_REWORK:       return "BGA_START";
        case STATE_BGA_RUNNING:      return "BGA_RUN";
        case STATE_COOLING:          return "COOLING";
        case STATE_COMPLETE:         return "COMPLETE";
        case STATE_ABORT:            return "ABORT";
        case STATE_ERROR:            return "ERROR";
        default:                     return "UNKNOWN";
    }
}

// ─── Telemetry Tick ───────────────────────────────────────────────────────────
//
// Emits one CSV DATA line. Call this from tick_sensors() after sensors_read_all().

void telemetry_tick(uint32_t now) {
    const SensorData *s = &g_ctx.sensors;

    // PID outputs as percentage of SSR window
    float out_top_pct = (float)(g_ctx.pid_top.output / SSR_WINDOW_MS * 100.0);
    float out_bot_pct = (float)(g_ctx.pid_bot.output / SSR_WINDOW_MS * 100.0);

    // Fault/warn bitmask
    uint8_t faults = 0;
    if (s->zone_fault)      faults |= 0x01;
    if (s->edge_fault)      faults |= 0x02;
    if (s->heater_top_warn) faults |= 0x04;
    if (s->heater_bot_warn) faults |= 0x08;

    // htop / hbot: print "F" on fault, value otherwise
    char htop_buf[10], hbot_buf[10];
    if (s->heater_top == TC_FAULT_VALUE)
        snprintf(htop_buf, sizeof(htop_buf), "F");
    else
        snprintf(htop_buf, sizeof(htop_buf), "%.1f", s->heater_top);

    if (s->heater_bot == TC_FAULT_VALUE)
        snprintf(hbot_buf, sizeof(hbot_buf), "F");
    else
        snprintf(hbot_buf, sizeof(hbot_buf), "%.1f", s->heater_bot);

    Serial.printf("DATA,%lu,%s,%.2f,%.2f,%s,%s,%.1f,%.1f,%.1f,%.1f,%u\n",
        (unsigned long)now,
        state_name(g_ctx.state),
        s->zone_temp,
        s->edge_temp,
        htop_buf,
        hbot_buf,
        g_ctx.pid_top.setpoint,
        (double)out_top_pct,
        g_ctx.pid_bot.setpoint,
        (double)out_bot_pct,
        (unsigned)faults
    );
}

// ─── Event Line ───────────────────────────────────────────────────────────────

void telemetry_event(const char *tag, const char *msg) {
    Serial.printf("EVENT,%lu,%s,%s\n", (unsigned long)millis(), tag, msg);
}
