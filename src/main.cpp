#include <Arduino.h>
#include "common.h"
#include "config.h"
#include "sensors.h"
#include "pid_control.h"
#include "enc_display.h"
#include "tuning.h"
#include "storage.h"
#include "telemetry.h"

// ─── Global Context ───────────────────────────────────────────────────────────
SystemCtx g_ctx  = {};
TuneCtx   g_tune = {};

// ─── Forward Declarations ─────────────────────────────────────────────────────
static void system_init(void);
static void tick_sensors(uint32_t now);
static void tick_pid(uint32_t now);
static void tick_ssr(void);
static void tick_display(void);
static void tick_encoder(void);
static void handle_state(void);
static void enter_abort(const char *reason);

// ─── Setup ────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    system_init();
}

// ─── Main Loop ────────────────────────────────────────────────────────────────
//
// Timing model:
//   tick_sensors()  — runs every TEMP_READ_INTERVAL_MS (1000ms)
//                     reads all 4 TCs into g_ctx.sensors
//
//   tick_pid()      — runs when BOTH conditions are true:
//                     1. at least PID_MIN_INTERVAL_MS (2000ms) since last compute
//                     2. a fresh sensor read has occurred since last compute
//                     This naturally gives ~2000ms compute rate without
//                     even/odd counting — if a sensor read is delayed or
//                     skipped, PID simply waits for the next valid one.
//
//   tick_ssr()      — runs every loop iteration (non-blocking)
//                     applies the latched PID output to the SSR pins
//
//   tick_display()  — runs every loop iteration (non-blocking, partial updates)
//
//   tick_encoder()  — runs every loop iteration

void loop() {
    uint32_t now = millis();

    tick_encoder();
    tick_sensors(now);
    tick_pid(now);
    tick_ssr();
    tick_display();
    handle_state();
}

// ─── System Init ──────────────────────────────────────────────────────────────

static void system_init(void) {
    sensors_init();
    ssr_init();
    display_init();

    // Load persisted settings (PID gains, preheat config, profile selection)
    storage_init();

    // Init top heater PID with stored gains
    pid_init(&g_ctx.pid_top,
             g_settings.kp_top, g_settings.ki_top, g_settings.kd_top,
             PID_OUT_MIN, PID_OUT_MAX);

    // Init bottom heater PID with stored gains
    pid_init(&g_ctx.pid_bot,
             g_settings.kp_bot, g_settings.ki_bot, g_settings.kd_bot,
             PID_OUT_MIN, PID_OUT_MAX);

    g_ctx.state                = STATE_IDLE;
    g_ctx.last_temp_read_ms    = 0;
    g_ctx.last_pid_compute_ms  = 0;
    g_ctx.heater_top_enabled   = false;
    g_ctx.heater_bot_enabled   = false;

    telemetry_event("BOOT", "temper2040 init ok");
}

// ─── Sensor Tick ─────────────────────────────────────────────────────────────
//
// Timestamped flag pattern: last_temp_read_ms is updated here.
// tick_pid() uses it to detect that a fresh sample is available.

static bool fresh_sensor_data = false; // set by tick_sensors, cleared by tick_pid

static void tick_sensors(uint32_t now) {
    if (now - g_ctx.last_temp_read_ms < TEMP_READ_INTERVAL_MS) return;

    bool ok = sensors_read_all();
    g_ctx.last_temp_read_ms = now;
    fresh_sensor_data = true;

    // If autotuner is active, feed it fresh data immediately
    if (tuning_is_active()) {
        tuning_tick();
    }

    // Safety: if mission-critical TCs fail during an active run, abort
    if (!ok && g_ctx.state == STATE_BGA_RUNNING) {
        enter_abort("TC fault during run");
    }

    // Heater overheat warnings
    if (g_ctx.sensors.heater_top_warn) {
        telemetry_event("WARN", "top heater internal TC over threshold");
    }
    if (g_ctx.sensors.heater_bot_warn) {
        telemetry_event("WARN", "bottom heater internal TC over threshold");
    }

    // Structured 1-second telemetry line
    telemetry_tick(now);
}

// ─── PID Tick ─────────────────────────────────────────────────────────────────
//
// Runs when:
//   - fresh sensor data is available (we have a new measurement)
//   - AND at least PID_MIN_INTERVAL_MS has passed since last compute
//
// dt_ms is the actual elapsed time — passed into pid_compute() so the
// integrator accumulates correctly even if timing drifts.

static void tick_pid(uint32_t now) {
    if (!fresh_sensor_data) return;
    if (now - g_ctx.last_pid_compute_ms < PID_MIN_INTERVAL_MS) return;

    double dt_ms = (double)(now - g_ctx.last_pid_compute_ms);

    // Only compute and update SSR if heaters are enabled
    if (g_ctx.heater_top_enabled && g_ctx.pid_top.active) {
        pid_compute(&g_ctx.pid_top, g_ctx.sensors.zone_temp, dt_ms);
        ssr_set_output(&g_ctx.ssr_top, g_ctx.pid_top.output);
    }

    if (g_ctx.heater_bot_enabled && g_ctx.pid_bot.active) {
        pid_compute(&g_ctx.pid_bot, g_ctx.sensors.edge_temp, dt_ms);
        ssr_set_output(&g_ctx.ssr_bot, g_ctx.pid_bot.output);
    }

    g_ctx.last_pid_compute_ms = now;
    fresh_sensor_data = false; // consumed
}

// ─── SSR Tick ────────────────────────────────────────────────────────────────
//
// Non-blocking — called every loop iteration.
// Applies the currently latched output value to the SSR pins.
// Output only changes when tick_pid() calls ssr_set_output().

static void tick_ssr(void) {
    if (g_ctx.heater_top_enabled) {
        ssr_tick(&g_ctx.ssr_top, PIN_SSR_TOP);
    } else {
        digitalWrite(PIN_SSR_TOP, LOW);
    }

    if (g_ctx.heater_bot_enabled) {
        ssr_tick(&g_ctx.ssr_bot, PIN_SSR_BOTTOM);
    } else {
        digitalWrite(PIN_SSR_BOTTOM, LOW);
    }
}

// ─── Display Tick ────────────────────────────────────────────────────────────
static void tick_display(void) {
    display_tick();
}

// ─── Encoder Tick ─────────────────────────────────────────────────────────────
// Encoder polling is handled inside display_tick() — no separate tick needed.
static void tick_encoder(void) {}

// ─── State Machine ───────────────────────────────────────────────────────────

static void handle_state(void) {
    switch (g_ctx.state) {

    case STATE_IDLE:
        // Heaters off, waiting for user input
        g_ctx.heater_top_enabled = false;
        g_ctx.heater_bot_enabled = false;
        break;

    case STATE_PREHEAT:
        // Bottom heater only, holding at preheat setpoint
        // pid_bot setpoint set by UI before entering this state
        g_ctx.heater_top_enabled = false;
        g_ctx.heater_bot_enabled = true;
        pid_set_active(&g_ctx.pid_bot, true);
        // Seed input with the current reading so pid_reset's last_input
        // is accurate and the first compute won't produce a derivative spike.
        g_ctx.pid_bot.input = g_ctx.sensors.edge_temp;
        pid_reset(&g_ctx.pid_bot);
        g_ctx.state = STATE_PREHEAT_RUNNING;
        break;

    case STATE_PREHEAT_RUNNING:
        // Running — UI can transition to STATE_BGA_REWORK or STATE_IDLE
        break;

    case STATE_BGA_REWORK:
        // Both heaters active — profile execution handled by separate
        // profile runner (future: profile.cpp)
        g_ctx.heater_top_enabled = true;
        g_ctx.heater_bot_enabled = true;
        pid_set_active(&g_ctx.pid_top, true);
        pid_set_active(&g_ctx.pid_bot, true);
        // Seed inputs with current readings so first compute is derivative-spike-free.
        g_ctx.pid_top.input = g_ctx.sensors.zone_temp;
        g_ctx.pid_bot.input = g_ctx.sensors.edge_temp;
        pid_reset(&g_ctx.pid_top);
        pid_reset(&g_ctx.pid_bot);
        g_ctx.state = STATE_BGA_RUNNING;
        break;

    case STATE_BGA_RUNNING:
        // Active rework — profile runner updates setpoints each step
        break;

    case STATE_COOLING:
        // Both heaters off, monitoring temp descent
        g_ctx.heater_top_enabled = false;
        g_ctx.heater_bot_enabled = false;
        ssr_all_off();
        // Transition to COMPLETE when zone_temp drops below safe threshold
        if (g_ctx.sensors.zone_temp < 50.0) {
            g_ctx.state = STATE_COMPLETE;
        }
        break;

    case STATE_COMPLETE:
        ssr_all_off();
        // UI notifies user, waits for acknowledgement → STATE_IDLE
        break;

    case STATE_ABORT:
    case STATE_ERROR:
        ssr_all_off();
        g_ctx.heater_top_enabled = false;
        g_ctx.heater_bot_enabled = false;
        pid_set_active(&g_ctx.pid_top, false);
        pid_set_active(&g_ctx.pid_bot, false);
        break;
    }
}

// ─── Abort Helper ────────────────────────────────────────────────────────────

static void enter_abort(const char *reason) {
    ssr_all_off();
    g_ctx.heater_top_enabled = false;
    g_ctx.heater_bot_enabled = false;
    pid_set_active(&g_ctx.pid_top, false);
    pid_set_active(&g_ctx.pid_bot, false);
    g_ctx.state = STATE_ABORT;
    telemetry_event("ABORT", reason);
}