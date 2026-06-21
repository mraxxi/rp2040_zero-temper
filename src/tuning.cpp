#include "tuning.h"
#include "common.h"
#include "config.h"
#include "pid_control.h"
#include "sensors.h"
#include "storage.h"
#include "telemetry.h"
#include <Arduino.h>

// ─── Constants ────────────────────────────────────────────────────────────────
#define TUNE_SETTLE_WINDOW_MS   5000UL   // must stay within ±2°C for this long
#define TUNE_SETTLE_BAND        2.0      // °C — settle tolerance
#define TUNE_RELAY_BAND         1.0      // °C — hysteresis around setpoint for relay
#define TUNE_MIN_CYCLES         3        // relay cycles to average before declaring done
#define TUNE_RELAY_AMP_MS       1000.0   // relay output when HIGH (ms of SSR_WINDOW)

// ─── Settle Helpers ───────────────────────────────────────────────────────────

static inline double current_pv(void) {
    return g_tune.tuning_top ? g_ctx.sensors.zone_temp : g_ctx.sensors.edge_temp;
}

// ─── Start ────────────────────────────────────────────────────────────────────

void tuning_start(double target_temp, bool tune_top) {
    g_tune.phase          = TUNE_SETTLE;
    g_tune.tuning_top     = tune_top;
    g_tune.tune_setpoint  = target_temp;
    g_tune.relay_amplitude = TUNE_RELAY_AMP_MS;
    g_tune.relay_high     = false;
    g_tune.peak           = -999.0;
    g_tune.trough         =  999.0;
    g_tune.last_switch_ms = millis();
    g_tune.period_sum     = 0.0;
    g_tune.period_count   = 0;
    g_tune.amplitude_sum  = 0.0;
    g_tune.amplitude_count = 0;
    g_tune.Ku             = 0.0;
    g_tune.Tu             = 0.0;
    g_tune.result_kp      = 0.0;
    g_tune.result_ki      = 0.0;
    g_tune.result_kd      = 0.0;
    g_tune.settle_start_ms = millis();
    g_tune.cycles_needed  = TUNE_MIN_CYCLES;

    // Disable normal PID during tuning
    if (tune_top) {
        pid_set_active(&g_ctx.pid_top, false);
        g_ctx.heater_top_enabled = true;
        ssr_set_output(&g_ctx.ssr_top, 0.0);
    } else {
        pid_set_active(&g_ctx.pid_bot, false);
        g_ctx.heater_bot_enabled = true;
        ssr_set_output(&g_ctx.ssr_bot, 0.0);
    }

    char start_msg[48];
    snprintf(start_msg, sizeof(start_msg), "target=%.1f top=%d", target_temp, (int)tune_top);
    telemetry_event("TUNE_START", start_msg);
}

// ─── Abort ────────────────────────────────────────────────────────────────────

void tuning_abort(void) {
    g_tune.phase = TUNE_ABORT;
    ssr_all_off();
    g_ctx.heater_top_enabled = false;
    g_ctx.heater_bot_enabled = false;
    telemetry_event("TUNE", "aborted");
}

// ─── Query ────────────────────────────────────────────────────────────────────

bool tuning_is_active(void) {
    return (g_tune.phase == TUNE_SETTLE || g_tune.phase == TUNE_RELAY);
}

bool tuning_is_done(void) {
    return (g_tune.phase == TUNE_DONE);
}

// ─── Apply Results ────────────────────────────────────────────────────────────

void tuning_apply_results(void) {
    if (g_tune.phase != TUNE_DONE) return;

    char msg[64];
    if (g_tune.tuning_top) {
        pid_set_tunings(&g_ctx.pid_top,
                        g_tune.result_kp,
                        g_tune.result_ki,
                        g_tune.result_kd);
        g_settings.kp_top = g_tune.result_kp;
        g_settings.ki_top = g_tune.result_ki;
        g_settings.kd_top = g_tune.result_kd;
        snprintf(msg, sizeof(msg), "TOP kp=%.3f ki=%.4f kd=%.3f",
                 g_tune.result_kp, g_tune.result_ki, g_tune.result_kd);
    } else {
        pid_set_tunings(&g_ctx.pid_bot,
                        g_tune.result_kp,
                        g_tune.result_ki,
                        g_tune.result_kd);
        g_settings.kp_bot = g_tune.result_kp;
        g_settings.ki_bot = g_tune.result_ki;
        g_settings.kd_bot = g_tune.result_kd;
        snprintf(msg, sizeof(msg), "BOT kp=%.3f ki=%.4f kd=%.3f",
                 g_tune.result_kp, g_tune.result_ki, g_tune.result_kd);
    }
    storage_save();
    telemetry_event("TUNE_APPLY", msg);
}

// ─── Tick ─────────────────────────────────────────────────────────────────────
//
// Called from main loop when fresh sensor data is available.
// Implements Ziegler-Nichols relay (bang-bang) method:
//   1. SETTLE  — drive heater toward setpoint, wait for temp to stabilize
//   2. RELAY   — switch output above/below setpoint, measure oscillation
//   3. DONE    — compute Ku/Tu, derive PID gains (PID formula)

void tuning_tick(void) {
    double pv  = current_pv();
    double sp  = g_tune.tune_setpoint;
    uint32_t now = millis();

    switch (g_tune.phase) {

    // ── Settle: ramp toward setpoint and wait for stability ──────────────────
    case TUNE_SETTLE: {
        // Drive heater at 50% until within band
        double out = (pv < sp - TUNE_SETTLE_BAND) ? TUNE_RELAY_AMP_MS : 0.0;
        if (g_tune.tuning_top)
            ssr_set_output(&g_ctx.ssr_top, out);
        else
            ssr_set_output(&g_ctx.ssr_bot, out);

        // Check stability: restart timer if out of band
        if (fabs(pv - sp) > TUNE_SETTLE_BAND) {
            g_tune.settle_start_ms = now;
        } else if (now - g_tune.settle_start_ms >= TUNE_SETTLE_WINDOW_MS) {
            // Stable — begin relay oscillation
            g_tune.phase          = TUNE_RELAY;
            g_tune.relay_high     = true;
            g_tune.last_switch_ms = now;
            g_tune.peak           = pv;
            g_tune.trough         = pv;
            telemetry_event("TUNE", "settled -> relay phase");
        }
        break;
    }

    // ── Relay: bang-bang control, measure oscillation peaks ──────────────────
    case TUNE_RELAY: {
        double out;

        // Relay switching with hysteresis
        if (!g_tune.relay_high && pv < sp - TUNE_RELAY_BAND) {
            // Switch HIGH
            double half_period = (double)(now - g_tune.last_switch_ms);
            g_tune.last_switch_ms = now;
            g_tune.relay_high     = true;

            // Record trough at end of LOW half-cycle
            // Full period = 2 × half_period (approximate)
            if (g_tune.period_count > 0) {
                // We accumulate full-cycle time by pairing two half-periods
                g_tune.period_sum += 2.0 * half_period;
                g_tune.period_count++;
                double amp = (g_tune.peak - g_tune.trough) / 2.0;
                g_tune.amplitude_sum += amp;
                g_tune.amplitude_count++;
                char msg[64];
                snprintf(msg, sizeof(msg), "cycle %d half=%.0f ms amp=%.2f",
                         g_tune.period_count, half_period, amp);
                telemetry_event("TUNE", msg);
            }

            g_tune.peak   = pv;
            g_tune.trough = pv;

        } else if (g_tune.relay_high && pv > sp + TUNE_RELAY_BAND) {
            // Switch LOW
            double half_period = (double)(now - g_tune.last_switch_ms);
            g_tune.last_switch_ms = now;
            g_tune.relay_high     = false;
            // Count this as start of a new trackable half-pair
            g_tune.period_count++;
            g_tune.period_sum    += 2.0 * half_period; // rough estimate
        }

        // Track peak/trough within current half-cycle
        if (pv > g_tune.peak)   g_tune.peak   = pv;
        if (pv < g_tune.trough) g_tune.trough = pv;

        // Apply relay output
        out = g_tune.relay_high ? g_tune.relay_amplitude : 0.0;
        if (g_tune.tuning_top)
            ssr_set_output(&g_ctx.ssr_top, out);
        else
            ssr_set_output(&g_ctx.ssr_bot, out);

        // Check if we have enough cycles
        if (g_tune.amplitude_count >= g_tune.cycles_needed) {
            double avg_period   = g_tune.period_sum   / g_tune.period_count;
            double avg_amplitude = g_tune.amplitude_sum / g_tune.amplitude_count;

            // Ku = (4 × relay_amplitude) / (π × avg_amplitude)
            // Tu = avg_period (ms)
            g_tune.Tu = avg_period;
            g_tune.Ku = (4.0 * g_tune.relay_amplitude) / (3.14159265 * avg_amplitude);

            // Ziegler-Nichols PID: Kp=0.6Ku, Ti=0.5Tu, Td=0.125Tu
            // In parallel form: Ki = Kp/Ti, Kd = Kp*Td
            double Tu_s = g_tune.Tu / 1000.0; // convert to seconds
            g_tune.result_kp = 0.6  * g_tune.Ku;
            g_tune.result_ki = g_tune.result_kp / (0.5 * Tu_s);
            g_tune.result_kd = g_tune.result_kp * (0.125 * Tu_s);

            g_tune.phase = TUNE_DONE;
            ssr_all_off();
            g_ctx.heater_top_enabled = false;
            g_ctx.heater_bot_enabled = false;

            char done_msg[80];
            snprintf(done_msg, sizeof(done_msg),
                     "Ku=%.3f Tu=%.0f ms kp=%.3f ki=%.4f kd=%.3f",
                     g_tune.Ku, g_tune.Tu,
                     g_tune.result_kp, g_tune.result_ki, g_tune.result_kd);
            telemetry_event("TUNE_DONE", done_msg);
        }
        break;
    }

    default:
        break;
    }
}
