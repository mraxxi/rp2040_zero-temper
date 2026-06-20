#include "pid_control.h"
#include "config.h"
#include <math.h>

// ─── Init ─────────────────────────────────────────────────────────────────────

void pid_init(PIDState *pid, double kp, double ki, double kd,
              double out_min, double out_max) {
    pid->kp         = kp;
    pid->ki         = ki;
    pid->kd         = kd;
    pid->out_min    = out_min;
    pid->out_max    = out_max;
    pid->setpoint   = 0.0;
    pid->input      = 0.0;
    pid->output     = 0.0;
    pid->integrator = 0.0;
    pid->last_input = 0.0;
    pid->active     = false;
}

void pid_set_tunings(PIDState *pid, double kp, double ki, double kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_setpoint(PIDState *pid, double setpoint) {
    pid->setpoint = setpoint;
}

void pid_reset(PIDState *pid) {
    pid->integrator = 0.0;
    pid->last_input = pid->input; // avoid derivative kick on resume
}

void pid_set_active(PIDState *pid, bool active) {
    if (active && !pid->active) {
        // Waking up — seed last_input to avoid derivative spike
        pid->last_input = pid->input;
    }
    pid->active = active;
}

// ─── Compute ──────────────────────────────────────────────────────────────────
//
// Derivative on measurement (not error) — avoids derivative kick on
// setpoint changes. Integrator clamped to output limits to prevent windup.
// dt_ms must be the actual elapsed time since the last compute call.

void pid_compute(PIDState *pid, double input, double dt_ms) {
    if (!pid->active) return;
    if (dt_ms <= 0.0) return;

    double dt_s = dt_ms / 1000.0;

    pid->input = input;

    double error   = pid->setpoint - input;
    double d_input = input - pid->last_input; // change in measurement

    // Proportional term
    double p_term = pid->kp * error;

    // Integral term with anti-windup clamp
    pid->integrator += pid->ki * error * dt_s;
    if (pid->integrator >  pid->out_max) pid->integrator =  pid->out_max;
    if (pid->integrator <  pid->out_min) pid->integrator =  pid->out_min;

    // Derivative on measurement (negative because d_input opposes correction)
    double d_term = -pid->kd * (d_input / dt_s);

    // Sum and clamp output
    double output = p_term + pid->integrator + d_term;
    if (output >  pid->out_max) output =  pid->out_max;
    if (output <  pid->out_min) output =  pid->out_min;

    pid->output     = output;
    pid->last_input = input;
}
