#pragma once
#include "common.h"

// ─── Init ─────────────────────────────────────────────────────────────────────
void pid_init(PIDState *pid, double kp, double ki, double kd,
              double out_min, double out_max);

// ─── Compute ──────────────────────────────────────────────────────────────────
// Runs one PID iteration. dt_ms = elapsed ms since last compute.
// Updates pid->output. Call only when you actually want a new output value.
void pid_compute(PIDState *pid, double input, double dt_ms);

// ─── Helpers ──────────────────────────────────────────────────────────────────
void pid_set_tunings(PIDState *pid, double kp, double ki, double kd);
void pid_set_setpoint(PIDState *pid, double setpoint);
void pid_reset(PIDState *pid);   // clears integrator and last_input
void pid_set_active(PIDState *pid, bool active);