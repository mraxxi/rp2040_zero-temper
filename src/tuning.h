#pragma once
#include "common.h"

// ─── Autotune Control ─────────────────────────────────────────────────────────
// Start a relay-feedback autotune session.
// target_temp: temperature at which to tune (e.g. 150.0°C)
// tune_top:    true = tune top heater PID, false = tune bottom heater PID
void tuning_start(double target_temp, bool tune_top);

// Feed the autotuner a new temperature measurement and elapsed time.
// Call this from tick_sensors() whenever fresh data is available.
void tuning_tick(void);

// Abort an in-progress autotune session.
void tuning_abort(void);

// Returns true while a tune session is active (TUNE_SETTLE or TUNE_RELAY).
bool tuning_is_active(void);

// Returns true when tuning has completed successfully (TUNE_DONE).
bool tuning_is_done(void);

// Apply the calculated gains to the appropriate PID.
// Call after tuning_is_done() returns true.
void tuning_apply_results(void);
