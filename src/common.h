#pragma once
#include <stdint.h>
#include <stdbool.h>

// ─── Reflow State Machine ─────────────────────────────────────────────────────
typedef enum {
    STATE_IDLE = 0,
    STATE_PREHEAT,
    STATE_PREHEAT_RUNNING,
    STATE_BGA_REWORK,
    STATE_BGA_RUNNING,
    STATE_COOLING,
    STATE_COMPLETE,
    STATE_ABORT,
    STATE_ERROR
} SystemState;

// ─── Sensor Data ─────────────────────────────────────────────────────────────
typedef struct {
    double zone_temp;       // MAX31855 CS4 — top heater PID input (PCB at BGA site)
    double edge_temp;       // MAX31855 CS3 — bottom heater PID input (PCB edge)
    double heater_top;      // MAX6675  CS1 — top heater internal (logging/warning only)
    double heater_bot;      // MAX6675  CS2 — bottom heater internal (logging/warning only)
    bool   zone_fault;      // true if zone_temp read failed
    bool   edge_fault;      // true if edge_temp read failed
    bool   heater_top_warn; // true if heater_top >= MAX6675_WARN_TEMP
    bool   heater_bot_warn; // true if heater_bot >= MAX6675_WARN_TEMP
} SensorData;

// ─── PID State ───────────────────────────────────────────────────────────────
typedef struct {
    double setpoint;
    double input;
    double output;          // 0..out_max
    double kp, ki, kd;
    double out_min, out_max; // output clamp limits (set by pid_init)
    double integrator;
    double last_input;
    bool   active;
} PIDState;

// ─── SSR State ───────────────────────────────────────────────────────────────
typedef struct {
    uint32_t window_start;  // millis() at start of current window
    double   output;        // latched PID output for this window
} SSRChannel;

// ─── System Context (global singleton) ───────────────────────────────────────
typedef struct {
    SystemState  state;
    SensorData   sensors;
    PIDState     pid_top;
    PIDState     pid_bot;
    SSRChannel   ssr_top;
    SSRChannel   ssr_bot;

    uint32_t last_temp_read_ms;   // millis() of last TC read
    uint32_t last_pid_compute_ms; // millis() of last PID compute

    bool     heater_top_enabled;
    bool     heater_bot_enabled;
} SystemCtx;

// ─── Encoder Input Event ─────────────────────────────────────────────────────
typedef enum {
    ENC_NONE = 0,
    ENC_CW,         // clockwise
    ENC_CCW,        // counter-clockwise
    ENC_PRESS,      // button press
    ENC_LONG_PRESS  // button held
} EncoderEvent;

// ─── Tuning State ────────────────────────────────────────────────────────────
typedef enum {
    TUNE_IDLE = 0,
    TUNE_SETTLE,        // wait for temp to stabilize at tune_setpoint
    TUNE_RELAY,         // relay feedback oscillation in progress
    TUNE_DONE,          // gains calculated, ready to apply
    TUNE_ABORT
} TunePhase;

typedef struct {
    TunePhase phase;
    bool      tuning_top;       // true = tuning top heater, false = bottom

    double    tune_setpoint;    // temperature to tune at (e.g. 150°C)
    double    relay_amplitude;  // relay step size in SSR ms (e.g. 1000ms = 50%)

    // Oscillation tracking
    double    peak;             // highest temp seen in current half-cycle
    double    trough;           // lowest temp seen in current half-cycle
    double    last_switch_ms;   // millis() when relay last switched
    bool      relay_high;       // current relay state

    // Accumulated period samples
    double    period_sum;
    uint8_t   period_count;
    double    amplitude_sum;    // (peak - trough) / 2 per cycle
    uint8_t   amplitude_count;

    // Results
    double    Ku;               // ultimate gain
    double    Tu;               // ultimate period (ms)
    double    result_kp;
    double    result_ki;
    double    result_kd;

    uint32_t  settle_start_ms;
    uint8_t   cycles_needed;    // how many full oscillation cycles to average
} TuneCtx;

// ─── Menu State ──────────────────────────────────────────────────────────────
typedef enum {
    MENU_SPLASH = 0,
    MENU_MAIN,
    MENU_PREHEAT,
    MENU_PREHEAT_CONFIG,
    MENU_PREHEAT_RUNNING,
    MENU_BGA,
    MENU_BGA_CONFIG,
    MENU_BGA_PROFILE_SELECT,
    MENU_BGA_RUNNING,
    MENU_CONFIGS,
    MENU_TUNE_SELECT,
    MENU_TUNE_RUNNING,
    MENU_TUNE_RESULT,
    MENU_ERROR
} MenuState;

// Global context — defined in main.cpp, extern everywhere else
extern SystemCtx g_ctx;
extern TuneCtx   g_tune;