#pragma once
#include <stdint.h>

// ─── Serial Telemetry ─────────────────────────────────────────────────────────
//
// Emits a structured CSV line every TEMP_READ_INTERVAL_MS (1 s) to Serial.
// All other ad-hoc Serial.printf() calls in main/tuning should route through
// the event helpers below so the Python plotter can distinguish them.
//
// CSV format (one line per second):
//
//   DATA,<t_ms>,<state>,<zone>,<edge>,<htop>,<hbot>,<sp_top>,<out_top_%>,<sp_bot>,<out_bot_%>,<faults>
//
// Fields:
//   t_ms       — millis() timestamp
//   state      — system state string (IDLE, PREHEAT, BGA, COOLING, etc.)
//   zone       — zone_temp °C  (top heater PID input)
//   edge       — edge_temp °C  (bottom heater PID input)
//   htop       — heater_top °C (internal, logging only; "F" if fault)
//   hbot       — heater_bot °C (internal, logging only; "F" if fault)
//   sp_top     — top PID setpoint
//   out_top_%  — top PID output as 0-100 %
//   sp_bot     — bottom PID setpoint
//   out_bot_%  — bottom PID output as 0-100 %
//   faults     — bitmask: bit0=zone_fault, bit1=edge_fault, bit2=htop_warn, bit3=hbot_warn
//
// Event lines (not time-gated):
//
//   EVENT,<t_ms>,<tag>,<message>
//
// Examples:
//   EVENT,12345,WARN,top heater over threshold
//   EVENT,12345,ABORT,TC fault during run
//   EVENT,12345,TUNE,settled -> relay phase
//   EVENT,12345,STORAGE,saved

void telemetry_tick(uint32_t now);   // call once per sensor tick (replaces old debug prints)
void telemetry_event(const char *tag, const char *msg);  // emit an EVENT line
