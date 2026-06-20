#pragma once

// ─── Display (ST7920 SW-SPI) ─────────────────────────────────────────────────
#define PIN_DISP_DATA   0
#define PIN_DISP_CS     1
#define PIN_DISP_CLK    2

// ─── Rotary Encoder ──────────────────────────────────────────────────────────
#define PIN_ENC_B       3
#define PIN_ENC_A       4
#define PIN_ENC_BTN     5

// ─── Beeper ──────────────────────────────────────────────────────────────────
#define PIN_BEEPER      6

// ─── SSR Outputs ─────────────────────────────────────────────────────────────
#define PIN_SSR_BOTTOM  8
#define PIN_SSR_TOP     9

// ─── Sensor SPI (shared bus, individual CS) ──────────────────────────────────
#define PIN_TC_CS_1     10   // MAX6675  — top heater internal temp (logging)
#define PIN_TC_CS_2     11   // MAX6675  — bottom heater internal temp (logging)
#define PIN_TC_CS_3     12   // MAX31855 — edge_temp (bottom PID input)
#define PIN_TC_CS_4     13   // MAX31855 — zone_temp (top PID input)
#define PIN_SPI_CLK     14   // Shared sensor SPI clock
#define PIN_SPI_MISO    28   // Shared sensor SPI MISO

// ─── Timing ──────────────────────────────────────────────────────────────────
#define TEMP_READ_INTERVAL_MS   1000UL   // TC read + display update
#define PID_MIN_INTERVAL_MS     2000UL   // minimum time between PID computes
#define SSR_WINDOW_MS           2000UL   // time-proportional window period

// ─── PID Output Limits ───────────────────────────────────────────────────────
#define PID_OUT_MIN     0.0
#define PID_OUT_MAX     (double)SSR_WINDOW_MS

// ─── Safety Thresholds ───────────────────────────────────────────────────────
#define MAX6675_WARN_TEMP       350.0    // internal heater TC warning threshold (°C)
#define TC_FAULT_VALUE          -999.0   // sentinel for read failure

// ─── Default PID Gains ───────────────────────────────────────────────────────
// These are placeholders — replace after autotuning
#define DEFAULT_KP_TOP      10.0
#define DEFAULT_KI_TOP      0.1
#define DEFAULT_KD_TOP      1.0

#define DEFAULT_KP_BOT      8.0
#define DEFAULT_KI_BOT      0.08
#define DEFAULT_KD_BOT      0.8