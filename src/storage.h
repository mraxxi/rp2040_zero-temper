#pragma once
#include <stdint.h>
#include <stdbool.h>

// ─── Persisted Settings ───────────────────────────────────────────────────────
//
// Stored in RP2040 flash via arduino-pico EEPROM emulation.
// Layout is versioned — if STORAGE_MAGIC doesn't match on load,
// defaults from config.h are written instead.

#define STORAGE_MAGIC  0xB6A2u   // change when struct layout changes

typedef struct {
    uint16_t magic;

    // PID gains — top heater
    double kp_top;
    double ki_top;
    double kd_top;

    // PID gains — bottom heater
    double kp_bot;
    double ki_bot;
    double kd_bot;

    // Preheat config
    float    preheat_target_temp;
    uint16_t preheat_duration_s;

    // Last selected BGA profile index
    uint8_t  profile_sel;

    uint8_t  _pad[3];   // align to 4 bytes
} StoredSettings;

// ─── API ──────────────────────────────────────────────────────────────────────
void     storage_init(void);          // load from flash; writes defaults if blank/corrupt
void     storage_save(void);          // write current settings to flash
bool     storage_is_valid(void);      // true if loaded data passed magic check

// Access to the live settings struct (populated by storage_init)
extern StoredSettings g_settings;
