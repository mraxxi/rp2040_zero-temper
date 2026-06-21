#include "storage.h"
#include "config.h"
#include <EEPROM.h>
#include <Arduino.h>

// RP2040 arduino-pico EEPROM emulation — backed by 4KB of flash.
// We only use sizeof(StoredSettings) bytes starting at address 0.

StoredSettings g_settings = {};

static bool g_valid = false;

// ─── Defaults ─────────────────────────────────────────────────────────────────

static void load_defaults(void) {
    g_settings.magic               = STORAGE_MAGIC;
    g_settings.kp_top              = DEFAULT_KP_TOP;
    g_settings.ki_top              = DEFAULT_KI_TOP;
    g_settings.kd_top              = DEFAULT_KD_TOP;
    g_settings.kp_bot              = DEFAULT_KP_BOT;
    g_settings.ki_bot              = DEFAULT_KI_BOT;
    g_settings.kd_bot              = DEFAULT_KD_BOT;
    g_settings.preheat_target_temp = 150.0f;
    g_settings.preheat_duration_s  = 120;
    g_settings.profile_sel         = 0;
}

// ─── Init ─────────────────────────────────────────────────────────────────────

void storage_init(void) {
    EEPROM.begin(sizeof(StoredSettings));
    EEPROM.get(0, g_settings);

    if (g_settings.magic != STORAGE_MAGIC) {
        Serial.println("[storage] blank/corrupt — writing defaults");
        load_defaults();
        storage_save();
        g_valid = false;
    } else {
        g_valid = true;
        Serial.println("[storage] loaded ok");
    }
}

// ─── Save ─────────────────────────────────────────────────────────────────────

void storage_save(void) {
    g_settings.magic = STORAGE_MAGIC;
    EEPROM.put(0, g_settings);
    EEPROM.commit();
    Serial.println("[storage] saved");
}

// ─── Query ────────────────────────────────────────────────────────────────────

bool storage_is_valid(void) {
    return g_valid;
}
