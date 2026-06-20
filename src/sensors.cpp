#include <Arduino.h>
#include "sensors.h"
#include "config.h"
#include "common.h"

// ─── Init ─────────────────────────────────────────────────────────────────────

void sensors_init(void) {
    pinMode(PIN_TC_CS_1, OUTPUT); digitalWrite(PIN_TC_CS_1, HIGH);
    pinMode(PIN_TC_CS_2, OUTPUT); digitalWrite(PIN_TC_CS_2, HIGH);
    pinMode(PIN_TC_CS_3, OUTPUT); digitalWrite(PIN_TC_CS_3, HIGH);
    pinMode(PIN_TC_CS_4, OUTPUT); digitalWrite(PIN_TC_CS_4, HIGH);
    pinMode(PIN_SPI_CLK,  OUTPUT); digitalWrite(PIN_SPI_CLK, LOW);
    pinMode(PIN_SPI_MISO, INPUT);
}

void ssr_init(void) {
    pinMode(PIN_SSR_TOP,    OUTPUT); digitalWrite(PIN_SSR_TOP,    LOW);
    pinMode(PIN_SSR_BOTTOM, OUTPUT); digitalWrite(PIN_SSR_BOTTOM, LOW);

    uint32_t now = millis();
    g_ctx.ssr_top.window_start = now;
    g_ctx.ssr_top.output       = 0.0;
    g_ctx.ssr_bot.window_start = now;
    g_ctx.ssr_bot.output       = 0.0;
}

// ─── Raw SPI Read Helpers ─────────────────────────────────────────────────────
//
// Bit-bang SPI read — shared CLK/MISO, individual CS per chip.
// MAX31855: 32-bit transfer, upper 16 bits contain temp + fault.
// MAX6675:  16-bit transfer.

static uint32_t spi_read_raw(uint8_t cs_pin, uint8_t bits) {
    uint32_t result = 0;
    digitalWrite(cs_pin, LOW);
    delayMicroseconds(1);
    for (int i = bits - 1; i >= 0; i--) {
        digitalWrite(PIN_SPI_CLK, HIGH);
        delayMicroseconds(1);
        if (digitalRead(PIN_SPI_MISO)) result |= (1UL << i);
        digitalWrite(PIN_SPI_CLK, LOW);
        delayMicroseconds(1);
    }
    digitalWrite(cs_pin, HIGH);
    return result;
}

double read_max31855(uint8_t cs_pin) {
    uint32_t raw = spi_read_raw(cs_pin, 32);

    // Bit 16 = fault bit
    if (raw & 0x00010000UL) return TC_FAULT_VALUE;

    // Bits 31:18 = 14-bit signed thermocouple temp, 0.25°C LSB
    int16_t temp_raw = (int16_t)(raw >> 18);
    // Sign-extend 14-bit value to 16-bit
    if (temp_raw & 0x2000) temp_raw |= (int16_t)0xC000;

    return (double)temp_raw * 0.25;
}

double read_max6675(uint8_t cs_pin) {
    uint16_t raw = (uint16_t)spi_read_raw(cs_pin, 16);

    // Bit 2 = open thermocouple fault
    if (raw & 0x0004) return TC_FAULT_VALUE;

    // Bits 14:3 = 12-bit temp, 0.25°C LSB
    return (double)((raw >> 3) & 0x0FFF) * 0.25;
}

// ─── Read All Sensors ────────────────────────────────────────────────────────

bool sensors_read_all(void) {
    SensorData *s = &g_ctx.sensors;

    // Mission-critical: MAX31855 external TCs
    // CS4 = zone_temp (top heater PID — healthy unit)
    s->zone_temp  = read_max31855(PIN_TC_CS_4);
    // CS3 = edge_temp (bottom heater PID — previously diagnosed as possibly damaged;
    //        replace if readings are erratic)
    s->edge_temp  = read_max31855(PIN_TC_CS_3);

    s->zone_fault = (s->zone_temp == TC_FAULT_VALUE);
    s->edge_fault = (s->edge_temp == TC_FAULT_VALUE);

    // Non-critical: MAX6675 internal heater temps (logging + warning)
    s->heater_top = read_max6675(PIN_TC_CS_1);
    s->heater_bot = read_max6675(PIN_TC_CS_2);

    s->heater_top_warn = (s->heater_top != TC_FAULT_VALUE &&
                          s->heater_top >= MAX6675_WARN_TEMP);
    s->heater_bot_warn = (s->heater_bot != TC_FAULT_VALUE &&
                          s->heater_bot >= MAX6675_WARN_TEMP);

    // Return true only if both PID-critical sensors are healthy
    return (!s->zone_fault && !s->edge_fault);
}

// ─── SSR Time-Proportional Control ───────────────────────────────────────────

void ssr_set_output(SSRChannel *ch, double output_ms) {
    if (output_ms < 0.0)           output_ms = 0.0;
    if (output_ms > SSR_WINDOW_MS) output_ms = (double)SSR_WINDOW_MS;
    // Only restart the window when transitioning from fully-off,
    // so active PID updates don't clip the current ON-time mid-window.
    if (ch->output == 0.0) {
        ch->window_start = millis();
    }
    ch->output = output_ms;
}

void ssr_tick(SSRChannel *ch, uint8_t pin) {
    uint32_t now     = millis();
    uint32_t elapsed = now - ch->window_start;

    // Roll window if expired
    if (elapsed >= SSR_WINDOW_MS) {
        ch->window_start += SSR_WINDOW_MS;
        elapsed = now - ch->window_start;
    }

    // SSR ON for the first output_ms of the window, OFF for the rest
    if (ch->output > 0.0 && elapsed < (uint32_t)ch->output) {
        digitalWrite(pin, HIGH);
    } else {
        digitalWrite(pin, LOW);
    }
}

void ssr_all_off(void) {
    digitalWrite(PIN_SSR_TOP,    LOW);
    digitalWrite(PIN_SSR_BOTTOM, LOW);
    // Reset outputs so next ssr_tick doesn't re-fire
    g_ctx.ssr_top.output = 0.0;
    g_ctx.ssr_bot.output = 0.0;
}
