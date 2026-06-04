# rp2040_zero-temper

A reflow oven temperature monitor and controller firmware for the **Waveshare RP2040-Zero**, built with [PlatformIO](https://platformio.org/) and the Arduino framework. It reads dual thermocouple sensors (MAX31855) and renders a live temperature graph with phase indicators on a **128×64 ST7920 OLED/LCD** display.

***

## Features

- **Dual thermocouple input** — reads TOP and BOT zone temperatures simultaneously via two MAX31855 modules over SPI
- **PCB probe reading** — dedicated `readPcb()` abstraction, assignable to either thermocouple
- **Live graph** — scrolling 128-pixel wide temperature history plot with dotted grid lines at 50 °C intervals (up to 280 °C)
- **Reflow phase indicator** — displays current phase: `IDLE`, `PREHEAT`, `SOAK`, `REFLOW`, `COOLDOWN`
- **Elapsed timer** — `MM:SS` elapsed time shown on screen
- **Setpoint & ramp rate** — bottom status bar shows current setpoint and heating/cooling rate (°C/s)
- **Error display** — shows error code and string when sensor fault is detected

***

## Hardware

| Component | Details |
|-----------|---------|
| MCU Board | [Waveshare RP2040-Zero](https://www.waveshare.com/wiki/RP2040-Zero) |
| Thermocouple modules | 2× MAX31855 (K-type thermocouple amplifier) |
| Display | 128×64 ST7920 LCD (SPI, software SPI via U8g2) |

### Pin Mapping

| Signal | RP2040 GPIO |
|--------|------------|
| SPI MISO | GPIO 29 |
| SPI CLK  | GPIO 28 |
| TC1 CS   | GPIO 12 |
| TC2 CS   | GPIO 13 |
| Display CLK | GPIO 3 |
| Display DATA | GPIO 2 |
| Display CS | GPIO 1 |
| Display RST | GPIO 8 |

> Pin assignments are defined in `src/sensors.cpp` and `src/display.cpp`. Adjust to match your wiring.

***

## Project Structure

```
rp2040_zero-temper/
├── src/
│   ├── main.cpp        # Arduino entry point (setup / loop)
│   ├── sensors.cpp     # MAX31855 initialization & temperature reads
│   ├── sensors.h
│   ├── display.cpp     # U8g2 rendering: graph, phase, timer, status bar
│   ├── display.h
│   └── tuning.cpp      # PID / profile tuning (WIP)
├── include/            # Shared headers
├── lib/                # Local libraries
├── test/               # Unit tests (PlatformIO native)
├── platformio.ini      # Build configuration
└── .gitignore
```

***

## Getting Started

### Prerequisites

- [PlatformIO IDE](https://platformio.org/install) (VS Code extension or CLI)
- K-type thermocouples connected to MAX31855 modules
- ST7920 128×64 display

### Build & Flash

```bash
# Clone the repository
git clone https://github.com/mraxxi/rp2040_zero-temper.git
cd rp2040_zero-temper

# Build the project
pio run

# Upload to Waveshare RP2040-Zero (hold BOOT button, plug USB, then run)
pio run --target upload
```

### Dependencies

Managed automatically by PlatformIO:

| Library | Version | Purpose |
|---------|---------|---------|
| [robtillaart/MAX31855](https://github.com/RobTillaart/MAX31855) | `^0.6.2` | MAX31855 thermocouple driver |
| [robtillaart/MAX6675](https://github.com/RobTillaart/MAX6675) | `^0.3.4` | MAX6675 thermocouple driver (alternate) |
| [olikraus/U8g2](https://github.com/olikraus/u8g2) | `^2.36.18` | Display graphics library |

***

## Display Layout

```
┌──────────────────────────────┐
│ PREHEAT   00:42               │  ← Phase + Timer
│ TOP:145 BOT:138      P:141   │  ← Temperatures
├──────────────────────────────┤
│  ·····················       │
│         /‾‾‾‾‾‾\             │  ← Scrolling temp graph
│   ___/‾‾        \            │     with grid at 50/100/150/200/250°C
├──────────────────────────────┤
│ Sp:150 +2.3C/s               │  ← Setpoint & ramp rate (or error)
└──────────────────────────────┘
```

***

## API Reference

### `sensors.h`

```cpp
void  sensors_init();   // Initialize both MAX31855 sensors
float readTop();        // Read TOP thermocouple (°C), returns -999 on error
float readBot();        // Read BOT thermocouple (°C), returns -999 on error
float readPcb();        // Read PCB probe — mapped to TC1 by default
```

### `display.h`

```cpp
void display_init();
void display_draw(
    float top, float bot, float pcb,
    uint32_t elapsed,   // seconds
    uint8_t  phase,     // 0=IDLE 1=PREHEAT 2=SOAK 3=REFLOW 4=COOLDOWN
    float    setpoint,
    float    rate,      // °C/s, negative = cooling
    uint8_t  errCode,
    const char* errStr
);
```

***

## Roadmap

- [ ] Complete `main.cpp` reflow profile state machine
- [ ] Implement `tuning.cpp` PID controller
- [ ] Add profile configuration (soak temp/time, peak temp)
- [ ] Serial logging / plotter support
- [ ] MAX6675 fallback support

***

## License

No license specified — contact the author before using in commercial projects.

***

## Author

**mraxxi** — [github.com/mraxxi](https://github.com/mraxxi)
