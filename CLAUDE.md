# BrewForge - Espresso Controller

Raspberry Pi Pico 2W (RP2350) based espresso machine controller with WiFi.
Forked from PicoTest (2026-02-05).

## Current Hardware

**Pico 2W (RP2350 + CYW43 WiFi)**
- Dual ARM Cortex-M33 @ 150MHz
- 520KB SRAM, 4MB Flash
- WiFi 2.4GHz (CYW43439)
- 26 GPIO pins

## Pinout (Active HIGH Relays)

| Pin | Function | Notes |
|-----|----------|-------|
| GP2 | Solenoid relay | RO water gate valve, mirrors pump |
| GP3 | Cup warmer relay | Active HIGH (unused for now) |
| GP4 | Water pump relay | Active HIGH |
| GP5 | Boiler heater relay | Active HIGH |
| GP6 | Flow sensor | Pulse input with ISR |
| GP7 | External brew button | Active LOW, internal pullup, wire to GND |
| GP26 | Thermistor | ADC0 (10K NTC) |
| GP27 | Optional group head temp | ADC1 (reserved) |
| GP28 | Optional pressure sensor | ADC2 (reserved) |
| LED_BUILTIN | Status LED | Via CYW43 chip |

**Reserved (WiFi chip):** GP23, GP24, GP25, GP29

## WiFi Networks

Configured networks (tries in order):
1. `Founders3-Office` / `Gu1fR3serVe13`
2. `DropitlikeitsHotspot` / `Nutmeg21`

Connection is **non-blocking** - relays work immediately while WiFi connects in background.

**Fallback AP Mode:** If WiFi fails after 3 retry cycles, creates hotspot:
- SSID: `BrewForge`
- Password: `coffee123`
- IP: `192.168.4.1`

Serial command `C` retries WiFi from AP mode.

## Web Interface

After WiFi connects, access at:
- `http://<IP>` (shown on serial)
- `http://brewforge.local` (mDNS)

**Features:**
- Real-time temperature display (C or F)
- Temperature rate-of-change display (+X.X deg/s)
- Estimated time to target during preheat
- Clickable relay toggles (with debounce)
- BREW/STOP buttons
- Adjustable target temp (50-100C)
- Adjustable brew time (5-60 seconds)
- 7-column step timing table (CHECK/PRESS/HEAT/PRIME/PAUSE/BREW/COOL)
- Flow rate and volume display
- Flow sensor enable/disable checkbox
- External brew button status indicator
- Last brew time display
- WiFi status and reconnect button
- SVG coffee cup logo
- **Celsius/Fahrenheit toggle**
- **Password-protected Info panel** (password: `Coffee4Me!`)
  - System stats (uptime, free heap, SSID)
  - Reboot device
  - Deep sleep mode

## Brew Cycle State Machine (9 steps)

```
IDLE(0) -> TEMP_CHECK(1) -> PRESSURIZE(2) -> PREHEAT(3) -> PRIME(4) -> PAUSE(5) -> BREW(6) -> COOLDOWN(7) -> DONE(8)
```

| Step | Pump | Boiler | Solenoid | Warmer | Notes |
|------|------|--------|----------|--------|-------|
| IDLE | OFF | OFF | OFF | OFF | Waiting for button |
| TEMP_CHECK | OFF | OFF | OFF | OFF | Read initial temp (1s) |
| PRESSURIZE | ON | OFF | ON | OFF | Quick RO burst (2s), skipped if recent brew |
| PREHEAT | OFF | ON | OFF | OFF | Wait for target temp (15s), adaptive reduction |
| PRIME | ON | ON | ON | OFF | Pop pod + pressurize (4s) |
| PAUSE | OFF | ON | OFF | OFF | Brief pause (1s) |
| BREW | ON | ON | ON | OFF | Main extraction (15s) |
| COOLDOWN | ON | OFF | ON | OFF | Gradient cool (3s) |
| DONE | OFF | OFF | OFF | OFF | Show results, back to idle |

**Solenoid rule:** Mirrors pump - opens when pump runs, closes when pump stops.

**Smart features:**
- **Skip pressurize:** If last brew was < 120 seconds ago, skips PRESSURIZE step
- **Adaptive preheat:** If brewed within 5 min, reduces preheat time by up to 70% (minimum 3s)
- **Predictive heater cutoff:** If heating rate > 2 deg/s and predicted temp in 2s exceeds target+hysteresis, cuts heater early
- **Temperature rate-of-change:** Circular buffer of 10 readings (2.5s window), shows deg/s and ETA

## Serial Commands (115200 baud)

| Key | Action |
|-----|--------|
| H | Help |
| S | Status (temp, flow, relays, rate) |
| B | Start/Stop brew |
| N | Next step |
| V | Previous step |
| P | Toggle pump |
| O | Toggle solenoid |
| W | Toggle cup warmer |
| +/- | Adjust target temp |
| R | Reset flow counter |
| X | EMERGENCY STOP |
| C | Retry WiFi connection |
| T | Toggle test mode (skip temp check) |
| K | Toggle soft pump mode (pulse) |
| F | Toggle flow sensor bypass |
| L | Show last brew time |
| 1-4 | Direct relay toggle |

**Brew triggers:** GP7 button or BOOTSEL button starts/stops/resets brew.

## PlatformIO

```bash
cd BrewForge
pio run -e pico2w           # Build
pio run -e pico2w -t upload # Flash
pio device monitor          # Serial (115200)
```

**platformio.ini:**
```ini
[env:pico2w]
platform = https://github.com/maxgerhardt/platform-raspberrypi.git
board = rpipico2w
framework = arduino
board_build.core = earlephilhower
monitor_speed = 115200
```

## EEPROM Settings

```cpp
uint32_t magic = 0xC0FFEE43         // Changed from 0xC0FFEE42 (PicoTest)
uint8_t stepTimes[9]                // Step durations in seconds
float targetTemp                    // Default 93.0C
bool useFahrenheit                  // Temp unit preference
bool softPumpMode                   // Pulsed pump mode
bool flowSensorEnabled              // Flow sensor bypass (default true)
uint16_t skipPrimeThresholdSec      // Skip pressurize if brewed within this (default 120s)
```

Old PicoTest settings are ignored (magic mismatch) - defaults apply on first boot.

## Key Settings

```cpp
const bool RELAY_ACTIVE_LOW = false;   // Board is ACTIVE HIGH
const float MIN_BREW_TEMP = 85.0;      // Min temp to start brewing
float targetTemp = 93.0;               // Default brew temp
float tempHysteresis = 2.0;            // Bang-bang control band
uint16_t skipPrimeThresholdSec = 120;  // Skip pressurize if recent brew
```

## Known Issues

- **OTA not working** - RP2350 Updater library broken (Error 4). Use USB/BOOTSEL upload.
- GP3 (cup warmer) may need different pin if relay module is faulty
- WiFi can take 8-15 seconds to connect on first boot
- Web page loads slowly on weak WiFi signal

## Files

- `src/main.cpp` - Main controller code (~1878 lines)
- `platformio.ini` - Build configuration
- `CLAUDE.md` - This file
