---
title: Hardware Platform — Teensy 4.1 Flight Computer
type: entity
tags: [hardware, teensy, sensors, wiring, i2c]
created: 2026-04-22
updated: 2026-07-02
related_files: [src/config.h, platformio.ini, src/TripleT_Flight_Firmware.cpp]
---

Reference for the physical flight computer: Teensy 4.1 main controller, attached sensors, I2C bus layout, and pin assignments.

## Main Controller

| Spec | Value |
|------|-------|
| MCU | Teensy 4.1 (NXP i.MX RT1062) |
| Core | ARM Cortex-M7 @ 600 MHz |
| Flash | 8 MB (firmware ~145 KB) |
| RAM | 1024 KB (ITCM/DTCM/OCRAM) |
| EEPROM | 4284 bytes emulated |
| SD Card | Built-in SDIO (FIFO_SDIO mode) |
| Watchdog | WDT_T4 (`WATCHDOG_TIMEOUT_MS` = 5000 ms reset timeout, programmed in `setup()`) |

## Sensor Suite

| Sensor | Role | Bus | Address | Range | Notes |
|--------|------|-----|---------|-------|-------|
| ICM-20948 | Primary IMU (9-DoF) | I2C | 0x69 (AD0 high; 0x68 alt) | Accel ±16g, Gyro ±250°/s, Mag | Source of orientation via Kalman |
| KX134 | High-G backup accel | I2C | 0x1E / 0x1F | ±64g | Kalman accel source auto-switches when ICM magnitude > 16 g |
| MS5611 | Barometric altimeter | I2C | 0x77 (0x76 alt) | 0–50 km | Primary altitude source |
| u-blox GNSS (NEO-M8/ZOE-M8Q) | GPS/time | I2C *or* SPI | 0x42 | — | Feature flag `GPS_USE_SPI`; dynamic model set to `DYN_MODEL_AIRBORNE4g` at init (`src/gps_functions.cpp:137`) |
| BNO085 (optional) | Alternative IMU | I2C | 0x28 / 0x29 | Accel/Gyro/Mag + fused quat | Stub adapter; see `src/sensors/bno085_sensor.h` |

See [[concepts/sensor-evaluation]] for selection rationale and upgrade paths.

## Actuators & Indicators

| Device | Pin | Source |
|--------|-----|--------|
| NeoPixel status LED | `NEOPIXEL_PIN` (2) | Adafruit_NeoPixel |
| Buzzer (recovery beacon, `RECOVERY_BEACON_FREQUENCY_HZ` = 2500 Hz) | `BUZZER_PIN` (9) | Tone generator |
| Servos (fins) | `PWMServo` pins 21 (pitch), 23 (roll), 20 (yaw) | Teensy PWMServo lib |
| Pyro channel 1 (drogue) | `PYRO_CHANNEL_1` (**4**) | GPIO digitalWrite, non-blocking 1000 ms fire |
| Pyro channel 2 (main) | `PYRO_CHANNEL_2` (3) | GPIO digitalWrite, non-blocking 1000 ms fire |

**Wiring note:** `PYRO_CHANNEL_1` was moved from pin 2 to pin 4 (2026-07) because pin 2 collided with `NEOPIXEL_PIN` — NeoPixel data writes drove the same GPIO as the drogue fire line. Existing boards wired to pin 2 **must be rewired** to pin 4 (`src/config.h:30-34`).

## I2C Bus

All sensors share a single I2C bus (except SPI GPS variant).

Integrity checks:
- Verify 3.3 V rail is stable under load.
- Pull-up resistors 2.2–4.7 kΩ on SDA/SCL.
- No bridged traces between address-select pads (they flip sensor address).
- At rest: expect sensors to respond to `scan_i2c` serial command.

## Power

- Input: 3S LiPo typical (≥10 V).
- 3.3 V regulator feeds Teensy + sensors.
- Battery voltage is read on ADC A7 (half divider) every 5 s and logged to the CSV. **Nothing acts on it** — there is no brownout cutoff, low-voltage gate, or graceful-shutdown logic in the firmware; use the log for post-flight analysis.

## SD Card

- Format: FAT32, 4 KB clusters, ≥5 MB free.
- Log file: `DATA_YYYYMMDD_HHMMSS.csv` (with GPS time fix) or `LOG_<millis>.csv` (fallback without fix).
- Mode: `SdFat` with `FIFO_SDIO` (Teensy-specific); see [[concepts/data-logging]].

## Build / Flash

Production build:
```bash
pio run -e teensy41
pio run -e teensy41 -t upload
```

See [[concepts/developer-workflow]] for the full build + upload flow.

## Related

- [[concepts/hal-abstraction]] — how hardware access is decoupled for testing
- [[concepts/sensor-redundancy]] — primary/backup failover
- [[entities/configuration-system]] — pin aliases and hardware feature flags
