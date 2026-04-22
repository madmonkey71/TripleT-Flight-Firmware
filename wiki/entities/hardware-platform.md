---
title: Hardware Platform — Teensy 4.1 Flight Computer
type: entity
tags: [hardware, teensy, sensors, wiring, i2c]
created: 2026-04-22
updated: 2026-04-22
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
| Watchdog | WDT_T4 (1000 ms timeout) |

## Sensor Suite

| Sensor | Role | Bus | Address | Range | Notes |
|--------|------|-----|---------|-------|-------|
| ICM-20948 | Primary IMU (9-DoF) | I2C | 0x68 / 0x69 | Accel ±16g, Gyro ±2000°/s, Mag | Source of orientation via Kalman |
| KX134 | High-G backup accel | I2C | 0x1E / 0x1F | ±64g | Auto-switch > ~2g threshold |
| MS5611 | Barometric altimeter | I2C | 0x76 / 0x77 | 0–50 km | Primary altitude source |
| u-blox GNSS (NEO-M8/ZOE-M8Q) | GPS/time | I2C *or* SPI | 0x42 | — | Feature flag `GPS_USE_SPI` |
| BNO085 (optional) | Alternative IMU | I2C | 0x28 / 0x29 | Accel/Gyro/Mag + fused quat | Stub adapter; see `src/sensors/bno085_sensor.h` |

See [[concepts/sensor-evaluation]] for selection rationale and upgrade paths.

## Actuators & Indicators

| Device | Pin Concept | Source |
|--------|-------------|--------|
| NeoPixel status LED | `NEOPIXEL_PIN` | Adafruit_NeoPixel |
| Buzzer (recovery beacon, 4 kHz) | `BUZZER_PIN` | Tone generator |
| Servos (fins) | `PWMServo` channels | Teensy PWMServo lib |
| Pyro channels (drogue / main) | `PYRO_CHANNEL_*` | GPIO digitalWrite, non-blocking fire |

## I2C Bus

All sensors share a single I2C bus (except SPI GPS variant).

Integrity checks:
- Verify 3.3 V rail is stable under load.
- Pull-up resistors 2.2–4.7 kΩ on SDA/SCL.
- No bridged traces between address-select pads (they flip sensor address).
- At rest: expect sensors to respond to `scan_i2c` serial command.

## Power

- Input: 3S LiPo typical (≥10 V). Brownout around ~10.5 V.
- 3.3 V regulator feeds Teensy + sensors.
- Pre-flight gate: battery ≥ configured threshold (see [[entities/configuration-system]]).

## SD Card

- Format: FAT32, 4 KB clusters, ≥5 MB free.
- Log file: `FLIGHT_XXX.csv` (auto-incremented).
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
