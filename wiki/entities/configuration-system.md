---
title: Configuration System — config.h and Feature Flags
type: entity
tags: [config, compile-flags, tuning, pid, thresholds]
created: 2026-04-22
updated: 2026-04-22
related_files: [src/config.h, src/debug_flags.h, platformio.ini]
---

`src/config.h` is the single source of truth for all compile-time parameters: flight thresholds, PID gains, safety limits, hardware presence flags, and pin aliases. `src/debug_flags.h` controls granular serial-debug output. `platformio.ini` defines build-time feature selection.

## Organization of `config.h`

| Category | Examples |
|----------|----------|
| Version | `FIRMWARE_VERSION "v0.10.0"` |
| Flight thresholds | `BOOST_ACCEL_THRESHOLD` (2.0 g), `BURNOUT_ACCEL_THRESHOLD` (0.5 g), `APOGEE_*` |
| Apogee detection | `APOGEE_BARO_DESCENT_THRESHOLD`, `APOGEE_CONFIRMATION_COUNT`, `BACKUP_APOGEE_TIME_MS` (20000) |
| Landing detection | accel stability window, timeout |
| Guidance / PID | `PID_ROLL_KP/KI/KD`, `PID_PITCH_*`, `PID_YAW_*`, integral limits, output clamp ±1.0 |
| Stability monitor | angular-rate limits (180 °/s pitch/yaw, 360 °/s roll), attitude-error limits (20°/30°), violation grace 500 ms |
| Hardware presence | `DROGUE_PRESENT`, `MAIN_PRESENT`, `USE_KX134`, `BUZZER_OUTPUT`, `ENABLE_GUIDANCE`, `DISABLE_SDCARD_LOGGING` |
| Pin aliases | `NEOPIXEL_PIN`, `BUZZER_PIN`, `PYRO_CHANNEL_DROGUE`, `PYRO_CHANNEL_MAIN` |
| Recovery signalling | SOS beep timing, strobe cadence, GPS transmit interval |
| Battery monitor | ADC pin, divider ratio, thresholds |
| EEPROM layout | signature/version/slot constants |

See `src/config.h` directly for up-to-date values — this wiki page documents the *categories*, not individual numbers (which drift).

## Compile-Time Feature Flags (`platformio.ini`)

| Flag | Effect |
|------|--------|
| `-DNATIVE_TEST_BUILD` | Selects mock HAL and mock sensors for desktop Unity tests |
| `-DUSE_BNO085_VARIANT` | Swap primary IMU to BNO085 adapter |
| `-DUSE_BONO85_BACKUP` | Swap backup sensor to BNO085 (instead of KX134) |
| `-DGPS_USE_SPI` | Use SPI for GPS instead of I2C |
| `-DENABLE_TELEMETRY` *(planned)* | Emit telemetry packets on Serial5 to ESP32 bridge |

Flags map to conditional compilation in `src/hal/hal_factory.h` and `src/sensors/sensor_factory.cpp`. See [[concepts/hal-abstraction]] and [[concepts/sensor-redundancy]].

## Runtime Debug Flags (`debug_flags.h`)

Turned on/off by numeric commands `1` – `9` and named commands like `debug_serial_csv on/off`. They only affect log verbosity — not flight behavior:

| Flag | Purpose |
|------|---------|
| `enableSerialCSV` | Stream 62-field CSV telemetry line to USB serial |
| `enableSystemDebug` | System-level status prints |
| `enableIMUDebug` | Raw IMU sample prints |
| `enableGPSDebug` | GPS fix/status prints |
| `enableBaroDebug` | Barometer / altitude prints |
| `enableStateDebug` | State transition prints |
| `enableGuidanceDebug` | PID / stability-monitor prints |
| `enableBatteryDebug` | Battery voltage prints |

See [[entities/command-processor]] for the command list.

## Tuning Workflow

1. Edit parameter in `src/config.h`.
2. Rebuild: `pio run -e teensy41`.
3. Upload: `pio run -e teensy41 -t upload`.
4. Validate change via `status_sensors` and debug streams.
5. Run unit tests if the parameter touches flight logic: `pio test -e native_test`.

Never ship a tuning change without running the full suite — apogee/landing parameters have test coverage.

## Related

- [[concepts/hal-abstraction]] — HAL selected by compile flag
- [[concepts/sensor-redundancy]] — sensor selection by compile flag
- [[entities/command-processor]] — runtime command catalogue
- [[concepts/testing-strategy]] — how compile-time selection enables native tests
