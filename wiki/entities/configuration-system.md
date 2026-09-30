---
title: Configuration System — config.h and Feature Flags
type: entity
tags: [config, compile-flags, tuning, pid, thresholds]
created: 2026-04-22
updated: 2026-09-30
related_files: [src/config.h, src/debug_flags.h, platformio.ini]
---

`src/config.h` is the single source of truth for all compile-time parameters: flight thresholds, PID gains, safety limits, hardware presence flags, and pin aliases. `src/debug_flags.h` controls granular serial-debug output. `platformio.ini` defines build-time feature selection.

## Organization of `config.h`

| Category | Examples |
|----------|----------|
| Version | `FIRMWARE_VERSION` (`v0.58.0-beta` on branch beta-0.58) |
| Flight thresholds | `BOOST_ACCEL_THRESHOLD` (2.0 g), `BURNOUT_ACCEL_THRESHOLD` (0.5 g), `APOGEE_*` |
| Apogee detection | `APOGEE_BARO_DESCENT_THRESHOLD`, `APOGEE_CONFIRMATION_COUNT`, `BACKUP_APOGEE_TIME_MS` (20000) |
| Landing detection | `LANDING_ACCEL_MIN_G/MAX_G`, `LANDING_ALTITUDE_STABLE_THRESHOLD` (baro span over the window), `LANDING_WINDOW_SAMPLES`, `LANDING_CONFIRMATION_COUNT/TIME_MS`, `DESCENT_STATE_TIMEOUT_MS` |
| Fresh-sample staleness (beta-0.58) | `BARO_STALE_TIMEOUT_MS`, `ACCEL_STALE_TIMEOUT_MS`, `GPS_STALE_TIMEOUT_MS` — data older than this is treated as unavailable |
| Apogee gates (beta-0.58) | `APOGEE_MIN_TIME_AFTER_BURNOUT_MS`, `APOGEE_MIN_ALTITUDE_GAIN_M`, `APOGEE_BARO_TRANSONIC_LOCKOUT_MS`, `APOGEE_ACCEL_FREEFALL_G/…_WINDOW_MS`, `APOGEE_CLIMB_VETO_MPS`, `APOGEE_HIGH_FORCE_VETO_G`, `APOGEE_GPS_DESCENT_THRESHOLD_M` — see [[concepts/apogee-detection]] |
| Launch / burnout (beta-0.58) | `LAUNCH_CONFIRMATION_COUNT`, `BOOST_TIMEOUT_MS`, `BOOST_BURNOUT_PEAK_FRACTION`, `BOOST_BURNOUT_SETTLE_FRACTION`, `BOOST_ACCEL_EMA_ALPHA`, `LAUNCH_ALT_AVG_SAMPLES/MIN_SAMPLES` |
| Main deploy (beta-0.58) | `MAIN_DEPLOY_CONFIRMATION_COUNT`, `MAIN_DEPLOY_FALLBACK_TIME_MS`, `MAIN_DEPLOY_ASSUMED_DROGUE_RATE_MPS`, `MAIN_DEPLOY_FALLBACK_MARGIN/MIN_MS`, `MAIN_DEPLOY_MAX_DROGUE_TIME_MS` — **tune per airframe** |
| Recovery (beta-0.58) | `RECOVERY_MIN_AGL_M`, `RECOVERY_ALT_MARGIN_M`, `RECOVERY_MAX_RESUMES`, `RECOVERY_MIN_VERTICAL_RATE_MPS`, `RECOVERY_EVIDENCE_*`, `RECOVERY_BACKUP_TIMER_ALLOWANCE_MS`, `EEPROM_PROGRESS_SAVE_INTERVAL_MS`, `GROUND_AGL_TOLERANCE_M`, `RESET_FLIGHT_*` — see [[entities/state-management]] |
| Kalman gating (beta-0.58) | `KALMAN_ACCEL_GATE_LOW_G/HIGH_G`, `KALMAN_ACCEL_GATE_MAX_GYRO_RPS` |
| Guidance timing (beta-0.58) | `GUIDANCE_UPDATE_INTERVAL_MS`, `GUIDANCE_MAX_DT_S` |
| Bench-only commands | `ENABLE_TEST_COMMANDS` (default 0) gates `TEST_FREEZE` |
| Guidance / PID | `PID_ROLL_KP/KI/KD`, `PID_PITCH_*`, `PID_YAW_*`, integral limits, output clamp ±1.0 |
| Stability monitor | one unified threshold set (`GUIDANCE_STABILITY_*`; the legacy `STABILITY_*` names alias it): rate limits 180 °/s roll, 180 °/s pitch, 360 °/s yaw; attitude-error limits 30° roll / 20° pitch / 20° yaw; saturation 95%; violation grace 500 ms |
| Servo smoothing | `SERVO_RATE_LIMIT_DPS` (10 deg/100 ms), `SERVO_DEADBAND_DEG` (0.5°), `SERVO_LOWPASS_CUTOFF_HZ` (2 Hz) — specified in servo degrees, converted ÷90 to the normalized ±1 command domain at init |
| Hardware presence | `DROGUE_PRESENT`, `MAIN_PRESENT`, `USE_KX134`, `BUZZER_OUTPUT`, `ENABLE_GUIDANCE`, `DISABLE_SDCARD_LOGGING` (defined but currently unreferenced) |
| Pin aliases | `NEOPIXEL_PIN` (2), `BUZZER_PIN` (9), `PYRO_CHANNEL_1` (drogue — pin 4, moved off pin 2 to resolve the NeoPixel conflict), `PYRO_CHANNEL_2` (main — pin 3) |
| Watchdog | `WATCHDOG_TIMEOUT_MS` (5000 ms) — now actually programmed into `WDT_T4` by `setup()` |
| Recovery signalling | SOS beep timing, strobe cadence, GPS transmit interval |
| Battery monitor | ADC pin, divider ratio, thresholds |
| EEPROM layout | signature/version/slot constants |

See `src/config.h` directly for up-to-date values — this wiki page documents the *categories*, not individual numbers (which drift).

## Compile-Time Feature Flags (`platformio.ini`)

| Flag | Effect |
|------|--------|
| `-D ENABLE_TEST_COMMANDS=1` | Bench builds only: compiles in the `TEST_FREEZE` serial command (hangs the loop 6 s to prove the watchdog; refused outside `PAD_IDLE`). Never in a flight build ([[queries/flight-logic-audit-2026-09]] #13) |
| `native` env: `-I test/stubs -std=gnu++17` | Host stand-in for the Arduino core/libraries so the real flight sources compile natively ([[concepts/testing-strategy]]); ArduinoFake was dropped from this env |
| `-DNATIVE_TEST_BUILD` | Would select mock HAL / mock sensors in `hal_factory.h` and `sensor_factory.cpp` — **no build environment defines it**; the `native` test env defines `-D UNIT_TEST_NATIVE` instead, and the HAL/sensor factories are dormant in the flight build |
| `-DUSE_BNO085_VARIANT` | Swap primary IMU to BNO085 adapter (dormant OO sensor stack; never defined by any env) |
| `-DUSE_BONO85_BACKUP` | Swap backup sensor to BNO085 (note the misspelling in code; never defined by any env) |
| `-DGPS_USE_SPI` | Use SPI for GPS instead of I2C (`config.h` default `0` = I2C; GPS init also sets the u-blox `DYN_MODEL_AIRBORNE4g` dynamic model) |
| `ENABLE_TELEMETRY` *(in `config.h`, default `0`)* | When `1`: opens `Serial5` at `TELEMETRY_BAUD` (115200) in `setup()` and writes a 43-byte framed `TelemetryPacket` inside `WriteLogData()`. Pure addition — does not change SD CSV or USB serial. Companion defines: `TELEMETRY_BAUD`, `TELEMETRY_PERIOD_MS` (defined but unused — sending piggybacks on the 5 Hz log cadence, and is skipped when SD logging is unavailable). See [[entities/esp32-telemetry]]. |

The HAL/sensor factory flags map to conditional compilation in `src/hal/hal_factory.h` and `src/sensors/sensor_factory.cpp` — code that the flight build currently does not invoke. See [[concepts/hal-abstraction]] and [[concepts/sensor-redundancy]].

## Runtime Debug Flags (`debug_flags.h`)

Turned on/off by numeric commands (digits `0`–`6` toggle output; `7`–`9` are actions: start logging / SD status / shutdown) and named commands like `debug_serial_csv on/off`. They only affect log verbosity — not flight behavior:

| Flag | Purpose |
|------|---------|
| `enableSerialCSV` | Stream the 63-field CSV telemetry line to USB serial (digit `0`) |
| `enableSystemDebug` | System-level status prints (digit `1`) |
| `enableIMUDebug` | Raw IMU sample prints (digit `2`) |
| `enableGPSDebug` | GPS fix/status prints (digit `3`) |
| `enableBaroDebug` | Barometer / altitude prints (digit `4`) |
| `enableStorageDebug` | SD/storage prints (digit `5`) |
| `enableICMRawDebug` | ICM-20948 raw output (digit `6`) |
| `enableSensorDebug` | Detailed sensor prints |
| `enableStatusSummary` | Periodic status summary |
| `enableBatteryDebug` | Battery voltage prints |
| `enableDetailedOutput` | Extra verbosity on supported prints |

There are no `enableStateDebug` / `enableGuidanceDebug` flags. See [[entities/command-processor]] for the command list.

## Tuning Workflow

1. Edit parameter in `src/config.h`.
2. Rebuild: `pio run -e teensy41`.
3. Upload: `pio run -e teensy41 -t upload`.
4. Validate change via `status` and debug streams.
5. Run unit tests if the parameter touches flight logic: `pio test -e native`.

Never ship a tuning change without running the full suite — apogee/landing parameters have test coverage.

## Related

- [[concepts/hal-abstraction]] — HAL selected by compile flag
- [[concepts/sensor-redundancy]] — sensor selection by compile flag
- [[entities/command-processor]] — runtime command catalogue
- [[concepts/testing-strategy]] — how compile-time selection enables native tests
