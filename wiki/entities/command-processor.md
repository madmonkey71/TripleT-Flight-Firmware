---
title: command_processor — Serial Command Interface
type: entity
tags: [commands, serial, interface, debugging]
created: 2026-04-15
updated: 2026-09-30
related_files: [src/command_processor.cpp, src/command_processor.h, src/flight_commands.cpp, src/flight_commands.h, src/debug_flags.h]
---

Text-based serial command interface for controlling and diagnosing the flight computer. Operates at 115200 baud. Used from a serial terminal or from the [[entities/web-interface|web console]].

## Command Catalogue

### Flight control

| Command | Description | State guard |
|---------|-------------|-------------|
| `arm` | Transition to `ARMED`, enabling launch detection | `PAD_IDLE` only; rejected with error 71 on unhealthy sensors |
| `disarm` | Return from `ARMED` to `PAD_IDLE` | `ARMED` only. (A 5-minute `ARMED_TIMEOUT_MS` auto-disarm also returns to `PAD_IDLE` if no launch is detected.) |
| `clear_errors` | Manual recovery from `ERROR` state | `ERROR` only, **and** the vehicle must be provably on the ground and never have flown (`flight_is_provably_on_ground()`); otherwise `REFUSED` |
| `clear_to_calibration` | Recover from `ERROR` into `CALIBRATION` | Same as `clear_errors` |
| `skip_calibration` | Skip GPS-based baro calibration, use raw baro altitude (offset 0) | `CALIBRATION`, or `ERROR` with the same ground proof |
| `reset_flight` | **Post-flight reset.** Clears the recorded flight (flight-in-progress flag, pyro-fired flags, resume count, max altitude) and returns to `PAD_IDLE` (or `CALIBRATION` / `ERROR` if not calibrated / unhealthy). Two steps: `reset_flight` prints a 4-digit token valid for `RESET_FLIGHT_TOKEN_TIMEOUT_MS` (30 s); `reset_flight <token>` executes it (single use) | `LANDED`, `RECOVERY`, `ERROR`, `PAD_IDLE` only (refused, no token issued, elsewhere) **and** the vehicle at rest: barometric vertical speed < `RESET_FLIGHT_MAX_VERTICAL_SPEED_MPS` and ~1 g. Does not compare with the launch altitude |

### Calibration

| Command | Description |
|---------|-------------|
| `calibrate` | Barometric calibration ([[concepts/calibration]]); allowed in `PAD_IDLE` / `CALIBRATION` / `ERROR` |
| `calibrate_gyro` | Sample stationary gyro bias |
| `calibrate_mag` | Interactive magnetometer calibration — 30 s figure-eight capture routine (requires ICM ready) |
| `save_mag_cal` | Persist current mag calibration to EEPROM |

### Diagnostics

| Command | Description |
|---------|-------------|
| `status` | System/sensor status summary (there is no `status_sensors` command) |
| `sd_status` | SD card / logging status |
| `sensor_requirements` | Show per-state sensor health requirements ([[entities/error-handling]]) |
| `scan_i2c` | List responding I2C addresses |
| `start_log` | Start SD logging / create a new log file |
| `help` | Emit full command list |

### Digit shortcuts and debug toggles

Runtime flags; no rebuild required. Digits `0`–`6` toggle debug output; digits `7`–`9` are actions:

| Shortcut | Named | Purpose |
|----------|-------|---------|
| `0` | `debug_serial_csv on/off` | Toggle 63-field CSV stream |
| `1` | `debug_system on/off` | System messages |
| `2` | `debug_imu on/off` | IMU raw samples |
| `3` | `debug_gps on/off` | GPS fix/status |
| `4` | `debug_baro on/off` | Barometer / altitude |
| `5` | `debug_storage on/off` | SD/storage debug |
| `6` | `debug_icm_raw on/off` | ICM-20948 raw output |
| `7` | — | Start logging (action, not a toggle) |
| `8` | — | SD card status (action) |
| `9` | — | Shutdown (action) |

Additional named forms: `debug_battery`, `debug_all_off`, `summary`, `set_orientation_filter` / `get_orientation_filter`, and — **only in a build with `ENABLE_TEST_COMMANDS=1`** (default 0; the command does not exist otherwise) — `TEST_FREEZE` (blocks 6 s to exercise the watchdog; refused outside `PAD_IDLE`). Letter shortcuts `a`–`j` cover help/status/storage/display utilities (three flash commands are "Not Implemented" stubs). There are no `debug_state` / `debug_guidance` toggles. See [[entities/configuration-system]] for the full flag catalogue.

## Where the state-changing commands live

`clear_errors`, `clear_to_calibration`, `skip_calibration`, `reset_flight` and `TEST_FREEZE` are handled by `handleFlightStateCommand()` in `src/flight_commands.cpp`, which `processCommand()` calls first for every multi-character command. It is separate from `command_processor.cpp` (which drags in SdFat) so the guards can be unit-tested against the real code — see `test/test_real_commands` and `test/test_real_test_commands_*` ([[queries/flight-logic-audit-2026-09]]).

## Architecture

Each command is dispatched through a `SystemStatusContext` struct that provides access to all system state without global variable coupling:

```cpp
struct SystemStatusContext {
  // All sensors, state, and HAL references
  // Passed to every command handler function
};
```

## Adding Commands

1. Add handler: `void handleMyCommand(const SystemStatusContext& ctx)`
2. Register in dispatcher: `if (strcmp(cmd, "mycommand") == 0) handleMyCommand(ctx);`
3. Add help text to `printHelpText()`

## Debug Flags

`src/debug_flags.h` controls subsystem-specific verbose output. Toggle at runtime via single-digit commands or named variants without recompile.

## Planned Additions (Phase 6.3)

`.archived/PRODUCTION_READINESS_PLAN.md` scopes these new commands; spec only, not yet implemented:

- `preflight` — run `PreflightChecker` (7 checks, PASS/WARN/FAIL each).
- `telemetry_on` / `telemetry_off` — enable/disable ESP32 bridge ([[entities/esp32-telemetry]]).
- `servo_test` / `pyro_test` — actuator ground-test modes.
- `load_trajectory` / `start_trajectory` — waypoint file loader.
- `power_mode <mode>` — force a `PowerManager` mode.
- `battery` — detailed battery/runtime readout.

Track progress in [[queries/roadmap-2026]].

## Related

- [[entities/configuration-system]] — debug-flag definitions and compile-time flags
- [[concepts/calibration]] — what the calibration commands do
- [[entities/error-handling]] — meaning of `status` / `sensor_requirements` output
- [[entities/web-interface]] — GUI client for this same command stream
