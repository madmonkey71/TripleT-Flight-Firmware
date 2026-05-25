---
title: command_processor — Serial Command Interface
type: entity
tags: [commands, serial, interface, debugging]
created: 2026-04-15
updated: 2026-04-22
related_files: [src/command_processor.cpp, src/command_processor.h, src/debug_flags.h]
---

Text-based serial command interface for controlling and diagnosing the flight computer. Operates at 115200 baud. Used from a serial terminal or from the [[entities/web-interface|web console]].

## Command Catalogue

### Flight control

| Command | Description | State guard |
|---------|-------------|-------------|
| `arm` | Transition to `ARMED`, enabling launch detection | `PAD_IDLE` only; rejected with error 71 on unhealthy sensors |
| `clear_errors` | Manual recovery from `ERROR` state | `ERROR` only |

### Calibration

| Command | Description |
|---------|-------------|
| `calibrate` | Barometric calibration: set current altitude as 0 m AGL ([[concepts/calibration]]) |
| `calibrate_gyro` | Sample stationary gyro bias |
| `calibrate_mag` | Interactive magnetometer calibration (rotate through orientations) |
| `save_mag_cal` | Persist current mag calibration to EEPROM |

### Diagnostics

| Command | Description |
|---------|-------------|
| `status_sensors` | Per-sensor health summary ([[entities/error-handling]]) |
| `scan_i2c` | List responding I2C addresses |
| `help` | Emit full command list |

### Debug output toggles

Runtime flags; no rebuild required. Both shortcut (digit) and named forms exist:

| Shortcut | Named | Purpose |
|----------|-------|---------|
| `0` | `debug_serial_csv off` | Stop 62-field CSV stream |
| — | `debug_serial_csv on` | Start CSV stream |
| `1` | `debug_system on/off` | System messages |
| `2` | `debug_imu on/off` | IMU raw samples |
| `3` | `debug_gps on/off` | GPS fix/status |
| `4` | `debug_baro on/off` | Barometer / altitude |
| `5` | `debug_state on/off` | State transitions |
| `6` | `debug_guidance on/off` | PID / stability monitor |
| `7` | `debug_battery on/off` | Battery voltage |
| `8`–`9` | additional subsystems | See `src/debug_flags.h` |

See [[entities/configuration-system]] for the full flag catalogue.

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
- [[entities/error-handling]] — meaning of `status_sensors` output
- [[entities/web-interface]] — GUI client for this same command stream
