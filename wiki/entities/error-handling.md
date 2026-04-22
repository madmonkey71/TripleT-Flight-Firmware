---
title: Error Codes & Error Handling
type: entity
tags: [errors, recovery, safety, degradation]
created: 2026-04-22
updated: 2026-04-22
related_files: [src/error_codes.h, src/flight_logic.cpp, src/command_processor.cpp]
---

Error handling uses a single `ErrorCode_t` enum (`src/error_codes.h`) and a two-track response model: **hard errors** trigger the `ERROR` state (red LED, abort); **soft errors** log and continue with graceful degradation.

## Error Code Table (authoritative: `src/error_codes.h`)

| Range | Category | Examples |
|-------|----------|----------|
| 0 | No error | `NO_ERROR` |
| 10–29 | Sensor init failures | `SENSOR_INIT_FAIL_MS5611` (10), `SENSOR_INIT_FAIL_ICM20948` (11), `SENSOR_INIT_FAIL_KX134` (12), `SENSOR_INIT_FAIL_GPS` (13) |
| 30–49 | Sensor read failures | `SENSOR_READ_FAIL_MS5611` (30), `SENSOR_READ_FAIL_ICM20948` (31), `SENSOR_READ_FAIL_KX134` (32), `SENSOR_READ_FAIL_GPS` (33) |
| 50–59 | SD / logging | `SD_CARD_INIT_FAIL` (50), `SD_CARD_MOUNT_FAIL` (51), `LOG_FILE_CREATE_FAIL` (52), `SD_CARD_WRITE_FAIL` (53), `SD_CARD_LOW_SPACE` (54, warn only) |
| 60–69 | Calibration | `BARO_CALIBRATION_FAIL_NO_GPS` (60), `BARO_CALIBRATION_FAIL_TIMEOUT` (61), `MAG_CALIBRATION_LOAD_FAIL` (62) |
| 70–79 | State machine | `STATE_TRANSITION_INVALID_HEALTH` (70), `ARM_FAIL_HEALTH_CHECK` (71) |
| 80–89 | EEPROM | `EEPROM_SIGNATURE_INVALID` (80) |
| 90–99 | Guidance (soft) | `GUIDANCE_STABILITY_FAIL` (90), `GUIDANCE_TARGET_NOT_SET` (91) |
| 250–254 | Config | `CONFIG_ERROR_MAIN_PARACHUTE` (250, e.g. `MAIN_PRESENT` false at runtime) |
| 255 | Unknown | `UNKNOWN_ERROR` |

## Hard vs Soft Errors

**Hard errors** (10–89, 250–254) → transition to `ERROR` state, red LED, abort flight operations. Auto-recovery attempted every 10 s; returns to `PAD_IDLE` if sensors recover. Manual override via `clear_errors`.

**Soft errors** (90–99) → log `g_last_error_code`, continue flight. Example: `GUIDANCE_STABILITY_FAIL` sets `g_guidance_active = false`, centers servos, switches LED to orange, but apogee detection and parachute deployment proceed normally. See [[concepts/guidance-degradation]].

## Recovery Paths

```mermaid
flowchart TD
  E[Error detected] --> C{Hard or soft?}
  C -->|hard| H[enter ERROR state<br/>red LED, abort]
  C -->|soft| S[log code<br/>degrade feature<br/>continue flight]
  H --> R{auto-recovery<br/>every 10s}
  R -->|sensors OK| P[PAD_IDLE]
  R -->|still failing| H
  H --> M[manual clear_errors]
  M --> P
```

## Diagnostic Commands

| Command | Purpose |
|---------|---------|
| `status_sensors` | Per-sensor health summary |
| `scan_i2c` | List responding I2C devices |
| `clear_errors` | Manually reset `ERROR` state (only valid while in `ERROR`) |

See [[entities/command-processor]] for the full command catalogue.

## Sensor Health Algorithm

Continuous monitoring (pseudocode — see `src/sensor_validator.h` and `isSensorSuiteHealthy()` in `src/flight_logic.cpp`):

1. **Response check** — sensor returns data on polling interval (10 ms IMU, 20 ms baro, 100 ms GPS).
2. **Freshness** — last good read < threshold (IMU <100 ms, baro <1 s).
3. **Range** — accel ±100 m/s², altitude 0–50 km, descent rate <500 m/s, quaternion normalised.
4. **Consistency** — cross-sensor sanity (baro altitude vs GPS altitude vs integrated accel).
5. **Count** — 3 consecutive failures mark sensor unhealthy; failover to backup if available.

## Graceful-Degradation Modes

| Missing sensor | Effect |
|----------------|--------|
| KX134 | ICM-20948 only for high-G; apogee vote lacks backup accel |
| GPS | Barometer + accel only for apogee; recovery beacon still operates |
| Barometer | GPS + accel only (degrades apogee accuracy) |
| Primary IMU (ICM) | KX134 backup for accel; no orientation (no Kalman input) → guidance disabled |

## Post-Flight Diagnosis

All errors are logged to the CSV (`error_code` column) — see [[concepts/data-logging]]. Walk through the log to correlate error codes with flight-state timings.

## Related

- [[concepts/guidance-degradation]] — soft-error code 90 handling
- [[concepts/sensor-redundancy]] — failover logic
- [[concepts/system-robustness]] — 4-layer defence model
