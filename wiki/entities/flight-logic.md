---
title: flight_logic — State Machine & Event Detection
type: entity
tags: [flight-logic, state-machine, apogee, landing, boost]
created: 2026-04-15
updated: 2026-07-02
related_files: [src/flight_logic.cpp, src/flight_logic.h, src/state_management.cpp]
---

Core flight state machine logic. Drives all state transitions and detects key flight events. See [[overview]] for full state diagram.

## Key Functions

| Function | Description |
|----------|-------------|
| `ProcessFlightState()` | Main dispatcher — calls per-state logic each loop iteration |
| `detectApogee() → bool` | OR / first-match of 4 methods (baro AGL descent, accel Z, GPS, 20 s backup timer); see [[concepts/apogee-detection]] |
| `detectLanding() → bool` | 10-sample avg altitude within 1 m of launch + accel 0.9–1.1 G held 2 s |
| `detectBoostEnd()` | Checks for 3 consecutive readings below 0.5G (COAST_CONFIRMATION_COUNT) |
| `resetApogeeDetectionCounters()` | Reset all counters on COAST entry |

Note: `IsStable()` is declared in `flight_logic.h` but has no definition and no callers — vestigial.

## State Transition Thresholds (from config.h)

| Transition | Key Threshold |
|-----------|--------------|
| PAD_IDLE → ARMED | `arm` serial command (gated by `isSensorSuiteHealthy(ARMED)`) |
| ARMED → PAD_IDLE | `disarm` command, or `ARMED_TIMEOUT_MS` = 300,000ms auto-disarm |
| ARMED → BOOST | `BOOST_ACCEL_THRESHOLD` = 2.0G |
| BOOST → COAST | `COAST_ACCEL_THRESHOLD` = 0.5G × `COAST_CONFIRMATION_COUNT` = 3 readings |
| COAST → APOGEE | any single method: baro AGL drop > `APOGEE_BARO_DESCENT_THRESHOLD` (1.0 m) × 5, accel Z < 0 × 5, GPS 5 m drop × 3, or `BACKUP_APOGEE_TIME_MS` = 20,000ms after burnout |
| DROGUE_DESCENT → MAIN_DEPLOY | altitude < launch AGL + `MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M` = 100m |
| MAIN_DESCENT → LANDED | accel 0.9–1.1G for `LANDING_CONFIRMATION_TIME_MS` = 2000ms |
| LANDED → RECOVERY | `LANDED_TIMEOUT_MS` = 10,000ms |
| ERROR → PAD_IDLE / CALIBRATION | auto-recovery health check every 2 s (`ERROR_RECOVERY_ATTEMPT_MS` is defined but unused), or `clear_errors` / `clear_to_calibration` |

## Global State

```cpp
extern float g_main_deploy_altitude_m_agl;  // Dynamic deploy altitude (default 100m)
```

## State Persistence

Every state change calls `state_management.cpp` to write to EEPROM (throttled to 60 s except at APOGEE / DROGUE_DEPLOY / MAIN_DEPLOY / LANDED, which always save). On boot, state is read back and remapped to a safe resume state; for ARMED-or-later states the barometer calibration offset is restored as well. See [[entities/state-management]].
