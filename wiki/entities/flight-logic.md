---
title: flight_logic — State Machine & Event Detection
type: entity
tags: [flight-logic, state-machine, apogee, landing, boost]
created: 2026-04-15
updated: 2026-04-15
related_files: [src/flight_logic.cpp, src/flight_logic.h, src/state_management.cpp]
---

Core flight state machine logic. Drives all state transitions and detects key flight events. See [[overview]] for full state diagram.

## Key Functions

| Function | Description |
|----------|-------------|
| `ProcessFlightState()` | Main dispatcher — calls per-state logic each loop iteration |
| `detectApogee() → bool` | 2-of-3 voting (baro + accel + GPS) + backup timer; see [[concepts/apogee-detection]] |
| `detectLanding() → bool` | Stable accel 0.9–1.1G for 2s + altitude stability |
| `detectBoostEnd()` | Checks for 3 consecutive readings below 0.5G (COAST_CONFIRMATION_COUNT) |
| `IsStable() → bool` | True if rocket is stable on the ground (landing precondition) |
| `resetApogeeDetectionCounters()` | Reset all counters on COAST entry |

## State Transition Thresholds (from config.h)

| Transition | Key Threshold |
|-----------|--------------|
| PAD_IDLE → ARMED | `arm` serial command |
| ARMED → BOOST | `BOOST_ACCEL_THRESHOLD` = 2.0G |
| BOOST → COAST | `COAST_ACCEL_THRESHOLD` = 0.5G × `COAST_CONFIRMATION_COUNT` = 3 readings |
| COAST → APOGEE | 2-of-3 vote + `BACKUP_APOGEE_TIME_MS` = 20,000ms |
| DROGUE_DESCENT → MAIN_DEPLOY | altitude < `MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M` = 100m AGL |
| MAIN_DESCENT → LANDED | accel 0.9–1.1G for `LANDING_CONFIRMATION_TIME_MS` = 2000ms |
| LANDED → RECOVERY | `LANDED_TIMEOUT_MS` = 10,000ms |
| ERROR → PAD_IDLE | `ERROR_RECOVERY_ATTEMPT_MS` = 10,000ms auto-recovery |

## Global State

```cpp
extern float g_main_deploy_altitude_m_agl;  // Dynamic deploy altitude (default 100m)
```

## State Persistence

Every state change calls `state_management.cpp` to write to EEPROM. On boot, state is read back so the firmware can resume correctly after a power-loss or watchdog reset. See [[entities/state-management]].
