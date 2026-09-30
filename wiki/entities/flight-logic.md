---
title: flight_logic — State Machine & Event Detection
type: entity
tags: [flight-logic, state-machine, apogee, landing, boost]
created: 2026-04-15
updated: 2026-09-30
related_files: [src/flight_logic.cpp, src/flight_logic.h, src/state_management.cpp, src/startup_state.cpp, src/pyro_control.cpp, src/flight_commands.cpp, src/sensor_samples.h]
---

Core flight state machine logic. Drives all state transitions and detects key flight events. See [[overview]] for full state diagram.

## Key Functions

| Function | Description |
|----------|-------------|
| `ProcessFlightState()` | Main dispatcher — calls per-state logic each loop iteration; returns immediately while a boot recovery is pending |
| `detectApogee() → bool` | OR / first-match of 4 methods (baro, free-fall accel, GPS, 20 s backup timer), each fresh-sample-confirmed, gated and cross-checked; see [[concepts/apogee-detection]] |
| `detectLanding() → bool` | Stationarity on fresh baro samples (span < 1 m over 10 samples) + ~1 g + IMU quiet, consecutive, 2 s; works at any elevation |
| `detectBoostEnd()` | 3 fresh samples below 0.5 g, or below 35 % of the peak boost level and settled (drag-robust) |
| `flightGuidanceStep()` | Guidance servo policy: runs only in COAST; centres the fins once afterwards; primes `dt` on entry |
| `flight_is_airborne_state()`, `flight_error_allowed()`, `flight_is_provably_on_ground()`, `flight_is_stationary_on_ground()` | ERROR policy / ground-proof predicates ([[queries/flight-logic-audit-2026-09]] #2, #3, #7) |
| `resetApogeeDetectionCounters()`, `flightLogicReset()` | Reset detector state (COAST entry / PAD_IDLE entry / resume / tests) |

Pyro outputs are **not** driven here: the deploy states request a fire from `pyro_control.cpp`, whose `pyro_service()` (every loop pass) owns the pins. Boot-time state resolution lives in `startup_state.cpp`; the state-changing serial commands in `flight_commands.cpp`.

Note: `IsStable()` is declared in `flight_logic.h` but has no definition and no callers — vestigial.

## State Transition Thresholds (from config.h)

| Transition | Key Threshold |
|-----------|--------------|
| PAD_IDLE → ARMED | `arm` serial command (gated by `isSensorSuiteHealthy(ARMED)`) |
| ARMED → PAD_IDLE | `disarm` command, or `ARMED_TIMEOUT_MS` = 300,000ms auto-disarm |
| ARMED → BOOST | `BOOST_ACCEL_THRESHOLD` = 2.0G for `LAUNCH_CONFIRMATION_COUNT` = 5 fresh samples |
| BOOST → COAST | burnout (absolute 0.5 G or relative-and-settled) × `COAST_CONFIRMATION_COUNT` = 3 fresh samples, or `BOOST_TIMEOUT_MS` = 12,000ms |
| COAST → APOGEE | any single gated + cross-checked method (see [[concepts/apogee-detection]]), or `BACKUP_APOGEE_TIME_MS` = 20,000ms after burnout (ungated) |
| DROGUE_DESCENT → MAIN_DEPLOY | `MAIN_DEPLOY_CONFIRMATION_COUNT` = 3 fresh samples below launch AGL + `MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M` = 100m; fallbacks if the baro fails (estimated descent time; `MAIN_DEPLOY_MAX_DROGUE_TIME_MS`) |
| DROGUE/MAIN_DESCENT → LANDED | stationarity (see `detectLanding()`), or `DESCENT_STATE_TIMEOUT_MS` |
| LANDED → RECOVERY | `LANDED_TIMEOUT_MS` = 10,000ms |
| RECOVERY / LANDED / ERROR / PAD_IDLE → PAD_IDLE | `reset_flight <token>` (at rest) |
| pre-flight → ERROR | failed health check (never in flight: it degrades instead) |
| ERROR → PAD_IDLE / CALIBRATION | auto-recovery every 2 s or `clear_errors` / `clear_to_calibration` — only when provably on the ground and never flown |

## Global State

```cpp
extern float g_main_deploy_altitude_m_agl;  // Dynamic deploy altitude (default 100m)
```

## State Persistence

Every state change writes the EEPROM record (unthrottled, put-if-changed; BOOST/COAST also refresh once a second). On boot an in-flight saved state is only resumed with live barometer evidence; otherwise the vehicle restarts in the pyro-inert `RECOVERY`. Pyro completion flags are persisted so a completed channel never re-fires. See [[entities/state-management]].
