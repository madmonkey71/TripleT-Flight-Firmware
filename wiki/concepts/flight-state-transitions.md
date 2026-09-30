---
title: Flight State Machine — Transition Reference
type: concept
tags: [state-machine, flight-logic, transitions, timing]
created: 2026-04-22
updated: 2026-09-30
related_files: [src/flight_logic.cpp, src/flight_logic.h, src/data_structures.h, src/state_management.cpp, src/startup_state.cpp, src/flight_commands.cpp, src/pyro_control.cpp]
---

Detailed reference for every transition in the 14-state flight state machine — entry conditions, guards, timing budgets, and edge cases. The top-level diagram lives in [[overview]]; this page is the per-transition detail.

## States (from `FlightState` enum)

| # | State | Purpose |
|---|-------|---------|
| 0 | `STARTUP` | Boot, hardware init |
| 1 | `CALIBRATION` | Sensor bias + baro reference, waits for GPS |
| 2 | `PAD_IDLE` | Armed-but-waiting; accepts config commands |
| 3 | `ARMED` | Watching for launch |
| 4 | `BOOST` | Motor burning |
| 5 | `COAST` | Post-burnout ballistic ascent |
| 6 | `APOGEE` | Instantaneous transition state |
| 7 | `DROGUE_DEPLOY` | Fires drogue pyro |
| 8 | `DROGUE_DESCENT` | Descending under drogue |
| 9 | `MAIN_DEPLOY` | Fires main pyro |
| 10 | `MAIN_DESCENT` | Descending under main |
| 11 | `LANDED` | Stable on ground |
| 12 | `RECOVERY` | Beacon active (audio + LED + GPS) |
| 13 | `ERROR` | Pre-flight hard-error halt only (never entered in flight); auto-recovery checked every 2 s, and only when provably on the ground and never flown |

## Transition Table

Updated for beta-0.58 ([[queries/flight-logic-audit-2026-09]]). "Fresh" = counted per new sensor sample, not per loop pass.

| From → To | Trigger | Guards | Notes |
|-----------|---------|--------|-------|
| `STARTUP → CALIBRATION` | Hardware init complete, baro not yet calibrated | System healthy (SD present if logging enabled; ≥ 1 IMU alive — baro failure is warn-only) | Unhealthy → `ERROR` (70); already calibrated → `PAD_IDLE` directly. A pending in-flight recovery holds the vehicle in `STARTUP` (inert) until decided |
| `STARTUP → (saved state)` | Boot recovery | See the recovery table in [[entities/state-management]] | Never restores `DROGUE_DEPLOY`/`MAIN_DEPLOY`; needs live baro evidence to resume in flight |
| `CALIBRATION → PAD_IDLE` | GPS 3D fix (fixType ≥ 3, pDOP < 3.0) OR `CALIBRATION_AUTO_TIMEOUT_MS` (120 s) fallback (offset 0) OR `skip_calibration` | Baro zeroed | |
| `PAD_IDLE → ARMED` | `arm` command | `isSensorSuiteHealthy()` = true | Else error 71. Entering ARMED **re-zeroes the launch altitude from the mean of the last `LAUNCH_ALT_AVG_SAMPLES` pad samples** and freezes it |
| `ARMED → PAD_IDLE` | `disarm` OR `ARMED_TIMEOUT_MS` (300 s) | — | |
| `ARMED → BOOST` | |a| > `BOOST_ACCEL_THRESHOLD` (2.0 g) for `LAUNCH_CONFIRMATION_COUNT` (5) **fresh** samples | Fresh, non-stale accelerometer | A pad bump cannot launch. Sets & persists `flightInProgress` |
| `BOOST → COAST` | Burnout: `COAST_CONFIRMATION_COUNT` (3) fresh samples with |a| < 0.5 g, or below 35 % of the peak boost level *and settled* | — | OR `BOOST_TIMEOUT_MS` (12 s) elapsed (burnout time set) |
| `COAST → APOGEE` | Any one of baro / free-fall accel / GPS (each fresh-confirmed, gated and cross-checked) OR `BACKUP_APOGEE_TIME_MS` (20 s) since burnout | Min time after burnout, min climb, transonic lockout, vetoes — see [[concepts/apogee-detection]] | |
| `APOGEE → DROGUE_DEPLOY` | `DROGUE_PRESENT` | — | Otherwise → `MAIN_DEPLOY` |
| `DROGUE_DEPLOY → DROGUE_DESCENT` | `pyro_service()` completes the `PYRO_FIRE_DURATION` window (records the fired bit) | Channel not already fired | The state machine only *requests* the fire; the service owns the pin |
| `DROGUE_DESCENT → MAIN_DEPLOY` | `MAIN_DEPLOY_CONFIRMATION_COUNT` (3) consecutive fresh baro samples below the deploy altitude; OR baro unavailable and the estimated-descent-time fallback expired; OR `MAIN_DEPLOY_MAX_DROGUE_TIME_MS` | `MAIN_PRESENT` | |
| `DROGUE_DESCENT → LANDED` | Stationary (baro span < 1 m over 10 fresh samples, ~1 g, IMU quiet) for 2 s, or `DESCENT_STATE_TIMEOUT_MS` | | Main never deployed |
| `MAIN_DEPLOY → MAIN_DESCENT` | Pyro window complete | | |
| `MAIN_DESCENT → LANDED` | Same stationarity test (works at any elevation, not "back at the launch altitude") OR `DESCENT_STATE_TIMEOUT_MS` | | |
| `LANDED → RECOVERY` | `LANDED_TIMEOUT_MS` (10 s) | — | Beacon begins |
| `RECOVERY / LANDED / ERROR / PAD_IDLE → PAD_IDLE` | `reset_flight <token>` | At rest (baro vertical speed ~0, ~1 g); token from the first `reset_flight` | The only way back after a flight |
| `(pre-flight state) → ERROR` | Hard error / failed health check | **`flight_error_allowed()`**: `STARTUP/CALIBRATION/PAD_IDLE/ARMED` only, and `flightInProgress` clear | In flight the same failures *degrade* (log, orange LED, guidance off, error code) and never leave the state machine |
| `ERROR → PAD_IDLE / CALIBRATION` | Auto-recovery (every 2 s) OR `clear_errors` / `clear_to_calibration` / `skip_calibration` | Healthy **and `flight_is_provably_on_ground()`** (never flown; within `GROUND_AGL_TOLERANCE_M` of launch altitude when the baro can tell) | Refusals print `REFUSED: …` |

## Grace Periods

| Gate | Duration | Reason |
|------|----------|--------|
| Launch confirmation | 5 fresh samples | Reject pad bumps |
| Burnout confirmation | 3 fresh samples | Reject thrust-curve noise/dip |
| Apogee confirmation | 5 fresh baro / 5 fresh accel / 3 fresh GPS samples | Reject descent spikes (each with gates and cross-checks) |
| Main deploy | 3 fresh baro samples | Reject a single glitchy reading |
| Landing confirmation | 10 consecutive fresh stationary samples + 2 s | Avoid parachute-descent / frozen-baro false trigger |
| Error auto-recovery | checked every 2 s; 5 s grace after clearing | Avoid oscillating in/out of `ERROR` (`ERROR_RECOVERY_ATTEMPT_MS` = 10 s is defined but unused) |
| Stability violation | 500 ms | Debounce transient gyro spikes before soft-error 90 |

## Timing Budget (per main-loop iteration, ~10 ms)

| Activity | Approx cost |
|----------|-------------|
| IMU read + Kalman update | 2–3 ms |
| Baro / GPS / KX134 read (conditional) | 1 ms total |
| Flight logic + transition check | <1 ms |
| Guidance update (if active) | 1–2 ms |
| Servo output / smoother | <1 ms |
| Data logging (5 Hz / 200 ms gate; SD flush every 10 writes) | 1 ms amortised |
| Slack | 2–4 ms |

If a loop ever blocks longer than `WATCHDOG_TIMEOUT_MS` (5 s), the watchdog fires. See [[concepts/system-robustness]].

## Edge Cases

- **Reset mid-flight** → EEPROM is read at boot but an in-flight state is only resumed with live barometer evidence (altitude window *and* vertical rate); otherwise the vehicle restarts in `RECOVERY`. BOOST/COAST resume as `COAST` with the backup timer restored. Stale EEPROM on the pad never fires a pyro. See [[entities/state-management]].
- **Sensor failure during flight** → degrade (never `ERROR`): guidance off, deployment logic unchanged; a dead barometer falls back to time-based main deployment, dead accelerometers to the BOOST timeout and backup timer.
- **Pyro window interrupted by a state change** → `pyro_service()` still ends it after `PYRO_FIRE_DURATION`; a completed channel is never fired again.
- **Stability violation during `COAST`** → soft error 90, guidance disabled, parachutes still deploy normally (critical safety behaviour; verified by unit test).
- **GPS never locks during calibration** → baro calibration proceeds on timeout fallback.
- **Arming while sensors unhealthy** → rejected with `ARM_FAIL_HEALTH_CHECK` (71); remain in `PAD_IDLE`.

## Related

- [[overview]] — top-level state diagram
- [[entities/flight-logic]] — code entry points
- [[entities/state-management]] — persistence
- [[concepts/apogee-detection]] — the COAST→APOGEE detection chain
- [[concepts/guidance-degradation]] — soft-error path
- [[entities/error-handling]] — hard-error path
