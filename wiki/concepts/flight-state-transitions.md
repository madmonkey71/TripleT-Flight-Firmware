---
title: Flight State Machine — Transition Reference
type: concept
tags: [state-machine, flight-logic, transitions, timing]
created: 2026-04-22
updated: 2026-04-22
related_files: [src/flight_logic.cpp, src/flight_logic.h, src/data_structures.h, src/state_management.cpp]
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
| 13 | `ERROR` | Hard-error halt; auto-recovery 10 s |

## Transition Table

| From → To | Trigger | Guards | Notes |
|-----------|---------|--------|-------|
| `STARTUP → CALIBRATION` | Hardware init complete | All `SENSOR_INIT_FAIL_*` = 0 | Else → `ERROR` |
| `CALIBRATION → PAD_IDLE` | GPS fix (≥ 4 sats, pDOP ≤ 5) OR user `calibrate` timeout | Baro zeroed | Auto-calibrate on GPS fix (see commit `eecbf86`) |
| `PAD_IDLE → ARMED` | `arm` command | `isSensorSuiteHealthy()` = true | Else error 71 (`ARM_FAIL_HEALTH_CHECK`) |
| `ARMED → BOOST` | Z-accel > `BOOST_ACCEL_THRESHOLD` (2.0 g) | None (launch is unmissable) | Non-reversible |
| `BOOST → COAST` | Z-accel < `BURNOUT_ACCEL_THRESHOLD` (0.5 g) for **3 consecutive** reads | Anti-jitter | Fix applied in commit `9a6700c` |
| `COAST → APOGEE` | 2-of-3 vote (baro + accel + GPS) OR `BACKUP_APOGEE_TIME_MS` (20 s) since burnout | Only in `COAST` | See [[concepts/apogee-detection]] |
| `APOGEE → DROGUE_DEPLOY` | `DROGUE_PRESENT` = true | — | Otherwise → `MAIN_DEPLOY` directly |
| `APOGEE → MAIN_DEPLOY` | `DROGUE_PRESENT` = false AND `MAIN_PRESENT` = true | — | Single-deploy configuration |
| `DROGUE_DEPLOY → DROGUE_DESCENT` | Pyro fire cycle complete (non-blocking, ~250 ms) | — | Pyro firing uses non-blocking timer; see [[entities/flight-logic]] |
| `DROGUE_DESCENT → MAIN_DEPLOY` | Baro altitude < main-deploy altitude AGL (default ~100 m) | `MAIN_PRESENT` = true | |
| `MAIN_DEPLOY → MAIN_DESCENT` | Pyro cycle complete | — | |
| `MAIN_DESCENT → LANDED` | Accel magnitude 0.9–1.1 g for 2 s AND altitude rate ~0 | Anti-false-landing | |
| `LANDED → RECOVERY` | 5 s post-landing stabilisation | — | Beacon begins |
| `RECOVERY → (terminal)` | None | — | Persistent beacon until power-off |
| `(any non-ERROR) → ERROR` | Hard error code set (see [[entities/error-handling]]) | — | Red LED; servos centred |
| `ERROR → PAD_IDLE` | 10 s auto-recovery loop passes health check, OR `clear_errors` command | `isSensorSuiteHealthy()` = true | Preserves launch viability |

## Grace Periods

| Gate | Duration | Reason |
|------|----------|--------|
| Burnout confirmation | 3 reads | Reject thrust-curve noise/dip |
| Apogee confirmation | 5 baro samples / 3 GPS samples | Reject descent spikes |
| Landing confirmation | 2 s stable accel | Avoid parachute-snap false trigger |
| Error auto-recovery | 10 s | Avoid oscillating in/out of `ERROR` |
| Stability violation | 500 ms | Debounce transient gyro spikes before soft-error 90 |

## Timing Budget (per main-loop iteration, ~10 ms)

| Activity | Approx cost |
|----------|-------------|
| IMU read + Kalman update | 2–3 ms |
| Baro / GPS / KX134 read (conditional) | 1 ms total |
| Flight logic + transition check | <1 ms |
| Guidance update (if active) | 1–2 ms |
| Servo output / smoother | <1 ms |
| Data logging (every 10 ms; flush batched) | 1 ms amortised |
| Slack | 2–4 ms |

If a loop ever exceeds 1000 ms, the watchdog fires. See [[concepts/system-robustness]].

## Edge Cases

- **Power loss mid-flight** → Teensy resets → EEPROM restores state → flight resumes in its pre-reset state. See [[entities/state-management]].
- **Sensor failure during `BOOST`** → degrade to remaining sensors; hard failure of all accel sources → still rely on baro for apogee.
- **Stability violation during `COAST`** → soft error 90, guidance disabled, parachutes still deploy normally (critical safety behaviour; verified by unit test).
- **GPS never locks during calibration** → baro calibration proceeds on timeout fallback.
- **Arming while sensors unhealthy** → rejected with `ARM_FAIL_HEALTH_CHECK` (71); remain in `PAD_IDLE`.

## Related

- [[overview]] — top-level state diagram
- [[entities/flight-logic]] — code entry points
- [[entities/state-management]] — persistence
- [[concepts/apogee-detection]] — the COAST→APOGEE vote
- [[concepts/guidance-degradation]] — soft-error path
- [[entities/error-handling]] — hard-error path
