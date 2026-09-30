---
title: Multi-Path Apogee Detection
type: concept
tags: [apogee, safety, redundancy, flight-logic, fresh-samples]
created: 2026-04-15
updated: 2026-09-30
related_files: [src/flight_logic.cpp, src/flight_logic.h, src/sensor_samples.h, src/apogee_detector.h, src/config.h]
---

Apogee detection runs four methods with OR / first-match logic — any single method that passes its own confirmation **and** its gates confirms apogee; there is no cross-method vote. Only active in `COAST`. Rewritten in beta-0.58 ([[queries/flight-logic-audit-2026-09]] #4, #5, #8): confirmation now counts *fresh sensor samples*, the accelerometer method is mounting-independent, and every sensor method is gated and cross-checked.

## Fresh-sample confirmation

The flight logic runs on every main-loop pass (kHz) but the sensors update at 10 Hz. Before beta-0.58 a "5 consecutive readings" count was satisfied by **one** cached sample re-read in about a millisecond. Each driver now stamps a `SampleClock` (`seq`, `lastMs`, in `src/sensor_samples.h`) when it stores a genuinely new sample (`ms5611_read`, `ICM_20948_read`, `kx134_read`, `gps_read`); a `FreshCounter` only advances when `seq` moved, so *N counts = N sensor periods*. A sensor with no fresh sample within its stale timeout (`BARO_STALE_TIMEOUT_MS`, `ACCEL_STALE_TIMEOUT_MS`, `GPS_STALE_TIMEOUT_MS`) is treated as unavailable.

## Detection Methods

Evaluated in priority order inside `detectApogee()`; each later method is skipped once an earlier one has fired.

| Method | Trigger | Config keys |
|--------|---------|-------------|
| **Barometric** (primary) | AGL falls more than `APOGEE_BARO_DESCENT_THRESHOLD` (1.0 m) below the peak AGL seen **after the transonic lockout**, for `APOGEE_CONFIRMATION_COUNT` (5) consecutive fresh samples | `APOGEE_BARO_DESCENT_THRESHOLD`, `APOGEE_CONFIRMATION_COUNT`, `APOGEE_BARO_TRANSONIC_LOCKOUT_MS` |
| **Accelerometer** (secondary) | Magnitude of specific force below `APOGEE_ACCEL_FREEFALL_G` (0.3 g) for `APOGEE_ACCEL_SAMPLES` (5) consecutive fresh samples and `APOGEE_ACCEL_FREEFALL_WINDOW_MS` (longer, `…_NO_BARO_MS`, without a corroborating barometer). ICM preferred (low-g resolution), KX134 fallback. Independent of IMU mounting and axis sign | `APOGEE_ACCEL_FREEFALL_G`, `APOGEE_ACCEL_SAMPLES`, `APOGEE_ACCEL_FREEFALL_WINDOW_MS` |
| **GPS** (tertiary) | 3D fix, fresh PVT, altitude ≥ `APOGEE_GPS_DESCENT_THRESHOLD_M` (5 m) below its max for `APOGEE_GPS_CONFIRMATION_COUNT` (3) *consecutive* fresh samples | `APOGEE_GPS_CONFIRMATION_COUNT`, `APOGEE_GPS_DESCENT_THRESHOLD_M` |
| **Backup timer** (failsafe) | `BACKUP_APOGEE_TIME_MS` (20 s) after burnout — **ungated** | `BACKUP_APOGEE_TIME_MS` |

## Gates and cross-checks (decision D-4)

OR / first-match is kept, but no sensor method may fire alone:

- **Common gates:** at least `APOGEE_MIN_TIME_AFTER_BURNOUT_MS` (2 s) since burnout, and — with a working barometer — max AGL ≥ `APOGEE_MIN_ALTITUDE_GAIN_M` (15 m).
- **Independent cross-check per method** (absent or stale cross-check data never vetoes, so one dead sensor cannot block deployment):
  - baro is vetoed while the accelerometer still shows hard deceleration (`APOGEE_HIGH_FORCE_VETO_G`, 1.5 g);
  - accel is vetoed while the barometer is still climbing faster than `APOGEE_CLIMB_VETO_MPS` (10 m/s) — near apogee drag → 0, so a low-drag vehicle reads "free fall" for the last seconds of coast; the climb veto is what keeps it from firing early;
  - GPS is vetoed by either.
- **Transonic lockout:** the barometric method is disabled for `APOGEE_BARO_TRANSONIC_LOCKOUT_MS` (3 s) after burnout and its descent reference *restarts* when the lockout ends, so a shock-induced pressure spike cannot poison it.

```
if (baro fresh x5 below ref-1 m, past lockout, gates ok, no high-force veto)  → APOGEE (Barometer)
elif (|a| < 0.3 g fresh x5 over window, gates ok, baro not climbing)          → APOGEE (Accelerometer free fall)
elif (3D fix, GPS 5 m below max fresh x3, gates ok, no vetoes)                → APOGEE (GPS)
elif (time_since_burnout > 20 s)                                              → APOGEE forced (Backup Timer)
```

The barometric comparison is AGL-to-AGL (`ms5611_get_altitude() − g_launchAltitude`); the launch altitude is the averaged pad reference (see [[concepts/flight-state-transitions]]).

## Burnout, launch and BOOST timeout

- **Launch** (`ARMED → BOOST`): `LAUNCH_CONFIRMATION_COUNT` (5) consecutive fresh samples above `BOOST_ACCEL_THRESHOLD`.
- **Burnout** (`BOOST → COAST`): `COAST_CONFIRMATION_COUNT` (3) fresh samples where |a| is below `COAST_ACCEL_THRESHOLD` (0.5 g, low-drag vehicles) **or** below `BOOST_BURNOUT_PEAK_FRACTION` (0.35) × the smoothed peak boost level *and settled* (`BOOST_BURNOUT_SETTLE_FRACTION`), for high-drag vehicles. Uses magnitude, not a thrust-axis component, because IMU mounting is not known to the firmware. The settle test keeps a gradual thrust tail-off (the H125W) from being taken for burnout.
- **`BOOST_TIMEOUT_MS`** (12 s): forces `COAST` with the burnout timestamp set, so the sensor gates and the backup timer are always reachable.

## Key Functions

- `detectApogee()`, `detectBoostEnd()`, `detectLanding()` — `src/flight_logic.cpp`
- `resetApogeeDetectionCounters()` / `flightLogicReset()` — clear detector state (PAD_IDLE entry, resume after reset)
- `readAccel(bool preferKx134)` — fresh, non-zero accelerometer selection (`src/flight_logic.cpp`)

## Why OR / First-Match

- Single sensor failure → the next method in the chain still detects apogee
- Fresh-sample counts and gates filter noise and glitches within each method; cross-checks stop a single faulty sensor from deploying alone
- Backup timer → last resort if all sensors fail (guarantees drogue deploy)
- Trade-off vs. voting: faster detection with a single healthy sensor; the cross-checks give a cheap plausibility test without the latency of a full 2-of-3 vote

## Dormant Voting Implementation

A full **2-of-3 voting** implementation (`ApogeeDetector` class, `src/apogee_detector.h`) exists but is **never instantiated** — dormant scaffolding, not the live algorithm. [[concepts/architecture-decisions]] ADR-003 describes it.

## Verified behaviour

Replaying the H125W flight from `flight_simulation_h125w.py` through the real code (`test/test_real_h125w_replay`): burnout 2.8 s (sim 2.8 s), apogee detected 19.2 s (sim 19.8 s — the free-fall path fires ≈ 0.6 s before the apex at ≈ 5 m/s upward), well before the backup timer. See [[queries/flight-logic-audit-2026-09]].
