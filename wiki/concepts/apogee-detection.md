---
title: Multi-Path Apogee Detection
type: concept
tags: [apogee, safety, redundancy, flight-logic]
created: 2026-04-15
updated: 2026-07-02
related_files: [src/flight_logic.cpp, src/flight_logic.h, src/apogee_detector.h, src/config.h]
---

Apogee detection runs four independent methods with OR / first-match logic — any single method confirms apogee; there is no cross-method vote. Only active in `COAST` state.

## Detection Methods

Evaluated in priority order inside `detectApogee()` (`src/flight_logic.cpp:1034`); each later method is skipped once an earlier one has fired.

| Method | Trigger Condition | Config Key |
|--------|------------------|------------|
| **Barometric** (primary) | Current AGL altitude drops more than `APOGEE_BARO_DESCENT_THRESHOLD` (1.0 m) below the tracked AGL maximum for `APOGEE_CONFIRMATION_COUNT` (5) consecutive readings | `APOGEE_BARO_DESCENT_THRESHOLD` |
| **Accelerometer** (secondary) | ICM-20948 Z-axis accel < 0.0 g for `APOGEE_ACCEL_CONFIRMATION_COUNT` (5) consecutive samples | `APOGEE_ACCEL_CONFIRMATION_COUNT` |
| **GPS** (tertiary) | GPS altitude ≥ 5 m (hard-coded) below the tracked GPS maximum for `APOGEE_GPS_CONFIRMATION_COUNT` (3) samples; requires any fix | `APOGEE_GPS_CONFIRMATION_COUNT` |
| **Backup Timer** (failsafe) | `BACKUP_APOGEE_TIME_MS` (20,000 ms) after motor burnout | `BACKUP_APOGEE_TIME_MS` |

The barometric comparison is AGL-to-AGL: `ms5611_get_altitude() − g_launchAltitude` against `g_maxAltitudeReached` (also tracked in AGL) — `src/flight_logic.cpp:1044-1055`. (An earlier revision compared absolute altitude against the AGL maximum, which silently disabled this method at launch sites above sea level.)

Note: `APOGEE_ACCEL_THRESHOLD` (−0.1 g) and `APOGEE_ACCEL_SAMPLES` are defined in `src/config.h` but **unused** — the live accelerometer check uses 0.0 g and `APOGEE_ACCEL_CONFIRMATION_COUNT`.

## Detection Logic

```
if (baro AGL drop > 1.0 m below max, x5)     → APOGEE (Barometer)
elif (accel Z < 0.0 g, x5)                   → APOGEE (Accelerometer)
elif (GPS alt 5 m below max, x3)             → APOGEE (GPS)
elif (time_since_burnout > 20 s)             → APOGEE forced (Backup Timer)
```

## Key Functions

- `detectApogee()` → `bool` — returns true when apogee confirmed (`src/flight_logic.cpp:1034`)
- `resetApogeeDetectionCounters()` — called on COAST entry to clear stale counts (`src/flight_logic.cpp:1027`)

## Why OR / First-Match

- Single sensor failure → the next method in the chain still detects apogee
- Confirmation counts (5/5/3 consecutive samples) filter noise spikes within each method
- Backup timer → last resort if all sensors fail (guarantees drogue deploy)
- Trade-off vs. voting: faster detection with a single healthy sensor, but no cross-sensor plausibility check — a single noisy-but-persistent sensor can fire the transition

## Dormant Voting Implementation

A full **2-of-3 voting** implementation (`ApogeeDetector` class, votes ≥ 2 of baro/accel/GPS with a 60 s timeout) exists in `src/apogee_detector.h` but is **never instantiated** anywhere in `src/` — it is dormant scaffolding, not the live algorithm. [[concepts/architecture-decisions]] ADR-003 describes that voting design; the shipped behaviour is the OR chain above.
