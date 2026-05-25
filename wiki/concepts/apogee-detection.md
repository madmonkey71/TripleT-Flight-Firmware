---
title: Multi-Path Apogee Detection
type: concept
tags: [apogee, safety, redundancy, flight-logic]
created: 2026-04-15
updated: 2026-04-15
related_files: [src/flight_logic.cpp, src/flight_logic.h, src/apogee_detector.h, src/config.h]
---

Apogee detection uses a 2-of-3 voting scheme from three independent methods, plus a backup timer. Only active in `COAST` state.

## Detection Methods

| Method | Trigger Condition | Config Key |
|--------|------------------|------------|
| **Barometric** | Altitude drops `APOGEE_BARO_DESCENT_THRESHOLD` (1.0 m) for `APOGEE_CONFIRMATION_COUNT` (5) consecutive readings | `APOGEE_BARO_DESCENT_THRESHOLD` |
| **Accelerometer** | Z-axis accel < `APOGEE_ACCEL_THRESHOLD` (-0.1G) for `APOGEE_ACCEL_SAMPLES` (5) samples | `APOGEE_ACCEL_THRESHOLD` |
| **GPS** | Altitude readings show descent for `APOGEE_GPS_CONFIRMATION_COUNT` (3) samples | `APOGEE_GPS_CONFIRMATION_COUNT` |
| **Backup Timer** | `BACKUP_APOGEE_TIME_MS` (20,000 ms) after motor burnout | `BACKUP_APOGEE_TIME_MS` |

## Voting Logic

```
if (baro_votes + accel_votes + gps_votes >= 2) → APOGEE confirmed
elif (time_since_burnout > 20s) → APOGEE forced (failsafe)
```

## Key Functions

- `detectApogee()` → `bool` — returns true when apogee confirmed (`src/flight_logic.cpp`)
- `resetApogeeDetectionCounters()` — called on COAST entry to clear stale counts

## Why 2-of-3

- Single sensor failure → system still detects apogee correctly
- Noisy data → majority vote filters spikes
- Backup timer → last resort if all sensors fail (guarantees drogue deploy)

Full rationale and rejected alternatives: [[concepts/architecture-decisions]] ADR-003.
