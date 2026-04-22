---
title: Defense-in-Depth Robustness Model
type: concept
tags: [safety, redundancy, reliability, architecture]
created: 2026-04-22
updated: 2026-04-22
related_files: [src/flight_logic.cpp, src/sensor_validator.h, src/watchdog_recovery.h, src/state_management.cpp]
---

Safety-critical firmware that must not fail silently. The design uses four defensive layers and documents every single-point-of-failure (SPF) with a mitigation.

## Four Layers

| Layer | Purpose | Examples |
|-------|---------|----------|
| 1. Prevention | Stop bad inputs at the door | Compile-time config checks, range validators, pre-flight sensor checks |
| 2. Detection | Notice something is wrong | Continuous sensor health polling, timeouts, rate-of-change checks |
| 3. Mitigation | Keep flying if possible | Automatic failover ([[concepts/sensor-redundancy]]), graceful degradation ([[concepts/guidance-degradation]]) |
| 4. Recovery | Survive the unrecoverable | Watchdog reset with EEPROM state restore, backup timer for parachute deploy, recovery beacon |

Core rules:
- **Fail-safe default**: when uncertain, deploy the parachute.
- **Redundancy at every critical point**: apogee (4 methods), altitude (baro + GPS), acceleration (ICM + KX134).
- **Strict input validation**: every external value is range-checked.
- **Human override**: serial commands can clear errors or force recovery; never disabled.

## Single-Point-of-Failure (SPF) Ledger

| Component | Failure mode | Mitigation |
|-----------|--------------|------------|
| Teensy (MCU hang) | Main loop stalls | Watchdog reset (1000 ms); EEPROM restores last state |
| Primary IMU (ICM-20948) | No data | KX134 backup for acceleration; apogee still works (baro + GPS + timer) |
| Barometer (MS5611) | No data | GPS altitude + accelerometer methods still vote |
| GPS | No fix | Baro + accel methods keep apogee working; recovery uses beacon |
| SD card | Write failure | Flight continues; telemetry over USB/ESP32 unaffected |
| Pyro circuit | Open or shorted | Backup timer guarantees deploy at 20 s post-burnout; second channel (if present) provides redundancy |
| Battery | Voltage sag | Graceful shutdown above ~10.5 V; logged low-voltage warning before |

## Redundancy Map

```
Apogee detection   : barometer  + accelerometer + GPS + 20-s timer  → 2-of-3 vote
Altitude           : barometer primary → GPS fallback
Acceleration       : ICM-20948 (±16 g) primary → KX134 (±64 g) auto-switch > ~2 g
Orientation        : Kalman fusion (gyro + accel + mag) → gravity-only fallback if gyro dies
Recovery signaling : audio beacon (4 kHz) + LED strobe + last-known GPS position
State persistence  : EEPROM write after every state change → read on boot
```

## Watchdog & EEPROM Recovery

1. `WDT_T4` fires if main loop doesn't pet the dog inside 1000 ms.
2. Hardware reset → boot runs normally.
3. `state_management.cpp` reads EEPROM on startup; if signature valid and state ≥ `ARMED`, resume that state.
4. Startup banner logs "RECOVERED_FROM_WATCHDOG" for post-flight correlation.

See [[entities/state-management]] for the EEPROM layout.

## Health-Validation Algorithm (summary)

From `isSensorSuiteHealthy()` in `src/flight_logic.cpp` and `src/sensor_validator.h`:

1. Every sensor must respond within its polling interval.
2. Data must be fresh (< 100 ms IMU, < 1 s baro).
3. Values must be in range (accel ±100 m/s², altitude 0–50 km, quaternion ≈ normalised).
4. Rate-of-change sanity (no 1 km altitude jumps in 100 ms).
5. 3 consecutive failures → sensor marked unhealthy; failover runs.

## Failure-Scenario Walkthroughs

| Scenario | What happens |
|----------|--------------|
| Primary IMU dies mid-`COAST` | `IMUManager` switches to KX134; orientation degraded but apogee unaffected |
| Baro + GPS both fail | Accel method alone cannot vote 2-of-3 → backup timer (20 s) forces APOGEE → drogue deploys |
| Loop hang during `BOOST` | Watchdog resets Teensy; EEPROM returns to `BOOST`; flight continues |
| Guidance stability violation | Soft error 90; servos centre; LED orange; apogee + parachutes unaffected ([[concepts/guidance-degradation]]) |
| Brownout | Voltage monitor logs warning; flight continues if above hard cutoff; on recovery EEPROM state restored |

## Related

- [[concepts/apogee-detection]] — 4-method voting
- [[concepts/sensor-redundancy]] — primary/backup failover
- [[concepts/guidance-degradation]] — soft-error handling
- [[entities/error-handling]] — error-code taxonomy
- [[entities/state-management]] — EEPROM state persistence
