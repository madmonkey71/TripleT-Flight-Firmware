---
title: Defense-in-Depth Robustness Model
type: concept
tags: [safety, redundancy, reliability, architecture]
created: 2026-04-22
updated: 2026-07-02
related_files: [src/flight_logic.cpp, src/utility_functions.cpp, src/state_management.cpp, src/sensor_validator.h, src/watchdog_recovery.h]
---

Safety-critical firmware that must not fail silently. The design uses four defensive layers and documents every single-point-of-failure (SPF) with a mitigation.

## Four Layers

| Layer | Purpose | Examples |
|-------|---------|----------|
| 1. Prevention | Stop bad inputs at the door | Init-time plausibility gates (baro pressure 800–1100 hPa), pyro pins safed LOW at boot, pre-flight health gate on `arm` |
| 2. Detection | Notice something is wrong | `isSensorSuiteHealthy()` polled by the state machine, GPS fix sanity checks, guidance stability monitoring |
| 3. Mitigation | Keep flying if possible | High-G accel fallback ([[concepts/sensor-redundancy]]), graceful degradation ([[concepts/guidance-degradation]]) |
| 4. Recovery | Survive the unrecoverable | Watchdog reset with EEPROM state restore, backup timer for parachute deploy, recovery beacon |

Core rules:
- **Fail-safe default**: when uncertain, deploy the parachute.
- **Redundancy at every critical point**: apogee (4 methods), altitude (baro + GPS), acceleration (ICM + KX134).
- **Input sanity checks at critical boundaries**: GPS fix/SIV validation, baro pressure gate, Kalman dt window. (A fuller range/freshness/consistency validator exists in `src/sensor_validator.h` but is **dormant** — never instantiated.)
- **Human override**: serial commands can clear errors or force recovery; never disabled.

## Single-Point-of-Failure (SPF) Ledger

| Component | Failure mode | Mitigation |
|-----------|--------------|------------|
| Teensy (MCU hang) | Main loop stalls | Watchdog reset (`WATCHDOG_TIMEOUT_MS` = 5000 ms); EEPROM restores a safe resume state |
| Primary IMU (ICM-20948) | No data | KX134 backup for acceleration; apogee still works (baro + GPS + timer) |
| Barometer (MS5611) | No data | GPS altitude + accelerometer apogee methods still available |
| GPS | No fix | Baro + accel methods keep apogee working; recovery uses beacon |
| SD card | Write failure | Flight continues; telemetry over USB/ESP32 unaffected |
| Pyro circuit | Open or shorted | Backup timer guarantees deploy at 20 s post-burnout; the second channel is the *main* chute channel (not a redundant drogue driver) |
| Battery | Voltage sag | Voltage read every 5 s and logged only — **no shutdown or cutoff logic exists**; post-flight analysis only |

## Redundancy Map

```
Apogee detection   : barometer + accelerometer + GPS + 20-s timer  → OR / first-match (any one fires)
Altitude           : barometer primary → GPS fallback
Acceleration       : ICM-20948 (±16 g) primary → KX134 (±64 g) auto-switch when ICM > 16 g
Orientation        : Kalman fusion (gyro + accel + mag) → gravity-only fallback if gyro dies
Recovery signaling : audio beacon (2500 Hz SOS) + LED strobe + last-known GPS position
State persistence  : EEPROM write on state change (60 s throttle; forced at critical states) → read on boot
```

## Watchdog & EEPROM Recovery

1. `WDT_T4` fires if the main loop doesn't pet the dog inside `WATCHDOG_TIMEOUT_MS` (5000 ms, programmed in `setup()` — `src/TripleT_Flight_Firmware.cpp:626-629`; fed once per loop iteration).
2. Hardware reset → boot runs normally. **No reset-cause detection exists** — a watchdog reset is indistinguishable from a power cycle. (A `WatchdogRecovery` class in `src/watchdog_recovery.h` that would read the i.MX RT reset-cause register and log a "RECOVERED_FROM_WATCHDOG" banner exists as scaffolding but is **never compiled in**.)
3. `recoverFromPowerLoss()` reads EEPROM on startup; if the signature is valid it maps the saved state to a safe resume state (e.g. ARMED → PAD_IDLE, BOOST/COAST → DROGUE_DESCENT) and, for ARMED-or-later states, restores the barometer calibration reference.

See [[entities/state-management]] for the EEPROM layout and the full remapping table.

## Health-Validation Algorithm (summary)

The live health check is `isSensorSuiteHealthy()` (`src/utility_functions.cpp:406`), polled by the state machine:

1. Per-sensor init flags (`ms5611_initialized_ok`, `g_icm20948_ready`, `g_kx134_initialized_ok`, GPS init) — baro failure is warn-only before arming.
2. Barometer must be calibrated once past `CALIBRATION`.
3. At least one working IMU required for `ARMED` through `LANDED`.
4. GPS data sanity (implausible fix types/satellite counts rejected at read time).

A richer validator (`SensorValidator`, `src/sensor_validator.h`) with freshness windows, range checks (accel ±100 m/s², altitude 0–50 km), cross-sensor consistency, and 3-strike failure counting exists in the tree but is **dormant — never instantiated**; none of those checks run in the flight build.

## Failure-Scenario Walkthroughs

| Scenario | What happens |
|----------|--------------|
| Primary IMU dies mid-`COAST` | Kalman accel source falls back to KX134 (the live failover is the inline 16 g source switch — the `IMUManager` failover class is dormant); apogee unaffected |
| Baro + GPS both fail | Accelerometer method alone can still fire apogee (OR logic: Z-accel < 0 ×5); backup timer (20 s) is the last resort → drogue deploys |
| Loop hang during `BOOST` | Watchdog resets Teensy after 5 s; EEPROM recovery conservatively resumes as `DROGUE_DESCENT` (assumes apogee missed/passed) |
| Guidance stability violation | Soft error 90; servos centre; LED orange; apogee + parachutes unaffected ([[concepts/guidance-degradation]]) |
| Brownout | Battery voltage is logged (nothing acts on it in flight); on reboot EEPROM restores a safe resume state and the baro reference |

## Related

- [[concepts/apogee-detection]] — 4 methods, OR / first-match
- [[concepts/sensor-redundancy]] — acceleration fallback
- [[concepts/guidance-degradation]] — soft-error handling
- [[entities/error-handling]] — error-code taxonomy
- [[entities/state-management]] — EEPROM state persistence
