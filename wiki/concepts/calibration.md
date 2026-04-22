---
title: Sensor Calibration Procedures
type: concept
tags: [calibration, barometer, magnetometer, gyroscope, gps]
created: 2026-04-22
updated: 2026-04-22
related_files: [src/command_processor.cpp, src/ms5611_functions.cpp, src/icm_20948_functions.cpp, src/gps_functions.cpp]
---

Three sensors require calibration before flight: barometer (ground reference), gyroscope (bias), magnetometer (hard/soft iron). Barometer calibration is **mandatory** — every flight zeroes altitude to the current pad elevation.

## Barometer Calibration

Sets current altitude as 0 m AGL by sampling pressure and deriving a reference offset.

**Manual**: `calibrate` serial command.
**Automatic**: fires on GPS fix acquisition during `CALIBRATION` state; falls back to a timeout if GPS never locks (commit `eecbf86`).

Preconditions:
- Rocket sitting at the pad (do not move during cal).
- GPS: ≥ 4 satellites, pDOP ≤ 5 (preferred, gives an absolute reference via GPS MSL altitude).
- MS5611 responding to I2C polls.

Outcome:
- Reference pressure/altitude saved to RAM; current `altitude_m_agl = 0`.
- Offset persisted to EEPROM for watchdog-reset recovery.
- On failure: error `BARO_CALIBRATION_FAIL_NO_GPS` (60) or `BARO_CALIBRATION_FAIL_TIMEOUT` (61).

## Gyroscope Bias Calibration

Samples stationary gyro readings to compute a per-axis bias that is subtracted during flight.

**Command**: `calibrate_gyro` (or included in a combined `calibrate` where defined).
**Procedure**: hold the rocket motionless and level for a few seconds; the firmware averages samples and stores the bias.

Why this matters: an uncalibrated gyro drifts attitude during the Kalman prediction step, which poisons guidance and the orientation log field. See [[concepts/kalman-filter]].

## Magnetometer Calibration

Corrects hard-iron and soft-iron distortion from nearby metal / electronics.

**Command**: `calibrate_mag` → follow prompts; rotate the rocket through multiple orientations (figure-of-eight motion for a few seconds).
**Persist**: `save_mag_cal` → writes bias/scale matrix to EEPROM.
**Load**: on boot, firmware reads the stored calibration; failure logs `MAG_CALIBRATION_LOAD_FAIL` (62).

Skip scenario: if the firmware doesn't use the magnetometer in the Kalman update (configurable), calibration is optional.

## Recalibration Triggers

Do a fresh calibration whenever any of the following is true:

- First flight of the day (baro).
- Firmware update (any calibration can be affected).
- Sensor replacement (mandatory).
- Long storage (bias drift).
- Moved to a significantly different launch site (baro for altitude ref, mag for magnetic field changes).

## Verification

After calibration, run `status_sensors` and check:

| Sensor | Expected at rest |
|--------|-----------------|
| ICM accel magnitude | ≈ 9.81 m/s² |
| Baro altitude | 0 ± a few metres |
| GPS | 3D fix, sats listed, reasonable pDOP |
| Gyro bias (post-cal) | ±a few °/s per axis |

See [[entities/command-processor]] for command syntax.

## Common Failure Modes

| Symptom | Probable cause | Fix |
|---------|----------------|-----|
| Altitude starts > 100 m | Baro never calibrated (no `calibrate` run) | Run `calibrate`; verify GPS fix or accept timeout fallback |
| Orientation drifts visibly on pad | Gyro bias not captured | Re-run `calibrate_gyro`; hold still |
| Heading rotates while stationary | Bad mag cal | Re-run `calibrate_mag` + `save_mag_cal` |
| Error 62 at boot | EEPROM mag-cal signature invalid | Rerun mag calibration and save |
| Error 60 at cal | GPS not locked | Wait for GPS, or accept timeout fallback |

## Related

- [[entities/command-processor]] — command list
- [[entities/error-handling]] — error codes 60–62
- [[concepts/kalman-filter]] — consumes calibrated gyro/accel/mag
