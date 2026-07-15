---
title: Sensor Calibration Procedures
type: concept
tags: [calibration, barometer, magnetometer, gyroscope, gps]
created: 2026-04-22
updated: 2026-07-02
related_files: [src/command_processor.cpp, src/ms5611_functions.cpp, src/icm_20948_functions.cpp, src/gps_functions.cpp]
---

Three sensors require calibration before flight: barometer (ground reference), gyroscope (bias), magnetometer (hard/soft iron). Barometer calibration is **mandatory** — every flight zeroes altitude to the current pad elevation.

## Barometer Calibration

Sets current altitude as 0 m AGL by sampling pressure and deriving a reference offset.

**Manual**: `calibrate` serial command (valid in `PAD_IDLE`, `CALIBRATION`, or `ERROR`), or `skip_calibration` to force offset 0.
**Automatic**: fires on GPS fix acquisition during `CALIBRATION` state; falls back after `CALIBRATION_AUTO_TIMEOUT_MS` (120 s) to offset 0 if GPS never locks (commit `eecbf86`).

Preconditions:
- Rocket sitting at the pad (do not move during cal).
- GPS: 3D fix (fixType ≥ 3) with pDOP < 3.0 — there is no satellite-count check; the GPS MSL altitude gives the absolute reference (`offset = GPS_alt − raw_baro_alt`).
- MS5611 responding to I2C polls, pressure within 700–1200 hPa.

Outcome:
- Reference offset applied in RAM; current `altitude_m_agl = 0`.
- Offset also persisted to EEPROM as part of `FlightStateData` (`baroAltitudeOffset` / `baroCalibrated` fields) on state saves; it is restored on reboot only when recovering into `ARMED` or a later state — pre-arm recovery recalibrates fresh. See [[entities/state-management]].
- On failure: error `BARO_CALIBRATION_FAIL_NO_GPS` (60) or `BARO_CALIBRATION_FAIL_TIMEOUT` (61).

## Gyroscope Bias Calibration

Samples stationary gyro readings to compute a per-axis bias that is subtracted during flight.

**Command**: `calibrate_gyro` (separate from the baro-only `calibrate` command).
**Procedure**: hold the rocket motionless and level; the firmware discards 100 warm-up reads, averages 2000 samples, and stores the per-axis bias in RAM (not persisted across reboots).

Why this matters: an uncalibrated gyro drifts attitude during the Kalman prediction step, which poisons guidance and the orientation log field. See [[concepts/kalman-filter]].

## Magnetometer Calibration

Corrects hard-iron and soft-iron distortion from nearby metal / electronics.

**Command**: `calibrate_mag` → interactive 30-second routine (`ICM_20948_calibrate_mag_interactive()`, `src/icm_20948_functions.cpp:492`): rotate the rocket slowly through all orientations (figure-of-eight motion) while the firmware records each axis' min/max envelope. It then computes the hard-iron bias (envelope midpoints) and a diagonal soft-iron scale (per-axis radius normalisation; off-diagonals left at 0) and applies them immediately. Send `x` to abort. The routine rejects the capture (existing calibration unchanged) if fewer than 100 samples were taken or any axis spread is under 5 µT.
**Persist**: `save_mag_cal` → writes bias/scale matrix to EEPROM (`MAG_CAL_EEPROM_ADDR` = 100).
**Load**: on boot, firmware reads the stored calibration (falls back to hard-coded defaults); failure logs `MAG_CALIBRATION_LOAD_FAIL` (62).

Skip scenario: if the firmware doesn't use the magnetometer in the Kalman update (configurable), calibration is optional.

## Recalibration Triggers

Do a fresh calibration whenever any of the following is true:

- First flight of the day (baro).
- Firmware update (any calibration can be affected).
- Sensor replacement (mandatory).
- Long storage (bias drift).
- Moved to a significantly different launch site (baro for altitude ref, mag for magnetic field changes).

## Verification

After calibration, run `status` (and `sensor_requirements` for per-state requirements) and check:

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
