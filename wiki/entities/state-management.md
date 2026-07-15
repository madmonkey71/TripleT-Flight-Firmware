---
title: state_management — EEPROM Persistence
type: entity
tags: [state, eeprom, persistence, recovery]
created: 2026-04-15
updated: 2026-07-02
related_files: [src/state_management.cpp, src/state_management.h, src/data_structures.h]
---

Persists flight state, key altitudes, and the barometer calibration reference to EEPROM. Enables recovery from power-loss or watchdog reset mid-flight. The save/restore path operates directly on the live `g_` globals used by the state machine (`src/state_management.cpp:14-25`); the historical "alias variable" decoupling that neutralised recovery was removed in 2026-07.

## EEPROM Structure

Stored at `EEPROM_STATE_ADDR` (address 0):

```cpp
// src/data_structures.h:124
struct FlightStateData {
  uint8_t state;                 // FlightState enum cast to uint8_t
  float launchAltitude;          // Barometric altitude at launch
  float maxAltitude;             // Peak altitude reached so far
  float currentAltitude;         // Last known altitude
  float mainDeployAltitudeAgl;   // Dynamic main deploy altitude
  float baroAltitudeOffset;      // GPS-referenced barometer calibration offset (m)
  uint8_t baroCalibrated;        // 1 if barometer was calibrated at save time
  unsigned long timestamp;       // millis() at save time
  uint16_t signature;            // Magic number for validation (EEPROM_SIGNATURE_VALUE)
};
```

The `signature` field detects corrupted/uninitialized EEPROM (rejects invalid data on boot). Note: the 2026-07 layout change (adding `baroAltitudeOffset`/`baroCalibrated`) shifts the signature offset, which **intentionally invalidates records written by older firmware** — they fail the signature check and boot proceeds fresh from `STARTUP`.

## When State is Saved

- `saveStateToEEPROM()` is called on state transitions, throttled to one write per `EEPROM_UPDATE_INTERVAL` (60 s) — except at the critical states `APOGEE`, `DROGUE_DEPLOY`, `MAIN_DEPLOY`, and `LANDED`, which always save immediately (`src/state_management.cpp:31-35`)
- Not saved on every sensor read (avoids EEPROM wear)

## Boot Recovery

On startup, `recoverFromPowerLoss()` (`src/state_management.cpp:96`) reads EEPROM:

1. Validates signature; invalid → start fresh from `STARTUP`
2. If valid, restores `g_launchAltitude`, `g_maxAltitudeReached`, `g_main_deploy_altitude_m_agl`
3. If the saved state is `ARMED` or later and `baroCalibrated` was set, restores `baro_altitude_offset` and `g_baroCalibrated` — an in-flight reset keeps its altitude reference (AGL math, main-deploy gate, landing detection)
4. Maps the saved state to a **safe resume state** rather than resuming verbatim:

| Saved state | Resumes as |
|-------------|------------|
| `STARTUP` / `CALIBRATION` / `PAD_IDLE` | `STARTUP` (fresh sequence, recalibrates) |
| `ARMED` | `PAD_IDLE` (must re-arm) |
| `BOOST` / `COAST` | `DROGUE_DESCENT` (conservative: assume apogee passed) |
| `APOGEE` / `DROGUE_DEPLOY` | `DROGUE_DEPLOY` (ensures drogue fires) |
| `DROGUE_DESCENT` | `DROGUE_DESCENT` |
| `MAIN_DEPLOY` | `MAIN_DEPLOY` |
| `MAIN_DESCENT` / `LANDED` / `RECOVERY` | `RECOVERY` |
| `ERROR` | `ERROR` |

## EEPROM Wear

Each rocket flight has ~13 state transitions maximum, and the 60 s throttle keeps steady-state writes rarer still. At ~13 writes per flight on a 100,000-cycle EEPROM, the device can survive well over 7,000 flights before EEPROM wear becomes a concern.
