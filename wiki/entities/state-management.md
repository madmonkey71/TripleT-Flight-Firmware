---
title: state_management — EEPROM Persistence
type: entity
tags: [state, eeprom, persistence, recovery]
created: 2026-04-15
updated: 2026-04-15
related_files: [src/state_management.cpp, src/state_management.h, src/data_structures.h]
---

Persists flight state and key altitudes to EEPROM after every state change. Enables recovery from power-loss or watchdog reset mid-flight.

## EEPROM Structure

```cpp
struct FlightStateData {
  uint8_t state;                 // FlightState enum cast to uint8_t
  float launchAltitude;          // Barometric altitude at launch
  float maxAltitude;             // Peak altitude reached so far
  float currentAltitude;         // Last known altitude
  float mainDeployAltitudeAgl;   // Dynamic main deploy altitude
  unsigned long timestamp;       // millis() at save time
  uint16_t signature;            // Magic number for validation
};
```

The `signature` field detects corrupted/uninitialized EEPROM (rejects invalid data on boot).

## When State is Saved

- After every `setFlightState()` call in `flight_logic.cpp`
- Not saved on every sensor read (avoids EEPROM wear — ~1000 write cycles per state, well within EEPROM endurance)

## Boot Recovery

On startup, `state_management` reads EEPROM:
1. Validates signature
2. If valid + state is not STARTUP/CALIBRATION → resume from saved state
3. If invalid → start fresh from STARTUP

## EEPROM Wear

Each rocket flight has ~13 state transitions maximum. At 1000 writes per state on a 100,000-cycle EEPROM, the device can survive ~7,000 flights before EEPROM wear becomes a concern.
