---
title: Layered Architecture — HAL, Sensors, Flight Logic, Guidance
type: concept
tags: [architecture, layering, design-philosophy]
created: 2026-04-22
updated: 2026-04-22
related_files: [src/hal/hal_interfaces.h, src/sensors/imu_interface.h, src/flight_logic.cpp, src/guidance_control.cpp]
---

The firmware is organised in four layers, each one depending only on the layer below. This is what makes desktop unit testing possible and lets sensors / hardware be swapped without touching flight logic.

## Layer Diagram

```
┌─────────────────────────────────────────┐
│ Guidance & Control                       │   PID, trajectory, stability monitor, servo smoother
│   src/guidance_control.cpp              │
│   src/stability_monitor.cpp             │
│   src/servo_smoother.cpp                │
├─────────────────────────────────────────┤
│ Flight Logic & State Machine             │   14-state FSM, apogee/landing detection,
│   src/flight_logic.cpp                  │   sensor-health arbitration, EEPROM persistence
│   src/state_management.cpp              │
│   src/apogee_detector.h                 │
├─────────────────────────────────────────┤
│ Sensor Interfaces                        │   IMUInterface contract; adapters for ICM/KX134/BNO085;
│   src/sensors/imu_interface.h           │   IMUManager redundancy; sensor_factory selection
│   src/sensors/*_sensor.h                │
│   src/sensors/imu_manager.h             │
├─────────────────────────────────────────┤
│ Hardware Abstraction Layer (HAL)         │   ITimer/ISerial/IGPIO/II2C/IEEPROM/ISDCard/
│   src/hal/hal_interfaces.h              │   IServo/IWatchdog pure-virtual interfaces
│   src/hal/teensy_hal.h  (production)    │
│   src/hal/mock_hal.h    (native tests)  │
└─────────────────────────────────────────┘
```

## Dependency Rules

- Upper layers **may** call lower layers.
- Lower layers **must not** call upper layers.
- No direct Arduino/Teensy API calls from Layer 1+ — go through the HAL.
- No hardware-specific sensor driver calls from Layer 2+ — go through `IMUInterface`.

Violations are defects. Build/tests fail on native if a higher layer reaches past its abstraction (`Arduino.h` is not in the native include path).

## Per-Layer Summary

### Layer 0: HAL

8 pure-virtual interfaces: `ITimer`, `ISerial`, `IGPIO`, `II2C`, `IEEPROM`, `ISDCard`, `IServo`, `IWatchdog`.

Selection: `src/hal/hal_factory.h` picks `teensy_hal` or `mock_hal` at compile time (via `-DNATIVE_TEST_BUILD`).

Deep dive: [[concepts/hal-abstraction]].

### Layer 1: Sensor Interfaces

`IMUInterface` (20 methods) is the common contract. Each physical sensor has an adapter that wraps its driver and exposes the interface:

- `ICM20948Sensor` — primary
- `KX134Sensor` — backup (high-g)
- `BNO085Sensor` — stub for evaluation

`IMUManager` tries primary then backup, so flight logic asks for "a reading" rather than "a reading from the ICM".

Deep dive: [[concepts/sensor-redundancy]].

### Layer 2: Flight Logic

The state machine ([[concepts/flight-state-transitions]]), apogee detection ([[concepts/apogee-detection]]), landing detection, error handling ([[entities/error-handling]]), and EEPROM persistence ([[entities/state-management]]).

Consumes sensor data via `IMUManager` + direct driver calls for baro/GPS. Does not care whether it's running on Teensy or desktop.

### Layer 3: Guidance & Control

PID loops (roll / pitch / yaw), trajectory following, stability monitor, servo smoother. Uses Kalman-filtered orientation from the flight-logic layer and writes to servos via HAL.

Disabled in non-active flight states; disabled permanently on stability violation ([[concepts/guidance-degradation]]).

Deep dive: [[entities/guidance-control]].

## Why This Matters for Testing

```
native test build             Teensy production build
──────────────────────────    ──────────────────────────────
test harness                  main loop (TripleT_Flight_Firmware.cpp)
  ↓                             ↓
flight_logic                  flight_logic           (same code)
  ↓                             ↓
IMUManager                    IMUManager             (same code)
  ↓                             ↓
MockIMUSensor                 ICM20948Sensor         (different adapter)
  ↓                             ↓
MockHAL                       TeensyHAL              (different backend)
```

The flight-logic and guidance code paths are **byte-identical** in both builds. Bugs found on native are real bugs in production. See [[concepts/testing-strategy]].

## Adding a New Layer Participant

| Goal | Touch |
|------|-------|
| New MCU port | Write a new HAL implementation; no flight-logic changes |
| New IMU model | Write an `IMUInterface` adapter; add sensor-factory branch |
| New flight phase | Add state to `FlightState` enum; add transitions in `flight_logic.cpp`; update tests |
| New guidance algorithm | Add module under Layer 3; wire through `guidance_control.cpp` |

## Related

- [[concepts/hal-abstraction]]
- [[concepts/sensor-redundancy]]
- [[concepts/testing-strategy]]
- [[entities/flight-logic]]
- [[entities/guidance-control]]
