---
title: Layered Architecture — HAL, Sensors, Flight Logic, Guidance
type: concept
tags: [architecture, layering, design-philosophy]
created: 2026-04-22
updated: 2026-07-02
related_files: [src/hal/hal_interfaces.h, src/sensors/imu_interface.h, src/flight_logic.cpp, src/guidance_control.cpp]
---

The firmware is **designed** as four layers, each one depending only on the layer below. The goal is to make desktop unit testing possible and let sensors / hardware be swapped without touching flight logic.

> **Status (2026-07):** Layers 0 and 1 exist as code but are **not wired into the flight build** — `hal_init()` is never called, `IMUManager`/`initSensors()` are referenced only inside `src/sensors/`, and the compiled flight path (`env:teensy41`) calls `Wire`/`Serial`/`EEPROM`/`SdFat` and the C-style sensor drivers (`icm_20948_functions.cpp` etc.) directly. The layer diagram below describes the *intended* architecture; the executing system is flatter. See [[queries/system-workflow-audit-2026-07]] §10.

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

## Dependency Rules (design intent)

- Upper layers **may** call lower layers.
- Lower layers **must not** call upper layers.
- No direct Arduino/Teensy API calls from Layer 1+ — go through the HAL.
- No hardware-specific sensor driver calls from Layer 2+ — go through `IMUInterface`.

These rules are **not currently enforced or followed** in the flight build: the main loop, flight logic, and guidance code call Arduino/Teensy APIs and the C-style sensor drivers directly. Native tests compile only the modules under test (with `-D UNIT_TEST_NATIVE`), so they do not catch layering violations in the main firmware. Migrating the flight path onto the abstractions is outstanding work.

## Per-Layer Summary

### Layer 0: HAL — implemented, dormant

8 pure-virtual interfaces: `ITimer`, `ISerial`, `IGPIO`, `II2C`, `IEEPROM`, `ISDCard`, `IServo`, `IWatchdog`.

Selection: `src/hal/hal_factory.h` picks `teensy_hal` or `mock_hal` at compile time via `NATIVE_TEST_BUILD` — a flag **no build environment defines** (`env:native` defines `UNIT_TEST_NATIVE` instead). `hal_init()` is never called from `setup()`, so the `g_timer`…`g_watchdog` globals stay null and the firmware uses Arduino APIs directly.

Deep dive: [[concepts/hal-abstraction]].

### Layer 1: Sensor Interfaces — implemented, dormant

`IMUInterface` (20 methods) is the common contract. Each physical sensor has an adapter that wraps its driver and exposes the interface:

- `ICM20948Sensor` — primary
- `KX134Sensor` — backup (high-g)
- `BNO085Sensor` — stub for evaluation

`IMUManager` tries primary then backup, so flight logic could ask for "a reading" rather than "a reading from the ICM". **None of this is instantiated in the flight build** — the loop reads the C drivers directly, and the only live failover is an inline Kalman accel-source switch to the KX134 when the ICM saturates above 16 g.

Deep dive: [[concepts/sensor-redundancy]].

### Layer 2: Flight Logic — live

The state machine ([[concepts/flight-state-transitions]]), apogee detection ([[concepts/apogee-detection]]), landing detection, error handling ([[entities/error-handling]]), and EEPROM persistence ([[entities/state-management]]).

Consumes sensor data via direct driver calls (globals populated by the C-style drivers) for IMU, baro, and GPS — not via `IMUManager`. Note: `src/apogee_detector.h` (the 2-of-3 voting class) sits at this layer but is dormant; the live `detectApogee()` in `flight_logic.cpp` is OR/first-match.

### Layer 3: Guidance & Control — live

PID loops (roll / pitch / yaw), trajectory following, stability monitor, servo smoother. Uses Kalman-filtered orientation and writes to servos directly through `PWMServo` (not the HAL `IServo`).

Active only in COAST / DROGUE_DESCENT / MAIN_DESCENT (BOOST has stability monitoring only); disabled permanently on sustained stability violation ([[concepts/guidance-degradation]]).

Deep dive: [[entities/guidance-control]].

## Why This Matters for Testing

The *intended* symmetry:

```
native test build             Teensy production build (intended)
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

Today the right-hand column is aspirational: production bypasses `IMUManager` and the HAL, so the native suites (`pio test -e native`, 12 suites in `test/test_<name>/`) exercise the algorithms and the dormant abstraction classes rather than the exact production wiring. Bugs in the direct-call glue code are only caught on hardware. See [[concepts/testing-strategy]] and [[queries/system-workflow-audit-2026-07]] §12.11.

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
