---
title: TripleT Flight Firmware — Architecture Overview
type: overview
tags: [architecture, firmware, teensy, rocketry]
created: 2026-04-15
updated: 2026-04-22
related_files: [src/config.h, src/data_structures.h, src/TripleT_Flight_Firmware.cpp, platformio.ini]
---

Flight computer firmware for a guided amateur rocket on Teensy 4.1 (ARM Cortex-M7). Handles sensor fusion, 14-state flight state machine, guidance/control, data logging, and post-flight recovery. Current version: **v0.10.0** (Safety Features phase complete).

## Tech Stack

| Layer | Technology |
|-------|-----------|
| MCU | Teensy 4.1 (ARM Cortex-M7 @ 600 MHz) |
| Framework | Arduino + PlatformIO |
| Build envs | `teensy41` (production), `native` (desktop unit tests, compile flag `-D UNIT_TEST_NATIVE`) |
| Test framework | Unity + ArduinoFake; auto-discovered from `test/test_<name>/` |
| CI/CD | GitHub Actions (`.github/workflows/test.yml`) |

## Key Libraries

- `SparkFun_ICM-20948` — Primary IMU
- `SparkFun_u-blox_GNSS` — GPS receiver
- `MS5611` (RobTillaart) — Barometric altimeter
- `SparkFun_KX13X` — High-G accelerometer (backup)
- `Adafruit_NeoPixel` — Status LED
- `PWMServo`, `WDT_T4` — Servo control, watchdog (Teensy only)

## Directory Layout

```
src/
├── TripleT_Flight_Firmware.cpp   # Main loop, sensor reads, data logging
├── config.h                      # All compile-time parameters (SINGLE TRUTH)
├── data_structures.h             # LogData, FlightState enum, Trajectory types
├── debug_flags.h                 # Granular debug output control
├── flight_logic.cpp/.h           # State machine transitions, apogee/landing detect
├── state_management.cpp/.h       # EEPROM persistence of flight state
├── guidance_control.cpp/.h       # PID guidance, trajectory following
├── guidance_failsafe.cpp         # Stability-triggered failsafe escalation
├── stability_monitor.cpp/.h      # Real-time angular rate / attitude monitoring
├── kalman_filter.cpp/.h          # Kalman-based AHRS (gyro predict + accel update)
├── command_processor.cpp/.h      # Serial command handling
├── log_format_definition.cpp/.h  # CSV headers and log format
├── servo_smoother.cpp/.h         # Rate-limited servo output
├── sensor_validator.h            # Cross-sensor sanity checks
├── utility_functions.cpp/.h      # Shared math, hypsometric formula
├── apogee_detector.h             # Multi-path apogee voting logic
├── watchdog_recovery.h           # Watchdog reset recovery helpers
├── gps_functions.cpp/.h          # GPS driver wrapper
├── icm_20948_functions.cpp/.h    # ICM-20948 driver wrapper
├── kx134_functions.cpp/.h        # KX134 driver wrapper
├── ms5611_functions.cpp/.h       # MS5611 driver wrapper
├── hal/                          # Hardware Abstraction Layer (Phase 1)
│   ├── hal_interfaces.h          # 8 pure-virtual interfaces
│   ├── teensy_hal.h              # Production Teensy implementations
│   ├── mock_hal.h                # Mock implementations for desktop tests
│   ├── hal_factory.h             # Compile-time HAL selection
│   └── hal_config.cpp            # Global g_timer, g_serial, etc. init
└── sensors/                      # Sensor Modularity Layer (Phase 2)
    ├── imu_interface.h           # IMUInterface — 20 pure-virtual methods
    ├── icm20948_sensor.h         # ICM-20948 adapter
    ├── kx134_sensor.h            # KX134 high-G adapter
    ├── bno085_sensor.h           # BNO085 stub (future evaluation)
    ├── imu_manager.h             # Dual-sensor redundancy + failover
    ├── sensor_factory.h/.cpp     # Factory pattern for sensor selection
    └── sensor_factory.cpp
test/
├── test_state_machine/           # Unity suites (one folder per suite — PlatformIO layout)
├── test_flight_logic/
├── test_apogee_detection/
├── test_landing_detection/
├── test_guidance_failsafe/
├── test_stability_monitor/
├── test_sensor_health/
├── test_gps_validation/
├── test_altitude_calculations/
├── test_math_functions/
├── test_servo_smoother/
├── fixtures/                     # Recorded flight data for replay
├── mocks/                        # Mock sensor implementations
├── GPS_Test.ino                  # Hardware-only GPS integration sketch
├── compile_gps_test.sh           # Wrapper to build GPS test
└── platformio.ini                # Test-only PIO config (gps_test_serial env)
web_interface/                    # Real-time data visualization (Web Serial API)
esp32_ground_station_receiver/    # ESP32 telemetry ground station
esp32_telemetry_transmitter/      # ESP32 telemetry transmitter
```

## Flight State Machine

14 states managed in `src/flight_logic.cpp` and persisted to EEPROM via `src/state_management.cpp`:

```mermaid
stateDiagram-v2
    [*] --> STARTUP
    STARTUP --> CALIBRATION
    CALIBRATION --> PAD_IDLE : sensors healthy
    PAD_IDLE --> ARMED : arm command
    ARMED --> BOOST : accel > 2g
    BOOST --> COAST : accel < 0.5g (3 consecutive)
    COAST --> APOGEE : 2-of-3 vote (baro + accel + GPS)
    APOGEE --> DROGUE_DEPLOY : if DROGUE_PRESENT
    APOGEE --> MAIN_DEPLOY : if single deploy
    DROGUE_DEPLOY --> DROGUE_DESCENT
    DROGUE_DESCENT --> MAIN_DEPLOY : altitude < 100m AGL
    MAIN_DEPLOY --> MAIN_DESCENT
    MAIN_DESCENT --> LANDED : stable accel 0.9–1.1g for 2s
    LANDED --> RECOVERY
    RECOVERY --> [*]
    BOOST --> ERROR : critical failure
    ERROR --> PAD_IDLE : auto-recovery (10s)
```

## Data Pipeline

```
Sensors (20-100ms) → Kalman Filter (AHRS) → Flight Logic → Guidance/PID
       ↓                                                         ↓
  Data Logging (CSV on SD) ←────── LogData struct ──────── Actuators (servos, pyros)
       ↓
  Web Interface (Web Serial API, real-time)
```

## Critical Design Decisions

See [[concepts/layered-architecture]] for the dependency model, and the individual deep dives: [[concepts/hal-abstraction]], [[concepts/sensor-redundancy]], [[concepts/apogee-detection]], [[concepts/guidance-degradation]], [[concepts/system-robustness]].

1. **Layered architecture** — HAL → sensors → flight logic → guidance; upper layers never reach past their abstraction
2. **HAL abstraction** — enables desktop unit testing without Teensy hardware
3. **IMUInterface adapters** — runtime polymorphism for sensor swap/failover
4. **2-of-3 apogee voting** — barometer + accelerometer + GPS; backup timer at 20 s
5. **EEPROM state persistence** — survives power-loss and watchdog resets
6. **Kalman filter** — replaces deprecated Madgwick; handles gyro + accel + mag (Euler state; quaternion migration planned — see [[concepts/kalman-filter]])
7. **Graceful guidance degradation** — stability failure disables guidance (orange LED), does NOT trigger `ERROR` state; parachutes still deploy normally

## Phase Progression

Development is organised into phases tracked against semantic versions. Status as of 2026-04-22:

| Phase | Version target | Scope | Status |
|-------|---------------|-------|--------|
| 1 HAL foundation | v0.7.0 | 8 HAL interfaces, Teensy + Mock | ✅ merged |
| 2 Sensor modularity | v0.8.0 | `IMUInterface`, adapters, `IMUManager` | ✅ merged |
| 3 Testing infrastructure | v0.9.0 | Unity native, ArduinoFake, CI | ✅ merged |
| 4 Safety & redundancy | v0.10.0 | Multi-path apogee, cross-validation, graceful degrade | ✅ merged (current) |
| 5 Documentation & polish | v1.0.0-rc1 | `docs/` suite + wiki | ⏳ partial |
| 6 Advanced features + production readiness | v1.0.0 | Trajectory, power/thermal, pre-flight, validation | 🟡 6.2 merged; 6.1/6.3/6.4 in progress |

Full plan: [[queries/roadmap-2026]]. Current gaps: [[queries/development-status-2026-04]]. Historical audit: [[queries/code-review-findings-2026]].

## Known Limitations

- Kalman filter uses Euler angles internally — avoid sustained pitch beyond ±80° (quaternion migration planned).
- No live wireless telemetry yet — ESP32 link is stubs only ([[entities/esp32-telemetry]]).
- Trajectory SD-card loading incomplete; only hard-coded test trajectory today.
- Fixed-timestep loop not enforced; Kalman `dt` varies slightly with loop load.

## Related

- [[concepts/layered-architecture]] — dependency model
- [[concepts/flight-state-transitions]] — detailed state transition reference
- [[concepts/system-robustness]] — 4-layer defence model
- [[entities/hardware-platform]] — physical loadout
- [[entities/configuration-system]] — `config.h` and feature flags
- [[entities/error-handling]] — error codes and recovery
- [[entities/web-interface]] — visualisation client
- [[concepts/developer-workflow]] — build / test / flash cycle
