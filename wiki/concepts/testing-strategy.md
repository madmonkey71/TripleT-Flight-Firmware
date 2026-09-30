---
title: Testing Strategy
type: concept
tags: [testing, unity, native, ci, mocks]
created: 2026-04-15
updated: 2026-09-30
related_files: [test/, test/stubs/, test/support/, platformio.ini, .github/workflows/test.yml]
---

Three-tier testing strategy: desktop unit tests (no hardware), integration tests (real hardware), and flight validation.

## Tier 1: Desktop Unit Tests

- **Framework**: Unity via PlatformIO `native` environment
- **Location**: `test/test_<name>/test_<name>.cpp` (PlatformIO auto-discovery layout)
- **Hardware**: None required — compile flags `-D UNIT_TEST_NATIVE -std=gnu++17 -I test/stubs`. Since beta-0.58 the `native` env no longer uses ArduinoFake (its only role was supplying `<Arduino.h>` types, it collided with the stub layer and fails to build on newer Apple clang); `test/stubs/Arduino.h` provides them
- **Run**: `pio test -e native -vv`
- **CI**: Runs on every push via `.github/workflows/test.yml`

### Test Suites

| Folder | Coverage |
|--------|----------|
| `test_state_machine/` | Flight state transitions, EEPROM persistence |
| `test_flight_logic/` | High-level state-machine driver, integrations |
| `test_apogee_detection/` | All three detection methods + backup timer |
| `test_landing_detection/` | Stable-accel window, recovery transition |
| `test_guidance_failsafe/` | Escalation levels, passive mode, gain reduction |
| `test_stability_monitor/` | Threshold checks, violation persistence |
| `test_sensor_health/` | Cross-sensor sanity checks, fault flagging |
| `test_gps_validation/` | Fix quality, sat count, lat/lon sanity |
| `test_altitude_calculations/` | Hypsometric formula, AGL vs MSL |
| `test_math_functions/` | Vector/quaternion math, utility helpers |
| `test_servo_smoother/` | Rate limiting, clamping |

### Real-code suites (beta-0.58)

The older `test_flight_logic` and `test_apogee_detection` suites test *copies* of the logic (`testable_detectApogee`, mocks) and could not catch the defects in [[queries/flight-logic-audit-2026-09]]. The `test_real_*` suites `#include` the **shipped** `src/*.cpp` files into one translation unit and fake only the hardware/IO boundary:

- `test/stubs/` — host Arduino core: injectable `millis()`, recorded pin modes/levels/rising edges, captured `Serial`, a RAM `EEPROM` with write accounting; header stubs for NeoPixel, MS5611, ICM, KX134, u-blox, Wire.
- `test/support/flight_harness.h` — firmware globals + recording stubs (sensor values, sensor health, guidance, logging); `flight_harness_post.h` — `harness_reset()`, fresh-sample delivery at 10 Hz, baro ramps, `harness_pass()` (same call order as `loop()`); `real_sources.h` — includes the real sources.
- Suites: `test_real_recovery`, `test_real_commands`, `test_real_persistence`, `test_real_apogee`, `test_real_boost`, `test_real_main_deploy`, `test_real_landing`, `test_real_pyro`, `test_real_guidance`, `test_real_kalman`, `test_real_setup_sequence`, `test_real_test_commands_off/on`, `test_real_flight_logic`, and `test_real_h125w_replay` (end-to-end replay of the `flight_simulation_h125w.py` flight).
- Convention: every fix ships with a test that fails if the fix is reverted (mutation-checked by hand). A few checks read the real `setup()`/`loop()`/`command_processor.cpp` text where the code cannot be compiled natively (source-order lint).

### Mock Stack

```
Unity test → test/stubs (Arduino core, pins, EEPROM, Serial) + test/support harness → REAL src/ logic
          (legacy suites: Mock HAL / mock_sensors.h → copies of the logic)
```

## Tier 2: Integration Tests (Hardware)

- GPS test via `test/compile_gps_test.sh`
- Bench test procedure archived at `.archived/BENCH_TEST_PROCEDURE.md`
- Use `pio device monitor --baud 115200` + serial commands for validation

## Tier 3: Flight Testing

- Minimum 5 test flights before release
- Pre-flight checklist archived at `.archived/FLIGHT_TEST_PREPARATION.md`
- Post-flight analysis via CSV log + web interface

## Coverage Targets

- Overall: > 70%
- Flight-critical paths (apogee, landing, state transitions): > 95%

## Mock Sensor Strategy

1. **Synthetic**: `MockIMUSensor::setAcceleration(ax, ay, az)` — deterministic input
2. **Recorded flights**: `MockIMUSensor::loadFlightData("fixtures/nominal_flight.log")` — real data replay
3. **Failure injection**: `FailureInjector::injectSensorFailure()` — test error handling

## CI/CD Pipeline

`.github/workflows/test.yml` defines two jobs that run on every push and PR:

- **`test`** — `pio test -e native -vv`; runs the Unity suites (the real-code suites against `test/stubs`).
- **`build`** — `pio run -e teensy41`; verifies the production build still links and fits the flash budget.

Branch protection on `master` / `develop` requires both green before merge. Performance budget: full CI < 5 min; unit tests < 10 s; firmware build < 30 s.

## Flight-Critical Code Coverage (non-negotiable)

These paths MUST reach > 95 %:

- Apogee detection and backup timer ([[concepts/apogee-detection]])
- Landing detection
- Pyro-firing sequencing
- State transitions at every boundary ([[concepts/flight-state-transitions]])
- Error-state entry + auto-recovery ([[entities/error-handling]])
- Graceful guidance degradation (soft error 90) ([[concepts/guidance-degradation]])

A change that drops coverage on any of these is a regression; tests should land with the code change, not after.

## Related

- [[concepts/developer-workflow]] — how to run tests day-to-day
- [[concepts/hal-abstraction]] — why native tests can exist at all
- [[concepts/sensor-redundancy]] — `IMUInterface` is the mock seam
