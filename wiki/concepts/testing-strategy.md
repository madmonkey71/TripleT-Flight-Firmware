---
title: Testing Strategy
type: concept
tags: [testing, unity, native, ci, mocks]
created: 2026-04-15
updated: 2026-04-15
related_files: [test/unit/, platformio.ini, .github/workflows/test.yml]
---

Three-tier testing strategy: desktop unit tests (no hardware), integration tests (real hardware), and flight validation.

## Tier 1: Desktop Unit Tests

- **Framework**: Unity via PlatformIO `native` environment
- **Location**: `test/unit/*.cpp`
- **Hardware**: None required — uses ArduinoFake + Mock HAL
- **Run**: `pio test -e native_test` (or `pio test -e native`)
- **CI**: Runs on every push via `.github/workflows/test.yml`

### Test Suites

| File | Coverage |
|------|----------|
| `test_state_machine.cpp` | Flight state transitions, EEPROM persistence |
| `test_apogee_detection.cpp` | All three detection methods + backup timer |
| `test_guidance_failsafe.cpp` | Escalation levels, passive mode, gain reduction |
| `test_math_functions.cpp` | Hypsometric formula, quaternion math |
| `test_servo_smoother.cpp` | Rate limiting, clamping |
| `test_stability_monitor.cpp` | Threshold checks, violation persistence |

### Mock Stack

```
Unity test → Mock HAL (MockTimer, MockSerial, etc.) → src/ logic
          → ArduinoFake (Arduino.h stubs for native)
```

## Tier 2: Integration Tests (Hardware)

- GPS test via `test/compile_gps_test.sh`
- Bench test procedure documented in `BENCH_TEST_PROCEDURE.md`
- Use `pio device monitor --baud 115200` + serial commands for validation

## Tier 3: Flight Testing

- Minimum 5 test flights before release
- `FLIGHT_TEST_PREPARATION.md` — pre-flight checklist
- Post-flight analysis via CSV log + web interface

## Coverage Targets

- Overall: > 70%
- Flight-critical paths (apogee, landing, state transitions): > 95%

## Mock Sensor Strategy

1. **Synthetic**: `MockIMUSensor::setAcceleration(ax, ay, az)` — deterministic input
2. **Recorded flights**: `MockIMUSensor::loadFlightData("fixtures/nominal_flight.log")` — real data replay
3. **Failure injection**: `FailureInjector::injectSensorFailure()` — test error handling
