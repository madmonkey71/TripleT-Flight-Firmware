# TripleT Flight Firmware - Testing Infrastructure

## Overview

Unit tests for flight-critical code using PlatformIO's native platform and Unity test framework. Tests run on desktop without hardware.

## Directory Structure

```
test/
├── unit/              # Unit tests (.cpp files)
│   ├── test_state_machine.cpp
│   ├── test_apogee_detection.cpp
│   ├── test_math_functions.cpp
│   └── ...
├── mocks/             # Mock implementations
│   ├── mock_sensors.h
│   └── failure_injector.h
├── fixtures/          # Test data (recorded flights)
│   └── (TBD)
└── README.md
```

## Running Tests

```bash
pio test -e native          # Run all tests
pio test -e native -f test  # Run specific test file
pio test -e native -vv      # Verbose output
```

## Writing Tests

```cpp
#include <unity.h>
#include "../../src/[module].h"

extern "C" {

void setUp(void) {
  // Before each test
}

void test_descriptive_name(void) {
  TEST_ASSERT_EQUAL(expected, actual);
}

}
```

## Mock Sensor Usage

### Synthetic Data
```cpp
MockIMUSensor sensor;
sensor.setAcceleration(0, 0, 50);
sensor.read();
```

### Recorded Flight Playback
```cpp
sensor.loadFlightData(flight_data, size);
while (sensor.playNextFrame()) {
  // Test with real data
}
```

### Failure Injection
```cpp
sensor.injectFailure(true);
```

## Test Coverage Targets

- State Machine: 95%
- Apogee Detection: 100%
- Math Functions: 100%
- Overall: 70%+

## Phase 3 Status

- ✅ Mock sensor framework created
- ✅ Math function tests created
- ✅ State machine test stubs created
- ✅ Apogee detection test stubs created
- ⏳ GitHub Actions CI/CD integration (next)
- ⏳ Additional test suites (GPS, landing, etc.)
