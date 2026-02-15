# AI.md - Generic LLM Guidance

This file provides guidance for any AI assistant (Claude, Gemini, etc.) working on the TripleT Flight Firmware project.

## Project Overview

**TripleT Flight Firmware** is a comprehensive flight computer system for high-altitude rocket launches with:
- 13-state flight state machine (STARTUP → CALIBRATION → ... → LANDED → RECOVERY)
- Multiple redundant sensors (barometric altimeter, accelerometers, GPS)
- Real-time flight data logging and web interface
- Safety-critical pyro channel control
- Hardware Abstraction Layer for desktop testing

**Current Version**: Check `src/config.h` for actual version
**Target Hardware**: Teensy 4.1 ARM Cortex-M7
**Repository**: Local directory `/mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware`

## Key Files to Know

### Architecture & Planning
- **IMPLEMENTATION_PLAN_2026.md** - 5-phase refactoring roadmap (read first if starting new work)
- **CLAUDE.md** - Claude-specific guidance (also useful for other LLMs)
- **MEMORY.md** - Persistent notes across sessions (check before starting)
- **docs/DEVELOPER_GUIDE.md** - Detailed development patterns (check for new patterns added)

### Core Configuration
- **src/config.h** - Flight parameters, safety limits, hardware configuration, version number
- **src/debug_flags.h** - Enable/disable debug output for different subsystems
- **src/data_structures.h** - Shared types, LogData struct definition

### Flight Logic (Most Critical - Read First)
- **src/flight_logic.cpp** - State machine implementation, state transition logic
- **src/state_management.cpp** - State persistence to EEPROM, recovery on power loss
- **src/guidance_control.cpp** - Pyro channel control (SAFETY CRITICAL)
- **src/apogee_detection.cpp** or flight_logic.cpp - Apogee detection with 2-of-3 voting

### Hardware Abstraction Layer (HAL)
- **src/hal/hal_interfaces.h** - Pure virtual interfaces for all hardware
- **src/hal/teensy_hal.cpp** - Real hardware implementations (production use)
- **src/hal/mock_hal.cpp** - Mock implementations (desktop testing)
- **src/hal/hal_factory.h** - Dependency injection for HAL selection

### Sensors
- **src/icm_20948_functions.cpp** - IMU sensor driver (±16G accelerometer)
- **src/kx134_functions.cpp** - High-G accelerometer (±64G)
- **src/ms5611_functions.cpp** - Barometric pressure sensor
- **src/gps_functions.cpp** - GPS/GNSS receiver with SPI interface
- **src/sensors/imu_interface.h** - Abstract IMU interface for sensor swapping
- **src/sensors/imu_manager.cpp** - Redundancy and fallback logic between sensors
- **src/sensors/sensor_factory.h** - Factory pattern for sensor creation

### Data & Logging
- **src/log_format_definition.cpp** - SD card logging format and CSV headers
- **src/command_processor.cpp** - Serial commands (arm, calibrate, status_sensors, etc.)
- **web_interface/** - Web-based flight data viewer

### Testing
- **test/unit/*.cpp** - 47+ unit tests (no hardware required)
- **test/mocks/*.h** - Mock implementations of sensors and HAL
- **test/fixtures/*.log** - Recorded flight data for replay testing
- **.github/workflows/test.yml** - GitHub Actions CI/CD pipeline

## Essential Commands

### Build & Upload
```bash
pio run -e teensy41              # Compile for Teensy 4.1
pio run -t upload                # Compile and upload to hardware
pio run -t clean                 # Clean build artifacts
pio device monitor --baud 115200 # Monitor serial output
```

### Testing
```bash
pio test -e native_test          # Run all desktop unit tests
pio lib list                      # List dependencies
pio lib update                    # Update all libraries
```

### Git
```bash
git status                        # See current changes
git log --oneline -10             # Recent commits
git branch                        # Current branch
git diff HEAD                     # See uncommitted changes
```

## Project Structure

```
├── src/
│   ├── main.cpp                 # Entry point, setup loop
│   ├── config.h                 # Configuration (version, parameters)
│   ├── debug_flags.h            # Debug output control
│   ├── data_structures.h        # Shared type definitions
│   ├── flight_logic.cpp         # State machine & flight logic
│   ├── state_management.cpp     # State persistence
│   ├── guidance_control.cpp     # Pyro control (SAFETY CRITICAL)
│   ├── error_handling.cpp       # Error state machine
│   ├── apogee_detection.cpp     # Altitude peak detection
│   ├── kalman_filter.cpp        # IMU orientation filtering
│   ├── command_processor.cpp    # Serial commands
│   ├── log_format_definition.cpp# Data logging format
│   ├── hal/                     # Hardware Abstraction Layer
│   │   ├── hal_interfaces.h     # Pure virtual interfaces
│   │   ├── teensy_hal.cpp       # Real implementations
│   │   ├── mock_hal.cpp         # Test implementations
│   │   └── hal_factory.h        # Dependency injection
│   ├── sensors/                 # Sensor drivers & abstractions
│   │   ├── imu_interface.h      # IMU abstraction
│   │   ├── imu_manager.cpp      # Sensor redundancy
│   │   ├── sensor_factory.h     # Sensor creation
│   │   ├── icm20948_sensor.cpp  # IMU adapter
│   │   ├── bno085_sensor.cpp    # Alternative IMU
│   │   ├── kx134_functions.cpp  # High-G accelerometer
│   │   ├── ms5611_functions.cpp # Barometer
│   │   └── gps_functions.cpp    # GPS receiver
│   └── [other sensor functions]
├── test/
│   ├── unit/                    # Unit tests (Unity framework)
│   │   ├── test_state_machine.cpp
│   │   ├── test_apogee_detection.cpp
│   │   ├── test_gps_calibration.cpp
│   │   └── [47+ total test files]
│   ├── mocks/                   # Mock implementations
│   │   ├── mock_sensors.h
│   │   └── failure_injector.h
│   └── fixtures/                # Recorded flight data
│       ├── nominal_flight_2025.log
│       └── [other recorded flights]
├── web_interface/               # Web-based data viewer
├── docs/                        # Developer & user documentation
├── IMPLEMENTATION_PLAN_2026.md  # Refactoring roadmap (READ THIS FIRST)
├── CLAUDE.md                    # Claude-specific guidance
├── AI.md                        # This file (generic LLM guidance)
├── MEMORY.md                    # Persistent session notes
├── platformio.ini               # Build configuration
└── README.md                    # Project overview
```

## Safety-Critical Code Locations

⚠️ **These sections require extra care and testing:**

1. **Apogee Detection** - `src/flight_logic.cpp` or `src/apogee_detection.cpp`
   - Must detect peak altitude correctly
   - 2-of-3 voting (barometric + accelerometer + GPS or timer)
   - Extensive unit tests required
   - Always test on real hardware before deployment

2. **Pyro Channel Control** - `src/guidance_control.cpp`
   - Must fire parachute deployment charges at correct times
   - MUST NOT fire before APOGEE state
   - All state transitions must be verified
   - Code review required for any changes

3. **Error State Recovery** - `src/error_handling.cpp` + `src/flight_logic.cpp`
   - System must not enter ERROR state due to sensor noise
   - Graceful degradation when sensors fail
   - Recovery paths must maintain flight safety

4. **State Machine Transitions** - `src/state_management.cpp`
   - All 13 states must transition correctly
   - No skipped states (enforced by design)
   - EEPROM persistence on power loss

## Development Workflow

### Starting Work
1. Read `IMPLEMENTATION_PLAN_2026.md` to understand current phase
2. Check `MEMORY.md` for context from previous sessions
3. Verify current version in `src/config.h`
4. Check current branch: `git branch`
5. Review acceptance criteria for current checkpoint

### During Development
1. **Always maintain a working system** - commits should compile
2. **Run tests frequently**: `pio test -e native_test`
3. **Verify on hardware**: `pio run -e teensy41` compiles
4. **Write tests** for new flight-critical code
5. **Use conventional commits** (feat/fix/test/docs/refactor)
6. **Document architectural decisions** in MEMORY.md or docs/

### Before Committing
```bash
pio run -e teensy41              # Verify production build
pio test -e native_test          # Verify all tests pass
git diff HEAD                    # Review changes
git status                       # See all files changed
```

### Resuming After Break
1. Pull latest: `git fetch origin`
2. Check MEMORY.md for context
3. Review recent commits: `git log --oneline develop -10`
4. Identify your current work branch
5. Verify build state: `pio run -e teensy41`
6. Review current checkpoint acceptance criteria

## Testing Strategy

### Unit Tests (Desktop - No Hardware)
- Run via: `pio test -e native_test`
- Framework: Unity (C-based unit test framework)
- Mocks: All hardware abstracted via HAL
- 47+ tests covering flight logic, sensors, state machine
- Target: >70% overall coverage, >95% for flight-critical code

### Integration Tests (Real Hardware)
- Flight test with actual Teensy 4.1
- Sensor validation on real data
- State transitions with real timing
- Apogee detection accuracy
- Power loss recovery from EEPROM

### Test Fixtures
- **Synthetic data**: `MockIMUSensor::setAcceleration(ax, ay, az)`
- **Recorded flights**: `MockIMUSensor::loadFlightData("test/fixtures/nominal.log")`
- **Failure injection**: `FailureInjector::injectSensorFailure(SENSOR_GPS)`

## Common Patterns

### State Machine Pattern
```cpp
// In flight_logic.cpp
FlightState current_state = PAD_IDLE;

while (true) {
  switch (current_state) {
    case PAD_IDLE:
      if (arm_signal_received) {
        current_state = ARMED;
      }
      break;
    case ARMED:
      if (acceleration_threshold_exceeded) {
        current_state = BOOST;
      }
      break;
    case BOOST:
      if (apogee_detected()) {
        current_state = APOGEE;
        fire_drogue_chute();
      }
      break;
    // ... etc for all 13 states
  }
}
```

### Sensor Reading with HAL
```cpp
// Use HAL interfaces instead of direct hardware calls
ITimer* timer = HALFactory::createTimer();
ISerial* serial = HALFactory::createSerial();

uint32_t start = timer->millis();
imu->read();
serial->println("Accel X: ");
serial->print(imu->getAccelX());
```

### Sensor Abstraction (IMU)
```cpp
// Sensors implement IMUInterface
class ICM20948Sensor : public IMUInterface {
  bool read() override { /* read from hardware */ }
  float getAccelX() override { /* return X */ }
  // ... etc for all 11 methods
};

// Use through IMUManager for redundancy
imu_manager->read();
float ax = imu_manager->getAccelX();  // Auto-selects healthy sensor
```

### Testing Example
```cpp
// In test/unit/test_apogee.cpp
void test_apogee_detection_nominal_flight(void) {
  MockIMUSensor imu;
  imu.loadFlightData("test/fixtures/nominal_flight.log");

  while (imu.playNextFrame()) {
    FlightState state = flight_logic_update();
    if (state == APOGEE) {
      TEST_ASSERT_EQUAL_FLOAT(1255.5, get_altitude());  // ±5m
      return;
    }
  }
  TEST_FAIL("Apogee not detected");
}
```

## Version Management

**Current versions follow semantic versioning:**
- `v0.6.0-dev` - Current development version
- `v0.7.0` - HAL Foundation complete
- `v0.8.0` - Sensor Modularity complete
- `v0.9.0` - Testing Infrastructure complete
- `v0.10.0` - Safety Features complete
- `v1.0.0` - Production Release

**Update version in:** `src/config.h`
```cpp
#define FIRMWARE_VERSION "v0.7.0"  // Update this
```

**Tag releases in git:**
```bash
git tag -a v0.7.0 -m "v0.7.0 - HAL Foundation Complete"
git push origin v0.7.0
```

## Asking for Help

If you encounter unclear requirements, architectural questions, or blockers:

1. **Check existing documentation:**
   - IMPLEMENTATION_PLAN_2026.md (if implementing a phase)
   - CLAUDE.md (if Claude-specific patterns needed)
   - docs/DEVELOPER_GUIDE.md (if patterns not documented yet)
   - MEMORY.md (if context from previous sessions)

2. **For safety-critical code:**
   - Always err on the side of caution
   - Add more tests, not fewer
   - Consider edge cases and failure modes
   - Document assumptions and invariants

3. **For architectural decisions:**
   - Reference the HAL pattern for hardware abstraction
   - Use IMUInterface pattern for sensor modularity
   - Follow factory pattern for object creation
   - Use dependency injection for testability

## Success Metrics

✅ Code compiles: `pio run -e teensy41` succeeds
✅ Tests pass: `pio test -e native_test` all pass
✅ No new warnings in build output
✅ Safety-critical code has test coverage
✅ Commits have conventional message format
✅ Branch has no merge conflicts with develop
✅ PR checklist complete before merge

## Questions or Issues?

- **Build problems**: Check `platformio.ini` and library versions
- **Test failures**: Check mock sensor setup and test fixtures
- **Design questions**: Reference IMPLEMENTATION_PLAN_2026.md and MEMORY.md
- **Git issues**: Use `git status`, `git log`, `git diff` to understand current state
- **Sensor-specific**: Check the sensor datasheet and existing driver code

---

**Document Status**: Current as of February 2026
**For Claude specifically**: See CLAUDE.md
**For implementation guidance**: See IMPLEMENTATION_PLAN_2026.md
