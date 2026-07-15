# AI.md - Generic LLM Guidance

This file provides guidance for any AI assistant (Claude, Gemini, etc.) working on the TripleT Flight Firmware project.

## Project Overview

**TripleT Flight Firmware** is a comprehensive flight computer system for high-altitude rocket launches with:
- 14-state flight state machine (STARTUP → CALIBRATION → ... → LANDED → RECOVERY, plus ERROR; enum values 0-13)
- Multiple redundant sensors (barometric altimeter, accelerometers, GPS)
- Real-time flight data logging and web interface
- Safety-critical pyro channel control
- Hardware Abstraction Layer for desktop testing

**Current Version**: Check `src/config.h` for actual version
**Target Hardware**: Teensy 4.1 ARM Cortex-M7
**Repository**: Local directory `/mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware`

## Key Files to Know

### Architecture & Planning
- **wiki/index.md** - Wiki catalogue — start here for architecture, concepts, modules, queries
- **wiki/overview.md** - Tech stack, directory layout, state machine, phase summary
- **wiki/concepts/architecture-decisions.md** - ADRs (HAL, IMUInterface, apogee voting, EEPROM persistence, Kalman)
- **wiki/queries/roadmap-2026.md** - Phase plan and current version status
- **wiki/concepts/developer-workflow.md** - Build / test / flash cycle and common dev tasks
- **Claude auto-memory** - Persistent cross-session notes managed by Claude Code at `~/.claude/projects/-mnt-GAMES-SSD-matt-Code-TripleT-Flight-Firmware/memory/MEMORY.md` (not in the project tree). Check it at the start of a session for prior context.

### Core Configuration
- **src/config.h** - Flight parameters, safety limits, hardware configuration, version number
- **src/debug_flags.h** - Enable/disable debug output for different subsystems
- **src/data_structures.h** - Shared types, LogData struct definition

### Flight Logic (Most Critical - Read First)
- **src/flight_logic.cpp** - State machine implementation, state transition logic, pyro channel firing (SAFETY CRITICAL), apogee/landing detection
- **src/state_management.cpp** - State persistence to EEPROM, recovery on power loss
- **src/guidance_control.cpp** - PID fin guidance and trajectory following (pyro control is in flight_logic.cpp, not here)
- **src/apogee_detector.h** - Dormant 2-of-3 voting class; the LIVE apogee logic is `detectApogee()` in flight_logic.cpp, which is OR/first-match (any single method fires)

### Hardware Abstraction Layer (HAL) — implemented but DORMANT (not wired into the flight build)
- **src/hal/hal_interfaces.h** - Pure virtual interfaces for all hardware
- **src/hal/teensy_hal.h** - Real hardware implementations (header-only)
- **src/hal/mock_hal.h** - Mock implementations (desktop testing, header-only)
- **src/hal/hal_factory.h** - Dependency injection for HAL selection (`hal_init()` is never called; the flight build uses Arduino APIs directly)

### Sensors
- **src/icm_20948_functions.cpp** - IMU sensor driver (±16G accelerometer) — these C-style drivers are what the flight build actually calls
- **src/kx134_functions.cpp** - High-G accelerometer (±64G)
- **src/ms5611_functions.cpp** - Barometric pressure sensor
- **src/gps_functions.cpp** - GPS/GNSS receiver (I2C by default, `GPS_USE_SPI=0`; init sets the u-blox `DYN_MODEL_AIRBORNE4g` dynamic model)
- **src/sensors/imu_interface.h** - Abstract IMU interface for sensor swapping (dormant OO stack)
- **src/sensors/imu_manager.h** - Redundancy and fallback logic between sensors (dormant — never instantiated by the flight build)
- **src/sensors/sensor_factory.h** - Factory pattern for sensor creation (dormant)

### Data & Logging
- **src/log_format_definition.cpp** - SD card logging format and CSV headers (63 columns, single source of truth)
- **src/command_processor.cpp** - Serial commands (arm, disarm, calibrate, status, etc.)
- **src/telemetry.cpp/.h** - 40-byte packet / 43-byte framed Serial5 downlink (behind `ENABLE_TELEMETRY`, default 0)
- **web_interface/** - Web-based flight data viewer

### Testing
- **test/test_<name>/** - 12 Unity test suites (no hardware required), auto-discovered by PlatformIO
- **test/mocks/mock_sensors.h** - Mock sensor implementations
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
pio test -e native               # Run all desktop unit tests (12 suites)
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
│   ├── TripleT_Flight_Firmware.cpp # Entry point: setup(), loop(), sensor polls, logging
│   ├── config.h                 # Configuration (version, parameters)
│   ├── debug_flags.h            # Debug output control
│   ├── data_structures.h        # Shared type definitions (LogData, FlightState enum)
│   ├── error_codes.h            # ErrorCode_t enum + names
│   ├── flight_logic.cpp         # State machine, apogee/landing detection, pyro control (SAFETY CRITICAL)
│   ├── state_management.cpp     # State persistence
│   ├── guidance_control.cpp     # PID fin guidance & trajectory following
│   ├── guidance_failsafe.cpp    # Stability-triggered failsafe escalation
│   ├── stability_monitor.cpp    # Angular rate / attitude error monitoring
│   ├── servo_smoother.cpp       # Rate-limited servo output
│   ├── kalman_filter.cpp        # IMU orientation filtering (3-state scalar AHRS)
│   ├── command_processor.cpp    # Serial commands
│   ├── log_format_definition.cpp# Data logging format (63 CSV columns)
│   ├── telemetry.cpp/.h         # Serial5 downlink (behind ENABLE_TELEMETRY=0)
│   ├── apogee_detector.h        # 2-of-3 voting class (dormant)
│   ├── sensor_validator.h       # Cross-sensor checks (dormant)
│   ├── watchdog_recovery.h      # Watchdog reset helpers (dormant)
│   ├── icm_20948_functions.cpp  # IMU driver (live)
│   ├── kx134_functions.cpp      # High-G accelerometer driver (live)
│   ├── ms5611_functions.cpp     # Barometer driver (live)
│   ├── gps_functions.cpp        # GPS receiver driver (live)
│   ├── hal/                     # Hardware Abstraction Layer (dormant scaffolding)
│   │   ├── hal_interfaces.h     # Pure virtual interfaces
│   │   ├── teensy_hal.h         # Real implementations (header-only)
│   │   ├── mock_hal.h           # Test implementations (header-only)
│   │   └── hal_factory.h        # Dependency injection
│   └── sensors/                 # OO sensor abstractions (dormant scaffolding)
│       ├── imu_interface.h      # IMU abstraction (20 pure virtual methods)
│       ├── imu_manager.h        # Sensor redundancy
│       ├── sensor_factory.h/.cpp# Sensor creation
│       ├── icm20948_sensor.h    # IMU adapter
│       ├── kx134_sensor.h       # High-G adapter
│       └── bno085_sensor.h      # Alternative IMU (stub)
├── test/
│   ├── test_<name>/             # 12 Unity suites, one folder per suite (PlatformIO layout)
│   │   ├── test_state_machine/  test_flight_logic/  test_apogee_detection/
│   │   ├── test_landing_detection/  test_guidance_failsafe/  test_stability_monitor/
│   │   ├── test_sensor_health/  test_gps_validation/  test_altitude_calculations/
│   │   └── test_math_functions/  test_servo_smoother/  test_telemetry/
│   └── mocks/                   # Mock implementations (mock_sensors.h)
├── web_interface/               # Web-based data viewer
├── esp32_telemetry_transmitter/ # Onboard ESP32 radio TX firmware
├── esp32_ground_station_receiver/ # Ground station ESP32 firmware
├── wiki/                        # All project documentation (architecture, concepts, modules)
├── .archived/                   # Historical/superseded docs (preserved for context)
├── AI.md                        # This file (generic LLM guidance)
├── platformio.ini               # Build configuration
└── README.md                    # Project overview
```

## Safety-Critical Code Locations

⚠️ **These sections require extra care and testing:**

1. **Apogee Detection** - `detectApogee()` in `src/flight_logic.cpp` (the 2-of-3 voting class in `src/apogee_detector.h` is dormant)
   - Must detect peak altitude correctly
   - Live logic is OR/first-match across baro / accel / GPS descent, plus a 20 s backup timer
   - Extensive unit tests required
   - Always test on real hardware before deployment

2. **Pyro Channel Control** - `src/flight_logic.cpp` (DROGUE_DEPLOY / MAIN_DEPLOY handling; pins in `src/config.h`)
   - Must fire parachute deployment charges at correct times
   - MUST NOT fire before APOGEE state
   - All state transitions must be verified
   - Code review required for any changes

3. **Error State Recovery** - `src/error_codes.h` + `src/flight_logic.cpp`
   - System must not enter ERROR state due to sensor noise
   - Graceful degradation when sensors fail
   - Recovery paths must maintain flight safety

4. **State Machine Transitions** - `src/flight_logic.cpp` + `src/state_management.cpp`
   - All 14 states must transition correctly
   - No skipped states (enforced by design)
   - EEPROM persistence on power loss

## Development Workflow

### Starting Work
1. Read `wiki/queries/roadmap-2026.md` to understand current phase
2. Check Claude's auto-memory (`~/.claude/projects/-mnt-GAMES-SSD-matt-Code-TripleT-Flight-Firmware/memory/MEMORY.md`) for cross-session context
3. Verify current version in `src/config.h`
4. Check current branch: `git branch`
5. Review acceptance criteria for current checkpoint

### During Development
1. **Always maintain a working system** - commits should compile
2. **Run tests frequently**: `pio test -e native -vv`
3. **Verify on hardware**: `pio run -e teensy41` compiles
4. **Write tests** for new flight-critical code (see `wiki/concepts/testing-strategy.md`)
5. **Use conventional commits** (feat/fix/test/docs/refactor)
6. **Document architectural decisions** in `wiki/concepts/architecture-decisions.md`

### Before Committing
```bash
pio run -e teensy41              # Verify production build
pio test -e native -vv           # Verify all tests pass
git diff HEAD                    # Review changes
git status                       # See all files changed
```

### Resuming After Break
1. Pull latest: `git fetch origin`
2. Check Claude's auto-memory for cross-session context (`~/.claude/projects/-mnt-GAMES-SSD-matt-Code-TripleT-Flight-Firmware/memory/MEMORY.md`)
3. Review recent commits: `git log --oneline develop -10`
4. Identify your current work branch
5. Verify build state: `pio run -e teensy41`
6. Review current checkpoint acceptance criteria

## Testing Strategy

### Unit Tests (Desktop - No Hardware)
- Run via: `pio test -e native -vv`
- Framework: Unity (C-based unit test framework)
- Layout: 12 suites in `test/test_<name>/` (PlatformIO auto-discovery)
- Mocks: hardware mocked per suite (`test/mocks/`)
- Target: >70% overall coverage, >95% for flight-critical code (aspirational — some suites are stubs)

### Integration Tests (Real Hardware)
- Flight test with actual Teensy 4.1
- Sensor validation on real data
- State transitions with real timing
- Apogee detection accuracy
- Power loss recovery from EEPROM

### Test Fixtures
- **Synthetic data**: `MockIMUSensor::setAcceleration(ax, ay, az)` (see `test/mocks/mock_sensors.h`)
- Recorded-flight replay and failure injection are planned patterns — there is no `test/fixtures/` directory yet

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
    // ... etc for all 14 states
  }
}
```

### Sensor Reading with HAL
Note: the HAL and IMUManager patterns below are the *intended* architecture — the current flight build calls Arduino APIs and the C-style drivers directly (the abstraction layers are dormant scaffolding).
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
  // ... etc for all 20 methods
};

// Use through IMUManager for redundancy
imu_manager->read();
float ax = imu_manager->getAccelX();  // Auto-selects healthy sensor
```

### Testing Example
```cpp
// In test/test_apogee_detection/ (illustrative — fixture replay is not implemented yet)
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
- `v0.7.0` - HAL Foundation complete
- `v0.8.0` - Sensor Modularity complete
- `v0.9.0` - Testing Infrastructure complete
- `v0.10.0` - Safety Features complete (**current** — see `src/config.h`)
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
   - `wiki/index.md` (catalogue — start here)
   - `wiki/queries/roadmap-2026.md` (if implementing a phase)
   - `wiki/concepts/developer-workflow.md` (if asking "how do I…?")
   - `wiki/concepts/architecture-decisions.md` (if asking "why was it built this way?")
   - Claude auto-memory (`~/.claude/projects/-mnt-GAMES-SSD-matt-Code-TripleT-Flight-Firmware/memory/MEMORY.md`) for cross-session context

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
✅ Tests pass: `pio test -e native -vv` all pass
✅ No new warnings in build output
✅ Safety-critical code has test coverage
✅ Commits have conventional message format
✅ Branch has no merge conflicts with develop
✅ PR checklist complete before merge

## Questions or Issues?

- **Build problems**: Check `platformio.ini` and library versions
- **Test failures**: Check mock sensor setup and test fixtures
- **Design questions**: Reference `wiki/concepts/architecture-decisions.md` and Claude's auto-memory
- **Git issues**: Use `git status`, `git log`, `git diff` to understand current state
- **Sensor-specific**: Check the sensor datasheet and existing driver code

---

**Document Status**: Current as of 2026-07-02
**For wiki structure & conventions**: See `wiki/schema.md`
**For implementation guidance**: See `wiki/queries/roadmap-2026.md`
