# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) and other LLMs when working with code in this repository.

## Getting Started / Resuming Work

**For multi-session continuity**, check:
- `MEMORY.md` - Persistent notes across sessions (patterns, decisions, lessons learned)
- `IMPLEMENTATION_PLAN_2026.md` - Complete refactoring roadmap with checkpoints
- This file (CLAUDE.md) - Project conventions and architecture overview
- `AI.md` - Generic guidance for any LLM

**Current version**: Check `src/config.h` for `#define FIRMWARE_VERSION`
**Current branch**: Use `git branch` to see where you are
**Current state**: Use `git status` and `pio run -e teensy41` to verify build

## Work Environment Rules
1. **Directory Confinement**: ALL work (creating files, editing code, running commands) MUST be performed ONLY within the project directory: `/mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware`.
2. Do not modify files outside this directory unless explicitly instructed for system-level configuration relevant to the project.

## Build and Development Commands

### PlatformIO Commands
- **Build**: `pio run` or `pio run -e teensy41`
- **Upload**: `pio run -t upload` or `pio run -e teensy41 -t upload`
- **Clean**: `pio run -t clean`
- **Monitor Serial**: `pio device monitor --baud 115200`
- **List Libraries**: `pio lib list`
- **Update Dependencies**: `pio lib update`

### Testing

#### Unit Testing (Desktop - No Hardware Required)
- **Run all tests**: `pio test -e native_test`
- **Test framework**: Unity (included in PlatformIO)
- **Test structure**: `test/unit/*.cpp` - one file per test suite
- **CI/CD**: Tests run automatically on every push via GitHub Actions (`.github/workflows/test.yml`)
- **Coverage**: Target >70% overall, >95% for flight-critical code (apogee, landing, state transitions)

#### Integration Testing (Real Hardware)
- **GPS Test**: Use files in `test/` directory
- **Compile GPS Test**: Run `test/compile_gps_test.sh` (Linux/Mac) or `test/compile_gps_test.bat` (Windows)
- **Flight Testing**: Validate on real hardware with 5+ test flights before release

#### Mock Sensor Strategy (3-Tier)
1. **Synthetic data**: `MockIMUSensor::setAcceleration(ax, ay, az)` - perfect data with optional noise
2. **Recorded flights**: `MockIMUSensor::loadFlightData("test/fixtures/nominal_flight.log")` - replay real data
3. **Failure injection**: `FailureInjector::injectSensorFailure()` - test error handling

### Web Interface Development
- **Local Server**: Open `web_interface/index.html` in a modern browser with Web Serial API support
- **Testing**: Use `web_interface/test_message_filtering.html` for data parsing validation

## High-Level Architecture

### Flight State Machine
The system centers around a 13-state flight state machine:
`STARTUP → CALIBRATION → PAD_IDLE → ARMED → BOOST → COAST → APOGEE → DROGUE_DEPLOY → DROGUE_DESCENT → MAIN_DEPLOY → MAIN_DESCENT → LANDED → RECOVERY` (plus ERROR state)

Key architectural patterns:
- **State persistence** in EEPROM for power-loss recovery
- **Automatic error recovery** with sensor health monitoring
- **State-dependent processing** - different sensor/control behaviors per state

### Sensor Fusion Architecture (Modular Design)
- **Hardware Abstraction Layer (HAL)**: All hardware interaction abstracted through interfaces (ITimer, ISerial, IGPIO, II2C, IEEPROM, ISDCard, IServo, IWatchdog)
  - Enables desktop testing without Teensy hardware
  - Supports sensor modularity and swapping (ICM-20948 ↔ BNO085)
- **IMUInterface abstraction**: Dual accelerometer setup encapsulated
  - **Primary sensor**: ICM-20948 (±16G) for normal flight
  - **Backup sensor**: KX134 (±64G) for high-G events OR BNO085 (alternative evaluation)
  - **Automatic switching** based on acceleration magnitude thresholds
  - **Sensor redundancy**: IMUManager handles fallback logic
- **Kalman filter** for primary orientation estimation (replacing deprecated Madgwick)
- **Multi-path apogee detection**: Barometric + accelerometer + GPS + backup timer with 2-of-3 voting

### Data Pipeline
Flow: `Sensors (20-100ms) → Kalman Filter → Flight Logic → Guidance Control → Actuators + Data Logging → Web Interface`

### Module Organization
- **Core modules**: `flight_logic.cpp`, `state_management.cpp`, `guidance_control.cpp`
- **Hardware Abstraction Layer**: `src/hal/`
  - `hal_interfaces.h` - Pure virtual interfaces for all hardware
  - `teensy_hal.cpp` - Real Teensy implementations (production)
  - `mock_hal.cpp` - Mock implementations (desktop testing)
  - `hal_factory.h` - Dependency injection for HAL selection
- **Sensor drivers** (with HAL abstraction):
  - `icm_20948_functions.cpp` - IMU sensor (wraps library, implements IMUInterface)
  - `kx134_functions.cpp` - High-G accelerometer
  - `ms5611_functions.cpp` - Barometric pressure/altitude
  - `gps_functions.cpp` - GPS/GNSS receiver
  - `sensors/imu_manager.cpp` - Redundancy and fallback logic
  - `sensors/sensor_factory.h` - Factory pattern for sensor creation
- **I/O systems**: `command_processor.cpp`, `log_format_definition.cpp`
- **Web integration**: `web_interface/` directory with real-time data streaming
- **Testing**: `test/unit/` - 47+ unit tests, `test/mocks/` - mock sensors and fixtures

### Configuration System
- **Primary config**: `src/config.h` - flight parameters, hardware presence, safety limits
  - **Version tracking**: `#define FIRMWARE_VERSION "v0.6.0-dev"` (update on each release)
  - **HAL selection**: Compile-time flags to select Teensy vs Mock HAL
  - **Feature flags**: GPS_USE_SPI, USE_BNO085_VARIANT (for sensor evaluation)
- **Debug control**: `src/debug_flags.h` - granular diagnostic output control
- **Data structures**: `src/data_structures.h` - shared types and LogData struct

### Refactoring & Release Plan
- **See**: `IMPLEMENTATION_PLAN_2026.md` - Comprehensive 5-phase roadmap
  - Phase 1: HAL Foundation (v0.6.0-dev → v0.7.0)
  - Phase 2: Sensor Modularity (v0.7.0 → v0.8.0)
  - Phase 3: Testing Infrastructure (v0.8.0 → v0.9.0)
  - Phase 4: Safety Features (v0.9.0 → v0.10.0)
  - Phase 5: Polish & Documentation (v0.10.0 → v1.0.0)
- **Estimated completion**: 2-3 weeks coding (with hardware validation)

### Command System
Serial command processor with text-based commands:
- `arm` - Arms flight computer for launch detection
- `status_sensors` - Detailed sensor health report
- `calibrate` - Manual barometer calibration
- `clear_errors` - Manual error state recovery
- Debug flags `1-9` for different subsystem outputs

### Error Handling
- **Hierarchical detection**: Sensor validation → automatic recovery → manual override
- **Grace periods** prevent error state oscillation
- **Degraded operation** continues with partial sensor failures
- **Multiple recovery paths** via commands and automatic health checks

## Development Guidelines

### When Modifying Flight Logic
1. Update configuration in `src/config.h` if adding parameters
2. Consider impact on all flight states - many behaviors are state-dependent
3. Update `LogData` struct and CSV headers if adding logged data
4. Test state transitions thoroughly, especially error recovery paths
5. Write unit tests in `test/unit/` for new logic paths
6. Ensure tests pass on native platform: `pio test -e native_test`

### When Adding Sensors (Modern Approach)
1. Define new interface extending `IMUInterface` or create new HAL interface in `src/hal/hal_interfaces.h`
2. Create sensor adapter: `src/sensors/[sensor]_sensor.cpp` implementing the interface
3. Update `src/sensors/sensor_factory.h` to support new sensor selection
4. Add health monitoring to `isSensorSuiteHealthy()` function
5. Add unit tests for sensor behavior in `test/unit/test_[sensor]_sensor.cpp`
6. Update mock implementation in `test/mocks/mock_[sensor].h` for testing
7. If providing orientation data, integrate with Kalman filter via IMUInterface
8. Update command processor for sensor-specific diagnostics

### When Working on HAL Abstraction
1. Reference `src/hal/hal_interfaces.h` for available interfaces
2. Use HAL factory pattern: Don't call Teensy APIs directly, use HAL interfaces
3. Mock implementations in `test/mocks/` should match production HAL behavior
4. Verify both `pio run -e teensy41` (production) and `pio test -e native_test` (desktop) work

### When Modifying Web Interface
1. Test data parsing with `test_message_filtering.html`
2. Update `flight_console_data_mapping.json` for new data fields
3. Verify CSV data format matches what data parser expects
4. Consider impact on 3D visualization if changing orientation data

### Critical Safety Considerations
- Any changes to apogee detection logic require extensive testing
- Pyro channel control is safety-critical - verify all state transitions
- Error state recovery must not compromise flight safety
- Backup timer systems should remain as final failsafes

### Hardware Dependencies
- **Target platform**: Teensy 4.1 only (ARM Cortex-M7)
- **Required sensors**: ICM-20948, MS5611, GPS module
- **Optional sensors**: KX134 high-G accelerometer
- **I2C bus**: All sensors except GPS use single I2C bus
- **SD card**: Uses built-in Teensy 4.1 SDIO interface

## Git & Version Management

### Branch Strategy
- **main**: Stable releases only (tagged with version numbers)
- **develop**: Integration branch for feature work
- **feature/[name]**: Individual feature branches (create from develop, PR back to develop)
- **release/v[x.y.z]**: Release preparation branches
- **hotfix/[issue]**: Critical fixes to released versions

### Commit Convention
Use Conventional Commits format: `<type>(<scope>): <subject>`
- **feat**: New feature (e.g., `feat(hal): add ITimer interface`)
- **fix**: Bug fix (e.g., `fix(gps): validate data before use`)
- **test**: Test additions (e.g., `test(apogee): add high-G scenario`)
- **docs**: Documentation (e.g., `docs: update developer guide`)
- **refactor**: Code refactoring (e.g., `refactor(sensors): extract common code`)
- **chore**: Build, deps, etc. (e.g., `chore(release): bump to v0.8.0`)

### Version Management
- Follow semantic versioning: MAJOR.MINOR.PATCH-prerelease
- Current version stored in: `src/config.h` (#define FIRMWARE_VERSION "vX.Y.Z")
- Tag releases in git: `git tag -a vX.Y.Z -m "Release notes"`
- Alpha/beta versions: Use `-dev`, `-alpha`, `-beta` suffixes during development
- Production releases: No suffix (e.g., v1.0.0)

### Pull Request Workflow
1. Create feature branch from develop: `git checkout -b feature/my-feature develop`
2. Make commits with conventional messages
3. Push and create PR: `git push -u origin feature/my-feature`
4. Ensure all tests pass: `pio run -e teensy41` and `pio test -e native_test`
5. Get review approval
6. Merge with "Squash and merge" for clean history
7. Delete feature branch after merge

## Common Development Patterns

### Adding New Serial Commands
1. Add command string to `command_processor.cpp`
2. Implement handler function with `SystemStatusContext` parameter
3. Add help text to command list
4. Test both success and error cases

### Extending Data Logging
1. Modify `LogData` struct in `data_structures.h`
2. Update CSV header creation in `log_format_definition.cpp`
3. Update data collection in main firmware loop
4. Verify web interface data parsing handles new fields

### State-Dependent Features
Many systems activate only in specific states:
- **PID control**: Active only in COAST, DROGUE_DESCENT, MAIN_DESCENT
- **Apogee detection**: Active only in COAST state
- **Sensor switching**: Different thresholds per flight phase
- Check `flight_logic.cpp` for state-specific behaviors