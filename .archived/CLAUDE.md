# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) and other LLMs when working with code in this repository.

## Getting Started / Resuming Work

**For multi-session continuity**, check:
- `MEMORY.md` - Persistent notes across sessions (patterns, decisions, lessons learned)
- `IMPLEMENTATION_PLAN_2026.md` - Complete refactoring roadmap with checkpoints
- This file (CLAUDE.md) - Project conventions and architecture overview
- `AI.md` - Generic guidance for any LLM
- `wiki/index.md` - Project wiki with architecture docs, concept explanations, and module references. Consult for architecture questions. Keep wiki updated after structural code changes.

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

## Running Tests

### Desktop Unit Tests (No Hardware)

```bash
# Run all tests locally
pio test -e native_test

# Run specific test file
pio test -e native_test -f test_apogee_detection

# Run with verbose output
pio test -e native_test -v

# Expected output:
# test/unit/test_state_machine.cpp::<test_name> [PASSED]
# test/unit/test_apogee_detection.cpp::<test_name> [PASSED]
# ===== X passed in Y.YYs ======
```

### Understanding Test Results

- **[PASSED]**: Test completed successfully
- **[FAILED]**: Test assertion failed (fix code, re-run)
- **[SKIPPED]**: Test disabled with #if 0 (enable as needed)

### Adding New Tests

1. Create file: `test/unit/test_[feature].cpp`
2. Include Unity framework: `#include <unity.h>`
3. Write test cases with `TEST_ASSERT_*` macros
4. Add to CI/CD: Tests auto-run on every git push

See `DEVELOPER_GUIDE.md` Testing Strategy for examples.

### CI/CD (GitHub Actions)

Tests automatically run on every push:
- Repository: `.github/workflows/test.yml`
- Triggers: Every push, pull request
- Result: Green checkmark if all tests pass

## Architecture Decision Records (ADRs)

This section documents major architectural decisions and their rationale.

### ADR-001: Hardware Abstraction Layer (HAL)

**Date:** 2026-02-15
**Status:** ACCEPTED (implemented in v0.9.0)

**Decision:** Introduce HAL abstraction layer (ITimer, ISerial, IGPIO, etc.)

**Rationale:**
- Enable desktop testing without hardware
- Support platform migration (Teensy → STM32)
- Easy hardware swapping
- Clear separation of concerns

**Implementation:** `src/hal/hal_interfaces.h`

**Alternatives Considered:**
- Direct Arduino API calls (rejected: not testable)
- HAL generator tool (rejected: over-engineering)

**Consequences:**
- Extra abstraction layer (minor performance cost: negligible)
- All hardware interactions through HAL (required discipline)
- Easier to add mock implementations for testing

### ADR-002: Sensor Interface Pattern (IMUInterface)

**Date:** 2026-02-15
**Status:** ACCEPTED (implemented in v0.9.0)

**Decision:** All motion sensors implement common IMUInterface

**Rationale:**
- Enable sensor swaps (ICM-20948 ↔ BNO085) at compile-time
- Automatic failover with IMUManager
- Redundancy without code changes

**Implementation:** `src/sensors/imu_interface.h`

**Alternatives Considered:**
- Compile-time template specialization (rejected: too complex)
- Runtime polymorphism only (accepted: current approach)

**Consequences:**
- Virtual function overhead (minimal, ~2-3%)
- Easy to add new sensor types
- Clear interface contract

### ADR-003: Multi-Method Apogee Detection

**Date:** 2026-02-15
**Status:** ACCEPTED (implemented in v0.9.0)

**Decision:** Use 2-of-3 voting for apogee detection (barometer + accel + GPS + timer)

**Rationale:**
- Single sensor fails: system still works
- Noisy data filtered by majority vote
- Backup timer ensures deployment even if all fail
- Industry standard for safety-critical systems

**Implementation:** `src/flight_logic.cpp` (detectApogee function)

**Alternatives Considered:**
- Single-sensor detection (rejected: insufficient redundancy)
- All-or-nothing voting (rejected: too strict)
- 3-of-3 voting (rejected: too lenient if one fails)

**Consequences:**
- Slightly delayed apogee detection (wait for consensus)
- Very robust to sensor noise/failure
- Rare false positives or false negatives

### ADR-004: State Persistence in EEPROM

**Date:** 2026-02-15
**Status:** ACCEPTED (implemented in v0.9.0)

**Decision:** Save flight state to EEPROM after each state change

**Rationale:**
- Power-loss recovery: resume flight from saved state
- Watchdog reset recovery: don't lose flight phase
- Critical for long-duration flights

**Implementation:** `src/state_management.cpp`

**Alternatives Considered:**
- No persistence (rejected: lose data on power loss)
- Periodically save (rejected: might miss state change)
- Save after each sensor read (rejected: EEPROM wear)

**Consequences:**
- EEPROM wear (acceptable: ~1000 cycles per flight state)
- Small overhead per state change
- Very robust to power interruptions

### ADR-005: Kalman Filter for Orientation

**Date:** 2026-02-15
**Status:** ACCEPTED (replaces deprecated Madgwick)

**Decision:** Use Kalman filter instead of Madgwick complementary filter

**Rationale:**
- Better sensor fusion of gyro + accel
- Handles GPS altitude data
- More tunable (process/measurement noise)
- Better documentation for safety-critical systems

**Implementation:** `src/kalman_filter.cpp`

**Alternatives Considered:**
- Madgwick filter (rejected: less accurate)
- EKF (rejected: too complex)
- No fusion (rejected: noisy gyro)

**Consequences:**
- More accurate orientation estimates
- Additional computational cost (acceptable)
- Requires tuning process/measurement covariance

## Common Development Tasks

### Task: Adding a New Configuration Parameter

**Steps:**

1. Define in `src/config.h`:
   ```cpp
   #define NEW_PARAMETER 42
   ```

2. Use in code:
   ```cpp
   if (sensor_reading > NEW_PARAMETER) {
     // Take action
   }
   ```

3. Add to `status_sensors` command:
   ```cpp
   hal->serial()->println("NEW_PARAMETER: ");
   hal->serial()->println(NEW_PARAMETER);
   ```

4. Document in `docs/CONFIGURATION.md`

5. Test: Upload and verify behavior

### Task: Adding a New Data Field to Logging

**Steps:**

1. Extend `LogData` in `src/data_structures.h`:
   ```cpp
   struct LogData {
     // ... existing fields ...
     float new_field;
   };
   ```

2. Update CSV headers in `log_format_definition.cpp`:
   ```cpp
   const char* csv_headers[] = {
     // ... existing headers ...
     "new_field",
   };
   ```

3. Populate in main loop in `src/TripleT_Flight_Firmware.cpp`:
   ```cpp
   log_data.new_field = getNewValue();
   ```

4. Test: Run `log_test` command, verify CSV contains field

5. Update web interface parser if needed

### Task: Adding a Serial Command

**Steps:**

1. Add handler in `src/command_processor.cpp`:
   ```cpp
   void handleMyCommand(const SystemStatusContext& context) {
     context.serial->println("My command output");
   }
   ```

2. Register in command dispatcher:
   ```cpp
   if (strcmp(cmd, "mycommand") == 0) {
     handleMyCommand(context);
   }
   ```

3. Add help text:
   ```cpp
   void printHelpText() {
     // ... existing help ...
     context.serial->println("mycommand - Description");
   }
   ```

4. Test: Upload, type `mycommand` in serial monitor

### Task: Modifying Flight Logic State Transition

**Steps:**

1. Edit `src/flight_logic.cpp`

2. Find state transition:
   ```cpp
   case CURRENT_STATE:
     if (condition) {
       setFlightState(NEW_STATE);
     }
     break;
   ```

3. Modify condition or add preconditions

4. Write test in `test/unit/test_state_machine.cpp`:
   ```cpp
   void test_new_transition() {
     // Setup
     current_state = CURRENT_STATE;
     // Trigger
     // Assert: NEW_STATE
   }
   ```

5. Run tests: `pio test -e native_test`

6. Upload and verify flight behavior

### Task: Comparing to Baseline After Code Change

**Steps:**

```bash
# 1. Build current version
pio run -e teensy41
git add -A && git commit -m "test: my change"

# 2. Save build size/performance
pio run -e teensy41 -v | grep "RAM\|Flash" > current.txt

# 3. Revert to baseline
git checkout HEAD~1

# 4. Build baseline
pio run -e teensy41
pio run -e teensy41 -v | grep "RAM\|Flash" > baseline.txt

# 5. Compare
diff baseline.txt current.txt

# 6. Return to current work
git checkout -
```

## Quick Decision Matrix

**Use when deciding "What goes where?"**

| What | Where | Why |
|------|-------|-----|
| Configuration parameter | `src/config.h` | Compile-time selection |
| Flight logic | `src/flight_logic.cpp` | Core state machine |
| Sensor driver | `src/sensors/[name]_sensor.h` | Modular sensor code |
| HAL interface | `src/hal/hal_interfaces.h` | Hardware abstraction |
| Serial command | `src/command_processor.cpp` | Command handling |
| Data field | `src/data_structures.h` | Shared types |
| Unit test | `test/unit/test_[feature].cpp` | Test code |
| Mock object | `test/mocks/mock_[system].h` | Testing harness |
| Documentation | `docs/[TOPIC].md` | Developer reference |