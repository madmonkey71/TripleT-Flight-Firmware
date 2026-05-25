# TripleT Flight Firmware v1.0.0 Implementation Plan
**Created:** February 15, 2026
**Timeline:** 14 weeks (v0.6.0-dev → v1.0.0)
**Team:** User + Google Gemini (several hours/week available)
**Status:** Ready for execution

---

## Executive Summary

**⚡ Speed Note:** This plan is *much* more aggressive than typical refactoring timelines. With Claude Code handling the implementation and your oversight/testing, each phase completes in 1-2 days of focused work. The actual calendar time (2-3 weeks) is dominated by hardware testing, review cycles, and real-world validation—not the coding itself.

This document outlines a comprehensive refactoring roadmap to transform TripleT Flight Firmware into a production-ready v1.0.0 release with:
- **Hardware Abstraction Layer (HAL)** - Decouples firmware from Teensy-specific APIs
- **Sensor Modularity** - Support for dual accelerometers (ICM-20948 + BNO085) with redundancy
- **Desktop CI/CD** - 47+ unit tests with zero hardware dependencies
- **Semantic Versioning** - Clear version progression toward v1.0.0
- **Incremental Integration** - 4 checkpoints maintaining a working system throughout

**Key Constraint:** The system must remain flightworthy at all times. No breaking changes between checkpoints.

---

## Phase 1: HAL Foundation (Weeks 1-2) - v0.6.1-dev → v0.7.0

### Objectives
- Define 7 HAL interfaces (ITimer, ISerial, IGPIO, II2C, IEEPROM, ISDCard, IServo, IWatchdog)
- Create Teensy HAL implementation (production use)
- Create Mock HAL implementation (desktop testing)
- Zero behavior changes to flight logic

### Detailed Deliverables

#### 1.1 HAL Interface Definitions

Create new file: `src/hal/hal_interfaces.h`

```cpp
// ITimer - Replace millis(), micros(), delay()
class ITimer {
  virtual uint32_t millis() = 0;
  virtual uint32_t micros() = 0;
  virtual void delay(uint32_t ms) = 0;
  virtual void delayMicroseconds(uint16_t us) = 0;
};

// ISerial - Replace Serial.print, Serial.read
class ISerial {
  virtual void print(const char* str) = 0;
  virtual void println(const char* str) = 0;
  virtual void write(uint8_t byte) = 0;
  virtual int read() = 0;
  virtual bool available() = 0;
};

// IGPIO - Replace digitalWrite, digitalRead, pinMode
class IGPIO {
  virtual void pinMode(uint8_t pin, uint8_t mode) = 0;
  virtual void digitalWrite(uint8_t pin, uint8_t value) = 0;
  virtual int digitalRead(uint8_t pin) = 0;
};

// II2C - Replace Wire.begin, Wire.write, Wire.read
class II2C {
  virtual bool begin() = 0;
  virtual void beginTransmission(uint8_t addr) = 0;
  virtual size_t write(const uint8_t* data, size_t len) = 0;
  virtual uint8_t endTransmission() = 0;
  virtual size_t requestFrom(uint8_t addr, size_t len) = 0;
  virtual int read() = 0;
};

// IEEPROM - Replace EEPROM.read/write
class IEEPROM {
  virtual uint8_t read(uint16_t addr) = 0;
  virtual void write(uint16_t addr, uint8_t value) = 0;
  virtual bool begin() = 0;
};

// ISDCard - Replace SdFat operations
class ISDCard {
  virtual bool begin(uint8_t cs_pin) = 0;
  virtual bool writeData(const uint8_t* data, size_t len) = 0;
  virtual bool closeFile() = 0;
};

// IServo - Replace analogWrite for PWM
class IServo {
  virtual void analogWrite(uint8_t pin, uint8_t value) = 0;
  virtual void pinMode(uint8_t pin, uint8_t mode) = 0;
};

// IWatchdog - Replace WDT feeding
class IWatchdog {
  virtual void feed() = 0;
  virtual void reset() = 0;
};
```

#### 1.2 Teensy HAL Implementation

Create new file: `src/hal/teensy_hal.h` and `src/hal/teensy_hal.cpp`

- Real implementations calling actual Teensy APIs
- Zero overhead - inline where possible
- Direct mappings: `TeensyTimer::millis()` → `::millis()`

#### 1.3 Mock HAL Implementation

Create new file: `src/hal/mock_hal.h` and `src/hal/mock_hal.cpp`

- Simulated implementations for desktop testing
- Injectable state (e.g., `MockTimer.setTime(1000)` for testing time-dependent logic)
- No external dependencies (Serial, GPIO, I2C all no-ops)
- Tracks call counts for verification (e.g., `assert(mockSerial.printCount == 5)`)

#### 1.4 Dependency Injection System

Create new file: `src/hal/hal_factory.h`

```cpp
class HALFactory {
  static ITimer* createTimer();
  static ISerial* createSerial();
  static IGPIO* createGPIO();
  // ... etc for all interfaces
};

// Macro for compile-time selection
#ifdef TEST_BUILD
  #define GET_TIMER() HALFactory::createTimer() // Returns MockTimer
#else
  #define GET_TIMER() &TeensyTimer::instance()  // Returns TeensyTimer singleton
#endif
```

#### 1.5 Integration Points

Update `src/main.cpp`:
- Replace all `millis()` calls with `timer->millis()`
- Replace all `Serial.print()` calls with `serial->print()`
- Replace all `digitalWrite()` calls with `gpio->digitalWrite()`
- Wrap in HAL factory at startup

**Effort estimate:** 4-6 hours with Claude Code (1-2 days if working in focused sessions)
**Risk level:** Low (no behavior change, additive only)
**Files modified:** ~18 sensor/flight logic files (search and replace)
**New files created:** 5 (hal_interfaces.h, teensy_hal, mock_hal, hal_factory, hal_config.h)

#### 1.6 Checkpoint 1 Acceptance Criteria
- [ ] All 7 HAL interfaces compile without errors
- [ ] TeensyHAL provides working implementations for all interfaces
- [ ] MockHAL provides injectable/verifiable implementations
- [ ] Firmware compiles with no behavioral changes
- [ ] Existing flight test demonstrates same behavior as pre-refactor
- [ ] No increase in flash memory usage > 2%

---

## Phase 2: Sensor Modularity (Weeks 2-3) - v0.7.0 → v0.8.0

### Objectives
- Create IMUInterface for abstraction of ICM-20948 and BNO085
- Implement sensor factory for runtime selection
- Add sensor redundancy and switchover logic
- Design dual-sensor data fusion for reliability

### Detailed Deliverables

#### 2.1 IMUInterface Definition

Create new file: `src/sensors/imu_interface.h`

```cpp
class IMUInterface {
  // Core methods - must be implemented by all sensors
  virtual bool begin() = 0;
  virtual bool read() = 0;

  // Acceleration data (m/s²)
  virtual float getAccelX() = 0;
  virtual float getAccelY() = 0;
  virtual float getAccelZ() = 0;
  virtual float getAccelMagnitude() = 0;

  // Rotation data (degrees/sec)
  virtual float getGyroX() = 0;
  virtual float getGyroY() = 0;
  virtual float getGyroZ() = 0;

  // Magnetic field (if available)
  virtual float getMagX() = 0;
  virtual float getMagY() = 0;
  virtual float getMagZ() = 0;

  // Quaternion for orientation (primary method)
  virtual void getQuaternion(float& qw, float& qx, float& qy, float& qz) = 0;

  // Temperature
  virtual float getTemperature() = 0;

  // Sensor health
  virtual bool isHealthy() = 0;
  virtual const char* getErrorMessage() = 0;

  // Configuration
  virtual void setAccelScale(uint16_t g_range) = 0;
  virtual void setGyroScale(uint16_t dps_range) = 0;
};
```

#### 2.2 ICM-20948 Adapter

Create new file: `src/sensors/icm20948_sensor.h` and `.cpp`

Wraps existing `ICM_20948` from library:
- Implements `IMUInterface` virtual methods
- Maps ICM-20948 specific calls to interface
- Maintains existing calibration logic
- Handles high-G switchover to KX134

#### 2.3 BNO085 Adapter (Future Support)

Create new file: `src/sensors/bno085_sensor.h` and `.cpp`

- Implements same `IMUInterface`
- Mapping for BNO085 quaternion output
- Supports hardware sensor fusion (if available)
- Planned for hardware evaluation phase

#### 2.4 IMU Manager (Sensor Redundancy)

Create new file: `src/sensors/imu_manager.h` and `.cpp`

```cpp
class IMUManager {
  // Dual sensor setup
  IMUInterface* primary_sensor;    // ICM-20948 (normal mode)
  IMUInterface* backup_sensor;     // KX134 (high-G mode) or BNO085 (alternative)

  // Read from primary, fallback to backup if failed
  bool read() {
    if (!primary_sensor->read()) {
      return backup_sensor->read();
    }
    return true;
  }

  // Health monitoring
  bool isPrimaryHealthy() const;
  bool isBackupHealthy() const;

  // Get data (automatically from whichever sensor is healthy)
  float getAccelMagnitude();     // Returns from healthiest sensor
  void getQuaternion(float& qw, float& qx, float& qy, float& qz);
};
```

#### 2.5 Sensor Factory

Create new file: `src/sensors/sensor_factory.h`

```cpp
class SensorFactory {
  static IMUInterface* createPrimarySensor();  // ICM-20948
  static IMUInterface* createBackupSensor();   // KX134 or BNO085
  static IMUManager* createIMUManager();
};

// Compile-time configuration
#ifdef USE_BNO085_VARIANT
  #define PRIMARY_SENSOR BNO085Sensor
  #define BACKUP_SENSOR  ICM20948Sensor
#else
  #define PRIMARY_SENSOR ICM20948Sensor
  #define BACKUP_SENSOR  KX134Sensor  // High-G accelerometer
#endif
```

#### 2.6 Data Structure Updates

Modify `src/data_structures.h`:
- Add `imu_manager_ptr` to system context
- Add sensor health flags (primary_healthy, backup_healthy)
- Add sensor switching events to log (for debugging sensor failover)

#### 2.7 Integration with Kalman Filter

Modify `src/kalman_filter.cpp`:
- Change `ICM_20948_read()` to `imu_manager->read()`
- Kalman filter consumes data via `IMUInterface`, blind to physical sensor
- Automatic switchover transparent to filter

**Effort estimate:** 5-8 hours with Claude Code (1-2 days of focused work)
**Risk level:** Low (encapsulation prevents existing code changes)
**Files modified:** 3 (kalman_filter.cpp, flight_logic.cpp, main.cpp)
**New files created:** 6 (imu_interface.h, icm20948_sensor, bno085_sensor, imu_manager, sensor_factory, sensor_tests)

#### 2.8 Checkpoint 2 Acceptance Criteria
- [ ] IMUInterface fully defined with all 11 virtual methods
- [ ] ICM20948Sensor implements interface, passes all existing calibration tests
- [ ] IMUManager successfully reads from primary sensor
- [ ] Fallback to backup sensor works when primary fails
- [ ] Kalman filter unchanged (receives data via interface only)
- [ ] Firmware still flightworthy with existing hardware
- [ ] Memory overhead < 500 bytes
- [ ] Latency unchanged (< 1% difference in read timing)

---

## Phase 3: Testing Infrastructure (Weeks 3-4) - v0.8.0 → v0.9.0

### Objectives
- Establish PlatformIO native test harness
- Implement 47+ unit tests covering flight logic, sensors, and math
- Create mock sensor data (3-tier approach)
- Add GitHub Actions CI/CD pipeline

### Detailed Deliverables

#### 3.1 Test Framework Setup

Update `platformio.ini`:
```ini
[env:native_test]
platform = native
framework =
test_framework = unity
test_dir = test/unit
```

Verify Unity framework is in `platformio.ini` (already present in dependencies).

#### 3.2 Core Test Suites (47+ Tests)

Create `test/unit/`:

**Test Suite 1: State Machine Tests (8 tests)**
- `test_state_transitions.cpp` - State machine correctness
- `test_error_state_recovery.cpp` - Error handling paths
- `test_state_persistence.cpp` - EEPROM save/restore
- `test_graceful_degradation.cpp` - Partial sensor failure handling

**Test Suite 2: Apogee Detection Tests (12 tests)**
- `test_baro_apogee.cpp` - Barometric apogee detection with various pressure curves
- `test_accel_apogee.cpp` - Acceleration-based apogee with spike rejection
- `test_dual_apogee.cpp` - Concordance between baro and accel methods
- `test_apogee_with_noise.cpp` - Robustness to sensor noise
- `test_backup_timer_apogee.cpp` - Fallback timer behavior
- `test_apogee_hysteresis.cpp` - Prevent false apogee transitions

**Test Suite 3: GPS Calibration Tests (10 tests)**
- `test_gps_validation.cpp` - Invalid data rejection (fixType > 5, SIV > 100)
- `test_altitude_offset_calculation.cpp` - Hypsometric formula with known inputs
- `test_gps_timeout.cpp` - Timeout after N attempts
- `test_user_abort.cpp` - User can abort calibration
- `test_degraded_gps_fix.cpp` - 2D vs 3D vs RTK handling
- `test_baro_calibration_persistence.cpp` - Offset survives state changes

**Test Suite 4: Landing Detection Tests (8 tests)**
- `test_landing_from_main_descent.cpp` - Low velocity + low acceleration
- `test_false_landing_rejection.cpp` - Prevent premature landing declaration
- `test_landing_with_windy_conditions.cpp` - Robustness to noise
- `test_recovery_logging.cpp` - Post-landing data capture

**Test Suite 5: Math & Physics Tests (9 tests)**
- `test_hypsometric_formula.cpp` - Altitude calculations at various pressures
- `test_quaternion_math.cpp` - Orientation transformations
- `test_velocity_integration.cpp` - Numerical integration accuracy
- `test_unit_conversions.cpp` - hPa ↔ Pa, mm ↔ m, etc.
- `test_kalman_filter_stability.cpp` - Filter convergence
- `test_pd_control_laws.cpp` - Control response correctness

#### 3.3 Mock Sensor Implementations (3-tier Strategy)

**Tier 1: Synthetic Data Generation**

Create `test/mocks/mock_sensors.h`:
```cpp
class MockIMUSensor : public IMUInterface {
  // Synthetic perfect data
  bool setAcceleration(float ax, float ay, float az);

  // Simulated realistic noise
  void addGaussianNoise(float sigma);

  // Pre-recorded flight data playback
  bool loadFlightData(const char* filename);
  bool playNextFrame();
};
```

**Tier 2: Recorded Flight Data Replay**

Create `test/fixtures/`:
- `real_flight_2025_01_05.log` - Recorded altitude, acceleration, GPS
- `high_g_event_2025.log` - High-G aerodynamic event
- `failed_gps_calibration.log` - No GPS fix available
- `nominal_flight_east_field.log` - Reference good flight

Load with:
```cpp
MockIMUSensor sensor;
sensor.loadFlightData("test/fixtures/nominal_flight_east_field.log");

while (sensor.playNextFrame()) {
  flight_logic_update();
  // Verify apogee detected at correct altitude
}
```

**Tier 3: Failure Injection**

Create `test/mocks/failure_injector.h`:
```cpp
class FailureInjector {
  // Sensor failures
  void injectSensorFailure(SensorType type, uint32_t at_time_ms);
  void injectDataCorruption(uint8_t percentage);

  // Environmental conditions
  void setAtmosphericPressure(float hPa);
  void setTemperature(float celsius);
  void setWindSpeed(float mps);

  // Event triggers
  void triggerLowBatteryWarning();
  void triggerWatchdogReset();
};
```

#### 3.4 Test Execution Patterns

Example test structure:
```cpp
void test_apogee_detection_nominal_flight(void) {
  // Setup
  MockIMUSensor imu;
  imu.loadFlightData("test/fixtures/nominal_flight_east_field.log");
  FlightState state = COAST;
  float apogee_altitude = 0;

  // Execute
  while (imu.playNextFrame()) {
    state = flight_logic_update();
    if (state == APOGEE) {
      apogee_altitude = get_current_altitude();
      break;
    }
  }

  // Verify
  TEST_ASSERT_EQUAL_FLOAT(1255.5, apogee_altitude);  // ±5m tolerance
  TEST_ASSERT_EQUAL(APOGEE, state);
}
```

#### 3.5 GitHub Actions CI/CD Pipeline

Create `.github/workflows/test.yml`:
```yaml
name: Unit Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
        with:
          python-version: '3.11'
      - run: pip install platformio
      - run: pio test -e native_test
      - uses: actions/upload-artifact@v3
        with:
          name: test-results
          path: .pio/build/native_test/test_results.xml
```

Builds firmware on every push:
```yaml
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
      - run: pip install platformio
      - run: pio run -e teensy41
```

**Effort estimate:** 8-12 hours with Claude Code (2-3 days for core tests, additional days for test fixtures)
**Risk level:** Low (testing infrastructure only)
**New files created:** 30+ test files + mocks
**Coverage target:** 70%+ statement coverage, 100% for flight-critical paths

#### 3.6 Checkpoint 3 Acceptance Criteria
- [ ] All 47+ unit tests pass on native platform
- [ ] Mock sensors accurately reproduce real flight data
- [ ] GitHub Actions workflow runs on every push
- [ ] Test coverage report generated and > 70%
- [ ] Flight-critical code (apogee, landing, state transitions) > 95% coverage
- [ ] No performance degradation of production firmware
- [ ] Documentation of test data fixtures and how to add more

---

## Phase 4: Safety & Reliability Features (Weeks 4-5) - v0.9.0 → v0.10.0

### Objectives
- Add redundant apogee detection paths
- Implement GPS altitude verification for baro calibration
- Add watchdog recovery mechanisms
- Strengthen error state handling

### Key Enhancements

#### 4.1 Multi-Path Apogee Detection

Currently: Barometric pressure + accelerometer
Enhanced: Add GPS-based apogee as tertiary detector

```cpp
bool apogee_detected() {
  bool baro_apogee = detect_baro_apogee();       // Primary
  bool accel_apogee = detect_accel_apogee();     // Secondary
  bool gps_descent = detect_gps_descent();       // Tertiary (if GPS available)
  bool timeout = time_since_boost > APOGEE_TIMEOUT; // Failsafe

  // 2-of-3 voting (or 1-of-2 if GPS unavailable)
  int votes = baro_apogee + accel_apogee + gps_descent;
  return votes >= 2 || timeout;
}
```

#### 4.2 Sensor Cross-Validation

```cpp
class SensorValidator {
  // Baro altitude should match GPS altitude within 50m (after calibration)
  bool validateBaroAgainstGPS();

  // Acceleration magnitude should correlate with velocity changes
  bool validateAccelAgainstVelocity();

  // GPS altitude rate should match computed rate from accel
  bool validateGPSDescentRate();
};
```

#### 4.3 Watchdog Recovery

Enhanced watchdog handling:
```cpp
class WatchdogRecovery {
  // On WDT reset, check last state in EEPROM
  FlightState restoreFromEEPROM();

  // Re-initialize only sensors that failed
  void reinitializeSensors();

  // Log recovery event with timestamp
  void logRecoveryEvent();
};
```

**Effort estimate:** 3-4 hours with Claude Code (1 focused day)
**Files modified:** flight_logic.cpp, apogee_detection.cpp, error_handling.cpp

#### 4.4 Checkpoint 4 Acceptance Criteria
- [ ] Multi-path apogee detection passes all test scenarios
- [ ] GPS cross-validation rejects uncalibrated baro readings
- [ ] Watchdog recovery restores previous state on reset
- [ ] All recovery paths tested with failure injection
- [ ] No increase in code size > 5%
- [ ] Recovery time < 2 seconds

---

## Phase 5: Production Polish & Documentation (Weeks 5-7) - v0.10.0 → v1.0.0

### Objectives
- Comprehensive documentation for developers and users
- Final integration testing on real hardware
- Performance optimization and code cleanup
- Semantic versioning and release management

### Detailed Deliverables

#### 5.1 Developer Documentation

Create `docs/DEVELOPER_GUIDE.md`:
- HAL architecture overview with class diagrams
- Adding new sensors (step-by-step with example)
- Testing strategy and how to write new tests
- Code organization and module responsibilities
- Git workflow and PR process

Create `docs/ARCHITECTURE.md`:
- Data flow diagrams (sensor → filter → logic → control)
- State machine visualization
- Class hierarchy for sensors, HAL, and managers
- Thread safety considerations (if any)

#### 5.2 User Documentation

Create `docs/USER_GUIDE.md`:
- Serial commands reference (arm, status_sensors, calibrate, etc.)
- Web interface usage
- LED indicators and their meanings
- Troubleshooting common issues
- Firmware update procedure

Create `docs/SAFETY.md`:
- Hardware fault detection and recovery
- Single point of failure analysis
- Test procedures for flight-critical systems
- Pre-flight checklist

#### 5.3 Hardware Evaluation Documentation

Create `docs/SENSOR_EVALUATION.md`:
- Benchmarks: ICM-20948 vs BNO085
  - Response time comparison
  - Accuracy across temperature range
  - Power consumption
  - Cost analysis
- Migration path if swapping sensors
- Compatibility matrix for different hardware revisions

#### 5.4 Code Cleanup

- Remove dead code (commented functions already removed in previous sessions)
- Consistent naming conventions across all modules
- Replace magic numbers with named constants (already done in ms5611_functions.cpp pattern)
- Final linting and style enforcement

#### 5.5 Performance Optimization

Profile and optimize:
- Kalman filter computation (identify expensive operations)
- Sensor read timing (ensure < 20ms per read at 50Hz)
- Memory usage (verify no excessive allocations in ISRs)
- Serial communication (batch print operations)

#### 5.6 Release Management

Create `RELEASE_NOTES.md` format:
```markdown
# v1.0.0 - Production Release

## New Features
- Hardware Abstraction Layer for desktop testing
- Sensor modularity (ICM-20948/BNO085 support)
- 47+ unit tests with 70%+ coverage
- Multi-path apogee detection with redundancy
- Watchdog recovery mechanisms

## Breaking Changes
- None (fully backward compatible with v0.6.0 hardware)

## Bug Fixes
- GPS data corruption handling
- Watchdog timeout during initialization
- Baro calibration with invalid pressure values

## Known Issues
- BNO085 evaluation phase (hardware to be determined)
- Performance optimization ongoing

## Testing
- Flight tested: 5+ nominal flights
- Hardware failures: Tested sensor dropout scenarios
- Safety features: Verified apogee detection accuracy

## Upgrade Instructions
1. Read SAFETY.md pre-flight checklist
2. Verify serial commands: `status_sensors`
3. Calibrate barometer: `calibrate`
4. Arm and test on pad: `arm`
```

**Effort estimate:** 4-6 hours with Claude Code (1-2 days for documentation generation)
**Files created:** 7 documentation files

#### 5.7 Final Integration Testing

Testing plan before v1.0.0 release:
1. **Unit tests:** Automated on every commit (GitHub Actions)
2. **Hardware tests:** 3 real flights with different payloads
3. **Sensor failure tests:** Simulate each sensor failing independently
4. **Edge cases:**
   - Extreme altitude (sea level vs 15,000+ ft)
   - Temperature extremes
   - High wind/gusty conditions
   - Low battery scenarios
5. **Recovery tests:**
   - GPS initialization timeout
   - Watchdog reset mid-flight (simulated)
   - Sensor corruption detection

#### 5.8 Checkpoint 5 Acceptance Criteria
- [ ] All documentation complete and technical accurate
- [ ] 5+ successful real flights with v1.0.0 firmware
- [ ] All edge case scenarios tested
- [ ] Zero critical bugs found in final integration testing
- [ ] Performance meets requirements (apogee detection < 50ms)
- [ ] Code coverage maintained > 70%
- [ ] Release notes and upgrade path documented
- [ ] Tag v1.0.0 in git with signed release

---

## Git & Versioning Strategy

### Semantic Versioning Progression

```
v0.6.0-dev (starting point - current beta)
  ↓ (Phase 1: 1-2 days)
v0.7.0 (HAL interfaces + implementations)
  ↓ (Phase 2: 1-2 days)
v0.8.0 (IMUInterface + sensor factory)
  ↓ (Phase 3: 2-3 days)
v0.9.0 (47+ unit tests + CI/CD)
  ↓ (Phase 4: 1 day)
v0.10.0 (multi-path apogee + redundancy)
  ↓ (Phase 5: 1-2 days)
v1.0.0-rc1 (Release Candidate - after hardware testing)
  ↓ (Hardware validation: 5+ flights)
v1.0.0 (Production Release)
```

**Accelerated Schedule:** Each phase now completes in 1-2 days of focused coding work, rather than weeks. Calendar time is dominated by hardware testing and review cycles.

### Git Flow Branches

```
main (master)
├── beta (current development, v0.6-beta series)
├── develop (integration branch for Phase 1-5 work)
├── feature/hal-foundation (Phase 1 work)
├── feature/sensor-modularity (Phase 2 work)
├── feature/testing-infrastructure (Phase 3 work)
├── feature/safety-features (Phase 4 work)
├── feature/polish-docs (Phase 5 work)
├── release/v0.7.0 (preparing v0.7.0)
├── release/v0.8.0 (preparing v0.8.0)
└── hotfix/gps-corruption-fix (urgent fixes to current release)
```

### Branch Protection Rules (main)

- Require 1 code review before merge
- All checks must pass (GitHub Actions: unit tests + build)
- Safety-critical features require test evidence
- No direct commits to main (PR-only workflow)

### Commit Message Convention (Conventional Commits)

```
<type>(<scope>): <subject>

<body>

<footer>
```

Examples:
```
feat(hal): add ITimer interface for abstraction

Define ITimer pure virtual class for all timer operations
(millis, micros, delay). Allows desktop testing without
hardware dependencies.

Relates-to: PHASE1
```

```
fix(gps): validate fixType and SIV before using data

Reject GPS fix type > 5 or satellite count > 100 as corrupted.
Prevents health checks from displaying invalid data.

Fixes: #42
```

```
test(apogee): add unit tests for nominal flight scenario

Test apogee detection on nominal ascent/descent profile
with realistic sensor noise. Verifies 1255m ±5m accuracy.

Test-Evidence: test_apogee_detection_nominal_flight passes
```

### Pull Request Workflow

For each feature branch → develop → main cycle:

1. **Create feature branch** from `develop`:
   ```bash
   git checkout develop
   git pull origin develop
   git checkout -b feature/hal-foundation
   ```

2. **Make commits** with conventional messages:
   ```bash
   git commit -m "feat(hal): add ITimer interface"
   ```

3. **Push and create PR** against `develop`:
   ```bash
   git push -u origin feature/hal-foundation
   # Create PR on GitHub
   ```

4. **PR Description** includes:
   - What was changed and why
   - Test evidence (screenshots, unit test passes)
   - Files modified
   - Any breaking changes (should be none)

5. **Code Review** - Address feedback, push additional commits

6. **Merge** after approval and all checks pass:
   - Use "Squash and merge" for clean history
   - PR title becomes commit message

7. **Prepare Release** (v0.7.0, v0.8.0, etc.):
   ```bash
   git checkout develop
   git pull origin develop
   git checkout -b release/v0.7.0
   # Update version in config.h
   git commit -m "chore(release): bump to v0.7.0"
   git push -u origin release/v0.7.0
   # Create PR to main, get approval
   # After merge to main:
   git checkout main
   git pull origin main
   git tag -a v0.7.0 -m "v0.7.0 - HAL Foundation Complete"
   git push origin v0.7.0
   ```

---

## Parallel Work Opportunities

The plan supports parallel development as team grows:

### Week 2-3 (Overlap Possible)
- **Track A:** Finish HAL Foundation (Phase 1) + begin Sensor Modularity (Phase 2)
- **Track B:** Begin Testing Infrastructure setup (Phase 3) - mocks ready for Phase 2 use

### Week 3-4 (With Extra Resources)
- **Track A:** Phase 2 completion + Phase 3 test suite writing
- **Track B:** Phase 4 safety features design + documentation planning

### Realistic Execution Timeline
```
Day 1     Day 2-3   Day 4     Day 5-6   Day 7     Day 8-10   Day 11-14
|---------|---------|---------|---------|---------|---------|---------|
Phase 1   Phase 2   Phase 3a  Phase 3b  Phase 4   Phase 5    Testing +
HAL       Sensors   Core      Fixtures  Safety    Docs       Hardware
                    Tests               +Redundancy          Validation
                    ├─────────────────────────────────────┐
                    All phases complete, now test on hardware
                    Flight testing, edge cases, real-world validation
                    ├──────────────────────────────────────→ v1.0.0 ready
```

With full context and focused work sessions, each phase typically completes in 1-2 days of Claude time, with the majority of remaining time spent on hardware testing and validation.

---

## Recovery & Resumption Instructions

### If Breaking During Phase (e.g., mid-Week 3)

1. **Identify Current Checkpoint:**
   ```bash
   git branch -a | grep feature/
   git log --oneline -10  # See recent commits
   ```

2. **Find Acceptance Criteria:**
   - This document → Phase N → Checkpoint N Acceptance Criteria
   - Example: Stopped mid-Phase 3 → check Phase 3 Checkpoint 3 criteria

3. **Resume Work:**
   ```bash
   git checkout feature/[current-phase]
   git pull origin feature/[current-phase]
   # Review uncommitted changes
   git status
   # Continue from last working state
   ```

4. **Reference Key Files:**
   - HAL layer: `src/hal/hal_*.h`
   - Sensor interfaces: `src/sensors/imu_interface.h`
   - Tests: `test/unit/*.cpp`
   - Configuration: `src/config.h` (version number here)
   - Main entry: `src/main.cpp` (HAL injection here)

5. **Verify Build State:**
   ```bash
   pio run -e teensy41        # Should still compile
   pio test -e native_test    # Tests at current phase
   ```

### If Starting After Multi-Day Break

1. **Sync with latest:**
   ```bash
   git fetch origin
   git status
   ```

2. **Review Phase Documentation:**
   - Re-read the Phase section you're in
   - Check Acceptance Criteria vs current code
   - Identify what's done vs what remains

3. **Review Recent Commits:**
   ```bash
   git log --oneline develop -20  # See what merged while you were away
   git diff develop feature/[your-branch]  # See your unique changes
   ```

4. **Rebuild Understanding:**
   - Look at modified files: `git show [commit]:src/[file].cpp`
   - Check test results: `pio test -e native_test`
   - Run integration: `pio run -e teensy41`

5. **Document Gaps:**
   - Update MEMORY.md with patterns learned
   - Add notes on any tricky architectural decisions
   - Record lessons from debugging

---

## Resource Estimates

### Timeline Summary
- **Total Effort:** 24-36 hours with Claude Code (actual coding work)
- **Realistic Wall Clock:** 2-3 weeks depending on review cycles and hardware testing
- **Critical Path:** Phase 1 → Phase 2 → Phase 3 (can parallelize 4 & 5)

### Phase Breakdown
| Phase | Claude Hours | Duration | Key Risk |
|-------|---|---|---|
| 1: HAL Foundation | 4-6 | 1-2 days | Integration testing on hardware |
| 2: Sensor Modularity | 5-8 | 1-2 days | Sensor data format compatibility |
| 3: Testing | 8-12 | 2-3 days | Fixture data realism |
| 4: Safety | 3-4 | 1 day | Edge case discovery in flight |
| 5: Polish | 4-6 | 1-2 days | Documentation quality review |
| **Total** | **24-36 hours** | **2-3 weeks** | Hardware validation |

### Critical Path Dependencies
```
Phase 1 (HAL) ──→ Phase 2 (Sensors) ──┐
                                       ├─→ Phase 3 (Testing) ──→ Phase 4 (Safety) ──→ Phase 5 (Release)
                  Phase 2 (Sensors) ───┘
```

Phases 1-2 must complete before testing can be effective.
Phases 4-5 depend on Phases 1-3 being stable.

---

## Safety Considerations

### Flight-Critical Code Review Gates

These paths **must** have tests before merging to develop:

1. **Apogee Detection** (`flight_logic.cpp:apogee_detected()`)
   - Unit tests: 12+ test cases
   - Integration tests: 5+ recorded flight replays
   - Hardware tests: 3+ actual flights

2. **Pyro Channel Control** (`guidance_control.cpp:fire_drogue/main_parachute()`)
   - Unit tests: 5+ state transition tests
   - Hardware verification: Safe test on pad (no flight)
   - Code review: 2 reviewers for any changes

3. **Error State Recovery** (`error_handling.cpp`)
   - Unit tests: 8+ failure scenario tests
   - Watchdog recovery: Verified on hardware
   - Graceful degradation: Partial sensor failure handling

### Change Review Criteria for Safety-Critical Code

Any PR modifying these files requires:
- [ ] New unit tests demonstrating the fix
- [ ] Explanation of how it affects flight safety
- [ ] At least 2 code reviewers from team
- [ ] Test flight evidence (if feasible for change)
- [ ] Documented rollback procedure

---

## Success Metrics

### By Phase Completion

| Metric | Phase 1 | Phase 2 | Phase 3 | Phase 4 | Phase 5 |
|--------|---------|---------|---------|---------|---------|
| Code Coverage | - | - | >70% | >80% | >90% |
| Compilation Time | <5s | <5s | <5s | <5s | <5s |
| Flash Memory | +0% | +1-2% | +3-4% | +4-5% | +4-5% |
| Unit Tests Passing | - | - | 47/47 ✓ | 50+/50+ ✓ | 50+/50+ ✓ |
| Real Flight Tests | - | 1-2 flights | 2-3 flights | 3-4 flights | 5+ flights |
| Documentation | 30% | 50% | 60% | 80% | 100% |

### v1.0.0 Release Criteria (ALL must be true)

- ✓ All 5 phases complete
- ✓ 50+ unit tests with 90%+ coverage of flight-critical code
- ✓ 5+ successful flights on v1.0.0 firmware
- ✓ Zero unresolved critical bugs
- ✓ All GitHub Actions tests passing
- ✓ Complete developer documentation
- ✓ Complete user documentation
- ✓ Semantic versioning established (v0.6.0-dev → v1.0.0)
- ✓ Git workflow and branching strategy in place
- ✓ Code review and safety gates implemented
- ✓ Hardware evaluation documented (ICM vs BNO085 analysis)

---

## Key Files & Locations

### Configuration & Setup
- **Version number:** `src/config.h` - `#define FIRMWARE_VERSION "v0.6.0-dev"`
- **HAL configuration:** `src/hal/hal_config.h` (create in Phase 1)
- **Feature flags:** `src/debug_flags.h` (existing, for test selection)
- **Build config:** `platformio.ini` (add native test environment)

### Core Modules
- **Flight logic:** `src/flight_logic.cpp`
- **State machine:** `src/state_management.cpp`
- **Apogee detection:** `src/apogee_detection.cpp` (or in flight_logic.cpp)
- **Error handling:** `src/error_handling.cpp`

### Sensor Modules
- **HAL interfaces:** `src/hal/hal_interfaces.h` (create Phase 1)
- **HAL implementation:** `src/hal/teensy_hal.cpp` + `mock_hal.cpp` (Phase 1)
- **Sensor factory:** `src/sensors/sensor_factory.h` (Phase 2)
- **IMU interface:** `src/sensors/imu_interface.h` (Phase 2)
- **Existing sensors:** `src/icm_20948_functions.cpp`, `ms5611_functions.cpp`, `gps_functions.cpp`

### Testing
- **Unit tests:** `test/unit/*.cpp` (create Phase 3)
- **Mock sensors:** `test/mocks/*.h` and `.cpp` (Phase 3)
- **Test fixtures:** `test/fixtures/*.log` (recorded flight data, Phase 3)
- **Test harness:** `platformio.ini` with native environment (Phase 3)

### Documentation
- **This file:** `IMPLEMENTATION_PLAN_2026.md`
- **Developer guide:** `docs/DEVELOPER_GUIDE.md` (create Phase 5)
- **Architecture:** `docs/ARCHITECTURE.md` (Phase 5)
- **Safety:** `docs/SAFETY.md` (Phase 5)
- **User guide:** `docs/USER_GUIDE.md` (Phase 5)
- **Release notes:** `RELEASE_NOTES.md` (Phase 5)

---

## Getting Started (Next Steps)

### Immediate (Before Phase 1 Coding)

1. **Understand Current State:**
   ```bash
   git log --oneline -20            # Recent commits
   git status                        # Current changes
   pio run -e teensy41              # Verify it builds
   ```

2. **Prepare Development Environment:**
   ```bash
   pip install platformio           # Already done
   pio lib list                      # Verify dependencies
   ```

3. **Create Feature Branch for Phase 1:**
   ```bash
   git checkout develop
   git pull origin develop
   git checkout -b feature/hal-foundation
   ```

4. **Review CLAUDE.md** for project conventions

### Phase 1 (Day 1)

Single focused work session:
1. Create `src/hal/hal_interfaces.h` with all 7 interfaces
2. Create `src/hal/teensy_hal.h` with inline implementations
3. Create `src/hal/mock_hal.h` with injectable values
4. Create `src/hal/hal_factory.h` with factory pattern
5. Verify compilation: `pio run -e teensy41`

If desired, second session can add dependency injection to main files, but core HAL is complete after day 1.

### Monitoring Progress

- Track completion of Acceptance Criteria for each Checkpoint
- Run tests regularly: `pio test -e native_test`
- Keep MEMORY.md updated with learned patterns
- Document blockers or architectural questions as they arise

---

## Contact & Support

If you need help during implementation:

1. **Architectural questions:** Review DEVELOPER_GUIDE.md (to be created in Phase 5)
2. **Code review blockers:** Share PR with explanation of the challenge
3. **Test failures:** Check test fixtures, verify mock data setup
4. **Hardware issues:** Refer to sensor datasheets and HARDWARE_NOTES.md (if created)

---

**Document Status:** Ready for Phase 1 execution
**Last Updated:** February 15, 2026
**Next Review:** After Phase 1 completion (Week 2)
