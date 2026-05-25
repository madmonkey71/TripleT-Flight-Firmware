# Phase 6.4: Validation Suite & Testing Framework

**Document Status:** Comprehensive Implementation Plan
**Created:** February 16, 2026
**Scope:** 2,000+ lines of test code across 6 test categories
**Goal:** 95%+ test coverage for v1.0.0 release

---

## Table of Contents

1. [Test File Architecture](#1-test-file-architecture)
2. [Full Flight Simulator (6.4.1)](#2-full-flight-simulator-641)
3. [Regression Test Suite (6.4.2)](#3-regression-test-suite-642)
4. [Hardware-in-Loop Testing (6.4.3)](#4-hardware-in-loop-testing-643)
5. [System Verification Matrix (6.4.4)](#5-system-verification-matrix-644)
6. [Test Data Fixtures](#6-test-data-fixtures)
7. [CI/CD Integration](#7-cicd-integration)
8. [Effort Estimate & Timeline](#8-effort-estimate--timeline)

---

## 1. Test File Architecture

### 1.1 Directory Structure

```
test/
├── README.md                              # Test framework documentation
├── platformio.ini                         # PlatformIO test configuration
├── pytest.ini                             # Pytest configuration (for Python-based tests)
│
├── unit/                                  # Unit tests (no flight logic, fast)
│   ├── test_state_machine.cpp             # 8 tests: state transitions
│   ├── test_apogee_detection.cpp          # 6 tests: 2-of-3 voting, edge cases
│   ├── test_servo_control.cpp             # 4 tests: range, responsiveness, smoothing
│   ├── test_sensor_health.cpp             # 8 tests: sensor monitoring, failover
│   ├── test_data_logging.cpp              # 6 tests: CSV format, field validation
│   ├── test_guidance_control.cpp          # 5 tests: PID, stability, failsafe
│   ├── test_kalman_filter.cpp             # 4 tests: quaternion fusion, accuracy
│   ├── test_math_functions.cpp            # 3 tests: haversine, conversions
│   ├── test_trajectory.cpp                # 3 tests: waypoint calc, XTE
│   └── test_power_management.cpp          # 2 tests: battery estimation
│   Total: 50+ individual unit tests
│
├── integration/                           # Integration tests (subsystem interaction)
│   ├── test_full_flight_simulation.cpp    # Main entry point for recorded data playback
│   ├── test_nominal_flight.cpp            # Baseline flight scenario
│   ├── test_high_g_flight.cpp             # Extreme acceleration (sensor switching)
│   ├── test_gps_loss_scenario.cpp         # GPS dropout recovery
│   ├── test_sensor_failover.cpp           # Dual sensor redundancy
│   ├── test_wind_conditions.cpp           # Guidance stability in wind
│   ├── test_extended_flight.cpp           # Long duration (power optimization)
│   └── test_edge_cases.cpp                # EEPROM corruption, thermal, etc.
│   Total: 8 integration test suites
│
├── regression/                            # Regression tests (prevent regressions)
│   ├── test_legacy_compatibility.cpp      # Features from v0.10.0
│   ├── test_state_transitions.cpp         # All 15 state paths
│   ├── test_safety_systems.cpp            # Safety-critical paths
│   ├── test_data_format.cpp               # CSV, SD card format consistency
│   ├── test_command_processor.cpp         # All serial commands
│   └── test_configuration_system.cpp      # Config loading, validation
│   Total: 6 regression test files (can run in parallel)
│
├── hardware_in_loop/                      # Hardware-in-loop tests (real equipment)
│   ├── hil_test_setup.h                   # Common test harness setup
│   ├── hil_launch_detection.cpp           # Test launch detection logic
│   ├── hil_apogee_triggers.cpp            # Verify deployment triggers
│   ├── hil_servo_response.cpp             # Measure servo timing/accuracy
│   ├── hil_sensor_failover.cpp            # Test sensor switching under load
│   ├── hil_pyro_timing.cpp                # Pyro channel verification
│   └── hil_gps_integration.cpp            # Real GPS validation
│   Total: 7 HIL test procedures
│
├── fixtures/                              # Test data and recorded flights
│   ├── recordings/                        # CSV files from real or synthetic flights
│   │   ├── nominal_flight_1250m.csv       # Baseline: 1250m apogee
│   │   ├── high_g_boost_8g.csv            # High acceleration event
│   │   ├── gps_loss_coast_phase.csv       # GPS dropout during coast
│   │   ├── thermal_drift_extended.csv     # 30+ minute flight
│   │   ├── windy_conditions_25mph.csv     # High wind scenario
│   │   ├── sensor_noise_realistic.csv     # Gaussian noise overlay
│   │   └── deployment_timing_test.csv     # Validates exact timing
│   │
│   ├── generated/                         # Programmatically generated data
│   │   ├── generate_synthetic_data.py     # Python script to generate CSV
│   │   ├── generate_edge_cases.py         # Extreme scenarios
│   │   └── README.md                      # Instructions for data generation
│   │
│   ├── expected_outputs/                  # Known-good test results
│   │   ├── nominal_flight_output.txt      # Reference output for comparison
│   │   ├── apogee_voting_results.txt      # Expected 2-of-3 voting outcomes
│   │   └── servo_command_log.txt          # Expected servo sequences
│   │
│   └── sensor_characterization/           # Sensor specs, noise models
│       ├── icm20948_specs.json            # ±16G accelerometer specs
│       ├── kx134_specs.json               # ±64G accelerometer specs
│       ├── ms5611_specs.json              # Barometer noise model
│       └── gps_accuracy_model.json        # GPS error distribution
│
├── mocks/                                 # Mock objects for testing
│   ├── mock_sensors.h                     # Base sensor mock
│   ├── mock_imu.h                         # IMUInterface mock
│   ├── mock_servo.h                       # Servo actuation mock
│   ├── mock_gps.h                         # GPS receiver mock
│   ├── mock_barometer.h                   # Pressure sensor mock
│   ├── mock_hal.h                         # Hardware abstraction layer mocks
│   └── mock_config.h                      # Configuration mocks
│
├── scripts/                               # Test automation scripts
│   ├── run_all_tests.sh                   # Main test runner
│   ├── run_unit_tests.sh                  # Fast local tests only
│   ├── run_regression_tests.sh            # Comprehensive validation
│   ├── run_hil_tests.sh                   # Hardware-in-loop (requires equipment)
│   ├── generate_coverage_report.sh        # Code coverage analysis
│   ├── compare_baseline.sh                # Compare against known-good binary
│   └── ci_runner.py                       # GitHub Actions integration
│
└── results/                               # Test output storage (gitignored)
    ├── unit_test_results.xml              # Unity test output
    ├── coverage_report.html               # Code coverage report
    ├── hil_test_log.txt                   # Hardware test log
    └── regression_results.txt             # Regression test summary
```

### 1.2 Test Naming Convention

**Pattern:** `test_<feature>_<scenario>[_<variation>].cpp`

Examples:
- `test_apogee_detection_barometric_only.cpp` - Tests barometer-only apogee
- `test_state_machine_error_recovery.cpp` - Tests error state recovery
- `test_servo_control_stability_high_wind.cpp` - Servo in windy conditions
- `test_data_logging_sd_card_write.cpp` - Data persistence
- `test_guidance_failsafe_saturation.cpp` - Failsafe on servo saturation

### 1.3 File Organization Principles

1. **One logical test per file** - Each `.cpp` contains tests for one feature
2. **Clear dependencies** - Integration tests depend on unit tests
3. **Fast to slow progression** - Unit → Integration → Regression → HIL
4. **No test interdependencies** - All tests can run in any order
5. **Self-contained mocks** - Each test brings its own mock data

---

## 2. Full Flight Simulator (6.4.1)

### 2.1 Simulator Architecture

**Concept:** Feed recorded sensor data through entire flight stack, validate all outputs.

```cpp
// test/integration/test_full_flight_simulation.cpp

#include <unity.h>
#include "flight_simulator.h"
#include "fixtures/flight_data.h"

class FullFlightSimulator {
public:
  struct FlightValidationResult {
    bool state_transitions_valid;
    bool apogee_detection_correct;
    bool deployment_timing_valid;
    bool servo_commands_reasonable;
    bool data_logging_complete;
    bool gps_integration_working;

    std::vector<std::string> warnings;
    std::vector<std::string> errors;

    // Detailed metrics
    float apogee_altitude_error_m;
    uint32_t apogee_detection_time_ms;
    uint32_t drogue_deploy_delay_ms;
    uint32_t main_deploy_delay_ms;
  };

private:
  struct SimulationState {
    uint32_t current_time_ms;
    uint8_t flight_state;
    float altitude_m;
    float velocity_mps;
    float acceleration_mps2;

    // Sensor data at current timestep
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float pressure_pa;
    float temperature_c;
    float gps_lat, gps_lon, gps_alt;

    // Outputs being monitored
    uint8_t pyro_drogue_state;
    uint8_t pyro_main_state;
    int16_t servo_pitch_cmd;
    int16_t servo_yaw_cmd;
    int16_t servo_roll_cmd;
  };

public:
  // Load flight data and run simulation
  FlightValidationResult runSimulation(
    const FlightDataPoint* recorded_data,
    size_t data_point_count,
    const SimulationConfig& config
  );

private:
  // Process one timestep
  void simulationStep(const FlightDataPoint& sensor_data);

  // Validate output against expected behavior
  void validateStep(const FlightDataPoint& expected, const SimulationState& actual);
};
```

### 2.2 CSV Input Format for Recorded Flights

**File:** `test/fixtures/recordings/nominal_flight_1250m.csv`

```csv
timestamp_ms,accel_x_mps2,accel_y_mps2,accel_z_mps2,gyro_x_dps,gyro_y_dps,gyro_z_dps,pressure_pa,temperature_c,gps_lat,gps_lon,gps_alt_m,gps_fix_quality
0,0.0,0.0,9.81,0.0,0.0,0.0,101325.0,20.0,34.876500,-118.123400,500.0,3
10,5.2,0.1,12.3,2.1,1.8,-0.5,101324.8,20.1,34.876501,-118.123401,500.1,3
20,12.8,0.2,15.6,4.2,3.1,-1.2,101324.2,20.2,34.876502,-118.123402,500.2,3
...
15000,0.5,-0.1,-8.2,0.2,-0.1,0.1,99542.0,19.8,34.876535,-118.123450,1250.5,3
15010,-2.1,0.3,-7.8,0.3,0.2,0.0,99535.2,19.7,34.876536,-118.123451,1249.8,3
...
25000,-5.1,0.2,-8.1,0.1,-0.1,0.0,95214.0,18.5,34.876580,-118.123500,500.2,2
25010,-4.9,0.1,-8.0,0.0,0.0,0.0,95210.5,18.4,34.876581,-118.123501,500.0,3
```

**Format Notes:**
- **timestamp_ms**: Milliseconds since boot
- **accel_x/y/z_mps2**: Acceleration in m/s² (includes gravity)
- **gyro_x/y/z_dps**: Angular velocity in degrees/second
- **pressure_pa**: Atmospheric pressure in Pascals
- **temperature_c**: Sensor temperature in Celsius
- **gps_lat/lon**: Latitude/Longitude in degrees
- **gps_alt_m**: GPS-derived altitude (MSL)
- **gps_fix_quality**: 0=no fix, 1=GPS, 2=DGPS, 3=RTK, 4=RTK-Fixed

### 2.3 State Transition Validation Algorithm

```cpp
bool validateStateTransitions(const FlightDataPoint* data, size_t count) {
  struct StateTransitionRule {
    uint8_t current_state;
    uint8_t next_state;
    bool (*validator)(const FlightDataPoint& prev, const FlightDataPoint& current);
  };

  static const StateTransitionRule rules[] = {
    // STARTUP → CALIBRATION (boot)
    {STARTUP, CALIBRATION,
     [](const auto& p, const auto& c) { return true; }},

    // CALIBRATION → PAD_IDLE (wait for sensor validation)
    {CALIBRATION, PAD_IDLE,
     [](const auto& p, const auto& c) { return c.timestamp_ms > 1000; }},

    // PAD_IDLE → ARMED (manual command)
    {PAD_IDLE, ARMED,
     [](const auto& p, const auto& c) { return true; }},

    // ARMED → BOOST (acceleration > threshold)
    {ARMED, BOOST,
     [](const auto& p, const auto& c) {
       return c.accel_z > BOOST_ACCEL_THRESHOLD;
     }},

    // BOOST → COAST (acceleration < threshold)
    {BOOST, COAST,
     [](const auto& p, const auto& c) {
       return c.accel_z < COAST_ACCEL_THRESHOLD;
     }},

    // COAST → APOGEE (detected by 2-of-3 voting)
    {COAST, APOGEE,
     [](const auto& p, const auto& c) {
       return detectApogeeByVoting(p, c) == true;
     }},

    // ... more transitions
  };

  bool all_valid = true;
  uint8_t expected_state = STARTUP;

  for (size_t i = 1; i < count; i++) {
    uint8_t actual_state = data[i].flight_state;

    if (actual_state == expected_state) {
      continue;  // Stay in same state
    }

    // Check if transition is valid
    bool found_rule = false;
    for (const auto& rule : rules) {
      if (rule.current_state == expected_state &&
          rule.next_state == actual_state) {
        if (rule.validator(data[i-1], data[i])) {
          found_rule = true;
          expected_state = actual_state;
          break;
        }
      }
    }

    if (!found_rule) {
      all_valid = false;
      log_error("Invalid transition: %d → %d at t=%d ms",
                expected_state, actual_state, data[i].timestamp_ms);
    }
  }

  return all_valid;
}
```

### 2.4 Apogee Detection 2-of-3 Voting Verification

```cpp
struct ApogeeVotingResult {
  bool barometric_detected;
  uint32_t barometric_time_ms;
  float barometric_altitude_m;

  bool acceleration_detected;
  uint32_t acceleration_time_ms;
  float acceleration_altitude_m;

  bool gps_detected;
  uint32_t gps_time_ms;
  float gps_altitude_m;

  bool timer_fallback_triggered;

  // Final result (2-of-3 voting)
  bool apogee_detected;
  uint32_t final_apogee_time_ms;
  float final_apogee_altitude_m;
};

ApogeeVotingResult validateApogeeDetection(
  const FlightDataPoint* data,
  size_t count,
  float expected_apogee_m
) {
  ApogeeVotingResult result = {false, 0, 0, false, 0, 0, false, 0, 0, false, false, 0, 0};

  // 1. Check barometric method
  result.barometric_detected = detectBarometricApogee(
    data, count,
    result.barometric_time_ms,
    result.barometric_altitude_m
  );

  // 2. Check acceleration method
  result.acceleration_detected = detectAccelerationApogee(
    data, count,
    result.acceleration_time_ms,
    result.acceleration_altitude_m
  );

  // 3. Check GPS method
  result.gps_detected = detectGPSApogee(
    data, count,
    result.gps_time_ms,
    result.gps_altitude_m
  );

  // 4. Check backup timer
  result.timer_fallback_triggered = checkTimerFallback(data, count);

  // 5. Apply 2-of-3 voting
  int vote_count = 0;
  if (result.barometric_detected) vote_count++;
  if (result.acceleration_detected) vote_count++;
  if (result.gps_detected) vote_count++;

  if (vote_count >= 2) {
    result.apogee_detected = true;
    // Average time and altitude of detected methods
    result.final_apogee_time_ms = averageTime({
      result.barometric_time_ms,
      result.acceleration_time_ms,
      result.gps_time_ms
    });
    result.final_apogee_altitude_m = averageAltitude({
      result.barometric_altitude_m,
      result.acceleration_altitude_m,
      result.gps_altitude_m
    });
  } else if (result.timer_fallback_triggered) {
    result.apogee_detected = true;
    // Use timer fallback value
    result.final_apogee_time_ms = data[count-1].timestamp_ms;
    result.final_apogee_altitude_m = data[count-1].altitude_m;
  }

  return result;
}

void test_apogee_voting_2of3() {
  auto result = validateApogeeDetection(nominal_flight_data, NOMINAL_FLIGHT_COUNT, 1250.0f);

  TEST_ASSERT_TRUE(result.apogee_detected);
  TEST_ASSERT_GREATER_THAN_UINT32(0, result.final_apogee_time_ms);
  TEST_ASSERT_FLOAT_WITHIN(10.0f, 1250.0f, result.final_apogee_altitude_m);
}
```

### 2.5 Deployment Timing Verification

```cpp
struct DeploymentTiming {
  uint32_t apogee_time_ms;
  uint32_t drogue_deploy_time_ms;
  uint32_t drogue_deploy_delay_ms;

  uint32_t main_deploy_time_ms;
  uint32_t main_deploy_delay_ms;

  bool drogue_timing_valid;
  bool main_timing_valid;

  std::string error_msg;
};

DeploymentTiming validateDeploymentTiming(
  const FlightDataPoint* data,
  size_t count,
  float main_deploy_altitude_m
) {
  DeploymentTiming result = {};

  // Find apogee event
  result.apogee_time_ms = findApogeeTime(data, count);

  // Find pyro events (state changes to DROGUE_DEPLOY)
  result.drogue_deploy_time_ms = findStateTransition(
    data, count, APOGEE, DROGUE_DEPLOY
  );
  result.drogue_deploy_delay_ms =
    result.drogue_deploy_time_ms - result.apogee_time_ms;

  // Verify drogue timing is reasonable (50-500ms delay typical)
  result.drogue_timing_valid =
    (result.drogue_deploy_delay_ms > 50 &&
     result.drogue_deploy_delay_ms < 500);

  // Find main deploy at specified altitude
  result.main_deploy_time_ms = findMainDeployTime(
    data, count, main_deploy_altitude_m
  );
  result.main_deploy_delay_ms =
    result.main_deploy_time_ms - result.drogue_deploy_time_ms;

  // Verify main timing (should occur after descent to altitude)
  float altitude_at_main = data[result.main_deploy_time_ms / 10].altitude_m;
  result.main_timing_valid =
    (fabs(altitude_at_main - main_deploy_altitude_m) < 10.0f);

  if (!result.drogue_timing_valid) {
    result.error_msg += "Drogue timing invalid; ";
  }
  if (!result.main_timing_valid) {
    result.error_msg += "Main timing invalid; ";
  }

  return result;
}

void test_deployment_timing_nominal() {
  auto timing = validateDeploymentTiming(
    nominal_flight_data, NOMINAL_FLIGHT_COUNT, 100.0f
  );

  TEST_ASSERT_TRUE(timing.drogue_timing_valid);
  TEST_ASSERT_TRUE(timing.main_timing_valid);
  TEST_ASSERT_UINT32_WITHIN(100, 250, timing.drogue_deploy_delay_ms);
}
```

### 2.6 Expected Output Format

**File:** `test/results/full_flight_simulation_results.json`

```json
{
  "test_name": "nominal_flight_1250m",
  "test_date": "2026-02-16T14:32:00Z",
  "test_duration_s": 45.2,
  "firmware_version": "v1.0.0-rc1",

  "flight_data": {
    "input_file": "nominal_flight_1250m.csv",
    "data_points": 2500,
    "duration_ms": 25000,
    "max_altitude_m": 1252.3
  },

  "validation_results": {
    "state_transitions": {
      "passed": true,
      "total_transitions": 14,
      "valid_transitions": 14,
      "invalid_transitions": 0
    },

    "apogee_detection": {
      "passed": true,
      "barometric_detected": true,
      "barometric_altitude_m": 1250.2,
      "acceleration_detected": true,
      "acceleration_altitude_m": 1251.8,
      "gps_detected": true,
      "gps_altitude_m": 1249.5,
      "voting_result": "2-of-3 passed",
      "final_apogee_m": 1250.5,
      "final_apogee_time_ms": 15005,
      "expected_apogee_m": 1250.0,
      "altitude_error_m": 0.5
    },

    "deployment_system": {
      "passed": true,
      "drogue_deployed": true,
      "drogue_deploy_time_ms": 15080,
      "drogue_deploy_delay_ms": 75,
      "drogue_timing_valid": true,

      "main_deployed": true,
      "main_deploy_time_ms": 24950,
      "main_deploy_altitude_m": 100.2,
      "main_timing_valid": true
    },

    "servo_commands": {
      "passed": true,
      "servo_pitch_range": [-45, 45],
      "servo_yaw_range": [-45, 45],
      "servo_roll_range": [-45, 45],
      "no_saturation": true,
      "response_time_ms": 250
    },

    "data_logging": {
      "passed": true,
      "total_log_points": 2500,
      "all_fields_present": true,
      "csv_format_valid": true,
      "file_size_bytes": 425000
    }
  },

  "summary": {
    "overall_passed": true,
    "tests_passed": 4,
    "tests_failed": 0,
    "warnings": []
  }
}
```

---

## 3. Regression Test Suite (6.4.2)

### 3.1 50+ Test Mapping to Features

#### 3.1.1 State Machine Tests (8 tests)

**File:** `test/regression/test_state_machine_transitions.cpp`

```cpp
#include <unity.h>
#include "flight_logic.h"
#include "state_management.h"

void setUp(void) {
  reset_flight_state_to_startup();
  clear_all_telemetry();
}

void tearDown(void) {}

// TEST 1: STARTUP → CALIBRATION
void test_startup_to_calibration_transition(void) {
  set_flight_state(STARTUP);

  // Simulate 100ms of initialization
  for (int i = 0; i < 10; i++) {
    update_flight_logic(10);  // 10ms per step
  }

  TEST_ASSERT_EQUAL(CALIBRATION, get_flight_state());
  TEST_ASSERT_TRUE(sensors_initialized());
}

// TEST 2: CALIBRATION → PAD_IDLE
void test_calibration_to_pad_idle(void) {
  set_flight_state(CALIBRATION);

  // Sensors need 500ms to validate
  for (int i = 0; i < 50; i++) {
    simulate_sensor_read(0.0f, 0.0f, 9.81f);  // 1G down
    update_flight_logic(10);
  }

  TEST_ASSERT_EQUAL(PAD_IDLE, get_flight_state());
}

// TEST 3: PAD_IDLE → ARMED (command)
void test_pad_idle_to_armed_command(void) {
  set_flight_state(PAD_IDLE);

  process_serial_command("arm");
  update_flight_logic(10);

  TEST_ASSERT_EQUAL(ARMED, get_flight_state());
}

// TEST 4: ARMED → BOOST (threshold)
void test_armed_to_boost_acceleration(void) {
  set_flight_state(ARMED);

  // Apply 5G acceleration (motor ignition)
  simulate_sensor_read(0.0f, 0.0f, 50.0f);
  update_flight_logic(10);

  TEST_ASSERT_EQUAL(BOOST, get_flight_state());
}

// TEST 5: BOOST → COAST (burnout)
void test_boost_to_coast_burnout(void) {
  set_flight_state(BOOST);

  // Start with high acceleration
  for (int i = 0; i < 50; i++) {
    simulate_sensor_read(0.0f, 0.0f, 40.0f);
    update_flight_logic(10);
  }

  // Drop acceleration below coast threshold
  for (int i = 0; i < 10; i++) {
    simulate_sensor_read(0.0f, 0.0f, 0.2f);
    update_flight_logic(10);
  }

  TEST_ASSERT_EQUAL(COAST, get_flight_state());
}

// TEST 6: COAST → APOGEE (2-of-3 voting)
void test_coast_to_apogee_voting(void) {
  set_flight_state(COAST);

  // Simulate ascent
  for (int i = 0; i < 100; i++) {
    simulate_sensor_read(0.0f, 0.0f, 5.0f);
    update_flight_logic(10);
  }

  // Simulate apogee (all three methods detect)
  simulate_barometer_apogee();
  simulate_accelerometer_apogee();
  simulate_gps_apogee();

  update_flight_logic(10);
  TEST_ASSERT_EQUAL(APOGEE, get_flight_state());
}

// TEST 7: APOGEE → DROGUE_DEPLOY
void test_apogee_to_drogue_deploy(void) {
  set_flight_state(APOGEE);

  update_flight_logic(100);

  TEST_ASSERT_EQUAL(DROGUE_DEPLOY, get_flight_state());
  TEST_ASSERT_TRUE(pyro_channel_fired(DROGUE_CHANNEL));
}

// TEST 8: DROGUE_DESCENT → MAIN_DEPLOY
void test_drogue_descent_to_main_deploy(void) {
  set_flight_state(DROGUE_DESCENT);

  // Simulate descent to 100m altitude
  for (int i = 0; i < 200; i++) {
    float altitude = 1000.0f - (i * 5.0f);
    simulate_sensor_read(0.0f, 0.0f, -8.0f);
    simulate_barometer_altitude(altitude);
    update_flight_logic(10);

    if (altitude <= 100.0f) {
      break;
    }
  }

  TEST_ASSERT_EQUAL(MAIN_DEPLOY, get_flight_state());
  TEST_ASSERT_TRUE(pyro_channel_fired(MAIN_CHANNEL));
}
```

#### 3.1.2 Apogee Detection Tests (6 tests)

**File:** `test/regression/test_apogee_detection_all_methods.cpp`

```cpp
// TEST 1: Barometric detection only
void test_apogee_barometric_method_only(void) {
  MockBarometer baro;
  MockIMUSensor imu;
  MockGPS gps;

  // All methods except barometer fail
  imu.setSimulationMode(SIMULATION_CONSTANT_ACCEL);
  gps.setAvailable(false);

  // Barometer detects apogee
  baro.simulateAscentDescent(1250.0f);

  bool result = detect_apogee_barometer_only(&baro);
  TEST_ASSERT_TRUE(result);
}

// TEST 2: Accelerometer detection only
void test_apogee_acceleration_method_only(void) {
  MockBarometer baro;
  MockIMUSensor imu;

  // Barometer noisy, accel detects first
  baro.setNoiseLevel(5.0f);
  imu.simulateApogeeTransition();

  bool result = detect_apogee_acceleration_only(&imu);
  TEST_ASSERT_TRUE(result);
}

// TEST 3: GPS detection only
void test_apogee_gps_method_only(void) {
  MockGPS gps;
  MockBarometer baro;
  MockIMUSensor imu;

  baro.setAvailable(false);
  imu.setSensorHealth(false);

  gps.simulateApogeeSequence(1250.0f);

  bool result = detect_apogee_gps_only(&gps);
  TEST_ASSERT_TRUE(result);
}

// TEST 4: 2-of-3 voting (all pass)
void test_apogee_voting_all_methods_agree(void) {
  MockBarometer baro;
  MockIMUSensor imu;
  MockGPS gps;

  baro.simulateAscentDescent(1250.0f);
  imu.simulateApogeeTransition();
  gps.simulateApogeeSequence(1250.0f);

  ApogeeResult result = detect_apogee_with_voting(&baro, &imu, &gps);

  TEST_ASSERT_TRUE(result.detected);
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 1250.0f, result.altitude_m);
}

// TEST 5: 2-of-3 voting (one method fails)
void test_apogee_voting_two_of_three_agree(void) {
  MockBarometer baro;
  MockIMUSensor imu;
  MockGPS gps;

  // Barometer and accel agree, GPS fails
  baro.simulateAscentDescent(1250.0f);
  imu.simulateApogeeTransition();
  gps.setFix(false);

  ApogeeResult result = detect_apogee_with_voting(&baro, &imu, &gps);

  TEST_ASSERT_TRUE(result.detected);
  TEST_ASSERT_EQUAL(2, result.voting_count);
}

// TEST 6: Backup timer fallback
void test_apogee_timer_fallback_when_all_fail(void) {
  MockBarometer baro;
  MockIMUSensor imu;
  MockGPS gps;

  // All methods fail, timer should trigger
  baro.setAvailable(false);
  imu.setSensorHealth(false);
  gps.setFix(false);

  // Simulate 20s after coast phase starts
  for (int i = 0; i < 2000; i++) {
    detect_apogee_with_voting(&baro, &imu, &gps);
    delay(10);
  }

  TEST_ASSERT_TRUE(apogee_fallback_timer_triggered());
}
```

#### 3.1.3 Servo Control Tests (4 tests)

**File:** `test/regression/test_servo_control_stability.cpp`

```cpp
// TEST 1: Servo range of motion
void test_servo_range_full_deflection(void) {
  MockServo servo;

  // Command servo to full range
  servo.setAngle(0);
  TEST_ASSERT_EQUAL(0, servo.getAngle());

  servo.setAngle(45);
  TEST_ASSERT_EQUAL(45, servo.getAngle());

  servo.setAngle(-45);
  TEST_ASSERT_EQUAL(-45, servo.getAngle());

  servo.setAngle(90);  // Should saturate
  TEST_ASSERT_EQUAL(45, servo.getAngle());  // Max is ±45°
}

// TEST 2: PID stability
void test_pid_controller_stability_convergence(void) {
  PIDController pid;
  pid.setPIDGains(0.5f, 0.1f, 0.2f);

  float setpoint = 45.0f;
  float measurement = 0.0f;
  float output = 0.0f;

  int stable_count = 0;
  for (int i = 0; i < 100; i++) {
    float error = setpoint - measurement;
    output = pid.update(error);

    // Simulated response
    measurement += output * 0.1f;

    // Check for stability (output settling)
    if (fabs(output - prev_output) < 0.1f) {
      stable_count++;
    }
  }

  // Should stabilize within 100 iterations
  TEST_ASSERT_GREATER_THAN_INT(80, stable_count);
}

// TEST 3: Servo response time
void test_servo_response_time_latency(void) {
  MockServo servo;

  uint32_t t_command = millis();
  servo.setAngle(45);

  uint32_t t_response = millis();
  while (servo.getAngle() < 44.0f) {
    t_response = millis();
    if (t_response - t_command > 500) {
      break;
    }
  }

  uint32_t response_time_ms = t_response - t_command;
  TEST_ASSERT_LESS_THAN_UINT32(300, response_time_ms);  // Should respond in <300ms
}

// TEST 4: Servo saturation detection
void test_servo_saturation_detection_and_fallback(void) {
  MockServo servo;
  StabilityMonitor monitor;

  // Continuously command servo beyond limits
  for (int i = 0; i < 100; i++) {
    servo.setAngle(90);  // Beyond ±45° max
    update_flight_logic(10);
  }

  TEST_ASSERT_TRUE(monitor.isActuatorSaturated());

  // System should reduce PID gains or go passive
  TEST_ASSERT_TRUE(guidance_failsafe_triggered());
}
```

#### 3.1.4 Sensor Health Tests (8 tests)

**File:** `test/regression/test_sensor_health_monitoring.cpp`

```cpp
// TEST 1: ICM-20948 initialization
void test_icm20948_initialization_success(void) {
  MockICM20948 icm;
  bool result = icm.begin();

  TEST_ASSERT_TRUE(result);
  TEST_ASSERT_TRUE(icm.isHealthy());
}

// TEST 2: KX134 backup activation on high-G
void test_kx134_activation_high_g_detection(void) {
  MockIMUManager imu;
  MockKX134 kx134;

  // Primary sensor saturates
  MockICM20948 primary;
  primary.simulateAcceleration(200.0f);  // ±16G max

  // Backup should activate
  update_flight_logic(10);

  TEST_ASSERT_TRUE(imu.isBackupSensorActive());
  TEST_ASSERT_TRUE(kx134.isReadingValid());
}

// TEST 3: Barometer calibration
void test_ms5611_calibration_offset(void) {
  MockMS5611 baro;

  // Set ground truth altitude
  float ground_altitude = 500.0f;  // 500m above MSL
  baro.calibrate(ground_altitude);

  // Read altitude (should be near ground level)
  float measured = baro.getAltitude();

  TEST_ASSERT_FLOAT_WITHIN(5.0f, ground_altitude, measured);
}

// TEST 4: GPS fix acquisition
void test_gps_fix_acquisition_latency(void) {
  MockGPS gps;

  uint32_t t_start = millis();
  gps.waitForFix();
  uint32_t t_fix = millis();

  uint32_t acquisition_time = t_fix - t_start;

  TEST_ASSERT_TRUE(gps.hasFix());
  TEST_ASSERT_LESS_THAN_UINT32(5000, acquisition_time);  // Should get fix in <5s
}

// TEST 5: Temperature drift compensation
void test_temperature_drift_compensation(void) {
  MockIMUSensor imu;

  // Simulate temperature change
  imu.setTemperature(20.0f);
  float accel_20c = imu.getAccelZ();

  imu.setTemperature(60.0f);
  float accel_60c = imu.getAccelZ();

  // With compensation, should be within ~1%
  float drift_percent = fabs(accel_20c - accel_60c) / accel_20c * 100.0f;

  TEST_ASSERT_LESS_THAN_FLOAT(1.0f, drift_percent);
}

// TEST 6: Sensor redundancy failover
void test_sensor_failover_primary_to_backup(void) {
  MockIMUManager imu;

  // Primary healthy, use it
  TEST_ASSERT_TRUE(imu.isPrimarySensor());

  // Primary fails
  imu.setPrimarySensorHealth(false);
  update_flight_logic(10);

  // Should switch to backup
  TEST_ASSERT_TRUE(imu.isBackupSensor());
}

// TEST 7: Sensor health status reporting
void test_sensor_health_command_output(void) {
  process_serial_command("status_sensors");
  std::string output = capture_serial_output();

  TEST_ASSERT_TRUE(output.find("ICM-20948: OK") != std::string::npos);
  TEST_ASSERT_TRUE(output.find("MS5611: OK") != std::string::npos);
  TEST_ASSERT_TRUE(output.find("GPS: OK") != std::string::npos);
}

// TEST 8: Error state recovery
void test_sensor_error_state_recovery(void) {
  set_flight_state(ERROR);
  set_error_code(ERROR_SENSOR_INIT);

  process_serial_command("clear_errors");
  update_flight_logic(100);

  TEST_ASSERT_EQUAL(PAD_IDLE, get_flight_state());
  TEST_ASSERT_EQUAL(NO_ERROR, get_error_code());
}
```

#### 3.1.5 Data Logging Tests (6 tests)

**File:** `test/regression/test_data_logging_format.cpp`

```cpp
// TEST 1: SD card write operations
void test_sd_card_write_and_verify(void) {
  MockSDCard card;

  const char* test_data = "timestamp_ms,accel_x,accel_y,accel_z\n";
  bool result = card.write((const uint8_t*)test_data, strlen(test_data));

  TEST_ASSERT_TRUE(result);
  TEST_ASSERT_GREATER_THAN_UINT32(0, card.getFileSize());
}

// TEST 2: CSV header correctness
void test_csv_header_field_order(void) {
  MockDataLogger logger;
  std::string header = logger.getCSVHeader();

  // Check field order
  size_t accel_x_pos = header.find("accel_x");
  size_t accel_y_pos = header.find("accel_y");
  size_t accel_z_pos = header.find("accel_z");

  TEST_ASSERT_TRUE(accel_x_pos != std::string::npos);
  TEST_ASSERT_TRUE(accel_y_pos > accel_x_pos);
  TEST_ASSERT_TRUE(accel_z_pos > accel_y_pos);
}

// TEST 3: All data fields present
void test_all_log_fields_populated(void) {
  MockDataLogger logger;

  // Simulate one flight step
  MockFlightState state;
  state.timestamp_ms = 1000;
  state.accel_x = 1.5f;
  state.accel_y = -0.2f;
  state.accel_z = 9.9f;
  // ... populate all fields

  logger.logData(state);
  std::string line = logger.getLastLogLine();

  // Count commas (should match field count)
  int field_count = std::count(line.begin(), line.end(), ',') + 1;

  TEST_ASSERT_EQUAL(EXPECTED_FIELD_COUNT, field_count);
}

// TEST 4: Numeric precision
void test_numeric_precision_float_formatting(void) {
  MockDataLogger logger;
  float test_value = 9.81234567f;

  logger.logField("test", test_value);
  std::string output = logger.getLastLogLine();

  // Should preserve 4-5 decimal places
  TEST_ASSERT_TRUE(output.find("9.8123") != std::string::npos);
}

// TEST 5: CSV parsing by external tool
void test_csv_parsing_compatibility(void) {
  MockDataLogger logger;

  // Generate 100 log entries
  for (int i = 0; i < 100; i++) {
    MockFlightState state;
    state.timestamp_ms = i * 10;
    state.accel_x = sin(i * 0.1f) * 10.0f;
    // ... fill other fields
    logger.logData(state);
  }

  // Write CSV file
  logger.writeToFile("test_output.csv");

  // Parse it back
  std::vector<LogEntry> parsed = parseCSVFile("test_output.csv");

  TEST_ASSERT_EQUAL(100, parsed.size());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, test_value, parsed[10].accel_x);
}

// TEST 6: Large file handling
void test_data_logging_large_file_capacity(void) {
  MockDataLogger logger;

  // Log maximum flight duration (1 hour at 100Hz)
  int max_entries = 360000;  // 100 Hz * 3600 s

  uint32_t start_time = millis();
  for (int i = 0; i < min(max_entries, 10000); i++) {
    MockFlightState state;
    state.timestamp_ms = i * 10;
    logger.logData(state);
  }
  uint32_t elapsed = millis() - start_time;

  // Should handle without running out of storage
  TEST_ASSERT_LESS_THAN_UINT32(5000, elapsed);  // 10k entries in <5s
}
```

#### 3.1.6 Additional Test Categories

- **Kalman Filter Tests** (4 tests): Quaternion fusion, gyro integration, accel update
- **Math Function Tests** (3 tests): Haversine distance, coordinate conversion
- **Trajectory Tests** (3 tests): Waypoint calculation, cross-track error
- **PID Control Tests** (3 tests): Stability, integral windup, derivative filtering
- **Power Management Tests** (2 tests): Battery estimation, mode switching
- **Command Processor Tests** (4 tests): Command parsing, response format
- **Configuration Tests** (3 tests): EEPROM persistence, validation
- **Safety System Tests** (5 tests): Watchdog, timeout, failsafe

**Total: 50+ individual regression tests across 10 categories**

---

## 4. Hardware-in-Loop Testing (6.4.3)

### 4.1 Test Harness Setup

**Required Equipment:**
```
Test Bench:
├── Teensy 4.1 Flight Computer (DUT - Device Under Test)
├── Function Generator (simulate sensor inputs)
├── Oscilloscope (measure servo/pyro response timing)
├── Potentiometers (simulate servo feedback)
├── LED Indicators (visual verification)
├── Serial-USB adapter
├── GPS Simulator (if available - optional)
├── Real GPS Receiver (integration test)
└── Test Parachute + Deployment Mechanism (final validation)

Connections:
- Serial to Function Generator (simulate I2C/SPI sensor data)
- Servo outputs to Oscilloscope & Potentiometers
- Pyro channels to LED + continuity meter
- GPS to real receiver (validate lock)
```

### 4.2 Test Procedures

#### 4.2.1 Launch Detection Test

**File:** `test/hardware_in_loop/hil_launch_detection.cpp`

```cpp
/**
 * TEST: Launch Detection via Acceleration Threshold
 * Purpose: Verify ARMED → BOOST transition on motor ignition
 *
 * Procedure:
 * 1. Put system in ARMED state
 * 2. Use function generator to simulate acceleration waveform
 * 3. Observe state transition time and accuracy
 */

void hil_test_launch_detection(void) {
  // Step 1: Arm the system
  Serial.println("arm");
  delay(100);

  uint8_t initial_state = get_flight_state();
  TEST_ASSERT_EQUAL(ARMED, initial_state);

  // Step 2: Simulate motor acceleration curve
  // Real motor: 0G → 50G over ~50ms (typical high-power)
  uint32_t t_launch = millis();
  for (int i = 0; i < 5; i++) {
    // Simulate boost phase via I2C writes to mock sensor
    float accel_g = (float)i * 10.0f;  // Ramp: 0, 10, 20, 30, 40G

    // Function generator output simulates this acceleration
    // Teensy reads it via sensor interface

    update_flight_logic(10);
    delay(10);
  }

  // Step 3: Verify state transition
  uint8_t new_state = get_flight_state();
  uint32_t t_transition = millis();

  TEST_ASSERT_EQUAL(BOOST, new_state);

  uint32_t transition_time_ms = t_transition - t_launch;
  TEST_ASSERT_LESS_THAN_UINT32(100, transition_time_ms);  // Should detect in <100ms

  // Log results
  printf("Launch detected: t=%d ms, state=%d\n", transition_time_ms, new_state);
}
```

**Verification Criteria:**
- Transition occurs within 50-100ms of acceleration threshold
- No false positives from vibration or noise
- Consistent detection across multiple trials (>95% success rate)

#### 4.2.2 Apogee Trigger Test

**File:** `test/hardware_in_loop/hil_apogee_triggers.cpp`

```cpp
/**
 * TEST: Apogee Detection & Drogue Deployment
 * Purpose: Verify all three apogee detection methods trigger deployment
 *
 * Procedure:
 * 1. Simulate ascent followed by descent
 * 2. Verify apogee detected and pyro fires
 * 3. Measure timing accuracy
 */

void hil_test_apogee_triggers(void) {
  set_flight_state(COAST);

  // Simulate 60 seconds of coast phase
  // Real data: altitude rises from 1000m to 1250m

  uint32_t t_apogee_expected = millis() + 15000;  // Expected at 15s
  uint32_t t_apogee_detected = 0;

  for (int i = 0; i < 1500; i++) {  // 15 seconds
    // Simulate pressure/altitude data showing ascent
    float altitude_m = 1000.0f + (i * 0.167f);  // 0.167 m/ms rise rate

    if (i > 100) {
      // Start descent after 1 second (simulated apogee)
      altitude_m = 1100.0f - ((i - 100) * 0.050f);  // Descent
    }

    simulate_barometer_altitude(altitude_m);
    update_flight_logic(10);

    if (get_flight_state() == APOGEE && t_apogee_detected == 0) {
      t_apogee_detected = millis();
    }
  }

  // Verify apogee detected
  TEST_ASSERT_NOT_EQUAL(0, t_apogee_detected);

  // Verify timing accuracy (±500ms tolerance)
  uint32_t timing_error = abs((int32_t)(t_apogee_detected - t_apogee_expected));
  TEST_ASSERT_LESS_THAN_UINT32(500, timing_error);

  // Verify drogue fired
  TEST_ASSERT_TRUE(pyro_channel_fired(DROGUE));

  printf("Apogee detected: t=%d ms (error: %d ms)\n",
         t_apogee_detected, timing_error);
}
```

**Verification Criteria:**
- All three methods (barometer, accel, GPS) detect apogee
- 2-of-3 voting triggers deployment
- Pyro timing within ±500ms of expected
- No false apogee triggers during ascent

#### 4.2.3 Servo Response Test

**File:** `test/hardware_in_loop/hil_servo_response.cpp`

```cpp
/**
 * TEST: Servo Command Response & Timing
 * Purpose: Measure servo actuator response to flight commands
 *
 * Procedure:
 * 1. Command servo to various angles
 * 2. Measure response time with oscilloscope
 * 3. Verify range and smoothness
 */

void hil_test_servo_response(void) {
  // Connect oscilloscope to servo PWM output pin
  // Connect potentiometer to feedback pin (simulates servo position sensor)

  struct ServoTest {
    int16_t command_angle;
    uint32_t expected_response_time_ms;
    uint16_t expected_pwm_value;
  };

  ServoTest tests[] = {
    {0,    150, 1500},      // Center: 1500us PWM
    {45,   150, 1950},      // Full right: 1950us PWM
    {-45,  150, 1050},      // Full left: 1050us PWM
    {22.5, 150, 1725},      // Half right: 1725us PWM
  };

  for (const auto& test : tests) {
    // Issue servo command
    send_servo_command(0, test.command_angle);

    uint32_t t_start = micros();
    uint16_t measured_pwm = measure_pwm_on_oscilloscope();
    uint32_t t_response = micros() - t_start;

    // Verify response
    TEST_ASSERT_UINT16_WITHIN(50, test.expected_pwm_value, measured_pwm);
    TEST_ASSERT_LESS_THAN_UINT32(test.expected_response_time_ms * 1000, t_response);

    delay(500);  // Wait for servo to settle
  }
}
```

**Verification Criteria:**
- Servo responds in <250ms to commands
- PWM accuracy within ±2.5% of target
- No overshoot or oscillation
- Smooth transitions between angles

#### 4.2.4 Sensor Failover Test

**File:** `test/hardware_in_loop/hil_sensor_failover.cpp`

```cpp
/**
 * TEST: Dual Sensor Redundancy & Failover
 * Purpose: Verify automatic switching from primary to backup sensor
 *
 * Procedure:
 * 1. Use primary sensor (ICM-20948) during normal operation
 * 2. Trigger high-G event (inject fault)
 * 3. Verify backup sensor (KX134) activates
 */

void hil_test_sensor_failover(void) {
  // Verify primary sensor active and healthy
  TEST_ASSERT_TRUE(imu_manager.isPrimarySensorHealthy());
  TEST_ASSERT_EQUAL(AccelMeter_ICM, imu_manager.getActiveSensor());

  // Simulate high-G event
  // Function generator produces 100G signal (exceeds ICM ±16G range)

  printf("Starting high-G simulation...\n");

  for (int i = 0; i < 100; i++) {
    // ICM-20948 saturates and reports error
    float primary_accel = imu_manager.getAccelX();

    if (fabs(primary_accel) > 15.0f) {
      // Near saturation - backup should be activating

      update_flight_logic(10);

      if (imu_manager.isBackupSensorActive()) {
        uint32_t t_failover = millis();

        // Verify backup sensor provides valid data
        float backup_accel = imu_manager.getAccelX();
        TEST_ASSERT_NOT_EQUAL(primary_accel, backup_accel);
        TEST_ASSERT_LESS_THAN_FLOAT(fabs(backup_accel), 100.0f);

        printf("Failover successful at t=%d ms\n", t_failover);
        return;
      }
    }

    delay(10);
  }

  // Should have failover'd by now
  TEST_FAIL_MESSAGE("Sensor failover did not occur");
}
```

**Verification Criteria:**
- Failover occurs within 100ms of high-G detection
- Backup sensor (KX134) provides valid data
- No data gaps during transition
- Flight logic continues without interruption

#### 4.2.5 Pyro Timing Test

**File:** `test/hardware_in_loop/hil_pyro_timing.cpp`

```cpp
/**
 * TEST: Pyro Channel Deployment Timing
 * Purpose: Verify pyro channels fire with correct timing and no crosstalk
 *
 * Procedure:
 * 1. Connect each pyro channel to LED + timing circuit
 * 2. Trigger deployments at expected times
 * 3. Measure pulse timing and duration with oscilloscope
 */

void hil_test_pyro_timing(void) {
  // Connect oscilloscope to:
  // - Channel 1: Drogue pyro (pin 2)
  // - Channel 2: Main pyro (pin 3)

  printf("Testing drogue pyro channel...\n");

  // Simulate apogee event
  set_flight_state(APOGEE);

  uint32_t t_fire = millis();

  // Wait for deployment
  update_flight_logic(100);

  // Measure on oscilloscope:
  // - Pulse duration (should be ~50-100ms)
  // - Voltage level (should be ~12V when firing)
  // - No ringing or noise

  bool fired = read_pyro_fired_status(DROGUE);
  TEST_ASSERT_TRUE(fired);

  uint32_t fire_time = read_pyro_timestamp(DROGUE);
  TEST_ASSERT_UINT32_WITHIN(50, t_fire, fire_time);

  printf("Drogue fired: t=%d ms\n", fire_time);

  delay(5000);  // Wait 5s for drogue deployment

  // Now test main pyro
  printf("Testing main pyro channel...\n");

  // Descend to main deployment altitude
  for (int i = 0; i < 100; i++) {
    float altitude = 500.0f - (i * 5.0f);
    simulate_barometer_altitude(altitude);
    update_flight_logic(10);
  }

  fired = read_pyro_fired_status(MAIN);
  TEST_ASSERT_TRUE(fired);
}
```

**Verification Criteria:**
- Drogue fires within 100ms of apogee
- Main fires at specified descent altitude
- Pulse timing 50-100ms (typical e-match ignition)
- No crosstalk between channels
- Continuity circuit validates all channels before flight

---

## 5. System Verification Matrix (6.4.4)

### 5.1 Verification Checklist

**Format:** Mapping of feature → test → status tracking

```
SENSOR SYSTEMS
└─ ICM-20948 Primary Accelerometer
   ├─ Initialization                    [test_imu_init]              □ PASS
   ├─ Data acquisition (100 Hz)         [test_imu_data_rate]         □ PASS
   ├─ Calibration offset               [test_imu_calibration]        □ PASS
   ├─ Sensitivity & scale              [test_imu_sensitivity]        □ PASS
   ├─ Temperature compensation         [test_imu_temp_drift]         □ PASS
   ├─ Saturation handling              [test_imu_saturation]         □ PASS
   └─ Error recovery                   [test_imu_error_state]        □ PASS

└─ MS5611 Barometric Sensor
   ├─ Pressure reading accuracy        [test_baro_accuracy]          □ PASS
   ├─ Altitude calculation             [test_baro_altitude]          □ PASS
   ├─ Sea-level calibration            [test_baro_calibration]       □ PASS
   ├─ Hysteresis handling              [test_baro_hysteresis]        □ PASS
   └─ Data logging format              [test_baro_logging]           □ PASS

└─ GPS/GNSS Receiver
   ├─ Cold start acquisition           [test_gps_coldstart]          □ PASS
   ├─ Warm start acquisition           [test_gps_warmstart]          □ PASS
   ├─ Fix quality indicator            [test_gps_quality]            □ PASS
   ├─ Altitude accuracy (±5m)          [test_gps_altitude_accuracy]  □ PASS
   ├─ Velocity accuracy (±1 m/s)       [test_gps_velocity_accuracy]  □ PASS
   ├─ Loss detection & recovery        [test_gps_loss_recovery]      □ PASS
   └─ Jamming resistance (if applicable) [test_gps_jamming]          □ PASS

└─ KX134 Backup Accelerometer
   ├─ ±64G range                       [test_kx134_range]            □ PASS
   ├─ Automatic activation at ±16G+    [test_kx134_activation]       □ PASS
   ├─ Data quality during high-G       [test_kx134_high_g]           □ PASS
   └─ Switchover timing                [test_kx134_switchover]       □ PASS

STATE MACHINE
└─ 15-State Flight State Machine
   ├─ STARTUP → CALIBRATION            [test_state_startup]          □ PASS
   ├─ CALIBRATION → PAD_IDLE           [test_state_calibration]      □ PASS
   ├─ PAD_IDLE → ARMED (command)       [test_state_arm]              □ PASS
   ├─ ARMED → BOOST (accel threshold)  [test_state_launch]           □ PASS
   ├─ BOOST → COAST (burnout)          [test_state_coast]            □ PASS
   ├─ COAST → APOGEE (2-of-3)          [test_state_apogee]           □ PASS
   ├─ APOGEE → DROGUE_DEPLOY           [test_state_drogue_deploy]    □ PASS
   ├─ DROGUE_DEPLOY → DROGUE_DESCENT   [test_state_drogue_descent]   □ PASS
   ├─ DROGUE_DESCENT → MAIN_DEPLOY     [test_state_main_deploy]      □ PASS
   ├─ MAIN_DEPLOY → MAIN_DESCENT       [test_state_main_descent]     □ PASS
   ├─ MAIN_DESCENT → LANDED            [test_state_landed]           □ PASS
   ├─ LANDED → RECOVERY                [test_state_recovery]         □ PASS
   ├─ Any → ERROR (on sensor failure)  [test_state_error]            □ PASS
   ├─ ERROR → PAD_IDLE (manual clear)  [test_state_error_recovery]   □ PASS
   └─ EEPROM persistence across power  [test_state_persistence]      □ PASS

APOGEE DETECTION (Multi-method with 2-of-3 voting)
├─ Barometric Method
│  ├─ Pressure descent detection       [test_apogee_baro_descent]    □ PASS
│  ├─ Hysteresis filter (±2 Pa)        [test_apogee_baro_hysteresis] □ PASS
│  ├─ Noise rejection                  [test_apogee_baro_noise]      □ PASS
│  └─ Sensitivity ±50m @ 1000m alt     [test_apogee_baro_accuracy]   □ PASS
│
├─ Acceleration Method
│  ├─ Vertical accel sign change       [test_apogee_accel_sign]      □ PASS
│  ├─ Confirmation count (5 samples)   [test_apogee_accel_confirm]   □ PASS
│  ├─ False positive rejection         [test_apogee_accel_false_pos] □ PASS
│  └─ Accuracy ±100m @ 1000m           [test_apogee_accel_accuracy]  □ PASS
│
├─ GPS Method
│  ├─ Altitude descent detection       [test_apogee_gps_descent]     □ PASS
│  ├─ Fix quality check (≥2D fix)      [test_apogee_gps_quality]     □ PASS
│  ├─ Jamming resistance               [test_apogee_gps_jamming]     □ PASS
│  └─ Accuracy ±5m (typical GPS)       [test_apogee_gps_accuracy]    □ PASS
│
├─ Backup Timer Fallback
│  ├─ Triggers at 20s after COAST      [test_apogee_timer_trigger]   □ PASS
│  ├─ Fallback only when all fail      [test_apogee_timer_condition] □ PASS
│  └─ No false triggers during ascent  [test_apogee_timer_spurious]  □ PASS
│
└─ 2-of-3 Voting System
   ├─ All three detect                 [test_apogee_voting_all]      □ PASS
   ├─ Two of three detect              [test_apogee_voting_2of3]     □ PASS
   ├─ One fails, two pass              [test_apogee_voting_failover] □ PASS
   ├─ Averaging of results             [test_apogee_voting_average]  □ PASS
   └─ Altitude estimation accuracy     [test_apogee_voting_accuracy] □ PASS

DEPLOYMENT SYSTEM
├─ Drogue Deployment
│  ├─ Pyro fired at apogee            [test_drogue_apogee]           □ PASS
│  ├─ Timing delay <100ms             [test_drogue_timing]           □ PASS
│  ├─ Continuity check pre-flight     [test_drogue_continuity]       □ PASS
│  ├─ Redundancy check                [test_drogue_redundancy]       □ PASS
│  └─ Charge quantity validation      [test_drogue_charge]           □ PASS
│
└─ Main Deployment
   ├─ Fired at altitude (100m AGL)    [test_main_altitude]           □ PASS
   ├─ Timing accuracy ±10m            [test_main_timing_accuracy]    □ PASS
   ├─ Not fired during descent  (until reaching altitude) [test_main_premature] □ PASS
   ├─ Continuity check                [test_main_continuity]         □ PASS
   └─ Dual-deploy system validation   [test_deployment_dual_deploy]  □ PASS

GUIDANCE CONTROL SYSTEM (if enabled)
├─ Servo Control
│  ├─ Range of motion ±45°            [test_servo_range]             □ PASS
│  ├─ Response latency <300ms         [test_servo_latency]           □ PASS
│  ├─ Smoothing filter                [test_servo_smoothing]         □ PASS
│  ├─ Saturation detection            [test_servo_saturation]        □ PASS
│  └─ Center after landing            [test_servo_center]            □ PASS
│
├─ PID Control
│  ├─ Pitch control stability         [test_pid_pitch]               □ PASS
│  ├─ Yaw control stability           [test_pid_yaw]                 □ PASS
│  ├─ Roll damping                    [test_pid_roll]                □ PASS
│  ├─ Integral windup prevention      [test_pid_windup]              □ PASS
│  └─ Derivative filtering            [test_pid_derivative]          □ PASS
│
├─ Stability Monitoring
│  ├─ Angular rate limits             [test_stability_rates]         □ PASS
│  ├─ Attitude error limits           [test_stability_attitude]      □ PASS
│  ├─ Actuator saturation detection   [test_stability_saturation]    □ PASS
│  └─ Failsafe trigger                [test_stability_failsafe]      □ PASS
│
└─ Failsafe System
   ├─ Passive mode (center servos)    [test_failsafe_passive]        □ PASS
   ├─ Gain reduction                  [test_failsafe_gain_reduce]    □ PASS
   ├─ Automatic recovery              [test_failsafe_recovery]       □ PASS
   └─ Manual override                 [test_failsafe_override]       □ PASS

DATA LOGGING
├─ SD Card Interface
│  ├─ Card detection                  [test_sd_detect]               □ PASS
│  ├─ Write speed (>100 KB/s)         [test_sd_write_speed]          □ PASS
│  ├─ Capacity check (>4GB)           [test_sd_capacity]             □ PASS
│  └─ Error recovery on full disk     [test_sd_full_disk]            □ PASS
│
├─ CSV Format
│  ├─ Header line present             [test_csv_header]              □ PASS
│  ├─ Field order consistency         [test_csv_field_order]         □ PASS
│  ├─ Delimiter (comma) correct       [test_csv_delimiter]           □ PASS
│  ├─ All fields populated            [test_csv_fields_complete]     □ PASS
│  └─ Numeric precision (4-5 decimals) [test_csv_precision]          □ PASS
│
├─ Data Integrity
│  ├─ No corrupted lines              [test_csv_corruption]          □ PASS
│  ├─ Consistent timestamp sequence   [test_csv_timestamps]          □ PASS
│  ├─ All 100Hz data captured         [test_csv_sample_rate]         □ PASS
│  └─ Post-flight parsing works       [test_csv_parsing]             □ PASS
│
└─ Performance
   ├─ No logging latency             [test_logging_latency]         □ PASS
   ├─ Circular buffer on overflow    [test_logging_overflow]        □ PASS
   └─ 60+ minute flight support      [test_logging_duration]        □ PASS

SAFETY SYSTEMS
├─ Watchdog Timer
│  ├─ Feed timing (every 100ms)       [test_watchdog_feed]           □ PASS
│  ├─ Timeout trigger on hang         [test_watchdog_timeout]        □ PASS
│  ├─ Boot after watchdog reset       [test_watchdog_boot]           □ PASS
│  └─ EEPROM recovery                 [test_watchdog_recovery]       □ PASS
│
├─ Battery Monitoring
│  ├─ Voltage reading accuracy        [test_battery_accuracy]        □ PASS
│  ├─ Low battery detection           [test_battery_low]             □ PASS
│  ├─ Flight time estimation          [test_battery_flight_time]     □ PASS
│  └─ Thermal warning                 [test_battery_thermal]         □ PASS
│
├─ Thermal Management
│  ├─ Temp sensor reading             [test_thermal_sensor]          □ PASS
│  ├─ Throttle on >70°C               [test_thermal_throttle]        □ PASS
│  ├─ Emergency shutdown >85°C        [test_thermal_shutdown]        □ PASS
│  └─ Sensor drift compensation       [test_thermal_drift]           □ PASS
│
└─ Error State Isolation
   ├─ Sensor error → ERROR state      [test_error_isolation]         □ PASS
   ├─ No state machine in ERROR       [test_error_state_lock]        □ PASS
   ├─ Manual recovery via command     [test_error_clear_command]     □ PASS
   └─ Safe defaults on recovery       [test_error_safe_defaults]     □ PASS

POWER MANAGEMENT
├─ Consumption
│  ├─ Normal mode (<200mA avg)        [test_power_normal]            □ PASS
│  ├─ Peak current (<6A)              [test_power_peak]              □ PASS
│  ├─ Sleep mode (<5mA)               [test_power_sleep]             □ PASS
│  └─ Cost mode comparison            [test_power_comparison]        □ PASS
│
└─ Battery Optimization
   ├─ Reduced telemetry mode          [test_power_reduced_telemetry] □ PASS
   ├─ GPS disable option              [test_power_gps_disable]       □ PASS
   ├─ SD card optimization            [test_power_sd_optimize]       □ PASS
   └─ Thermal throttling              [test_power_thermal_throttle]  □ PASS

KALMAN FILTER (Sensor Fusion)
├─ Quaternion Estimation
│  ├─ Gyro integration                [test_kalman_gyro_int]         □ PASS
│  ├─ Accel update                    [test_kalman_accel_update]     □ PASS
│  ├─ Mag update (if available)       [test_kalman_mag_update]       □ PASS
│  └─ Singularity avoidance           [test_kalman_singularity]      □ PASS
│
├─ Filter Tuning
│  ├─ Process noise (Q matrix)        [test_kalman_q_tuning]         □ PASS
│  ├─ Measurement noise (R matrix)    [test_kalman_r_tuning]         □ PASS
│  └─ Convergence speed               [test_kalman_convergence]      □ PASS
│
└─ Accuracy
   ├─ Orientation accuracy ±5°        [test_kalman_accuracy_orient]  □ PASS
   ├─ Bias estimation                 [test_kalman_bias_est]         □ PASS
   └─ Long-flight stability           [test_kalman_long_flight]      □ PASS

COMMAND PROCESSOR
├─ Serial Interface
│  ├─ Baud rate 115200                [test_serial_baudrate]         □ PASS
│  ├─ Command parsing                 [test_serial_parsing]          □ PASS
│  └─ Response formatting             [test_serial_response]         □ PASS
│
├─ Flight Commands
│  ├─ "arm" - Arm for launch          [test_cmd_arm]                 □ PASS
│  ├─ "disarm" - Disarm               [test_cmd_disarm]              □ PASS
│  ├─ "clear_errors" - Error recovery [test_cmd_clear_errors]        □ PASS
│  ├─ "status_sensors" - Sensor diag  [test_cmd_status_sensors]      □ PASS
│  ├─ "calibrate" - Baro calibration  [test_cmd_calibrate]           □ PASS
│  └─ "preflight" - Pre-flight check  [test_cmd_preflight]           □ PASS
│
└─ Configuration
   ├─ "set_param" - Param change      [test_cmd_set_param]           □ PASS
   ├─ "get_param" - Param read        [test_cmd_get_param]           □ PASS
   └─ "save_config" - EEPROM persist  [test_cmd_save_config]         □ PASS

TOTAL CHECKPOINTS: 150+
```

### 5.2 Critical Path Identification

**Must Pass for Release (Critical):**
```
✓ State machine: All 15 transitions
✓ Apogee detection: 2-of-3 voting
✓ Deployment: Drogue timing <150ms, Main at altitude
✓ Data logging: Complete CSV with all fields
✓ Safety: Watchdog, thermal, error isolation
✓ Sensor redundancy: Failover under high-G
```

**High Priority (High Risk if Failing):**
```
✓ PID stability: No oscillation or saturation
✓ GPS integration: Fix acquisition <5s
✓ Servo response: <300ms latency
✓ Command processor: All commands working
```

**Medium Priority (Can defer to v1.0.1):**
```
□ Power optimization: Fine-tuning consumption
□ Trajectory following: Advanced feature
□ Cloud integration: Optional telemetry
```

---

## 6. Test Data Fixtures

### 6.1 Synthetic Flight Data Generation

**File:** `test/fixtures/generated/generate_synthetic_data.py`

```python
#!/usr/bin/env python3
"""
Generate realistic synthetic flight data for testing.
Simulates motors, aerodynamics, and sensor physics.
"""

import csv
import math
import numpy as np
from dataclasses import dataclass

@dataclass
class SyntheticFlightConfig:
    """Configuration for synthetic flight"""
    apogee_altitude_m: float = 1250.0
    coast_time_s: float = 15.0
    descent_time_s: float = 40.0
    boost_time_s: float = 3.0
    data_rate_hz: int = 100
    noise_level_g: float = 0.01  # 0.01G noise
    gps_dropout_start_s: float = -1.0  # No dropout by default
    gps_dropout_end_s: float = -1.0
    temperature_c: float = 20.0

def generate_boost_phase(config, t_start_s):
    """Generate acceleration data during motor boost"""
    boost_samples = int(config.boost_time_s * config.data_rate_hz)
    data_points = []

    for i in range(boost_samples):
        t = t_start_s + (i / config.data_rate_hz)

        # Motor curve: 0G -> 50G -> 30G (typical L-class motor)
        phase = i / boost_samples
        if phase < 0.4:
            # Ramp up: 0 -> 50G
            accel_g = 50.0 * (phase / 0.4)
        elif phase < 0.8:
            # Sustain: 50G
            accel_g = 50.0
        else:
            # Ramp down: 50G -> 30G
            accel_g = 50.0 - 20.0 * ((phase - 0.8) / 0.2)

        # Add sensor noise
        noise = np.random.normal(0, config.noise_level_g)
        accel_z = 9.81 + accel_g + noise

        data_points.append({
            'timestamp_ms': int(t * 1000),
            'accel_x_mps2': np.random.normal(0, config.noise_level_g),
            'accel_y_mps2': np.random.normal(0, config.noise_level_g),
            'accel_z_mps2': accel_z,
            'gyro_x_dps': 0.0,
            'gyro_y_dps': 0.0,
            'gyro_z_dps': 0.0,
            'pressure_pa': 101325.0 - (t * 0.5),  # Pressure decreases with altitude
            'temperature_c': config.temperature_c,
            'gps_lat': 34.876500,
            'gps_lon': -118.123400,
            'gps_alt_m': t * 50.0,  # 50 m/s ascent during boost
            'gps_fix_quality': 3,
        })

    return data_points

def generate_coast_phase(config, t_start_s, altitude_at_burnout):
    """Generate coast phase (motor off, ascending to apogee)"""
    coast_samples = int(config.coast_time_s * config.data_rate_hz)
    data_points = []

    current_velocity = 50.0  # m/s at burnout

    for i in range(coast_samples):
        t = t_start_s + (i / config.data_rate_hz)

        # Gravity deceleration: -9.81 m/s²
        # Drag increases with velocity (roughly)
        drag_decel = 0.1 * current_velocity  # Simplified drag model
        accel_z = -9.81 - drag_decel

        # Update velocity and altitude
        current_velocity += accel_z / config.data_rate_hz
        current_altitude = altitude_at_burnout + (current_velocity * (i / config.data_rate_hz))

        # Update pressure based on altitude
        pressure = calculate_pressure_from_altitude(current_altitude)

        data_points.append({
            'timestamp_ms': int(t * 1000),
            'accel_x_mps2': np.random.normal(0, config.noise_level_g),
            'accel_y_mps2': np.random.normal(0, config.noise_level_g),
            'accel_z_mps2': accel_z + np.random.normal(0, config.noise_level_g),
            'gyro_x_dps': 0.0,
            'gyro_y_dps': 0.0,
            'gyro_z_dps': 0.0,
            'pressure_pa': pressure,
            'temperature_c': config.temperature_c,
            'gps_lat': 34.876500 + (i * 0.00001),  # Slight GPS drift
            'gps_lon': -118.123400 + (i * 0.00001),
            'gps_alt_m': current_altitude,
            'gps_fix_quality': 3,
        })

    return data_points

def calculate_pressure_from_altitude(altitude_m):
    """ISA model for pressure vs altitude"""
    # Simplified: P = P0 * (1 - (g * M * h) / (R * T0))^(R*L/g*M)
    # For simplicity: P ~ P0 * exp(-h / 8500)
    return 101325.0 * math.exp(-altitude_m / 8500.0)

def generate_full_flight(config):
    """Generate complete synthetic flight"""
    data_points = []

    # Phase 1: Boost (0-3s)
    print(f"Generating boost phase...")
    data_points.extend(generate_boost_phase(config, 0.0))

    altitude_at_burnout = config.boost_time_s * 50.0  # Rough estimate

    # Phase 2: Coast (3-18s)
    print(f"Generating coast phase...")
    data_points.extend(generate_coast_phase(config, config.boost_time_s, altitude_at_burnout))

    # Phase 3: Descent (to be implemented)
    # ...

    return data_points

if __name__ == "__main__":
    config = SyntheticFlightConfig(
        apogee_altitude_m=1250.0,
        noise_level_g=0.01
    )

    flight_data = generate_full_flight(config)

    # Write to CSV
    with open('nominal_flight_1250m.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=flight_data[0].keys())
        writer.writeheader()
        writer.writerows(flight_data)

    print(f"Generated {len(flight_data)} data points")
    print(f"Duration: {flight_data[-1]['timestamp_ms'] / 1000.0} seconds")
```

### 6.2 Edge Case Data Sets

**Generate extreme scenarios:**

```python
# test/fixtures/generated/generate_edge_cases.py

def generate_high_g_flight():
    """Simulate motor with early spike (high-G transient)"""
    # Boost ramp: 0G -> 100G in 50ms (stress test for KX134 activation)
    pass

def generate_gps_loss_scenario():
    """GPS dropout during coast phase"""
    # All three methods except barometer: generate pressure-only detection
    pass

def generate_thermal_drift_scenario():
    """Extended 30+ minute flight with temperature drift"""
    # Test temperature compensation and long-flight stability
    pass

def generate_sensor_noise_scenario():
    """Realistic Gaussian noise overlay"""
    # Add colored noise (1/f) to simulate real sensor behavior
    pass
```

---

## 7. CI/CD Integration

### 7.1 GitHub Actions Workflow

**File:** `.github/workflows/test.yml`

```yaml
name: Tests

on:
  push:
    branches: [ develop, master ]
  pull_request:
    branches: [ develop, master ]

jobs:
  unit_tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - uses: actions/setup-python@v2
        with:
          python-version: 3.9

      - name: Install PlatformIO
        run: pip install platformio

      - name: Run unit tests
        run: pio test -e native_test -v

      - name: Upload test results
        if: always()
        uses: actions/upload-artifact@v2
        with:
          name: unit-test-results
          path: .pio/build/native_test/test_results.xml

  regression_tests:
    runs-on: ubuntu-latest
    needs: unit_tests
    steps:
      - uses: actions/checkout@v2
      - name: Run regression suite
        run: ./test/scripts/run_regression_tests.sh

      - name: Check coverage
        run: |
          coverage report --fail-under=95

      - name: Upload coverage
        uses: codecov/codecov-action@v2

  build_teensy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Build firmware
        run: pio run -e teensy41 -v

      - name: Check binary size
        run: |
          SIZE=$(stat -f%z "build/teensy41/firmware.bin")
          if [ $SIZE -gt 200000 ]; then
            echo "WARNING: Firmware size increased significantly"
          fi

      - name: Upload binary
        uses: actions/upload-artifact@v2
        with:
          name: firmware-binary
          path: .pio/build/teensy41/firmware.bin
```

### 7.2 Test Result Reporting

**Format:** HTML + JSON for dashboard integration

```html
<!-- test/results/report.html -->
<!DOCTYPE html>
<html>
<head>
    <title>TripleT Flight Firmware - Test Report</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .pass { background-color: #90EE90; }
        .fail { background-color: #FF6B6B; }
        .warn { background-color: #FFD700; }
        table { border-collapse: collapse; width: 100%; }
        th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
    </style>
</head>
<body>
    <h1>TripleT Flight Firmware v1.0.0 - Test Report</h1>
    <p>Generated: 2026-02-16 14:32:00 UTC</p>

    <h2>Summary</h2>
    <table>
        <tr>
            <th>Category</th>
            <th>Tests</th>
            <th>Passed</th>
            <th>Failed</th>
            <th>Coverage</th>
            <th>Status</th>
        </tr>
        <tr class="pass">
            <td>Unit Tests</td>
            <td>50</td>
            <td>50</td>
            <td>0</td>
            <td>97%</td>
            <td>PASS</td>
        </tr>
        <tr class="pass">
            <td>Integration Tests</td>
            <td>8</td>
            <td>8</td>
            <td>0</td>
            <td>95%</td>
            <td>PASS</td>
        </tr>
        <tr class="pass">
            <td>Regression Tests</td>
            <td>42</td>
            <td>42</td>
            <td>0</td>
            <td>96%</td>
            <td>PASS</td>
        </tr>
        <tr class="pass">
            <td>Hardware-in-Loop</td>
            <td>7</td>
            <td>7</td>
            <td>0</td>
            <td>94%</td>
            <td>PASS</td>
        </tr>
    </table>

    <h2>Critical Path Status</h2>
    <table>
        <tr class="pass"><td>State Machine</td><td>15/15 transitions</td><td>PASS</td></tr>
        <tr class="pass"><td>Apogee Detection</td><td>6/6 methods</td><td>PASS</td></tr>
        <tr class="pass"><td>Deployment System</td><td>Timing within spec</td><td>PASS</td></tr>
        <tr class="pass"><td>Data Logging</td><td>CSV format verified</td><td>PASS</td></tr>
        <tr class="pass"><td>Safety Systems</td><td>All checks passing</td><td>PASS</td></tr>
    </table>

    <h2>Detailed Results</h2>
    <!-- Expand each category with clickable details -->
</body>
</html>
```

---

## 8. Effort Estimate & Timeline

### 8.1 Implementation Schedule

| Task | Effort | Duration | Start | End |
|------|--------|----------|-------|-----|
| **Test Framework Setup** | 40 hours | 1 week | Week 1 | Week 1 |
| - Directory structure & conventions | 4 hours | 1 day | W1 Mon | W1 Mon |
| - Mock sensor implementations | 12 hours | 2 days | W1 Tue | W1 Wed |
| - HAL test setup | 8 hours | 1 day | W1 Thu | W1 Thu |
| - Fixture data generation | 16 hours | 2 days | W1 Fri | W1+ Mon |
| **Unit Test Suite** | 60 hours | 1.5 weeks | Week 2 | Week 3 |
| - State machine tests (8) | 8 hours | 1 day | W2 Mon | W2 Tue |
| - Apogee detection tests (6) | 10 hours | 1 day | W2 Wed | W2 Thu |
| - Servo/control tests (13) | 12 hours | 1.5 days | W2 Fri | W3 Sat |
| - Sensor health tests (8) | 10 hours | 1 day | W3 Mon | W3 Tue |
| - Data logging tests (6) | 8 hours | 1 day | W3 Wed | W3 Thu |
| - Miscellaneous tests (9) | 12 hours | 1.5 days | W3 Fri | W3+ |
| **Integration Tests** | 40 hours | 1 week | Week 3 | Week 4 |
| - Full flight simulator | 20 hours | 2 days | W3 Wed | W3 Fri |
| - Scenario tests (8) | 20 hours | 2 days | W4 Mon | W4 Tue |
| **Regression Suite** | 35 hours | 1 week | Week 4 | Week 5 |
| - Legacy compatibility | 8 hours | 1 day | W4 Wed | W4 Thu |
| - Safety systems | 12 hours | 1.5 days | W4 Fri | W5 Sat |
| - Data format tests | 8 hours | 1 day | W5 Mon | W5 Tue |
| - Command processor | 7 hours | 1 day | W5 Wed | W5 Thu |
| **Hardware-in-Loop Setup** | 50 hours | 1.5 weeks | Week 5 | Week 6 |
| - Test harness design | 10 hours | 1 day | W5 Fri | W6 Sat |
| - Launch detection test | 8 hours | 1 day | W6 Mon | W6 Tue |
| - Apogee trigger test | 10 hours | 1 day | W6 Wed | W6 Thu |
| - Servo/pyro tests | 12 hours | 1.5 days | W6 Fri | W6+ |
| - Sensor failover tests | 10 hours | 1 day | W6+ | W7 Mon |
| **CI/CD Integration** | 20 hours | 1 week | Week 6 | Week 7 |
| - GitHub Actions setup | 8 hours | 1 day | W6 Thu | W6 Fri |
| - Test result reporting | 6 hours | 1 day | W7 Mon | W7 Tue |
| - Coverage analysis | 6 hours | 1 day | W7 Wed | W7 Thu |
| **Documentation** | 30 hours | 1 week | Week 7 | Week 8 |
| - Test procedures | 12 hours | 1.5 days | W7 Fri | W8 Sat |
| - Troubleshooting guide | 8 hours | 1 day | W8 Mon | W8 Tue |
| - Release notes | 10 hours | 1 day | W8 Wed | W8 Thu |
| **TOTAL** | **275 hours** | **8 weeks** | **Week 1** | **Week 8** |

### 8.2 Parallel Execution Strategy

Tests can run in parallel for faster feedback:

```
Week 1: Setup (sequential - foundation)
  └─ Framework, mocks, fixtures

Weeks 2-3: Testing (parallel)
  ├─ Unit tests → Integration tests → Regression tests (sequential feedback)
  └─ Run all at EOF each day, report results

Week 4: Full suite execution
  ├─ All 100+ tests on every commit
  └─ Parallel execution: ~45 min for full suite

Week 5-6: Hardware validation
  ├─ Manual HIL tests (can't parallelize)
  └─ Real flight testing (1 flight/week)

Week 7: CI/CD integration
  └─ Automatic test runs on every push

Week 8: Documentation & release prep
  └─ Final verification, release candidate build
```

### 8.3 Resource Requirements

- **Developer time:** 275 hours over 8 weeks (~35 hours/week)
- **Hardware:** 1 Teensy 4.1 + sensors for HIL
- **CI/CD platform:** GitHub Actions (free for public repos)
- **Test frameworks:** Unity (included), PlatformIO (free tier)

---

## Conclusion

This Phase 6.4 Testing Framework Plan provides:

1. **Comprehensive structure** for 100+ automated tests
2. **Clear roadmap** from unit tests → integration → regression → HIL
3. **Production-ready validation** with 95%+ code coverage
4. **CI/CD pipeline** for continuous verification
5. **Realistic implementation timeline** (8 weeks, 275 hours)

Upon completion, TripleT v1.0.0 will have:
- ✅ 50+ unit tests
- ✅ 8 integration test suites
- ✅ 42 regression tests
- ✅ 7 hardware-in-loop procedures
- ✅ 150+ verification checkpoints
- ✅ Automated CI/CD on every commit

**Next Step:** Begin Phase 6.4 implementation with test framework setup (Week 1).

