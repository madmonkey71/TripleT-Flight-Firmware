# Phase 6.4: Quick Reference Checklist

**Quick guide for implementing the testing framework**
**Use alongside:** PHASE_6_4_TESTING_FRAMEWORK.md

---

## Implementation Checklist

### Week 1: Setup Foundation

- [ ] **Create test directory structure**
  ```bash
  mkdir -p test/{unit,integration,regression,hardware_in_loop,fixtures/{recordings,generated,sensor_characterization},scripts}
  mkdir -p test/results
  ```

- [ ] **Create mock sensor base classes** (in `test/mocks/mock_sensors.h`)
  - [ ] MockIMUSensor with setAcceleration(), getAccelX/Y/Z(), setSimulationMode()
  - [ ] MockBarometer with setAltitude(), getPressure()
  - [ ] MockGPS with setPosition(), hasFix()
  - [ ] MockServo with setAngle(), getAngle()
  - [ ] MockSDCard with write(), closeFile()

- [ ] **Create fixture infrastructure**
  - [ ] CSV format specification (timestamp_ms, accel_x, accel_y, ...)
  - [ ] CSV loader function (parseCSVFile())
  - [ ] Generate synthetic data script (Python)

- [ ] **Update platformio.ini**
  - [ ] Ensure `test_framework = unity` is set
  - [ ] Configure `-DNATIVE_TEST_BUILD` flag

- [ ] **Create test runner scripts**
  - [ ] `test/scripts/run_all_tests.sh`
  - [ ] `test/scripts/run_unit_tests.sh`
  - [ ] `test/scripts/run_regression_tests.sh`
  - [ ] `test/scripts/generate_coverage_report.sh`

### Week 2-3: Unit Tests (50+ tests)

#### State Machine Tests (8 tests)
**File:** `test/unit/test_state_machine_transitions.cpp`

- [ ] test_startup_to_calibration_transition
- [ ] test_calibration_to_pad_idle
- [ ] test_pad_idle_to_armed_command
- [ ] test_armed_to_boost_acceleration
- [ ] test_boost_to_coast_burnout
- [ ] test_coast_to_apogee_voting
- [ ] test_apogee_to_drogue_deploy
- [ ] test_error_recovery_manual_clear

#### Apogee Detection Tests (6 tests)
**File:** `test/unit/test_apogee_detection_all_methods.cpp`

- [ ] test_apogee_barometric_method_only
- [ ] test_apogee_acceleration_method_only
- [ ] test_apogee_gps_method_only
- [ ] test_apogee_voting_all_methods_agree
- [ ] test_apogee_voting_two_of_three_agree
- [ ] test_apogee_timer_fallback_when_all_fail

#### Servo/Control Tests (13 tests)
**File:** `test/unit/test_servo_control_stability.cpp`

- [ ] test_servo_range_full_deflection
- [ ] test_servo_saturation_detection
- [ ] test_pid_controller_stability_convergence
- [ ] test_pid_integral_windup_prevention
- [ ] test_servo_smoothing_filter
- [ ] test_servo_response_time_latency
- [ ] test_guidance_failsafe_triggers
- [ ] test_stability_monitor_angular_rates
- [ ] test_stability_monitor_attitude_error
- [ ] test_stability_monitor_actuator_saturation
- [ ] test_servo_feedback_validation
- [ ] test_servo_center_on_landing
- [ ] test_servo_commands_logged

#### Sensor Health Tests (8 tests)
**File:** `test/unit/test_sensor_health_monitoring.cpp`

- [ ] test_icm20948_initialization_success
- [ ] test_kx134_activation_high_g_detection
- [ ] test_ms5611_calibration_offset
- [ ] test_gps_fix_acquisition_latency
- [ ] test_temperature_drift_compensation
- [ ] test_sensor_redundancy_failover
- [ ] test_sensor_health_command_output
- [ ] test_error_state_recovery

#### Data Logging Tests (6 tests)
**File:** `test/unit/test_data_logging_format.cpp`

- [ ] test_sd_card_write_and_verify
- [ ] test_csv_header_field_order
- [ ] test_all_log_fields_populated
- [ ] test_numeric_precision_float_formatting
- [ ] test_csv_parsing_compatibility
- [ ] test_data_logging_large_file_capacity

#### Math/Utility Tests (3 tests)
**File:** `test/unit/test_math_functions.cpp`

- [ ] test_haversine_distance_calculation
- [ ] test_coordinate_conversions
- [ ] test_quaternion_math_operations

#### Other Unit Tests (6 tests)
**File:** `test/unit/test_misc.cpp`

- [ ] test_kalman_filter_convergence
- [ ] test_kalman_filter_gyro_integration
- [ ] test_power_management_estimation
- [ ] test_battery_low_detection
- [ ] test_thermal_monitoring
- [ ] test_watchdog_feed_timing

### Week 3-4: Integration Tests (8 test suites)

**File:** `test/integration/test_full_flight_simulation.cpp`

- [ ] Implement FullFlightSimulator class
- [ ] loadFlightDataCSV() function
- [ ] validateStateTransitions()
- [ ] validateApogeeDetection()
- [ ] validateDeploymentTiming()
- [ ] validateServoCommands()
- [ ] generateTestReport()

**Create scenario test files:**

- [ ] `test_nominal_flight.cpp` - Perfect conditions
- [ ] `test_high_g_flight.cpp` - Acceleration spike
- [ ] `test_gps_loss_scenario.cpp` - GPS dropout
- [ ] `test_sensor_failover.cpp` - Primary sensor failure
- [ ] `test_wind_conditions.cpp` - High wind stability
- [ ] `test_extended_flight.cpp` - Long duration (power test)
- [ ] `test_edge_cases.cpp` - EEPROM corruption, thermal
- [ ] `test_deployment_timing.cpp` - Precise timing validation

### Week 4: Regression Tests (6 test files, 42 total tests)

**File:** `test/regression/test_state_transitions.cpp`

- [ ] Verify all 15 state transitions work as before
- [ ] Verify no regressions from Phase 6.1-6.3 changes
- [ ] Test error recovery paths

**File:** `test/regression/test_safety_systems.cpp`

- [ ] test_watchdog_feed_cycle
- [ ] test_battery_monitoring
- [ ] test_thermal_throttling
- [ ] test_error_state_isolation
- [ ] test_pyro_channel_continuity
- [ ] test_backup_timer_fallback

**File:** `test/regression/test_data_format.cpp`

- [ ] Verify CSV format unchanged
- [ ] Verify all fields present
- [ ] Verify parsing compatibility

**File:** `test/regression/test_command_processor.cpp`

- [ ] test_arm_command
- [ ] test_disarm_command
- [ ] test_status_sensors_output
- [ ] test_calibrate_command
- [ ] test_clear_errors_command
- [ ] test_set_parameter_command
- [ ] test_get_parameter_command
- [ ] test_invalid_command_handling

**File:** `test/regression/test_legacy_compatibility.cpp`

- [ ] Ensure v0.10.0 functionality preserved
- [ ] No breaking changes to flight logic
- [ ] All documented features working

**File:** `test/regression/test_configuration_system.cpp`

- [ ] test_eeprom_persistence
- [ ] test_config_validation
- [ ] test_parameter_ranges
- [ ] test_defaults_on_reset

### Week 5-6: Hardware-in-Loop Tests

**Create test harness documentation:**

- [ ] `test/hardware_in_loop/hil_test_setup.h` - Common setup code
- [ ] `test/hardware_in_loop/README.md` - Equipment requirements

**Create HIL test procedures:**

- [ ] `hil_launch_detection.cpp` - Test ARMED→BOOST
- [ ] `hil_apogee_triggers.cpp` - Test apogee detection
- [ ] `hil_servo_response.cpp` - Test servo timing
- [ ] `hil_sensor_failover.cpp` - Test redundancy
- [ ] `hil_pyro_timing.cpp` - Test deployment channels
- [ ] `hil_gps_integration.cpp` - Test GPS lock
- [ ] `hil_full_flight_test.cpp` - End-to-end with real hardware

**Create HIL procedures documentation:**

- [ ] Equipment list
- [ ] Oscilloscope setup
- [ ] Function generator programming
- [ ] Step-by-step test procedures
- [ ] Verification criteria
- [ ] Troubleshooting guide

### Week 6-7: CI/CD Integration

- [ ] **Create GitHub Actions workflow** (`.github/workflows/test.yml`)
  - [ ] Unit tests on every push
  - [ ] Build firmware
  - [ ] Check binary size
  - [ ] Generate coverage report
  - [ ] Run regression suite

- [ ] **Create test result reporting**
  - [ ] `test/results/report.html` template
  - [ ] JSON output format
  - [ ] Coverage metrics dashboard

- [ ] **Create test automation scripts**
  - [ ] `scripts/run_all_tests.sh` - Full test suite
  - [ ] `scripts/compare_baseline.sh` - Regression detection
  - [ ] `scripts/ci_runner.py` - CI integration

- [ ] **Create documentation**
  - [ ] How to run tests locally
  - [ ] How to interpret results
  - [ ] How to debug failing tests

### Week 7-8: Documentation & Finalization

- [ ] **Write test procedures documentation**
  - [ ] HIL setup guide
  - [ ] Serial command reference
  - [ ] Troubleshooting guide
  - [ ] Failure modes & recovery

- [ ] **Create fixture data**
  - [ ] Generate synthetic flights
  - [ ] Record real test flights
  - [ ] Publish fixture library

- [ ] **Final verification**
  - [ ] All 50+ unit tests passing
  - [ ] All 8 integration suites passing
  - [ ] All 42 regression tests passing
  - [ ] 7 HIL procedures documented & tested
  - [ ] 95%+ code coverage achieved

---

## File Naming Convention

### Test Files
```
test_<feature>_<scenario>[_<variation>].cpp

Examples:
test_apogee_detection_barometric_only.cpp
test_state_machine_error_recovery.cpp
test_servo_control_stability_high_wind.cpp
```

### Mock Files
```
mock_<component>.h

Examples:
mock_imu.h
mock_sensors.h
mock_servo.h
```

### Fixture Files
```
<scenario>_<description>.csv

Examples:
nominal_flight_1250m.csv
high_g_boost_8g.csv
gps_loss_coast_phase.csv
```

---

## Test Counts by Category

| Category | Count | Status |
|----------|-------|--------|
| Unit Tests - State Machine | 8 | □ |
| Unit Tests - Apogee | 6 | □ |
| Unit Tests - Servo/Control | 13 | □ |
| Unit Tests - Sensor Health | 8 | □ |
| Unit Tests - Data Logging | 6 | □ |
| Unit Tests - Math/Misc | 9 | □ |
| **Unit Tests Total** | **50** | **□** |
| Integration Tests | 8 | □ |
| Regression Tests | 42 | □ |
| Hardware-in-Loop Tests | 7 | □ |
| **GRAND TOTAL** | **107** | **□** |

---

## Critical Path Dependencies

```
Prerequisite Tests (must pass first):
1. Unit Tests: State Machine
   └─ Required by: All integration tests
   └─ Reason: State transitions are foundation

2. Unit Tests: Sensor Health
   └─ Required by: Apogee & Deployment tests
   └─ Reason: Valid sensor data is critical

3. Unit Tests: Data Logging
   └─ Required by: Regression & HIL tests
   └─ Reason: Results depend on CSV format

Order of Execution:
Week 2: Unit Tests (foundation)
  ↓
Week 3: Integration Tests (system behavior)
  ↓
Week 4-5: Regression Tests (no regressions)
  ↓
Week 6-7: Hardware-in-Loop (real validation)
  ↓
Week 8: CI/CD Integration (automation)
```

---

## Quick Command Reference

### Run Tests During Development

```bash
# Run one specific test file
cd /mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware
pio test -e native_test -f test_apogee_detection

# Run all unit tests
pio test -e native_test

# Run with verbose output
pio test -e native_test -v

# Build firmware after tests pass
pio run -e teensy41

# Generate coverage report
./test/scripts/generate_coverage_report.sh
```

### Creating New Test File

```cpp
// Minimal test file template
#include <unity.h>

void setUp(void) { }
void tearDown(void) { }

void test_something(void) {
  TEST_ASSERT_TRUE(true);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_something);
  return UNITY_END();
}
```

### Common Test Assertions

```cpp
// Integer
TEST_ASSERT_EQUAL(expected, actual);
TEST_ASSERT_EQUAL_INT(expected, actual);
TEST_ASSERT_NOT_EQUAL(expected, actual);
TEST_ASSERT_GREATER_THAN(threshold, value);
TEST_ASSERT_LESS_THAN(threshold, value);

// Float
TEST_ASSERT_FLOAT_WITHIN(tolerance, expected, actual);
TEST_ASSERT_FLOAT_EQUAL(expected, actual);

// Boolean
TEST_ASSERT_TRUE(condition);
TEST_ASSERT_FALSE(condition);

// Strings
TEST_ASSERT_EQUAL_STRING(expected, actual);

// Arrays
TEST_ASSERT_EQUAL_INT_ARRAY(expected, actual, count);

// Memory
TEST_ASSERT_NULL(pointer);
TEST_ASSERT_NOT_NULL(pointer);

// Custom failure
TEST_FAIL_MESSAGE("Detailed error message");
```

---

## Verification Matrix Quick Reference

**Must Pass Before Release:**
```
☑ State Machine: All 15 transitions
☑ Apogee Detection: 2-of-3 voting working
☑ Deployment: Drogue within 150ms, Main at altitude
☑ Data Logging: CSV format correct
☑ Safety Systems: Watchdog, thermal, error isolation
☑ Code Coverage: 95%+
☑ No critical bugs in flight logs
```

**Should Pass:**
```
☑ Regression Tests: 100% pass rate
☑ HIL Tests: All procedures validated
☑ Integration Tests: All scenarios work
```

**Can Defer to v1.0.1:**
```
☐ Performance optimization
☐ Advanced features
☐ Nice-to-have enhancements
```

---

## Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| Test won't compile | Check include paths, ensure mock headers exist |
| Assertion failures | Add debug printf, step through logic |
| Timeout on test | Check infinite loops, add timeout config |
| Coverage <95% | Identify untested code, add edge case tests |
| Serial output garbled | Check baud rate (should be 115200) |
| Mock sensor not working | Verify simulation mode set, check method order |
| CSV parsing fails | Verify delimiter (comma), check field count |
| HIL test equipment issue | Check oscilloscope range, verify connections |

---

## Success Criteria Checklist

### Framework Setup ✓
- [ ] Test directory structure created
- [ ] Mock sensors functional
- [ ] Fixture loading works
- [ ] Test runner scripts operational

### Unit Tests ✓
- [ ] 50 tests written
- [ ] All tests passing
- [ ] Coverage > 90%
- [ ] No flaky tests (repeatable results)

### Integration Tests ✓
- [ ] 8 test suites complete
- [ ] Full flight simulator working
- [ ] Apogee detection validated
- [ ] Deployment timing verified

### Regression Tests ✓
- [ ] 42 tests passing
- [ ] No v0.10.0 features broken
- [ ] Safety systems verified
- [ ] Data format unchanged

### Hardware-in-Loop ✓
- [ ] 7 procedures documented
- [ ] All procedures tested
- [ ] Equipment validated
- [ ] Troubleshooting guide complete

### CI/CD ✓
- [ ] GitHub Actions configured
- [ ] Tests run on every push
- [ ] Results reported clearly
- [ ] Coverage tracked

### Documentation ✓
- [ ] Test procedures written
- [ ] Troubleshooting guide created
- [ ] All fixtures documented
- [ ] Release notes updated

### Final Release ✓
- [ ] 95%+ code coverage
- [ ] Zero critical bugs found
- [ ] All 107 tests passing
- [ ] Flight validation complete
- [ ] Ready for v1.0.0 release

---

## Next Steps

1. **Print this checklist** - Refer to it throughout Phase 6.4
2. **Review PHASE_6_4_TESTING_FRAMEWORK.md** - Full detailed specification
3. **Review PHASE_6_4_TEST_IMPLEMENTATION_GUIDE.md** - Code examples
4. **Start with Week 1 setup** - Create directories and mock framework
5. **Use GitHub Actions** - Automate test runs as you go
6. **Track coverage** - Aim for 95%+ on critical paths

---

**Estimated Total Effort:** 275 hours over 8 weeks
**Estimated Payoff:** Production-grade quality assurance for v1.0.0 release

Good luck! 🚀
