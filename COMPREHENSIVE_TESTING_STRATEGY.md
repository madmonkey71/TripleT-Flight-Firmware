# TripleT Flight Firmware - Comprehensive Testing Strategy

**Version:** 1.0
**Date:** 2025-02-14
**Status:** Strategic Planning Document
**Target Coverage Growth:** From ~5% to 60-80% over Phase 1 (6 months)

---

## Executive Summary

The TripleT Flight Firmware is a safety-critical rocket flight computer with:
- **864+ Hardware API calls** tightly coupled throughout codebase
- **~5% Current Unit Test Coverage** (primarily Kalman filter + apogee detection mocks)
- **13-State Flight State Machine** with complex transitions
- **Multi-Sensor Fusion** (ICM-20948, KX134, MS5611, GPS)
- **Critical Systems:** Pyro deployment, apogee detection, guidance control

This strategy addresses the fundamental challenge: **achieving high-confidence testing without complete hardware abstraction refactoring**.

**Key Insight:** Rather than a "big bang" refactor (costly, risky), implement **progressive refactoring** using a Hardware Abstraction Layer (HAL) with dependency injection, starting with highest-impact, lowest-cost items.

---

## 1. Hardware-Available Testing Strategy

### 1.1 Test Infrastructure for Hardware Access

#### Bench Test Environment Setup

**Required Equipment:**
- Teensy 4.1 flight computer with all sensors soldered
- ICM-20948 9-DoF IMU
- MS5611 barometric sensor
- KX134 high-G accelerometer
- u-blox GPS module (real GPS or simulator capable)
- Servo outputs (load-tested with dummy load resistors)
- Pyro channels (monitored via multimeter/oscilloscope, NOT live igniters)
- 5V power supply with current limiting (for safety)
- Oscilloscope (optional but valuable for timing analysis)
- Serial debug interface (USB-to-Serial adapter)

**Physical Test Stand:**
- Vibration isolation mounting (reduce external noise)
- Rotation mount for testing different orientations
- Tilt stand for gravity testing
- Faraday cage recommended for EMI isolation

#### Bench Test Procedures

**Test Phase 1: Component Validation (Before Flight)**

```
PROCEDURE: SENSOR HEALTH CHECK
Duration: 15-20 minutes
Equipment: Flight computer + serial monitor
Pre-requisites: Fresh firmware load

STEPS:
1. Power on system → observe startup sequence
2. Send "status_sensors" command
   - Verify all 5 sensors respond: ICM-20948, KX134, MS5611, GPS, NeoPixel
   - Check sensor health flags
   - Verify sample rates: ICM (100+ Hz), MS5611 (25 Hz), GPS (1 Hz)

3. ICM-20948 Validation
   - Command: "imu_debug"
   - Verify accel data: ~0.0, ~0.0, ~1.0g (z-axis gravity)
   - Verify gyro data: <0.5 rad/s (stationary)
   - Verify mag calibration: values not saturated
   - Acceptance: All three axes responsive, <1% noise floor

4. MS5611 Validation
   - Send "calibrate" command in PAD_IDLE state
   - Observe altitude value (should be stable ±2m)
   - Verify pressure/temperature readings reasonable
   - Acceptance: 5 consecutive reads within 1m, temp in range [-20, +50°C]

5. KX134 Validation (if enabled)
   - Check accel output matches ICM during stationary periods
   - Verify switchover logic (manual force with test code if needed)
   - Acceptance: Output within ±0.5g of ICM

6. GPS Validation
   - Wait 60-90 seconds for lock (stationary location with clear sky view)
   - Observe "getFixType" returns 3 (3D fix)
   - Verify altitude within ±10m of known elevation
   - Acceptance: 3D fix, valid lat/lon, altitude within ±15m

ACCEPTANCE CRITERIA:
- All 5 sensors report healthy status
- No sensor health flags set
- Data ranges reasonable for stationary Earth surface operation
- Repeat 3x on 3 different days (accounts for environmental variation)

LOG: Capture serial output to bench_test_log_DATE.txt for records
```

**Test Phase 2: State Machine Validation (Critical Path)**

```
PROCEDURE: STATE TRANSITION TEST - NORMAL SEQUENCE
Duration: 5 minutes
Equipment: Flight computer + serial monitor
Pre-requisites: Phase 1 passed

TEST SCENARIO:
- Force state transitions via manual commands (add temporary test commands)
- Verify each state handles transitions correctly
- Monitor for unintended side effects (e.g., pyro firing)

STATE SEQUENCE TO TEST:
  STARTUP → CALIBRATION → PAD_IDLE → ARMED → BOOST (simulated) → COAST
  → APOGEE → DROGUE_DEPLOY → DROGUE_DESCENT → MAIN_DEPLOY → MAIN_DESCENT
  → LANDED → RECOVERY

TEST STEPS:
1. Boot system → verify STARTUP state
   - Acceptance: LED shows correct color, boot banner printed

2. Command: "FORCE_CALIBRATION" (temp debug command)
   - Verify transition to CALIBRATION state
   - Verify sensor initialization completes
   - Acceptance: All sensors initialized, transitions to PAD_IDLE automatically

3. Command: "arm"
   - Verify transition to ARMED state
   - Acceptance: LED changes color, system reports "ARMED"
   - Verify pyro channels are SAFE (not fired)

4. Command: "FORCE_STATE BOOST 0" (temp debug)
   - Verify transition to BOOST state
   - Monitor pyro channels: must remain SAFE
   - Verify accel threshold detection is disabled
   - Acceptance: State transitions, LEDs update, pyro safe

5. Command: "FORCE_STATE COAST 0" (temp debug)
   - Verify transition to COAST state
   - Verify apogee detection activates
   - Acceptance: Apogee detection running (monitor via debug output)

6. Simulate apogee via manual altitude write
   - Command: "SET_ALTITUDE 500" (temp debug)
   - Send 5 consecutive commands with decreasing altitude
   - Expected: Apogee detected, transition to APOGEE state
   - Acceptance: State transitions automatically, no manual intervention required

7. Verify DROGUE_DEPLOY state
   - Expected automatic transition from APOGEE
   - Monitor pyro channel 1 (drogue): should fire briefly
   - Acceptance: Pyro fires, duration ~1s, LED shows deployment state

8. Verify MAIN_DEPLOY state
   - Command: "FORCE_STATE MAIN_DEPLOY 0"
   - Monitor pyro channel 2 (main): should fire briefly
   - Acceptance: Pyro fires, duration ~1s

9. Verify LANDED state
   - Force LANDED state, monitor LED and state name
   - Acceptance: Transitions to RECOVERY after timeout

CLEANUP: Remove temporary test commands before flight firmware build

ACCEPTANCE CRITERIA:
- All 13 state transitions occur without crashes
- No unintended pyro firing
- State names print correctly in serial output
- LED colors match state definitions
- System recovers correctly from each state
```

**Test Phase 3: Apogee Detection Validation (Safety-Critical)**

```
PROCEDURE: APOGEE DETECTION BENCH TEST
Duration: 20 minutes
Equipment: Flight computer + serial monitor + manual altitude simulator
Pre-requisites: Phase 1 & 2 passed

SETUP:
- Create temporary test code in flight_logic.cpp or a bench test mode
- Add command to inject mock altitude values
- Monitor apogee detection counter and flags

TEST SCENARIO A: Barometric Apogee Detection
Goal: Verify 5 consecutive descending barometer readings trigger apogee

STEPS:
1. Force system to COAST state
2. Set initial max altitude via "SET_ALT_MAX 1000" (1000m)
3. Send altitude sequence via "INJECT_ALTITUDE" commands:
   - Read 1: 999.0m  (desc)
   - Read 2: 998.0m  (desc)
   - Read 3: 997.0m  (desc)
   - Read 4: 996.0m  (desc)
   - Read 5: 995.0m  (desc) → EXPECTED: Apogee detected
4. Verify state transitions to APOGEE
5. Verify event logged with timestamp

ACCEPTANCE: Apogee detected on exactly the 5th consecutive reading

TEST SCENARIO B: Apogee Reset on Ascent (False Positive Prevention)
Goal: Verify counter resets if rocket climbs during COAST

STEPS:
1. Start at 1000m max altitude, COAST state
2. Send altitude sequence:
   - Read 1: 999.0m  (desc)
   - Read 2: 998.0m  (desc)
   - Command: "FORCE_ASCENT" (inject 1001m)
   - Counter should reset
   - Read 3: 1000.5m (desc from new max)
   - Read 4: 1000.0m (desc)
   - ... (continue for 5 total from reset)
3. Verify apogee triggers after 5 readings from reset point

ACCEPTANCE: Counter resets on ascent, requires new 5-count sequence

TEST SCENARIO C: Backup Timer Failsafe
Goal: Verify apogee triggers via timer if barometer fails

SETUP:
- Disable barometric sensor in firmware temporarily
- Or set initial altitude to indicate sensor failure

STEPS:
1. Force system to BOOST state with motor_end time = 0
2. Force transition to COAST (sets boostEndTime)
3. Wait BACKUP_APOGEE_TIME_MS (20 seconds default)
4. Verify state auto-transitions to APOGEE

ACCEPTANCE: Apogee occurs at BOOST_END + BACKUP_APOGEE_TIME_MS even without barometer

TEST SCENARIO D: Multi-Sensor Apogee Confirmation
Goal: Verify GPS and accel redundancy

STEPS:
1. Inject GPS descending altitude (consecutive 3 readings)
2. Verify GPS apogee detection flag sets
3. Inject accel Z <-0.1g for 5 consecutive readings
4. Verify accel apogee detection flag sets
5. Verify system uses any sensor that confirms apogee

ACCEPTANCE: Apogee confirmed via any sensor method, not just barometer

CLEANUP: Remove temporary apogee test commands
LOGGING: Log all apogee detection triggers with sensor source to debug output

CRITICAL SAFETY NOTE:
- These tests MUST use simulated altitude, never real rocket flight
- Verify pyro channels remain SAFE during all non-DEPLOY states
- If hardware is armed with igniters, test with inert charges only
```

**Test Phase 4: Landing Detection Validation**

```
PROCEDURE: LANDING DETECTION BENCH TEST
Duration: 15 minutes
Equipment: Flight computer + serial monitor + altitude simulator
Pre-requisites: Phase 3 passed

GOAL: Verify robust landing detection with no false positives

TEST SCENARIO A: Stable Ground Landing
Steps:
1. Force MAIN_DESCENT state (simulates parachute descent)
2. Set altitude to ground level (0m AGL)
3. Inject 10 consecutive stable readings (~0.0m, altitude change <0.1m)
4. Monitor landing detection timer
5. After LANDING_CONFIRMATION_TIME_MS (2000ms), verify transition to LANDED

ACCEPTANCE: Lands after sustained stability, no jitter

TEST SCENARIO B: Wind Gusts (False Positive Prevention)
Steps:
1. Start descent at 50m AGL
2. Inject sequence: 49.5, 49.0, 49.5, 48.8, 48.9, 48.7, 48.6m
   (simulates wind buffeting)
3. Verify landing detection doesn't trigger
4. Continue until stable for 2 seconds

ACCEPTANCE: Wind gusts don't trigger landing prematurely

TEST SCENARIO C: Accidental Sensor Glitch
Steps:
1. MAIN_DESCENT state at 10m AGL
2. Inject altitude spike: -5m (sensor glitch)
3. Verify landing counter resets
4. Continue with normal descent
5. Verify lands normally after stability resumes

ACCEPTANCE: Single glitch doesn't cause landing, stability check resets

CLEANUP: Remove temporary altitude injection commands
LOG: Record landing detection triggers and sensor state
```

**Test Phase 5: Guidance System Bench Test (If ENABLE_GUIDANCE == 1)**

```
PROCEDURE: GUIDANCE & CONTROL SYSTEM BENCH TEST
Duration: 30 minutes
Equipment: Flight computer + servo load tester + oscilloscope (optional)
Pre-requisites: Phase 1 passed, Guidance enabled in config.h

SETUP:
- Connect dummy load resistors to servo outputs (don't connect real servos)
- Oscilloscope on servo output pins if available
- Serial debug monitor active

TEST SCENARIO A: PID Controller Initialization
Steps:
1. Boot system, observe startup
2. Send "guidance_init" command
3. Verify PID state resets (zero integrals)
4. Command: "get_pid_status"
5. Verify output shows zero integrals, zero errors

ACCEPTANCE: PID system initializes cleanly

TEST SCENARIO B: Servo Output Linearity
Steps:
1. Force COAST state (guidance active)
2. Set target attitude: 0°, 0°, 0° (level)
3. Inject simulated attitude errors: 5°, 10°, 20°, 45°
4. Monitor servo pulse widths via oscilloscope or debug output
5. Verify servo command increases proportionally with error

EXPECTED: Servo command ≈ Kp * error (for small errors)

ACCEPTANCE: Servo output linear across error range

TEST SCENARIO C: Stability Monitoring
Steps:
1. Configure aggressive test with high rotation rates
2. Force attitude hold in COAST with initial 45° roll error
3. Monitor stability flags via "get_guidance_status"
4. Verify high angular rate detection works
5. Verify stability compromised flag sets after threshold

ACCEPTANCE: Stability violations logged and flagged correctly

CLEANUP: Remove test commands
LOG: Record servo linearity and stability metrics
```

**Test Phase 6: Data Logging Validation**

```
PROCEDURE: DATA LOGGING BENCH TEST
Duration: 10 minutes
Equipment: Flight computer + SD card + serial monitor
Pre-requisites: SD card inserted, formatted FAT32

STEPS:
1. Boot system, initialize SD card
2. Command: "logging_start"
3. Run system through multiple states (manual force commands)
4. Inject various sensor data
5. Command: "logging_stop"
6. Extract SD card, read log file on PC

VERIFICATION:
- Log file created with correct timestamp filename
- CSV header matches data_structures.h LogData struct
- All 89 columns present (seqNum through max_yaw_att_err_deg_so_far)
- Column alignment correct (no truncation)
- Timestamps increment monotonically
- Sensor values in reasonable ranges
- State values match expected state machine states

ACCEPTANCE CRITERIA:
- Log file is valid CSV, readable in Excel/Python
- All rows contain complete data (no truncation)
- Column count matches expected
- Timestamps valid and increasing
- No NaN or inf values in numeric columns
- Flight state values correspond to state enum
```

**Test Phase 7: Command Processor Validation**

```
PROCEDURE: SERIAL COMMAND PROCESSOR TEST
Duration: 15 minutes
Equipment: Flight computer + serial monitor
Pre-requisites: Phases 1-3 passed

GOAL: Verify all serial commands execute correctly and safely

TEST EACH COMMAND:
1. help               → Print help message
2. status            → Print system status (sensors, state, battery)
3. status_sensors    → Detailed sensor health report
4. calibrate         → Barometer calibration (only in PAD_IDLE)
5. arm               → Transition to ARMED (only in PAD_IDLE)
6. clear_errors      → Reset error state to PAD_IDLE
7. imu_debug [0|1]   → Toggle IMU debug output
8. gps_debug [0|1]   → Toggle GPS debug output
9. sensor_debug [0|1] → Toggle sensor debug output
10. logging_start    → Start SD card logging
11. logging_stop     → Stop logging
12. orientation_filter [kalman|quaternion] → Set filter type

EDGE CASE TESTS:
- Send calibrate in BOOST state → verify rejection "not in safe state"
- Send arm in COAST state → verify rejection "already in flight"
- Send help with long command queue → verify non-blocking
- Spam serial port → verify buffer doesn't overflow
- Send partial command (no newline) → verify waits for newline
- Send invalid command → verify "unknown command" message

ACCEPTANCE: All commands work as documented, safety checks active
```

---

### 1.2 Flight Test Procedures (Hardware + Real Environment)

**IMPORTANT:** Only conduct flight tests after all bench tests pass and firmware stability is proven.

**Test Flight 1: Passive Flight (No Guidance)**

```
FLIGHT TEST CHECKLIST - PASSIVE FLIGHT
Mission Objective: Verify state machine, apogee detection, parachute deployment

PRE-FLIGHT:
- Perform complete bench tests (Phases 1-7)
- Load flight firmware from stable branch
- Verify SD card inserted and logging enabled
- Simulate flight sequence 3x with temporary code
- Visual inspection of all solder joints
- Continuity check on pyro channels (both channels separately)
- Barometer pre-flight calibration at pad altitude
- GPS lock acquired (3D fix) before armed

LAUNCH SEQUENCE:
1. Power on system at pad (motor not loaded)
2. Verify PAD_IDLE state (LED steady green)
3. Wait for GPS 3D fix (if enabled, may take 30-90 sec)
4. Send "arm" command → verify ARMED state (LED yellow)
5. Load motor and igniters (FINAL SAFETY CHECK by Range Officer)
6. Igniter continuity check by Range Officer
7. Launch on pad signal

FLIGHT MONITORING:
- Monitor LED color changes (visual confirmation of state transitions)
- Monitor serial output if wireless telemetry available
- Observe rocket flight (visual tracking, altitude estimation)

EXPECTED SEQUENCE:
- ARMED state with ignition
- Liftoff → BOOST state (LED magenta)
- Motor burnout → COAST state (LED cyan)
- Apogee reached → APOGEE state (LED white)
- Drogue deploys → LED turns red (DROGUE_DEPLOY)
- Descent under drogue → LED dark red (DROGUE_DESCENT)
- Main altitude reached (100m AGL) → LED turns red again (MAIN_DEPLOY)
- Descent under main → LED dim
- Landing → LED changes

RECOVERY:
1. Locate rocket post-flight
2. Note LED state (should be LANDED or RECOVERY)
3. Connect to serial port, extract log file from SD card
4. Verify flight log contains complete flight event

PASS/FAIL CRITERIA:
- ✓ Rocket reached apogee without unintended parachute deployment
- ✓ Drogue deployed at apogee
- ✓ Main deployed at ~100m AGL (or configured altitude)
- ✓ Flight log contains complete, coherent data
- ✓ All state transitions occurred in correct order
- ✓ No sensor health errors during flight
- ✓ GPS altitude tracks barometric altitude within ±30m

ANALYSIS POST-FLIGHT:
- Extract CSV log, plot altitude, attitude, acceleration
- Verify apogee detection method (baro/accel/GPS/timer)
- Check for any sensor dropouts or anomalies
- Review guidance data if guidance enabled
- Store flight log in archive with date/time
```

**Test Flight 2: Guidance Enabled Flight (If guidance is production-ready)**

```
FLIGHT TEST CHECKLIST - GUIDED FLIGHT
PREREQUISITE: At least 2 successful passive flights, guidance bench tests passed

PRE-FLIGHT: (All passive flight checks PLUS)
- Verify servos connected to flight computer (dummy loads, not real servos)
- Verify servo pulse widths output correctly via oscilloscope or bench test
- Verify servo travel limits set correctly in config.h
- Load guidance trajectory (hardcoded test trajectory)
- Verify PID gains appropriate for vehicle

FLIGHT OBJECTIVES:
- Maintain level attitude in COAST phase
- Demonstrate servo actuation in response to rocket dynamics
- Verify no instability or oscillation
- Verify graceful degradation if guidance fails

EXPECTED BEHAVIOR:
- BOOST: Servos passive (high-g accel switchover to KX134)
- COAST: Servos active, attempting to maintain 0° roll/pitch
- If attitude error >30°: Stability check triggers, log warning
- If servo saturates: Log flag, continue with degraded performance
- APOGEE onward: Guidance disables, passive descent

PASS/FAIL:
- ✓ Rocket reaches apogee
- ✓ Attitude during COAST bounded within ±45°
- ✓ No servo chatter or oscillation
- ✓ Servos respond smoothly to disturbances
- ✓ System degrades gracefully if guidance encounters error

POST-FLIGHT ANALYSIS:
- Plot attitude setpoint vs actual
- Verify servo commands track control law
- Analyze stability margin, damping
- Identify any tuning needs for next flight
```

---

## 2. Hardware-Independent Testing Strategy

### 2.1 Unit Test Architecture

**Challenge:** 864+ hardware calls (Wire.beginTransmission, digitalWrite, etc.) deeply embedded throughout codebase.

**Solution:** Hardware Abstraction Layer (HAL) + Dependency Injection + Mock/Stub framework

#### 2.1.1 Current Testing Infrastructure

```
Current State:
- Framework: Unity + ArduinoFake
- Test Count: 2 test files (test_core_logic, test_flight_logic)
- Coverage: ~5% (Kalman filter, apogee detection mocks)
- Build: PlatformIO native environment (non-hardware)

Entry Point: platformio.ini [env:native] section
Commands:
  pio test -e native                  # Run all native tests
  pio test -e native -f test_core     # Run specific test
```

#### 2.1.2 Proposed HAL Architecture

**Design Pattern:** Three-layer abstraction

```
Layer 1: HARDWARE ABSTRACTION LAYER (HAL)
┌─────────────────────────────────────────────────┐
│ hal_i2c.h, hal_gpio.h, hal_timer.h, hal_serial.h │
│ - Pure virtual C++ interfaces                    │
│ - No Arduino dependencies                        │
│ - One .h file per hardware subsystem             │
└─────────────────────────────────────────────────┘
          ↓ Implements
Layer 2: CONCRETE IMPLEMENTATIONS
┌─────────────────────────────────────────────────┐
│ hal_i2c_teensy.cpp    (Real Teensy hardware)   │
│ hal_i2c_mock.cpp      (Mock for testing)        │
│ hal_gpio_teensy.cpp   (Real GPIO)               │
│ hal_gpio_mock.cpp     (Mock GPIO)               │
│ hal_timer_teensy.cpp  (Real millis/micros)      │
│ hal_timer_mock.cpp    (Simulated time)          │
│ hal_serial_teensy.cpp (Real UART)               │
│ hal_serial_mock.cpp   (String buffer mock)      │
└─────────────────────────────────────────────────┘
          ↓ Used by
Layer 3: BUSINESS LOGIC
┌─────────────────────────────────────────────────┐
│ flight_logic.cpp, state_management.cpp           │
│ guidance_control.cpp, sensor drivers             │
│ - Depends only on HAL interfaces, not hardware   │
│ - Testable without hardware                      │
│ - Swappable implementations                      │
└─────────────────────────────────────────────────┘
```

**Example HAL Header (hal_i2c.h):**

```cpp
#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <cstdint>

class I2CInterface {
public:
    virtual ~I2CInterface() = default;

    // Hardware abstraction for I2C
    virtual bool beginTransmission(uint8_t address) = 0;
    virtual bool write(uint8_t byte) = 0;
    virtual uint8_t endTransmission() = 0;
    virtual uint8_t requestFrom(uint8_t addr, uint8_t len) = 0;
    virtual uint8_t read() = 0;
    virtual bool isConnected(uint8_t address) = 0;
};

// Global HAL instance - can be swapped between real and mock
extern I2CInterface* g_i2c_hal;

#endif // HAL_I2C_H
```

**Example Mock Implementation (hal_i2c_mock.cpp):**

```cpp
#include "hal_i2c.h"
#include <vector>
#include <queue>

class I2CMock : public I2CInterface {
private:
    uint8_t current_address = 0;
    std::vector<uint8_t> write_buffer;
    std::queue<uint8_t> read_queue;
    std::vector<std::vector<uint8_t>> device_responses; // Per-address responses

public:
    bool beginTransmission(uint8_t address) override {
        current_address = address;
        write_buffer.clear();
        return true; // Mock always succeeds
    }

    bool write(uint8_t byte) override {
        write_buffer.push_back(byte);
        return true;
    }

    uint8_t endTransmission() override {
        // Mock: return 0 for success
        return 0;
    }

    // ... other methods

    // Test helpers (not in real HAL)
    void setDeviceResponse(uint8_t address, const std::vector<uint8_t>& response) {
        device_responses[address] = response;
    }

    std::vector<uint8_t> getLastWrite() const {
        return write_buffer;
    }
};
```

#### 2.1.3 Dependency Injection Strategy

**Goal:** Allow flight_logic.cpp to work with real hardware OR mocks

**Approach: Constructor Injection**

```cpp
// Before (tightly coupled):
void ProcessFlightState() {
    Wire.beginTransmission(0x68); // Hard-coded, untestable
    // ...
}

// After (dependency injection):
class FlightLogic {
private:
    I2CInterface* i2c_hal;
    BarometerInterface* baro;
    IMUInterface* imu;

public:
    FlightLogic(I2CInterface* i2c,
                BarometerInterface* baro_in,
                IMUInterface* imu_in)
        : i2c_hal(i2c), baro(baro_in), imu(imu_in) {}

    void ProcessFlightState() {
        // Use injected dependencies, testable!
        imu->read();
        baro->getAltitude();
    }
};

// In tests:
I2CMock i2c_mock;
BarometerMock baro_mock;
IMUMock imu_mock;
FlightLogic logic(&i2c_mock, &baro_mock, &imu_mock);

baro_mock.setAltitude(1000.0f); // Inject test data
logic.ProcessFlightState();      // Deterministic test execution
```

**Transition Plan (Low-Risk Approach):**

1. **Phase 1:** Create HAL headers (non-breaking)
2. **Phase 2:** Implement real HAL layer (wraps Arduino calls)
3. **Phase 3:** Create mock implementations
4. **Phase 4:** Gradually refactor modules to use HAL (one at a time)
5. **Phase 5:** Enable dependency injection in tests

---

### 2.2 Unit Test Strategy by Component

#### 2.2.1 Kalman Filter Tests (Highest Priority)

**Current Status:** 2 basic tests exist (initialization, simple predict)

**Expand to:**

```
Test Suite: kalman_filter_tests.cpp
Tests needed: 25+ cases

1. INITIALIZATION (3 tests)
   - test_kalman_init_zero_state
   - test_kalman_init_with_initial_values
   - test_kalman_init_convergence_time

2. PREDICT STEP (8 tests)
   - test_predict_identity_rotation (zero rate → state unchanged)
   - test_predict_constant_rate_roll
   - test_predict_constant_rate_pitch
   - test_predict_constant_rate_yaw
   - test_predict_combined_rates (XYZ rotation)
   - test_predict_numerical_stability (large dt)
   - test_predict_integration_accuracy (vs analytical solution)
   - test_predict_quaternion_normalization

3. UPDATE STEP (8 tests)
   - test_update_accelerometer_only (gravity alignment)
   - test_update_magnetometer_only (yaw alignment)
   - test_update_accel_mag_fusion (full 9-axis)
   - test_update_accel_gyro_fusion
   - test_update_noisy_accel (filter noise rejection)
   - test_update_strong_magnetic_disturbance
   - test_update_dynamic_motion (high accel, moving platform)
   - test_update_gimbal_lock_avoidance (90° pitch)

4. BOUNDARY CONDITIONS (4 tests)
   - test_kalman_extreme_roll_180deg
   - test_kalman_extreme_pitch_90deg
   - test_kalman_rapid_yaw_spin
   - test_kalman_fast_sensor_update_rate

5. EULER ANGLE CONVERSION (2 tests)
   - test_quaternion_to_euler_accuracy
   - test_euler_gimbal_lock_handling

Success Criteria:
- All tests pass with reference implementation
- Root mean square error <0.1° in converged state
- Quaternions remain normalized (magnitude 1.0)
- Handles 45° per second rotation rates smoothly
```

**Test Implementation Example:**

```cpp
void test_kalman_predict_constant_roll_rate(void) {
    // Given: Kalman filter initialized
    kalman_init(0.0f, 0.0f, 0.0f); // Start level

    // When: Apply constant roll rate (1 rad/s) for 1 second
    float roll_rate = 1.0f; // rad/s
    float dt = 0.01f;       // 10ms timestep (100 Hz update)

    for (int i = 0; i < 100; i++) {
        kalman_predict(roll_rate, 0.0f, 0.0f, dt);
    }

    // Then: Roll angle should be ~1.0 radian after 1 second
    float roll, pitch, yaw;
    kalman_get_orientation(roll, pitch, yaw);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, roll); // ±0.01 rad tolerance
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, yaw);
}
```

#### 2.2.2 Flight Logic Tests

**Current Status:** Apogee detection mocked, basic tests only

**Expand to:**

```
Test Suite: flight_logic_tests.cpp
Tests needed: 40+ cases

1. APOGEE DETECTION (10 tests) ← Currently ~2 tests
   - test_apogee_baro_descent_threshold
   - test_apogee_accel_negative_g
   - test_apogee_gps_altitude_descent
   - test_apogee_timer_fallback
   - test_apogee_multi_sensor_confirmation
   - test_apogee_false_positive_ascent
   - test_apogee_noise_rejection (sensor noise)
   - test_apogee_sensor_dropout_recovery
   - test_apogee_state_transition
   - test_apogee_boundary_cases (exactly at threshold)

2. LANDING DETECTION (8 tests)
   - test_landing_velocity_low
   - test_landing_accel_near_1g
   - test_landing_altitude_stable
   - test_landing_vertical_alignment
   - test_landing_false_positive_wind_gust
   - test_landing_sensor_dropout
   - test_landing_state_transition
   - test_landing_timeout_ms

3. BOOST END DETECTION (5 tests)
   - test_boost_end_accel_threshold
   - test_boost_end_integration_window
   - test_boost_end_state_transition
   - test_boost_end_false_positive_vibration
   - test_boost_end_timing_accuracy

4. STATE TRANSITIONS (12 tests)
   - test_transition_startup_to_calibration
   - test_transition_calibration_to_pad_idle
   - test_transition_pad_idle_to_armed (with arm command)
   - test_transition_armed_to_boost (liftoff detection)
   - test_transition_boost_to_coast (burnout)
   - test_transition_coast_to_apogee (detection)
   - test_transition_apogee_to_drogue_deploy
   - test_transition_drogue_to_drogue_descent
   - test_transition_drogue_descent_to_main_deploy (altitude trigger)
   - test_transition_main_deploy_to_main_descent
   - test_transition_main_descent_to_landed
   - test_transition_landed_to_recovery

5. ERROR STATE HANDLING (5 tests)
   - test_error_transition_on_sensor_failure
   - test_error_recovery_via_command
   - test_error_timeout_recovery
   - test_error_state_safety_locks (pyro disabled)
   - test_error_and_armed_state_conflict

Success Criteria:
- All state transitions tested with valid inputs
- Invalid transitions rejected safely
- No unintended state changes
- Pyro safety locks active during all unsafe states
```

#### 2.2.3 Guidance Control Tests

**Current Status:** No tests exist

**Create:**

```
Test Suite: guidance_control_tests.cpp
Tests needed: 35+ cases

1. PID CONTROLLER BASICS (10 tests)
   - test_pid_proportional_response
   - test_pid_integral_accumulation
   - test_pid_derivative_damping
   - test_pid_integral_anti_windup
   - test_pid_error_sign_handling
   - test_pid_zero_error_zero_output
   - test_pid_steady_state_accuracy
   - test_pid_stability_margin
   - test_pid_output_saturation
   - test_pid_reset_on_mode_change

2. ATTITUDE CONTROL (10 tests)
   - test_attitude_hold_level_flight
   - test_attitude_hold_roll_control
   - test_attitude_hold_pitch_control
   - test_attitude_hold_combined_axes
   - test_attitude_decoupling (roll vs pitch independence)
   - test_attitude_response_to_disturbance
   - test_attitude_gyro_feedback
   - test_attitude_convergence_time
   - test_attitude_overshoot
   - test_attitude_settling_time

3. SERVO OUTPUT (5 tests)
   - test_servo_output_range (-1 to +1)
   - test_servo_output_linearity
   - test_servo_output_deadband
   - test_servo_output_rate_limiting
   - test_servo_output_failsafe

4. STABILITY MONITORING (6 tests)
   - test_stability_rate_threshold_violation
   - test_stability_attitude_error_threshold
   - test_stability_saturation_detection
   - test_stability_timer_accumulation
   - test_stability_flag_setting
   - test_stability_reset

5. TRAJECTORY FOLLOWING (4 tests) [Future: when trajectory loading implemented]
   - test_waypoint_navigation
   - test_cross_track_error_calculation
   - test_bearing_to_waypoint
   - test_altitude_hold_to_waypoint

Success Criteria:
- PID loop stable for all test cases
- Attitude error decays exponentially
- No oscillation or instability
- Servo commands within physical limits
- Disturbance rejection verified
```

#### 2.2.4 State Management Tests

**Current Status:** No automated tests

**Create:**

```
Test Suite: state_management_tests.cpp
Tests needed: 15+ cases

1. EEPROM PERSISTENCE (6 tests)
   - test_save_state_to_eeprom
   - test_load_state_from_eeprom
   - test_state_persistence_power_loss
   - test_corrupt_eeprom_recovery
   - test_eeprom_write_endurance (1000+ writes)
   - test_eeprom_crc_validation

2. STATE MACHINE INTEGRITY (5 tests)
   - test_valid_state_enum_values
   - test_state_transition_table_completeness
   - test_state_timeout_handling
   - test_state_entry_action_execution
   - test_state_exit_action_execution

3. ERROR RECOVERY (4 tests)
   - test_power_loss_recovery_to_last_state
   - test_corrupted_state_recovery
   - test_recovery_timeout
   - test_recovery_sensor_validation

Success Criteria:
- State persists correctly across power cycles (with EEPROM mock)
- Recovery logic activates on boot after power loss
- No state corruption or loss
```

#### 2.2.5 Sensor Fusion Tests

**Current Status:** No tests

**Create:**

```
Test Suite: sensor_fusion_tests.cpp
Tests needed: 20+ cases

1. MULTI-SENSOR APOGEE (5 tests)
   - test_apogee_baro_confidence_level
   - test_apogee_accel_confidence_level
   - test_apogee_gps_confidence_level
   - test_apogee_multi_sensor_voting
   - test_apogee_sensor_weighting

2. ALTITUDE ESTIMATE (5 tests)
   - test_altitude_fusion_baro_primary
   - test_altitude_fusion_gps_secondary
   - test_altitude_fusion_gps_baro_disagreement
   - test_altitude_estimate_uncertainty
   - test_altitude_rate_validation

3. VELOCITY ESTIMATION (5 tests)
   - test_velocity_from_gps
   - test_velocity_from_altitude_rate
   - test_velocity_fusion_disagreement
   - test_velocity_zero_on_ground
   - test_velocity_during_descent

4. ACCELEROMETER SWITCHING (5 tests)
   - test_accel_switch_to_kx134_on_high_g
   - test_accel_switch_back_to_icm_on_low_g
   - test_accel_hysteresis (avoid chatter)
   - test_accel_noise_floor
   - test_accel_calibration_persistence

Success Criteria:
- Sensor fusion produces smooth, physically plausible estimates
- Multi-sensor voting prevents false detections
- Sensor transitions smooth (no step changes)
- Estimates bounded within realistic ranges
```

#### 2.2.6 Data Logging Tests

**Current Status:** No tests

**Create:**

```
Test Suite: logging_tests.cpp
Tests needed: 12+ cases

1. LOG FILE CREATION (4 tests)
   - test_create_new_log_file_name
   - test_log_file_header_format
   - test_log_file_csv_column_count
   - test_log_file_column_order

2. DATA SERIALIZATION (4 tests)
   - test_serialize_flight_state
   - test_serialize_sensor_data
   - test_serialize_attitude_data
   - test_serialize_guidance_data

3. LOG FILE INTEGRITY (4 tests)
   - test_log_sync_on_state_change
   - test_log_sequence_number_increment
   - test_log_timestamp_monotonic
   - test_log_recovery_after_flash_write

Success Criteria:
- Log files valid CSV format
- All columns present and in correct order
- No truncation or corruption
- Timestamps valid and increasing
- All data types match schema
```

---

### 2.3 Simulation & Integration Tests

#### 2.3.1 Flight Simulation Framework

**Goal:** Simulate complete flight without hardware

**Implementation:**

```cpp
// FlightSimulator - generates synthetic sensor data for complete flight scenario

class FlightSimulator {
private:
    // Simulated trajectory model
    float simulated_time_ms = 0;
    float altitude_m = 0;
    float velocity_mps = 0;
    float acceleration_mps2 = 0;
    float attitude_roll_rad = 0;
    float attitude_pitch_rad = 0;
    float attitude_yaw_rad = 0;

    // Motor model
    bool motor_burning = false;
    unsigned long boost_end_time_ms = 0;

public:
    // Initialize simulator with flight parameters
    void init(float motor_thrust_n, float motor_burntime_s,
              float rocket_dry_mass_kg, float parachute_cd);

    // Advance simulation by dt
    void step(float dt_s);

    // Get current sensor readings (what flight computer would see)
    SensorReading getSensorData(SensorType type);

    // Inject disturbance (wind, tumble)
    void applyDisturbance(DisturbanceType type, float magnitude);

    // Get truth state (for verification)
    FlightState getTrueState();
};
```

**Simulation Test Example:**

```cpp
void test_complete_flight_mission(void) {
    FlightSimulator sim;
    sim.init(
        5000.0f,  // Motor thrust (N)
        3.5f,     // Burntime (s)
        2.5f,     // Dry mass (kg)
        0.75f     // Parachute drag coefficient
    );

    // Inject flight computer with same mock sensors
    I2CMock i2c_mock;
    BarometerMock baro_mock;
    IMUMock imu_mock;
    FlightLogic logic(&i2c_mock, &baro_mock, &imu_mock);

    // Run simulation
    float dt = 0.01f; // 10ms timesteps, 100 Hz
    unsigned long total_sim_time = 300000; // 5 minute flight

    for (unsigned long t = 0; t < total_sim_time; t += (unsigned long)(dt * 1000)) {
        // Advance simulation
        sim.step(dt);

        // Inject sensor data to flight computer
        SensorReading alt_reading = sim.getSensorData(ALTITUDE);
        baro_mock.setAltitude(alt_reading.altitude_m);

        SensorReading imu_reading = sim.getSensorData(ACCELERATION);
        imu_mock.setAccel(imu_reading.accel_mps2);

        // Run one iteration of flight logic
        logic.ProcessFlightState();

        // Verify flight computer behavior
        if (t > 3500 && t < 4500) {
            // BOOST phase - should be armed and see high acceleration
            TEST_ASSERT(logic.getCurrentState() == BOOST);
            TEST_ASSERT(logic.getAcceleration() > 5.0f); // >5g boost
        }

        if (t > 4000 && t < 5000) {
            // COAST phase - should detect apogee soon
            TEST_ASSERT(logic.getCurrentState() == COAST);
            TEST_ASSERT(alt_reading.altitude_m > 1000.0f); // Over 1km
        }

        if (t > 5000) {
            // Apogee should have occurred
            TEST_ASSERT(logic.getCurrentState() >= APOGEE);
            if (logic.getCurrentState() == APOGEE) {
                TEST_ASSERT(alt_reading.altitude_m < 100.0f ||
                           alt_reading.altitude_m > sim.getApogeeAltitude() - 20.0f);
            }
        }
    }

    // Final verification
    TEST_ASSERT(logic.getCurrentState() == LANDED ||
               logic.getCurrentState() == RECOVERY);
}
```

#### 2.3.2 Scenario-Based Tests

```
Test Set: End-to-end flight scenarios

SCENARIO 1: Nominal Flight
- Motor thrust nominal, optimal conditions
- Expected: Clean apogee detection, both parachutes deploy
- Verification: State transitions in correct order, apogee ±5% actual

SCENARIO 2: High Cross-Winds
- Inject simulated wind throughout flight
- Expected: Rocket tumbles, guidance system (if enabled) attempts stabilization
- Verification: Attitude oscillations within limits, no guidance saturation

SCENARIO 3: Early Motor Burnout
- Motor thrust drops at t=2s (vs nominal 3.5s)
- Expected: Shorter boost phase, lower apogee, no drogue premature deploy
- Verification: State transitions remain clean, apogee triggers on descent

SCENARIO 4: Sensor Dropout - Barometer Fails
- Barometer stops reporting at t=5s (mid-COAST)
- Expected: Backup timer triggers apogee if barometer fails
- Verification: Apogee detection via timer within 2% accuracy

SCENARIO 5: Sensor Dropout - GPS Fails
- GPS loses fix at t=8s (during COAST)
- Expected: Flight continues on baro/accel apogee
- Verification: No effect on apogee detection

SCENARIO 6: High-G Accelerometer Switchover
- Acceleration exceeds ICM range during boost
- Expected: Switch to KX134 for accel data
- Verification: Acceleration data continues without dropout

SCENARIO 7: Extreme Tumble
- Rocket enters tumble at apogee (unstable reentry)
- Expected: Large attitude errors, guidance (if enabled) attempts recovery
- Verification: Drogue still deploys at DROGUE_DEPLOY state

SCENARIO 8: Wind Gust on Landing
- Simulated wind gust 1m/s at landing
- Expected: Landing detection still robust
- Verification: No false landing trigger, lands when settled
```

---

## 3. Refactoring Plan for Testability

### 3.1 Minimum Viable Refactoring (Low Risk, High Impact)

**Goal:** Enable 40%+ test coverage with <20% code churn

**Phase 1: Create HAL Layer (2 weeks)**

Files to create:
```
src/hal/hal_i2c.h              [200 lines] - I2C abstraction
src/hal/hal_i2c_teensy.cpp     [150 lines] - Teensy Wire wrapper
src/hal/hal_i2c_mock.cpp       [200 lines] - Mock for testing
src/hal/hal_timer.h            [100 lines] - millis/micros abstraction
src/hal/hal_timer_teensy.cpp   [100 lines] - Real timer
src/hal/hal_timer_mock.cpp     [80 lines]  - Simulated time
src/hal/hal_gpio.h             [100 lines] - GPIO abstraction
src/hal/hal_gpio_teensy.cpp    [100 lines] - Real GPIO
src/hal/hal_gpio_mock.cpp      [80 lines]  - Mock GPIO

Total new code: ~1100 lines (non-critical path, helpers)
Risk: VERY LOW (no changes to existing code)
```

**Phase 2: Refactor Sensor Drivers (3 weeks)**

Target: ms5611_functions, icm_20948_functions, kx134_functions

Changes:
- Replace Wire.beginTransmission() calls with i2c_hal->beginTransmission()
- Replace digitalWrite() with hal_gpio->set()
- Replace millis() with hal_timer->millis()

```
Changes per file:
  ms5611_functions.cpp:   ~20 Wire → i2c_hal replacements
  icm_20948_functions.cpp: ~40 Wire → i2c_hal replacements
  kx134_functions.cpp:    ~30 Wire → i2c_hal replacements

Risk: LOW (isolated to driver files, well-defined changes)
Verification: Existing hardware tests must still pass
```

**Phase 3: Refactor Flight Logic (2 weeks)**

Target: flight_logic.cpp, state_management.cpp, guidance_control.cpp

Changes:
- Inject HAL dependencies via global HAL pointers (safe, backward compatible)
- Add optional dependency injection hooks for testing
- No changes to function signatures (backward compatible)

```cpp
// After refactoring, still works same way:
void ProcessFlightState() {
    g_i2c_hal->read(...); // Uses global HAL (can be real or mock)
}

// But can also inject in tests:
class ProcessFlightStateTestable {
    I2CInterface* i2c_hal; // Dependency
    void ProcessFlightState() {
        i2c_hal->read(...); // Testable!
    }
};
```

Risk: LOW (backward compatible changes, no API changes)
Timeline: 2 weeks
Effort: 1 senior developer

**Phase 4: Implement Mock Objects (2 weeks)**

Create reusable mock implementations:

```cpp
src/test/mocks/mock_i2c.h
src/test/mocks/mock_barometer.h
src/test/mocks/mock_imu.h
src/test/mocks/mock_gps.h
src/test/mocks/mock_timer.h
src/test/mocks/sensor_data_generator.h  // Helper to create test scenarios
```

**Phase 5: Create Test Suite (4 weeks)**

Write 80+ unit tests:

```
test/unit/kalman_filter_tests.cpp        [30 tests]
test/unit/flight_logic_tests.cpp         [40 tests]
test/unit/guidance_control_tests.cpp     [20 tests]
test/unit/state_management_tests.cpp     [15 tests]
test/unit/sensor_fusion_tests.cpp        [20 tests]
test/unit/logging_tests.cpp              [12 tests]

Total: ~137 test cases covering core logic
```

**Total Refactoring Effort:**
- Calendar time: 10-12 weeks (can be parallelized)
- Developer effort: 8 weeks senior dev + 4 weeks junior dev
- Risk: LOW (backward compatible, incremental)
- Coverage improvement: 5% → 40%+

---

### 3.2 Full Refactoring Plan (Medium-Term, 6-12 months)

**Phase 6: Dependency Injection for All Modules (4 weeks)**

Refactor to pass HAL dependencies explicitly:

```cpp
// Before: Global HAL accessed via extern
class FlightLogic {
    void ProcessFlightState() {
        g_i2c_hal->read(...); // Global
    }
};

// After: Dependencies injected
class FlightLogic {
    I2CInterface* i2c;
    TimerInterface* timer;

    FlightLogic(I2CInterface* i2c_in, TimerInterface* timer_in)
        : i2c(i2c_in), timer(timer_in) {}

    void ProcessFlightState() {
        i2c->read(...); // Explicit dependency
    }
};
```

Benefits:
- Explicit dependency clarity
- Easier testing (no global state)
- Enables multiple instances for simulation

Risk: MEDIUM (function signature changes)
Timeline: 4 weeks
Verification: All bench tests + flight tests must pass

**Phase 7: Extract Sensor Interfaces (3 weeks)**

Create abstract sensor interfaces:

```cpp
class BarometerInterface {
    virtual float getAltitude() = 0;
    virtual float getPressure() = 0;
    virtual float getTemperature() = 0;
    virtual bool isHealthy() = 0;
};

class IMUInterface {
    virtual void getAccel(float out[3]) = 0;
    virtual void getGyro(float out[3]) = 0;
    virtual void getMag(float out[3]) = 0;
    virtual bool isHealthy() = 0;
};
```

Allows:
- Easy sensor swapping (different barometer models)
- Testing with synthetic sensors
- Simulating sensor failures

Risk: LOW (interfaces only, implementation unchanged)
Timeline: 3 weeks

**Phase 8: Full Simulation Framework (4 weeks)**

Implement complete flight simulator with physics:

```cpp
class FlightSimulator {
private:
    // Rigid body dynamics
    PhysicsModel rocket_model;
    WindModel wind_model;
    GravityModel gravity;

public:
    // Provides sensor data as if real sensors
    float getBarometerAltitude(float time_ms);
    void getIMUAccel(float time_ms, float out[3]);
    // ...
};
```

Enables:
- Testing all flight scenarios in software
- Hardware-free development
- Edge case injection (sensor failures, extreme conditions)

Risk: MEDIUM (requires physics model validation)
Timeline: 4 weeks
Validation: Must match bench test measurements within ±5%

**Phase 9: CI/CD Integration (2 weeks)**

Implement continuous integration:

```yaml
# .github/workflows/test.yml
name: Test Suite

on: [push, pull_request]

jobs:
  unit-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Build and Test (native)
        run: pio test -e native
      - name: Code Coverage
        run: gcov test/unit/*.cpp --coverage
      - name: Upload Coverage
        uses: codecov/codecov-action@v2

  firmware-build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Build Teensy firmware
        run: pio run -e teensy41
      - name: Check binary size
        run: |
          SIZE=$(stat -f%z .pio/build/teensy41/firmware.elf)
          if [ $SIZE -gt $((200*1024)) ]; then exit 1; fi
```

Timeline: 2 weeks
Result: Automated test + build on every commit

**Total Full Refactoring:**
- Calendar time: 6-9 months (with parallelization)
- Developer effort: 15-20 weeks
- Coverage improvement: 40% → 70-80%
- Result: Production-grade testability

---

## 4. Test Coverage Strategy

### 4.1 Coverage Targets by Component

**Safety-Critical (Target: 85%+ coverage)**
- Flight state machine transitions: 85%
- Apogee detection (all methods): 90%
- Landing detection: 85%
- Pyro firing logic: 95% (CRITICAL)
- Error recovery: 85%

**High-Risk (Target: 70%+ coverage)**
- Kalman filter: 75%
- Guidance PID control: 70%
- Sensor fusion: 75%
- Data logging: 70%

**Medium-Risk (Target: 50%+ coverage)**
- Command processor: 60%
- Sensor drivers: 55%
- GPS integration: 50%
- State management: 60%

**Low-Priority (Target: 30%+ coverage)**
- LED control: 30%
- Debug output: 30%
- NeoPixel animations: 30%

### 4.2 Highest-Priority Code to Test First

**Phase 1 Priority (Weeks 1-4): Test these first**

1. **Apogee Detection** (flight_logic.cpp: ~150 lines)
   - Why: Most critical safety function, complex logic
   - Test count: 15+ tests
   - Current coverage: ~10%
   - Target: 90%

2. **Kalman Filter** (kalman_filter.cpp: ~300 lines)
   - Why: Core sensor fusion, affects all guidance
   - Test count: 25+ tests
   - Current coverage: ~5%
   - Target: 75%

3. **State Machine Transitions** (flight_logic.cpp: ~400 lines)
   - Why: Controls all flight phases
   - Test count: 15+ tests
   - Current coverage: <1%
   - Target: 85%

4. **Landing Detection** (flight_logic.cpp: ~100 lines)
   - Why: Safety-critical, prevents parachute separation
   - Test count: 10+ tests
   - Current coverage: <1%
   - Target: 85%

**Phase 2 Priority (Weeks 5-12): Follow with**

5. **Guidance PID Control** (guidance_control.cpp: ~250 lines)
6. **Pyro Firing Logic** (flight_logic.cpp: ~50 lines, but critical)
7. **Sensor Fusion** (multiple files: ~200 lines)
8. **Data Logging** (log_format_definition.cpp + TripleT_Flight.cpp: ~150 lines)

### 4.3 Test Case Inventory for Critical Paths

**Apogee Detection Critical Path:**

```
Test Cases (15 total):
1. Basic baro descent detection
2. Multi-sensor voting (baro + accel + GPS)
3. Timer fallback (sensor failure)
4. False positive prevention (wind gust, momentum)
5. Boundary conditions (exactly at threshold)
6. Rapid altitude changes (noise rejection)
7. Sensor dropout recovery
8. State transition on apogee
9. Multiple apogee candidates (ignore after first)
10. Extreme altitude testing
11. Integration with guidance system
12. Logging of apogee event
13. Error state on all sensors failed
14. Hysteresis in descent detection
15. Performance under 100Hz+ update rate

Code Coverage Target: 90% (34 of 37 lines)
```

**Pyro Firing Critical Path:**

```
Test Cases (12 total):
1. Only fire in DROGUE_DEPLOY state
2. Only fire in MAIN_DEPLOY state
3. Duration exactly PYRO_FIRE_DURATION ms
4. Non-blocking operation
5. Can't fire while in ERROR state
6. Can't fire while DISARMED
7. Don't fire on false state transitions
8. LED confirms firing state
9. Recovery state can't fire pyro
10. Both channels fire independently
11. Prevent refiring same channel
12. Watchdog prevents stuck pyro

Code Coverage Target: 95% (19 of 20 lines in firing logic)
```

### 4.4 Regression Test Suite

**Essential Tests (Must Run on Every Commit)**

```
SMOKE TESTS (5 minutes to run):
- Firmware compiles without warnings
- All 3 apogee detection methods work
- Landing detection robust to noise
- State machine doesn't crash on invalid state
- All serial commands execute

CRITICAL PATH TESTS (10 minutes):
- Complete flight simulation (nominal scenario)
- Sensor failure recovery (each sensor)
- Pyro firing doesn't interfere with main loop
- Kalman filter numerical stability

BOUNDARY CONDITION TESTS (20 minutes):
- Extreme altitudes (0m to 10000m)
- Extreme attitudes (gimbal lock)
- Extreme accelerations (0.5g to 50g)
- Fast update rates (100+ Hz)
- Slow update rates (<1 Hz)
```

**Nightly Full Test Suite (1-2 hours)**

```
- All 137 unit tests
- All 8 integration scenarios
- Code coverage report generated
- Performance benchmarks (execution time, RAM usage)
- Memory leak detection (if enabled)
- Static analysis (if enabled)
```

---

## 5. Testing Tools & Infrastructure

### 5.1 Testing Framework Recommendations

**Current:** Unity + ArduinoFake (good start, expandable)

**Enhancements:**

```
Framework Stack:
├── Unity (existing) - Core test framework
├── ArduinoFake (existing) - Arduino emulation
├── CMake (add) - Build system for native tests
├── GoogleTest / gtest (evaluate) - More features than Unity
├── Catch2 (evaluate) - Modern C++ test framework
├── Valgrind (add) - Memory leak detection
├── gcov (add) - Code coverage analysis
└── Clang static analyzer (add) - Static analysis
```

**Recommended Test Setup:**

```
Testing Framework: Unity (keep for compatibility)
Assertion Library: Unity built-in + custom helpers
Mock Library: Custom HAL-based mocks (simpler than fancy libraries)
Build System: CMake 3.15+ (cross-platform)
Coverage: gcov / lcov
CI/CD: GitHub Actions (free, integrated)
```

### 5.2 CI/CD Pipeline Design

**GitHub Actions Configuration**

```yaml
# .github/workflows/test.yml
name: Automated Tests & Code Quality

on:
  push:
    branches: [master, beta-*, develop]
  pull_request:
    branches: [master, develop]

jobs:
  unit-tests:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        platform: [native, native-coverage]
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
        with:
          python-version: '3.10'

      - name: Install dependencies
        run: |
          pip install platformio
          apt-get update && apt-get install -y \
            cmake clang gcov lcov valgrind

      - name: Build and Run Unit Tests
        run: pio test -e native -v

      - name: Generate Coverage Report
        run: |
          lcov -d .pio/build/native -c -o coverage.info
          lcov -r coverage.info '/usr/*' -o coverage.info
          lcov -l coverage.info

      - name: Upload Coverage to Codecov
        uses: codecov/codecov-action@v3
        with:
          files: ./coverage.info
          fail_ci_if_error: false

  firmware-build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
        with:
          python-version: '3.10'

      - name: Install PlatformIO
        run: pip install platformio

      - name: Build Teensy firmware
        run: pio run -e teensy41

      - name: Check firmware size
        run: |
          SIZE=$(stat -c%s .pio/build/teensy41/firmware.elf)
          LIMIT=$((220*1024))  # 220KB limit
          if [ $SIZE -gt $LIMIT ]; then
            echo "Firmware too large: $SIZE > $LIMIT"
            exit 1
          fi
          echo "Firmware size: $SIZE bytes"

  static-analysis:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Run clang-tidy
        run: |
          apt-get update && apt-get install -y clang-tidy
          clang-tidy src/*.cpp -- -Isrc -I.pio/libdeps/teensy41/**/include

  performance:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Run performance benchmarks
        run: pio test -e native -f test_performance

  report:
    needs: [unit-tests, firmware-build, static-analysis]
    runs-on: ubuntu-latest
    if: always()
    steps:
      - name: Create test report
        run: |
          echo "Test Results Summary" > summary.txt
          echo "===================" >> summary.txt
          echo "Unit Tests: $([ ${{ needs.unit-tests.result }} == 'success' ] && echo PASS || echo FAIL)" >> summary.txt
          echo "Build: $([ ${{ needs.firmware-build.result }} == 'success' ] && echo PASS || echo FAIL)" >> summary.txt
          echo "Analysis: $([ ${{ needs.static-analysis.result }} == 'success' ] && echo PASS || echo FAIL)" >> summary.txt

      - name: Upload summary
        uses: actions/upload-artifact@v3
        with:
          name: test-summary
          path: summary.txt
```

### 5.3 Test Data Management

**Test Data Organization:**

```
test/data/
├── sensor_recordings/          # Real flight data for replay testing
│   ├── flight_001_nominal.csv
│   ├── flight_002_high_winds.csv
│   └── flight_003_sensor_dropout.csv
├── expected_outputs/           # Golden reference data
│   ├── nominal_flight_log.csv
│   ├── apogee_detection_output.json
│   └── guidance_control_response.json
├── edge_cases/                 # Stress test data
│   ├── extreme_attitudes.json
│   ├── extreme_accelerations.json
│   └── fast_sensor_updates.json
└── README.md                   # Data documentation
```

**Test Data Format:**

```
For sensor playback tests, use JSON:
{
  "test_name": "nominal_flight",
  "description": "Typical flight with good weather",
  "duration_ms": 300000,
  "updates": [
    {
      "time_ms": 0,
      "barometer_altitude_m": 0,
      "imu_accel_g": [0.0, 0.0, 1.0],
      "imu_gyro_dps": [0.0, 0.0, 0.0],
      "gps_altitude_m": 0,
      "expected_state": "PAD_IDLE"
    },
    ...
  ]
}
```

### 5.4 Code Coverage Measurement

**Coverage Goals:**

```
Overall Target: 60% coverage in Phase 1, 75%+ in Phase 2

By component:
- flight_logic.cpp:        85% (safety-critical)
- kalman_filter.cpp:       75% (core algorithm)
- guidance_control.cpp:    70% (control system)
- state_management.cpp:    70% (state machine)
- ms5611_functions.cpp:    55% (hardware dependent)
- icm_20948_functions.cpp: 55% (hardware dependent)
- command_processor.cpp:   60% (I/O, medium priority)
- gps_functions.cpp:       50% (external module)
- log_format_definition.cpp: 70% (data handling)
- kx134_functions.cpp:     50% (optional sensor)
```

**Coverage Tools:**

```
Primary: gcov (built into GCC)
Analysis: lcov (creates HTML reports)
CI Integration: Codecov.io (free for public repos)

Setup:
1. Add -fprofile-arcs -ftest-coverage to CFLAGS
2. Run tests
3. Generate coverage with: lcov -d . -c -o coverage.info
4. View HTML: genhtml coverage.info -o html && open html/index.html
5. Upload to Codecov: curl -s https://codecov.io/bash | bash
```

### 5.5 Test Reporting & Metrics

**Weekly Report Template:**

```markdown
# Test Report - Week of YYYY-MM-DD

## Summary
- Unit Tests: 134/137 passing (97.8%)
- Coverage: 45% (target: 50%)
- Firmware Build: PASS
- Static Analysis: 3 warnings (decrease from 5)

## Test Results by Category
- Flight Logic: 28/30 PASS (93%)
- Kalman Filter: 18/18 PASS (100%)
- Guidance Control: 12/15 PASS (80%)
- State Management: 10/10 PASS (100%)
- Sensor Fusion: 14/20 PASS (70%)

## Failing Tests
1. test_guidance_saturation_recovery - Expected 0.95, got 0.87
   - Investigation: PID integral not resetting properly
   - Action: Fix integral anti-windup logic in guidance_control.cpp

2. test_landing_wind_gust_robustness - False landing detected
   - Investigation: Landing detector too sensitive to noise
   - Action: Increase LANDING_CONFIRMATION_COUNT threshold

## Coverage Gaps
- command_processor.cpp: 40% (was 35%) ← improving
- gps_functions.cpp: 30% (was 30%) ← hard to test without hardware
- kx134_functions.cpp: 35% (was 35%) ← optional sensor, lower priority

## Performance Metrics
- Average test runtime: 45ms per test
- Total test suite time: 6.2 seconds
- RAM usage during tests: 12MB peak
- Firmware binary size: 198KB (of 220KB limit)

## Next Week Goals
- Add 5 more landing detection edge cases
- Refactor guidance PID anti-windup (fix failing test)
- Achieve 48%+ coverage
```

---

## 6. Implementation Roadmap (6-Month Plan)

### Month 1: Foundation & Planning
**Week 1-2: Preparation**
- [x] Complete this testing strategy document
- [ ] Set up CI/CD pipeline (GitHub Actions)
- [ ] Create test/mocks directory structure
- [ ] Establish coding standards for tests

**Week 3-4: Create HAL Layer**
- [ ] Design and implement hal_i2c.h/.cpp
- [ ] Design and implement hal_timer.h/.cpp
- [ ] Create mock implementations
- [ ] Verify backwards compatibility with hardware

**Target: 5% → 10% coverage**

### Month 2: Unit Tests - Core Logic
**Week 5-6: Kalman Filter Tests**
- [ ] Create 25+ Kalman filter tests
- [ ] Verify numerical accuracy
- [ ] Test boundary conditions (gimbal lock, extreme rates)

**Week 7-8: Apogee Detection Tests**
- [ ] Create 15+ apogee detection tests
- [ ] Test all 4 detection methods (baro, accel, GPS, timer)
- [ ] Test multi-sensor voting
- [ ] Test false positive prevention

**Target: 10% → 25% coverage**

### Month 3: Unit Tests - State Machine
**Week 9-10: State Transition Tests**
- [ ] Create 15+ state transition tests
- [ ] Verify all valid transitions
- [ ] Verify invalid transitions are blocked
- [ ] Test error recovery paths

**Week 11-12: Landing Detection Tests**
- [ ] Create 10+ landing detection tests
- [ ] Test robustness to wind/noise
- [ ] Test state transitions

**Target: 25% → 35% coverage**

### Month 4: Integration Tests
**Week 13-14: Flight Simulation**
- [ ] Implement basic FlightSimulator class
- [ ] Create 8 scenario-based tests
- [ ] Verify simulation accuracy vs bench tests
- [ ] Document simulation assumptions

**Week 15-16: Guidance Control Tests**
- [ ] Create 20+ guidance control tests
- [ ] Test PID loops
- [ ] Test stability monitoring
- [ ] Test servo output linearity

**Target: 35% → 50% coverage**

### Month 5: Advanced Testing
**Week 17-18: Sensor Fusion Tests**
- [ ] Create multi-sensor apogee tests
- [ ] Test altitude estimation fusion
- [ ] Test accelerometer switchover
- [ ] Test sensor dropout recovery

**Week 19-20: Data Logging Tests**
- [ ] Create log file validation tests
- [ ] Test CSV format correctness
- [ ] Test data integrity
- [ ] Test flash write performance

**Target: 50% → 60% coverage**

### Month 6: Refinement & Documentation
**Week 21-22: Bench Testing**
- [ ] Execute all 7 bench test procedures
- [ ] Document results
- [ ] Identify gaps
- [ ] Plan production test procedures

**Week 23-24: Documentation & Handover**
- [ ] Create test procedure manual for field teams
- [ ] Create test data archive
- [ ] Create CI/CD documentation
- [ ] Train team on running tests

**Target: 60% coverage, production-ready test suite**

---

## 7. Risk Mitigation

### 7.1 Risks & Mitigation Strategies

| Risk | Impact | Mitigation |
|------|--------|-----------|
| Testing finds new bugs near flight date | HIGH | Start bench testing early, parallel tracks |
| Refactoring introduces regressions | HIGH | All existing hardware tests must pass, incremental |
| CI/CD pipeline false failures | MEDIUM | Quarantine flaky tests, increase timeout |
| Test data doesn't match reality | MEDIUM | Cross-validate with actual flight data |
| Coverage metric gaming (100% line coverage, low path coverage) | MEDIUM | Use condition/path coverage in addition to line |
| Sensor mocks too simplified, miss real behavior | MEDIUM | Replay real flight data in tests, validate assumptions |
| Performance testing on CI slower than dev machine | LOW | Use consistent hardware for benchmarks, accept variance |

### 7.2 Quality Gates (Before Flight)

**Mandatory Requirements:**

```
Before ANY flight test:
- [ ] 100% of apogee detection tests passing
- [ ] 100% of pyro firing logic tests passing
- [ ] 100% of state machine critical paths passing
- [ ] 100% of error recovery tests passing
- [ ] All 7 bench test procedures completed successfully
- [ ] Code review sign-off from 2 senior developers
- [ ] No high-severity static analysis warnings
- [ ] Firmware binary size <220KB
- [ ] All serial commands working correctly

Before NEXT production flight:
- [ ] Complete flight simulation shows recovery sequence
- [ ] Guidance system (if enabled) stability verified
- [ ] Landing detection robust to all test scenarios
- [ ] Data logging tested on actual hardware
- [ ] Post-flight log analysis completed for previous flight
- [ ] All hardware sensor health checks pass
```

---

## 8. Resource Requirements

### 8.1 Staffing

**Phase 1 (Months 1-3): Foundation & Core Tests**
- 1 Senior Developer (testing architecture, Kalman/apogee tests): 20 hrs/week
- 1 Junior Developer (state machine, landing tests): 20 hrs/week
- 1 Test Engineer (bench procedures, CI/CD setup): 15 hrs/week

**Phase 2 (Months 4-6): Integration & Production**
- 1 Senior Developer (guidance tests, simulation): 15 hrs/week
- 1 Junior Developer (sensor fusion, logging tests): 20 hrs/week
- 1 Test Engineer (flight tests, documentation): 20 hrs/week

**Total Effort:**
- Senior Dev: 8 weeks
- Junior Dev: 8 weeks
- Test Engineer: 8 weeks
- Total person-weeks: 24 weeks (6 people × 4 weeks, or 2-3 people × 8 weeks)

### 8.2 Equipment Costs

**Bench Testing Setup:**
- Teensy 4.1 Flight Computer: $35
- Sensor Stack (ICM, MS5611, KX134, GPS): $150
- Servo Load Tester: $50
- Oscilloscope (Hantek 1008C 8-channel): $300
- Power Supply + Current limiting: $100
- SD cards + adapters: $50
- Miscellaneous (cables, connectors): $50

**Total:** ~$735 (one-time equipment)

**Software Tools:**
- GitHub Actions: FREE (free tier public repos)
- Codecov: FREE (free tier public repos)
- Valgrind: FREE (open source)
- CMake: FREE (open source)
- Unity + ArduinoFake: FREE (already in use)

**Total Software:** $0 (for public project)

### 8.3 Timeline & Constraints

**Critical Path:**
- HAL layer creation → Sensor driver refactoring → Unit test writing
- Apogee tests must complete before any flight test
- Simulation framework needed before guidance tests

**Parallel Work Possible:**
- CI/CD setup (parallel to HAL creation)
- Bench test procedure documentation (parallel to unit tests)
- Flight simulation physics model (can start early)

**Dependency Graph:**
```
HAL Creation (2 weeks)
  ↓
Sensor Driver Refactoring (3 weeks)
  ├→ Apogee Tests (2 weeks, must finish before flight)
  ├→ State Machine Tests (3 weeks)
  ├→ Kalman Filter Tests (2 weeks, parallel to above)
  └→ Guidance Tests (3 weeks, after HAL ready)

Simulation Framework (4 weeks, starts at week 5)
  ├→ Scenario-based Tests (2 weeks)
  └→ Integration Tests (2 weeks)

Bench Test Procedures (3 weeks, parallel to all above)

CI/CD Setup (2 weeks, parallel to HAL)
```

**Total Calendar Time:** 10-12 weeks (compressed schedule with parallelization)

---

## 9. Success Metrics

### 9.1 Quantitative Metrics

```
COVERAGE METRICS (by phase):
Phase 1 (Week 12): 25% code coverage
Phase 2 (Week 24): 60% code coverage
Target (Flight ready): 60%+ overall, 85%+ safety-critical

TEST PASS RATE:
Phase 1: 90% of tests passing
Phase 2: 98%+ of tests passing
Phase 3: 100% of critical path tests passing

DEFECT DETECTION:
Phase 1: Identify 5-10 bugs via new tests
Phase 2: Identify 2-5 bugs via integration tests
Phase 3: 0 bugs found in field (goal)

EXECUTION TIME:
Unit tests: <10 seconds for all 137 tests
CI/CD full suite: <5 minutes
Nightly full tests: <30 minutes
```

### 9.2 Qualitative Metrics

```
CONFIDENCE METRICS:
- Senior dev confidence in flight readiness: 4/5 → 5/5
- Team familiarity with test suite: 2/5 → 4/5
- Ability to catch regressions: 1/5 → 4/5

PROCESS METRICS:
- % of code changes covered by tests: 10% → 80%
- Regression detection: Manual → Automated
- Flight readiness verification: Ad-hoc → Systematic

OPERATIONAL METRICS:
- Time to debug new flight anomaly: 2 hours → 30 minutes
- False test failures: N/A → <5%
- Test maintenance burden: Low → Medium (acceptable)
```

---

## 10. Next Steps & Recommendations

### Immediate Actions (This Week)

1. **Review & Approve Strategy**
   - [ ] Review with flight team
   - [ ] Identify any modifications needed
   - [ ] Get sign-off from project lead

2. **Set Up CI/CD Pipeline**
   - [ ] Create .github/workflows/test.yml
   - [ ] Enable GitHub Actions
   - [ ] Configure Codecov integration

3. **Create HAL Layer Stub**
   - [ ] Create src/hal/ directory
   - [ ] Add hal_i2c.h (interface only)
   - [ ] Add hal_i2c_teensy.cpp (wrapper)
   - [ ] Add hal_i2c_mock.cpp (mock)

### Short-Term (Weeks 1-4)

4. **Complete HAL Implementation**
   - [ ] Finish hal_timer, hal_gpio stubs
   - [ ] Verify no breaking changes to existing code
   - [ ] Add unit tests for HAL layer itself

5. **Start Apogee Detection Tests**
   - [ ] Create test_apogee_detection.cpp
   - [ ] Implement 15+ test cases
   - [ ] Achieve 85%+ coverage on apogee detection

6. **Bench Test Prep**
   - [ ] Gather all required equipment
   - [ ] Document Phase 1 bench test procedure
   - [ ] Schedule first bench test session

### Dependencies & Blockers

```
What blocks the refactoring?
- ✓ Nothing critical (can start immediately)
- ? Decision: Refactor in place or create parallel test-friendly code?
  Recommendation: Refactor in place (less code duplication, single truth)

What blocks bench testing?
- Need soldered Teensy 4.1 with all sensors
- Recommend: Complete electronics assembly first
- Estimated time: 2-4 weeks if not already done

What blocks flight testing?
- All Phase 1-3 bench tests must pass
- Motor casing and parachute system ready
- Launch site available and range safety officer trained
```

---

## Conclusion

This comprehensive testing strategy provides a **structured, low-risk pathway** to achieve production-grade test coverage while maintaining backward compatibility with existing hardware operations.

**Key Takeaways:**

1. **Achievable Goals:** 60% coverage in 6 months with 2-3 developers, starting from 5%

2. **Low Risk:** Incremental refactoring with HAL layer requires <1100 lines of new infrastructure code, maintains backward compatibility

3. **Safety-First:** Apogee detection, landing detection, and pyro firing logic can be 85%+ tested without hardware using mocks

4. **Automation Ready:** CI/CD pipeline eliminates manual regression testing, catches issues early

5. **Flight-Ready:** Bench test procedures provide systematic pre-flight verification, reducing launch-day surprises

**Recommended Start:** Begin with HAL layer creation + Kalman filter tests this week. Can be flying with comprehensive test coverage within 12 weeks.

---

**Document Version:** 1.0
**Last Updated:** 2025-02-14
**Status:** Ready for Implementation
**Next Review:** After Month 1 completion (2025-03-14)
