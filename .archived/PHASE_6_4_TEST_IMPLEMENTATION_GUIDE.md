# Phase 6.4: Test Implementation Guide

**Companion to:** `PHASE_6_4_TESTING_FRAMEWORK.md`
**Purpose:** Detailed code examples and step-by-step implementation
**Status:** Ready for implementation

---

## Table of Contents

1. [Quick Start: Create First Test](#quick-start-create-first-test)
2. [Mock Sensor Examples](#mock-sensor-examples)
3. [Test Fixture Examples](#test-fixture-examples)
4. [Full Flight Simulator Code](#full-flight-simulator-code)
5. [Regression Test Template](#regression-test-template)
6. [HIL Test Procedures](#hil-test-procedures)

---

## Quick Start: Create First Test

### Step 1: Create Test File Structure

```bash
# Create new unit test file
touch /mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware/test/unit/test_my_feature.cpp
```

### Step 2: Template for New Test File

**File:** `test/unit/test_my_feature.cpp`

```cpp
#include <unity.h>
#include <stdio.h>

// Include what you're testing
#include "../../src/flight_logic.h"
#include "../../src/data_structures.h"
#include "../mocks/mock_sensors.h"

// ============================================================================
// TEST SETUP/TEARDOWN
// ============================================================================

void setUp(void) {
  // Called before EACH test
  // Reset all global state
  reset_flight_state_to_startup();
  clear_telemetry_buffer();
  clear_sensor_cache();
}

void tearDown(void) {
  // Called after EACH test
  // Cleanup resources (if needed)
}

// ============================================================================
// TEST CASES
// ============================================================================

void test_feature_nominal_case(void) {
  // Arrange: Set up initial conditions
  MockIMUSensor sensor;
  sensor.setSimulationMode(SIMULATION_NOMINAL);

  // Act: Do the thing
  bool result = my_function(&sensor);

  // Assert: Verify expectations
  TEST_ASSERT_TRUE(result);
  TEST_ASSERT_EQUAL(EXPECTED_STATE, get_flight_state());
}

void test_feature_edge_case_1(void) {
  // Test boundary condition
  TEST_ASSERT_EQUAL(0, my_function(NULL));
}

void test_feature_edge_case_2(void) {
  // Test error condition
  MockIMUSensor sensor;
  sensor.setHealthy(false);

  bool result = my_function(&sensor);

  TEST_ASSERT_FALSE(result);
}

// ============================================================================
// TEST RUNNER
// ============================================================================

int main(void) {
  UNITY_BEGIN();

  // Register tests
  RUN_TEST(test_feature_nominal_case);
  RUN_TEST(test_feature_edge_case_1);
  RUN_TEST(test_feature_edge_case_2);

  return UNITY_END();
}
```

### Step 3: Add to Build Configuration

**File:** `test/platformio.ini` (existing - just reference)

```ini
[env:native_test]
platform = native
test_framework = unity
test_dir = test
```

### Step 4: Run the Test

```bash
cd /mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware
pio test -e native_test -f test_my_feature
```

---

## Mock Sensor Examples

### Create Basic Mock Sensor

**File:** `test/mocks/mock_sensors.h` (append to existing)

```cpp
#pragma once

#include <vector>
#include <cmath>

// ============================================================================
// MOCK IMU SENSOR
// ============================================================================

class MockIMUSensor {
private:
  float accel_x_ = 0.0f;
  float accel_y_ = 0.0f;
  float accel_z_ = 9.81f;  // Default: 1G down

  float gyro_x_ = 0.0f;
  float gyro_y_ = 0.0f;
  float gyro_z_ = 0.0f;

  float temp_c_ = 20.0f;
  bool healthy_ = true;

  enum SimMode {
    SIMULATION_CONSTANT_ACCEL,
    SIMULATION_APOGEE_TRANSITION,
    SIMULATION_HIGH_G,
    SIMULATION_NOMINAL,
  } sim_mode_ = SIMULATION_NOMINAL;

  int read_count_ = 0;

public:
  // === Configuration ===

  void setSimulationMode(SimMode mode) {
    sim_mode_ = mode;
    read_count_ = 0;
  }

  void setAcceleration(float ax, float ay, float az) {
    accel_x_ = ax;
    accel_y_ = ay;
    accel_z_ = az;
  }

  void setTemperature(float temp_c) {
    temp_c_ = temp_c;
  }

  void setHealthy(bool healthy) {
    healthy_ = healthy;
  }

  // === Simulation Behavior ===

  void update() {
    read_count_++;

    switch (sim_mode_) {
      case SIMULATION_CONSTANT_ACCEL:
        // Hold current acceleration
        break;

      case SIMULATION_APOGEE_TRANSITION: {
        // Simulate acceleration transition (positive → negative)
        // Ascent for first 50 reads, then transition to descent
        float progress = (float)read_count_ / 100.0f;
        if (progress < 0.5f) {
          accel_z_ = 15.0f;  // Ascending: 15 m/s² down relative to rocket frame
        } else {
          accel_z_ = -5.0f;  // Descending: rocket frame sees "up" acceleration
        }
        break;
      }

      case SIMULATION_HIGH_G: {
        // Simulate high-G boost event
        // Ramp up to 100G then drop
        float phase = (float)read_count_ / 50.0f;
        if (phase < 1.0f) {
          accel_z_ = 9.81f + (phase * 100.0f * 9.81f);
        } else {
          accel_z_ = 9.81f;
        }
        break;
      }

      case SIMULATION_NOMINAL:
        // No change
        break;
    }
  }

  // === Data Access ===

  float getAccelX() const { return accel_x_; }
  float getAccelY() const { return accel_y_; }
  float getAccelZ() const { return accel_z_; }

  float getAccelMagnitude() const {
    return std::sqrt(accel_x_*accel_x_ + accel_y_*accel_y_ + accel_z_*accel_z_);
  }

  float getGyroX() const { return gyro_x_; }
  float getGyroY() const { return gyro_y_; }
  float getGyroZ() const { return gyro_z_; }

  float getTemperature() const { return temp_c_; }

  bool isHealthy() const { return healthy_; }

  void getQuaternion(float& qw, float& qx, float& qy, float& qz) {
    // Return identity quaternion (no rotation)
    qw = 1.0f;
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;
  }
};

// ============================================================================
// MOCK BAROMETRIC SENSOR
// ============================================================================

class MockBarometer {
private:
  float pressure_pa_ = 101325.0f;  // Sea level
  float altitude_m_ = 0.0f;
  bool healthy_ = true;

public:
  void setAltitude(float altitude_m) {
    altitude_m_ = altitude_m;
    // ISA model: P = P0 * exp(-h / 8500)
    pressure_pa_ = 101325.0f * std::exp(-altitude_m_ / 8500.0f);
  }

  void setPressure(float pressure_pa) {
    pressure_pa_ = pressure_pa;
  }

  float getAltitude() const {
    return altitude_m_;
  }

  float getPressure() const {
    return pressure_pa_;
  }

  bool isHealthy() const { return healthy_; }

  void setHealthy(bool healthy) {
    healthy_ = healthy;
  }

  // Simulate ascent then descent
  void simulateAscentDescent(float max_altitude_m) {
    static float sim_time = 0.0f;
    sim_time += 0.01f;

    if (sim_time < 15.0f) {
      // Ascent phase (0-15s)
      setAltitude(max_altitude_m * (sim_time / 15.0f));
    } else {
      // Descent phase (15s+)
      setAltitude(max_altitude_m * (1.0f - ((sim_time - 15.0f) / 40.0f)));
    }
  }
};

// ============================================================================
// MOCK GPS RECEIVER
// ============================================================================

class MockGPS {
private:
  float latitude_ = 34.876500f;
  float longitude_ = -118.123400f;
  float altitude_m_ = 500.0f;
  float velocity_mps_ = 0.0f;

  bool has_fix_ = false;
  int fix_quality_ = 0;  // 0=none, 1=GPS, 2=DGPS, 3=RTK

  int fix_acquisition_time_ms_ = 3000;  // Time to first fix

public:
  void setPosition(float lat, float lon, float alt_m) {
    latitude_ = lat;
    longitude_ = lon;
    altitude_m_ = alt_m;
  }

  void setVelocity(float velocity_mps) {
    velocity_mps_ = velocity_mps;
  }

  void setFixQuality(int quality) {
    fix_quality_ = quality;
    has_fix_ = (quality >= 1);
  }

  void setFix(bool has_fix) {
    has_fix_ = has_fix;
    fix_quality_ = has_fix ? 3 : 0;
  }

  float getLatitude() const { return latitude_; }
  float getLongitude() const { return longitude_; }
  float getAltitude() const { return altitude_m_; }
  float getVelocity() const { return velocity_mps_; }

  bool hasFix() const { return has_fix_; }
  int getFixQuality() const { return fix_quality_; }

  void simulateApogeeSequence(float apogee_altitude_m) {
    // Show altitude rising then falling
    static float sim_time = 0.0f;
    sim_time += 0.01f;

    if (sim_time < 15.0f) {
      setPosition(34.876500f, -118.123400f, 500.0f + (apogee_altitude_m * sim_time / 15.0f));
    } else {
      setPosition(34.876500f, -118.123400f, apogee_altitude_m * (1.0f - ((sim_time - 15.0f) / 40.0f)));
    }
    setFix(true);
  }

  void waitForFix() {
    // Simulate acquisition delay
    uint32_t t_start = millis();
    while (!has_fix_ && (millis() - t_start) < fix_acquisition_time_ms_) {
      delay(100);
    }
    setFix(true);
  }
};

// ============================================================================
// MOCK SERVO ACTUATOR
// ============================================================================

class MockServo {
private:
  float angle_ = 0.0f;  // -45 to +45 degrees
  float prev_angle_ = 0.0f;
  uint32_t last_command_time_ms_ = 0;

  static const float MAX_ANGLE;
  static const float MIN_ANGLE;

public:
  void setAngle(float angle_deg) {
    prev_angle_ = angle_;

    // Saturate to physical limits
    angle_ = angle_deg;
    if (angle_ > MAX_ANGLE) angle_ = MAX_ANGLE;
    if (angle_ < MIN_ANGLE) angle_ = MIN_ANGLE;

    last_command_time_ms_ = millis();
  }

  float getAngle() const {
    return angle_;
  }

  float getPreviousAngle() const {
    return prev_angle_;
  }

  bool isSaturated() const {
    return (angle_ >= MAX_ANGLE || angle_ <= MIN_ANGLE);
  }

  uint32_t getLastCommandTime() const {
    return last_command_time_ms_;
  }
};

const float MockServo::MAX_ANGLE = 45.0f;
const float MockServo::MIN_ANGLE = -45.0f;

// ============================================================================
// MOCK SD CARD
// ============================================================================

class MockSDCard {
private:
  std::vector<uint8_t> buffer_;
  bool file_open_ = false;

public:
  bool openFile(const char* filename) {
    file_open_ = true;
    buffer_.clear();
    return true;
  }

  bool write(const uint8_t* data, size_t len) {
    if (!file_open_) return false;

    buffer_.insert(buffer_.end(), data, data + len);
    return true;
  }

  bool closeFile() {
    file_open_ = false;
    return true;
  }

  size_t getFileSize() const {
    return buffer_.size();
  }

  std::vector<uint8_t> getBuffer() const {
    return buffer_;
  }

  std::string getBufferAsString() const {
    return std::string(buffer_.begin(), buffer_.end());
  }
};
```

---

## Test Fixture Examples

### CSV Flight Data Fixture

**File:** `test/fixtures/recordings/nominal_flight_1250m.csv` (first 50 lines)

```csv
timestamp_ms,accel_x_mps2,accel_y_mps2,accel_z_mps2,gyro_x_dps,gyro_y_dps,gyro_z_dps,pressure_pa,temperature_c,gps_lat,gps_lon,gps_alt_m,gps_fix_quality
0,0.1,-0.2,9.81,0.0,0.0,0.0,101325.0,20.0,34.876500,-118.123400,500.0,3
10,2.5,0.3,45.2,0.5,0.3,-0.2,101324.8,20.1,34.876501,-118.123401,500.5,3
20,5.8,0.1,50.1,1.2,0.8,-0.5,101324.2,20.2,34.876502,-118.123402,501.2,3
30,8.2,-0.4,48.5,1.8,1.2,-0.8,101323.5,20.3,34.876503,-118.123403,502.1,3
40,10.5,0.2,52.3,2.5,1.5,-1.1,101322.8,20.4,34.876504,-118.123404,503.5,3
...
15000,0.3,-0.1,-8.2,0.2,-0.1,0.1,99542.0,19.8,34.876535,-118.123450,1250.5,3
15010,-2.1,0.3,-7.8,0.3,0.2,0.0,99535.2,19.7,34.876536,-118.123451,1249.8,3
15020,-1.8,0.0,-8.1,0.1,-0.1,0.0,99528.5,19.7,34.876537,-118.123452,1249.0,3
```

### Helper Function to Load Fixture

```cpp
// test/integration/test_full_flight_simulation.cpp

#include <vector>
#include <fstream>
#include <sstream>

struct FlightDataPoint {
  uint32_t timestamp_ms;
  float accel_x_mps2;
  float accel_y_mps2;
  float accel_z_mps2;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
  float pressure_pa;
  float temperature_c;
  float gps_lat;
  float gps_lon;
  float gps_alt_m;
  int gps_fix_quality;
};

std::vector<FlightDataPoint> loadFlightDataCSV(const char* filename) {
  std::vector<FlightDataPoint> data;
  std::ifstream file(filename);

  if (!file.is_open()) {
    fprintf(stderr, "Failed to open fixture file: %s\n", filename);
    return data;
  }

  std::string line;

  // Skip header
  std::getline(file, line);

  while (std::getline(file, line)) {
    if (line.empty()) continue;

    FlightDataPoint point = {};

    // Parse CSV line
    std::istringstream iss(line);
    std::string token;

    // Read each field
    iss >> token; point.timestamp_ms = std::stoi(token);
    iss.ignore();

    iss >> token; point.accel_x_mps2 = std::stof(token);
    iss.ignore();

    iss >> token; point.accel_y_mps2 = std::stof(token);
    iss.ignore();

    iss >> token; point.accel_z_mps2 = std::stof(token);
    iss.ignore();

    // ... continue for all fields

    data.push_back(point);
  }

  file.close();
  return data;
}
```

---

## Full Flight Simulator Code

### Complete Simulator Implementation

**File:** `test/integration/flight_simulator.h`

```cpp
#pragma once

#include <vector>
#include <cmath>
#include <cstdint>
#include <cstdio>

struct SimulationState {
  uint32_t current_time_ms;
  uint8_t flight_state;

  // Sensor readings
  float accel_x_mps2;
  float accel_y_mps2;
  float accel_z_mps2;
  float pressure_pa;
  float altitude_m;
  float gps_lat;
  float gps_lon;
  float gps_alt_m;

  // Outputs being monitored
  uint8_t pyro_drogue;
  uint8_t pyro_main;
  int16_t servo_pitch;
  int16_t servo_yaw;

  // Metrics
  float apogee_altitude_m;
  uint32_t apogee_time_ms;
  uint32_t drogue_fire_time_ms;
  uint32_t main_fire_time_ms;
};

class FullFlightSimulator {
private:
  SimulationState state_;
  std::vector<SimulationState> history_;

public:
  FullFlightSimulator() : state_({0}) {}

  void initialize() {
    state_.flight_state = STARTUP;
    state_.current_time_ms = 0;
    state_.altitude_m = 500.0f;  // Ground level (500m MSL)
    state_.pyro_drogue = 0;
    state_.pyro_main = 0;
    history_.clear();
  }

  void simulationStep(
    uint32_t timestamp_ms,
    float accel_z,
    float pressure_pa,
    float gps_alt_m
  ) {
    state_.current_time_ms = timestamp_ms;
    state_.accel_z_mps2 = accel_z;
    state_.pressure_pa = pressure_pa;
    state_.gps_alt_m = gps_alt_m;

    // Calculate altitude from pressure (simplified)
    state_.altitude_m = 44330.0f * (1.0f - std::pow(pressure_pa / 101325.0f, 1.0f / 5.255f));

    // Call flight logic update
    update_flight_logic(state_);

    // Record state
    history_.push_back(state_);
  }

  bool validateApogeeDetection(float expected_altitude_m, float tolerance_m) {
    // Find when state transitions to APOGEE
    uint32_t apogee_time = 0;
    float apogee_altitude = 0.0f;

    for (size_t i = 1; i < history_.size(); i++) {
      if (history_[i].flight_state == APOGEE &&
          history_[i-1].flight_state == COAST) {
        apogee_time = history_[i].current_time_ms;
        apogee_altitude = history_[i].altitude_m;
        break;
      }
    }

    if (apogee_time == 0) {
      printf("ERROR: Apogee not detected\n");
      return false;
    }

    printf("Apogee detected at t=%u ms, altitude=%.1f m\n",
           apogee_time, apogee_altitude);

    float error = std::fabs(apogee_altitude - expected_altitude_m);
    if (error > tolerance_m) {
      printf("ERROR: Altitude error %.1f m exceeds tolerance %.1f m\n",
             error, tolerance_m);
      return false;
    }

    state_.apogee_time_ms = apogee_time;
    state_.apogee_altitude_m = apogee_altitude;
    return true;
  }

  bool validateDeploymentTiming() {
    // Drogue should fire within 100ms of apogee
    uint32_t drogue_time = 0;
    for (const auto& s : history_) {
      if (s.pyro_drogue && !drogue_time) {
        drogue_time = s.current_time_ms;
        break;
      }
    }

    if (drogue_time == 0) {
      printf("ERROR: Drogue pyro not fired\n");
      return false;
    }

    uint32_t delay_ms = drogue_time - state_.apogee_time_ms;
    printf("Drogue fired at t=%u ms (delay=%u ms)\n", drogue_time, delay_ms);

    if (delay_ms > 150) {
      printf("WARNING: Drogue delay %u ms exceeds nominal 100ms\n", delay_ms);
    }

    state_.drogue_fire_time_ms = drogue_time;
    return true;
  }

  const SimulationState& getFinalState() const {
    return history_.back();
  }

  size_t getDataPointCount() const {
    return history_.size();
  }

  void printSummary() {
    printf("\n=== Flight Simulation Summary ===\n");
    printf("Total data points: %zu\n", history_.size());
    printf("Flight duration: %.1f s\n", state_.current_time_ms / 1000.0f);
    printf("Max altitude: %.1f m\n", state_.apogee_altitude_m);
    printf("Apogee time: %u ms\n", state_.apogee_time_ms);
    printf("Drogue fire: %u ms\n", state_.drogue_fire_time_ms);
    printf("Main fire: %u ms\n", state_.main_fire_time_ms);
    printf("=================================\n\n");
  }

private:
  void update_flight_logic(SimulationState& state) {
    // This would call the actual flight logic functions
    // For now, stub implementation
    // In real implementation, inject mock sensors and call flight_logic.cpp functions
  }
};
```

---

## Regression Test Template

### Standard Regression Test Structure

**File:** `test/regression/test_template.cpp`

```cpp
#include <unity.h>
#include "../../src/flight_logic.h"
#include "../mocks/mock_sensors.h"

/**
 * Test Category: [Feature Name]
 * Purpose: Verify [what this test validates]
 * Critical Path: [YES/NO]
 * Regression Risk: [HIGH/MEDIUM/LOW]
 *
 * Test Cases:
 * 1. Nominal behavior
 * 2. Edge case 1
 * 3. Edge case 2
 * 4. Error condition
 */

// ============================================================================
// SETUP/TEARDOWN
// ============================================================================

void setUp(void) {
  // Reset to clean state
  reset_all_systems();
}

void tearDown(void) {
  // Cleanup
}

// ============================================================================
// TEST 1: NOMINAL BEHAVIOR
// ============================================================================

void test_feature_nominal_behavior(void) {
  // Setup
  MockIMUSensor imu;
  imu.setSimulationMode(SIMULATION_NOMINAL);
  imu.setAcceleration(0.0f, 0.0f, 50.0f);  // 5G boost

  // Execute
  set_flight_state(ARMED);
  update_flight_logic(10);

  // Verify
  TEST_ASSERT_EQUAL(BOOST, get_flight_state());
  TEST_ASSERT_TRUE(imu.isHealthy());
}

// ============================================================================
// TEST 2: EDGE CASE - BOUNDARY VALUE
// ============================================================================

void test_feature_boundary_at_threshold(void) {
  // Test exactly at threshold (3.0G boost threshold)
  MockIMUSensor imu;
  imu.setAcceleration(0.0f, 0.0f, 9.81f + 3.0f * 9.81f);

  set_flight_state(ARMED);
  update_flight_logic(10);

  // Should transition to BOOST
  TEST_ASSERT_EQUAL(BOOST, get_flight_state());
}

// ============================================================================
// TEST 3: EDGE CASE - JUST BELOW THRESHOLD
// ============================================================================

void test_feature_just_below_threshold(void) {
  MockIMUSensor imu;
  imu.setAcceleration(0.0f, 0.0f, 9.81f + 2.9f * 9.81f);  // 2.9G

  set_flight_state(ARMED);
  update_flight_logic(10);

  // Should NOT transition to BOOST
  TEST_ASSERT_EQUAL(ARMED, get_flight_state());
}

// ============================================================================
// TEST 4: ERROR CONDITION - SENSOR FAILURE
// ============================================================================

void test_feature_sensor_failure_recovery(void) {
  MockIMUSensor imu;
  imu.setHealthy(false);

  set_flight_state(ARMED);
  update_flight_logic(10);

  // Should transition to ERROR
  TEST_ASSERT_EQUAL(ERROR, get_flight_state());

  // Manual recovery
  process_serial_command("clear_errors");
  TEST_ASSERT_EQUAL(PAD_IDLE, get_flight_state());
}

// ============================================================================
// TEST RUNNER
// ============================================================================

int main(void) {
  UNITY_BEGIN();

  RUN_TEST(test_feature_nominal_behavior);
  RUN_TEST(test_feature_boundary_at_threshold);
  RUN_TEST(test_feature_just_below_threshold);
  RUN_TEST(test_feature_sensor_failure_recovery);

  return UNITY_END();
}
```

---

## HIL Test Procedures

### Hardware-in-Loop Test Script

**File:** `test/hardware_in_loop/hil_test_procedures.md`

```markdown
# Hardware-in-Loop Test Procedures

## Equipment Setup

### Physical Connections

```
Teensy 4.1 DUT
├─ Serial TX → USB-to-Serial → Computer Terminal
├─ SDA/SCL → I2C Mock Device (function generator simulating sensor)
├─ Servo Out 1 → Oscilloscope CH1 (measure PWM timing)
├─ Servo Out 2 → Oscilloscope CH2 (measure PWM timing)
├─ Pyro Output 1 → LED + 330Ω resistor (visual indicator)
├─ Pyro Output 2 → LED + 330Ω resistor (visual indicator)
└─ GPS RX → GPS Simulator (or real GPS receiver for integration test)
```

### Oscilloscope Configuration

```
CH1 (Servo 1):
- Timebase: 10ms/div
- Vertical: 1V/div
- Trigger: Edge, Rising
- Expected: 1-2ms pulses at 50Hz

CH2 (Servo 2):
- Same as CH1

CH1+2 Math:
- Measure: Pulse width in microseconds
- Tolerance: ±50µs
```

## Test Procedure 1: Launch Detection

### Objective
Verify automatic transition from ARMED to BOOST on motor acceleration.

### Setup
```
1. Connect Teensy to computer via serial
2. Open serial terminal (115200 baud)
3. Connect function generator to I2C mock acceleration sensor
4. Program function generator: Ramp from 0V to 5V over 100ms
```

### Procedure
```
1. Type "arm" in serial terminal
   Expected output: "System ARMED, ready for launch"

2. Start function generator ramp
   Expected: Teensy detects acceleration rise

3. Observe serial output:
   t=0ms: State=ARMED
   t=50ms: State=BOOST
   Expected: Transition occurs within 100ms of accel threshold

4. Stop function generator
   Expected: Smooth descent to normal acceleration, state remains BOOST
```

### Verification Criteria
- [ ] Transition occurs within 50-100ms
- [ ] Serial output confirms BOOST state
- [ ] No false positives on vibration or noise
- [ ] Repeatable on 3+ trials
- [ ] LED indicator blinks (if connected to state indicator)

### Troubleshooting
```
If transition doesn't occur:
1. Check I2C communication (add serial debug output)
2. Verify accelerometer scale calibration
3. Check BOOST_ACCEL_THRESHOLD in config.h (should be ~2.0G)
4. Add printf debug to accel reading function
```

## Test Procedure 2: Apogee Detection

### Objective
Verify detection of peak altitude and drogue deployment trigger.

### Setup
```
1. Same as Test 1, but modify function generator
2. Program ramp sequence:
   - 0-200ms: Ramp 0V to 2.5V (simulate boost)
   - 200-1500ms: Ramp 2.5V to 2.8V over ~1.3s (simulate coast)
   - 1500-2000ms: Ramp 2.8V down to 0.5V (simulate descent)

3. Barometer I2C: Set altitude rising then falling at peak
```

### Procedure
```
1. Arm system: "arm"
2. Start function generator
3. Observe state transitions:
   t=100ms: BOOST → COAST
   t=1500ms: COAST → APOGEE
   t=1520ms: APOGEE → DROGUE_DEPLOY (pyro fires)

4. Monitor Oscilloscope: LED should illuminate at apogee
5. Serial output should show all state transitions with timestamps
```

### Verification Criteria
- [ ] Apogee detected at correct altitude (±50m)
- [ ] Drogue fires within 100ms of apogee
- [ ] Pyro pulse detected on oscilloscope (~50-100ms pulse)
- [ ] LED indicator lights when pyro fires
- [ ] Timing consistent across 3+ runs (±200ms variation max)

---

## Test Procedure 3: Servo Response

### Objective
Measure servo actuator response timing and accuracy.

### Setup
```
1. Connect servo to PWM output pin
2. Connect servo feedback potentiometer to ADC input
3. Connect oscilloscope to servo PWM output
4. No function generator needed for this test
```

### Procedure
```
1. Upload test firmware with servo test function:

   test_servo_command(servo_id, angle);
   // Should command servo to specified angle
   // Record PWM timing on oscilloscope

2. Test sequence:
   Angle  | Expected PWM | Measured | ±Tolerance | Result
   ------ | ------------ | -------- | ---------- | ------
   0°     | 1500µs       |          | ±50µs      | [ ]
   +45°   | 1950µs       |          | ±50µs      | [ ]
   -45°   | 1050µs       |          | ±50µs      | [ ]
   +22.5° | 1725µs       |          | ±50µs      | [ ]
   -22.5° | 1275µs       |          | ±50µs      | [ ]

3. Measure response time:
   - Send command
   - Note time on oscilloscope when PWM changes
   - Record latency

4. Test extreme angles:
   - Command +90° (beyond limits)
   - Verify servo saturates at +45°
   - Verify no fault or error condition
```

### Verification Criteria
- [ ] All PWM values within ±50µs tolerance
- [ ] Response time <250ms
- [ ] Smooth transitions between angles (no jitter)
- [ ] Servo center-able after test

---

## Test Procedure 4: Sensor Failover

### Objective
Verify automatic activation of backup sensor under high-G.

### Setup
```
1. Configure dual sensors (primary ICM-20948 + backup KX134)
2. Function generator A: Simulate normal ICM acceleration
3. Function generator B: Simulate high-G (>100G for KX134 activation)
4. Configure Teensy to monitor and report active sensor via serial
```

### Procedure
```
1. Run normal boost simulation (0-50G range)
   Expected: Primary ICM-20948 active
   Serial: "Sensor: ICM (primary)"

2. Suddenly increase to 150G (beyond ICM ±16G range)
   Expected: System detects saturation
   Serial: "WARNING: Accel saturation, switching to backup"
   Serial: "Sensor: KX134 (backup)"

3. Verify data continuity:
   - Acceleration values should transition smoothly
   - No data gaps or jumps
   - Backup provides valid readings >100G range

4. Reduce back to normal range
   Expected: Primary sensor re-activates
   Serial: "Sensor: ICM (primary)"
```

### Verification Criteria
- [ ] Failover occurs within 100ms
- [ ] No interruption to data stream
- [ ] Backup sensor (KX134) provides valid measurements
- [ ] Recovery to primary sensor seamless
- [ ] No false failovers during normal flight

---

## Test Procedure 5: Pyro Channel Continuity

### Objective
Verify pyro channel electrical integrity and deployment mechanism.

### Setup
```
1. Disconnect all explosives (safety first!)
2. Connect test load resistors (100Ω) to each pyro output
3. Connect LED + 330Ω in parallel with resistor (visual indicator)
4. Multimeter to measure current draw
5. Oscilloscope to measure pulse timing
```

### Procedure
```
1. Pre-flight continuity check:
   - Measure resistance of each channel
   - Expected: ~5-10Ω (including wiring)
   - Record baseline

2. Issue deployment command:
   "deploy_drogue"

   Expected behavior:
   - LED illuminates
   - Oscilloscope shows ~12V pulse
   - Pulse duration 50-100ms
   - Current draw 1-2A
   - Multimeter confirms current flow

3. Verify second pyro channel independently:
   "deploy_main"

   Expected: Same behavior as drogue

4. Test timing accuracy:
   - Command with known timestamp
   - Measure pulse start time on oscilloscope
   - Timing accuracy ±10ms
```

### Verification Criteria
- [ ] Both channels functional
- [ ] Continuity within specification
- [ ] Pulse timing accurate
- [ ] No crosstalk between channels (simultaneous command should fire both)
- [ ] No leakage current when idle

```

---

## Quick Reference: Common Issues & Fixes

| Issue | Root Cause | Fix |
|-------|-----------|-----|
| Apogee not detected | Barometer not connected | Check I2C lines, verify sensor on network |
| Servo doesn't respond | PWM output not connected | Verify pin configuration in config.h |
| False boost trigger | Vibration/noise | Increase acceleration threshold, add filtering |
| Pyro won't fire | Wiring or power | Check 12V supply, measure continuity |
| GPS not locking | Cold start, weak signal | Wait longer, check antenna, verify baud rate |

---

## End of Test Procedures

Estimated HIL testing time per cycle: 2-3 hours (5+ test procedures)
Recommended: Perform before each release cycle
