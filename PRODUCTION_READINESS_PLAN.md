# Phase 6.3: Production Readiness Implementation Plan

**Target Release:** v1.0.0-RC1
**Duration:** 1.5 weeks
**Scope:** 1,000 lines of code + comprehensive testing
**Status:** Planning Phase

---

## Executive Summary

Phase 6.3 focuses on making the TripleT flight computer production-ready through power optimization, edge case handling, pre-flight verification, thermal management, and comprehensive validation. This plan provides detailed specifications, implementation architecture, and integration points for all production readiness features.

**Key Objectives:**
- Reduce power consumption from 180mA (ACTIVE) to 2mA (SLEEP)
- Handle all identified edge cases gracefully
- Provide pre-flight verification in <5 minutes
- Implement thermal throttling and battery monitoring
- Achieve 99% reliability in flight operations

---

## 1. Power Optimization (6.3.1)

### 1.1 PowerManager Class Design

```cpp
// src/power_management.h

class PowerManager {
public:
  enum PowerMode {
    ACTIVE,          // 180mA avg, 6A peak - Full operation
    COAST_OPTIMIZED, // 120mA avg - Reduced telemetry during coast
    RECOVERY,        // 30mA avg - Minimal systems, beacon only
    SLEEP            // 2mA avg - Deep sleep, watchdog active
  };

  enum PowerState {
    MODE_CHANGED,
    BATTERY_LOW,
    THERMAL_THROTTLE,
    NORMAL
  };

  struct PowerMetrics {
    float current_consumption_ma;     // Instantaneous draw
    float battery_voltage_v;          // Measured supply voltage
    float estimated_flight_time_min;  // Predicted remaining time
    PowerMode current_mode;
    PowerState current_state;
    float temperature_c;              // From IMU sensor
    uint8_t battery_percent;          // Estimated SOC (0-100%)
  };

  // Initialization and management
  PowerManager(IEEPROM* eeprom, ISerial* serial, II2C* i2c);

  void init();                              // Initialize power monitoring
  void update();                            // Call every main loop (10Hz)
  void setPowerMode(PowerMode new_mode);   // Transition to new mode

  // Get current metrics
  PowerMetrics getMetrics();
  float getBatteryVoltage();
  float getEstimatedFlightTime();
  bool isBatteryOK();
  bool isBatteryLow();  // <3.0V for 2S LiPo
  bool isBatteryCritical();  // <2.7V

  // Subsystem control
  void disableWebTelemetry();      // Reduce serial output 80%
  void enableWebTelemetry();

  void disableSDCard();            // Stop all SD writes
  void enableSDCard();

  void reduceSensorFrequency();    // Sample at 5Hz instead of 10Hz
  void normalSensorFrequency();    // Restore 10Hz sampling

  void enableThermalThrottling();  // Reduce PID update rate
  void disableThermalThrottling();

  // Power budget calculation
  float calculatePowerBudget(uint32_t remaining_flight_time_s);

private:
  // ADC monitoring for battery voltage (Teensy ADC pin)
  float readBatteryVoltage();

  // Current estimation based on active systems
  float estimateCurrentDraw();

  // State machine for power transitions
  void transitionToPowerMode(PowerMode mode);

  // Battery capacity model (calibrated for 2S LiPo)
  float batteryCapacityModel(float voltage);  // Returns SOC %

  IEEPROM* eeprom;
  ISerial* serial;
  II2C* i2c;

  PowerMode current_mode;
  PowerMetrics metrics;
  uint32_t last_update_ms;
  uint32_t mode_transition_ms;

  // Configuration constants (in config.h)
  static constexpr float BATTERY_MIN_VOLTAGE = 2.7f;
  static constexpr float BATTERY_LOW_THRESHOLD = 3.0f;
  static constexpr float BATTERY_OK_THRESHOLD = 3.3f;
  static constexpr uint32_t POWER_UPDATE_INTERVAL_MS = 100;  // 10Hz update
};
```

### 1.2 Power Consumption Targets & Breakdown

```
ACTIVE Mode (180mA average):
├─ IMU sampling (ICM-20948)           12mA    (10Hz reads)
├─ Barometer (MS5611)                  3mA    (10Hz reads)
├─ GPS (UBLOX M8N)                    40mA    (1Hz updates)
├─ Servo control loop                  20mA    (PWM generation)
├─ Web telemetry (Serial at 230400)   35mA    (continuous stream)
├─ SD card writes                      25mA    (periodic bursts)
├─ Teensy 4.1 core                     30mA    (running at 600MHz)
└─ Miscellaneous (I2C, logic)          15mA
    Total: ~180mA

COAST_OPTIMIZED Mode (120mA):
├─ IMU sampling (reduce to 5Hz)         8mA
├─ Barometer (keep at 10Hz)             3mA
├─ GPS (reduce to 0.5Hz)               30mA
├─ Servo control loop                  15mA    (reduced frequency)
├─ Web telemetry (decimated 50%)       18mA    (every 2nd reading)
├─ SD card writes (reduced)            15mA    (every 10s instead of 5s)
├─ Teensy 4.1 core (reduced clock)     20mA    (400MHz instead of 600MHz)
└─ Miscellaneous                       11mA
    Total: ~120mA

RECOVERY Mode (30mA):
├─ IMU disabled                         0mA
├─ Barometer disabled                   0mA
├─ GPS enabled (location beacon)        15mA    (every 5 minutes)
├─ Servo control disabled               0mA
├─ Web telemetry disabled               0mA
├─ SD card disabled                     0mA
├─ Teensy 4.1 core (minimal)            10mA    (100MHz, waiting)
├─ Low-power timer                      3mA
└─ Watchdog monitoring                  2mA
    Total: ~30mA

SLEEP Mode (2mA):
├─ All systems disabled
├─ Teensy in deep sleep
├─ Only watchdog timer active
└─ Can wake on external interrupt
    Total: ~2mA
```

### 1.3 Subsystem Control Implementation

#### 1.3.1 Web Telemetry Reduction

```cpp
// In src/TripleT_Flight_Firmware.cpp main loop

void handlePowerOptimization() {
  PowerManager::PowerMetrics metrics = power_manager.getMetrics();

  // Determine if we should decimate telemetry output
  static uint8_t telemetry_counter = 0;
  bool should_send_telemetry = true;

  if (metrics.current_mode == PowerManager::COAST_OPTIMIZED) {
    // Send only every 2nd reading (50% reduction)
    should_send_telemetry = (telemetry_counter % 2 == 0);
  } else if (metrics.current_mode == PowerManager::RECOVERY) {
    // Disable telemetry entirely
    should_send_telemetry = false;
  }

  if (should_send_telemetry) {
    // Send: timestamp, altitude, velocity, state, warnings
    sendTelemetryPacket(log_data);
  }

  telemetry_counter++;
}
```

**Telemetry Decimation Levels:**
- ACTIVE: Send every 100ms (all data points)
- COAST_OPTIMIZED: Send every 200ms (50% decimation)
- RECOVERY: Disabled
- SLEEP: Disabled

**Data Reduction Strategy:**
- Remove non-critical fields: servo positions, filter diagnostic data
- Compress timestamp to delta encoding
- Round float values to 2 decimal places
- Aggregate statistics instead of raw samples

#### 1.3.2 SD Card Write Frequency Reduction

```cpp
// In src/log_format_definition.cpp

class SDCardWriter {
private:
  uint32_t write_interval_ms;
  uint32_t last_write_ms;
  bool buffered_writes_enabled;

public:
  void setWriteMode(PowerManager::PowerMode mode) {
    switch (mode) {
      case PowerManager::ACTIVE:
        write_interval_ms = 500;  // Write every 500ms (2 Hz)
        buffered_writes_enabled = false;
        break;
      case PowerManager::COAST_OPTIMIZED:
        write_interval_ms = 1000; // Write every 1s (1 Hz)
        buffered_writes_enabled = true;
        break;
      case PowerManager::RECOVERY:
        write_interval_ms = UINT32_MAX;  // Disable SD writes
        buffered_writes_enabled = false;
        break;
      case PowerManager::SLEEP:
        write_interval_ms = UINT32_MAX;
        buffered_writes_enabled = false;
        break;
    }
  }

  void writeDataPoint(const LogData& data) {
    if (!buffered_writes_enabled) {
      // Direct write mode
      if (millis() - last_write_ms >= write_interval_ms) {
        performSDWrite(data);
        last_write_ms = millis();
      }
      return;
    }

    // Buffered mode - accumulate data
    if (buffer_count < BUFFER_SIZE) {
      buffer[buffer_count++] = data;
    }

    // Write buffer when full or timeout
    if (buffer_count >= BUFFER_SIZE ||
        millis() - last_write_ms >= write_interval_ms) {
      flushBufferToSD();
      last_write_ms = millis();
    }
  }

private:
  static constexpr uint16_t BUFFER_SIZE = 10;  // 10 data points per buffer
  LogData buffer[BUFFER_SIZE];
  uint16_t buffer_count = 0;
};
```

**Write Frequency Strategy:**
- ACTIVE: Write every 500ms (minimal buffer)
- COAST_OPTIMIZED: Write every 1s (buffer 10 samples)
- RECOVERY: No writes (save power for beacon)
- SLEEP: No writes

**Expected Savings:** 25→15mA during COAST phase (40% reduction)

#### 1.3.3 Sensor Frequency Reduction

```cpp
// In src/sensors/imu_manager.h

class IMUManager {
private:
  uint8_t sample_rate_hz;  // 10Hz or 5Hz
  uint32_t sample_interval_ms;
  uint32_t last_sample_ms;

public:
  void setSampleRate(uint8_t hz) {
    if (hz != 10 && hz != 5) return;  // Only support 5Hz or 10Hz

    sample_rate_hz = hz;
    sample_interval_ms = 1000 / hz;
  }

  bool shouldSample() {
    return (millis() - last_sample_ms) >= sample_interval_ms;
  }

  bool read() {
    if (!shouldSample()) return false;

    // Perform sensor read
    bool success = primary_sensor->read();
    if (!success && backup_sensor) {
      success = backup_sensor->read();
    }

    last_sample_ms = millis();
    return success;
  }

  uint8_t getSampleRateHz() const { return sample_rate_hz; }
};
```

**Sampling Rate Impact:**
- 10Hz mode: 0.1s latency, higher responsiveness, 8mA baseline
- 5Hz mode: 0.2s latency, acceptable, 6mA baseline (25% power reduction)

### 1.4 Battery Monitoring Integration

```cpp
// src/power_management.cpp - Battery voltage monitoring

float PowerManager::readBatteryVoltage() {
  // ADC pin connected to battery through voltage divider
  // Teensy pin A0 -> (Battery) -[R1=100k]- -[R2=47k]- (GND)
  // Measured voltage = ADC_reading * 3.3 / 1023 * (R1+R2)/R2
  //                  = ADC_reading * 3.3 / 1023 * 147/47
  //                  ≈ ADC_reading * 0.01026

  uint16_t adc_raw = analogRead(BATTERY_ADC_PIN);
  float adc_voltage = adc_raw * 3.3f / 1023.0f;
  float battery_voltage = adc_voltage * 3.162f;  // Divider ratio

  // Moving average filter to smooth noise
  static float voltage_history[8] = {0};
  static uint8_t history_index = 0;

  voltage_history[history_index] = battery_voltage;
  history_index = (history_index + 1) % 8;

  float sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += voltage_history[i];
  }

  return sum / 8.0f;  // Return filtered average
}

// Battery capacity model using lookup table
// Calibrated for 2S LiPo (7.4V nominal, 6.0-8.4V range)
float PowerManager::batteryCapacityModel(float voltage) {
  // Cell voltage -> SOC lookup table
  static constexpr float voltage_levels[] = {
    3.00, 3.20, 3.40, 3.50, 3.60, 3.80, 4.00, 4.10, 4.20
  };
  static constexpr float soc_levels[] = {
    0,    5,    20,   35,   50,   75,   85,   92,   100
  };

  float cell_voltage = voltage / 2.0f;  // Convert 2S to single cell

  // Linear interpolation between lookup points
  for (int i = 0; i < 8; i++) {
    if (cell_voltage >= voltage_levels[i] &&
        cell_voltage <= voltage_levels[i+1]) {
      float fraction = (cell_voltage - voltage_levels[i]) /
                      (voltage_levels[i+1] - voltage_levels[i]);
      return soc_levels[i] + fraction * (soc_levels[i+1] - soc_levels[i]);
    }
  }

  // Clamp to 0-100%
  return (cell_voltage <= 3.0f) ? 0.0f : 100.0f;
}

// Estimate remaining flight time based on consumption rate
float PowerManager::getEstimatedFlightTime() {
  float battery_ma = metrics.battery_voltage_v * 2000;  // Assume 2000mAh battery
  float soc_percent = metrics.battery_percent;
  float available_mah = battery_ma * (soc_percent / 100.0f);

  float avg_consumption_ma = metrics.current_consumption_ma;
  if (avg_consumption_ma < 1.0f) return 999.0f;  // Avoid division by zero

  float remaining_hours = available_mah / avg_consumption_ma;
  return remaining_hours * 60.0f;  // Convert to minutes
}
```

**Battery Monitoring Thresholds:**
```
Battery State      Voltage    SOC%    Action
─────────────────────────────────────────────
Critical           <2.7V      0-5%    ERROR state, land immediately
Low                <3.0V      5-15%   Switch to RECOVERY mode
Warning            <3.3V      15-30%  Switch to COAST_OPTIMIZED mode
OK                 ≥3.3V      >30%    Continue ACTIVE mode
Full               ≥4.15V     ≥95%    Ready to fly
```

### 1.5 Flight Time Calculation

```cpp
// Flight time estimation formula

/*
Remaining Flight Time (minutes) =
  (Battery Capacity * SOC%) / Average Current Draw (mA)

Example:
- 2000 mAh battery at 60% SOC = 1200 mAh available
- ACTIVE mode: 180mA average
- Remaining time = 1200 / 180 = 6.67 minutes = 6m 40s

- If switching to COAST_OPTIMIZED after apogee:
- 3000 mAh nominal, 30% remaining = 900 mAh
- COAST_OPTIMIZED: 120mA
- Remaining time = 900 / 120 = 7.5 minutes = 7m 30s

Total Mission Time = Boost Phase (ACTIVE) + Coast Phase (COAST_OPTIMIZED)
                   + Descent Phase (mix of both)
*/

struct FlightPowerProfile {
  float boost_phase_duration_s;      // Time from ARMED to APOGEE
  float coast_phase_duration_s;      // Time from APOGEE to DROGUE_DEPLOY
  float descent_phase_duration_s;    // Time from DROGUE_DEPLOY to LANDED

  float active_mode_consumption_ma;
  float coast_mode_consumption_ma;

  // Calculate total battery consumed
  float calculateBatteryConsumed() {
    float boost_mah = active_mode_consumption_ma * (boost_phase_duration_s / 3600.0f);
    float coast_mah = coast_mode_consumption_ma * (coast_phase_duration_s / 3600.0f);
    float descent_mah = active_mode_consumption_ma * (descent_phase_duration_s / 3600.0f);
    return boost_mah + coast_mah + descent_mah;
  }

  // Verify battery is adequate
  bool isFlightFeasible(float available_mah) {
    return calculateBatteryConsumed() <= available_mah;
  }
};
```

---

## 2. Edge Case Handling (6.3.2)

### 2.1 GPS Loss During Flight

**Problem:** GPS receiver loses fix during flight, disabling return-to-pad guidance

**Solution Architecture:**

```cpp
// src/edge_cases/gps_loss_handler.h

class GPSLossHandler {
public:
  enum GPSRecoveryMode {
    BAROMETER_ONLY,      // Use altitude for apogee detection
    INERTIAL_DEAD_RECK,  // Dead reckoning from last known position
    PASSIVE_DESCENT,     // No guidance, just monitor altitude
    BEACON_MODE          // Enable recovery beacon
  };

  GPSLossHandler(const ApogeeDetector* apogee,
                 const GuidanceControl* guidance);

  void handleGPSLoss();
  void handleGPSReacquire();

  GPSRecoveryMode getCurrentMode() const;
  bool isGPSHealthy() const;

private:
  GPSRecoveryMode current_mode;
  uint32_t gps_loss_time_ms;
  bool apogee_detected_without_gps;

  static constexpr uint32_t GPS_LOSS_TIMEOUT_MS = 5000;  // 5s grace period
};
```

**Apogee Detection Without GPS:**

```cpp
bool ApogeeDetector::detectApogeeWithoutGPS() {
  // Use 2-of-2 voting: Barometer + Accelerometer (GPS excluded)

  bool baro_apogee = detectApogeeBarometric();
  bool accel_apogee = detectApogeeAcceleration();

  int vote_count = (baro_apogee ? 1 : 0) + (accel_apogee ? 1 : 0);

  if (vote_count >= 2) {
    apogee_detected_source = "Barometer+Accel (GPS unavailable)";
    return true;
  }

  // Fallback to timer method if both fail for >30 seconds
  if (!baro_apogee && !accel_apogee &&
      time_since_boost > APOGEE_TIMER_BACKUP_DELAY_MS) {
    apogee_detected_source = "Timer fallback (GPS+Sensors unavailable)";
    return true;
  }

  return false;
}
```

**Return-to-Pad Guidance Behavior:**

```cpp
void GuidanceControl::handleGPSLoss() {
  if (!gps_healthy) {
    // Disable precision return-to-pad
    trajectory_enabled = false;

    // Switch to passive mode - center servos, no active steering
    setServoCommand(0, 0);  // Roll=0, Pitch=0 (neutral)

    // Log the event
    logEvent("GPS lost - guidance disabled, passive mode active");

    // Still allow manual servo commands if needed
    manual_control_enabled = true;
  }
}
```

**Recovery Beacon Activation:**

```cpp
// Enable GPS reporting every 30 seconds for recovery beacon
void enableRecoveryBeacon() {
  gps_update_interval_ms = 30000;  // 30s reporting
  gps_send_position_to_serial = true;
  log_event("Recovery beacon enabled - GPS reports every 30s");
}

void sendGPSBeacon() {
  // Minimal telemetry: timestamp, lat, lon, alt, flight_state
  char beacon[64];
  snprintf(beacon, sizeof(beacon),
    "BEACON,%lu,%.6f,%.6f,%.1f,%d\n",
    millis(), current_latitude, current_longitude,
    current_altitude_m, (int)current_flight_state);
  serial->println(beacon);
}
```

### 2.2 High Wind Conditions

**Problem:** Strong winds cause unstable servo behavior, oscillation

**Solution:**

```cpp
// src/edge_cases/wind_handler.h

class WindHandler {
public:
  enum WindSeverity {
    CALM,           // <5 mph
    LIGHT,          // 5-10 mph (normal ops)
    MODERATE,       // 10-20 mph (guidance active)
    STRONG,         // 20-30 mph (stability margin increase)
    EXTREME,        // >30 mph (guidance disabled)
  };

  WindHandler(GuidanceControl* guidance, StabilityMonitor* stability);

  void updateWindEstimate(float apparent_wind_ms);
  WindSeverity getWindSeverity() const;

  void applyWindCompensation();

private:
  // Estimate wind from servo activity and gyro rates
  float estimateWindFromAcceleration();

  WindSeverity wind_severity;
  float wind_speed_ms;
};

void WindHandler::applyWindCompensation() {
  GuidanceControl::PIDGains base_gains = guidance->getBaseGains();
  GuidanceControl::PIDGains adjusted_gains = base_gains;

  switch (wind_severity) {
    case WindSeverity::CALM:
    case WindSeverity::LIGHT:
      // Normal operation
      adjusted_gains = base_gains;
      break;

    case WindSeverity::MODERATE:
      // Slight reduction - reduce integral term
      adjusted_gains.Ki *= 0.9f;
      adjusted_gains.Kd *= 1.05f;  // Increase damping
      break;

    case WindSeverity::STRONG:
      // Significant reduction - more conservative
      adjusted_gains.Kp *= 0.8f;   // Reduce proportional gain
      adjusted_gains.Ki *= 0.7f;
      adjusted_gains.Kd *= 1.2f;   // Increase damping significantly

      // Increase stability margins
      stability_monitor->setAngularRateLimit(120.0f);  // Down from 180 DPS
      stability_monitor->setAttitudeErrorLimit(15.0f); // Down from 20 deg
      break;

    case WindSeverity::EXTREME:
      // Disable guidance entirely
      guidance->setMode(GuidanceControl::PASSIVE);
      logEvent("Wind extreme - guidance disabled (passive mode)");
      break;
  }

  guidance->setPIDGains(adjusted_gains);
}
```

**Wind Estimation Algorithm:**

```cpp
float WindHandler::estimateWindFromAcceleration() {
  // Use gyro spin-up rate as proxy for wind disturbance
  // Higher gyro rates with small servo commands = higher wind

  float gyro_rate = sqrt(gyro_x*gyro_x + gyro_y*gyro_y + gyro_z*gyro_z);
  float servo_command = fabs(current_servo_roll) + fabs(current_servo_pitch);

  // If high gyro with low servo = wind, not guidance
  if (gyro_rate > 100.0f && servo_command < 10.0f) {
    // Estimate wind speed (simplified model)
    return (gyro_rate - 50.0f) * 0.02f;  // Rough conversion to m/s
  }

  return 0.0f;  // Low wind
}
```

### 2.3 Sensor Saturation Handling

**Status:** Already handled in Phase 4 (IMUManager with automatic failover)

**Reference Implementation:**

```cpp
// Verify from src/sensors/imu_manager.h
bool IMUManager::read() {
  // Primary sensor (ICM-20948, ±16G)
  if (primary_healthy && primary->read()) {
    if (!checkForSaturation(primary->getAcceleration())) {
      return true;  // Healthy and not saturated
    }
  }

  // If primary saturated or unhealthy, try backup (KX134, ±64G)
  if (backup_healthy && backup->read()) {
    return true;  // Backup provides high-G data
  }

  // Both saturated or failed
  return false;
}

bool IMUManager::checkForSaturation(float ax, float ay, float az) {
  float magnitude = sqrt(ax*ax + ay*ay + az*az);

  // ICM-20948 saturates at ±16G
  if (magnitude >= 15.8f) {
    switchToBackupSensor();
    return true;
  }

  return false;
}
```

**Additional Protection - Both Saturate Case:**

```cpp
void IMUManager::handleBothSaturated() {
  // If both sensors saturate, we have a high-G event
  // Options:
  // 1. Log high-G event and continue (conservative)
  // 2. Reduce guidance gains temporarily
  // 3. Switch to passive mode (safest)

  logEvent("CRITICAL: Both IMU sensors saturated - high-G event");

  // Use last known good quaternion
  // Don't try to correct attitude during extreme forces
  last_valid_quaternion_time = millis();

  // Inform guidance to be conservative
  guidance_control->setMode(GuidanceControl::PASSIVE);
  guidance_control->setHighGEvent(true);

  // Re-enable guidance after 1 second without saturation
}
```

### 2.4 EEPROM Corruption Recovery

**Problem:** Power loss during EEPROM write corrupts flight state, can't recover

**Solution:**

```cpp
// src/edge_cases/eeprom_recovery.h

class EEPROMRecovery {
public:
  enum CorruptionLevel {
    NO_CORRUPTION,
    MINOR_CORRUPTION,     // Checksum mismatch, data salvageable
    MAJOR_CORRUPTION,     // Multiple fields invalid
    TOTAL_CORRUPTION      // Entire state unreadable
  };

  struct RecoveryState {
    CorruptionLevel level;
    FlightState safe_state;
    uint32_t recovery_timestamp;
    const char* error_description;
  };

  EEPROMRecovery(IEEPROM* eeprom);

  RecoveryState checkEEPROMHealth();
  bool recoverFlightState(FlightState& recovered_state);
  void writeFlightStateWithChecksum(const FlightState& state);

private:
  bool verifyChecksum();
  uint8_t calculateChecksum(const uint8_t* data, size_t len);

  IEEPROM* eeprom;
};
```

**Checksum Verification:**

```cpp
// src/state_management.cpp

struct FlightStateEEPROM {
  FlightState state;
  uint8_t checksum;
  uint32_t write_timestamp;
  uint8_t version;  // Detect format changes
};

bool verifyEEPROMChecksum() {
  FlightStateEEPROM stored;
  eeprom->read(EEPROM_STATE_ADDRESS, (uint8_t*)&stored, sizeof(stored));

  // Calculate checksum of state without checksum field
  uint8_t calculated = 0;
  uint8_t* ptr = (uint8_t*)&stored.state;
  for (size_t i = 0; i < sizeof(FlightState); i++) {
    calculated ^= ptr[i];  // XOR checksum
  }

  if (calculated != stored.checksum) {
    logError("EEPROM checksum mismatch!");
    return false;
  }

  return true;
}

void writeFlightStateWithChecksum(const FlightState& state) {
  // Calculate checksum
  uint8_t checksum = 0;
  uint8_t* ptr = (uint8_t*)&state;
  for (size_t i = 0; i < sizeof(FlightState); i++) {
    checksum ^= ptr[i];
  }

  // Write state + checksum + metadata
  FlightStateEEPROM stored = {state, checksum, millis(), EEPROM_VERSION};
  eeprom->write(EEPROM_STATE_ADDRESS, (uint8_t*)&stored, sizeof(stored));
}
```

**Safe State Fallback:**

```cpp
RecoveryState EEPROMRecovery::checkEEPROMHealth() {
  RecoveryState recovery = {NO_CORRUPTION, PAD_IDLE, 0, nullptr};

  if (!verifyChecksum()) {
    // Can't trust stored state
    recovery.level = MAJOR_CORRUPTION;
    recovery.safe_state = PAD_IDLE;
    recovery.error_description = "Checksum failed - reset to PAD_IDLE";
    return recovery;
  }

  FlightStateEEPROM stored;
  eeprom->read(EEPROM_STATE_ADDRESS, (uint8_t*)&stored, sizeof(stored));

  // Sanity checks on the stored state
  if (stored.state < 0 || stored.state >= ERROR) {
    recovery.level = MINOR_CORRUPTION;
    recovery.safe_state = PAD_IDLE;
    recovery.error_description = "Invalid state value - reset to PAD_IDLE";
    return recovery;
  }

  recovery.level = NO_CORRUPTION;
  recovery.safe_state = stored.state;
  recovery.error_description = nullptr;
  return recovery;
}
```

---

## 3. Pre-Flight Verification System (6.3.3)

### 3.1 PreflightChecker Class Design

```cpp
// src/preflight_checks.h

class PreflightChecker {
public:
  enum CheckStatus {
    PASS = 0,
    WARN = 1,
    FAIL = 2
  };

  struct PreflightResult {
    CheckStatus sensors_status;       // All sensors responding
    CheckStatus power_status;         // Battery voltage OK
    CheckStatus storage_status;       // SD card formatted, space
    CheckStatus firmware_status;      // Config values valid
    CheckStatus eeprom_status;        // Flight state not corrupted
    CheckStatus servos_status;        // Servo range of motion
    CheckStatus pyro_status;          // Continuity detected

    uint32_t total_checks;
    uint32_t passed_checks;
    uint32_t warning_checks;
    uint32_t failed_checks;

    uint32_t duration_ms;             // How long checks took
    const char* primary_error;        // First fatal error, if any
  };

  PreflightChecker(IMUManager* imu, IEEPROM* eeprom, ISDCard* sd,
                   IServo* servo, IGuidance* guidance);

  // Run all checks (blocking, ~30 seconds)
  PreflightResult runFullCheck();

  // Individual checks
  CheckStatus checkSensors();
  CheckStatus checkPower();
  CheckStatus checkStorage();
  CheckStatus checkFirmware();
  CheckStatus checkEEPROM();
  CheckStatus checkServos();
  CheckStatus checkPyro();

  // Timeout protection
  bool isCheckInProgress() const;

private:
  IMUManager* imu;
  IEEPROM* eeprom;
  ISDCard* sd;
  IServo* servo;
  IGuidance* guidance;

  uint32_t check_start_time_ms;
  static constexpr uint32_t CHECK_TIMEOUT_MS = 30000;  // 30s total
};
```

### 3.2 Individual Check Implementations

#### 3.2.1 Sensor Health Check

```cpp
PreflightChecker::CheckStatus PreflightChecker::checkSensors() {
  CheckStatus overall_status = PASS;

  // ICM-20948
  if (!imu->isPrimarySensorHealthy()) {
    logWarning("ICM-20948 not responding");
    overall_status = FAIL;
  }

  // KX134 (backup)
  if (!imu->isBackupSensorHealthy()) {
    logWarning("KX134 backup not responding");
    overall_status = (overall_status == FAIL) ? FAIL : WARN;
  }

  // Barometer
  if (!baro->isHealthy()) {
    logWarning("MS5611 barometer not responding");
    overall_status = FAIL;  // Critical for apogee detection
  }

  // GPS
  if (!gps->hasGPSFix()) {
    logWarning("GPS has no fix - will use barometer for apogee");
    overall_status = (overall_status == FAIL) ? FAIL : WARN;
  }

  if (overall_status == PASS) {
    logMessage("✓ All sensors healthy");
  }

  return overall_status;
}
```

#### 3.2.2 Power Check

```cpp
PreflightChecker::CheckStatus PreflightChecker::checkPower() {
  PowerManager::PowerMetrics metrics = power_manager->getMetrics();

  if (metrics.battery_voltage_v < 2.7f) {
    logError("Battery critical: %.2fV (minimum 2.7V)",
             metrics.battery_voltage_v);
    return FAIL;
  }

  if (metrics.battery_voltage_v < 3.3f) {
    logWarning("Battery low: %.2fV (recommended >3.3V)",
               metrics.battery_voltage_v);
    return WARN;
  }

  logMessage("✓ Battery OK: %.2fV (%.0f%% capacity)",
             metrics.battery_voltage_v, metrics.battery_percent);

  return PASS;
}
```

#### 3.2.3 SD Card Check

```cpp
PreflightChecker::CheckStatus PreflightChecker::checkStorage() {
  if (!sd->isCardPresent()) {
    logError("SD card not inserted");
    return FAIL;
  }

  if (!sd->isCardFormatted()) {
    logError("SD card not formatted - format as FAT32");
    return FAIL;
  }

  uint64_t free_space_bytes = sd->getFreeSpace();
  uint64_t total_space_bytes = sd->getTotalSpace();

  // Estimate: ~500 bytes per second of flight, typical ~60s flight
  uint64_t min_required_bytes = 500 * 60;  // 30KB

  if (free_space_bytes < min_required_bytes) {
    logWarning("SD card low on space: %lld bytes free", free_space_bytes);
    return WARN;
  }

  float used_percent = 100.0f * (1.0f - free_space_bytes / (float)total_space_bytes);
  logMessage("✓ SD card OK: %.1f%% used, %.1f MB free",
             used_percent, free_space_bytes / 1024.0f / 1024.0f);

  return PASS;
}
```

#### 3.2.4 Firmware Configuration Check

```cpp
PreflightChecker::CheckStatus PreflightChecker::checkFirmware() {
  CheckStatus status = PASS;

  // Verify config.h settings are reasonable
  #ifdef CONFIG_VERSION
    if (CONFIG_VERSION != 0x01) {
      logWarning("Config version mismatch - update may be needed");
      status = WARN;
    }
  #endif

  // Check safety-critical parameters
  if (APOGEE_DETECTION_TIMEOUT_MS < 1000 ||
      APOGEE_DETECTION_TIMEOUT_MS > 60000) {
    logError("Invalid apogee timeout in config.h");
    return FAIL;
  }

  if (GUIDANCE_STABILITY_MARGIN < 0.5f ||
      GUIDANCE_STABILITY_MARGIN > 30.0f) {
    logError("Invalid guidance stability margin");
    return FAIL;
  }

  #ifdef FIRMWARE_VERSION
    logMessage("✓ Firmware version: %s", FIRMWARE_VERSION);
  #endif

  return status;
}
```

#### 3.2.5 EEPROM Check

```cpp
PreflightChecker::CheckStatus PreflightChecker::checkEEPROM() {
  if (!verifyEEPROMChecksum()) {
    logWarning("EEPROM checksum mismatch - will reset to PAD_IDLE");
    return WARN;  // Recoverable
  }

  FlightState stored_state = readFlightStateFromEEPROM();

  if (stored_state < 0 || stored_state >= ERROR_STATE) {
    logWarning("Invalid state in EEPROM: %d", stored_state);
    return WARN;
  }

  logMessage("✓ EEPROM OK - current state: %s",
             getFlightStateName(stored_state));

  return PASS;
}
```

#### 3.2.6 Servo Check

```cpp
PreflightChecker::CheckStatus PreflightChecker::checkServos() {
  logMessage("Testing servo 1 (roll)...");

  // Command roll servo full range
  servo->setRollCommand(-90.0f);  // Full left
  delay(500);
  if (!servo->isMoving()) {
    logError("Servo 1 (roll) not responding");
    return FAIL;
  }

  servo->setRollCommand(90.0f);   // Full right
  delay(500);
  servo->setRollCommand(0.0f);    // Center
  delay(500);

  logMessage("Testing servo 2 (pitch)...");

  // Command pitch servo full range
  servo->setPitchCommand(-45.0f); // Full down
  delay(500);
  if (!servo->isMoving()) {
    logError("Servo 2 (pitch) not responding");
    return FAIL;
  }

  servo->setPitchCommand(45.0f);  // Full up
  delay(500);
  servo->setPitchCommand(0.0f);   // Center
  delay(500);

  logMessage("✓ Servos OK - full range verified");

  return PASS;
}
```

#### 3.2.7 Pyro Continuity Check

```cpp
PreflightChecker::CheckStatus PreflightChecker::checkPyro() {
  // Check drogue pyro channel continuity
  if (!guidance->pyroChannels.checkContinuity(PYRO_DROGUE)) {
    logError("Drogue pyro channel open (no continuity)");
    return FAIL;
  }
  logMessage("✓ Drogue pyro continuity OK");

  // Check main pyro channel continuity
  if (!guidance->pyroChannels.checkContinuity(PYRO_MAIN)) {
    logError("Main pyro channel open (no continuity)");
    return FAIL;
  }
  logMessage("✓ Main pyro continuity OK");

  return PASS;
}
```

### 3.3 User-Facing Output Format

```
╔════════════════════════════════════════╗
║      PRE-FLIGHT VERIFICATION (v1.0)     ║
╚════════════════════════════════════════╝

[1/7] Sensor Health Check...
  ✓ ICM-20948 primary sensor: OK
  ✓ KX134 backup accelerometer: OK
  ✓ MS5611 barometer: OK
  ⚠ GPS receiver: No fix (will use barometer)
  Status: WARN (acceptable, barometer sufficient)

[2/7] Power System Check...
  ✓ Battery voltage: 7.45V
  ✓ Battery capacity: 85%
  ✓ Estimated flight time: 8.2 minutes
  Status: PASS

[3/7] Storage System Check...
  ✓ SD card present: Yes
  ✓ SD card formatted: FAT32
  ✓ Free space: 2.1 GB (98% available)
  Status: PASS

[4/7] Firmware Configuration Check...
  ✓ Config version: 1
  ✓ Apogee timeout: 60,000ms
  ✓ Stability margin: 20°
  ✓ Firmware version: v1.0.0-RC1
  Status: PASS

[5/7] Flight State Check...
  ✓ EEPROM checksum: Valid
  ✓ Stored state: PAD_IDLE
  Status: PASS

[6/7] Servo Actuation Test...
  Testing roll servo (1/2)...
    ✓ Full left (-90°): Response OK
    ✓ Full right (+90°): Response OK
    ✓ Centered (0°): Response OK
  Testing pitch servo (2/2)...
    ✓ Full up (+45°): Response OK
    ✓ Full down (-45°): Response OK
    ✓ Centered (0°): Response OK
  Status: PASS

[7/7] Pyro Channel Continuity...
  ✓ Drogue channel: Continuity confirmed
  ✓ Main channel: Continuity confirmed
  Status: PASS

════════════════════════════════════════

SUMMARY:
  Checks passed: 20/20
  Warnings: 1 (GPS - acceptable)
  Failures: 0
  Total time: 28 seconds

RESULT: ✓ SYSTEM READY TO FLY

Press ENTER to proceed or 'C' to cancel...
```

### 3.4 Timeout Handling

```cpp
PreflightResult PreflightChecker::runFullCheck() {
  uint32_t check_start = millis();
  PreflightResult result = {PASS, PASS, PASS, PASS, PASS, PASS, PASS,
                            0, 0, 0, 0, 0, nullptr};

  // Safety: enforce 30-second timeout
  auto checkTimeout = [&]() {
    if (millis() - check_start > CHECK_TIMEOUT_MS) {
      result.primary_error = "Check timeout exceeded (>30s)";
      return true;
    }
    return false;
  };

  result.sensors_status = checkSensors();
  if (checkTimeout()) goto abort_checks;

  result.power_status = checkPower();
  if (checkTimeout()) goto abort_checks;

  result.storage_status = checkStorage();
  if (checkTimeout()) goto abort_checks;

  result.firmware_status = checkFirmware();
  if (checkTimeout()) goto abort_checks;

  result.eeprom_status = checkEEPROM();
  if (checkTimeout()) goto abort_checks;

  result.servos_status = checkServos();
  if (checkTimeout()) goto abort_checks;

  result.pyro_status = checkPyro();
  if (checkTimeout()) goto abort_checks;

  // Calculate summary
  result.total_checks = 7;
  result.passed_checks = countPassed();
  result.warning_checks = countWarnings();
  result.failed_checks = countFailed();

abort_checks:
  result.duration_ms = millis() - check_start;
  return result;
}
```

---

## 4. Command Processing Enhancements (6.3.4)

### 4.1 New Serial Commands

```cpp
// src/command_processor.cpp - New commands for Phase 6.3

void handlePreflight(const SystemStatusContext& context) {
  context.serial->println("\nRunning pre-flight verification...");
  context.serial->println("This will take approximately 30 seconds.\n");

  PreflightResult result = preflight_checker->runFullCheck();

  // Print detailed results
  context.serial->printf("Pre-flight check complete in %u ms\n", result.duration_ms);
  context.serial->printf("Passed: %u, Warnings: %u, Failed: %u\n",
    result.passed_checks, result.warning_checks, result.failed_checks);

  if (result.primary_error) {
    context.serial->printf("ERROR: %s\n", result.primary_error);
  }

  if (result.failed_checks > 0) {
    context.serial->println("\n⚠ System not ready to fly due to failures.");
  } else if (result.warning_checks > 0) {
    context.serial->println("\n✓ System ready (with warnings - review above)");
  } else {
    context.serial->println("\n✓ System ready to fly");
  }
}

void handleTelemetryOn(const SystemStatusContext& context) {
  power_manager->enableWebTelemetry();
  context.serial->println("Telemetry enabled - full data stream active");
}

void handleTelemetryOff(const SystemStatusContext& context) {
  power_manager->disableWebTelemetry();
  context.serial->println("Telemetry disabled - power saving mode");
}

void handleServoTest(const SystemStatusContext& context) {
  context.serial->println("Servo test sequence starting...\n");

  // Roll servo
  context.serial->println("Roll servo: moving to -90° (full left)...");
  servo_control->setRollCommand(-90.0f);
  delay(1000);
  context.serial->println("Roll servo: moving to +90° (full right)...");
  servo_control->setRollCommand(90.0f);
  delay(1000);
  context.serial->println("Roll servo: centering...");
  servo_control->setRollCommand(0.0f);
  delay(500);

  // Pitch servo
  context.serial->println("Pitch servo: moving to -45° (full down)...");
  servo_control->setPitchCommand(-45.0f);
  delay(1000);
  context.serial->println("Pitch servo: moving to +45° (full up)...");
  servo_control->setPitchCommand(45.0f);
  delay(1000);
  context.serial->println("Pitch servo: centering...");
  servo_control->setPitchCommand(0.0f);
  delay(500);

  context.serial->println("\n✓ Servo test complete - both servos functional");
}

void handlePyroTest(const SystemStatusContext& context) {
  context.serial->println("Pyro channel test starting...\n");

  // Drogue channel
  context.serial->println("Testing drogue channel continuity...");
  if (pyro_channels->checkContinuity(PYRO_DROGUE)) {
    context.serial->println("✓ Drogue continuity OK");
  } else {
    context.serial->println("✗ Drogue channel OPEN");
  }

  // Main channel
  context.serial->println("Testing main channel continuity...");
  if (pyro_channels->checkContinuity(PYRO_MAIN)) {
    context.serial->println("✓ Main continuity OK");
  } else {
    context.serial->println("✗ Main channel OPEN");
  }

  context.serial->println("\nNote: If testing with igniters connected:");
  context.serial->println("  - Continuity indicates igniter ready");
  context.serial->println("  - Do NOT fire charges during testing");
}

void handleLoadTrajectory(const SystemStatusContext& context) {
  context.serial->println("Trajectories on SD card:");
  context.serial->println("  1. competition_flight_1.json");
  context.serial->println("  2. hover_test.json");
  context.serial->println("\nUsage: load_trajectory <filename>");

  // This would be extended to actually parse command args
  if (trajectory_controller->loadTrajectory("competition_flight_1.json")) {
    context.serial->println("✓ Trajectory loaded: competition_flight_1.json");
    context.serial->println("  Ready to start - use 'start_trajectory' when armed");
  } else {
    context.serial->println("✗ Failed to load trajectory");
  }
}

void handleStartTrajectory(const SystemStatusContext& context) {
  if (flight_state != COAST) {
    context.serial->println("✗ Trajectory can only start in COAST state");
    context.serial->println("  Current state: " << getFlightStateName(flight_state));
    return;
  }

  trajectory_controller->startTrajectory();
  context.serial->println("✓ Trajectory following active");
  context.serial->println("  Tracking to waypoint 1 of 3");
}

void handlePowerMode(const SystemStatusContext& context) {
  context.serial->println("Current power mode: " <<
                          (int)power_manager->getCurrentMode());
  context.serial->println("\nAvailable modes:");
  context.serial->println("  0 = ACTIVE (180mA)");
  context.serial->println("  1 = COAST_OPTIMIZED (120mA)");
  context.serial->println("  2 = RECOVERY (30mA)");
  context.serial->println("  3 = SLEEP (2mA)");
  context.serial->println("\nUsage: power_mode <mode_number>");
}

void handleBattery(const SystemStatusContext& context) {
  PowerManager::PowerMetrics metrics = power_manager->getMetrics();

  context.serial->printf("\nBattery Status:\n");
  context.serial->printf("  Voltage: %.2fV\n", metrics.battery_voltage_v);
  context.serial->printf("  Capacity: %.0f%%\n", metrics.battery_percent);
  context.serial->printf("  Current draw: %.0f mA\n",
                         metrics.current_consumption_ma);
  context.serial->printf("  Estimated flight time: %.1f minutes\n",
                         metrics.estimated_flight_time_min);
  context.serial->printf("  Temperature: %.1f°C\n\n",
                         metrics.temperature_c);

  if (metrics.battery_voltage_v < 2.7f) {
    context.serial->println("  ⚠ CRITICAL: Battery too low");
  } else if (metrics.battery_voltage_v < 3.3f) {
    context.serial->println("  ⚠ WARNING: Battery low");
  } else {
    context.serial->println("  ✓ Battery OK");
  }
}

void handleReboot(const SystemStatusContext& context) {
  context.serial->println("System rebooting in 2 seconds...");
  delay(2000);

  // Teensy reboot using watchdog
  wdt->enable(100);  // 100ms timeout
  while(1);  // Watchdog will trigger reset
}
```

### 4.2 Command Dispatcher Integration

```cpp
// In src/command_processor.cpp - main command handler

void processCommand(const char* cmd, const SystemStatusContext& context) {
  // Convert to lowercase for comparison
  char cmd_lower[64];
  strncpy(cmd_lower, cmd, sizeof(cmd_lower));
  for (char* p = cmd_lower; *p; ++p) *p = tolower(*p);

  // Existing commands (from Phase 1-5)
  if (strcmp(cmd_lower, "arm") == 0) {
    handleArm(context);
  }
  else if (strcmp(cmd_lower, "disarm") == 0) {
    handleDisarm(context);
  }
  else if (strcmp(cmd_lower, "status_sensors") == 0) {
    handleStatusSensors(context);
  }
  // ... other existing commands ...

  // NEW Phase 6.3 commands
  else if (strcmp(cmd_lower, "preflight") == 0) {
    handlePreflight(context);
  }
  else if (strcmp(cmd_lower, "telemetry_on") == 0) {
    handleTelemetryOn(context);
  }
  else if (strcmp(cmd_lower, "telemetry_off") == 0) {
    handleTelemetryOff(context);
  }
  else if (strcmp(cmd_lower, "servo_test") == 0) {
    handleServoTest(context);
  }
  else if (strcmp(cmd_lower, "pyro_test") == 0) {
    handlePyroTest(context);
  }
  else if (strcmp(cmd_lower, "load_trajectory") == 0) {
    handleLoadTrajectory(context);
  }
  else if (strcmp(cmd_lower, "start_trajectory") == 0) {
    handleStartTrajectory(context);
  }
  else if (strcmp(cmd_lower, "power_mode") == 0) {
    handlePowerMode(context);
  }
  else if (strcmp(cmd_lower, "battery") == 0) {
    handleBattery(context);
  }
  else if (strcmp(cmd_lower, "reboot") == 0) {
    handleReboot(context);
  }
  else if (strcmp(cmd_lower, "help") == 0) {
    printHelpText(context);
  }
  else {
    context.serial->println("Unknown command. Type 'help' for list.");
  }
}

void printHelpText(const SystemStatusContext& context) {
  context.serial->println("\n╔════════════════════════════════════════╗");
  context.serial->println("║   TripleT Flight Computer v1.0        ║");
  context.serial->println("║   Available Serial Commands            ║");
  context.serial->println("╚════════════════════════════════════════╝\n");

  context.serial->println("Flight Control:");
  context.serial->println("  arm              - Arm system for launch");
  context.serial->println("  disarm           - Disarm system");

  context.serial->println("\nPre-Flight:");
  context.serial->println("  preflight        - Run full pre-flight check (30s)");
  context.serial->println("  servo_test       - Test servo motion");
  context.serial->println("  pyro_test        - Check pyro continuity");
  context.serial->println("  battery          - Show voltage & flight time");

  context.serial->println("\nDiagnostics:");
  context.serial->println("  status_sensors   - Detailed sensor report");
  context.serial->println("  power_mode       - Show/change power mode");
  context.serial->println("  telemetry_on     - Enable telemetry");
  context.serial->println("  telemetry_off    - Disable telemetry");

  context.serial->println("\nTrajectory (experimental):");
  context.serial->println("  load_trajectory  - Load trajectory file");
  context.serial->println("  start_trajectory - Begin trajectory following");

  context.serial->println("\nSystem:");
  context.serial->println("  reboot           - Restart system");
  context.serial->println("  help             - Show this help text");
  context.serial->println();
}
```

---

## 5. Thermal Management (6.3.5)

### 5.1 Temperature Monitoring Integration

```cpp
// src/thermal_management.h

class ThermalManager {
public:
  enum ThermalState {
    NORMAL,      // <70°C - No throttling
    WARNING,     // 70-85°C - Reduced update rates
    CRITICAL,    // >85°C - Minimal operation
    SHUTDOWN     // >95°C - Emergency shutdown
  };

  struct ThermalMetrics {
    float current_temp_c;
    float max_temp_c;
    float avg_temp_c;
    ThermalState current_state;
    uint32_t time_in_warning_ms;
    uint32_t time_in_critical_ms;
  };

  ThermalManager(IMUManager* imu, GuidanceControl* guidance,
                 PowerManager* power);

  void update();
  ThermalMetrics getMetrics();

  void applyThermalThrottling();
  void clearThermalThrottling();

private:
  ThermalState checkTemperature(float temp);

  IMUManager* imu;
  GuidanceControl* guidance;
  PowerManager* power;

  ThermalMetrics metrics;
  ThermalState previous_state;

  static constexpr float TEMP_WARNING = 70.0f;
  static constexpr float TEMP_CRITICAL = 85.0f;
  static constexpr float TEMP_SHUTDOWN = 95.0f;
};
```

### 5.2 Throttling Strategy

```cpp
void ThermalManager::applyThermalThrottling() {
  float temp = imu->getTemperature();
  ThermalState new_state = checkTemperature(temp);

  if (new_state == previous_state) return;  // No state change

  switch (new_state) {
    case NORMAL:
      // Resume normal operation
      guidance->setUpdateRate(10);  // 10Hz
      power->setPowerMode(PowerManager::ACTIVE);
      logMessage("Thermal: Normal (%.1f°C)", temp);
      break;

    case WARNING:
      // Reduce update rates
      guidance->setUpdateRate(5);   // 5Hz instead of 10Hz
      power->setSensorFrequency(5);
      power->setPowerMode(PowerManager::COAST_OPTIMIZED);
      logWarning("Thermal: Temperature warning (%.1f°C) - reduced rates", temp);
      break;

    case CRITICAL:
      // Minimal operation
      guidance->setUpdateRate(2);   // 2Hz - very conservative
      power->disableWebTelemetry();
      power->disableSDCard();
      power->setPowerMode(PowerManager::RECOVERY);
      logError("CRITICAL: Temperature %.1f°C - minimal mode", temp);
      break;

    case SHUTDOWN:
      // Emergency shutdown
      guidance->setMode(GuidanceControl::PASSIVE);
      power->setPowerMode(PowerManager::SLEEP);
      logError("FATAL: Temperature %.1f°C - shutdown", temp);
      break;
  }

  previous_state = new_state;
}
```

### 5.3 Temperature Logging

```cpp
// In flight data logging - add temperature field

struct LogData {
  // ... existing fields ...
  float imu_temperature_c;       // From ICM-20948 internal sensor
  uint8_t thermal_state;         // 0=normal, 1=warning, 2=critical
};

void logThermalData(const LogData& data) {
  // Add to CSV header if not present
  // Log periodically (every 10 seconds to not overwhelm SD card)

  static uint32_t last_thermal_log = 0;
  if (millis() - last_thermal_log < 10000) return;

  ThermalMetrics metrics = thermal_manager->getMetrics();
  sd_card->printf("THERMAL,%.1f,%.1f,%.1f,%d\n",
    metrics.current_temp_c,
    metrics.max_temp_c,
    metrics.avg_temp_c,
    (int)metrics.current_state);

  last_thermal_log = millis();
}
```

---

## 6. Signal Integrity & Noise Filtering (6.3.6)

### 6.1 Kalman Filter Tuning Parameters

```cpp
// src/kalman_filter.h - Tuning from Phase 4 test data

class KalmanFilter {
private:
  // Process noise covariance (Q matrix)
  // Higher Q = less trust in model, more trust in measurements
  const float Q[3][3] = {
    {0.01f, 0.0f, 0.0f},     // Accelerometer process noise
    {0.0f, 0.01f, 0.0f},     // Magnetometer process noise
    {0.0f, 0.0f, 0.005f}     // Gyroscope process noise
  };

  // Measurement noise covariance (R matrix)
  // Higher R = more trust in model, less trust in measurements
  const float R[3][3] = {
    {0.5f, 0.0f, 0.0f},      // Accelerometer measurement noise
    {0.0f, 1.0f, 0.0f},      // Magnetometer measurement noise
    {0.0f, 0.0f, 0.1f}       // Gyroscope measurement noise
  };

public:
  // Tuned for hover condition (minimal motion)
  // Process: predict attitude using gyro, correct with accel/mag
  Quaternion update(const Quaternion& gyro_rate,
                    const Vector3& accel,
                    const Vector3& mag,
                    float dt_s);
};
```

**Tuning Rationale:**
- Q values from Phase 4 accelerometer drift analysis (~0.01 deg/s²)
- R values from sensor noise characterization (±2% for typical aerospace sensors)
- Gyroscope R lower because gyro noise is minimal during boost phase

### 6.2 Servo Command Filtering

```cpp
// src/servo_control.h - Low-pass filter on servo commands

class ServoCommandFilter {
private:
  // First-order low-pass filter
  // Cutoff frequency: 20 Hz (servo natural frequency ~5Hz)
  float alpha = 0.3f;  // Time constant ~0.05s at 10Hz update

  float filtered_roll_command = 0.0f;
  float filtered_pitch_command = 0.0f;

public:
  float filterRollCommand(float raw_command) {
    filtered_roll_command = alpha * raw_command +
                           (1.0f - alpha) * filtered_roll_command;
    return filtered_roll_command;
  }

  float filterPitchCommand(float raw_command) {
    filtered_pitch_command = alpha * raw_command +
                            (1.0f - alpha) * filtered_pitch_command;
    return filtered_pitch_command;
  }

  // Reset when guidance disabled
  void reset() {
    filtered_roll_command = 0.0f;
    filtered_pitch_command = 0.0f;
  }
};

/*
Filter calculation:
  y(n) = α*x(n) + (1-α)*y(n-1)

  At 10Hz update rate:
  α = 2*π*Fc*Ts / (2*π*Fc*Ts + 1)

  For Fc=20Hz, Ts=0.1s:
  α = 2*π*20*0.1 / (2*π*20*0.1 + 1) = 12.57 / 13.57 ≈ 0.93

  But we use α=0.3 for more aggressive smoothing to reduce
  servo oscillation during high-wind conditions.
*/
```

### 6.3 Gyro High-Pass Filter

```cpp
// src/gyro_filter.h - Remove gyro bias drift

class GyroHighPassFilter {
private:
  // High-pass filter: remove DC offset and slow drift
  // Cutoff: 0.1 Hz (remove everything below 10-second periods)
  float alpha = 0.001f;

  Vector3 gyro_offset = {0, 0, 0};
  Vector3 filtered_gyro = {0, 0, 0};

public:
  Vector3 filterGyro(const Vector3& raw_gyro) {
    // Update bias estimate (very slow time constant)
    gyro_offset.x = 0.999f * gyro_offset.x + 0.001f * raw_gyro.x;
    gyro_offset.y = 0.999f * gyro_offset.y + 0.001f * raw_gyro.y;
    gyro_offset.z = 0.999f * gyro_offset.z + 0.001f * raw_gyro.z;

    // Remove bias
    filtered_gyro.x = raw_gyro.x - gyro_offset.x;
    filtered_gyro.y = raw_gyro.y - gyro_offset.y;
    filtered_gyro.z = raw_gyro.z - gyro_offset.z;

    return filtered_gyro;
  }

  void reset() {
    gyro_offset = {0, 0, 0};
    filtered_gyro = {0, 0, 0};
  }
};

/*
Gyro bias at idle: ±5 DPS typical
Bias drift: ±0.02 DPS/minute (temperature dependent)

High-pass filter removes this constant bias while preserving
real angular rates during flight maneuvers (which are >50 DPS).
*/
```

### 6.4 Optional Notch Filter for Servo Resonance

```cpp
// src/notch_filter.h - Remove servo resonance peaks

class NotchFilter {
private:
  // Tuned to servo resonance frequency (typically 8-12 Hz)
  float resonance_freq_hz = 10.0f;
  float q_factor = 10.0f;  // Sharpness of notch

  float b0, b1, b2, a1, a2;  // IIR coefficients
  float x1 = 0, x2 = 0, y1 = 0, y2 = 0;  // State variables

public:
  void setResonanceFrequency(float freq_hz) {
    resonance_freq_hz = freq_hz;
    updateCoefficients();
  }

  float filter(float sample) {
    float y = b0*sample + b1*x1 + b2*x2 - a1*y1 - a2*y2;

    x2 = x1;
    x1 = sample;
    y2 = y1;
    y1 = y;

    return y;
  }

private:
  void updateCoefficients() {
    // Calculate IIR notch filter coefficients
    // Based on Butterworth design
    float w0 = 2.0f * M_PI * resonance_freq_hz / SAMPLE_RATE_HZ;
    float alpha = sin(w0) / (2.0f * q_factor);

    b0 = 1.0f;
    b1 = -2.0f * cos(w0);
    b2 = 1.0f;
    a1 = -2.0f * cos(w0) * (1.0f + 2.0f*alpha) / (1.0f + 2.0f*alpha);
    a2 = (1.0f - 2.0f*alpha) / (1.0f + 2.0f*alpha);
  }
};
```

**Filter Activation:**
- Applied only if servo resonance detected (periodic oscillation >5Hz at constant frequency)
- Disabled for normal flight (added latency not needed)
- Can be tuned based on servo characterization tests

---

## 7. Test Plan (6.3.7)

### 7.1 Bench Unit Tests

```cpp
// test/unit/test_power_management.cpp

void test_power_consumption_per_mode() {
  PowerManager pm(mock_eeprom, mock_serial, mock_i2c);

  pm.setPowerMode(PowerManager::ACTIVE);
  float active_current = pm.getMetrics().current_consumption_ma;
  TEST_ASSERT_TRUE(active_current >= 150.0f && active_current <= 200.0f);

  pm.setPowerMode(PowerManager::COAST_OPTIMIZED);
  float coast_current = pm.getMetrics().current_consumption_ma;
  TEST_ASSERT_TRUE(coast_current >= 100.0f && coast_current <= 150.0f);

  pm.setPowerMode(PowerManager::RECOVERY);
  float recovery_current = pm.getMetrics().current_consumption_ma;
  TEST_ASSERT_TRUE(recovery_current >= 20.0f && recovery_current <= 50.0f);

  pm.setPowerMode(PowerManager::SLEEP);
  float sleep_current = pm.getMetrics().current_consumption_ma;
  TEST_ASSERT_TRUE(sleep_current <= 5.0f);
}

void test_battery_voltage_reading() {
  PowerManager pm(mock_eeprom, mock_serial, mock_i2c);

  // Mock 7.2V battery (2S LiPo)
  mock_adc->setVoltage(7.2f);
  float voltage = pm.getBatteryVoltage();
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 7.2f, voltage);
}

void test_battery_capacity_model() {
  PowerManager pm(mock_eeprom, mock_serial, mock_i2c);

  // 4.2V cell = 100% SOC
  float soc = pm.batteryCapacityModel(8.4f);
  TEST_ASSERT_EQUAL(100, (int)soc);

  // 3.7V cell = 50% SOC
  soc = pm.batteryCapacityModel(7.4f);
  TEST_ASSERT_TRUE(soc >= 40 && soc <= 60);

  // 3.0V cell = 0% SOC
  soc = pm.batteryCapacityModel(6.0f);
  TEST_ASSERT_TRUE(soc <= 5);
}

void test_gps_loss_fallback() {
  MockGPS gps;
  ApogeeDetector detector(&gps, mock_baro, mock_imu);

  gps.setHealthy(false);

  // Should still detect apogee with barometer + accel
  mock_baro->setApogeeVelocity();
  mock_imu->setApogeeAcceleration();

  bool detected = detector.detectApogeeWithoutGPS();
  TEST_ASSERT_TRUE(detected);
}

void test_sensor_saturation_handling() {
  IMUManager imu(mock_primary, mock_backup);

  // Primary sensor saturates
  mock_primary->setSaturation(true);
  mock_backup->setHealthy(true);

  imu.read();
  TEST_ASSERT_TRUE(imu.isUsingBackupSensor());
}

void test_eeprom_checksum() {
  EEPROMRecovery recovery(mock_eeprom);

  // Write valid state
  FlightState state = COAST;
  recovery.writeFlightStateWithChecksum(state);

  // Read back and verify
  TEST_ASSERT_TRUE(recovery.verifyChecksum());
}

void test_preflight_checker() {
  PreflightChecker checker(mock_imu, mock_eeprom, mock_sd,
                           mock_servo, mock_guidance);

  // All healthy
  mock_imu->setHealthy(true);
  mock_sd->setHealthy(true);

  PreflightResult result = checker.runFullCheck();
  TEST_ASSERT_EQUAL(0, result.failed_checks);
}

void test_thermal_throttling() {
  ThermalManager tm(mock_imu, mock_guidance, mock_power);

  mock_imu->setTemperature(75.0f);
  tm.update();
  TEST_ASSERT_EQUAL(ThermalManager::WARNING,
                    tm.getMetrics().current_state);
}

void test_servo_command_filter() {
  ServoCommandFilter filter;

  // Step input
  float filtered = filter.filterRollCommand(90.0f);
  TEST_ASSERT_TRUE(filtered < 90.0f);  // Smoothed

  // Second update should be closer
  float filtered2 = filter.filterRollCommand(90.0f);
  TEST_ASSERT_TRUE(filtered2 > filtered);  // Converging
}

void test_all_new_commands() {
  SystemStatusContext ctx;
  ctx.serial = mock_serial;

  // Test each command doesn't crash
  handlePreflight(ctx);
  handleTelemetryOn(ctx);
  handleTelemetryOff(ctx);
  handleServoTest(ctx);
  handlePyroTest(ctx);
  handleBattery(ctx);
  handlePowerMode(ctx);

  TEST_ASSERT_TRUE(mock_serial->messageCount() > 7);
}

void test_reboot_command() {
  SystemStatusContext ctx;
  ctx.serial = mock_serial;

  // Should not crash, watchdog will trigger
  handleReboot(ctx);

  // Verify "rebooting" message sent
  TEST_ASSERT_TRUE(mock_serial->containsString("rebooting"));
}
```

### 7.2 Flight Test Scenarios

```
╔════════════════════════════════════════════════════════════════════╗
║          Phase 6.3 Flight Test Matrix                              ║
╚════════════════════════════════════════════════════════════════════╝

TEST 1: Long Duration Flight (Battery Drain)
─────────────────────────────────────────────
Objective: Verify power consumption targets and flight time estimation
Motor:     L1390 (medium power, ~10 second burn)
Wind:      <5 mph
Battery:   2S 2000mAh LiPo, fully charged
Success:   ✓ Flight time matches estimate within ±20%
           ✓ Battery voltage monitored continuously
           ✓ No power-related errors in flight logs
Expected:  8-10 minute flight, land with >20% SOC

TEST 2: High Wind Conditions (Stability)
─────────────────────────────────────────
Objective: Verify guidance stability in windy conditions
Motor:     H54XX (medium-high power, ~8s burn)
Wind:      15-20 mph sustained
Battery:   2S 2200mAh LiPo
Success:   ✓ Guidance gains automatically reduced
           ✓ No excessive servo oscillation
           ✓ All attitude errors within limits
Expected:  Stable flight, normal apogee detection

TEST 3: GPS Loss Scenario (Sensor Redundancy)
──────────────────────────────────────────────
Objective: Verify barometric fallback and passive descent
Motor:     H123SS (similar energy to Test 1)
Wind:      <5 mph
Setup:     Disable GPS receiver before flight
Battery:   2S 2000mAh LiPo
Success:   ✓ Apogee detected by barometer+accel
           ✓ Drogue deployed at correct altitude
           ✓ No guidance attempted (passive descent)
           ✓ Flight logs show "GPS unavailable" event
Expected:  Normal flight profile despite GPS loss

TEST 4: High-G Boost (Sensor Saturation)
────────────────────────────────────────
Objective: Verify KX134 activation and high-G logging
Motor:     I218 (high power, ~12s burn, ~8G peak)
Wind:      <5 mph
Battery:   2S 2200mAh LiPo
Success:   ✓ Primary sensor (ICM) saturates detected
           ✓ Automatic failover to backup (KX134)
           ✓ High-G data properly logged
           ✓ Quaternion estimates remain valid
Expected:  Complete high-G profile captured

TEST 5: Extended Recovery Beacon (Power Modes)
───────────────────────────────────────────────
Objective: Verify recovery beacon and power modes after landing
Motor:     H45 (low power for short flight, fast land)
Wind:      <5 mph
Setup:     Enter RECOVERY mode manually after landing
Battery:   2S 2000mAh LiPo
Success:   ✓ System switches to RECOVERY mode (30mA)
           ✓ GPS beacon transmits every 30 seconds
           ✓ System can operate for >24 hours
Expected:  Locate via GPS beacon for 24+ hours

╔════════════════════════════════════════════════════════════════════╗
║ Success Criteria: All 5 tests pass with no critical errors         ║
╚════════════════════════════════════════════════════════════════════╝
```

---

## 8. Integration Points

### 8.1 PowerManager Integration into Main Loop

```cpp
// In src/TripleT_Flight_Firmware.cpp

void setup() {
  // Initialize all subsystems
  hal_init();
  serial->begin(230400);

  // ... other init ...

  // Initialize power manager
  power_manager = new PowerManager(eeprom, serial, i2c);
  power_manager->init();

  // Initialize thermal manager
  thermal_manager = new ThermalManager(imu_manager, guidance_control,
                                        power_manager);
}

void loop() {
  // Main loop runs at 10 Hz (100ms period)
  static uint32_t last_loop_ms = 0;
  uint32_t now = millis();

  if (now - last_loop_ms < 100) return;
  last_loop_ms = now;

  // 1. Update sensors
  imu_manager->update();
  barometer->update();
  gps->update();

  // 2. Power management (call every loop)
  power_manager->update();
  thermal_manager->update();

  // Handle power-triggered transitions
  if (power_manager->isBatteryCritical()) {
    setFlightState(LANDED);
    logEvent("BATTERY CRITICAL - force landing");
  }

  // 3. Flight logic
  updateFlightLogic();

  // 4. Apply power/thermal optimization
  applyPowerOptimizations();
  applyThermalThrottling();

  // 5. Data logging (respects power mode)
  logFlightData();

  // 6. Telemetry (respects power mode)
  sendTelemetry();
}
```

### 8.2 PreflightChecker Activation

```cpp
// Option 1: Manual activation via serial command "preflight"
// (Already implemented in command processor)

// Option 2: Automatic check at startup (optional)
void runAutoPreflightOnStartup() {
  #ifdef AUTO_PREFLIGHT_ENABLED
    logMessage("Running automatic pre-flight check...");
    PreflightResult result = preflight_checker->runFullCheck();

    if (result.failed_checks > 0) {
      logError("Pre-flight FAILED - system not ready");
      setFlightState(ERROR);
    } else if (result.warning_checks > 0) {
      logWarning("Pre-flight passed with warnings");
      setFlightState(PAD_IDLE);
    } else {
      logMessage("Pre-flight PASSED - system ready");
      setFlightState(PAD_IDLE);
    }
  #endif
}
```

### 8.3 Thermal Throttling with Guidance

```cpp
// In src/guidance_control.cpp

void GuidanceControl::setUpdateRate(uint8_t hz) {
  switch (hz) {
    case 10:  // Normal
      update_interval_ms = 100;
      break;
    case 5:   // Thermal warning
      update_interval_ms = 200;
      break;
    case 2:   // Thermal critical
      update_interval_ms = 500;
      break;
    default:
      break;
  }
}

bool GuidanceControl::shouldUpdateThisFrame() {
  return (millis() - last_update_ms) >= update_interval_ms;
}
```

### 8.4 Edge Case Handler Architecture

```cpp
// src/edge_cases/edge_case_handler.h

class EdgeCaseHandler {
public:
  EdgeCaseHandler(IMUManager* imu, GPSReceiver* gps,
                  BarometerSensor* baro, GuidanceControl* guidance,
                  ThermalManager* thermal, PowerManager* power);

  void update();

private:
  void checkGPSHealth();
  void checkWindConditions();
  void checkSensorSaturation();
  void checkEEPROMHealth();
  void checkThermalConditions();

  IMUManager* imu;
  GPSReceiver* gps;
  BarometerSensor* baro;
  GuidanceControl* guidance;
  ThermalManager* thermal;
  PowerManager* power;

  // State tracking
  GPSLossHandler gps_loss_handler;
  WindHandler wind_handler;
  EEPROMRecovery eeprom_recovery;
};

void EdgeCaseHandler::update() {
  checkGPSHealth();
  checkWindConditions();
  checkSensorSaturation();
  checkEEPROMHealth();
  checkThermalConditions();
}
```

---

## 9. Effort Estimate

### 9.1 Code Development

| Component | Lines | Time (days) | Difficulty |
|-----------|-------|-------------|------------|
| PowerManager class | 400 | 1.5 | Medium |
| Edge case handlers | 300 | 1.5 | Medium |
| PreflightChecker class | 350 | 1 | Low |
| New serial commands | 250 | 0.5 | Low |
| ThermalManager class | 200 | 0.5 | Low |
| Filter implementations | 200 | 0.5 | Low |
| Integration & testing | 0 | 2 | High |
| **TOTAL** | **1,700** | **7.5** | - |

### 9.2 Testing Timeline

| Phase | Duration | Effort |
|-------|----------|--------|
| Unit tests development | 1 day | 8 tests written + debugged |
| Unit test execution | 0.5 day | All 8 tests passing |
| Bench integration testing | 1 day | Subsystem integration verified |
| Flight test prep | 0.5 day | Hardware setup, safety checks |
| Flight tests (5 scenarios) | 3 days | 1 flight/day + analysis |
| **TOTAL** | **6 days** | - |

### 9.3 Total Phase 6.3 Timeline

**Week 1:**
- Mon-Tue: PowerManager + edge cases (2 days)
- Wed-Thu: PreflightChecker + commands (1.5 days)
- Fri: ThermalManager + filters (1 day)

**Week 2:**
- Mon: Integration into main loop (0.5 day)
- Tue-Wed: Unit test development + execution (1.5 days)
- Thu-Fri: Bench testing + flight prep (1 day)

**Week 3:**
- Mon-Wed: Flight test campaign (5 flights)
- Thu-Fri: Analysis, fixes, documentation

**Total: 2.5 weeks** (includes contingency buffer)

---

## 10. Success Criteria

| Criterion | Target | Verification |
|-----------|--------|--------------|
| Power consumption | 180→120→30→2 mA | Ammeter measurements |
| Edge case handling | 100% of 4 cases | Flight logs analysis |
| Pre-flight time | <5 minutes | Timed execution |
| All commands functional | 8/8 new commands | Manual testing |
| Thermal throttling | Works at thresholds | Lab temperature test |
| Flight reliability | 99% uptime | 5 successful flights |
| Test coverage | >95% code coverage | Coverage report |

---

## 11. Risk Mitigation

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| Power optimization regression | Medium | Medium | Regression test suite, before/after measurements |
| GPS loss handling failure | Low | High | Multiple unit tests, dedicated flight test |
| Thermal throttling bugs | Low | High | Lab oven testing before flight |
| Pre-flight timeout issues | Low | Medium | Timeout protection with 30s hard limit |
| Servo filter instability | Low | Medium | Gradual alpha tuning, benchtop testing |

---

## Appendix A: Configuration Parameters (in config.h)

```cpp
// Power Management
#define BATTERY_MIN_VOLTAGE 2.7f
#define BATTERY_LOW_THRESHOLD 3.0f
#define BATTERY_OK_THRESHOLD 3.3f
#define BATTERY_CAPACITY_MAH 2000
#define POWER_UPDATE_INTERVAL_MS 100

// Thermal Management
#define TEMP_WARNING_CELSIUS 70.0f
#define TEMP_CRITICAL_CELSIUS 85.0f
#define TEMP_SHUTDOWN_CELSIUS 95.0f

// GPS Loss Handling
#define GPS_LOSS_TIMEOUT_MS 5000
#define GPS_BEACON_INTERVAL_S 30

// Servo Filtering
#define SERVO_FILTER_ALPHA 0.3f
#define SERVO_DEADBAND_DEG 0.5f

// Pre-flight Checks
#define PREFLIGHT_CHECK_TIMEOUT_MS 30000

// Thermal Throttling
#define THERMAL_THROTTLE_UPDATE_RATE 5  // Hz
#define THERMAL_CRITICAL_UPDATE_RATE 2  // Hz
```

---

## Conclusion

Phase 6.3 production readiness implementation provides:

✅ **Power Efficiency:** 180mA → 2mA with intelligent mode selection
✅ **Robustness:** Edge case handling for GPS loss, high winds, sensor saturation
✅ **User Confidence:** Pre-flight verification in <5 minutes
✅ **Reliability:** Thermal management and multi-layer error recovery
✅ **Maintainability:** 8 new serial commands for diagnostics
✅ **Safety:** Comprehensive testing with 5 flight validation scenarios

**Estimated Completion:** 2.5 weeks
**Target Release:** v1.0.0-RC1 (Ready for release candidate testing)

Upon completion, TripleT will be production-ready for educational, research, and advanced hobby flights with confidence and reliability.
