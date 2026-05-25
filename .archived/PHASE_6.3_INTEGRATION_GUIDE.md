# Phase 6.3 Integration Guide: Architecture & Code Examples

**Purpose:** Detailed implementation guidance for integrating Phase 6.3 components into the existing codebase
**Audience:** Developers implementing PowerManager, ThermalManager, PreflightChecker, and edge case handlers
**Status:** Ready for implementation

---

## Table of Contents

1. [Main Loop Integration](#main-loop-integration)
2. [File Structure & Dependencies](#file-structure--dependencies)
3. [Initialization Sequence](#initialization-sequence)
4. [Power Mode Implementation](#power-mode-implementation)
5. [Edge Case Handler Architecture](#edge-case-handler-architecture)
6. [Command Processor Integration](#command-processor-integration)
7. [Data Logging Integration](#data-logging-integration)
8. [Telemetry Integration](#telemetry-integration)
9. [Testing Integration](#testing-integration)

---

## Main Loop Integration

### Current Main Loop (Phases 1-5)

```cpp
// src/TripleT_Flight_Firmware.cpp (BEFORE Phase 6.3)

void loop() {
  // Main loop runs at ~10 Hz (target 100ms period)
  static uint32_t last_loop_ms = 0;
  uint32_t now = millis();

  if (now - last_loop_ms < 100) return;
  last_loop_ms = now;

  // 1. Update sensors
  imu_manager->update();
  barometer->update();
  gps->update();

  // 2. Flight logic
  updateFlightLogic();

  // 3. Guidance control
  guidance_control->update();

  // 4. Data logging
  logFlightData();

  // 5. Telemetry
  sendTelemetryPacket();

  // 6. Watchdog kick
  watchdog->kick();
}
```

### Updated Main Loop (AFTER Phase 6.3)

```cpp
// src/TripleT_Flight_Firmware.cpp (AFTER Phase 6.3)

void loop() {
  static uint32_t last_loop_ms = 0;
  uint32_t now = millis();

  if (now - last_loop_ms < 100) return;
  last_loop_ms = now;

  // ═══════════════════════════════════════════════════════════════════
  // PHASE 6.3: Power & Thermal Management (MUST RUN EVERY LOOP)
  // ═══════════════════════════════════════════════════════════════════

  power_manager->update();        // Updates metrics, checks thresholds
  thermal_manager->update();      // Checks temperature, applies throttling

  // Emergency exit if power critical
  if (power_manager->isBatteryCritical()) {
    logEvent("BATTERY CRITICAL - forcing LANDED state");
    setFlightState(LANDED);
    return;  // Skip rest of loop
  }

  // ═══════════════════════════════════════════════════════════════════
  // SENSORS (respecting power mode throttling)
  // ═══════════════════════════════════════════════════════════════════

  if (imu_manager->shouldSample()) {  // 10Hz or 5Hz based on power mode
    imu_manager->update();
  }

  if (barometer->shouldSample()) {
    barometer->update();
  }

  if (gps->shouldSample()) {
    gps->update();
  }

  // ═══════════════════════════════════════════════════════════════════
  // EDGE CASE HANDLING (Phase 6.3 new)
  // ═══════════════════════════════════════════════════════════════════

  edge_case_handler->update();  // Monitors GPS, wind, saturation, etc.

  // ═══════════════════════════════════════════════════════════════════
  // FLIGHT LOGIC
  // ═══════════════════════════════════════════════════════════════════

  updateFlightLogic();

  // ═══════════════════════════════════════════════════════════════════
  // GUIDANCE CONTROL (respecting thermal throttling)
  // ═══════════════════════════════════════════════════════════════════

  if (guidance_control->shouldUpdateThisFrame()) {  // Throttled by temp
    guidance_control->update();
  }

  // ═══════════════════════════════════════════════════════════════════
  // DATA LOGGING (respecting power mode)
  // ═══════════════════════════════════════════════════════════════════

  if (shouldLogThisFrame()) {  // Interval depends on power mode
    logFlightData();
  }

  // ═══════════════════════════════════════════════════════════════════
  // TELEMETRY (respecting power mode)
  // ═══════════════════════════════════════════════════════════════════

  if (shouldSendTelemetryThisFrame()) {  // Decimated based on power mode
    sendTelemetryPacket();
  }

  // ═══════════════════════════════════════════════════════════════════
  // WATCHDOG
  // ═══════════════════════════════════════════════════════════════════

  watchdog->kick();
}
```

### Helper Functions

```cpp
// In src/TripleT_Flight_Firmware.cpp

static uint8_t telemetry_send_counter = 0;
static uint8_t logging_send_counter = 0;

bool shouldLogThisFrame() {
  PowerManager::PowerMode mode = power_manager->getCurrentMode();

  switch (mode) {
    case PowerManager::ACTIVE:
      // Log every 2 frames (every 200ms)
      logging_send_counter = (logging_send_counter + 1) % 2;
      return logging_send_counter == 0;

    case PowerManager::COAST_OPTIMIZED:
      // Log every 5 frames (every 500ms)
      logging_send_counter = (logging_send_counter + 1) % 5;
      return logging_send_counter == 0;

    case PowerManager::RECOVERY:
    case PowerManager::SLEEP:
      return false;  // No logging

    default:
      return true;  // Safety fallback
  }
}

bool shouldSendTelemetryThisFrame() {
  PowerManager::PowerMode mode = power_manager->getCurrentMode();
  PowerManager::PowerMetrics metrics = power_manager->getMetrics();

  // Disable if power explicitly turned off
  if (!metrics.telemetry_enabled) {
    return false;
  }

  switch (mode) {
    case PowerManager::ACTIVE:
      return true;  // Send every frame

    case PowerManager::COAST_OPTIMIZED:
      // Send every 2 frames (50% decimation)
      telemetry_send_counter = (telemetry_send_counter + 1) % 2;
      return telemetry_send_counter == 0;

    case PowerManager::RECOVERY:
    case PowerManager::SLEEP:
      return false;  // No telemetry

    default:
      return true;  // Safety fallback
  }
}
```

---

## File Structure & Dependencies

### New Files to Create

```
src/
├── power_management.h           NEW - PowerManager class definition
├── power_management.cpp         NEW - PowerManager implementation
│
├── thermal_management.h         NEW - ThermalManager class definition
├── thermal_management.cpp       NEW - ThermalManager implementation
│
├── preflight_checks.h           NEW - PreflightChecker class definition
├── preflight_checks.cpp         NEW - PreflightChecker implementation
│
├── edge_cases/                  NEW - Directory for edge case handlers
│   ├── edge_case_handler.h      NEW - Main coordinator
│   ├── gps_loss_handler.h       NEW - GPS loss fallback
│   ├── wind_handler.h           NEW - High wind compensation
│   └── eeprom_recovery.h        NEW - EEPROM corruption recovery
│
├── servo_smoother.h             NEW - Servo command filtering
└── gyro_filter.h                NEW - Gyro high-pass filter

test/unit/
├── test_power_management.cpp    NEW - PowerManager tests
├── test_thermal_management.cpp  NEW - ThermalManager tests
├── test_preflight_checks.cpp    NEW - PreflightChecker tests
├── test_edge_cases.cpp          NEW - Edge case handler tests
└── test_integration_phase63.cpp NEW - Integration tests

test/flight/
├── test_flight_1_long_duration.md   NEW - Flight test procedures
├── test_flight_2_high_wind.md
├── test_flight_3_gps_loss.md
├── test_flight_4_high_g.md
└── test_flight_5_extended_recovery.md
```

### Modified Files

```
src/
├── TripleT_Flight_Firmware.cpp  MODIFIED - Add power/thermal calls to main loop
├── command_processor.cpp        MODIFIED - Add 8 new serial commands
├── data_structures.h            MODIFIED - Add thermal_state to LogData
├── flight_logic.cpp             MODIFIED - Call edge_case_handler->update()
├── kalman_filter.h              MODIFIED - Update Q/R matrices with Phase 4 data
└── config.h                     MODIFIED - Add new config parameters

test/
├── mocks/mock_hal.h             MODIFIED - Add power mode support
└── unity_config.h               MODIFIED - Configure for extended tests
```

### Dependencies

```
PowerManager depends on:
  ├── IEEPROM (HAL interface)
  ├── ISerial (HAL interface)
  ├── II2C (HAL interface)
  └── IMUManager (for temperature reading)

ThermalManager depends on:
  ├── IMUManager (for temperature)
  ├── GuidanceControl (for update rate control)
  └── PowerManager (for mode control)

PreflightChecker depends on:
  ├── IMUManager (sensor health)
  ├── IEEPROM (EEPROM health)
  ├── ISDCard (storage health)
  ├── IServo (servo motion)
  ├── IGuidance (pyro channels)
  └── PowerManager (battery status)

EdgeCaseHandler depends on:
  ├── GPSReceiver (GPS loss detection)
  ├── BarometerSensor (fallback method)
  ├── IMUManager (saturation detection)
  ├── GuidanceControl (for gain adjustment)
  ├── ThermalManager (temperature monitoring)
  ├── PowerManager (for power state)
  └── IEEPROM (for corruption detection)
```

---

## Initialization Sequence

### Setup Function (Called Once)

```cpp
// src/TripleT_Flight_Firmware.cpp

// Global instances (declared at file scope)
PowerManager* power_manager = nullptr;
ThermalManager* thermal_manager = nullptr;
PreflightChecker* preflight_checker = nullptr;
EdgeCaseHandler* edge_case_handler = nullptr;

void setup() {
  // Initialize serial
  Serial.begin(230400);
  logMessage("TripleT Flight Computer v1.0.0 Starting...");

  // Initialize HAL (Phases 1-2)
  hal_init();
  delay(100);

  // Initialize sensors (Phases 1-2)
  imu_manager = new IMUManager(&mock_hal_i2c);
  barometer = new BarometerSensor(&mock_hal_i2c);
  gps = new GPSReceiver(&mock_hal_serial);

  // Phase 3: Flight logic and state machine
  flight_state = STARTUP;
  watchdog->enable(5000);  // 5 second watchdog

  // ═══════════════════════════════════════════════════════════════════
  // PHASE 6.3: INITIALIZATION
  // ═══════════════════════════════════════════════════════════════════

  // 1. Initialize PowerManager (MUST BE FIRST)
  logMessage("[6.3] Initializing PowerManager...");
  power_manager = new PowerManager(eeprom, serial, i2c);
  power_manager->init();

  if (!power_manager->isBatteryOK()) {
    logError("Battery voltage too low - system in ERROR state");
    setFlightState(ERROR);
    return;
  }

  // 2. Initialize ThermalManager
  logMessage("[6.3] Initializing ThermalManager...");
  thermal_manager = new ThermalManager(imu_manager, guidance_control,
                                       power_manager);

  // 3. Initialize PreflightChecker
  logMessage("[6.3] Initializing PreflightChecker...");
  preflight_checker = new PreflightChecker(imu_manager, eeprom, sd_card,
                                            servo_control, guidance_control);

  // 4. Initialize EdgeCaseHandler
  logMessage("[6.3] Initializing EdgeCaseHandler...");
  edge_case_handler = new EdgeCaseHandler(imu_manager, gps, barometer,
                                          guidance_control, thermal_manager,
                                          power_manager);

  // 5. Verify all critical systems
  logMessage("[6.3] Verifying critical systems...");
  if (!imu_manager->isPrimarySensorHealthy()) {
    logError("Primary IMU not responding - ERROR state");
    setFlightState(ERROR);
    return;
  }

  if (!barometer->isHealthy()) {
    logError("Barometer not responding - ERROR state");
    setFlightState(ERROR);
    return;
  }

  // Recovery from EEPROM if needed
  logMessage("[6.3] Checking EEPROM integrity...");
  if (!power_manager->verifyEEPROMChecksum()) {
    logWarning("EEPROM corruption detected - resetting to PAD_IDLE");
    setFlightState(PAD_IDLE);
  } else {
    // Try to recover saved state
    FlightState saved_state = readFlightStateFromEEPROM();
    if (saved_state != ERROR && saved_state != STARTUP) {
      logMessage("Recovered flight state from EEPROM: %s",
                 getFlightStateName(saved_state));
      setFlightState(saved_state);
    }
  }

  logMessage("[6.3] Phase 6.3 initialization complete - System READY");
  logMessage("Battery: %.2fV (%.0f%% capacity)",
             power_manager->getBatteryVoltage(),
             power_manager->getMetrics().battery_percent);
}
```

---

## Power Mode Implementation

### Detailed Mode Transitions

```cpp
// src/power_management.cpp

void PowerManager::setPowerMode(PowerMode new_mode) {
  if (new_mode == current_mode) {
    return;  // Already in this mode
  }

  logMessage("Power mode transition: %s → %s",
             modeToString(current_mode), modeToString(new_mode));

  uint32_t transition_start = millis();

  switch (new_mode) {
    case ACTIVE:
      transitionToActiveMode();
      break;

    case COAST_OPTIMIZED:
      transitionToCoastMode();
      break;

    case RECOVERY:
      transitionToRecoveryMode();
      break;

    case SLEEP:
      transitionToSleepMode();
      break;
  }

  current_mode = new_mode;
  mode_transition_ms = millis();

  logMessage("  Transition complete in %u ms",
             millis() - transition_start);
}

void PowerManager::transitionToActiveMode() {
  logMessage("  Enabling: Full sensor sampling (10Hz)");
  imu_manager->setSampleRate(10);

  logMessage("  Enabling: Full telemetry output");
  enableWebTelemetry();

  logMessage("  Enabling: SD card logging");
  enableSDCard();

  logMessage("  Setting CPU clock to 600 MHz");
  set_arm_clock(600000000);  // Teensy function
}

void PowerManager::transitionToCoastMode() {
  logMessage("  Reducing: Sensor sampling to 5Hz");
  imu_manager->setSampleRate(5);

  logMessage("  Decimating: Telemetry to 50%");
  telemetry_decimation = 2;

  logMessage("  Buffering: SD card writes (1s intervals)");
  sd_card_buffering = true;

  logMessage("  Reducing CPU clock to 400 MHz");
  set_arm_clock(400000000);

  current_consumption_ma = 120.0f;  // Updated estimate
}

void PowerManager::transitionToRecoveryMode() {
  logMessage("  Disabling: All telemetry");
  disableWebTelemetry();

  logMessage("  Disabling: SD card logging");
  disableSDCard();

  logMessage("  Disabling: Non-critical sensors");
  imu_manager->setSampleRate(0);  // Disable
  barometer->disable();

  logMessage("  Enabling: GPS beacon (every 30s)");
  enableRecoveryBeacon();

  logMessage("  Reducing CPU clock to 100 MHz");
  set_arm_clock(100000000);

  current_consumption_ma = 30.0f;  // Updated estimate
}

void PowerManager::transitionToSleepMode() {
  logMessage("  Disabling: All sensors and subsystems");
  imu_manager->setSampleRate(0);
  barometer->disable();
  gps->disable();
  sd_card->disable();

  logMessage("  Entering: Deep sleep mode");
  // Teensy deep sleep - only watchdog can wake
  // See Teensy 4.1 sleep modes documentation

  current_consumption_ma = 2.0f;  // Deep sleep estimate
}
```

### Subsystem Control

```cpp
// src/power_management.cpp

void PowerManager::disableWebTelemetry() {
  telemetry_enabled = false;
  serial->println("DEBUG: Telemetry disabled");
}

void PowerManager::enableWebTelemetry() {
  telemetry_enabled = true;
  serial->println("DEBUG: Telemetry enabled");
}

void PowerManager::disableSDCard() {
  sd_write_enabled = false;
  sd_card->flushAndClose();  // Ensure buffer flushed
  serial->println("DEBUG: SD card logging disabled");
}

void PowerManager::enableSDCard() {
  sd_write_enabled = true;
  sd_card->open();
  serial->println("DEBUG: SD card logging enabled");
}

void PowerManager::reduceSensorFrequency() {
  imu_manager->setSampleRate(5);  // 10Hz → 5Hz
  barometer->setSampleRate(5);
  gps->setSampleRate(1);
  serial->println("DEBUG: Sensor frequency reduced");
}

void PowerManager::normalSensorFrequency() {
  imu_manager->setSampleRate(10);  // Back to 10Hz
  barometer->setSampleRate(10);
  gps->setSampleRate(1);
  serial->println("DEBUG: Sensor frequency normal");
}
```

---

## Edge Case Handler Architecture

### Main Coordinator Class

```cpp
// src/edge_cases/edge_case_handler.h

class EdgeCaseHandler {
public:
  EdgeCaseHandler(IMUManager* imu, GPSReceiver* gps,
                  BarometerSensor* baro, GuidanceControl* guidance,
                  ThermalManager* thermal, PowerManager* power);

  void update();  // Call from main loop

  // Status queries
  bool isGPSLost() const;
  bool isHighWind() const;
  bool isSensorSaturating() const;
  bool isEEPROMCorrupted() const;

private:
  // Individual handlers
  void checkGPSHealth();
  void checkWindConditions();
  void checkSensorSaturation();
  void checkEEPROMHealth();
  void checkThermalConditions();

  // References to subsystems
  IMUManager* imu;
  GPSReceiver* gps;
  BarometerSensor* baro;
  GuidanceControl* guidance;
  ThermalManager* thermal;
  PowerManager* power;

  // Handler instances
  GPSLossHandler gps_loss;
  WindHandler wind_handler;
  EEPROMRecovery eeprom_recovery;

  // Tracking
  uint32_t last_update_ms;
  uint32_t gps_loss_start_ms;
  float estimated_wind_speed_ms;
};
```

### Update Function

```cpp
// src/edge_cases/edge_case_handler.cpp

void EdgeCaseHandler::update() {
  uint32_t now = millis();

  // Limit update frequency to 1 Hz (100ms)
  if (now - last_update_ms < 1000) {
    return;
  }
  last_update_ms = now;

  // Run all checks
  checkGPSHealth();
  checkWindConditions();
  checkSensorSaturation();
  checkEEPROMHealth();
  checkThermalConditions();
}

void EdgeCaseHandler::checkGPSHealth() {
  if (!gps->hasGPSFix()) {
    if (gps_loss_start_ms == 0) {
      gps_loss_start_ms = millis();
      logWarning("GPS fix lost");
    }

    uint32_t loss_duration = millis() - gps_loss_start_ms;
    if (loss_duration > 5000) {  // 5 second grace period
      // Activate GPS loss handler
      gps_loss.handleGPSLoss();
      guidance->disableTrajectoryFollowing();
      logEvent("GPS loss > 5s - trajectory disabled");
    }
  } else {
    if (gps_loss_start_ms != 0) {
      uint32_t loss_duration = millis() - gps_loss_start_ms;
      logMessage("GPS fix reacquired after %u ms", loss_duration);
      gps_loss_start_ms = 0;
      gps_loss.handleGPSReacquire();
    }
  }
}

void EdgeCaseHandler::checkWindConditions() {
  // Estimate wind from servo activity and gyro rates
  estimated_wind_speed_ms = wind_handler.estimateWindSpeed();

  WindHandler::WindSeverity severity = wind_handler.getWindSeverity();

  static WindHandler::WindSeverity previous_severity =
    WindHandler::CALM;

  if (severity != previous_severity) {
    logWarning("Wind condition changed: %s (%.1f m/s)",
               windSeverityToString(severity),
               estimated_wind_speed_ms);
    previous_severity = severity;

    // Apply compensation
    wind_handler.applyWindCompensation();
  }
}

void EdgeCaseHandler::checkSensorSaturation() {
  // Check if primary sensor is saturating
  if (imu->isPrimarySaturated()) {
    if (!imu->isUsingBackupSensor()) {
      logWarning("Primary sensor saturation detected - switching to backup");
      imu->switchToBackupSensor();
    }
  }

  // Check if both sensors saturated
  if (imu->isPrimarySaturated() && imu->isBackupSaturated()) {
    logError("BOTH sensors saturated - high-G event!");
    guidance->setHighGEvent(true);
    guidance->setMode(GuidanceControl::PASSIVE);
  }
}

void EdgeCaseHandler::checkEEPROMHealth() {
  // Periodic EEPROM health check (every 10 minutes)
  static uint32_t last_eeprom_check = 0;
  if (millis() - last_eeprom_check < 600000) {
    return;  // 10 minute interval
  }
  last_eeprom_check = millis();

  EEPROMRecovery::RecoveryState recovery =
    eeprom_recovery.checkEEPROMHealth();

  if (recovery.level != EEPROMRecovery::NO_CORRUPTION) {
    logWarning("EEPROM corruption detected: %s",
               recovery.error_description);
    // Don't force state change - just warn
  }
}

void EdgeCaseHandler::checkThermalConditions() {
  // Thermal monitoring delegated to ThermalManager
  // (called separately in main loop)
}
```

---

## Command Processor Integration

### New Command Registration

```cpp
// In src/command_processor.cpp

void processSerialCommand(const char* cmd,
                          const SystemStatusContext& context) {
  // Convert to lowercase
  char cmd_lower[64];
  strncpy(cmd_lower, cmd, sizeof(cmd_lower) - 1);
  for (char* p = cmd_lower; *p; ++p) *p = tolower(*p);

  // ─────────────────────────────────────────────────────────────────
  // Existing commands (Phases 1-5)
  // ─────────────────────────────────────────────────────────────────

  if (strcmp(cmd_lower, "arm") == 0) {
    handleArm(context);
    return;
  }

  if (strcmp(cmd_lower, "disarm") == 0) {
    handleDisarm(context);
    return;
  }

  if (strcmp(cmd_lower, "status_sensors") == 0) {
    handleStatusSensors(context);
    return;
  }

  // ... other existing commands ...

  // ─────────────────────────────────────────────────────────────────
  // PHASE 6.3: New commands
  // ─────────────────────────────────────────────────────────────────

  if (strcmp(cmd_lower, "preflight") == 0) {
    handlePreflight(context);
    return;
  }

  if (strcmp(cmd_lower, "telemetry_on") == 0) {
    handleTelemetryOn(context);
    return;
  }

  if (strcmp(cmd_lower, "telemetry_off") == 0) {
    handleTelemetryOff(context);
    return;
  }

  if (strcmp(cmd_lower, "servo_test") == 0) {
    handleServoTest(context);
    return;
  }

  if (strcmp(cmd_lower, "pyro_test") == 0) {
    handlePyroTest(context);
    return;
  }

  if (strcmp(cmd_lower, "load_trajectory") == 0) {
    handleLoadTrajectory(context);
    return;
  }

  if (strcmp(cmd_lower, "start_trajectory") == 0) {
    handleStartTrajectory(context);
    return;
  }

  if (strcmp(cmd_lower, "power_mode") == 0) {
    handlePowerMode(context);
    return;
  }

  if (strcmp(cmd_lower, "battery") == 0) {
    handleBattery(context);
    return;
  }

  if (strcmp(cmd_lower, "reboot") == 0) {
    handleReboot(context);
    return;
  }

  if (strcmp(cmd_lower, "help") == 0) {
    printHelpText(context);
    return;
  }

  // Unknown command
  context.serial->println("Unknown command. Type 'help' for available commands.");
}
```

---

## Data Logging Integration

### Updated LogData Structure

```cpp
// src/data_structures.h

struct LogData {
  // Existing fields from Phases 1-5
  uint32_t timestamp_ms;
  FlightState flight_state;
  float altitude_m;
  float velocity_ms;
  float acceleration_ms2;

  // ... other existing fields ...

  // PHASE 6.3: New fields for production monitoring
  float battery_voltage_v;        // Battery voltage at this sample
  float battery_capacity_percent; // Estimated SOC
  uint8_t power_mode;             // Current power mode (0-3)
  float imu_temperature_c;        // Temperature from IMU sensor
  uint8_t thermal_state;          // 0=normal, 1=warning, 2=critical
  int16_t wind_estimate_ms;       // Estimated wind speed
  uint8_t edge_case_flags;        // Bitmask: GPS lost, saturation, etc.
};
```

### CSV Header Generation

```cpp
// src/log_format_definition.cpp

const char* getCSVHeader() {
  static const char header[] =
    "timestamp_ms,"
    "flight_state,"
    "altitude_m,"
    "velocity_ms,"
    "acceleration_ms2,"
    // ... existing fields ...
    "battery_voltage_v,"         // NEW
    "battery_capacity_percent,"  // NEW
    "power_mode,"                // NEW
    "imu_temperature_c,"         // NEW
    "thermal_state,"             // NEW
    "wind_estimate_ms,"          // NEW
    "edge_case_flags\n";         // NEW

  return header;
}
```

### Logging Integration in Main Loop

```cpp
// In src/TripleT_Flight_Firmware.cpp - logFlightData()

void logFlightData() {
  // Populate existing fields
  log_data.timestamp_ms = millis();
  log_data.flight_state = flight_state;
  log_data.altitude_m = barometer->getAltitude();
  log_data.velocity_ms = flight_logic->getCurrentVelocity();

  // ────────────────────────────────────────────────────────────────
  // PHASE 6.3: Populate new fields
  // ────────────────────────────────────────────────────────────────

  PowerManager::PowerMetrics power_metrics = power_manager->getMetrics();
  log_data.battery_voltage_v = power_metrics.battery_voltage_v;
  log_data.battery_capacity_percent = power_metrics.battery_percent;
  log_data.power_mode = (uint8_t)power_metrics.current_mode;

  ThermalManager::ThermalMetrics thermal_metrics =
    thermal_manager->getMetrics();
  log_data.imu_temperature_c = thermal_metrics.current_temp_c;
  log_data.thermal_state = (uint8_t)thermal_metrics.current_state;

  log_data.wind_estimate_ms = edge_case_handler->getWindEstimateMS();

  // Set edge case flags
  log_data.edge_case_flags = 0;
  if (edge_case_handler->isGPSLost()) {
    log_data.edge_case_flags |= 0x01;  // Bit 0
  }
  if (edge_case_handler->isSensorSaturating()) {
    log_data.edge_case_flags |= 0x02;  // Bit 1
  }
  if (edge_case_handler->isEEPROMCorrupted()) {
    log_data.edge_case_flags |= 0x04;  // Bit 2
  }

  // Write to SD card (respecting power mode)
  uint32_t write_interval = power_manager->getSDCardWriteInterval();
  if (millis() - last_log_write_ms >= write_interval) {
    writeLogDataToSD(log_data);
    last_log_write_ms = millis();
  }
}
```

---

## Telemetry Integration

### Real-Time Telemetry Packet

```cpp
// src/TripleT_Flight_Firmware.cpp - sendTelemetryPacket()

void sendTelemetryPacket() {
  // Format: JSON for web interface parsing
  char buffer[512];

  PowerManager::PowerMetrics power_metrics = power_manager->getMetrics();
  ThermalManager::ThermalMetrics thermal_metrics =
    thermal_manager->getMetrics();

  int written = snprintf(buffer, sizeof(buffer),
    "{"
    "\"timestamp\":%lu,"
    "\"state\":%d,"
    "\"altitude\":%.2f,"
    "\"velocity\":%.2f,"
    "\"battery_v\":%.2f,"
    "\"battery_pct\":%.0f,"
    "\"temp_c\":%.1f,"
    "\"thermal_state\":%d,"
    "\"power_mode\":%d"
    "}\n",
    millis(),
    (int)flight_state,
    barometer->getAltitude(),
    flight_logic->getCurrentVelocity(),
    power_metrics.battery_voltage_v,
    power_metrics.battery_percent,
    thermal_metrics.current_temp_c,
    (int)thermal_metrics.current_state,
    (int)power_metrics.current_mode
  );

  if (written > 0 && written < (int)sizeof(buffer)) {
    Serial.print(buffer);
  }
}
```

---

## Testing Integration

### Unit Test Example

```cpp
// test/unit/test_power_management.cpp

#include <unity.h>
#include "../src/power_management.h"
#include "../test/mocks/mock_hal.h"

void setUp(void) {
  // Initialize mocks before each test
  mock_eeprom_reset();
  mock_serial_reset();
  mock_i2c_reset();
}

void tearDown(void) {
  // Cleanup after each test
}

void test_power_manager_initialization(void) {
  PowerManager pm(mock_eeprom, mock_serial, mock_i2c);
  pm.init();

  PowerManager::PowerMetrics metrics = pm.getMetrics();

  TEST_ASSERT_EQUAL(PowerManager::ACTIVE, metrics.current_mode);
  TEST_ASSERT_TRUE(metrics.current_consumption_ma > 0);
  TEST_ASSERT_TRUE(metrics.battery_voltage_v > 6.0f);
}

void test_power_mode_transition_active_to_coast(void) {
  PowerManager pm(mock_eeprom, mock_serial, mock_i2c);
  pm.init();

  // Transition to COAST
  pm.setPowerMode(PowerManager::COAST_OPTIMIZED);

  PowerManager::PowerMetrics metrics = pm.getMetrics();
  TEST_ASSERT_EQUAL(PowerManager::COAST_OPTIMIZED, metrics.current_mode);
  TEST_ASSERT_TRUE(metrics.current_consumption_ma < 150.0f);  // Less than ACTIVE
}

void test_battery_critical_threshold(void) {
  PowerManager pm(mock_eeprom, mock_serial, mock_i2c);
  pm.init();

  // Mock battery voltage to critical level
  mock_adc_set_voltage(2.6f);

  TEST_ASSERT_TRUE(pm.isBatteryCritical());
  TEST_ASSERT_FALSE(pm.isBatteryOK());
}

void test_estimated_flight_time(void) {
  PowerManager pm(mock_eeprom, mock_serial, mock_i2c);
  pm.init();

  // Battery at 60% SOC
  mock_adc_set_voltage(7.4f);

  float flight_time = pm.getEstimatedFlightTime();
  // 1200 mAh available / 180 mA = 6.67 minutes
  TEST_ASSERT_TRUE(flight_time >= 6.0f && flight_time <= 7.0f);
}

void test_all_tests(void) {
  RUN_TEST(test_power_manager_initialization);
  RUN_TEST(test_power_mode_transition_active_to_coast);
  RUN_TEST(test_battery_critical_threshold);
  RUN_TEST(test_estimated_flight_time);
}

int main(void) {
  UNITY_BEGIN();
  test_all_tests();
  UNITY_END();
  return 0;
}
```

---

## Configuration Parameters (in config.h)

```cpp
// ═════════════════════════════════════════════════════════════════
// PHASE 6.3: Production Readiness Configuration
// ═════════════════════════════════════════════════════════════════

// Power Management Thresholds
#define BATTERY_MIN_VOLTAGE 2.7f           // Critical shutdown
#define BATTERY_LOW_THRESHOLD 3.0f         // Switch to RECOVERY
#define BATTERY_OK_THRESHOLD 3.3f          // Switch to ACTIVE
#define BATTERY_CAPACITY_MAH 2000          // Assumed battery capacity
#define POWER_UPDATE_INTERVAL_MS 100       // Update every 10Hz

// Thermal Management Thresholds
#define TEMP_WARNING_CELSIUS 70.0f         // Start throttling
#define TEMP_CRITICAL_CELSIUS 85.0f        // Aggressive throttling
#define TEMP_SHUTDOWN_CELSIUS 95.0f        // Emergency shutdown

// GPS Loss Handling
#define GPS_LOSS_TIMEOUT_MS 5000           // Grace period before fallback
#define GPS_BEACON_INTERVAL_S 30           // Beacon frequency in RECOVERY mode

// Servo Filtering
#define SERVO_FILTER_ALPHA 0.3f            // Low-pass filter coefficient
#define SERVO_DEADBAND_DEG 0.5f            // Minimum command change

// Pre-Flight Checks
#define PREFLIGHT_CHECK_TIMEOUT_MS 30000   // 30 second hard limit

// Edge Case Thresholds
#define WIND_SPEED_STRONG_MPS 20.0f        // Wind speed for gain reduction
#define WIND_SPEED_EXTREME_MPS 30.0f       // Wind speed to disable guidance

// Sensor Saturation Thresholds
#define SENSOR_SATURATION_G 15.8f          // ICM-20948 saturation point
#define SENSOR_FALLBACK_DELAY_MS 100       // Delay before failover

// SD Card Configuration
#define SDCARD_WRITE_INTERVAL_ACTIVE_MS 500
#define SDCARD_WRITE_INTERVAL_COAST_MS 1000
#define SDCARD_BUFFER_SIZE 10              // Buffer 10 samples before write

// Telemetry Configuration
#define TELEMETRY_UPDATE_INTERVAL_ACTIVE_MS 100
#define TELEMETRY_DECIMATION_COAST 2       // Send every 2nd sample
```

---

## Verification Checklist

### Integration Verification Steps

- [ ] All new .h/.cpp files compile without errors
- [ ] PowerManager initializes in setup() without crashes
- [ ] Power mode transitions work (ACTIVE → COAST → RECOVERY)
- [ ] Edge cases detected (GPS loss, wind, saturation)
- [ ] Battery monitoring shows realistic voltage
- [ ] Thermal monitoring responds to temperature
- [ ] Pre-flight checker runs in <30 seconds
- [ ] All 8 new serial commands execute
- [ ] Data logging includes new fields
- [ ] Telemetry includes new fields
- [ ] Main loop still runs at 10Hz without deadlock
- [ ] Unit tests all pass (>95% coverage)

### Hardware Verification

- [ ] Serial monitor shows battery voltage changing
- [ ] Power consumption measured with ammeter matches targets
- [ ] Temperature increase detected when heating sensor
- [ ] GPS signal loss triggers fallback
- [ ] High-G event triggers sensor failover
- [ ] Wind handler responds to servo oscillation

---

## Next Steps

1. **Week 1:** Create file structure and implement PowerManager
2. **Week 2:** Implement ThermalManager, PreflightChecker, EdgeCaseHandler
3. **Week 3:** Integrate into main loop, run unit tests
4. **Week 4:** Flight test validation (5 scenarios)
5. **Release:** Tag v1.0.0-RC1

See **PRODUCTION_READINESS_PLAN.md** for complete implementation details.

---

**Document Version:** 1.0
**Created:** February 16, 2026
**Status:** Ready for Development
**Last Updated:** February 16, 2026
