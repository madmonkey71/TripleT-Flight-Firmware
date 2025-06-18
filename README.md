# TripleT Flight Firmware

An eventually comprehensive flight controller firmware for Teensy 4.1 microcontrollers, designed for model rockets and high-power rocketry applications with active guidance.

## Project Lead
**Matthew Thom** - Project Lead and Primary Developer

## Project Status - Beta (Ready for Controlled Test Flights)
**Current Version**: v0.48  
**Branch**: Beta

### Development Status Assessment (Based on Updated Gap Analysis)

#### ✅ PRODUCTION-READY FEATURES
- ✅ **Core Sensor Integration**: GPS, Barometer, ICM-20948 (9-DOF IMU), and KX134 (High-G Accelerometer) fully integrated and operational
- ✅ **Critical Safety Systems**: 
  - Sensor health monitoring integrated into flight state machine
  - Multi-method redundant apogee detection (barometric, accelerometer, GPS, backup timer)
  - Automatic error state transitions for sensor failures
- ✅ **Flight State Machine**: Complete 14-state flight state machine with robust state transitions and error handling
- ✅ **State Persistence & Recovery**: EEPROM-based state saving with power-loss recovery capabilities
- ✅ **Parachute Deployment**: Reliable pyro channel control with multiple deployment trigger methods
- ✅ **SD Card Data Logging**: Comprehensive CSV logging including GNC data (PID targets, integrals, outputs)
- ✅ **Interactive Serial Interface**: Feature-rich command system for diagnostics, calibration, and control
- ✅ **Web Interface**: Complete real-time data visualization with Web Serial API integration

#### ✅ BASIC FLIGHT CONTROL READY
- ✅ **PID Control System**: 3-axis PID controllers with state-based activation (BOOST/COAST only)
- ✅ **Attitude Hold**: Maintains orientation captured at motor burnout during coast phase
- ✅ **Actuator Integration**: PWM servo control with configurable mapping and limits

#### 🔴 HIGH PRIORITY - MISSING FEATURES
- ❌ **Live Telemetry**: Radio communication system not implemented (critical for operational flights)
- ❌ **Advanced Guidance**: Only basic attitude hold implemented; lacks trajectory following, gravity turns, wind compensation
- 🚧 **Sensor Fusion**: Kalman filter implementation paused; orientation accuracy limited

#### 🟡 MEDIUM PRIORITY - ENHANCEMENTS NEEDED
- 🚧 **Recovery System**: Basic implementation; lacks GPS beacon, audio locator, visual indicators
- 🚧 **Calibration Persistence**: Magnetometer calibration not saved between power cycles
- 🚧 **Enhanced Diagnostics**: Basic error handling; needs expanded error codes and recovery procedures

**Current Capability Assessment**: Ready for controlled test flights with basic guidance. Advanced operational flights require telemetry and enhanced guidance algorithms.

### Redundant Apogee Detection
To ensure the highest reliability for parachute deployment, the firmware now employs a multi-method apogee detection strategy. This system is designed to detect apogee accurately and within two seconds of the event, even in the case of a single sensor malfunction. Apogee is triggered if any of the following conditions are met:

1.  **Primary: Barometric Pressure:**
    *   **Method:** The primary and most sensitive method tracks the altitude from the MS5611 barometer. It records the maximum altitude reached during the flight.
    *   **Trigger:** Apogee is confirmed if the barometer registers a specific number of consecutive readings (`APOGEE_CONFIRMATION_COUNT`) that are lower than the maximum recorded altitude. This indicates the rocket has started its descent.

2.  **Secondary: Accelerometer Freefall:**
    *   **Method:** A secondary check uses the Z-axis of the ICM-20948 accelerometer. At apogee, the rocket experiences a brief period of near-zero gravity (freefall) as it transitions from upward to downward motion.
    *   **Trigger:** Apogee is confirmed if the accelerometer measures negative g-force on its vertical axis for a specific number of consecutive readings (`APOGEE_ACCEL_CONFIRMATION_COUNT`), indicating that the rocket is no longer accelerating upwards.

3.  **Tertiary: GPS Altitude:**
    *   **Method:** As a third layer of redundancy, the GPS module's altitude data is monitored. While typically having a slower update rate than the barometer, it provides an independent source of altitude information.
    *   **Trigger:** Similar to the barometer, if the GPS reports a number of consecutive altitude readings (`APOGEE_GPS_CONFIRMATION_COUNT`) lower than its previously recorded maximum, apogee is signaled. A 5-meter hysteresis is used to prevent triggering from GPS noise.

4.  **Failsafe: Backup Timer:**
    *   **Method:** A final, time-based failsafe ensures parachute deployment even if all other sensors fail to detect apogee. This timer starts at motor burnout (end of the `BOOST` phase).
    *   **Trigger:** If a pre-configured amount of time (`BACKUP_APOGEE_TIME_MS`, typically ~20 seconds) passes after motor burnout without any other method detecting apogee, the system will force an apogee event. This is a critical safety feature to prevent a total loss of the vehicle.

This layered approach ensures that the flight computer can reliably detect the peak of its flight and initiate recovery procedures under a wide range of conditions.

## State Machine
The firmware operates on a state machine that dictates the rocket's behavior throughout its flight, from startup to recovery.

### AI Assistance
This project utilizes AI assistance for:
- Code documentation and implementation
- System architecture design and review
- Gap analysis and project planning

## Overview

TripleT Flight Firmware is an open-source flight controller software built for the Teensy 4.1. It provides robust sensor integration, data logging, a complete flight state machine, and a PID-based control system framework designed for actively guided rockets.

## Hardware Requirements

- **Microcontroller**: Teensy 4.1
- **Sensors**:
  - SparkFun ZOE-M8Q GPS Module (I2C address 0x42)
  - MS5611 Barometric Pressure Sensor (I2C address 0x77)
  - SparkFun KX134 Accelerometer (I2C address 0x1F)
  - SparkFun ICM-20948 9-DOF IMU (I2C address 0x69)
- **Storage**: Micro SD Card (using the built-in SDIO on Teensy 4.1).
- **Actuators**: PWM Servo motors for control surfaces or Thrust Vector Control (TVC).
- **Other Components**:
  - Pyro channels for recovery system deployment.
  - Optional: Buzzer, WS2812 LEDs.

## Features

- **Multi-sensor Integration**: Fuses data from GPS, barometer, accelerometer, and 9-DOF IMU.
- **Flight State Machine**: A sophisticated 14-state machine manages the entire flight profile, from `PAD_IDLE` through `BOOST`, `COAST`, `APOGEE`, multiple `DESCENT` phases, and finally to `RECOVERY`. State transitions are handled automatically based on sensor data.
- **State Persistence & Recovery**: Automatically saves the flight state and critical data (max altitude, launch altitude) to EEPROM. In case of power loss, the firmware attempts to resume the flight in a safe state (e.g., resuming in `DROGUE_DESCENT` if power was lost during ascent).
- **PID-based Actuator Control**: A full 3-axis PID controller is implemented to manage hardware actuators (e.g., servos). The controller's core logic is in place.
  - Includes an Attitude Hold mode during the COAST phase, which maintains the orientation captured at motor burnout.
- **Audible Recovery Aid**: Audible buzzer sequence in RECOVERY state to aid in locating the rocket.
- **SD Card Logging**: Logs a comprehensive set of data points to a CSV file on the SD card, including sensor readings, flight state, and timestamps.
- **GPS/Barometer Calibration**: Calibrates the barometric altimeter using GPS data for accurate altitude-above-ground-level (AGL) readings.
- **Error Handling**: Includes watchdog timers, sensor initialization checks, and in-flight sensor health monitoring integrated with the state machine.
- **Interactive Serial Interface**: A command-driven system for real-time data monitoring, configuration, and diagnostics.

## Installation

1.  **Set up PlatformIO Environment**: Install PlatformIO, clone this repository, and open the project.
2.  **Libraries**: Ensure all libraries listed in `platformio.ini` are installed.
3.  **Hardware Connections**: Connect sensors via I2C, servos to their designated PWM pins, and insert an SD card.
4.  **Compile and Upload**: Select the `teensy41` environment and upload.

## Usage

### Basic Operation

On startup, the firmware initializes all hardware, performs calibrations, creates a new log file, and enters the `PAD_IDLE` state, ready for the `arm` command.

### Serial Commands

The firmware supports a rich set of serial commands for interaction:

| Command | Description |
|---|---|
| `arm` | Arms the flight computer, transitioning to the ARMED state to listen for liftoff. |
| `calibrate` / `h` | Manually triggers barometer calibration using GPS data. |
| `status_sensors` | Displays detailed status information for all connected sensors. |
| `b` | Shows a brief system status summary. |
| `f` | Shows SD card storage statistics. |
| `0` | Toggles continuous CSV data output over serial. |
| `1-9` | Toggle various debug output levels (system, IMU, GPS, etc.). |
| `help` / `a` | Shows the full list of available commands. |

### Data Logging

Data is logged to the SD card in CSV format. The log includes:
- Timestamp, Flight State (integer), GPS data, Barometric data, Accelerometer data, IMU data.
- Includes logging for the PID guidance system: target orientation, PID integral values, and final actuator outputs.

### Configuration

Key parameters are configured via `#define` statements in `src/config.h`:
- **Flight Logic**: `MAIN_DEPLOY_ALTITUDE`, `BOOST_ACCEL_THRESHOLD`, `COAST_ACCEL_THRESHOLD`, `APOGEE_CONFIRMATION_COUNT`.
- **Hardware Presence**: `DROGUE_PRESENT`, `MAIN_PRESENT`, `USE_KX134`.
- **PID Gains**: `PID_ROLL_KP`, `PID_ROLL_KI`, `PID_ROLL_KD` (and for Pitch/Yaw).
- **Actuator Pins**: `ACTUATOR_PITCH_PIN`, `ACTUATOR_ROLL_PIN`, `ACTUATOR_YAW_PIN`.
- **Servo Throws**: `SERVO_MIN_PULSE_WIDTH`, `SERVO_MAX_PULSE_WIDTH`.

## Documentation

For more detailed design information, please refer to:
- [System Documentation](docs/TripleT_Flight_Firmware_Documentation.md)
- [State Machine Design](State Machine.md)
- [Original Gap Analysis](GAP_ANALYSIS.md)
- [Updated Gap Analysis & Roadmap](UPDATED_GAP_ANALYSIS_2025.md) ⭐ **Current**

## Recent Changes

### Compilation Fixes (Latest)
Fixed several critical compilation errors that were preventing the firmware from building:

1. **Reference vs Pointer Error**: Fixed incorrect usage of address operator (&) when calling guidance functions. The functions expect references (`float&`) but were being passed pointers (`float*`). Changed:
   - `guidance_get_target_euler_angles(&logEntry.target_roll, ...)` → `guidance_get_target_euler_angles(logEntry.target_roll, ...)`
   - Similar fixes for `guidance_get_pid_integrals()` and `guidance_get_actuator_outputs()`

2. **Function Name Error**: Corrected the ICM-20948 initialization function call from `icm_20948_init()` to `ICM_20948_init()` to match the actual function declaration in the header file.

3. **Missing Closing Braces**: Added missing closing braces at the end of the `loop()` function that were causing syntax errors.

4. **Linker Error**: Implemented missing `icm_20948_get_mag()` function in `icm_20948_functions.cpp` that was declared in the header but not defined, causing a linker error.

5. **Unused Variable Warnings**: Cleaned up unused variables in the `loop()` function (`lastDisplayTime`, `lastDetailedTime`, `lastAccelReadTime`, `lastGPSCheckTime`, `lastStorageCheckTime`, `lastGuidanceUpdateTime`) that were causing compiler warnings.

6. **ERROR State Management Fixes**: 
   - **Missing Command Implementation**: Added the `clear_errors` command that was documented in help but not implemented
   - **Automatic Error Clearing**: Added logic to automatically clear ERROR state during startup if all systems are healthy
   - **State Persistence**: Both manual and automatic error clearing now save the cleared state to EEPROM
   - **Web Interface Sync**: Fixed issue where recovered ERROR state wasn't properly reflected in web interface state display

7. **CRITICAL: Serial CSV Output Fix**: 
   - **Root Cause**: The `loop()` function was never calling `WriteLogData()`, so no CSV data was being output even when `enableSerialCSV` was true
   - **Solution**: Added call to `WriteLogData()` when sensors are updated in the main loop
   - **Additional Fix**: Added call to `ProcessFlightState()` for proper flight state machine operation
   - **Impact**: Serial CSV output now works correctly and can be used with the web interface

8. **3D Visualization Not Updating**: Fixed critical issue where 3D position visualization in web interface wasn't updating:
   - **Root Cause**: When using Kalman filter (default), quaternions were never populated, causing web interface to show static orientation
   - **Solution**: Added `convertEulerToQuaternion()` function to convert Kalman filter Euler angles back to quaternions for logging and visualization
   - **Web Interface Mapping**: Updated CSV header mapping to match firmware output (`TgtRoll` vs `TargetRoll_rad`, `ActuatorOutRoll` vs `ActuatorX`)
   - **Filter Management**: Ensured Madgwick filter remains disabled (as requested) while Kalman filter provides proper orientation data
   - **Data Flow**: Now when `useKalmanFilter=true`, Euler angles come from Kalman filter and quaternions are calculated from those angles

9. **Compilation Warning Fixes**: Fixed remaining compiler warnings:
   - **Unused Variable**: Removed unused `systemHealthy` variable in `clear_errors` command implementation
   - **EEPROM Signature Overflow**: Changed `EEPROM_SIGNATURE_VALUE` from `0xDEADBEEF` (32-bit) to `0xBEEF` (16-bit) to match the `uint16_t` signature field in `FlightStateData` structure

These fixes ensure the firmware can now compile successfully for the Teensy 4.1 platform without errors or warnings, properly manage error state recovery and clearing, and most critically, the data logging/CSV output functionality now works properly.

### KX134 Sensor & 3D Visualization Fixes (Latest)
Fixed critical issues with KX134 high-G accelerometer data logging and 3D visualization:

1. **KX134 Data Logging Issue**: 
   - **Root Cause**: While `kx134_init()` was called during setup, `kx134_read()` was never called in the main loop, so KX134 data was never actually read or logged
   - **Solution**: Added `kx134_read()` call in the main loop when `kx134_initialized_ok` is true
   - **Flag Management**: Fixed `kx134_initialized_ok` flag to properly reflect initialization status based on `kx134_init()` return value

2. **3D Visualization Not Working**: 
   - **Root Cause**: Kalman filter was enabled by default but never initialized, so orientation data (Euler angles) remained at zero values
   - **Solution**: Added Kalman filter initialization in `setup()` function when `useKalmanFilter` is true and ICM-20948 is ready
   - **Impact**: 3D visualization now properly displays rocket orientation in real-time

3. **CRITICAL FIX: 3D Visualization Still Not Working (Follow-up)**
   - **Root Cause**: A follow-up investigation revealed that the `icm20948_ready` flag, which the Kalman filter depends on, was never set to `true`. Additionally, the `dt` (delta-time) calculation was misplaced, preventing the filter from updating.
   - **Solution**:
     - Set `icm20948_ready = true;` immediately after the ICM-20948 sensor is initialized.
     - Moved the `dt` calculation to the correct position within the main loop, right before the `kalman_predict` call.
   - **Impact**: The Kalman filter is now correctly initialized **and** updated every cycle, providing accurate, real-time orientation data and fully enabling the 3D visualization.

4. **Sensor Fusion Integration**: 
   - **KX134 Availability**: High-G accelerometer data is now properly logged and available for sensor fusion during high-G phases
   - **Orientation Data**: Both Kalman filter quaternions and Euler angles are now properly generated and logged
   - **Web Interface**: 3D visualizer now receives valid orientation data for real-time rocket attitude display

These fixes ensure that all sensor data is properly read, logged, and visualized, providing complete situational awareness for flight operations.

## Development Roadmap (Priority-Based)

### 🔴 Phase 1: Critical Flight Operations (1-2 months)
- [ ] **Live Telemetry System**: Implement radio communication for real-time flight monitoring (LoRa/XBee)
- [ ] **Advanced Guidance Algorithms**: Gravity turn maneuvers, trajectory following, wind compensation
- [ ] **Complete Sensor Fusion**: Finish Kalman filter implementation for improved orientation accuracy

### 🟡 Phase 2: System Enhancement (2-3 months)  
- [ ] **Enhanced Recovery System**: GPS beacon transmission, audio locator, LED strobe patterns
- [ ] **Persistent Calibration**: Save/load magnetometer calibration to EEPROM/SD card
- [ ] **Expanded Error Handling**: Specific error codes, recovery procedures, comprehensive health monitoring

### 🟢 Phase 3: Quality & Performance (Ongoing)
- [ ] **User Interface Improvements**: Command validation, configuration management, simulation mode
- [ ] **Documentation & Testing**: Complete API docs, automated testing, user manual
- [ ] **Performance Optimization**: Intelligent logging, memory optimization, profiling capabilities

### ✅ Recently Completed Major Features
- [x] **Critical Safety Systems**: Sensor health integration, redundant apogee detection with backup timer
- [x] **State-Based Guidance Control**: PID controllers active only during appropriate flight phases
- [x] **Enhanced Data Logging**: Complete GNC data logging for post-flight analysis and tuning
- [x] **Web Interface**: Real-time data visualization with Web Serial API integration
- [x] **Sensor Fusion & Calibration**: Kalman filter with dynamic accelerometer switching and persistent magnetometer calibration is complete.
- [x] **Compilation Fixes**: Resolved pointer/reference errors, function naming issues, and missing braces in main loop

**Estimated Timeline to Production-Ready**: 4-6 months  
**Current Status**: Ready for controlled test flights with basic guidance functionality

## Acknowledgements

This project was inspired by the work of Joe Barnard at BPS.Space.

## License
This project is licensed under the GNU General Public License v3.0 (GPL-3.0).

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.


