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

**Estimated Timeline to Production-Ready**: 4-6 months  
**Current Status**: Ready for controlled test flights with basic guidance functionality

## Acknowledgements

This project was inspired by the work of Joe Barnard at BPS.Space.

## License
This project is licensed under the GNU General Public License v3.0 (GPL-3.0).

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.


