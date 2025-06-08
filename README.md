# TripleT Flight Firmware

An eventually comprehensive flight controller firmware for Teensy 4.1 microcontrollers, designed for model rockets and high-power rocketry applications with active guidance.

## Project Lead
**Matthew Thom** - Project Lead and Primary Developer

## Project Status - Alpha
**Current Version**: v0.41

### Development Status (Reflecting Implemented Code)
- ✅ **Core Sensor Integration**: GPS, Barometer, ICM-20948 (9-DOF IMU), and KX134 (High-G Accelerometer) are integrated.
- ✅ **SD Card Data Logging**: Comprehensive CSV data logging to SD card is operational.
- ✅ **Interactive Serial Interface**: A rich command-driven interface for diagnostics and control is implemented.
- ✅ **Flight State Machine**: A full 14-state flight state machine is implemented, managing the rocket from startup to recovery.
- ✅ **State Persistence**: The current flight state and key altitude data are saved to EEPROM, allowing for recovery after a power loss.
- ✅ **Apogee Detection**: Primary apogee detection is implemented based on barometric altitude readings.
- ✅ **Parachute Deployment**: Pyro channels for drogue and main parachutes are controlled based on the flight state.
- ✅ **Actuator Control**: A 3-axis PID controller is implemented and connected to PWM servos.
- 🚧 **Guidance System**: A basic framework for guidance exists, but the current implementation is a placeholder (time-based yaw target). It is not yet integrated with the flight state machine.
- 🚧 **Sensor Fusion**: An orientation filter is in place, but development on it is currently paused.
- 🚧 **System Robustness**:
    - Sensor health checks exist but are not yet integrated into the flight state machine to trigger error states.
    - Redundant apogee detection (e.g., backup timer) is designed but not implemented in the flight logic.
- 🚧 **Enhanced Telemetry**: Live data transmission via radio is planned but not yet implemented.

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
- **PID-based Actuator Control**: A full 3-axis PID controller is implemented to manage hardware actuators (e.g., servos). The controller's core logic is in place, ready for a functional guidance engine to provide it with targets.
- **SD Card Logging**: Logs a comprehensive set of data points to a CSV file on the SD card, including sensor readings, flight state, and timestamps.
- **GPS/Barometer Calibration**: Calibrates the barometric altimeter using GPS data for accurate altitude-above-ground-level (AGL) readings.
- **Error Handling**: Includes watchdog timers and sensor initialization checks. (Note: In-flight sensor health monitoring is not yet fully integrated with the state machine).
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
- Timestamp, Flight State (integer), GPS data, Barometric data, Accelerometer data, and IMU data.
- **Note**: Logging for the PID guidance system (targets, errors, outputs) is planned but not yet implemented.

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
- [Gap Analysis](GAP_ANALYSIS.md)

## Potential Improvements and Future Work (Based on Gap Analysis)

- [ ] **Implement Functional Guidance:** Replace the placeholder guidance logic with a meaningful system, starting with a simple "Attitude Hold" mode during the coast phase.
- [ ] **Integrate Guidance with State Machine:** The PID control system should be active only during appropriate flight states (e.g., `BOOST`, `COAST`). In other states, actuators should be disabled or centered.
- [ ] **Fortify the State Machine:**
    - [ ] Integrate the existing sensor health checks into the main flight loop to trigger the `ERROR` state upon sensor failure.
    - [ ] Implement the designed backup apogee detection timer for redundancy.
- [ ] **Enhance Data Logging:** Add PID controller data (target orientation, integral values, actuator outputs) to the SD card log for post-flight tuning and analysis.
- [ ] **Live Telemetry:** Implement the planned live data transmission system using a radio module.
- [ ] **Persistent Magnetometer Calibration:** Save and load magnetometer calibration values to/from EEPROM or the SD card to avoid the need for recalibration or manual code changes.
- [ ] **Refine `isStationary` Detection:** The thresholds for detecting a stationary state may need tuning for different physical systems and environments.

## Acknowledgements

This project was inspired by the work of Joe Barnard at BPS.Space.

## License
This project is licensed under the GNU General Public License v3.0 (GPL-3.0).

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.


