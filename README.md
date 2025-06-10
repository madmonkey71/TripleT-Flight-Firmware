# TripleT Flight Firmware

An eventually comprehensive flight controller firmware for Teensy 4.1 microcontrollers, designed for model rockets and high-power rocketry applications with active guidance.

## Project Lead
**Matthew Thom** - Project Lead and Primary Developer

## Project Status - Beta
**Current Version**: v0.47

### Development Status (Reflecting Implemented Code)
- ✅ **Core Sensor Integration**: GPS, Barometer, ICM-20948 (9-DOF IMU), and KX134 (High-G Accelerometer) are integrated.
- ✅ **SD Card Data Logging**: Comprehensive CSV data logging to SD card is operational. This now includes GNC data (PID targets, integrals, and outputs).
- ✅ **Interactive Serial Interface**: A rich command-driven interface for diagnostics and control is implemented.
- ✅ **Flight State Machine**: A full 14-state flight state machine is implemented, managing the rocket from startup to recovery.
- ✅ **State Persistence**: The current flight state and key altitude data are saved to EEPROM, allowing for recovery after a power loss.
- ✅ **Parachute Deployment**: Pyro channels for drogue and main parachutes are controlled based on the flight state.
- ✅ **Actuator Control**: A 3-axis PID controller is implemented and connected to PWM servos.
- ✅ **Web Interface**: A web-based interface for live data visualization is available and aligned with the current data logging format.
- 🚧 **Guidance System**: A basic framework for guidance exists, but the current implementation is a placeholder (time-based yaw target). It is not yet integrated with the flight state machine.
- 🚧 **Sensor Fusion**: An orientation filter is in place, but development on it is currently paused.
- ✅ **System Robustness**:
    - Sensor health checks exist but are not yet integrated into the flight state machine to trigger error states.
    - Redundant apogee detection (e.g., backup timer) is designed but not implemented in the flight logic.
    - **SD Card Handling**: The system now gracefully handles a missing SD card at startup by disabling logging and issuing a warning, preventing a boot failure.
- 🚧 **Enhanced Telemetry**: Live data transmission via radio is planned but not yet implemented.

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

This layered approach ensures that the flight computer can reliably detect the peak of its flight. Once apogee is confirmed by any of these methods, the system proceeds to activate the appropriate parachute deployment sequences (drogue and/or main) based on the flight configuration. This ensures timely recovery procedures under a wide range of conditions.

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
- **Flight State Machine**: A sophisticated 14-state machine manages the entire flight profile. State transitions are handled automatically based on sensor data and pre-configured thresholds:
    - **Liftoff (ARMED to BOOST)**: Transition occurs when the overall acceleration magnitude exceeds the configurable `BOOST_ACCEL_THRESHOLD` (default: 2.0g), indicating launch.
    - **Motor Burnout (BOOST to COAST)**: Transition occurs when the overall acceleration magnitude drops below the `BOOST_TO_COAST_ACCEL_DROP_THRESHOLD` (default: 1.0g), signaling that the motor has ceased thrusting and the vehicle is entering the coast phase.
    - **Apogee Detection & Recovery**: The system detects apogee when altitude stops increasing and begins to decrease (see "Redundant Apogee Detection" below). Upon apogee, parachute deployment sequences are initiated to ensure safe recovery.
- **State Persistence & Recovery**: Automatically saves the flight state and critical data (max altitude, launch altitude) to EEPROM. In case of power loss, the firmware attempts to resume the flight in a safe state (e.g., resuming in `DROGUE_DESCENT` if power was lost during ascent).
- **PID-based Actuator Control**: A full 3-axis PID controller is implemented to manage hardware actuators (e.g., servos). The controller's core logic is in place, ready for a functional guidance engine to provide it with targets.
- **SD Card Logging**: Logs a comprehensive set of data points to a CSV file on the SD card, including sensor readings, flight state, and timestamps. If the SD card is not present at startup, logging is automatically disabled, and the system proceeds without entering an error state.
- **GPS/Barometer Calibration**: Calibrates the barometric altimeter using GPS data for accurate altitude-above-ground-level (AGL) readings.
- **Error Handling**: Includes watchdog timers and sensor initialization checks. If the system enters an `ERROR` state (e.g., due to a sensor failure), the `clear_errors` command can be used to attempt a return to the `PAD_IDLE` state if the underlying issue is resolved. If power is cycled while the device is in an `ERROR` state, it will reboot into `STARTUP` to ensure a clean start.
- **Interactive Serial Interface**: A command-driven system for real-time data monitoring, configuration, and diagnostics.
- **Vehicle Orientation Detection (PAD_IDLE)**: Upon entering the `PAD_IDLE` state, the firmware attempts to determine the vehicle's vertical axis using accelerometer data (primarily from the ICM-20948). The axis (X, Y, or Z) that measures approximately +/-1g is identified as the vehicle's vertical orientation. This information, including the identified axis index (`verticalAxisIndex`) and the measured gravitational force (`verticalAxisMagnitudeG`), is logged for analysis and can aid in understanding pre-launch setup.

## Installation

1.  **Set up PlatformIO Environment**: Install PlatformIO, clone this repository, and open the project.
2.  **Libraries**: Ensure all libraries listed in `platformio.ini` are installed.
3.  **Hardware Connections**: Connect sensors via I2C, servos to their designated PWM pins, and insert an SD card.
4.  **Compile and Upload**: Select the `teensy41` environment and upload.

## Usage

### Basic Operation

On startup, the firmware initializes all hardware, performs calibrations, creates a new log file if an SD card is present, and enters the `PAD_IDLE` state, ready for the `arm` command.

### Serial Commands

The firmware supports a rich set of serial commands for interaction:

| Command | Description |
|---|---|
| `arm` | Arms the flight computer, transitioning to the ARMED state to listen for liftoff. |
| `calibrate` / `h` | Manually triggers barometer calibration using GPS data. |
| `clear_errors` | Attempts to clear an `ERROR` state and return to `PAD_IDLE`. This will only succeed if the underlying fault has been resolved. The new state is immediately pushed to the web interface. |
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
- **Flight Logic**:
    - `MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M`: Target altitude (AGL) for main parachute deployment.
    - `BOOST_ACCEL_THRESHOLD`: Minimum acceleration (in g) to detect liftoff (default: `2.0f`). This triggers the transition from `ARMED` to `BOOST`.
    - `BOOST_TO_COAST_ACCEL_DROP_THRESHOLD`: Acceleration (in g) below which motor burnout is assumed (default: `1.0f`). This triggers the transition from `BOOST` to `COAST`.
    - `COAST_ACCEL_THRESHOLD`: An older threshold related to motor burnout detection (default: `0.5f`). The primary logic for BOOST to COAST now uses `BOOST_TO_COAST_ACCEL_DROP_THRESHOLD`.
    - `APOGEE_CONFIRMATION_COUNT`: Number of consecutive sensor readings required to confirm apogee by various methods.
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

- [ ] **Implement Functional Guidance:** Replace the placeholder guidance logic with a meaningful system, starting with a "Attitude Hold" mode during the coast phase.
- [ ] **Integrate Guidance with State Machine:** The PID control system should be active only during appropriate flight states (e.g., `BOOST`, `COAST`). In other states, actuators should be disabled or centered.
- [ ] **Fortify the State Machine:**
    - [ ] Integrate the existing sensor health checks into the main flight loop to trigger the `ERROR` state upon sensor failure.
    - [ ] Implement the designed backup apogee detection timer for redundancy.
- [x] **Enhance Data Logging:** Add PID controller data (target orientation, integral values, actuator outputs) to the SD card log for post-flight tuning and analysis.
- [ ] **Live Telemetry:** Implement the planned live data transmission system using a radio module.
- [ ] **Persistent Magnetometer Calibration:** Save and load magnetometer calibration values to/from EEPROM or the SD card to avoid the need for recalibration or manual code changes.
- [ ] **Refine `isStationary` Detection:** The thresholds for detecting a stationary state may need tuning for different physical systems and environments.

## Acknowledgements

This project was inspired by the work of Joe Barnard at BPS.Space.

## License
This project is licensed under the GNU General Public License v3.0 (GPL-3.0).

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.


