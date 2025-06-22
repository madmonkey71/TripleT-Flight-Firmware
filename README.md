# TripleT Flight Firmware

**Current Version:** v0.48  
**Current State:** Beta  
**Last Updated:** January 2025

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

This firmware is designed for the **Teensy 4.1** microcontroller and provides comprehensive flight control capabilities for model rockets. The system manages all phases of flight from launch detection through recovery, with robust sensor fusion, data logging, and safety features.

## Key Features

- **Multi-Phase Flight Management**: Handles all flight phases from pad idle through recovery
- **Sensor Fusion**: Combines data from multiple sensors (ICM-20948 IMU, KX134 accelerometer, MS5611 barometer, GPS)
- **Dual Accelerometer Support**: Primary KX134 for high-G events, secondary ICM-20948 for attitude
- **Advanced Orientation Filtering**: Kalman filter with Madgwick filter fallback option
- **Redundant Apogee Detection**: Multiple methods including barometric, accelerometer, GPS, and backup timer
- **Configurable Parachute Deployment**: Support for single or dual-deploy configurations
- **Comprehensive Data Logging**: Real-time CSV logging to SD card with 50+ data points
- **Interactive Command Interface**: Serial commands for configuration, calibration, and diagnostics
- **Visual Status Indicators**: NeoPixel LEDs for flight state indication
- **Audio Feedback**: Buzzer patterns for different states and alerts
- **Error Recovery**: Automatic sensor health monitoring and recovery mechanisms
- **EEPROM State Persistence**: Flight state recovery after power loss

## Hardware Requirements

### Core Components
- **Teensy 4.1** microcontroller
- **ICM-20948** 9-DOF IMU (gyroscope, accelerometer, magnetometer)
- **KX134** high-G accelerometer (±64g range)
- **MS5611** barometric pressure sensor
- **u-blox GPS module** (e.g., ZOE-M8Q)
- **SD card** for data logging (built-in Teensy 4.1 slot)
- **NeoPixel LEDs** (2x for status indication)
- **Buzzer** for audio feedback
- **Pyrotechnic channels** for parachute deployment

### Pin Configuration
- **NeoPixel**: Pin 2
- **Buzzer**: Pin 9
- **Pyro Channel 1** (Drogue): Pin 2
- **Pyro Channel 2** (Main): Pin 3
- **I2C**: Pins 18 (SDA), 19 (SCL) - for sensors
- **GPS Serial**: Hardware serial port
- **SD Card**: Built-in SDIO interface

## Installation

1. **Set up PlatformIO Environment**: Install PlatformIO, clone this repository, and open the project.
2. **Libraries**: All required libraries are specified in `platformio.ini` and will be automatically installed.
3. **Hardware Connections**: Connect sensors via I2C, GPS via serial, pyro channels to designated pins, and insert an SD card.
4. **Compile and Upload**: Select the `teensy41` environment and upload to your Teensy 4.1.

## Usage

### Basic Operation

On startup, the firmware:
1. Initializes all hardware and sensors
2. Performs sensor health checks
3. Waits for GPS time sync and barometer calibration
4. Creates a new log file with timestamp
5. Enters `PAD_IDLE` state, ready for the `arm` command

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
| `set_orientation_filter [madgwick\|kalman]` | Sets the orientation filter type. |
| `clear_errors` | Manually clears error state if sensors have recovered. |

### Data Logging

Data is logged to the SD card in CSV format with timestamps. The log includes:
- **Flight State Information**: Current state, timestamps, state durations
- **Sensor Data**: All accelerometer, gyroscope, magnetometer, barometer, and GPS readings
- **Orientation Data**: Quaternions, Euler angles from both Kalman and Madgwick filters
- **Flight Metrics**: Altitude AGL, velocity, maximum altitude reached
- **System Status**: Sensor health, calibration status, error flags
- **Control Outputs**: Servo commands for attitude control (if enabled)

### Flight States

The firmware manages the following flight states:

1. **STARTUP**: Initial power-on and hardware initialization
2. **CALIBRATION**: Waiting for barometer calibration with GPS
3. **PAD_IDLE**: Ready state, waiting for arm command
4. **ARMED**: Armed and ready, monitoring for liftoff
5. **BOOST**: Motor burn phase, detecting burnout
6. **COAST**: Coasting to apogee, monitoring for peak altitude
7. **APOGEE**: Peak altitude reached, preparing for deployment
8. **DROGUE_DEPLOY**: Deploying drogue parachute
9. **DROGUE_DESCENT**: Descending under drogue
10. **MAIN_DEPLOY**: Deploying main parachute
11. **MAIN_DESCENT**: Descending under main parachute
12. **LANDED**: Touchdown confirmed
13. **RECOVERY**: Post-flight recovery mode with location beeper
14. **ERROR**: Error state with diagnostic information

## Configuration

Key configuration parameters can be modified in `src/config.h`:

### Flight Parameters
- `BOOST_ACCEL_THRESHOLD`: Liftoff detection threshold (default: 2.0g)
- `COAST_ACCEL_THRESHOLD`: Motor burnout detection (default: 0.5g)
- `MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M`: Main parachute deployment altitude (default: 100m AGL)
- `APOGEE_CONFIRMATION_COUNT`: Readings required to confirm apogee (default: 5)

### Hardware Configuration
- `DROGUE_PRESENT` / `MAIN_PRESENT`: Configure parachute deployment type
- `USE_KX134`: Enable/disable KX134 high-G accelerometer
- `BUZZER_OUTPUT`: Enable/disable buzzer functionality
- `NEOPIXEL_COUNT`: Number of status LEDs

### Safety Parameters
- `MAX_SENSOR_FAILURES`: Sensor failure threshold before error state
- `ERROR_RECOVERY_ATTEMPT_MS`: Time before attempting error recovery
- `BACKUP_APOGEE_TIME_MS`: Failsafe apogee detection timer

## Safety Features

- **Sensor Health Monitoring**: Continuous monitoring of all critical sensors
- **Redundant Apogee Detection**: Multiple independent methods to ensure reliable deployment
- **Error Recovery**: Automatic recovery from transient sensor failures
- **State Persistence**: Flight state saved to EEPROM for power-loss recovery
- **Backup Timers**: Failsafe mechanisms if primary detection methods fail
- **Comprehensive Logging**: Detailed data logging for post-flight analysis

## Development

### Project Structure
```
src/
├── TripleT_Flight_Firmware.cpp    # Main firmware file
├── config.h                       # Configuration parameters
├── flight_logic.cpp               # Flight state machine
├── command_processor.cpp          # Serial command handling
├── sensor functions/              # Individual sensor modules
├── utility_functions.cpp          # Helper functions
└── data_structures.h              # Data type definitions
```

### Adding New Features
1. Update configuration in `config.h` if needed
2. Implement functionality in appropriate module
3. Add command interface in `command_processor.cpp` if required
4. Update logging format in `log_format_definition.h` if adding new data
5. Test thoroughly with hardware-in-the-loop

## Troubleshooting

### Common Issues
- **Sensor Initialization Failures**: Check I2C connections and power supply
- **GPS Not Getting Fix**: Ensure clear sky view and allow time for cold start
- **SD Card Issues**: Verify card is formatted as FAT32 and has sufficient free space
- **Compilation Errors**: Ensure all libraries are installed via PlatformIO

### Debug Commands
- Use `status_sensors` to check sensor health
- Enable debug flags with numeric commands (1-9) for detailed output
- Check storage with `f` command
- Use `scan_i2c` to verify sensor connections

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Original inspiration from BPS.Space flight computer projects
- SparkFun and Adafruit for excellent sensor libraries
- PJRC for the powerful Teensy 4.1 platform
- PlatformIO for the excellent development environment

## Support

For issues, questions, or contributions, please use the GitHub repository's issue tracker and pull request system.

## Recent Changes (Latest)

### Watchdog Timer Removal and Compilation Fixes
- **Removed all watchdog timer functionality** as requested
- **Disabled ENABLE_WATCHDOG** in `src/config.h` 
- **Deleted Watchdog_t4 library** from `lib/` directory
- **Fixed multiple compilation errors** including:
  - Missing global variable declarations (`g_pixels`, `g_ms5611Sensor`)
  - SystemStatusContext member access issues
  - Function signature mismatches
  - Jump to case label errors in switch statements
  - Missing function parameters for `kalman_init()`
- **Re-enabled Madgwick filter configuration** macros in config.h
- **Updated command processor** to use correct pointer syntax for SystemStatusContext


