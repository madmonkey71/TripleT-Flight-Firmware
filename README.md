# TripleT Flight Firmware

**Current Version:** v0.50  
**Current State:** Beta (Ready for Controlled Test Flights)
**Last Updated:** July 2025

## Project Lead
**Matthew Thom** - Project Lead and Primary Developer

### Recent Updates (v0.50)
- ✅ **Critical Bug Fixes**: Fixed multiple compilation errors including missing extern declarations, variable naming inconsistencies, and function structure issues.
- ✅ **Enhanced Recovery State**: Implemented SOS audible beacon pattern.
- ✅ **Enhanced Recovery State**: Added visual LED strobe pattern for location.
- ✅ **Enhanced Recovery State**: Added mock-up GPS coordinate serial beacon output.
- ✅ **Battery Voltage Monitoring**: Implemented basic battery voltage reading, logging, and serial debug output.
- ✅ **Kalman Filter as Primary**: Kalman filter is the sole orientation filter, integrating accelerometer, gyroscope, and magnetometer data. Yaw drift corrected using tilt-compensated magnetometer.
- ✅ **Magnetometer Calibration Persistence**: Bias and scale factors saved to EEPROM.
- ✅ **Dual Accelerometer Fusion**: Using KX134 for high-G and ICM-20948 otherwise.
- ✅ **Dynamic Main Parachute Deployment**: Altitude calculated dynamically.
- ✅ **Enhanced Error Recovery**: Automatic error recovery with grace period.
- ✅ **Comprehensive GNC Data Logging**: PID data logged.
- ✅ **Code Quality Improvements**: Fixed compilation errors, improved variable naming consistency, and ensured proper function structure organization.

### Development Status Assessment (Based on Updated Gap Analysis)

#### ✅ PRODUCTION-READY FEATURES
- ✅ **Core Sensor Integration**: GPS, Barometer, ICM-20948 (9-DOF IMU), and KX134 (High-G Accelerometer) fully integrated and operational.
- ✅ **Critical Safety Systems**: 
  - Sensor health monitoring integrated into flight state machine.
  - Multi-method redundant apogee detection (barometric, accelerometer, GPS, backup timer).
  - Automatic error state transitions for sensor failures with recovery mechanisms.
- ✅ **Flight State Machine**: Complete 14-state flight state machine with robust state transitions and error handling.
- ✅ **State Persistence & Recovery**: EEPROM-based state saving (`FlightStateData`) with power-loss recovery capabilities.
- ✅ **Parachute Deployment**: Reliable pyro channel control with multiple deployment trigger methods and dynamic main deployment altitude.
- ✅ **SD Card Data Logging**: Comprehensive CSV logging including detailed GNC data (PID targets, integrals, outputs) and Kalman filter orientation.
- ✅ **Interactive Serial Interface**: Feature-rich command system for diagnostics, calibration (including magnetometer), and control.
- ✅ **Web Interface**: Complete real-time data visualization with Web Serial API integration.
- ✅ **Orientation Filtering**: Robust Kalman filter implemented for 9-DOF sensor fusion, including magnetometer for yaw correction.
- ✅ **Magnetometer Calibration**: Interactive magnetometer calibration routine with EEPROM persistence.
- ✅ **Dual Accelerometer Strategy**: Intelligent switching between KX134 (high-G) and ICM-20948 accelerometers.
- ✅ **Recovery Beacon System**: Comprehensive recovery aids including SOS audio pattern, LED strobe, and GPS coordinate beacon.
- ✅ **Code Quality**: Clean, well-structured code with proper error handling and variable naming consistency.

#### ✅ BASIC FLIGHT CONTROL READY
- ✅ **PID Control System**: 3-axis PID controllers (configurable via `PID_ROLL_KP` etc.) with state-based activation (BOOST/COAST only).
- ✅ **Attitude Hold**: Maintains orientation captured at motor burnout (BOOST to COAST transition) using Kalman filter data.
- ✅ **Actuator Integration**: PWM servo control (`ACTUATOR_PITCH_PIN`, `ACTUATOR_ROLL_PIN`, `ACTUATOR_YAW_PIN`) with configurable mapping and limits.

#### 🔴 HIGH PRIORITY - MISSING FEATURES
- ❌ **Hardware reference design**: Hardware platform design is still being worked on. 
- ❌ **Live Telemetry**: Radio communication system not implemented (critical for operational flights). Plan exists for ESP32 bridge.
- ❌ **Advanced Guidance**: Only basic attitude hold implemented; lacks trajectory following, gravity turns, wind compensation.

#### 🟡 MEDIUM PRIORITY - ENHANCEMENTS NEEDED
- 🚧 **Sensor Fusion Validation**: Kalman filter and sensor fusion implemented, but orientation accuracy needs validation against known reference data (physical testing).
- 🚧 **Expanded Sensor Support**: Extend the platform to allow for a wider variety of sensor hardware and eventually the microprocessor platform (Long term)

**Current Capability Assessment**: Code quality issues resolved. Ready for controlled test flights with robust attitude hold, comprehensive recovery systems, and extensive data logging. Advanced operational flights require telemetry and more sophisticated stabilisation / guidance algorithms.

### Compilation Status
- ✅ **Fixed Critical Compilation Errors**: 
  - Added missing `extern ErrorCode_t g_last_error_code;` declarations in `command_processor.cpp` and `icm_20948_functions.cpp`
  - Corrected variable naming inconsistencies in SOS pattern implementation
  - Removed duplicate static variable declarations 
  - Fixed function structure issues (functions now properly defined outside of parent functions)
  - Corrected variable name typos in GPS altitude detection

### Recovery System Enhancements
The Recovery state now includes comprehensive location aids:

1. **SOS Audio Beacon**: Repeating SOS pattern (···---···) using buzzer at configurable frequency
2. **LED Strobe Pattern**: High-visibility strobe pattern for visual location assistance
3. **GPS Coordinate Beacon**: Serial output of GPS coordinates for recovery teams
4. **Configurable Timing**: All patterns have individually configurable timing constants

### Redundant Apogee Detection
To ensure the highest reliability for parachute deployment, the firmware employs a multi-method apogee detection strategy. This system is designed to detect apogee accurately, even in the case of a single sensor malfunction. Apogee is triggered if any of the following conditions are met:

1.  **Primary: Barometric Pressure (`APOGEE_CONFIRMATION_COUNT`):**
    *   Tracks maximum altitude from MS5611. Apogee if current altitude is consistently lower than max.
2.  **Secondary: Accelerometer Freefall (`APOGEE_ACCEL_CONFIRMATION_COUNT`):**
    *   Uses ICM-20948 Z-axis. Apogee if negative g-force (freefall) is detected consecutively.
3.  **Tertiary: GPS Altitude (`APOGEE_GPS_CONFIRMATION_COUNT`):**
    *   Monitors u-blox GPS altitude. Apogee if GPS altitude is consistently lower than its recorded max (with hysteresis).
4.  **Failsafe: Backup Timer (`BACKUP_APOGEE_TIME_MS`):**
    *   Time-based failsafe starting at motor burnout. Forces apogee if other methods fail within the configured time.

This layered approach ensures that the flight computer can reliably detect the peak of its flight and initiate recovery procedures under a wide range of conditions.

## State Machine
The firmware operates on a 14-state state machine (see `State Machine.md` and `src/data_structures.h`) that dictates the rocket's behavior throughout its flight, from startup to recovery.

## Overview

This firmware is designed for the **Teensy 4.1** microcontroller and provides comprehensive flight control capabilities for model rockets. The system manages all phases of flight from launch detection through recovery, utilizing a Kalman filter for sensor fusion, robust data logging, and extensive safety features.

## Key Features

- **Multi-Phase Flight Management**: Handles all 14 flight phases from pad idle through recovery.
- **Advanced Sensor Fusion**: Kalman filter combining data from ICM-20948 IMU (accel, gyro, mag), KX134 high-G accelerometer, MS5611 barometer, and u-blox GPS.
- **Dual Accelerometer Strategy**: Intelligently uses KX134 for high-G events (e.g., launch detection, Kalman input during high acceleration) and ICM-20948 for general flight and attitude determination.
- **Kalman Orientation Filtering**: Sole orientation filter providing roll, pitch, and tilt-compensated yaw from magnetometer data. Magnetometer calibration data is persisted to EEPROM.
- **Redundant Apogee Detection**: Four distinct methods: barometric, accelerometer, GPS, and a backup timer.
- **Configurable Parachute Deployment**: Supports single or dual-deploy configurations with dynamic calculation of main parachute deployment altitude based on ground level.
- **Comprehensive Recovery System**: SOS audio beacon, LED strobe patterns, and GPS coordinate transmission for post-flight recovery.
- **Comprehensive Data Logging**: Real-time CSV logging to SD card with 50+ data points, including detailed GNC (PID targets, integrals, actuator outputs), Kalman filter orientation, and battery voltage.
- **Interactive Command Interface**: Serial commands for configuration, calibration (gyro, magnetometer with persistence), diagnostics, and system control.
- **Visual Status Indicators**: NeoPixel LEDs for clear flight state indication, including recovery strobe pattern.
- **Audio Feedback**: Buzzer patterns for different states and alerts, including SOS beacon in `RECOVERY` state.
- **Robust Error Recovery**: Automatic sensor health monitoring, transition to `ERROR` state on critical failures, and automatic recovery attempts with a grace period. Manual recovery commands available.
- **EEPROM State Persistence**: Critical flight state information (`FlightStateData` struct) saved to EEPROM for recovery after power loss.
- **Battery Voltage Monitoring**: Reads and logs battery voltage for system health monitoring.

## Hardware Requirements

### Core Components
- **Teensy 4.1** microcontroller
- **ICM-20948** 9-DOF IMU (gyroscope, accelerometer, magnetometer)
- **KX134** high-G accelerometer (typically configured for ±64g range)
- **MS5611** barometric pressure sensor
- **u-blox GPS module** (e.g., ZOE-M8Q or similar, configured for UBX protocol)
- **SD card** for data logging (using Teensy 4.1 built-in SDIO slot)
- **NeoPixel LEDs** (2x configured via `NEOPIXEL_COUNT` and `NEOPIXEL_PIN`)
- **Buzzer** (connected to `BUZZER_PIN`)
- **Pyrotechnic channels** for parachute deployment (connected to `PYRO_CHANNEL_1`, `PYRO_CHANNEL_2`)
- **Servos** for attitude control (optional, connected to `ACTUATOR_PITCH_PIN`, `ACTUATOR_ROLL_PIN`, `ACTUATOR_YAW_PIN`)


### Pin Configuration
- **NeoPixel**: Pin `NEOPIXEL_PIN` (default: 2)
- **Buzzer**: Pin `BUZZER_PIN` (default: 9)
- **Pyro Channel 1** (Drogue): Pin `PYRO_CHANNEL_1` (default: 2)
- **Pyro Channel 2** (Main): Pin `PYRO_CHANNEL_2` (default: 3)
- **Actuator Pitch**: Pin `ACTUATOR_PITCH_PIN` (default: 21)
- **Actuator Roll**: Pin `ACTUATOR_ROLL_PIN` (default: 23)
- **Actuator Yaw**: Pin `ACTUATOR_YAW_PIN` (default: 20)
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
| `arm` | Arms the flight computer, transitioning to ARMED state. Checks sensor health for ARMED state requirements. |
| `calibrate` / `h` | Manually triggers barometer calibration using GPS data (requires GPS fix). |
| `calibrate_mag` | Starts interactive magnetometer calibration routine. |
| `calibrate_gyro` | Performs gyroscope bias calibration while stationary. |
| `save_mag_cal` | Saves current magnetometer calibration data to EEPROM. |
| `status_sensors` / `b` | Displays detailed status information for all connected sensors and system components. |
| `sd_status` / `f` | Shows SD card status, available space, and current log file. |
| `0` / `debug_serial_csv [on|off]` | Toggles continuous CSV data output over serial. |
| `1-6` / `debug_system [on|off]` etc. | Toggle specific debug output levels (system, IMU, GPS, baro, storage, ICM raw). |
| `help` / `a` | Shows the full list of available commands. |
| `set_orientation_filter kalman` | Sets the orientation filter type (Kalman is the only option). |
| `get_orientation_filter` | Shows the currently active orientation filter. |
| `clear_errors` | Manually attempts to clear `ERROR` state if sensors have recovered, transitioning to `PAD_IDLE`. |
| `clear_to_calibration` | Clears `ERROR` state to `CALIBRATION` if barometer needs calibration. |
| `sensor_requirements` | Displays sensor requirements for each flight state. |
| `scan_i2c` | Scans the I2C bus and lists detected devices. |
| `start_log` / `7` | Attempts to initialize SD card and start a new log file. |
| `status` | Alias for `status_sensors`. |
| `summary` / `j` | Toggles the periodic display of a status summary. |
| `debug_battery [on|off]` | Toggles periodic serial printing of battery voltage. |

### Data Logging

Data is logged to the SD card in CSV format with timestamps. The log (`logDataToString()` driven by `LOG_COLUMNS` in `log_format_definition.h`) includes:
- **Flight State Information**: Current state (`FlightState`), timestamps, sequence number.
- **Sensor Data**: Raw and calibrated barometer altitude, pressure, temperature. KX134 and ICM-20948 accelerometer, gyroscope, magnetometer readings, and ICM temperature.
- **GPS Data**: Latitude, longitude, altitude (ellipsoid and MSL), speed, heading, fix type, satellites, pDOP, RTK status.
- **Orientation Data (Kalman Filter)**: Quaternions (q0-q3), Euler angles (roll, pitch, yaw in radians), gyroscope biases.
- **Battery Voltage**: Measured battery voltage (`battery_voltage`).
- **Flight Metrics**: `g_launchAltitude`, `g_maxAltitudeReached`, `currentAglAlt` (calculated).
- **GNC Data**: Target Euler angles (roll, pitch, yaw), PID integral terms for each axis, and final actuator outputs for roll, pitch, and yaw.
- **System Status**: Implicit through logged data and flight state.

### Flight States

The firmware manages the following 14 flight states (defined in `src/data_structures.h`):

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
- `BOOST_ACCEL_THRESHOLD`: Liftoff detection threshold (default: 2.0g).
- `COAST_ACCEL_THRESHOLD`: Motor burnout detection (default: 0.5g).
- `MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M`: Default height in meters AGL for main parachute deployment if dynamic calculation fails or for reference (default: 100m). Actual deployment uses `g_main_deploy_altitude_m_agl`.
- `APOGEE_CONFIRMATION_COUNT`: Barometer readings to confirm apogee (default: 5).
- `APOGEE_ACCEL_CONFIRMATION_COUNT`: Accelerometer readings for apogee (default: 5).
- `APOGEE_GPS_CONFIRMATION_COUNT`: GPS readings for apogee (default: 3).
- `BACKUP_APOGEE_TIME_MS`: Failsafe apogee timer after boost (default: 20s).

### Hardware Configuration
- `DROGUE_PRESENT` / `MAIN_PRESENT`: Configure parachute deployment type (e.g., `true`/`false`).
- `USE_KX134`: Enable/disable KX134 high-G accelerometer (default: 1, enabled).
- `BUZZER_OUTPUT`: Enable/disable buzzer functionality (default: 1, enabled).
- `NEOPIXEL_COUNT`: Number of status LEDs (default: 2).
- `PYRO_CHANNEL_1`, `PYRO_CHANNEL_2`: Pins for pyro deployment.
- `ACTUATOR_PITCH_PIN`, `ACTUATOR_ROLL_PIN`, `ACTUATOR_YAW_PIN`: Pins for servo actuators.

### PID Controller Gains (Example)
- `PID_ROLL_KP`, `PID_ROLL_KI`, `PID_ROLL_KD`: Gains for roll axis.
- `PID_PITCH_KP`, `PID_PITCH_KI`, `PID_PITCH_KD`: Gains for pitch axis.
- `PID_YAW_KP`, `PID_YAW_KI`, `PID_YAW_KD`: Gains for yaw axis.

### Recovery Beacon & GPS
- `RECOVERY_BEACON_SOS_DOT_MS`, `_DASH_MS`, `_SYMBOL_PAUSE_MS`, `_LETTER_PAUSE_MS`, `_WORD_PAUSE_MS`: Timings for SOS buzzer.
- `RECOVERY_BEACON_FREQUENCY_HZ`: Buzzer frequency for SOS.
- `RECOVERY_STROBE_ON_MS`, `_OFF_MS`, `_BRIGHTNESS`, `_R`, `_G`, `_B`: Parameters for LED recovery strobe.
- `RECOVERY_GPS_BEACON_INTERVAL_MS`: Interval for serial GPS beacon mock-up.

### Battery Monitoring
- `ENABLE_BATTERY_MONITORING`: `1` to enable, `0` to disable.
- `BATTERY_VOLTAGE_PIN`: Analog pin for voltage sensing (e.g., `A7`).
- `ADC_REFERENCE_VOLTAGE`: ADC reference (e.g., `3.3f` for Teensy 4.1).
- `ADC_RESOLUTION`: ADC resolution (e.g., `1024.0f` for 10-bit).
- `VOLTAGE_DIVIDER_R1`, `VOLTAGE_DIVIDER_R2`: Resistor values for voltage divider.
  - **Voltage Divider Construction**: Connect Battery(+) to R1. Connect the junction of R1 and R2 to `BATTERY_VOLTAGE_PIN`. Connect R2 to Ground.
  - `V_battery = V_adc * (R1 + R2) / R2`. Ensure `V_adc` (voltage at the pin) does not exceed `ADC_REFERENCE_VOLTAGE`.
- `BATTERY_VOLTAGE_READ_INTERVAL_MS`: Read and print interval.


### Safety & EEPROM
- `MAX_SENSOR_FAILURES`: Sensor failure threshold before `ERROR` state (default: 3).
- `EEPROM_STATE_ADDR`: Address for `FlightStateData` struct (default: 0).
- `EEPROM_SIGNATURE_VALUE`: Signature for EEPROM data validity (default: 0xBEEF).
- `MAG_CAL_EEPROM_ADDR`: EEPROM address for magnetometer calibration data.

## Error Recovery System

The firmware includes comprehensive automatic sensor health monitoring and recovery mechanisms, primarily managed in `src/flight_logic.cpp` and `src/command_processor.cpp`:

- **Automatic Health Checks**: Continuous monitoring of critical sensors. If `isSensorSuiteHealthy()` (in `src/utility_functions.cpp`) reports a failure for the current operational state (excluding `LANDED`, `RECOVERY`, `ERROR`), the system transitions to the `ERROR` state.
- **Automatic Error Recovery**: While in the `ERROR` state, the system periodically checks (every 2 seconds via `autoRecoveryCheckInterval`) if sensor health has been restored (suitable for `PAD_IDLE` or `CALIBRATION`).
  - If healthy for `PAD_IDLE` (all systems go, barometer calibrated), transitions to `PAD_IDLE`.
  - If healthy for `CALIBRATION` (barometer initialized but needs calibration), transitions to `CALIBRATION`.
  - Recovery is prevented if critical hardware (e.g., barometer) is not initialized.
- **Manual Recovery Options**:
  - `clear_errors`: Attempts to transition from `ERROR` to `PAD_IDLE` if health checks pass.
  - `clear_to_calibration`: Attempts to transition from `ERROR` to `CALIBRATION` if the barometer is initialized but needs calibration.
- **Grace Period Protection**: A 5-second grace period (`errorClearGracePeriod`) is activated after clearing an error (manually or automatically) to prevent immediate re-entry into the `ERROR` state due to transient sensor fluctuations.
- **State Persistence**: The `FlightStateData` struct (including `g_main_deploy_altitude_m_agl`) is saved to EEPROM via `saveStateToEEPROM()` (in `src/state_management.cpp`) to survive power cycles and support recovery scenarios defined in `recoverFromPowerLoss()`.
- **Diagnostic Tools**: `isSensorSuiteHealthy()` provides verbose output for troubleshooting. Serial commands like `status_sensors` and debug flags offer detailed insights.

### Error State Behavior

When in `ERROR` state:
- System performs continuous health monitoring for automatic recovery.
- LED shows red error indication (managed by `setFlightStateLED` in `src/flight_logic.cpp`).
- Buzzer emits a fast beeping pattern (if `BUZZER_OUTPUT` enabled).
- Web interface (if connected) should reflect the `ERROR` state.
- Serial output provides diagnostic information if system debug is enabled.

### Recovery Process (Automatic)

1. **Detection**: `isSensorSuiteHealthy()` identifies if conditions for `PAD_IDLE` or `CALIBRATION` are met.
2. **State Decision**: System determines the appropriate target state.
3. **Transition**: Automatically transitions to `PAD_IDLE` or `CALIBRATION`.
4. **Grace Period**: Activates the 5-second grace period.
5. **Confirmation**: System logs the recovery and updates status indicators.

Key constants involved: `errorCheckInterval`, `autoRecoveryCheckInterval`, `errorClearGracePeriod`.

## Safety Features

- **Sensor Health Monitoring**: Continuous checks via `isSensorSuiteHealthy()`.
- **Redundant Apogee Detection**: Four independent methods (barometric, accelerometer, GPS, timer).
- **Error Recovery**: Automatic and manual recovery from `ERROR` state with grace period.
- **State Persistence**: Flight state (`FlightStateData`) saved to EEPROM for power-loss recovery.
- **Backup Timers**: Failsafe apogee detection timer (`BACKUP_APOGEE_TIME_MS`).
- **Comprehensive Logging**: Detailed data logging (`LogData` struct) for post-flight analysis.
- **Dynamic Main Deployment Altitude**: Calculates main chute deployment altitude AGL at arming.

## Development

### Project Structure
```
src/
├── TripleT_Flight_Firmware.cpp    # Main firmware file, setup(), loop()
├── config.h                       # Centralized configuration parameters
├── constants.h                    # Global constants
├── flight_logic.cpp               # Core flight state machine logic (ProcessFlightState, detectApogee, etc.)
├── state_management.cpp           # EEPROM state saving and recovery (saveStateToEEPROM, recoverFromPowerLoss)
├── command_processor.cpp          # Serial command parsing and handling
├── gps_functions.cpp              # u-blox GPS module interaction
├── ms5611_functions.cpp           # MS5611 barometer interaction
├── icm_20948_functions.cpp        # ICM-20948 IMU interaction (includes mag calibration)
├── kx134_functions.cpp            # KX134 accelerometer interaction
├── kalman_filter.cpp              # Kalman filter implementation for orientation
├── guidance_control.cpp           # PID controllers and guidance logic
├── utility_functions.cpp          # Helper functions (I2C scan, debug prints, etc.)
├── data_structures.h              # Struct definitions (LogData, FlightStateData)
├── log_format_definition.h        # Defines the structure of the CSV log file
└── debug_flags.h                  # Debug flag definitions
```

### Adding New Features
1. Define any new configurations in `src/config.h` or `src/constants.h`.
2. Implement core logic in a relevant existing module (e.g., `flight_logic.cpp` for state changes) or create a new `.cpp`/`.h` pair in `src/` for significant new functionality.
3. If user interaction is needed, add commands to `src/command_processor.cpp` and update `printHelpMessage()`.
4. If new data needs to be logged, update the `LogData` struct in `src/data_structures.h` AND the `LOG_COLUMNS` array in `src/log_format_definition.h` to maintain synchronization.
5. Test thoroughly, utilizing serial commands and debug flags for verification.

## Troubleshooting

### Common Issues
- **Sensor Initialization Failures**: Use `scan_i2c` to check connections. Verify power and I2C pull-up resistors.
- **GPS Not Getting Fix**: Ensure clear sky view. Allow time for cold start (can be several minutes). Check antenna connection. Use `debug_gps on` for u-blox messages.
- **SD Card Issues**: Verify card is FAT32 formatted. Check `sd_status` for available space and errors. Ensure `DISABLE_SDCARD_LOGGING` in `config.h` is `false`.
- **Compilation Errors**: Ensure all libraries in `platformio.ini` are correctly installed/updated. Check for missing includes or incorrect function signatures.
- **Unexpected State Transitions**: Enable `debug_system on` to trace state changes and sensor readings that might be causing them. Review `isSensorSuiteHealthy()` logic.

### Debug Commands
- Use `status_sensors` (or `b`) to check overall sensor health and initialization status.
- Enable specific debug flags (e.g., `debug_imu on`, `debug_gps on`, `debug_baro on`) for detailed output from sensor modules.
- Use `debug_serial_csv on` (or `0`) to see raw data being logged.
- Check SD card status and free space with `sd_status` (or `f`).
- Use `scan_i2c` to verify I2C device detection and addresses.
- Use `sensor_requirements` to understand what sensors are needed for each flight state.
- If in `ERROR` state, `clear_errors` or `clear_to_calibration` can be used after addressing the underlying issue.

## License

This project is licensed under the MIT License - see the `LICENSE` file (if present, otherwise assume MIT).

## Acknowledgments

- Original inspiration from BPS.Space flight computer projects.
- SparkFun and Adafruit for excellent sensor libraries and example code.
- PJRC (Paul Stoffregen) for the powerful Teensy 4.1 platform and core libraries.
- PlatformIO for the excellent development environment.

## Support

For issues, questions, or contributions, please use the GitHub repository's issue tracker and pull request system.

## Recent Changes (Reflects codebase state around v0.48)

### Key Enhancements & Fixes:
- **Kalman Filter**: Now the sole, robust orientation filter using accelerometer, gyroscope, and magnetometer. Includes tilt-compensation for yaw.
- **Magnetometer Calibration Persistence**: Calibration data saved to and loaded from EEPROM.
- **Dual Accelerometer Logic**: Uses KX134 for high-G scenarios and ICM-20948 otherwise, integrated into Kalman filter input.
- **Dynamic Main Deployment Altitude**: `g_main_deploy_altitude_m_agl` calculated at ARMED state.
- **GNC Data Logging**: Target orientation, PID integrals, and actuator outputs are now part of the CSV log.
- **Error Handling**: Automatic recovery from `ERROR` state implemented with a grace period. `clear_errors` and `clear_to_calibration` commands refined.
- **State Management**: `FlightStateData` in EEPROM includes `mainDeployAltitudeAgl`. `recoverFromPowerLoss` logic updated.
- **Command Processor**: Added commands for magnetometer (`calibrate_mag`, `save_mag_cal`) and gyroscope (`calibrate_gyro`) calibration.
- **Refactoring**: Significant code organization, moving functions to appropriate modules (e.g., `utility_functions.cpp`, `state_management.cpp`). Global variable usage reduced by passing context/objects as parameters where feasible.
- **Documentation**: Ongoing updates to align with current codebase.

### Architectural Changes:
- **Centralized Configuration**: `config.h` and `constants.h` for parameters.
- **Modular Design**: Clearer separation of concerns across different `.cpp` files (sensors, flight logic, state management, command processing, etc.).
- **Data Structures**: Defined `LogData` and `FlightStateData` in `data_structures.h`.
- **Logging Definition**: `log_format_definition.h` now drives CSV header and `logDataToString()` structure.

### Known Issues / Areas for Improvement (from `UPDATED_GAP_ANALYSIS_2025.md`):
- **Live Telemetry**: Not yet implemented.
- **Advanced Guidance**: Only basic attitude hold exists.
- **Recovery System**: Basic; needs GPS beacon, advanced audio/visuals.
- **Sensor Validation**: Orientation accuracy requires physical testing.

*This "Recent Changes" section provides a summary based on the code analysis for updating documentation to v0.48. For a detailed historical changelog, refer to commit history.*

---
## Error Codes

The firmware uses a system of error codes to help diagnose issues. When the system enters an `ERROR` state, or through certain status commands, the last recorded error code will be displayed. These codes are defined in `src/error_codes.h`.

| Code (Dec) | Enum Name                         | Description & Potential Causes                                                                 |
|------------|-----------------------------------|------------------------------------------------------------------------------------------------|
| 0          | `NO_ERROR`                        | No error recorded.                                                                             |
| 10         | `SENSOR_INIT_FAIL_MS5611`         | Failed to initialize the MS5611 barometer. Check I2C wiring, power, sensor health.             |
| 11         | `SENSOR_INIT_FAIL_ICM20948`       | Failed to initialize the ICM-20948 IMU. Check I2C wiring, power, sensor health.                 |
| 12         | `SENSOR_INIT_FAIL_KX134`          | Failed to initialize the KX134 accelerometer. Check I2C wiring, power, sensor health.           |
| 13         | `SENSOR_INIT_FAIL_GPS`            | Failed to initialize the GPS module (e.g., `myGNSS.begin()` failed). Check I2C/UART, power.    |
| 30         | `SENSOR_READ_FAIL_MS5611`         | Failed to read data from MS5611 (e.g., during stabilization or calibration). Sensor might be faulty. |
| 31         | `SENSOR_READ_FAIL_ICM20948`       | Failed to read data from ICM-20948.                                                            |
| 32         | `SENSOR_READ_FAIL_KX134`          | Failed to read data from KX134.                                                                |
| 33         | `SENSOR_READ_FAIL_GPS`            | Failed to get PVT data from GPS repeatedly.                                                    |
| 50         | `SD_CARD_INIT_FAIL`               | SD card initialization failed (e.g., `sd.begin()` returned false). Check card, format (FAT32).   |
| 51         | `SD_CARD_MOUNT_FAIL`              | Could be synonymous with `SD_CARD_INIT_FAIL` depending on library specifics.                   |
| 52         | `LOG_FILE_CREATE_FAIL`            | Failed to create a new log file on the SD card. Card might be full or write-protected.         |
| 53         | `SD_CARD_WRITE_FAIL`              | Failed to write data to the open log file. Card might be full or corrupted.                    |
| 54         | `SD_CARD_LOW_SPACE`               | Warning: SD card free space is below the configured minimum.                                   |
| 60         | `BARO_CALIBRATION_FAIL_NO_GPS`    | Barometer calibration attempted but failed due to insufficient GPS fix (type or pDOP).         |
| 61         | `BARO_CALIBRATION_FAIL_TIMEOUT`   | Barometer calibration timed out waiting for a good GPS fix or valid pressure.                  |
| 62         | `MAG_CALIBRATION_LOAD_FAIL`       | Failed to load magnetometer calibration data from EEPROM (invalid signature/magic number).     |
| 70         | `STATE_TRANSITION_INVALID_HEALTH` | System health check (`isSensorSuiteHealthy`) failed, preventing a critical state transition or during initialization. |
| 71         | `ARM_FAIL_HEALTH_CHECK`           | Attempt to `arm` the system failed due to a health check failure for the ARMED state.          |
| 80         | `EEPROM_SIGNATURE_INVALID`        | EEPROM data for flight state recovery was found but had an invalid signature. Default state used. |
| 250        | `CONFIG_ERROR_MAIN_PARACHUTE`     | Example: `MAIN_PRESENT` is false in `config.h` (not currently enforced by an error code).    |
| 255        | `UNKNOWN_ERROR`                   | An unspecified error occurred.                                                                 |

## AI Assistance
This project utilizes AI assistance for:
- Code documentation and implementation.
- System architecture design and review.
- Gap analysis and project planning.
