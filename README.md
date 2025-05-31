# TripleT Flight Firmware

An eventually comprehensive flight controller firmware for Teensy 4.0/4.1 microcontrollers, designed for model rockets and high-power rocketry applications.

## Project Lead
**Matthew Thom** - Project Lead and Primary Developer

## Project Status - Alpha (at best)
**Current Version**: v0.40

### Recent Updates
 - ✅ Take stock of where I am and what's been lost. I'm not sure that I'll ever know 100%
 - ✅ Get the firmware to build
 - ✅ Fixed compilation errors:
    - Added forward declaration for `prepareForShutdown` in `src/flight_logic.cpp`.
    - Ensured `EEPROM_UPDATE_INTERVAL` is accessible in `src/state_management.cpp` by including `constants.h`. (Note: A persistent linter error for this suggests a build environment issue for the user to resolve).
 - ✅ Test it
 - ✅ Updated data logging to include flight state as the third field.
 - ✅ Updated web interface to display the current flight state.
 - ✅ Moved flight state display in web interface to its own section above real-time data.
 - ✅ Updated `State Machine.md` documentation, including the diagram, to reflect current system state.
 - ✅ Bugs
	 - ✅ Test GPS
	 - ✅ Fix GPS numbers (verified with a Type 2 fix)
		 - ✅ Long
		 - ✅ Lat
		 - ✅ Altitude
	 - ✅ Baro Altitude not getting logged.
	 - ✅ Verify KX134 data array
	 - ✅ Verify ICM_20948 data arrays
		 - ✅ Accel
		 - ✅ Gyro
		 - ✅ Mag
	 - ✅ SDCard Write issues
		 - ✅ Verify without SDCard
		 - ✅ Adjust when logging starts (State: Armed)
     - ✅ Removed support for other boards and changed all SDCard routines to the Teensy 4.1
     - ✅ Testing
      - ✅ With a card on startup
      - ✅ Without a card on startup and adding it later.

- ✅ Implemented static gyroscope bias calibration during startup.
- ✅ Enabled ICM-20948 raw data debug output by default for tuning.
- ✅ Iteratively tuned Madgwick filter parameters (`MADGWICK_BETA_STATIONARY`, `MADGWICK_GYRO_BIAS_LEARN_RATE`) and motion detection thresholds (`ACCEL_VARIANCE_THRESHOLD`, `STATE_CHANGE_THRESHOLD`) to improve stationary stability and reduce RPY drift.
- ✅ Updated magnetometer calibration to use full hard and soft iron correction (3x3 matrix) derived from `calibrate3.py` for more accurate heading.

### Development Status
- ✅ Core sensor integration (GPS, Barometer, IMU, Accelerometer)
- ✅ SD Card data logging
- ✅ Interactive serial interface
- ✅ Basic diagnostic tools
- ✅ GPS/Barometer calibration
- ✅ Configurable debug outputs
- 🚧 Flight state detection (liftoff, boost, coast, apogee, descent)
  - [ ] This got lost in a github mishap on my system. Roadmap to re-implement it  
- ✅ Apogee detection (using multiple redundant methods)
- ✅ Initial Parachute deployment control (drogue and main deployment logic)
- ✅ Basic Actuator Control (PID-based, 3-axis for servos)
- ✅ 9-axis MARG Sensor Fusion (Madgwick AHRS for orientation)
- ✅ Dynamic Target Orientation System (Time-based example framework)
- ✅ Logging of PID Controller States and Dynamic Targets
- ✅ IMU Calibration (How do we fix drift in the madgwick implementation)
  - ✅ The current setup has significant drift in it's current form
  - ✅ Implemented static gyro bias calibration.
  - ✅ Tuned Madgwick filter parameters and motion detection thresholds.
  - ✅ Implemented full hard and soft iron magnetometer calibration.
- 🚧 Enhanced telemetry (Planned)
  - ✅ Add all data to the logging so that it's exposed to being processed as telemetry.
- 🚧 Live Transmission of data via radio (Planned)

### AI Assistance
This project utilizes AI assistance for:
- Code documentation
- System architecture design
- Implementation suggestions
- Code review and optimization

## Overview

TripleT Flight Firmware is an open-source flight controller software built for Teensy 4.0/4.1. It provides robust sensor integration, data logging, and a command-driven interface to configure and interact with your flight computer.

## Hardware Requirements

- **Microcontroller**: Teensy 4.0 or 4.1
- **Sensors**:
  - SparkFun ZOE-M8Q GPS Module (I2C address 0x42)
  - MS5611 Barometric Pressure Sensor (I2C address 0x77)
  - SparkFun KX134 Accelerometer (I2C address 0x1F)
  - SparkFun ICM-20948 9-DOF IMU (I2C address 0x69)
- **Storage**:
  - SD Card (connected via SPI on Teensy 4.0, via SDIO on Teensy 4.1)
- **Other Components**:
  - Optional: Servo motors for control surfaces or Thrust Vector Control (TVC). Pins are configurable in `src/config.h` (e.g., `ACTUATOR_PITCH_PIN`, `ACTUATOR_ROLL_PIN`, `ACTUATOR_YAW_PIN`).
  - Optional: Pyro channels (pins 5, 6, 7, 8)
  **Desirable additions**
  - 8Mb of additional flash
  - Optional: Buzzer (pin 23)
  - Optional: WS2812 LEDs (pin 7)

## Features

- **Multi-sensor Integration**: Combined data from GPS, barometer, accelerometer, and 9-DOF IMU.
- **MARG Sensor Fusion**: Employs a 9-axis Madgwick AHRS filter to fuse accelerometer, gyroscope, and magnetometer data from the ICM-20948. This provides robust orientation estimation (roll, pitch, yaw) with yaw drift correction essential for navigation and control.
- **PID-based Actuator Control**: Features a configurable 3-axis PID controller designed to manage hardware actuators (e.g., PWM servos for Thrust Vector Control or control surfaces). PID outputs are mapped to servo signals. Gains, output limits, and integral anti-windup limits are configurable via `src/config.h`.
- **Dynamic Target Updates**: Includes a system for dynamic updates to the PID controllers' target orientation (roll, pitch, yaw). The current implementation provides a basic time-based example for yaw target changes, serving as a framework for more complex mission-specific guidance logic.
- **SD Card Logging**: Comprehensive data logging to SD card with CSV format
- **GPS/Barometer Calibration**: Automatic or manual calibration of barometric altitude based on GPS data
  - Robust calibration with validation checks for pressure and GPS data
  - Visual feedback via LED indicators (purple during calibration, green for success, red for failure)
  - Limited retry mechanism to prevent infinite calibration loops
- **Flight State Machine**: Sophisticated flight state detection and management system
  - 14 distinct flight states from startup through recovery
  - Redundant detection methods for critical events (liftoff, apogee, landing)
  - Automatic state transitions with configurable parameters
- **Error Handling & Recovery**: Robust error detection and recovery mechanisms
  - Watchdog timer to prevent system lockups
  - Sensor health monitoring with graceful degradation
  - Automatic recovery procedures when possible
- **State Persistence**: EEPROM-based state storage for recovery from power loss or resets
  - Saves critical flight parameters (altitude, state, timestamps)
  - Automatic recovery to appropriate state after power interruption
- **Comprehensive Data Collection**:
  - GPS position, altitude, speed, and fix quality
  - Barometric pressure, temperature, and calibrated altitude
  - High-G acceleration measurements
  - Orientation data from IMU (quaternions and Euler angles).
  - Full orientation quaternions (q0, q1, q2, q3) and derived Euler angles (roll, pitch, yaw) from the MARG filter.
  - PID controller internal states including commanded target Euler angles, integral term values, and final actuator output commands.
- **Interactive Serial Interface**: Command-driven system for data retrieval and configuration
- **Diagnostic Tools**: I2C scanner, sensor status reporting, and storage space monitoring
- **Configurable Debug Outputs**: Selectively enable/disable debug information for specific sensors

## Installation

1. **Set up PlatformIO Environment**:
   - Install [PlatformIO](https://platformio.org/)
   - Clone this repository
   - Open the project in PlatformIO
   
2. **Required Libraries**:
   - Adafruit_NeoPixel
   - SdFat
   - SerialFlash
   - PWMServo
   - SparkFun_KX13X
   - SparkFun_u-blox_GNSS_Arduino_Library
   - MS5611 library
   - SparkFun ICM-20948 IMU library
   - T4 Watchdog library

3. **Hardware Connections**:
   - Connect sensors via I2C (SDA/SCL)
   - Connect SD card via SPI (pins 10-13) for Teensy 4.0 or SDIO for Teensy 4.1
   - Connect buzzer, servos, and other components to designated pins

4. **Compile and Upload**:
   - Select your target board (TripleTFlight, teensy41 or teensy40)
   - Compile and upload using PlatformIO

## Usage

### Basic Operation

On startup, the firmware:
1. Initializes all sensors and storage devices
2. Creates a new log file (with timestamp if GPS is available)
3. Begins collecting and logging sensor data

### Serial Commands

The firmware supports both single-character and extended commands:

| Command | Description |
|---------|-------------|
| `0`     | Toggle serial CSV output |
| `1-9`   | Toggle various debug outputs (system, IMU, GPS, barometer, etc.) |
| `a`     | Show help message |
| `b`     | Show system status |
| `f`     | Show storage statistics |
| `g`     | Toggle detailed display mode |
| `h`     | Calibrate barometer with GPS |
| `i`     | Display IMU data |
| `j`     | Toggle status summary |
| `9`     | Initiate system shutdown |
| `debug_all_off` | Disable all debugging output |
| `calibrate` | Manually calibrate barometer with GPS |
| `help`  | Show help message |
| `arm`   | Arm the flight computer (transition to ARMED state) |
| `test_error` | Simulate sensor error (for testing) |
| `test_watchdog` | Simulate watchdog timeout (for testing) |
| `clear_errors` | Reset all error flags and return to normal operation |
| `status_sensors` | Display detailed sensor status information |

### Debug Options

You can selectively enable/disable various debug outputs:

- System debug
- IMU debug
- GPS debug
- Barometer debug
- Storage debug
- ICM raw debug
- Serial CSV output

### Data Logging

Data is logged to the SD card in CSV format with the following information:
- Timestamp
- Flight State (as an integer corresponding to the FlightState enum)
- GPS information (fix type, satellites, position, altitude, speed)
- Barometric data (pressure, temperature)
- Accelerometer readings (X, Y, Z in g)
- IMU data (acceleration, gyro, magnetometer)

### Barometer Altitude Calibration using GPS

The system supports calibration of the barometric altitude using GPS data:
- Automatic calibration when a good GPS fix is available (pDOP < 3.0)
- Requires stable sensor readings before attempting calibration
- Limited to 3 attempts with 30-second intervals between attempts
- Manual calibration via the `h` command or `calibrate` command
- Visual feedback via LED (purple during calibration, green for success, red for failure)
- Detailed debug output to diagnose calibration issues

### Configuration

Key firmware features can be configured by modifying `#define` statements in `src/config.h`:

- `ENABLE_WATCHDOG`: Set to `true` (default) to enable the hardware watchdog timer, which helps recover from system hangs. Set to `false` to disable the watchdog, which can be useful during debugging or testing when breakpoints might cause the watchdog to reset the system unintentionally.
- **Features & Hardware Presence**:
  - `DROGUE_PRESENT`: `true` if a drogue parachute is physically present and should be controlled.
  - `MAIN_PRESENT`: `true` if a main parachute is physically present and should be controlled.
  - `BUZZER_OUTPUT`: `true` to enable audible feedback via the buzzer.
  - `USE_KX134`: `true` to utilize the KX134 high-G accelerometer (typically alongside the ICM-20948).
- **Pin Definitions**:
  - `FLASH_CS_PIN`: Chip Select pin for the external serial flash memory (if used).
  - `NEOPIXEL_PIN`: Data pin for the WS2812 NeoPixel LEDs.
  - `BUZZER_PIN`: Pin connected to the buzzer.
- **NeoPixel Configuration**:
  - `NEOPIXEL_COUNT`: The number of NeoPixel LEDs connected.
- **Flight Logic Parameters**:
  - `MAIN_DEPLOY_ALTITUDE`: Target altitude (in meters Above Ground Level - AGL) for main parachute deployment.
  - `BOOST_ACCEL_THRESHOLD`: Acceleration threshold (in g) to detect liftoff.
  - `COAST_ACCEL_THRESHOLD`: Acceleration threshold (in g) to detect the end of the boost phase (motor burnout).
  - `APOGEE_CONFIRMATION_COUNT`: Number of consecutive readings required to confirm apogee detection.
  - `LANDING_CONFIRMATION_COUNT`: Number of consecutive readings required to confirm landing.
- **Sensor Error & Timeout Thresholds**:
  - `MAX_SENSOR_FAILURES`: General threshold for maximum sensor failures before marking a sensor as non-functional (currently used implicitly in some checks).
  - `BAROMETER_ERROR_THRESHOLD`: Maximum acceptable pressure change (in hPa) between consecutive barometer readings.
  - `ACCEL_ERROR_THRESHOLD`: Maximum acceptable acceleration change (in g) between consecutive accelerometer readings.
  - `GPS_TIMEOUT_MS`: Maximum time (in milliseconds) without a valid GPS update before considering the GPS to have failed.
- **Storage & Logging Configuration**:
  - `SD_CARD_MIN_FREE_SPACE`: Minimum required free space (in bytes) on the SD card for logging to continue.
  - `SD_CACHE_SIZE`: Cache size factor for SD card operations (used by SdFat).
  - `LOG_PREALLOC_SIZE`: Size (in bytes) to pre-allocate for the log file on the SD card to improve performance.
  - `DISABLE_SDCARD_LOGGING`: `true` to completely disable logging to the SD card (useful for testing without an SD card).
  - `MAX_LOG_ENTRIES`: Maximum number of log entries to buffer in RAM before forcing a flush to the SD card.
  - `FLUSH_INTERVAL`: Maximum time (in milliseconds) between flushes of the log buffer to the SD card, even if the buffer isn't full.
  - `MAX_BUFFER_SIZE`: Maximum size (in bytes) allocated for a single log entry string in the RAM buffer.
  - `EXTERNAL_FLASH_MIN_FREE_SPACE`: Minimum required free space (in bytes) on the external flash memory (if used).
- **EEPROM Configuration**:
  - `EEPROM_STATE_ADDR`: Starting memory address in EEPROM where the flight state data is stored.
  - `EEPROM_SIGNATURE_VALUE`: A magic number used to validate the integrity of preoccupied data stored in EEPROM.

- **Madgwick Filter Configuration**:
  - `MADGWICK_BETA_INIT`: Initial beta value for the Madgwick filter. This gain determines the filter's overall responsiveness. Higher values rely more on accelerometer/magnetometer data, while lower values trust the gyroscope more.
  - `MADGWICK_BETA_STATIONARY`: Reduced beta value used when the system detects it is stationary. This promotes stability of the orientation estimate when there is no actual movement.
  - `MADGWICK_BETA_MOTION`: Increased beta value used when the system detects motion. This allows the filter to respond more quickly to changes in orientation.
  - `MADGWICK_GYRO_BIAS_LEARN_RATE`: Learning rate for the algorithm that estimates and compensates for gyroscope bias drift over time.

- **PID Controller & Actuator Configuration**:
  - `PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD`: Proportional, Integral, and Derivative gains for the roll axis PID controller. These values tune the controller's response to roll errors.
  - `PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD`: Proportional, Integral, and Derivative gains for the pitch axis PID controller.
  - `PID_YAW_KP, PID_YAW_KI, PID_YAW_KD`: Proportional, Integral, and Derivative gains for the yaw axis PID controller.
  - `PID_OUTPUT_MIN, PID_OUTPUT_MAX`: Minimum and maximum normalized output values for the PID controllers (e.g., -1.0 to 1.0). This limits the command sent to actuators.
  - `PID_INTEGRAL_LIMIT_ROLL, PID_INTEGRAL_LIMIT_PITCH, PID_INTEGRAL_LIMIT_YAW`: Anti-windup limits for the integral term of each PID axis. Prevents the integral term from growing too large, which can cause instability.
  - `ACTUATOR_PITCH_PIN, ACTUATOR_ROLL_PIN, ACTUATOR_YAW_PIN`: Microcontroller pin assignments for the pitch, roll, and yaw actuator servos.
  - `SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH`: Minimum and maximum pulse width (in microseconds) for servo control. These values define the servo's range of motion (typically 1000µs to 2000µs for 0-180 degrees).
  - `SERVO_DEFAULT_ANGLE`: Default angle (in degrees) to which servos are set upon initialization and when no specific command is active.

- **SD Card Driver Configuration (Board Specific)**:
  - These settings (`SD_CONFIG`, `SD_BUF_SIZE`, `SD_DETECT_PIN`, `SD_CS_PIN`) configure the SdFat library for the specific board (Teensy 4.1 SDIO or Teensy 4.0 SPI) and optimize performance. They are typically handled automatically based on the detected board type.

## Documentation

For detailed documentation, please refer to:
- [System Documentation](docs/TripleT_Flight_Firmware_Documentation.md)
- [State Machine Design](State Machine.md)

## Potential Improvements and Future Work

- [ ] **Persistent Magnetometer Calibration:** Implement a mechanism to save and load magnetometer calibration values (bias and scale) to/from EEPROM or the SD card. This would avoid the need for recalibration or manual code changes after each power cycle.
- [ ] **Advanced Flight State Machine for Guidance:** Integrate the dynamic target update system more deeply with a comprehensive flight state machine. Target orientations for the PID controllers should ideally be determined by the active flight phase (e.g., ascent, coast, specific pointing maneuvers, landing orientation).
- [ ] **UKF and MARG Synergy:** Review and clarify the roles of the existing 1D Unscented Kalman Filter (UKF for altitude/velocity) and the new 9-axis Madgwick (MARG for attitude). Explore options for tighter integration, such as using MARG attitude to improve the UKF's state estimation or extending the UKF to a full 3D state estimator.
- [ ] **Consolidate Helper Functions:** Resolve any duplicate helper functions (e.g., `convertQuaternionToEuler`) by ensuring a single, canonical version is used throughout the codebase, properly declared in a shared header.
- [ ] **Configurable Actuator Axis Mapping:** Provide clear documentation and potentially runtime or compile-time configuration options for mapping logical PID axes (X, Y, Z from `guidance_control`) to physical servo outputs and their corresponding control effects (e.g., which servo controls pitch, roll, or yaw, and in which direction).
- [ ] **Advanced PID Control Techniques:** Investigate and potentially implement advanced PID features such as derivative-on-measurement (to reduce setpoint kick), feedforward control (to improve response to known disturbances), or even auto-tuning capabilities.
- [ ] **Guidance System Failsafes:** Develop more specific error handling and failsafe mechanisms for the guidance and control system. This could include strategies for sensor disagreements, actuator saturation, or unexpected flight dynamics.
- [ ] **Refine `isStationary` Detection:** The current `isStationary` detection in `icm_20948_functions.cpp` relies on gyro magnitude and accelerometer variance. Its thresholds (`GYRO_THRESHOLD`, `ACCEL_VARIANCE_THRESHOLD`, `STATE_CHANGE_THRESHOLD`) might need tuning for different physical systems and environments.
- [ ]  **Magnetometer Disturbance Rejection:** Explore techniques for detecting and mitigating the effects of local magnetic disturbances on the MARG filter.
- [ ] Enhanced telemetry and wireless data transmission.
- [ ] Flight stabilization via more advanced thrust vector control implementation.
- [ ] Improved power management and battery monitoring.
- [ ] User-configurable settings saved to flash.
- [ ] Live data transmission via radio.
- [ ] Custom smartphone companion app for ground control.
- [ ] Advanced filtering algorithms for sensor fusion (beyond current MARG/UKF).
- [ ] Auto-diagnostics and self-testing capabilities.
- [ ] Dual-deployment redundancy systems.

## Acknowledgements

This project was inspired by:
- BPS.Space flight computer designs by Joe Barnard
- He makes awesome rocketry content so you should go and support him https://www.youtube.com/@BPSspace

## License
This project is licensed under the GNU General Public License v3.0 (GPL-3.0) - a copyleft license that requires anyone who distributes this code or a derivative work to make the source available under the same terms.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

When contributing:
- Fork the repository
- Create a feature branch
- Make your changes
- Submit a pull request with a clear description of the changes


