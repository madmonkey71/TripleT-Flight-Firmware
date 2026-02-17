# TripleT Flight Firmware - Feature Usage and Configuration

This document details the usage and configuration of major features within the TripleT Flight Firmware.

## MARG Sensor Fusion (Orientation Sensing)

*   **Purpose**: Provides accurate 3D orientation (roll, pitch, yaw) using the ICM-20948's accelerometer, gyroscope, and magnetometer. It employs the Madgwick AHRS (Attitude and Heading Reference System) algorithm to fuse these sensor inputs, effectively correcting for gyroscope drift, especially in the yaw axis, which is crucial for navigation and stable control.
*   **Key Files**:
    *   `src/icm_20948_functions.cpp`
    *   `src/icm_20948_functions.h`
    *   `src/config.h`
*   **Configuration (`config.h`)**:
    *   `` `MADGWICK_BETA_INIT` ``: Initial beta value for the Madgwick filter. This gain determines the filter's overall responsiveness. Higher values make the filter rely more on accelerometer/magnetometer data for orientation correction, while lower values trust the gyroscope integration more.
    *   `` `MADGWICK_BETA_STATIONARY` ``: Reduced beta value used when the system detects it is stationary. This promotes stability of the orientation estimate when there is no actual movement, reducing noise-induced drift.
    *   `` `MADGWICK_BETA_MOTION` ``: Increased beta value used when the system detects motion. This allows the filter to respond more quickly to actual changes in orientation.
    *   `` `MADGWICK_GYRO_BIAS_LEARN_RATE` ``: Learning rate for the algorithm that estimates and compensates for gyroscope bias drift over time. A non-zero value enables this feature.
    *   Magnetometer Calibration:
        *   `magBias[3]` (in `icm_20948_functions.cpp`): Hard-iron offsets for the magnetometer (X, Y, Z). Currently manually set or determined via the `ICM_20948_calibrate()` function.
        *   `magScale[3]` (in `icm_20948_functions.cpp`): Soft-iron scale factors for the magnetometer (X, Y, Z). Currently manually set or determined via the `ICM_20948_calibrate()` function.
*   **Usage and Interaction**:
    *   The MARG filter runs automatically. In the main firmware loop, `ICM_20948_read()` is called, which fetches raw sensor data from the ICM-20948.
    *   Inside `ICM_20948_read()`, after sensor data acquisition, the `MadgwickAHRSupdateMARG()` function is called to update the orientation quaternion (`icm_q0` to `icm_q3`).
    *   The system includes motion detection logic (`isStationary` flag in `icm_20948_functions.cpp`) which dynamically adjusts the `beta` value of the Madgwick filter between `MADGWICK_BETA_STATIONARY` and `MADGWICK_BETA_MOTION`.
    *   Gyroscope bias (`gyroBias[3]`) is continuously estimated and subtracted from raw gyro readings before being fed into the filter if `MADGWICK_GYRO_BIAS_LEARN_RATE` is positive.
    *   Magnetometer calibration can be performed by calling the `ICM_20948_calibrate()` function. While this function exists, it is not directly exposed as a serial command in the current firmware version and would typically be invoked manually during a dedicated calibration routine.
*   **Logged Data**:
    *   `q0`, `q1`, `q2`, `q3` (orientation quaternions)
    *   `euler_roll`, `euler_pitch`, `euler_yaw` (Euler angles in radians, derived from quaternions)
    *   `gyro_bias_x`, `gyro_bias_y`, `gyro_bias_z` (estimated gyroscope biases in rad/s)
    *   Raw sensor values: `icm_accel_x, _y, _z`, `icm_gyro_x, _y, _z`, `icm_mag_x, _y, _z`
*   **Dependencies**:
    *   ICM-20948 Sensor Hardware & associated SparkFun library.
    *   `config.h` for filter parameters.
    *   Motion detection logic within `icm_20948_functions.cpp`.

## PID-based Actuator Control

*   **Purpose**: Provides closed-loop control for up to three hardware actuators (typically servos for control surfaces or Thrust Vector Control - TVC). It uses PID (Proportional-Integral-Derivative) controllers to adjust actuator positions based on the difference between a target orientation and the current orientation provided by the MARG sensor fusion system.
*   **Key Files**:
    *   `src/guidance_control.cpp`
    *   `src/guidance_control.h`
    *   `src/TripleT_Flight_Firmware.cpp` (for initialization, target setting, update calls, and actuator output mapping)
    *   `src/config.h`
*   **Configuration (`config.h`)**:
    *   PID Gains:
        *   `` `PID_ROLL_KP` ``, `` `PID_ROLL_KI` ``, `` `PID_ROLL_KD` ``: Proportional, Integral, and Derivative gains for the roll axis PID controller.
        *   `` `PID_PITCH_KP` ``, `` `PID_PITCH_KI` ``, `` `PID_PITCH_KD` ``: Gains for the pitch axis PID controller.
        *   `` `PID_YAW_KP` ``, `` `PID_YAW_KI` ``, `` `PID_YAW_KD` ``: Gains for the yaw axis PID controller.
    *   Output Limits:
        *   `` `PID_OUTPUT_MIN` ``: Minimum normalized output value from the PID controllers (e.g., -1.0).
        *   `` `PID_OUTPUT_MAX` ``: Maximum normalized output value from the PID controllers (e.g., +1.0).
    *   Integral Limits (Anti-Windup):
        *   `` `PID_INTEGRAL_LIMIT_ROLL` ``: Maximum absolute value for the integral term of the roll PID controller.
        *   `` `PID_INTEGRAL_LIMIT_PITCH` ``: Maximum absolute value for the integral term of the pitch PID controller.
        *   `` `PID_INTEGRAL_LIMIT_YAW` ``: Maximum absolute value for the integral term of the yaw PID controller.
    *   Actuator Pin Assignments:
        *   `` `ACTUATOR_PITCH_PIN` ``: Microcontroller pin connected to the pitch actuator servo.
        *   `` `ACTUATOR_ROLL_PIN` ``: Microcontroller pin connected to the roll actuator servo.
        *   `` `ACTUATOR_YAW_PIN` ``: Microcontroller pin connected to the yaw actuator servo.
    *   Servo Settings:
        *   `` `SERVO_MIN_PULSE_WIDTH` ``: Minimum pulse width (in microseconds) for the servos, corresponding to 0 degrees.
        *   `` `SERVO_MAX_PULSE_WIDTH` ``: Maximum pulse width (in microseconds) for the servos, corresponding to 180 degrees.
        *   `` `SERVO_DEFAULT_ANGLE` ``: Default angle (in degrees) to which servos are set upon initialization.
*   **Usage and Interaction**:
    *   The guidance system is initialized in `setup()` by `guidance_init()`, which resets PID states.
    *   Target orientations (roll, pitch, yaw in radians) are set by calling `guidance_set_target_orientation_euler()`. This is typically done by the `update_guidance_targets()` function from the flight logic module or by specific state logic (like Attitude Hold).
    *   In the main `loop()`, `guidance_update()` (which computes PID outputs) and the subsequent mapping of these outputs to servo commands (e.g., `servo_pitch.write()`) are performed conditionally. These operations are active only when the `currentFlightState` is `BOOST` or `COAST`. In other flight states, the servos are typically commanded to their neutral/default positions to ensure safety and prevent unintended movement.
    *   The normalized PID outputs (-1.0 to 1.0) are then retrieved from `guidance_get_actuator_outputs()`.
    *   These outputs are mapped to servo angle commands (0-180 degrees) using the `map_float()` utility and then written to the respective servos using the `PWMServo` library in `TripleT_Flight_Firmware.cpp`.
*   **Logged Data**:
    *   `actuator_x`, `actuator_y`, `actuator_z` (normalized PID output commands)
    *   `pid_integral_roll`, `pid_integral_pitch`, `pid_integral_yaw` (integral term values for each PID controller)
*   **Dependencies**:
    *   MARG Sensor Fusion system (provides current orientation).
    *   Dynamic Target Update system (provides target orientation).
    *   `config.h` for all PID parameters, actuator pins, and servo settings.
    *   `PWMServo` library for servo control.
    *   `utility_functions.h` for `map_float()` and `constrain()`.

## Attitude Hold Mode

*   **Purpose**: To maintain the rocket's orientation (roll, pitch, and yaw) automatically during the `COAST` phase of flight. This helps stabilize the rocket along its trajectory after motor burnout.
*   **Key Files**:
    *   `src/flight_logic.cpp` (specifically `ProcessFlightState()` where Attitude Hold is triggered)
    *   `src/guidance_control.cpp` (for `guidance_set_target_orientation_euler()` which sets the hold target)
    *   `src/TripleT_Flight_Firmware.cpp` (where `guidance_update()` is conditionally called)
*   **Configuration**:
    *   Attitude Hold is an automatic feature and is not directly enabled/disabled by a single `#define` in `config.h`.
    *   Its performance is dependent on a well-tuned PID controller (see "PID-based Actuator Control" section for gains like `` `PID_ROLL_KP` ``, etc.) and an accurate orientation estimate from the MARG sensor fusion system.
*   **Usage and Interaction**:
    *   The Attitude Hold mode is automatically engaged when the flight state transitions from `BOOST` to `COAST`.
    *   At this transition, the firmware captures the rocket's current roll, pitch, and yaw angles from the active orientation sensor (Kalman filter or Madgwick).
    *   These captured angles are then immediately set as the new target orientation for the PID control system using `guidance_set_target_orientation_euler()`.
    *   Throughout the `COAST` state, the PID controllers will work to minimize any deviation from this locked-in target orientation. The `update_guidance_targets()` function, in its current implementation, sets its target once at initialization and does not interfere with the Attitude Hold target during `COAST`.
*   **Logged Data**:
    *   `target_euler_roll`, `target_euler_pitch`, `target_euler_yaw`: During the `COAST` phase, these fields will reflect the orientation captured at the start of the coast.
    *   Current orientation data (`euler_roll`, `euler_pitch`, `euler_yaw`, or quaternions) to compare against the target.
    *   PID controller internal states (`pid_integral_roll`, etc.) and actuator outputs (`actuator_x`, etc.) to assess performance.
*   **Dependencies**:
    *   PID-based Actuator Control system.
    *   MARG Sensor Fusion system (or Kalman filter if active) for providing current orientation.
    *   Flight State Machine for triggering the mode at the correct phase.

## Dynamic Target Updates

*   **Purpose**: Allows the flight software to dynamically change the target orientation (roll, pitch, yaw) for the PID controllers based on the current mission logic, flight phase, or external commands. This enables the rocket to actively pursue varying pointing objectives during flight.
*   **Key Files**:
    *   `src/flight_logic.cpp` (primarily the `update_guidance_targets()` function)
    *   `src/flight_logic.h`
    *   `src/TripleT_Flight_Firmware.cpp` (where `update_guidance_targets()` is called from the main loop)
*   **Configuration (`config.h`)**:
    *   The example logic in `update_guidance_targets()` does not have its own specific `#define` options in `config.h`.
    *   It does use the `enableSystemDebug` runtime flag (toggled via serial command) to control diagnostic print statements.
    *   The actual target-setting logic (e.g., specific angles, timing for changes) is currently hardcoded within the `update_guidance_targets()` function as a time-based demonstration. Future enhancements might involve making this logic more configurable.
*   **Usage and Interaction**:
    *   The `update_guidance_targets()` function is called once per iteration of the main `loop()` in `TripleT_Flight_Firmware.cpp`, typically just before the `guidance_update()` call.
    *   Inside `update_guidance_targets()`:
        *   On its first run, it captures the current yaw (from the MARG system via `convertQuaternionToEuler` and global quaternions) and sets this as the `initial_yaw_target`. This ensures the system initially targets its starting orientation, preventing sudden movements.
        *   The current example logic then modifies `target_yaw_rad` based on `millis()`:
            *   0-30 seconds: Target yaw is `initial_yaw_target`.
            *   30-60 seconds: Target yaw is `initial_yaw_target + PI / 4.0` (initial yaw + 45 degrees).
            *   >60 seconds: Target yaw reverts to `initial_yaw_target`.
        *   Target roll and pitch are maintained at 0 radians in the example.
        *   Finally, it calls `guidance_set_target_orientation_euler()` to pass these calculated targets to the PID control system.
*   **Logged Data**:
    *   `target_euler_roll`, `target_euler_pitch`, `target_euler_yaw` (target orientation in radians, as set by this module)
*   **Dependencies**:
    *   Guidance Control system (specifically `guidance_set_target_orientation_euler()`).
    *   MARG Sensor Fusion system (specifically `convertQuaternionToEuler()` and global quaternions `icm_q0`, `icm_q1`, `icm_q2`, `icm_q3` to get the initial orientation).
    *   Arduino `millis()` function for timing.

## Guidance Failsafe Mechanisms

*   **Purpose**: To automatically detect and react to unstable flight conditions, enhancing safety. These mechanisms monitor angular rates, attitude errors, and control effort. If predefined thresholds are exceeded for a specified duration, the system flags a stability issue, logs an error (`GUIDANCE_STABILITY_FAIL`), and may transition to an `ERROR` state, typically disengaging active guidance.
*   **Key Files**:
    *   `src/guidance_control.cpp` (implements `guidance_check_stability()`, `guidance_is_stability_compromised()`)
    *   `src/guidance_control.h`
    *   `src/flight_logic.cpp` (integrates stability checks into `BOOST` and `COAST` states)
    *   `src/config.h` (for all related thresholds and durations)
    *   `src/error_codes.h` (for `GUIDANCE_STABILITY_FAIL`)
*   **Configuration (`config.h`)**:
    *   Deflection Limits (for context in saturation check):
        *   `` `MAX_FIN_DEFLECTION_PITCH_DEG` ``, `` `MAX_FIN_DEFLECTION_YAW_DEG` ``, `` `MAX_FIN_DEFLECTION_ROLL_DEG` ``
    *   Angular Rate Thresholds:
        *   `` `STABILITY_MAX_PITCH_RATE_DPS` ``, `` `STABILITY_MAX_ROLL_RATE_DPS` ``, `` `STABILITY_MAX_YAW_RATE_DPS` ``
    *   Attitude Error Thresholds:
        *   `` `STABILITY_MAX_ATTITUDE_ERROR_PITCH_DEG` ``, `` `STABILITY_MAX_ATTITUDE_ERROR_ROLL_DEG` ``, `` `STABILITY_MAX_ATTITUDE_ERROR_YAW_DEG` ``
    *   Control Saturation Threshold:
        *   `` `STABILITY_ACTUATOR_SATURATION_LEVEL_PERCENT` ``
    *   Violation Duration:
        *   `` `STABILITY_VIOLATION_DURATION_MS` ``
*   **Usage and Interaction**:
    *   The stability checks are automatically performed within `flight_logic.cpp` during `BOOST` and `COAST` flight states if guidance is active.
    *   `guidance_control.cpp` contains the core logic (`guidance_check_stability`) that compares current flight parameters (attitude, rates, actuator commands) against the configured thresholds.
    *   If any criterion (high angular rate, large attitude error, or prolonged actuator saturation) is met for longer than `STABILITY_VIOLATION_DURATION_MS`, the `guidance_is_stability_compromised()` flag is set.
    *   `flight_logic.cpp` checks this flag. If true, it sets `g_last_error_code` to `GUIDANCE_STABILITY_FAIL` and transitions the main flight state to `ERROR`.
    *   This system aims to prevent further uncontrolled flight by disarming guidance during instability.
*   **Logged Data**:
    *   `stability_flags` (a bitfield indicating type of stability violation, currently general if any)
    *   `max_pitch_rate_dps_so_far`, `max_roll_rate_dps_so_far`, `max_yaw_rate_dps_so_far` (maximum rates observed in the current state)
    *   `max_pitch_att_err_deg_so_far`, `max_roll_att_err_deg_so_far`, `max_yaw_att_err_deg_so_far` (maximum attitude errors observed)
    *   `LastErrorCode` will show `GUIDANCE_STABILITY_FAIL` if triggered.
*   **Dependencies**:
    *   Attitude (Kalman/MARG) and rate data from IMU.
    *   Actuator command outputs from the PID controllers.
    *   Flight state machine for conditional activation.

## Trajectory Following (Path Guidance) - Initial Implementation

*   **Purpose**: To enable the vehicle to follow a predefined 3D path consisting of waypoints. This initial version uses a simplified "go-to-waypoint" strategy, adjusting target pitch and yaw for the attitude control system.
*   **Key Files**:
    *   `src/guidance_control.cpp` (implements trajectory PIDs, waypoint logic in `guidance_update()`)
    *   `src/guidance_control.h`
    *   `src/data_structures.h` (defines `Trajectory_t`, `TrajectoryWaypoint_t`)
    *   `src/config.h` (for trajectory PID gains, waypoint limits)
*   **Configuration (`config.h`)**:
    *   General Trajectory Settings:
        *   `` `MAX_TRAJECTORY_WAYPOINTS` ``
        *   `` `DEFAULT_WAYPOINT_ACCEPTANCE_RADIUS_M` ``
    *   Trajectory Heading/XTE PID Gains:
        *   `` `TRAJ_XTE_PID_KP` ``, `` `TRAJ_XTE_PID_KI` ``, `` `TRAJ_XTE_PID_KD` ``
        *   `` `TRAJ_XTE_PID_INTEGRAL_LIMIT` ``, `` `TRAJ_XTE_PID_OUTPUT_LIMIT` ``
    *   Trajectory Altitude PID Gains:
        *   `` `TRAJ_ALT_PID_KP` ``, `` `TRAJ_ALT_PID_KI` ``, `` `TRAJ_ALT_PID_KD` ``
        *   `` `TRAJ_ALT_PID_INTEGRAL_LIMIT` ``, `` `TRAJ_ALT_PID_OUTPUT_LIMIT` ``
*   **Usage and Interaction (Conceptual - Full Integration Pending)**:
    *   **Loading**: A trajectory (list of waypoints: lat, lon, alt_msl) would be loaded, e.g., from an SD card file (future) or using `guidance_load_test_trajectory()` for now.
    *   **Activation**: Trajectory following is activated (e.g., via a serial command `traj_activate` calling `guidance_activate_trajectory(true)`). This would typically occur in a specific flight phase (e.g., `COAST` or a dedicated `GUIDED` state).
    *   **Guidance Logic**:
        *   When active, `guidance_update()` determines the current target waypoint.
        *   It calculates the bearing and distance to this waypoint using current GPS data.
        *   It also calculates the altitude error to the waypoint's MSL altitude.
        *   The `target_yaw_rad_g` for the attitude PID is set to the calculated bearing to the waypoint.
        *   The `target_pitch_rad_g` for the attitude PID is determined by the output of the Trajectory Altitude PID, which tries to nullify the altitude error.
        *   `target_roll_rad_g` is typically set to 0 for wings-level flight.
        *   **Waypoint Switching**: When the vehicle is within `DEFAULT_WAYPOINT_ACCEPTANCE_RADIUS_M` of the current target waypoint, the system advances to the next waypoint in the trajectory.
    *   **Deactivation**: Trajectory following can be deactivated (e.g., `traj_deactivate` or automatically at the end of the path). The system then reverts to standard attitude hold or other guidance modes.
*   **Logged Data**:
    *   `current_target_wp_idx` (index of the current target waypoint)
    *   `distance_to_target_wp_m` (distance in meters to the current target waypoint)
    *   `bearing_to_target_wp_rad` (bearing in radians to the current target waypoint)
    *   `altitude_error_to_wp_m` (altitude error in meters to the current target waypoint's MSL altitude)
*   **Dependencies**:
    *   GPS for current position (latitude, longitude, altitude MSL) and heading/velocity.
    *   Attitude (Kalman/MARG) and rate data from IMU for the underlying attitude PIDs.
    *   Attitude PID control system.
    *   (Future) SD card interface for loading trajectory files.
    *   (Future) Serial command interface for controlling trajectory loading/activation.

## Data Logging System

*   **Purpose**: To record a wide array of sensor data, system states, derived values, and control parameters to an SD card for post-flight analysis, debugging, and performance evaluation.
*   **Key Files**:
    *   `src/TripleT_Flight_Firmware.cpp` (contains `WriteLogData()`, `createNewLogFile()`, and SD card initialization logic)
    *   `src/data_structures.h` (defines the `LogData` struct)
    *   `src/log_format_definition.cpp` & `src/log_format_definition.h` (define `LOG_COLUMNS` array, `LogColumnDescriptor_t` struct, and `LogFieldType_e` enum, which map `LogData` fields to CSV columns and types)
    *   `src/utility_functions.cpp` (contains `logDataToString()` for converting `LogData` struct to a CSV string)
    *   `src/config.h` (for storage and buffering settings)
*   **Configuration (`config.h`)**:
    *   SD Card Settings:
        *   `` `SD_CARD_MIN_FREE_SPACE` ``: Minimum required free space (in bytes) on the SD card. Logging may be affected if space falls below this.
        *   `` `SD_CACHE_SIZE` ``: Cache size factor used by the SdFat library for SD card operations to optimize write performance.
        *   `` `LOG_PREALLOC_SIZE` ``: Size (in bytes) to pre-allocate for the log file on the SD card. This can help reduce file fragmentation and improve write consistency.
        *   `` `DISABLE_SDCARD_LOGGING` ``: If set to `true`, all SD card logging is disabled. Useful for testing or when an SD card is not present.
    *   Buffering Settings (RAM):
        *   `` `MAX_LOG_ENTRIES` ``: (Currently not directly used, `WriteLogData` writes per sensor update, but `FLUSH_INTERVAL` and a flush counter in `WriteLogData` manage physical writes).
        *   `` `FLUSH_INTERVAL` ``: Maximum time (in milliseconds) between periodic flushes of the log buffer to the SD card. In `TripleT_Flight_Firmware.cpp`, there's a separate timer for flushing every 10 seconds, and `WriteLogData` flushes every 10 writes.
        *   `` `MAX_BUFFER_SIZE` ``: Maximum size (in bytes) allocated for a single log entry string when converting `LogData` to a string.
    *   Runtime Flags (not in `config.h`):
        *   `enableSerialCSV` (toggled via serial command): If true, formatted log data is also printed to the serial console.
*   **Usage and Interaction**:
    *   Logging is initialized during `setup()` via `initSDCard()` and `createNewLogFile()`.
    *   If the SD card is available, a new log file is created. The filename includes a timestamp if GPS time synchronization has occurred; otherwise, it uses `millis()` for a unique name.
    *   The header row for the CSV file is generated based on the `LOG_COLUMNS` array from `log_format_definition.cpp`.
    *   `WriteLogData()` is called in the main `loop()` whenever new sensor data is processed (typically after `sensorsUpdated` flag is true).
    *   Inside `WriteLogData()`:
        *   The `LogData` struct is populated with current data from all relevant modules (GPS, Baro, IMU, MARG, Guidance).
        *   `logDataToString()` converts the `LogData` struct into a comma-separated string based on `LOG_COLUMNS`.
        *   This string is written as a new line to the log file on the SD card.
        *   Data is flushed to the SD card periodically (every 10 writes within `WriteLogData` or every 10 seconds by a timer in `loop()`) to ensure data integrity without excessive wear.
*   **Logged Data**:
    *   All fields defined in the `LogData` struct (see `src/data_structures.h`) and described by `LOG_COLUMNS` (see `src/log_format_definition.cpp`). This includes, but is not limited to:
        *   `SeqNum`, `Timestamp`
        *   GPS data (`FixType`, `Sats`, `Lat`, `Long`, `Alt`, `AltMSL`, `Speed`, `Heading`, `pDOP`, `RTK`)
        *   Barometer data (`Pressure`, `Temperature`, `RawAltitude`, `CalibratedAltitude`)
        *   Raw Accelerometer/Gyro/Magnetometer data from KX134 and ICM-20948
        *   MARG filter outputs (`Q0, Q1, Q2, Q3`, `EulerRoll_rad, EulerPitch_rad, EulerYaw_rad`, `GyroBiasX_rps, GyroBiasY_rps, GyroBiasZ_rps`)
        *   Guidance & Control data (`TargetRoll_rad, TargetPitch_rad, TargetYaw_rad`, `PIDIntRoll, PIDIntPitch, PIDIntYaw`, `ActuatorX, ActuatorY, ActuatorZ`)
*   **Dependencies**:
    *   SD card hardware and physical presence.
    *   SdFat library for SD card filesystem operations.
    *   All sensor modules (GPS, Barometer, IMU, Accelerometer) to provide data.
    *   MARG Sensor Fusion system.
    *   Guidance Control and Dynamic Target Update systems.
    *   `data_structures.h` and `log_format_definition.h/.cpp` for log structure.

## GPS Module (SparkFun ZOE-M8Q)

*   **Purpose**: Provides global positioning data including latitude, longitude, altitude (ellipsoidal and Mean Sea Level), ground speed, heading of motion, and precise time. This information is vital for location tracking, trajectory analysis, and is used for calibrating the barometric altimeter.
*   **Key Files**:
    *   `src/gps_functions.cpp`
    *   `src/gps_functions.h`
    *   `src/gps_config.h` (though this file might be minimal or integrated elsewhere)
*   **Configuration (`config.h` or runtime)**:
    *   `` `GPS_TIMEOUT_MS` `` (from `config.h`): Maximum time in milliseconds to wait for a GPS update before considering the connection potentially problematic (used in `checkGPSConnection()`).
    *   `enableGPSDebug` (runtime flag, toggled via serial command): Enables/disables detailed debug output from the u-blox GPS library.
    *   The I2C address for the ZOE-M8Q is typically fixed at `0x42` and is used directly in `gps_init()`.
*   **Usage and Interaction**:
    *   The GPS module is initialized by `gps_init()` during the `setup()` phase. This configures the I2C communication and attempts to establish communication with the GPS unit.
    *   Raw GPS data is read by calling `gps_read()` in the main `loop()`. This function polls the GPS module for new PVT (Position, Velocity, Time) data and updates global variables (e.g., `GPS_latitude`, `GPS_longitude`, `GPS_altitude`, `SIV`, `GPS_fixType`).
    *   The MSL altitude obtained from the GPS (`GPS_altitudeMSL`) is used by the `ms5611_calibrate_with_gps()` function to determine the current pressure offset for the barometric altimeter, improving its accuracy.
    *   `checkGPSConnection()` is called periodically to monitor the health of the GPS data stream.
*   **Logged Data**:
    *   `FixType` (e.g., 0=no fix, 2=2D, 3=3D)
    *   `Sats` (number of satellites in view)
    *   `Lat` (Latitude, degrees * 1e7)
    *   `Long` (Longitude, degrees * 1e7)
    *   `Alt` (Altitude above ellipsoid, mm)
    *   `AltMSL` (Altitude above Mean Sea Level, mm)
    *   `Speed` (Ground speed, mm/s)
    *   `Heading` (Heading of motion, degrees * 1e5)
    *   `pDOP` (Position Dilution of Precision, unitless * 100)
    *   `RTK` (RTK fix status/type)
*   **Dependencies**:
    *   SparkFun ZOE-M8Q hardware module.
    *   SparkFun u-blox GNSS Arduino Library.
    *   I2C communication bus.

## Barometric Pressure Sensor (MS5611)

*   **Purpose**: Measures atmospheric pressure and ambient temperature. The primary use of pressure readings is to calculate altitude above sea level or ground level. Temperature readings can be used for sensor compensation or general environmental data.
*   **Key Files**:
    *   `src/ms5611_functions.cpp`
    *   `src/ms5611_functions.h`
*   **Configuration (`config.h` or runtime)**:
    *   `` `BAROMETER_ERROR_THRESHOLD` `` (from `config.h`): Defines a threshold for detecting erroneous jumps in pressure readings. (Currently, this seems to be used more as a concept in comments than an active filter in `ms5611_functions.cpp`).
    *   `` `APOGEE_CONFIRMATION_COUNT` `` (from `config.h`): Used by the flight logic (`detectApogee` in `flight_logic.cpp`) when using barometric altitude to confirm apogee.
    *   `enableBaroDebug` (runtime flag, toggled via serial command): Enables detailed debug output for barometer operations.
    *   The I2C address for the MS5611 is typically `0x77` (or `0x76` depending on CSB pin) and is used in `ms5611_init()`.
*   **Usage and Interaction**:
    *   The MS5611 sensor is initialized by `ms5611_init()` during `setup()`. This function checks for sensor presence and reads calibration coefficients from the sensor.
    *   Pressure and temperature readings are obtained by calling `ms5611_read()` in the main `loop()`. This function updates global variables `pressure` (hPa) and `temperature` (°C).
    *   `ms5611_get_altitude()` can be called to convert the current pressure reading into an altitude estimate (meters).
    *   `ms5611_calibrate_with_gps()` is a key function that uses GPS altitude (MSL) to calculate an `baro_altitude_offset`. This offset is then applied in `ms5611_get_altitude()` to provide a calibrated altitude, compensating for local weather conditions or initial ground elevation. Calibration can be triggered manually via serial command or automatically under certain conditions.
    *   The altitude data is a primary input for flight state detection logic (e.g., apogee detection in `flight_logic.cpp`).
*   **Logged Data**:
    *   `Pressure` (in hPa)
    *   `Temperature` (in °C)
    *   `RawAltitude` (altitude calculated directly from pressure, before GPS calibration)
    *   `CalibratedAltitude` (altitude after applying GPS-derived offset)
*   **Dependencies**:
    *   MS5611 hardware sensor.
    *   MS5611 Arduino Library (or compatible custom implementation).
    *   I2C communication bus.
    *   GPS module (for calibration data, specifically altitude MSL).

## High-G Accelerometer (SparkFun KX134)

*   **Purpose**: Provides high-range acceleration measurements (up to +/-64g, depending on configuration). Its primary role is to capture high-impact events such as motor burnout shock, staging events, parachute deployment shock, or landing impact. It can also supplement main IMU data during high-G flight phases if its range is more suitable.
*   **Key Files**:
    *   `src/kx134_functions.cpp`
    *   `src/kx134_functions.h`
    *   `src/config.h`
*   **Configuration (`config.h`)**:
    *   `` `USE_KX134` ``: A boolean flag. If set to `1` (or `true`), the firmware will attempt to initialize and use the KX134 accelerometer. If `0` (or `false`), the KX134 will be ignored.
    *   The I2C address for the KX134 is typically `0x1F` (or `0x1E` depending on ADDR pin) and is used in `kx134_init()`. The specific range and Output Data Rate (ODR) are set within `kx134_init()`.
*   **Usage and Interaction**:
    *   If `USE_KX134` is enabled, the sensor is initialized by `kx134_init()` during `setup()`. This configures the sensor's operating range and data rate.
    *   Acceleration data (X, Y, Z) is read by calling `kx134_read()` in the main `loop()`. This updates the global `kx134_accel[3]` array with values in 'g'.
    *   The `get_accel_magnitude()` utility function in `utility_functions.cpp` can use KX134 data (if available and preferred) to calculate overall acceleration magnitude, which is used in flight state detection logic (e.g., `detectLanding`, `detectBoostEnd`).
*   **Logged Data**:
    *   `KX134_AccelX`, `KX134_AccelY`, `KX134_AccelZ` (acceleration in g for each axis)
*   **Dependencies**:
    *   SparkFun KX134 hardware sensor.
    *   SparkFun KX13X Arduino Library.
    *   I2C communication bus.

## Serial Command Interface

*   **Purpose**: Provides a way for users to interact with the flight computer in real-time via a serial terminal. This interface is used for debugging, manually triggering actions (like calibration), viewing sensor data, checking system status, and configuring runtime parameters (like debug message verbosity).
*   **Key Files**:
    *   `src/TripleT_Flight_Firmware.cpp` (contains the main `processCommand()` function and helpers like `printHelpMessage()`, `toggleDebugFlag()`, `printStatusSummary()`, etc.)
    *   `src/README.md` (contains a table of available serial commands)
*   **Configuration (`config.h` or runtime)**:
    *   Many of the `enable<Feature>Debug` flags (e.g., `enableSystemDebug`, `enableIMUDebug`, `enableGPSDebug`, `enableBaroDebug`, `enableStorageDebug`, `enableICMRawDebug`, `enableSerialCSV`) are runtime boolean variables that are toggled on/off using specific serial commands. They are not typically defined in `config.h` as compile-time constants, but rather initialized to `false` at startup in `setup()`.
*   **Usage and Interaction**:
    *   The user connects to the Teensy board via a serial terminal program (e.g., Arduino Serial Monitor, PuTTY, PlatformIO Serial Monitor).
    *   Commands are typed into the terminal and sent to the firmware.
    *   The main `loop()` in `TripleT_Flight_Firmware.cpp` checks for available serial data. If data is present, it reads the command string.
    *   The `processCommand()` function parses the received string and executes the corresponding action. Actions can include:
        *   Toggling debug flags.
        *   Displaying help messages or status reports.
        *   Initiating calibration routines.
        *   Other diagnostic or operational tasks.
    *   A list of available commands and their descriptions can be found in the `README.md` file and by typing `help` or `a` into the serial console.
*   **Logged Data**:
    *   The serial command interface itself does not directly log data to the SD card. However, the commands executed can significantly affect system behavior, such as enabling `enableSerialCSV` which mirrors log data to the serial port, or toggling debug flags that might alter the verbosity of other data sources or operations that are logged.
*   **Dependencies**:
    *   Arduino `Serial` library for communication.
    *   Various other firmware modules whose functions are invoked by commands (e.g., sensor modules for data display, calibration routines, debug flag variables).
