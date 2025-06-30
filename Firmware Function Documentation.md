# TripleT Flight Firmware Function Documentation
## Version 0.50

## Flow Diagram (Conceptual v0.50)

```mermaid
graph TD
    A[Start] --> B(Setup);

    subgraph Setup
        direction LR
        B1(Init HW: Serial, I2C, Pins) --> B_Neo(Init NeoPixel);
        B_Neo --> B_Pyro(Init Pyro Pins);
        B_Pyro --> B_Servo(Init Servos);
        B_Servo --> B_Buzz(Init Buzzer);
        B_Buzz --> B2(State Recovery: recoverFromPowerLoss());
        B2 --> B3(Init SD Card: initSDCard());
        B3 --> B4(Init GPS: gps_init());
        B4 --> B5(Init Sensors: ms5611_init, ICM_20948_init, kx134_init);
        B5 --> B6(Init Kalman Filter: kalman_init());
        B6 --> B7(Init Guidance: guidance_init());
        B7 --> B8(Create Initial Log File: createNewLogFile());
        B8 --> B9(Print Initial Sensor Status);
        B9 --> B10(Setup Complete);
    end

    B --> C{Loop};

    subgraph Loop
        direction TB
        C0(Handle Initial State: handleInitialStateManagement()) --> C1(Process Serial Commands: processCommand());
        C1 --> C2(Read Sensors: gps_read, ms5611_read, ICM_20948_read, kx134_read);
        C2 --> C3(Update Kalman Filter: kalman_predict, kalman_update_accel, kalman_update_mag);
        C3 --> C4(Log Data: WriteLogData());
        C4 --> C5(Update Guidance Control: guidance_update, Servo Writes);
        C5 --> C6(Process Flight State: ProcessFlightState());
        C6 --> C0; % Loop back
    end

    % Simplified State Machine (details in State Machine.md)
    subgraph StateMachine_Conceptual [State Machine via ProcessFlightState()]
        direction TD
        PAD_IDLE --> ARMED;
        ARMED --> BOOST;
        BOOST --> COAST;
        COAST --> APOGEE;
        APOGEE --> DROGUE_DEPLOY;
        DROGUE_DEPLOY --> DROGUE_DESCENT;
        DROGUE_DESCENT --> MAIN_DEPLOY;
        MAIN_DEPLOY --> MAIN_DESCENT;
        MAIN_DESCENT --> LANDED;
        LANDED --> RECOVERY;
        %% Add ERROR state and transitions from various points
        COAST --> ERROR_STATE[ERROR];
        PAD_IDLE --> ERROR_STATE;
        ERROR_STATE --> PAD_IDLE; % Recovery
    end
     C6 --> StateMachine_Conceptual;

```
*Note: The "State Machine Detail" diagram above is a simplified representation. For the canonical list of all 14 flight states and their detailed transitions, please refer to `State Machine.md`.*

## Function Documentation (`src/TripleT_Flight_Firmware.cpp`)

This section details key functions, primarily focusing on those in `TripleT_Flight_Firmware.cpp` and their interactions with major modules.

---

### Core `setup()` and `loop()`

**`setup()`**
*   **Purpose:** Runs once on power-up or reset. Initializes hardware (Serial, I2C, NeoPixel, pyro channels, servos, buzzer), attempts to recover state from EEPROM via `recoverFromPowerLoss()`, initializes SD card, GPS, all sensors (MS5611, ICM-20948, KX134), Kalman filter, guidance system, and creates an initial log file. Prints initial sensor status.
*   **Called By:** Arduino Core / Teensyduino Bootloader.
*   **Trace:** Entry point after boot.

**`handleInitialStateManagement()`**
*   **Purpose:** Called once at the beginning of `loop()`. Performs initial system health checks using `isSensorSuiteHealthy()`. Based on the current state (either fresh `STARTUP` or recovered from EEPROM) and system health, it transitions to `CALIBRATION`, `PAD_IDLE`, or `ERROR` state.
*   **Called By:** `loop()`
*   **Trace:** `loop()` -> `handleInitialStateManagement()`

**`loop()`**
*   **Purpose:** Main repeating function.
    1.  Calls `handleInitialStateManagement()` (once).
    2.  Processes incoming serial commands via `processCommand()`.
    3.  Periodically reads sensor data: `gps_read()`, `ms5611_read()`, `ICM_20948_read()`, `kx134_read()`.
    4.  Periodically reads battery voltage via `read_battery_voltage()` (if `ENABLE_BATTERY_MONITORING` is 1).
    5.  Updates the Kalman filter (`kalman_predict()`, `kalman_update_accel()`, `kalman_update_mag()`) with new sensor data.
    6.  Logs data to SD card and/or serial via `WriteLogData()` if sensors were updated. This now includes battery voltage.
    7.  Updates the guidance control system via `guidance_update()` and writes to servo actuators if applicable.
    8.  Manages the main flight logic via `ProcessFlightState()`.
    9.  Periodically prints battery voltage to Serial if `g_debugFlags.enableBatteryDebug` is true.
*   **Called By:** Arduino Core.
*   **Trace:** Continuously called after `setup()`.

---
### Data Logging & Storage (`src/TripleT_Flight_Firmware.cpp`)

**`checkStorageSpace(SdFat& sd_obj_ref, uint64_t& availableSpace_out_ref)`**
*   **Purpose:** Checks available space on the SD card. Warns if space is low.
*   **Called By:** `setup()`, `printSDCardStatus()` (in `command_processor.cpp`)
*   **Trace:** `setup()` / `processCommand()` -> `printSDCardStatus()` -> `checkStorageSpace()`

**`createNewLogFile(SdFat& sd_obj, SFE_UBLOX_GNSS& gnss, FsFile& logFile_obj_ref, char* logFileName_out_buf, size_t logFileName_out_buf_size)`**
*   **Purpose:** Creates a new log file on the SD card. Filename is timestamped if GPS fix is available, otherwise uses `millis()`. Writes CSV header based on `LOG_COLUMNS` from `log_format_definition.h`.
*   **Called By:** `setup()`, `WriteLogData()` (for recovery), `attemptToStartLogging()` (in `command_processor.cpp`)
*   **Trace:** `setup()` / `WriteLogData()` -> `loop()` / `processCommand()` -> `attemptToStartLogging()` -> `createNewLogFile()`

**`closeAllFiles(FsFile& logFile_obj_to_close)`**
*   **Purpose:** Safely closes the provided log file object.
*   **Called By:** `prepareForShutdown()` (in `command_processor.cpp`)
*   **Trace:** `processCommand()` -> `prepareForShutdown()` -> `closeAllFiles()`

**`WriteLogData(bool forceLog)`**
*   **Purpose:** Populates a `LogData` struct with current sensor readings (GPS, barometer, IMUs), Kalman filter orientation (roll, pitch, yaw, quaternions), GNC data (PID targets, integrals, actuator outputs), **battery voltage (`g_battery_voltage`)**, flight state, and timestamps. Converts the struct to a CSV string using `logDataToString()` (from `utility_functions.cpp`) and writes it to the SD card via `g_LogDataFile.println()`. Handles log file opening/recovery if necessary. Also outputs CSV to serial if `g_debugFlags.enableSerialCSV` is true. Logs periodically or when `forceLog` is true.
*   **Called By:** `loop()` (after sensor updates), `ProcessFlightState()` (on state transitions, indirectly by setting `newStateSignal` that `ProcessFlightState` checks to then call `WriteLogData(true)`).
*   **Trace:** `loop()` / `ProcessFlightState()` -> `loop()`

---
### Status & Debug Output (`src/TripleT_Flight_Firmware.cpp`)

**`printStatusSummary()`**
*   **Purpose:** Prints a formatted summary of system status including time, GPS, IMU, barometer, storage, and debug flags to the Serial monitor. Called when `g_debugFlags.enableStatusSummary` is true.
*   **Called By:** `loop()` (indirectly, via `processCommand` toggling the flag, then `loop` checks flag) and `processCommand` if 'j' or 'summary' is used.
*   **Trace:** `processCommand()` -> `printStatusSummary()` or `loop()` checks flag set by `processCommand()`.

---
### Initialization Helper (`src/TripleT_Flight_Firmware.cpp`)

**`initSDCard(SdFat& sd_obj, bool& sdCardMounted_out, bool& sdCardPresent_out)`**
*   **Purpose:** Initializes the SD card using `SdioConfig(FIFO_SDIO)`. Updates output parameters indicating mount status and presence.
*   **Called By:** `setup()`, `attemptToStartLogging()` (in `command_processor.cpp`)
*   **Trace:** `setup()` / `processCommand()` -> `attemptToStartLogging()` -> `initSDCard()`

---
## Module Function Summaries

### Flight Logic (`src/flight_logic.cpp`)

**`ProcessFlightState()`**
*   **Purpose:** Core state machine. Handles transitions between `FlightState`s based on sensor data, timers, and system health (`isSensorSuiteHealthy()`). Manages state-specific actions:
    *   Sets LED indicators via `setFlightStateLED()` (except for `RECOVERY` state, which manages its own strobe).
    *   Calculates dynamic main deployment altitude (`g_main_deploy_altitude_m_agl`) in `ARMED` state.
    *   Captures orientation for attitude hold (`guidance_set_target_orientation_euler()`) at `BOOST` to `COAST` transition.
    *   Calls `detectBoostEnd()`, `detectApogee()`, `detectLanding()`.
    *   Triggers pyro channels (`PYRO_CHANNEL_1`, `PYRO_CHANNEL_2`) during `DROGUE_DEPLOY` and `MAIN_DEPLOY`.
    *   Activates SOS recovery buzzer pattern, LED strobe pattern, and GPS serial beacon in `RECOVERY` state.
    *   Manages `ERROR` state behavior, including automatic recovery attempts and grace period.
*   **Called By:** `loop()` (in `TripleT_Flight_Firmware.cpp`)

**`setFlightStateLED(FlightState state)`**
*   **Purpose:** Sets NeoPixel LED color based on the current `FlightState`. For `RECOVERY` state, it initializes LEDs to off, as `ProcessFlightState` handles the specific strobe pattern.
*   **Called By:** `ProcessFlightState()`, `setup()` (indirectly via `handleInitialStateManagement`)

**`detectBoostEnd()`**
*   **Purpose:** Detects motor burnout by checking if acceleration magnitude (`get_accel_magnitude()`) drops below `COAST_ACCEL_THRESHOLD`. Sets `boostEndTime`.
*   **Called By:** `ProcessFlightState()` (in `BOOST` state)

**`detectApogee()`**
*   **Purpose:** Detects apogee using four redundant methods:
    1.  Barometric: `g_maxAltitudeReached` vs current altitude (`APOGEE_CONFIRMATION_COUNT`).
    2.  Accelerometer (ICM-20948): Negative Z-axis acceleration (`APOGEE_ACCEL_CONFIRMATION_COUNT`).
    3.  GPS: Decreasing GPS altitude (`APOGEE_GPS_CONFIRMATION_COUNT`).
    4.  Timer: `BACKUP_APOGEE_TIME_MS` after `boostEndTime`.
*   **Called By:** `ProcessFlightState()` (in `COAST` state)

**`detectLanding()`**
*   **Purpose:** Detects landing using a combination of averaged barometric altitude stability near `g_launchAltitude` (`LANDING_ALTITUDE_STABLE_THRESHOLD`) and stable accelerometer magnitude (`LANDING_ACCEL_MIN_G` to `LANDING_ACCEL_MAX_G`) for `LANDING_CONFIRMATION_TIME_MS`.
*   **Called By:** `ProcessFlightState()` (in `DROGUE_DESCENT` if no main, and `MAIN_DESCENT` states)

### State Management (`src/state_management.cpp`)

**`saveStateToEEPROM()`**
*   **Purpose:** Saves the `FlightStateData` struct (current state, altitudes including `g_main_deploy_altitude_m_agl`, timestamp, signature) to EEPROM. Called periodically and on critical state changes.
*   **Called By:** `ProcessFlightState()` (indirectly via `g_currentFlightState` changes), `handleInitialStateManagement()`, `processCommand()` (indirectly via state changes like `arm`).

**`recoverFromPowerLoss()`**
*   **Purpose:** Loads `FlightStateData` from EEPROM. If valid, restores global state variables (`g_currentFlightState`, `g_launchAltitude`, `g_maxAltitudeReached`, `g_main_deploy_altitude_m_agl`) and determines the appropriate state to resume operation.
*   **Called By:** `setup()` (via `handleInitialStateManagement()` in `TripleT_Flight_Firmware.cpp`)

### Sensor Interface Modules

#### GPS Functions (`src/gps_functions.cpp`)
*   **`gps_init()`:** Initializes u-blox GPS, configures I2C, UBX output, message rates, Auto-PVT.
*   **`gps_read()`:** Reads PVT data, updates global GPS variables (latitude, longitude, altitude, speed, fixType, SIV, time).
*   **`gps_print()`:** Prints formatted GPS data if debug enabled.
*   **`getGPSDateTime(int& year, ...)`:** Safely provides GPS date/time.
*   **`getFixType()`:** Returns current GPS fix type.
*   **`getGPSAltitude()`:** Returns GPS altitude in meters.
*   **`setGPSDebugging(bool enable)`:** Enables/disables verbose GPS debug output.
*   **REMOVED:** `checkGPSConnection()` (unused).

#### MS5611 Barometer Functions (`src/ms5611_functions.cpp`)
*   **`ms5611_init()`:** Initializes MS5611 sensor, checks connection, reads calibration coefficients.
*   **`ms5611_read()`:** Reads raw pressure and temperature, performs calculations. Updates global `pressure`, `temperature`.
*   **`ms5611_get_altitude()`:** Calculates altitude from current pressure and `g_launchAltitude` (if calibrated).
*   **`ms5611_calibrate_with_gps(uint32_t timeout_ms)`:** Calibrates barometer using GPS altitude as reference. Sets `g_baroCalibrated` and `baro_altitude_offset`.
*   **`ms5611_get_pressure()` and `ms5611_get_temperature()`:** Return current readings.

#### ICM-20948 IMU Functions (`src/icm_20948_functions.cpp`)
*   **`ICM_20948_init()`:** Initializes ICM-20948, sets ranges, enables FIFO, loads mag calibration from EEPROM.
*   **`ICM_20948_read()`:** Reads accelerometer, gyroscope, magnetometer data. Updates global `icm_accel`, `icm_gyro`, `icm_mag`, `icm_temp`, and applies calibration. Calculates `isStationary`.
*   **`ICM_20948_print()`:** Prints formatted IMU data.
*   **`ICM_20948_calibrate_mag_interactive()`:** Interactive magnetometer calibration routine.
*   **`ICM_20948_calibrate_gyro_bias(int num_samples, int delay_ms)`:** Calibrates gyro biases.
*   **`icm_20948_save_calibration()`:** Saves magnetometer calibration to EEPROM.
*   **`icm_20948_load_calibration()`:** Loads magnetometer calibration from EEPROM.
*   **`ICM_20948_get_calibrated_gyro(float* dest_array)`:** Provides calibrated gyroscope data.
*   **`icm_20948_get_mag(float* dest_array)`:** Provides calibrated magnetometer data.

#### KX134 Accelerometer Functions (`src/kx134_functions.cpp`)
*   **`kx134_init()`:** Initializes KX134, sets range and output data rate.
*   **`kx134_read()`:** Reads accelerometer data. Updates global `kx134_accel`.
*   **`kx134_print()`:** Prints formatted KX134 data.

### Kalman Filter (`src/kalman_filter.cpp`)
*   **`kalman_init(float initial_roll, ...)`:** Initializes Kalman filter state and covariance.
*   **`kalman_predict(float gyro_x, ..., float dt)`:** Predicts next state using gyro data.
*   **`kalman_update_accel(float accel_x, ...)`:** Updates state with accelerometer measurements for roll/pitch.
*   **`kalman_update_mag(float mag_x, ...)`:** Updates yaw estimate using tilt-compensated magnetometer data.
*   **`kalman_get_orientation(float &roll, ...)`:** Provides current estimated roll, pitch, yaw.

### Guidance & Control (`src/guidance_control.cpp`)
*   **`guidance_init()`:** Resets PID controller states and target orientation.
*   **`guidance_set_target_orientation_euler(float tr_rad, ...)`:** Sets the desired target orientation.
*   **`guidance_update(float current_roll_rad, ..., float deltat)`:** Calculates PID outputs for roll, pitch, yaw based on current and target orientation, and rates. Updates internal actuator outputs.
*   **`guidance_get_actuator_outputs(float& out_x, ...)`:** Provides calculated actuator commands.
*   **`guidance_get_target_euler_angles(float& out_target_roll_rad, ...)`:** Provides current target orientation.
*   **`guidance_get_pid_integrals(float& out_integral_roll, ...)`:** Provides current PID integral terms.

### Utility Functions (`src/utility_functions.cpp`)
*   **`scan_i2c()`:** Scans I2C bus for connected devices.
*   **`initNeoPixel(Adafruit_NeoPixel& pixels_obj)`:** Initializes NeoPixel strip.
*   **`listRootDirectory()`:** Lists files on SD card.
*   **`get_accel_magnitude(...)`:** Calculates acceleration magnitude, preferring KX134 if available, else ICM-20948.
*   **`logDataToString(const LogData& data)`:** Converts `LogData` struct to a CSV string based on `LOG_COLUMNS`.
*   **`convertQuaternionToEuler(...)` and `convertEulerToQuaternion(...)`:** Quaternion/Euler angle conversion utilities.
*   **`isSensorSuiteHealthy(FlightState currentState, bool verbose)`:** Checks health of critical sensors based on current flight state requirements.
*   **`getStateName(FlightState state)`:** Returns human-readable string for a `FlightState` enum.
*   **`read_battery_voltage()`:** Reads the analog pin defined by `BATTERY_VOLTAGE_PIN`, applies voltage divider calculation using `VOLTAGE_DIVIDER_R1`, `VOLTAGE_DIVIDER_R2`, `ADC_REFERENCE_VOLTAGE`, and `ADC_RESOLUTION` to return the calculated battery voltage. Returns `0.0f` if `ENABLE_BATTERY_MONITORING` is `0`.
    *   **Voltage Divider Construction Note:** Connect Battery(+) to R1. Connect the junction of R1 and R2 to the `BATTERY_VOLTAGE_PIN`. Connect R2 to Ground. The formula used is `V_battery = V_adc * (R1 + R2) / R2`. Ensure the voltage at the ADC pin (`V_adc`) does not exceed `ADC_REFERENCE_VOLTAGE`.
*   **Debug print functions (`printDebugHeader`, `printDebugValue`, etc.):** Helpers for formatted serial output.
*   **REMOVED/UNUSED:** `formatNumber()` (superseded by `logDataToString`), `IsStable()`, `checkWatchdog()`.

### Command Processor (`src/command_processor.cpp`)
*   **`processCommand(String command, FlightState& currentFlightState_ref, ...)`:** Parses serial input and executes corresponding actions (toggling debug flags including `enableBatteryDebug`, initiating calibration, arming, error clearing, status reports, etc.). Interacts with most other modules.
*   **Helper functions:**
    *   `printHelpMessage()`, `printSDCardStatus()`, `attemptToStartLogging()`, `printStorageStatistics()`, `toggleDebugFlag()`, `performCalibration()`, `printSystemStatus()`, `prepareForShutdown()`, `setOrientationFilter()`, `getOrientationFilterStatus()`. These are called by `processCommand`.

---
### Potentially Orphaned/Unused Functions (Summary from previous analysis)

*   **`IsStable()`:** (Likely in `TripleT_Flight_Firmware.cpp` or older utility) - Unused.
*   **`checkWatchdog()`:** (Likely in `TripleT_Flight_Firmware.cpp` or older utility) - Watchdog system was removed/is inactive.
*   **`formatNumber(float num, int precision)`:** (In `utility_functions.cpp`) - Superseded by `logDataToString` and direct `dtostrf` usage.
*   **`checkGPSConnection()`:** (In `gps_functions.cpp`) - Unused.

*This document provides a high-level overview. For detailed arguments, return types, and specific behaviors, refer to the source code and header files.*
