# TripleT Flight Firmware Function Documentation

This document outlines the functions defined within `src/TripleT_Flight_Firmware.cpp`, their purposes, and where they are called from.

## Function Details

*   **`watchdogHandler()`**
    *   **Purpose:** Callback function executed when the watchdog timer triggers (times out). Sets the `watchdogTriggered` flag to true and prints a warning message. In a future implementation, it might perform emergency actions. Currently sets `flightState` to `ERROR` (though this is commented out in the callback itself, the `watchdogCallback()` function below does this).
    *   **Called By:** Watchdog timer (hardware/library interrupt). Registered in `setup()` via `watchdog.callback(watchdogHandler)`.

*   **`checkSensorStatus()`**
    *   **Purpose:** Checks the status and validity of readings from the barometer, accelerometer, gyroscope, magnetometer, and GPS. Updates their respective `SensorStatus` structs (e.g., `barometerStatus`, `accelerometerStatus`). If critical sensors fail based on the current flight state, it transitions the system to the `ERROR` state.
    *   **Called By:** `loop()` (periodically).

*   **`isSensorSuiteHealthy(FlightState currentState)`**
    *   **Purpose:** Determines if the required set of sensors for the given `currentState` are currently working correctly based on their `isWorking` flags. Returns `true` if healthy, `false` otherwise.
    *   **Called By:** `checkSensorStatus()`, `printStatusSummary()`, `processCommand()` (in `test_error` and `status_sensors` cases).

*   **`getStateName(FlightState state)`**
    *   **Purpose:** Returns a string representation (const char*) of the given `FlightState` enum value. Used for logging and debugging output.
    *   **Called By:** `ProcessFlightState()`, `printStatusSummary()`, `processCommand()` (in `arm` and `status_sensors` cases).

*   **`detectApogee()`**
    *   **Purpose:** Implements multiple methods (barometric, accelerometer, time-based) to detect if the rocket has reached apogee (peak altitude). Returns `true` if apogee is detected, `false` otherwise.
    *   **Called By:** `ProcessFlightState()` (in `COAST` state).

*   **`detectLanding()`**
    *   **Purpose:** Implements multiple methods (accelerometer stability, barometric stability) to detect if the rocket has landed. Returns `true` if landing is detected, `false` otherwise. Sets `landingDetectedFlag` when landing is first detected.
    *   **Called By:** `ProcessFlightState()` (in `MAIN_DESCENT` state).

*   **`detectBoostEnd()`**
    *   **Purpose:** Checks if the rocket's acceleration has dropped below the `COAST_ACCEL_THRESHOLD`. If it has and the `boostEndTime` hasn't been set yet, it records the current time (`millis()`) as the end of the boost phase.
    *   **Called By:** `ProcessFlightState()` (in `BOOST` state).

*   **`checkStorageSpace()`**
    *   **Purpose:** Checks the available free space on the SD card (if available). Prints a warning if the free space drops below `SD_CARD_MIN_FREE_SPACE`. Updates the global `availableSpace` variable.
    *   **Called By:** `loop()` (periodically).

*   **`createNewLogFile()`**
    *   **Purpose:** Creates a new log file on the SD card (if available). Closes any existing log file first. Generates a unique filename based on GPS date/time (if available) or `millis()`. Writes the CSV header to the new file. Updates the global `logFileName` and sets `loggingEnabled` to true on success. Returns `true` on success, `false` on failure.
    *   **Called By:** `setup()`, `WriteLogData()`.

*   **`closeAllFiles()`**
    *   **Purpose:** Safely flushes and closes the `LogDataFile` if it's open. Resets the `LogDataFile` object.
    *   **Called By:** `prepareForShutdown()`.

*   **`prepareForShutdown()`**
    *   **Purpose:** Prepares the system for a safe shutdown. Closes log files, sets the LED to blue, plays a final buzzer tone, prints a shutdown message, and enters an infinite loop.
    *   **Called By:** `processCommand()` (case '9').

*   **`WriteLogData(bool forceLog)`**
    *   **Purpose:** Formats the current sensor data into a comma-separated string (`LogDataString`). Optionally prints this string to the Serial monitor (`enableSerialCSV`). Writes the string to the `LogDataFile` on the SD card (if available and open, creating a new file if necessary). Implements a periodic flush mechanism (`LogDataFile.flush()`) to save data without excessive wear. The `forceLog` parameter bypasses the timing check, ensuring data is logged immediately (e.g., after sensor reads or state changes).
    *   **Called By:** `loop()` (after sensor updates).

*   **`formatNumber(float input, byte columns, byte places)`**
    *   **Purpose:** Formats a floating-point number (`input`) into a string with a specified total width (`columns`) and number of decimal places (`places`), then prints it to the Serial monitor.
    *   **Called By:** *(Currently Not Called)* - This function seems to be unused.

*   **`ProcessFlightState()`**
    *   **Purpose:** The main state machine handler. It checks for state timeouts. If the state has changed since the last call, it performs initialization for the new state (setting LED color, recording times, printing messages, playing tones). Then, it executes logic based on the current `flightState` (checking for liftoff, boost end, apogee, deployment altitudes, landing) and transitions to the next state when conditions are met.
    *   **Called By:** `loop()` (on every iteration).

*   **`printStatusSummary()`**
    *   **Purpose:** Prints a formatted summary of the system status to the Serial monitor. Includes time, flight state, flight timers, max altitude, GPS status, sensor readings (barometer, accelerometer, gyro), storage status, flight metrics, system health, and watchdog status.
    *   **Called By:** `loop()` (periodically, if `enableStatusSummary` is true).

*   **`printHelpMessage()`**
    *   **Purpose:** Prints a help menu to the Serial monitor, listing available debug flags, numeric toggle commands, alphabetic commands, utility commands, test commands, and legacy commands.
    *   **Called By:** `processCommand()` (case 'a' or command "help").

*   **`printStorageStatistics()`**
    *   **Purpose:** Prints statistics about storage usage (currently only SD card usage) to the Serial monitor. Shows used space and the current log file name if available.
    *   **Called By:** `processCommand()` (case 'f').

*   **`processCommand(String command)`**
    *   **Purpose:** Parses and handles commands received via the Serial input (though the serial reading part is commented out in `loop()`). Handles single-character numeric toggles for debug flags, single-character alphabetic commands for various actions (help, status, calibrate, etc.), and longer string commands (like "calibrate", "arm", "debug_...", "test_...", "status_sensors").
    *   **Called By:** *(Currently Not Called Directly from `loop()` due to commented-out code)*

*   **`setup()`**
    *   **Purpose:** Arduino standard setup function. Runs once at power-up/reset. Initializes Serial communication, NeoPixel LEDs, watchdog timer, buzzer, SPI/I2C, performs an I2C scan, initializes GPS, waits for GPS time sync, initializes other sensors (KX134, ICM-20948, MS5611), initializes the SD card, creates the initial log file, and transitions the `flightState` from `STARTUP` to `CALIBRATION` and finally to `PAD_IDLE`.
    *   **Called By:** Arduino framework (at startup).

*   **`loop()`**
    *   **Purpose:** Arduino standard loop function. Runs repeatedly after `setup()`. Contains the main operational logic: periodically reads sensors (GPS, Barometer, IMU, Accelerometer), performs automatic barometer calibration if conditions are met, calls `WriteLogData()` after sensor updates, calls `ProcessFlightState()` to manage the flight sequence, feeds the watchdog timer, periodically checks sensor status (`checkSensorStatus()`), GPS connection (`checkGPSConnection()`), and storage space (`checkStorageSpace()`), periodically flushes the log file, and optionally prints status (`printStatusSummary()`) or detailed debug data (`ICM_20948_print()`). (Note: Serial command processing is currently commented out).
    *   **Called By:** Arduino framework (repeatedly after `setup()`).

*   **`IsStable()`**
    *   **Purpose:** Checks if the rocket is stable (minimal motion) by comparing the magnitude of acceleration to 1g (±5%). Returns `true` if stable, `false` otherwise.
    *   **Called By:** *(Currently Not Called)* - This function seems to be unused.

*   **`getBaroAltitude()`**
    *   **Purpose:** Calculates the altitude in meters based on the current pressure reading (`pressure` global variable) using the standard barometric formula. Applies the `baro_altitude_offset` if calibration has been performed (`baroCalibrated` is true). Returns 0.0 if the barometer is not connected.
    *   **Called By:** `detectApogee()`, `detectLanding()`, `ProcessFlightState()` (in `PAD_IDLE`, `APOGEE`, `DROGUE_DESCENT` states), `printStatusSummary()`.

*   **`GetAccelMagnitude()`**
    *   **Purpose:** Calculates the magnitude of the acceleration vector using data from either the KX134 or ICM-20948 accelerometer, depending on which is available (`kx134_accel_ready` or `icm20948_ready`). Returns the magnitude in g's, or 0.0 if neither accelerometer is ready.
    *   **Called By:** `checkSensorStatus()`, `detectLanding()`, `detectBoostEnd()`, `ProcessFlightState()` (in `ARMED` state), `printStatusSummary()`, `IsStable()`.

*   **`handleSensorErrors()`**
    *   **Purpose:** Tracks consecutive failures for sensors (Barometer, Accelerometer, GPS). If consecutive failures exceed a threshold, marks the sensor as not working (`isWorking = false`). If critical sensors fail, transitions `flightState` to `ERROR`.
    *   **Called By:** *(Currently Not Called)* - This function seems to be unused. Its logic appears to be integrated into `checkSensorStatus()` instead.

*   **`watchdogCallback()`**
    *   **Purpose:** Alternative watchdog callback function (distinct from `watchdogHandler`). Sets the `flightState` to `ERROR` and sets the LED to red when the watchdog timer times out.
    *   **Called By:** `processCommand()` (in `test_watchdog` case, simulating a watchdog event). *Note: `watchdogHandler` is the function actually registered with the watchdog timer in `setup()`.*

*   **`setLEDColor(uint8_t p, uint8_t r, uint8_t g, uint8_t b)`**
    *   **Purpose:** Sets the color of a specific NeoPixel LED. Takes the pixel index (`p`) and RGB color values (`r`, `g`, `b`). Updates the pixel strip using `pixels.show()`.
    *   **Called By:** `prepareForShutdown()`, `ProcessFlightState()` (state initialization and `RECOVERY`/`ERROR` blinking), `processCommand()` (in `calibrate`, `test_error`, `clear_errors` cases), `setup()`, `loop()` (automatic calibration status), `handleSensorErrors()`, `watchdogCallback()`.

*   **`checkWatchdog()`**
    *   **Purpose:** Feeds (resets) the watchdog timer. Also includes backup logic to transition out of `BOOST` or `COAST` states if they persist for too long, preventing getting stuck if primary detection fails.
    *   **Called By:** *(Currently Not Called)* - Watchdog feeding is done directly in `loop()` using `watchdog.feed()`. The state timeout logic is handled within `ProcessFlightState()`.

*(Note: Functions like `gps_init`, `gps_read`, `ms5611_init`, `ms5611_read`, `ms5611_calibrate_with_gps`, `kx134_init`, `kx134_read`, `ICM_20948_init`, `ICM_20948_read`, `ICM_20948_print`, `initSDCard`, `scan_i2c`, `setGPSDebugging`, `getGPSDateTime`, `checkGPSConnection` are called but are assumed to be defined in the included header/source files (`gps_functions.h`, `ms5611_functions.h`, `kx134_functions.h`, `icm_20948_functions.h`, `utility_functions.h`) and are not detailed here.)*

## Program Flow Diagram

This diagram illustrates the overall program flow and the flight state machine.

```mermaid
graph TD
    A[Start] --> B(setup);

    subgraph setup
        direction TB
        B_HW(Initialize Hardware) --> B_SENS(Initialize Sensors);
        B_SENS --> B_STOR(Initialize Storage);
        B_STOR --> B_CALIB(CALIBRATION);
    end

    B_CALIB -- "Setup OK" --> F(PAD_IDLE);
    B_CALIB -- "Abort/Test/Data" --> LND(LANDED); // Special case path

    F --> G(loop); // Link PAD_IDLE to loop entry

    subgraph loop [Main Loop]
        G_Start(Enter Loop) --> G_Sensors(Read Sensors);
        G_Sensors --> G_Calibrate{Auto-Calibrate?};
        G_Calibrate -- Yes --> G_CalibFunc(ms5611_calibrate_with_gps);
        G_Calibrate -- No --> G_Log(WriteLogData);
        G_CalibFunc --> G_Log;
        G_Log --> G_State(ProcessFlightState);
        G_State --> G_WDT(Feed Watchdog);
        G_WDT --> Periodic_Checks_Subgraph;
        Periodic_Checks_Subgraph --> G_Display{Display Status?};
        G_Display -- Yes --> G_Print(printStatusSummary);
        G_Display -- No --> G_EndLoop(End Iteration);
        G_Print --> G_EndLoop;
        G_EndLoop --> G_Start;
    end

    subgraph ProcessFlightState [State Machine in loop()]
        F --> ARM[ARMED]; // PAD_IDLE is the entry point now
        ARM --> BST[BOOST];
        BST --> CST[COAST];
        CST --> APG[APOGEE];
        APG --> DD[DROGUE_DEPLOY];
        DD --> DDS[DROGUE_DESCENT];
        DDS --> MD[MAIN_DEPLOY];
        MD --> MDS[MAIN_DESCENT];
        MDS --> LND; // LANDED is shared with the special case
        LND --> RCV[RECOVERY];

        %% Error Transitions
        style ERROR fill:#f99,stroke:#333,stroke-width:2px
        F -->|"Sensor Failure / Timeout"| ERROR; // PAD_IDLE Error
        ARM -->|"Sensor Failure / Timeout"| ERROR;
        BST -->|"Sensor Failure / Timeout"| ERROR;
        CST -->|"Sensor Failure / Timeout"| ERROR;
        APG -->|"Sensor Failure / Timeout"| ERROR;
        DD -->|"Sensor Failure / Timeout"| ERROR;
        DDS -->|"Sensor Failure / Timeout"| ERROR;
        MD -->|"Sensor Failure / Timeout"| ERROR;
        MDS -->|"Sensor Failure / Timeout"| ERROR;
        LND -->|"Sensor Failure / Timeout"| ERROR;
        %% Can still enter ERROR from RECOVERY via watchdog
        RCV -->|"Watchdog/Error"| ERROR

        %% Error Recovery
        ERROR -->|"clear_errors Command"| F; // Recover to PAD_IDLE
    end

    subgraph Periodic_Checks_Subgraph [Periodic Checks in loop]
        Check1(checkSensorStatus)
        Check2(checkGPSConnection)
        Check3(checkStorageSpace)
        Check4(Flush LogDataFile)
    end

```

## State Machine Diagram Comparison

Comparing the Program Flow Diagram above with the diagram in `State Machine.md`, the following differences and necessary updates are noted:

1.  **CALIBRATION State & Special Path:** The diagram in `State Machine.md` explicitly shows the `CALIBRATION` state occurring within the `SETUP` function and includes a direct transition from `CALIBRATION` to `LANDED` for special cases (abort, testing, data retrieval). The Program Flow Diagram above currently implies `CALIBRATION` happens within `setup` but doesn't show it as a distinct node, nor does it show the direct path to `LANDED`.
2.  **Detail Level:** The Program Flow Diagram provides significantly more detail regarding the specific steps within `setup` (Hardware/Sensor/Storage Initialization) and the operations performed within the main `loop` (Sensor Reading, Logging, Watchdog, Periodic Checks). It also explicitly details the state transitions within the `ProcessFlightState` subgraph, including error transitions and recovery paths, which are omitted in the `State Machine.md` diagram.
3.  **Structure:** The Program Flow Diagram uses multiple subgraphs (`loop`, `ProcessFlightState`, `Periodic_Checks_Subgraph`) to organize the details, while the `State Machine.md` diagram uses only two high-level subgraphs (`SETUP FUNCTION`, `LOOP FUNCTION`).

**Conclusion & Required Fixes:**

The Program Flow Diagram above is more representative of the actual firmware implementation due to its detail, especially regarding error handling and loop operations. However, it needs to be updated to accurately reflect the CALIBRATION state and the special transition path shown in `State Machine.md`.

**Required modifications to the Program Flow Diagram:**

*   Add an explicit `CALIBRATION` node after the `Initialize Storage` step within the `setup` sequence.
*   Change the primary transition to the loop to originate from this new `CALIBRATION` node to `PAD_IDLE`.
*   Add the secondary transition path directly from the `CALIBRATION` node to the `LANDED` node (which exists within the `ProcessFlightState` subgraph) labeled appropriately for the special cases.

## Refactoring Plan for State Machine Alignment

This plan outlines the steps to refactor `TripleT_Flight_Firmware.cpp` to better align with the conceptual state machine structure described in `State Machine.md`, where `setup()` handles initialization and calibration, and `loop()` handles the active flight states.

**Goal:** Achieve better structural alignment with the Arduino framework (`setup()` vs `loop()`) while preserving existing functionality (redundancy, error handling, EEPROM recovery, detailed states).

**Tasks:**

1.  **Modify `setup()` Function:**
    *   **Task:** Explicitly manage `STARTUP` and `CALIBRATION` states within `setup()`.
    *   **Details:**
        *   Keep initialization steps (Serial, LEDs, Watchdog, I2C/SPI, Sensor `_init` calls).
        *   Perform EEPROM check (`recoverFromPowerLoss`) early to potentially override the initial state.
        *   If not recovering from a later state, proceed with standard `STARTUP` actions.
        *   Transition internally to a `CALIBRATION` phase/state *within* `setup()`.
        *   Attempt primary barometric calibration (`ms5611_calibrate_with_gps`) during this phase, respecting timeouts.
        *   Perform final sensor/system health checks (`isSensorSuiteHealthy`).
        *   **Crucially:** Set the *final* `flightState` variable at the end of `setup()` to `PAD_IDLE` if all initial checks and required calibrations pass, or to `ERROR` if critical initialization/sensor checks fail. The special `CALIBRATION` -> `LANDED` path will be handled by transitioning to `ERROR` if a critical failure occurs during setup.

2.  **Modify `loop()` Function:**
    *   **Task:** Ensure `loop()` starts processing based on the state set by `setup()` and handles periodic tasks.
    *   **Details:**
        *   Remove any logic that assumes `loop()` starts in a specific state other than what `setup()` determined (`PAD_IDLE` or `ERROR`).
        *   Continue the existing pattern of calling `ProcessFlightState()` on each iteration.
        *   Maintain all periodic tasks: sensor reads, `WriteLogData()`, `watchdog.feed()`, `checkSensorStatus()`, `checkStorageSpace()`, `saveStateToEEPROM()`, command processing (if used), status printing.
        *   Retain the logic for *automatic* barometer calibration attempt within `loop()` if `baroCalibrated` is still false and a good GPS fix becomes available (provides robustness if `setup()` calibration failed/timed out).

3.  **Modify `ProcessFlightState()` Function:**
    *   **Task:** Remove handling for states now fully managed by `setup()`.
    *   **Details:**
        *   Delete the `case STARTUP:` and `case CALIBRATION:` sections from the `switch(flightState)` blocks (both the state initialization part and the state processing part).
        *   Ensure all remaining state cases (`PAD_IDLE` through `ERROR`) and their transitions are correct and utilize the existing detection functions (`detectBoostEnd`, `detectApogee`, `detectLanding`).

4.  **Review Error Handling during Setup:**
    *   **Task:** Ensure critical failures during `setup()` correctly lead to the `ERROR` state being set before `loop()` begins.
    *   **Details:** Check sensor initialization return values (`kx134_init`, `ICM_20948_init`, `ms5611_init`, `gps_init`). If essential sensors fail, set `flightState = ERROR`. Check `isSensorSuiteHealthy(PAD_IDLE)` at the end of `setup()` and set `flightState = ERROR` if it returns false.

5.  **Verify EEPROM Logic:**
    *   **Task:** Confirm state recovery and saving logic remains functional.
    *   **Details:**
        *   `recoverFromPowerLoss()` should still be called early in `setup()`.
        *   `saveStateToEEPROM()` should continue to be called periodically from `loop()` and potentially on critical state *transitions* within `ProcessFlightState`.

6.  **Preserve Redundancy:**
    *   **Task:** Ensure redundant detection logic is unaffected.
    *   **Details:** The functions `detectApogee()`, `detectLanding()`, and `detectBoostEnd()` should remain unchanged and correctly called from `ProcessFlightState()`.

7.  **Update Documentation:**
    *   **Task:** Update the state machine diagrams and descriptions in both documentation files (`TripleT Flight Firmware Function Documentation.md` and `State Machine.md`) to reflect the final, aligned structure.
