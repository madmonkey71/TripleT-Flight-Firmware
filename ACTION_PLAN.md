# TripleT Flight Firmware - Implementation Action Plan

## 1. Introduction
This document outlines a concrete, step-by-step action plan to address the issues identified in the `GAP_ANALYSIS.md`. The plan is prioritized to tackle the most critical safety and functionality gaps first, following the recommendations from the analysis. Each task includes a description, the files to be modified, implementation steps, and a verification method.

---

## Epic 1: Fortify the State Machine
**Goal:** Enhance the robustness and reliability of the flight state machine by implementing critical safety checks and redundancy.
**Why:** Addresses Gap Analysis section 4, item 1: "Robust State Machine Failsafes". This is the highest priority for system safety.

### Task 1.1: Integrate Sensor Health Monitoring
- **Description:** Implement continuous sensor health checks within the main flight logic loop. A critical sensor failure during flight should immediately transition the system to the `ERROR` state.
- **Files to Modify:** `src/flight_logic.cpp`.
- **Implementation Steps:**
    1.  In the `ProcessFlightState()` function, at the beginning of the `switch (currentFlightState)` block, add a check for recoverable states (e.g., all states except `LANDED`, `RECOVERY`, `ERROR`).
    2.  Inside this check, call the existing `isSensorSuiteHealthy()` function.
    3.  If `isSensorSuiteHealthy()` returns `false`, immediately set `currentFlightState = ERROR;` and `return;` to prevent further processing in a bad state.
- **Verification:**
    1.  Create a new serial command (e.g., `test_error_baro`) that forces a sensor's status to "not working".
    2.  Arm the rocket and trigger the test command.
    3.  Confirm that the flight state immediately transitions to `ERROR` and the LED indicates an error.

### Task 1.2: Implement Backup Apogee Timer
- **Description:** Add a time-based backup for apogee detection to ensure drogue deployment even if the primary barometric sensor logic fails.
- **Files to Modify:** `src/flight_logic.cpp`, `src/config.h`.
- **Implementation Steps:**
    1.  In `config.h`, verify or add `#define BACKUP_APOGEE_TIME_MS 20000` (or a suitable default).
    2.  The variable `boostEndTime` is already set in `detectBoostEnd()`. This is correct.
    3.  In `detectApogee()`, add a new condition: `if (!apogeeDetected && boostEndTime > 0 && (millis() - boostEndTime > BACKUP_APOGEE_TIME_MS))`.
    4.  If this condition is true, set `apogeeDetected = true;` and log that apogee was detected via the backup timer.
- **Verification:**
    1.  Temporarily comment out the primary barometric apogee detection logic in `detectApogee()`.
    2.  In a test run (real or simulated), confirm that after the `BOOST` and `COAST` phases, the `APOGEE` state is triggered after the `BACKUP_APOGEE_TIME_MS` duration has passed since motor burnout.

### Task 1.3: Standardize Configuration Constants
- **Description:** Remove the hard-coded "magic number" for landing detection and replace it with a named constant from the configuration file.
- **Files to Modify:** `src/flight_logic.cpp`, `src/config.h`.
- **Implementation Steps:**
    1.  In `config.h`, add `#define LANDING_CONFIRMATION_COUNT 10`.
    2.  In `detectLanding()` in `flight_logic.cpp`, change the line `if (stability_counter >= 10)` to `if (stability_counter >= LANDING_CONFIRMATION_COUNT)`.
- **Verification:**
    1.  Perform a code review to ensure the constant is used correctly.
    2.  Confirm that landing detection still functions as expected during a test.

---

## Epic 2: Enhance GNC Data Logging
**Goal:** Add critical Guidance, Navigation, and Control (GNC) data to the SD card log to enable debugging and tuning.
**Why:** Addresses Gap Analysis section 4, item 3. This is a prerequisite for developing the guidance system.

### Task 2.1: Update Log Data Structure and Format
- **Description:** Add fields for PID target angles, integral term values, and final actuator outputs to the logging structure.
- **Files to Modify:** `src/data_structures.h`, `src/log_format_definition.h`, `src/log_format_definition.cpp`.
- **Implementation Steps:**
    1.  In `src/data_structures.h`, add the following fields to the `LogData` struct:
        ```cpp
        float target_roll, target_pitch, target_yaw;
        float pid_roll_integral, pid_pitch_integral, pid_yaw_integral;
        float actuator_output_pitch, actuator_output_roll, actuator_output_yaw;
        ```
    2.  In `src/log_format_definition.h` and `.cpp`, add corresponding entries to the `LOG_COLUMNS` array for each new field. Ensure the format specifiers and pointers are correct (e.g., `{"TgtRoll", &logData.target_roll, "%.3f"}`).
- **Verification:**
    1.  Compile the firmware.
    2.  Check the header of a newly created log file on the SD card to confirm the new columns (`TgtRoll`, `PIDIntRoll`, `ActOutRoll`, etc.) are present.

### Task 2.2: Populate New Log Fields
- **Description:** Populate the new log fields with live data from the GNC system before each log write.
- **Files to Modify:** `src/TripleT_Flight_Firmware.cpp`.
- **Implementation Steps:**
    1.  In the main `loop()` function, inside the `if (currentTime - lastLogTime >= logging_interval)` block where `logData` is populated, add calls to the guidance system.
    2.  Call `guidance_get_target_euler_angles()`, `guidance_get_pid_integrals()`, and `guidance_get_actuator_outputs()` to retrieve the latest GNC values.
    3.  Assign these retrieved values to the corresponding new fields in the `logData` struct.
- **Verification:**
    1.  Run the firmware and let it create a log file.
    2.  Inspect the CSV file and confirm that the new GNC columns are populated with numerical data.

---

## Epic 3: Implement State-Based GNC Activation
**Goal:** Ensure the PID controller and actuators are only active during the appropriate phases of flight.
**Why:** Addresses Gap Analysis section 3.2. This is a critical safety and functionality fix to prevent unintended actuator movement.

### Task 3.1: Conditionally Call Guidance and Actuator Updates
- **Description:** Modify the main loop to only call the `guidance_update()` and `update_actuators()` functions when the rocket is in the `BOOST` or `COAST` flight states.
- **Files to Modify:** `src/TripleT_Flight_Firmware.cpp`.
- **Implementation Steps:**
    1.  In the main `loop()`, locate the calls to `guidance_update()` and `update_actuators()`.
    2.  Wrap these calls in an `if` statement: `if (currentFlightState == BOOST || currentFlightState == COAST) { ... }`.
    3.  Add an `else` block. Inside the `else` block, write code to command the servos to their default/neutral positions (e.g., `servo_pitch.write(SERVO_DEFAULT_ANGLE);`). This ensures they are safely centered when GNC is inactive.
- **Verification:**
    1.  With the firmware running in `PAD_IDLE` or `ARMED` state, verify that the servos do not respond to physical tilting of the device.
    2.  If possible via a test mode, force the state to `BOOST` and verify that servos become active.

---

## Epic 4: Develop a Functional Guidance Engine
**Goal:** Replace the placeholder guidance logic with a functional system, starting with attitude hold.
**Why:** Addresses Gap Analysis section 4, item 2. This implements the core purpose of the GNC system.

### Task 4.1: Implement Attitude Hold
- **Description:** Implement an "Attitude Hold" mode. When the rocket enters the `COAST` state, the guidance system's target orientation should be locked to the orientation measured at that instant.
- **Files to Modify:** `src/flight_logic.cpp`.
- **Implementation Steps:**
    1.  In `ProcessFlightState()`, find the `case BOOST:` block inside the `Continuous State Processing Logic` switch.
    2.  Just before the state transitions to `COAST`, read the current roll, pitch, and yaw from the orientation filter.
    3.  Call `guidance_set_target_orientation_euler()` with these values to set the hold target. This target will then be used by the PID controller throughout the `COAST` state.
- **Verification:**
    1.  In a test environment, manually transition the state from `BOOST` to `COAST` while the device is tilted.
    2.  Observe the logged data (from Epic 2). The target angles should become fixed, and the actuator outputs should respond to try and maintain that orientation when the device is moved.

### Task 4.2: (Future) Implement Simple Maneuver
- **Description:** Implement a basic, pre-programmed pitch-over maneuver after liftoff.
- **Implementation:** This is a future goal. It would involve modifying `update_guidance_targets()` to set a non-zero pitch target for a defined duration when the state is `BOOST`.

---

## Epic 5: Refine Final States and Configuration
**Goal:** Add functionality to the final flight states and improve configuration.
**Why:** Addresses Gap Analysis section 4, item 5.

### Task 5.1: Implement Recovery Actions
- **Description:** Add a meaningful action to the `RECOVERY` state, such as a periodic buzzer sequence to help locate the rocket.
- **Files to Modify:** `src/flight_logic.cpp`.
- **Implementation Steps:**
    1.  In `ProcessFlightState()`, find the `case RECOVERY:` block.
    2.  Add logic that uses `millis()` to create a periodic beeping pattern (e.g., beep for 200ms every 2 seconds). Use the `tone()` and `noTone()` functions.
- **Verification:**
    1.  Via a serial command, force the state to `LANDED` and wait for the transition to `RECOVERY`.
    2.  Confirm that the buzzer begins its locating sequence. 