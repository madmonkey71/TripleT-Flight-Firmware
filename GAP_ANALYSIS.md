# TripleT Flight Firmware - Gap Analysis (Source Code Reviewed)

## 1. Introduction

This document provides an analysis of the TripleT Flight Firmware project to identify gaps between the documented goals and the **current implemented source code**. The goal is to create a clear, actionable picture of what is missing to achieve a feature-complete and robust flight controller. The analysis is based on a review of the source code in the `src/` directory as of 25/05/2025.

**Note:** Per user request, analysis of the Madgwick filter and related orientation data quality issues is omitted. The focus is on the state machine, guidance logic, and overall system architecture.

## 2. Summary of Project Goals vs. Implemented Reality

The project aims to be a comprehensive flight control system. Here is a summary of the implemented state:
- **Flight State Detection:** A full 14-state machine is implemented in `flight_logic.cpp` with logic for transitions from `STARTUP` to `RECOVERY`.
- **Guidance & Control:** A full PID controller is implemented in `guidance_control.cpp`. A simple time-based target update function exists in `flight_logic.cpp`.
- **State Persistence:** EEPROM saving and loading of the flight state is implemented in `state_management.cpp`, including power-loss recovery logic.
- **Data Logging:** A sophisticated logging system is in place with a defined format.
- **Hardware Abstraction:** Functions for all core sensors (GPS, Baro, IMUs) are present.

## 3. Analysis of Core Systems (Based on Source Code)

### 3.1. Flight State Machine

- **Status:** The code in `flight_logic.cpp` and `state_management.cpp` provides a nearly complete implementation of the state machine designed in `State Machine.md`.
    - All 14 states are defined and used.
    - `ProcessFlightState()` contains logic for every state transition from `ARMED` to `LANDED`.
    - Apogee detection is based on altitude decrease (`descendingCount`).
    - Landing detection is based on altitude stability and a low acceleration magnitude (`IsStable`).
    - Pyro channels are fired in the `DROGUE_DEPLOY` and `MAIN_DEPLOY` states.
    - State is saved to EEPROM periodically and on critical events.
    - Power-loss recovery logic exists and attempts to restore the flight to a safe state.

- **Gaps & Discrepancies:**
    - **Redundancy Not Implemented:** The design document (`State Machine.md`) mentions redundant apogee detection (e.g., backup timers like `BACKUP_APOGEE_TIME_MS`). The current `detectApogee()` in `flight_logic.cpp` **only** uses the barometer-based check (`currentAbsoluteBaroAlt < previousApogeeDetectAltitude`). The backup timer is not used, which is a critical gap in redundancy.
    - **Sensor Failure Integration:** The `isSensorSuiteHealthy()` function exists in `utility_functions.cpp`, but it is **not called** within the main `ProcessFlightState()` loop. This means a sensor failure (e.g., barometer stops updating) will **not** trigger a transition to the `ERROR` state during flight. The state machine will continue to operate with potentially invalid data. This is a major gap in robustness.
    - **"Magic Numbers" in Logic:** The apogee detection logic uses `APOGEE_CONFIRMATION_COUNT` (from `config.h`), but the landing detection uses a hard-coded `stability_counter >= 10`. This should be a configurable constant.
    - **Post-Landing Logic:** After transitioning to `LANDED`, the system stays there until it is presumably powered off or reset. The documented transition from `LANDED` to `RECOVERY` is implemented, but the `RECOVERY` state itself does not have any explicit actions (like activating a GPS beacon or buzzer sequence), making it functionally similar to `LANDED`.

### 3.2. Guidance, Navigation, and Control (GNC)

- **Status:** `guidance_control.cpp` contains a complete PID controller implementation (`guidance_init`, `guidance_update`, `guidance_set_target_orientation_euler`). Servos are initialized and attached in `TripleT_Flight_Firmware.cpp`. A function `update_guidance_targets()` in `flight_logic.cpp` provides a very basic time-based yaw target adjustment. The main loop in `TripleT_Flight_Firmware.cpp` calls `guidance_update()` and `update_actuators()`.
- **Gaps:**
    - **Simplistic Guidance Logic:** This remains the largest gap. The current `update_guidance_targets()` function is a placeholder. It sets a yaw target based on `millis()`, which is not a functional guidance system. There is no logic for:
        - Gravity turns.
        - Attitude hold/stabilization during coast.
        - Following a flight plan or vector.
    - **No Integration with State Machine:** The `guidance_update()` and `update_actuators()` functions are called in **every** iteration of the main `loop()`, regardless of the flight state. This means the PID controller is active and servos are attempting to correct errors even while the rocket is in `PAD_IDLE`, `DROGUE_DESCENT`, or `LANDED`. This is incorrect and potentially dangerous. The guidance system should be explicitly enabled/disabled based on the flight state (e.g., active only in `BOOST` and `COAST`).
    - **Actuator Mapping Ambiguity:** The `guidance_control.cpp` code contains comments like `// Assuming actuator_output_y_g controls roll`. The `update_actuators` function in the main `.cpp` file maps these logical outputs directly to `servo_pitch`, `servo_roll`, and `servo_yaw`. While this mapping exists, the documentation correctly identifies that this needs to be more clearly defined and configurable, as the physical orientation of the flight controller relative to the actuators is critical.
    - **No Guidance Failsafes:** There is no logic to handle a situation where the rocket's orientation deviates significantly from the target. The PID controller will simply continue to try to correct, potentially to maximum servo deflection, with no higher-level logic to intervene.

### 3.4. Data Logging & Telemetry

- **Status:** Logging is well-implemented. The `log_format_definition.cpp` and header define a clear structure, and the main file handles file creation and writing.
- **Gaps:**
    - **Live Telemetry:** This remains a planned, but entirely unimplemented, feature.
    - **Missing Critical Log Data:** The log data structure (`LogData`) is comprehensive but is missing key information for post-flight analysis of the GNC system. Specifically, it does not log:
        - **PID Target Angles:** The `target_roll`, `target_pitch`, `target_yaw` are not logged.
        - **PID Controller Internals:** The integral term values for each PID controller (`pid_x_integral`, etc.) are not logged.
        - **Actuator Outputs:** The final, normalized servo commands are not logged. Without this data, it is impossible to debug or tune the performance of the control system.

## 4. Summary of Key Missing Features (Revised)

1.  **Robust State Machine Failsafes:**
    - Integration of sensor health checks (`isSensorSuiteHealthy()`) into the main flight loop to trigger the `ERROR` state.
    - Implementation of the backup apogee timer logic in `detectApogee()`.
2.  **Functional Guidance Engine:**
    - Replacement of the placeholder `update_guidance_targets()` with mission-aware logic (e.g., attitude hold during coast as a first step).
    - State-based activation/deactivation of the PID controllers and actuators.
3.  **Complete GNC Data Logging:**
    - Addition of PID target angles, integral values, and final actuator outputs to the `LogData` structure and log file.
4.  **Live Radio Telemetry:** The entire subsystem for transmitting live flight data is absent.
5.  **Refinement and Configuration:**
    - Replace hard-coded values (like landing detection count) with constants in `config.h`.
    - Implement meaningful actions in the `RECOVERY` state.
    - Implement persistent storage for sensor calibration values (as noted in original analysis).

## 5. Recommendations & Path to Completion (Revised)

The recommended order of operations is adjusted based on the source code review:

1.  **Fortify the State Machine:**
    - **Priority 1:** Call `isSensorSuiteHealthy()` in `ProcessFlightState()` and transition to `ERROR` on failure. This is a critical safety feature.
    - **Priority 2:** Implement the backup apogee timer logic in `detectApogee()`.
    - **Priority 3:** Move hard-coded values to `config.h`.
2.  **Log GNC Data:** Before developing the guidance engine, update the logging format to include PID targets, integrals, and outputs. This is essential for debugging the next steps.
3.  **Implement State-Based GNC:** Modify the main `loop()` to only call `guidance_update()` and `update_actuators()` when `currentFlightState` is `BOOST` or `COAST`. In all other states, servos should be centered or detached.
4.  **Develop the Guidance Engine (Iterative Approach):**
    - **Phase 1 (Attitude Hold):** In the `COAST` state, set the target orientation to be the orientation captured at the moment of transition from `BOOST` to `COAST`. The PID controller should then work to maintain this orientation.
    - **Phase 2 (Simple Maneuver):** Implement a basic pitch-over maneuver that sets a non-zero pitch target for a few seconds after liftoff.
5.  **Implement Live Telemetry:** Treat this as a separate, parallel task once the core flight computer is reliably flying.
6.  **Flesh out Final States:** Implement meaningful actions for the `RECOVERY` state (e.g., buzzer sequence). 