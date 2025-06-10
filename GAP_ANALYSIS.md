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
    - **[COMPLETE] Redundancy Implemented:** The `detectApogee()` function in `flight_logic.cpp` successfully implements a multi-method detection strategy. It uses barometric pressure, accelerometer data, GPS altitude, and a failsafe backup timer (`BACKUP_APOGEE_TIME_MS`) to ensure redundant and reliable apogee detection.
    - **[COMPLETE] Sensor Failure Integration:** The `isSensorSuiteHealthy()` function in `utility_functions.cpp` is now called from the main `ProcessFlightState()` loop. A critical sensor failure during flight will correctly trigger a transition to the `ERROR` state, significantly improving system robustness.
    - **[COMPLETE] Configurable Logic Constants:** The logic for apogee and landing detection uses constants defined in `config.h` (e.g., `APOGEE_CONFIRMATION_COUNT`, `LANDING_CONFIRMATION_TIME_MS`, `LANDING_ALTITUDE_THRESHOLD_M`). Hard-coded "magic numbers" have been eliminated from the core flight logic.
    - **[COMPLETE] Post-Landing Logic:** The `RECOVERY` state implements a periodic beeping sequence using the buzzer, providing an audible beacon for locating the vehicle after landing.

### 3.2. Guidance, Navigation, and Control (GNC)

- **Status:** `guidance_control.cpp` contains a complete PID controller implementation (`guidance_init`, `guidance_update`, `guidance_set_target_orientation_euler`). Servos are initialized and attached in `TripleT_Flight_Firmware.cpp`. A function `update_guidance_targets()` in `flight_logic.cpp` provides a very basic time-based yaw target adjustment. The main loop in `TripleT_Flight_Firmware.cpp` calls `guidance_update()` and `update_actuators()`.
- **Gaps:**
    - **Simplistic Guidance Logic:** This remains the largest gap. The current `update_guidance_targets()` function is a placeholder. It sets a yaw target based on `millis()`, which is not a functional guidance system. There is no logic for:
        - Gravity turns.
        - Attitude hold/stabilization during coast.
        - Following a flight plan or vector.
    - **[COMPLETE] Integration with State Machine:** The main loop in `TripleT_Flight_Firmware.cpp` now correctly calls `guidance_update()` and `update_actuators()` only when the `currentFlightState` is `BOOST` or `COAST`. In all other states, the servos are commanded to their default neutral position.
    - **Actuator Mapping Ambiguity:** The `guidance_control.cpp` code contains comments like `// Assuming actuator_output_y_g controls roll`. The `update_actuators` function in the main `.cpp` file maps these logical outputs directly to `servo_pitch`, `servo_roll`, and `servo_yaw`. While this mapping exists, the documentation correctly identifies that this needs to be more clearly defined and configurable, as the physical orientation of the flight controller relative to the actuators is critical.
    - **No Guidance Failsafes:** There is no logic to handle a situation where the rocket's orientation deviates significantly from the target. The PID controller will simply continue to try to correct, potentially to maximum servo deflection, with no higher-level logic to intervene.

### 3.4. Data Logging & Telemetry

- **Status:** Logging is well-implemented. The `log_format_definition.cpp` and header define a clear structure, and the main file handles file creation and writing.
- **Gaps:**
    - **Live Telemetry:** This remains a planned, but entirely unimplemented, feature.
    - **[COMPLETE] GNC Data Logging:** The `LogData` structure is comprehensive and already includes all necessary fields for GNC analysis, including PID target angles, integral values, and final actuator outputs.

## 4. Summary of Key Missing Features (Revised)

1.  **[COMPLETE] Robust State Machine Failsafes:**
    - Integration of sensor health checks (`isSensorSuiteHealthy()`) into the main flight loop is complete.
    - Implementation of redundant apogee detection, including a backup timer, is complete.
2.  **Functional Guidance Engine:**
    - Replacement of the placeholder `update_guidance_targets()` with mission-aware logic (e.g., attitude hold during coast as a first step).
    - **[COMPLETE]** State-based activation/deactivation of the PID controllers and actuators is complete.
3.  **[COMPLETE] Complete GNC Data Logging:**
    - The `LogData` structure and log file already include PID target angles, integral values, and actuator outputs.
4.  **Live Radio Telemetry:** The entire subsystem for transmitting live flight data is absent.
5.  **[COMPLETE] Refinement and Configuration:**
    - Hard-coded values have been replaced with constants in `config.h`.
    - The `RECOVERY` state now has a meaningful buzzer sequence.
    - **[PARTIAL]** Persistent storage for sensor calibration values is noted as a future improvement, but magnetometer calibration is not yet persistent.

## 5. Recommendations & Path to Completion (Revised)

The recommended order of operations is adjusted based on the source code review:

1.  **[COMPLETE] Fortify the State Machine:** All critical safety features, including sensor health checks and redundant apogee detection, are now implemented.
2.  **[COMPLETE] Log GNC Data:** The logging format is already sufficient for GNC debugging.
3.  **[COMPLETE] Implement State-Based GNC:** The main loop now correctly enables/disables the GNC system based on the flight state.
4.  **Develop the Guidance Engine (Iterative Approach):** This is the primary remaining task.
    - **Phase 1 (Attitude Hold):** In the `COAST` state, set the target orientation to be the orientation captured at the moment of transition from `BOOST` to `COAST`. The PID controller should then work to maintain this orientation.
    - **Phase 2 (Simple Maneuver):** Implement a basic pitch-over maneuver that sets a non-zero pitch target for a few seconds after liftoff.
5.  **Implement Live Telemetry:** Treat this as a separate, parallel task once the core flight computer is reliably flying.
6.  **[COMPLETE] Flesh out Final States:** The `RECOVERY` state now has a meaningful buzzer sequence. 