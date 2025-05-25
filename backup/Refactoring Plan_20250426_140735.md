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