# TripleT Flight Firmware - Bench Test Procedure

This document outlines the steps to verify the recent safety and stability improvements on the implementation hardware (Teensy 4.1).

## 1. Watchdog Timer Verification

**Objective:** Verify that the system resets automatically if the main loop stops responding.

**Procedure:**
1.  **Modify Code Temporarily:**
    In `src/command_processor.cpp`, inside `processCommand`, add a test case:
    ```cpp
    else if (strcasecmp(command, "TEST_FREEZE") == 0) {
        Serial.println("Freezing system for 6 seconds (Watchdog should trigger)...");
        delay(6000); // Exceeds 5s watchdog timeout
    }
    ```
2.  **Upload Firmware:** Compile and upload the modified firmware.
3.  **Monitor Serial:** Open the Serial Monitor.
4.  **Trigger Freeze:** Send the command `TEST_FREEZE`.
5.  **Observe:**
    - You should see the "Freezing..." message.
    - Approximately 5 seconds later, the Teensy should reboot.
    - You will see the startup banner again.
6.  **Cleanup:** Remove the test code before flying.

## 2. Non-Blocking Serial Command Test

**Objective:** Ensure that processing serial commands does not pause sensor readings or flight logic.

**Procedure:**
1.  **Setup:** Connect the flight computer to the Serial Monitor.
2.  **Monitor Loop:** Observe the status LED (if implemented) or use the `debug` command to print loop times.
3.  **Spam Commands:** Quickly send a sequence of commands (e.g., `status`, `help`, `status` repeatedly) or type a very long string without hitting enter immediately.
4.  **Observe:**
    - The main loop should continue running smoothly.
    - If you have an LED strobing (e.g., in `PAD_IDLE`), the strobe rhythm should **not** stutter.
    - The system should respond to valid commands instantly once Enter is pressed.

## 3. Non-Blocking Pyro Firing

**Objective:** Verify that firing pyros does not pause the system (`delay()`).

**Procedure:**
1.  **Safety First:** **DISCONNECT ALL IGNITERS/CHARGES.** Use LEDs or a multimeter on the pyro output channels.
2.  **Simulate Flight:**
    - Use commands (or temporary code) to force the state machine into `DROGUE_DEPLOY` or `MAIN_DEPLOY`.
    - *Note: Since we added guards, you might need to artificially transition states in `flight_logic.cpp` for this test.*
3.  **Trigger Firing:** When the state transitions (e.g., detection of Apogee simulation):
    - The Pyro Output should turn HIGH.
4.  **Check Blocking:**
    - While the "Pyro" is ON (for the configured duration, e.g., 2 seconds), send a `status` command.
    - **Result:** The system **MUST** respond to the status command *immediately* while the pyro is still firing. If it waits until the pyro turns off, it is blocking (Fail).

## 4. Command State Guards

**Objective:** Verify that critical commands like `calibrate` are rejected in unsafe states.

**Procedure:**
1.  **Force Safe State:** In `PAD_IDLE`, send `calibrate`.
    - **Result:** Success message (Calibration performed).
2.  **Force Unsafe State:**
    - You may need to temporarily edit `setup()` to initialize state to `BOOST` or `COAST` for this test.
    - Or, assume `ERROR` state if you can trigger it.
3.  **Send Command:** Send `calibrate`.
    - **Result:** The system should print a warning ("Command not allowed in current state") and **NOT** perform calibration.

## 5. Landing Logic

**Objective:** Confirm the fix for the landing detection scope bug.

**Procedure:**
1.  **Simulate Descent:**
    - Use the mock input or modify `flight_logic.cpp` to force altitude readings to be constant.
2.  **Wait:** Allow the landing detection timer to run.
3.  **Result:** The state should transition to `LANDED` after the defined timeout (e.g., 5-10 seconds of constant altitude).
