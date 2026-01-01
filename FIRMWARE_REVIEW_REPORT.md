# Comprehensive Review: TripleT Flight Firmware

**Date:** 2025-12-06  
**Target:** TripleT Flight Firmware (v0.51 Beta)  
**Platform:** Teensy 4.1 (ARM Cortex-M7)  

---

## 1. Executive Summary

The **TripleT Flight Firmware** is a sophisticated rocketry flight computer software aiming for high-performance control using Kalman filtering and state-machine logic. The project demonstrates a strong grasp of domain logic (flight phases, sensor fusion). However, the codebase currently exhibits **critical real-time stability issues**, specifically blocking I/O operations and logic bugs that pose a significant risk to flight safety.

**Key Findings:**
*   **Critical Risk**: Blocking Serial input and Pyro firing delays can freeze the flight computer during operation.
*   **High Risk**: Command processing vulnerabilities allow for Denial-of-Service (30s hang) via simple commands.
*   **Reliability**: A bug in landing detection logic may cause premature or false "Landed" state detection.
*   **Maintainability**: Excessive use of global variables and the Arduino `String` class hinders long-term stability and testing.

---

## 2. Project Understanding & Architecture

*   **Purpose**: Autonomous flight control for model rockets, managing 14 flight states from Pad Idle to Recovery.
*   **Hardware**: Teensy 4.1, ICM-20948 (IMU), KX134 (High-G Accel), MS5611 (Baro), u-blox GPS.
*   **Architecture**: Polling-based main `loop()`. Sensors are polled triggers state machine updates.
*   **Safety**: Recovery relies on pyro channels fired by the state machine. Failsafes include backup timers and redundant checks.

---

## 3. Static Code Analysis

### 3.1. Critical Real-Time Violations
*   **Issue**: Blocking Serial Read.
*   **Location**: `src/TripleT_Flight_Firmware.cpp:780`
    ```cpp
    String command = Serial.readStringUntil('\n');
    ```
*   **Impact**: `readStringUntil` waits for a timeout (default 1s) or a newline. If a partial command is received (noise, loose cable), the main loop hangs. **This will cause the rocket to crash** if it happens during instability.
*   **Remediation**: Implement a non-blocking character buffer. Read `Serial.read()` byte-by-byte into a buffer and process only when `\n` is detected.

### 3.2. Memory Management Checks (__String Class__)
*   **Issue**: Heap Fragmentation Risk.
*   **Evidence**: Extensive use of `String` objects in `command_processor.cpp` and logging.
    ```cpp
    // src/command_processor.cpp:488
    String flagNamePart = command.substring(6);
    ```
*   **Impact**: On long-duration flights, heap fragmentation can lead to allocation failures (`command` string returns null/empty), potentially locking up command processing or logging.
*   **Remediation**: Replace `String` with C-style strings (`char[]`) and `snprintf`/`strncmp`.

### 3.3. Project Structure
*   **Observation**: Over-reliance on `extern` globals across files (`g_currentFlightState`, sensors).
*   **Impact**: Makes unit testing impossible and debugging difficult when state changes unexpectedly.

---

## 4. Security Review

### 4.1. Denial of Service (DoS) via Command
*   **Issue**: `calibrate` command blocks CPU.
*   **Location**: `src/command_processor.cpp:179` calling `ms5611_calibrate_with_gps(30000)`.
*   **Vulnerability**: The code does not check if the rocket is in a safe state (e.g., `PAD_IDLE`) before running this 30-second blocking function.
*   **Scenario**: A "calibrate" command sent (or noise interpreted as such) during `COAST` or `BOOST` would freeze guidance and state transitions for 30 seconds.
*   **Remediation**: Add a strict check: `if (state != PAD_IDLE && state != CALIBRATION) return;`.

### 4.2. Input Validation
*   **Issue**: Weak parsing.
*   **Location**: `src/command_processor.cpp`. Input is trimmed but not length-limited before assignment to `String`.
*   **Remediation**: Limit input buffer size (e.g., 64 bytes) to prevent memory exhaustion attacks.

---

## 5. Performance & Real-Time Constraints

*   **Blocking Delays**:
    *   `src/flight_logic.cpp:589`: `delay(PYRO_FIRE_DURATION);`
    *   If `PYRO_FIRE_DURATION` is >10ms, this disrupts the 50Hz guidance loop (20ms period). A 500ms pyro pulse means 25 missed guidance cycles.
*   **Optimization**: Use `millis()` based state checks for pyro channels (fire ON, record time, check in next loop to turn OFF).

---

## 6. Reliability & Safety

### 6.1. Logic Bugs
*   **Issue**: "Latching" Landing Detection.
*   **Location**: `src/flight_logic.cpp:970-973`
    ```cpp
    } else {
        // Reset landing timer if altitude condition is not met
        // firstLandedTime = 0;
    }
    ```
*   **Impact**: The reset line is commented out! If `detectLanding()` returns true transiently (e.g., pressure spike), `firstLandedTime` is set. It never resets. Minutes later, a second glitch will instantly trigger `LANDED` because `millis() - firstLandedTime` is now huge.
*   **Remediation**: Uncomment the reset line.

### 6.2. Safety Failsafes
*   **Good**: Backup timer for apogee (`BACKUP_APOGEE_TIME_MS`) is present.
*   **Good**: Redundant apogee checks (Baro, Accel, GPS).

---

## 7. Build & Tooling

*   **Build System**: PlatformIO (`platformio.ini`) is correctly configured for Teensy 4.1.
*   **CI/CD**: **Missing**. No GitHub Actions or automated build verification.
*   **Testing**: `test/` folder contains manual sketches (`.ino`), not automated unit tests.
*   **Remediation**:
    *   Add `.github/workflows/build_firmware.yml`.
    *   Convert logic (Kalman, State Machine) to testable libraries and use PlatformIO's `test` command (Unity framework).

---

## 8. Recommendations & Roadmap

### Priority 1: Critical Fixes (Immediate)
1.  **Rewrite Serial Input**: Remove `Serial.readStringUntil`. Use a non-blocking `static char buffer[]`.
2.  **Fix Landing Logic**: Uncomment the timer reset in `detectLanding`.
3.  **Non-Blocking Pyro**: Change pyro firing to use `millis()` timers, removing `delay()`.
4.  **Guard Calibrate Command**: Prevent `calibrate` execution unless in `PAD_IDLE`/`CALIBRATION`.

### Priority 2: Reliability (Next Sprint)
5.  **Remove String Class**: Refactor to `char arrays`.
6.  **Add CI/CD**: Implement a basic strict compile check on Pull Request.
7.  **Unit Tests**: Write tests for `detectApogee` and `flight_logic` state transitions.

### Priority 3: Architecture (Future)
8.  **Encapsulation**: Move globals into a `FlightController` class context.
9.  **Scheduler**: Consider a simple cooperative scheduler (TaskScheduler library) instead of raw `millis()` checks in `loop()`, to ensure deterministic timing.
