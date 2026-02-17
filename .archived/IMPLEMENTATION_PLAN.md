# Implementation Plan - Review Actions

This plan covers the "High" and "Medium" priority recommendations from the Comprehensive Firmware Review that have not yet been implemented.

## User Review Required

> [!NOTE]
> **Unit Testing**: I will introduce a `native` test environment in PlatformIO. This allows running logic tests on the development machine (Linux) without flashing the Teensy. This is a best practice for logic verification.

## Proposed Changes

### 1. Reliability: Watchdog Timer
#### [MODIFY] [TripleT_Flight_Firmware.cpp](file:///mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware/src/TripleT_Flight_Firmware.cpp)
*   **Goal**: Enable Teensy 4.1 hardware watchdog to reset system if loop hangs > 2 seconds.
*   **Implementation**:
    *   Include `Watchdog_t4.h` (library needs to be added).
    *   Setup WDT in `setup()` with 2s timeout.
    *   Call `wdt.feed()` in `loop()`.

### 2. Testing: Unit Tests
#### [MODIFY] [platformio.ini](file:///mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware/platformio.ini)
*   **Goal**: Add `native` environment for host-based testing.
#### [NEW] [test_flight_logic.cpp](file:///mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware/test/test_flight_logic.cpp)
*   **Goal**: specific tests for `detectApogee` and State Transitions.

### 3. Maintainability: Remove String Class
#### [MODIFY] [command_processor.cpp](file:///mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware/src/command_processor.cpp)
#### [MODIFY] [TripleT_Flight_Firmware.cpp](file:///mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware/src/TripleT_Flight_Firmware.cpp)
*   **Goal**: Replace `String` object usage with `const char*` and `strncmp` to prevent heap fragmentation.
*   **Step**: Refactor `processCommand` signature to accept `const char*` instead of `String`.

## Verification Plan

### Watchdog
*   **Manual**: Add a temporary "hang" command (infinite loop) to verify WDT resets the board (indicated by reboot).

### Unit Tests
*   **Automated**: Run `pio test -e native` to execute the new test suite.

### String Refactor
*   **Automated**: Compile check.
*   **Manual**: Check help command and status command over serial.
