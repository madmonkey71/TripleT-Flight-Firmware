# Migration Analysis: NXP (Teensy 4.1) to STM32

**Date:** 2025-05-18
**Project:** TripleT Flight Firmware
**Target Platform:** STM32 (Specific family TBD, likely F4, F7, or H7)

## 1. Executive Summary
The migration of the TripleT Flight Firmware from Teensy 4.1 (NXP i.MX RT1062) to an STM32-based platform is **highly viable**. The codebase is primarily written in C++ using the Arduino framework, which offers a high degree of portability. The core logic (Guidance, Kalman Filtering, Flight State Machine) is hardware-agnostic.

The primary challenges lie in hardware-specific libraries (`PWMServo`, `SerialFlash`), peripheral initialization (SDIO SD Card, I2C/SPI clocks), and pin mapping. No architectural blockers exist.

## 2. Hardware Platform Comparison

| Feature | Teensy 4.1 (Current) | STM32 Equivalent (e.g., STM32H7 / F7 / F4) | Impact |
| :--- | :--- | :--- | :--- |
| **MCU** | NXP i.MX RT1062 (Cortex-M7) | Cortex-M7 (H7/F7) or Cortex-M4 (F4) | **Low**. Code is standard C++. FPU is available on both. |
| **Clock Speed** | 600 MHz | 168 MHz - 480 MHz | **Negligible**. The 50Hz control loop is well within the capabilities of even a 168MHz STM32F4. |
| **Flash/RAM** | 8MB Flash / 1MB RAM | Var. (e.g., 1MB Flash / 192KB+ RAM) | **Medium**. Teensy has massive resources. STM32 selection must ensure sufficient Flash/RAM, specifically for buffering logs. |
| **EEPROM** | Emulated (4KB) | Emulated in Flash | **Low**. Requires verification of the persistence mechanism. |
| **Storage** | Built-in SDIO MicroSD | SDIO / SDMMC Peripheral | **Medium**. Requires specific SD hardware configuration. |

## 3. Codebase Analysis & Portability

### 3.1 Core Logic (High Portability)
The following modules are platform-independent and should compile with minimal to no changes:
*   `flight_logic.cpp` / `state_management.cpp` (State Machine)
*   `kalman_filter.cpp` (Math heavy, standard C++)
*   `guidance_control.cpp` (Control theory, standard C++)
*   `log_format_definition.cpp`
*   `command_processor.cpp`

### 3.2 Hardware Dependencies (Action Required)

The following areas contain Teensy-specific code that must be refactored:

#### **A. Libraries**
1.  **`PWMServo.h`**
    *   **Current:** Teensy-specific library by Paul Stoffregen for precise servo control.
    *   **Action:** Replace with the standard Arduino `Servo.h` library (which has STM32 support) or a specialized STM32 PWM library if high-precision timing is critical.
2.  **`SerialFlash.h`**
    *   **Current:** SPI Flash library optimized for Teensy.
    *   **Action:** Replace with a generic SPI Flash library compatible with STM32, such as `SPIMemory` or standard `SPI` transactions if writing raw drivers.
3.  **`SdFat` / `SdioConfig(FIFO_SDIO)`**
    *   **Current:** Uses `FIFO_SDIO` configuration which is specific to the Teensy's SD controller implementation in `SdFat`.
    *   **Action:** Update `initSDCard` in `TripleT_Flight_Firmware.cpp` and `config.h` to use standard STM32 SDIO/SDMMC configurations supported by the STM32 Arduino Core.

#### **B. Pin Definitions & Configuration (`config.h`)**
*   **Current:** Uses integer pin numbers (e.g., `2`, `9`, `21`) specific to the Teensy breakout.
*   **Action:** Remap all pins in `config.h`. STM32 pins are typically referenced by name (e.g., `PA0`, `PB_5`) or board-specific indices.
    *   *Recommendation:* Create a `hardware_pins.h` abstraction to separate pin definitions from logic.

#### **C. Initialization Code (`TripleT_Flight_Firmware.cpp`)**
*   **Board Check:** `#define BOARD_TEENSY41` and associated checks need removal or updating.
*   **Wire/SPI:** `Wire.setClock(400000)` is standard, but ensure the STM32 I2C pins are correctly defined in the board variant or `Wire.setSDA()`/`Wire.setSCL()` are called if using non-default pins.
*   **EEPROM:** `recoverFromPowerLoss()` uses `EEPROM.get/put`. The STM32 Arduino core supports this via Flash emulation, but it often requires `EEPROM.begin()` which is not present in the Teensy implementation.

## 4. Build System (`platformio.ini`)

The build environment needs a complete addition for STM32.

**Current:**
```ini
[env:teensy41]
platform = teensy
board = teensy41
framework = arduino
```

**Proposed (Example for STM32F405):**
```ini
[env:stm32f405]
platform = ststm32
board = genericSTM32F405RG
framework = arduino
build_flags =
    -D ENABLE_HW_SERIAL3
    -D USBCON
    -D PIO_FRAMEWORK_ARDUINO_ENABLE_CDC
lib_deps =
    ; ... (Common deps) ...
    ; Remove PWMServo, SerialFlash
    ; Add STM32 specific replacements if needed
```

## 5. Migration Step-by-Step Plan

1.  **Environment Setup:** Create a new `[env:stm32...]` in `platformio.ini`.
2.  **Pin Mapping:** Create a branch `feature/stm32-migration`. Modify `config.h` to use conditional compilation (`#ifdef TEENSY`, `#ifdef STM32`) for pin definitions.
3.  **Library Replacement:**
    *   Abstract Servo control: Create a wrapper (e.g., `ActuatorControl.h`) that uses `PWMServo` on Teensy and `Servo` on STM32.
    *   Abstract Storage: Refactor SD and Flash initialization into a `StorageManager` class that handles the platform-specific setups.
4.  **EEPROM Handling:** Add `#ifdef` blocks to handle `EEPROM.begin()` for STM32 in `setup()`.
5.  **Compilation & Dry Run:** Compile for STM32 target. Fix compiler errors related to missing Teensy headers.
6.  **Hardware Verification:**
    *   Verify I2C sensor scanning.
    *   Verify SD Card mounting (SDIO).
    *   Verify Servo PWM output on an oscilloscope.
    *   Verify Flash memory persistence.

## 6. Conclusion
The transition to STM32 is a standard porting exercise. The code is well-structured, minimizing the risk. The main effort will be in the "glue" code—pins, startup, and peripheral configuration—rather than the flight logic itself.

**Estimated Effort:** 2-3 Days for a skilled firmware engineer.
