# Rust Viability Analysis for the TripleT Flight Firmware Project

## 1. Project Overview

The TripleT Flight Firmware is a comprehensive flight control system for model rockets, built for the Teensy 4.1 microcontroller. It manages all flight phases, from launch to recovery, using a sophisticated sensor suite and a 14-state flight state machine. The system includes advanced features like a Kalman filter for sensor fusion, dual-accelerometer strategy, redundant apogee detection, and a real-time web interface for data visualization. The project is written in C++ and uses the Arduino framework.

## 2. Firmware Analysis

### 2.1. Hardware and Dependencies

The firmware is designed for the Teensy 4.1 and relies on several external sensors and libraries:

*   **Microcontroller**: Teensy 4.1
*   **Sensors**:
    *   ICM-20948 9-DOF IMU
    *   KX134 high-G accelerometer
    *   MS5611 barometric pressure sensor
    *   u-blox GPS module
*   **Libraries**:
    *   SparkFun and Adafruit libraries for the sensors
    *   `SdFat` for SD card logging
    *   `PWMServo` for actuator control

### 2.2. Architecture and Code Structure

The C++ firmware is well-structured, with clear separation of concerns. Key components include:

*   **Main Loop (`TripleT_Flight_Firmware.cpp`)**: Handles initialization, sensor reading, Kalman filter updates, data logging, and the main flight state machine.
*   **Sensor Functions**: Dedicated files for each sensor (`icm_20948_functions.cpp`, `kx134_functions.cpp`, etc.).
*   **State Management (`state_management.cpp`)**: Manages the 14-state flight logic.
*   **Kalman Filter (`kalman_filter.cpp`)**: Implements the sensor fusion algorithm.
*   **Guidance Control (`guidance_control.cpp`)**: Manages the guidance system.
*   **Web Interface**: A separate component that communicates with the firmware over serial.

### 2.3. Feasibility of a Rust Port

A full or partial rewrite of the firmware in Rust is not only feasible but also offers several advantages.

#### 2.3.1. Rust on Teensy 4.1

The Teensy 4.1 is well-supported in the Rust embedded ecosystem. The `imxrt-rs` project provides a hardware abstraction layer (HAL) for the i.MX RT1062 MCU used by the Teensy 4.1. This allows for direct, low-level control of the hardware, which is essential for a real-time application like a flight controller.

#### 2.3.2. Sensor Drivers

The biggest challenge in porting the firmware to Rust would be finding or writing drivers for the various sensors. Fortunately, the Rust embedded community is very active, and many drivers are already available:

*   **ICM-20948**: There are existing crates for the MPU-9250, which is a predecessor to the ICM-20948. While a dedicated driver for the ICM-20948 might not be available, the existing MPU-9250 drivers could be adapted.
*   **KX134**: A driver for this high-G accelerometer might need to be written from scratch, but the I2C communication protocol is well-supported in Rust.
*   **MS5611**: A `ms5611` crate is available and appears to be actively maintained.
*   **u-blox GPS**: There are several crates for parsing NMEA sentences, which is the standard output format for u-blox GPS modules.

#### 2.3.3. Key Advantages of Rust

*   **Memory Safety**: Rust's ownership and borrowing rules prevent common C++ pitfalls like null pointer dereferences, buffer overflows, and data races. This is a significant advantage in a safety-critical application like a flight controller.
*   **Concurrency**: Rust's `async/await` syntax makes it easier to write non-blocking, concurrent code. This would be beneficial for handling sensor data, logging, and communication with the web interface simultaneously.
*   **Performance**: Rust offers performance comparable to C++, with more control over memory layout and zero-cost abstractions.
*   **Modern Tooling**: The Rust ecosystem comes with modern tools like Cargo for package management and building, which can simplify the development process.

## 3. Web Interface Analysis

### 3.1. Technology Stack

The web interface is a client-side application written in HTML, CSS, and JavaScript. It uses the Web Serial API to communicate with the firmware and `Chart.js` for data visualization. The interface is well-designed and provides a rich, real-time view of the flight data.

### 3.2. Feasibility of a Rust-based Alternative

While the current JavaScript-based web interface is perfectly functional, a Rust-based alternative could be built using WebAssembly (Wasm). Several Rust frameworks, such as Yew and Dioxus, allow for building front-end applications that compile to Wasm and run in the browser.

However, rewriting the web interface in Rust would be a significant undertaking and might not provide substantial benefits over the current implementation. The Web Serial API is a JavaScript API, so any Rust-based front-end would still need to interface with JavaScript to communicate with the firmware.

## 4. Overall Recommendation

**I recommend a phased approach to re-implementing the project in Rust, starting with the firmware.**

The firmware is the most critical component of the system and stands to gain the most from Rust's safety, concurrency, and performance features. The embedded Rust ecosystem is mature enough to support the hardware and sensors used in this project.

**The web interface, while a candidate for a Rust/Wasm rewrite, is less critical and the current implementation is robust and feature-rich. I recommend keeping the existing web interface for the time being.**

### Proposed Roadmap

1.  **Phase 1: Firmware Rewrite in Rust**
    *   Set up a Rust embedded development environment for the Teensy 4.1.
    *   Develop or adapt sensor drivers for the ICM-20948, KX134, MS5611, and u-blox GPS.
    *   Re-implement the core flight logic, state machine, and Kalman filter in Rust.
    *   Ensure the Rust firmware maintains the same serial communication protocol as the C++ version to remain compatible with the existing web interface.

2.  **Phase 2: (Optional) Web Interface Rewrite in Rust/Wasm**
    *   Once the Rust firmware is stable and flight-tested, consider rewriting the web interface in Rust using a framework like Yew or Dioxus.
    *   This would provide a fully Rust-based toolchain but is a lower priority than the firmware.

By focusing on the firmware first, the project can leverage the key advantages of Rust where they matter most, while minimizing the initial development effort.
