# TripleT Flight Firmware - Gap Analysis

## 1. Introduction

This document provides an analysis of the TripleT Flight Firmware project to identify gaps between the documented goals and the current state of implementation. The goal is to create a clear picture of what is missing and to propose a path toward a feature-complete and robust flight controller. The analysis is based on a review of the `README.md`, `State Machine.md`, and `guidance_plan.md` documents.

## 2. Summary of Project Goals

The project aims to be a comprehensive flight control system for model and high-power rockets. Key objectives include:
- Robust flight state detection from launch to recovery.
- Accurate orientation estimation using a 9-axis MARG sensor fusion algorithm (Madgwick AHRS).
- A PID-based control system for guidance via actuators (e.g., TVC servos).
- A dynamic targeting system to guide the rocket during flight.
- Comprehensive data logging to an SD card.
- A user-friendly serial interface and a web-based real-time data visualizer.
- Resilience against sensor failures, power loss, and other in-flight anomalies.
- Planned live telemetry via radio.

## 3. Analysis of Core Systems

### 3.1. Flight State Machine

- **Status:** The `State Machine.md` provides an extremely detailed and robust design, including 14 states, error handling, and non-volatile storage for state recovery. The documentation appears to be more of a design specification than a reflection of a fully implemented and tested system.
- **Gaps:**
    - **Implementation Verification:** While the design is thorough, the `README.md` notes that flight state detection was "lost in a github mishap" and needs to be re-implemented. The core logic for transitioning between states (e.g., `BOOST` -> `COAST` -> `APOGEE`) based on sensor data needs to be written and validated.
    - **Redundancy and Failsafes:** The design document mentions redundant apogee detection (e.g., backup timers), but it's unclear if this logic is implemented. The practical implementation of sensor failure detection and its integration into the state machine is a critical gap.
    - **Recovery Logic:** The power-loss recovery logic outlined in `State Machine.md` is complex. For example, deciding to enter `DROGUE_DESCENT` after a power loss during `BOOST` is a critical safety feature that needs careful implementation and testing.

### 3.2. Guidance, Navigation, and Control (GNC)

- **Status:** The system has a 3-axis PID controller and the ability to control servos. The `guidance_plan.md` focuses heavily on improving the quality and stability of the orientation data from the IMU, which is a critical prerequisite for any guidance system. Magnetometer calibration has been completed, and a plan to improve gyro data is laid out.
- **Gaps:**
    - **Guidance Logic:** This is the most significant gap. The `README.md` states the "Dynamic Target Orientation System" is merely a "Time-based example framework." A true guidance system requires logic to determine the desired orientation in real-time based on the rocket's state (position, velocity) and mission objectives. There is currently no logic for:
        - Pitch-over maneuvers after launch.
        - Gravity turns.
        - Targeting a specific direction or angle of attack.
        - Maintaining a stable orientation during coast.
    - **Integration with State Machine:** The guidance logic needs to be tightly integrated with the flight state. The PID controllers should likely be disabled in most states (`PAD_IDLE`, `DESCENT`, etc.) and activated only during the powered and coasting phases of flight, with different targets for each phase. This integration is not defined.
    - **Actuator Mapping:** The `README.md` lists a "Potential Improvement" for configurable actuator axis mapping. It's unclear how the logical PID outputs (pitch, roll, yaw) are currently mapped to the physical servos. This is fundamental for the control system to function correctly.
    - **Guidance Failsafes:** There is no mention of failsafes for the guidance system, such as what to do if the rocket deviates too far from its target orientation (e.g., disable motors, disable control).

### 3.3. Sensor Integration & Data Processing

- **Status:** Core sensors are integrated, and a Madgwick AHRS filter is in place. A detailed plan exists to address the primary issue of noisy orientation data.
- **Gaps:**
    - **Execution of `guidance_plan.md`:** The plan to improve gyro stability (static calibration, online bias tuning) needs to be executed. Until the RPY data is stable and reliable, the GNC system cannot be effectively developed or tested.
    - **Persistent Calibration:** The `README.md` notes that magnetometer calibration values are hard-coded. They should be stored in non-volatile memory (EEPROM or SD card) so the device doesn't require new firmware every time it's recalibrated. This likely applies to the planned static gyro calibration as well.
    - **`isStationary` Logic:** The logic to detect if the rocket is stationary is crucial for switching Madgwick filter parameters. The `README.md` lists refining this as a future task, indicating the current implementation may be suboptimal.

### 3.4. Data Logging & Telemetry

- **Status:** SD card logging is implemented. A web interface exists for displaying data from the serial port.
- **Gaps:**
    - **Live Telemetry:** This is a planned feature but is completely missing. Implementation would require selecting a radio module (e.g., LoRa, RFM95), writing a driver for it, defining a data packet structure, and creating a ground station receiver (which could be a separate project).
    - **Completeness of Logged Data:** While many data points are logged, the `README.md` notes a goal to "Add all data to the logging." A review is needed to ensure that critical data for post-flight analysis is captured, including PID controller states (target, output, integral values) and the computed `gyroBias`.

## 4. Summary of Key Missing Features

1.  **Core Flight Logic:** The primary algorithms for detecting liftoff, motor burnout, apogee, and landing need to be re-implemented and tested.
2.  **Functional Guidance System:** The current time-based example needs to be replaced with a mission-aware guidance engine that can execute flight plans (e.g., gravity turn, attitude hold).
3.  **Live Radio Telemetry:** The entire subsystem for transmitting live flight data is absent.
4.  **Persistent Sensor Calibration:** Storing magnetometer and gyroscope calibration data on the device to avoid recompiling firmware.
5.  **GNC-State Machine Integration:** Clear logic defining how and when the PID controllers are activated/deactivated based on the flight state.
6.  **Guidance Failsafes:** Safety logic to handle off-nominal guidance situations.
7.  **Execution of Sensor Refinement Plan:** The steps outlined in `guidance_plan.md` to stabilize orientation data must be completed.

## 5. Recommendations & Path to Completion

The following is a recommended order of operations to address the identified gaps:

1.  **Stabilize Orientation Data:** Complete all phases of the `guidance_plan.md`. A reliable orientation estimate is the foundation for everything that follows. This includes implementing static gyro calibration and tuning the Madgwick filter.
2.  **Implement Persistent Calibration:** As part of the previous step, build the infrastructure to save and load sensor calibration profiles (magnetometer, gyroscope) from EEPROM or the SD card.
3.  **Re-implement Core Flight Logic:** Re-create and thoroughly test the state transition logic. Use the detailed `State Machine.md` as the blueprint. Focus on robust detection of liftoff, coast, apogee, and landing using real and simulated sensor data.
4.  **Develop the Guidance Engine:**
    - **Phase 1 (Attitude Hold):** Start with a simple guidance mode: "Attitude Hold." Once in `COAST` state, the PID controllers should be enabled to hold the orientation that existed at the moment of motor burnout. This is a crucial first step.
    - **Phase 2 (Simple Maneuver):** Implement a basic pitch-over maneuver right after liftoff.
    - **Phase 3 (Full Guidance):** Design and implement a more advanced guidance logic based on mission requirements.
5.  **Integrate GNC with State Machine:** Formally connect the guidance engine to the state machine, ensuring PID controllers are only active when appropriate.
6.  **Implement Live Telemetry:** Once the core flight computer is functional, add the hardware and software for radio telemetry. This should be treated as a separate module.
7.  **Develop Failsafes and Refine Logic:** Continuously test and add failsafes for all systems, and refine logic like `isStationary` detection based on test results. 