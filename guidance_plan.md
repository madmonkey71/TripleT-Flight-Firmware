# Guidance System Enhancement Plan: Improving Orientation Data Reliability

## 1. Introduction

The current guidance system experiences significant second-to-second variations in Roll, Pitch, and Yaw (RPY) estimates, while accelerometer data appears relatively stable. This suggests that sensor noise, particularly from the ICM-20948 gyroscope, is unduly influencing the Madgwick AHRS filter output. This plan outlines steps to diagnose and mitigate these issues to achieve stable and reliable orientation data.

## 2. Current Data Pipeline & Filtering

*   **ICM-20948 (Primary Orientation Sensor):**
    *   **Accelerometer:** Internal Digital Low-Pass Filter (DLPF) configured to ~50.4Hz bandwidth. Data is fed directly to Madgwick.
    *   **Gyroscope:** Internal DLPF configured to ~51.2Hz bandwidth. An online gyro bias estimation and correction is applied (`gyroBias` subtracted from raw, then `gyroBias` re-estimated using AHRS output). Corrected data is fed to Madgwick.
    *   **Magnetometer:** Calibrated data (`applyMagnetometerCalibration`) is fed to Madgwick.
    *   **Fusion:** `MadgwickAHRSupdateMARG` uses these three ICM-20948 sensor inputs to produce quaternions.
    *   **Output:** Quaternions are converted to Euler angles (Roll, Pitch, Yaw).

*   **KX134 (Secondary Accelerometer):**
    *   No explicit software filtering or DLPF configuration found in `kx134_functions.cpp`.
    *   Data is **not** currently used by the Madgwick AHRS filter for orientation. Used for logging and potentially by the UKF.

## 3. Problem Diagnosis & Hypotheses

*   **Primary Hypothesis:** High-frequency noise from the ICM-20948 gyroscope, not fully attenuated by its internal DLPF or adequately handled by the current gyro bias compensation, is causing instability in the Madgwick filter's orientation output.
*   **Secondary Hypotheses:**
    *   Inappropriate `beta` values for the Madgwick filter (either `MADGWICK_BETA_STATIONARY` or `MADGWICK_BETA_MOTION`) could be making the filter too sensitive to gyro noise or not responsive enough to accel/mag corrections.
    *   Suboptimal gyro bias estimation (`MADGWICK_GYRO_BIAS_LEARN_RATE` or the estimation algorithm itself).
    *   Vibrations affecting the gyroscope more than the accelerometer (or at frequencies the accel DLPF removes).
    *   Magnetometer noise or interference, primarily affecting yaw but potentially coupled into roll/pitch.

## 4. Enhancement Strategy

The strategy will focus on improving the signal-to-noise ratio of the sensor data fed into the Madgwick filter and optimizing the filter's tuning.

### Phase 4.1: Gyroscope Data Refinement (ICM-20948)

1.  **Initial Static Gyro Bias Calibration:**
    *   **Action:** Implement a routine (callable via serial command or run once at startup if no stored calibration exists) to average gyroscope readings over several seconds while the device is known to be perfectly stationary. Store these averages as initial `gyroBias` values.
    *   **Rationale:** The current online bias estimation might struggle to converge quickly or accurately if starting from zero. A good initial static bias provides a much better starting point.
    *   **Evaluation:** Observe `gyroBias` values and the corrected gyro readings (`icm_gyro[i] - gyroBias[i]`) with `enableICMRawDebug`. Corrected values should be very close to zero when stationary.

2.  **Review and Tune Online Gyro Bias Estimation:**
    *   **Action:** Analyze the effectiveness of the current online `gyroBias` estimation logic in `ICM_20948_read()`. Consider if the error calculation (`ex, ey, ez`) is optimal.
    *   **Tune `MADGWICK_GYRO_BIAS_LEARN_RATE` (in `config.h`):**
        *   If `gyroBias` drifts significantly while stationary (after static cal), slightly *increase* this rate.
        *   If `gyroBias` changes erratically during motion, *decrease* this rate.
    *   **Rationale:** Ensure the online estimation effectively tracks slow thermal drift without reacting to actual motion.

3.  **Consider Additional Low-Pass Filtering for Gyro (Software):**
    *   **Action (Conditional):** If steps 1 & 2 are insufficient, implement a simple software LPF (e.g., first-order IIR) on the *bias-corrected* gyroscope data before it's passed to the Madgwick filter.
    *   **Configuration:** Make the filter constant (alpha value) configurable in `config.h`.
    *   **Rationale:** The sensor's internal DLPF might not be aggressive enough or might have passband ripple. A software LPF adds another layer of smoothing.
    *   **Caution:** Excessive filtering introduces lag. Tune carefully.

### Phase 4.2: Madgwick Filter Tuning

1.  **Tune `beta` Parameters (`config.h`):**
    *   **Variables:** `MADGWICK_BETA_STATIONARY`, `MADGWICK_BETA_MOTION`.
    *   **Action (Iterative):**
        *   **Stationary (`MADGWICK_BETA_STATIONARY`):** With the device stationary and after gyro improvements, observe RPY stability.
            *   If still noisy/jittery, try slightly *decreasing* `MADGWICK_BETA_STATIONARY` (trusts gyro more, which should now be cleaner).
            *   If slow to correct to level or has minor residual drift due to accel/mag noise, a tiny increase might be needed, but prioritize clean sensor inputs.
        *   **Motion (`MADGWICK_BETA_MOTION`):**
            *   If orientation lags, try slightly *increasing* `MADGWICK_BETA_MOTION`.
            *   If orientation overshoots or is wobbly (especially roll/pitch affected by linear acceleration), try *decreasing* `MADGWICK_BETA_MOTION`.
    *   **Rationale:** Optimize the filter's reliance on accelerometer/magnetometer vs. gyroscope integration for both stationary and dynamic conditions.

### Phase 4.3: Accelerometer Data (ICM-20948)

1.  **Verify DLPF Impact:** The ICM-20948 accelerometer already has a DLPF. If RPY instability persists and gyro improvements are exhausted, confirm the accelerometer data fed to Madgwick (`ax, ay, az` in `MadgwickAHRSupdateMARG`) is indeed stable.
2.  **Consider Software LPF (Conditional):** As a last resort for accelerometer data, if it's identified as a noise source for Madgwick despite its DLPF, a gentle software LPF could be added.

### Phase 4.4: KX134 Accelerometer (Review and Potential Filtering)

1.  **Assess Necessity for DLPF Configuration:** Review the SparkFun KX134 library and datasheet to determine default filtering and if its internal DLPF can/should be explicitly configured in `kx134_init()`.
2.  **Software LPF (If used for sensitive tasks):** If the KX134 data (currently used in UKF and for logging) is too noisy for its intended applications, apply a software LPF in `kx134_read()` before `kx134_accel` is updated.
    *   **Rationale:** While not directly affecting Madgwick RPY, ensuring all sensor data is as clean as needed for its specific purpose is good practice.

### Phase 4.5: Magnetometer Review

1.  **Verify Calibration:** Ensure the magnetometer calibration routine (`ICM_20948_calibrate()` and `applyMagnetometerCalibration()`) is effective and being used correctly. Poor magnetometer data primarily affects yaw but can have secondary effects.
2.  **Check for Interference:** Log raw magnetometer data and look for signs of environmental magnetic interference.

## 5. Implementation Steps (Initial Focus on Gyro and Madgwick)

1.  **Create `guidance_plan.md`:** Document this plan. (This step)
2.  **Implement Static Gyro Bias Calibration:**
    *   Add a function `ICM_20948_calibrate_gyro_bias()` in `icm_20948_functions.cpp`.
    *   Call this during `setup()` in `TripleT_Flight_Firmware.cpp` (perhaps conditionally, or allow triggering via serial command).
    *   Modify `ICM_20948_read()` to use this pre-calibrated bias as the starting point for `gyroBias`.
3.  **Tune Online Gyro Bias & Madgwick Beta:**
    *   Systematically adjust `MADGWICK_GYRO_BIAS_LEARN_RATE`, `MADGWICK_BETA_STATIONARY`, and `MADGWICK_BETA_MOTION` in `config.h`.
    *   Use extensive logging (`enableICMRawDebug` and potentially add more for quaternion and Euler angle stability) and the existing web visualizer to observe effects. Test both stationary and with various motions.
4.  **Conditional: Software LPF for Gyro:**
    *   If needed, add a configurable IIR LPF to `ICM_20948_read()` for bias-corrected gyro data.
5.  **Review KX134 Filtering:** Investigate and, if necessary, configure/add filtering for KX134.
6.  **Iterate and Test:** Continuously evaluate orientation stability.

## 6. Expected Outcomes

*   Significantly reduced second-to-second variations in Roll, Pitch, and Yaw.
*   More stable and reliable orientation data for the guidance and control system.
*   Improved overall system performance and predictability. 