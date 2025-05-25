# Comprehensive Guide: Sensor Tuning, Drift Mitigation, and Flight Direction Control

This guide provides step-by-step instructions to calibrate sensors, tune the Attitude and Heading Reference System (AHRS) and Unscented Kalman Filter (UKF), and configure the flight control system for targeted flight, such as straight up.

## Part 1: Sensor Calibration - The Foundation

Accurate sensor data is paramount. Drift often originates from uncompensated sensor errors. Perform these calibrations in the order presented.

### A. Magnetometer Calibration (Hard and Soft Iron)

**Importance:** Critical for accurate yaw (heading) estimation. Must be done first and values saved persistently.

**Current Status:** Your `ICM_20948_calibrate()` function calculates biases (`magBias`) and scales (`magScale`), but **these are NOT currently saved to persistent storage (e.g., EEPROM). This is a critical action item.**

**Step-by-Step Tuning & Evaluation:**

1.  **Implement EEPROM Save/Load:**
    *   **Action:** Modify `icm_20948_functions.cpp`.
        *   In `ICM_20948_init()` (or a function it calls early):
            *   Include `<EEPROM.h>`.
            *   Define EEPROM start addresses (e.g., `int magCalibMagicAddr = 0; int magBiasAddr = magCalibMagicAddr + sizeof(uint16_t); int magScaleAddr = magBiasAddr + sizeof(float[3]);`).
            *   Define a magic number (e.g., `const uint16_t MAG_CALIB_MAGIC = 0xABCD;`).
            *   Read the magic number from `magCalibMagicAddr`.
            *   If it matches `MAG_CALIB_MAGIC`, use `EEPROM.get(magBiasAddr, magBias);` and `EEPROM.get(magScaleAddr, magScale);` to load saved values.
            *   If not, log that default values are used and calibration is needed.
        *   In `ICM_20948_calibrate()`:
            *   After calculating `magBias` and `magScale`, use `EEPROM.put(magBiasAddr, magBias);` and `EEPROM.put(magScaleAddr, magScale);`.
            *   Then, write the magic number: `EEPROM.put(magCalibMagicAddr, MAG_CALIB_MAGIC);`.
    *   **Evaluation:**
        *   After implementing, power cycle the device. Check serial logs to see if calibration data was loaded or if defaults are used.
        *   Run the calibration routine. Check logs to confirm values are saved. Power cycle again and verify the saved values are loaded.

2.  **Perform Calibration Procedure:**
    *   **Action:**
        *   Ensure the flight computer is in its final flight configuration or as close as possible, away from external magnetic disturbances.
        *   Trigger the `ICM_20948_calibrate()` function (e.g., via a serial command you might need to implement).
        *   Slowly rotate the device in multiple figure-8 patterns, ensuring all orientations are covered, for the duration of the calibration sampling (currently `numSamples = 500`).
    *   **Evaluation:**
        *   Observe the `magBias` and `magScale` values printed at the end of calibration. Biases should be non-zero, and scales should be around 1.0 but not exactly 1.0 for all axes.
        *   After calibration and saving to EEPROM, with the device stationary:
            *   Point the device's +X axis (or whichever axis aligns with "forward") towards Magnetic North. The reported yaw angle (from `convertQuaternionToEuler`) should be close to 0 degrees.
            *   Point East: Yaw ~90 degrees.
            *   Point South: Yaw ~180 degrees.
            *   Point West: Yaw ~270 degrees.
            *   Discrepancies indicate poor calibration or remaining magnetic interference. Re-calibrate in a cleaner magnetic environment if needed.

### B. Accelerometer Calibration (All Axes - ICM20948 & KX134)

**Importance:** Reduces errors in attitude estimation (Madgwick) and vertical measurements (UKF).

**Step-by-Step Tuning & Evaluation:**

1.  **Implement Calibration Routine & Storage:**
    *   **Action:**
        *   Create a new calibration function, e.g., `calibrate_accelerometers()`. This function will guide the user to place the device in 6 static orientations (+X up, -X up, +Y up, -Y up, +Z up, -Z up).
        *   For each orientation, average several seconds of raw accelerometer readings (e.g., 100-200 samples from `myICM.accX/Y/Z()` and `kx134_accel[0/1/2]`). Note: KX134 provides data in m/s^2, ICM in mg. Convert to a consistent unit (e.g., g's where 1g = 9.81 m/s^2) for calibration calculations.
        *   Calculate biases and scales:
            *   `bias_i = (reading_positive_i + reading_negative_i) / 2.0f`
            *   `scale_i = (reading_positive_i - reading_negative_i) / (2.0f * GRAVITY_G)` (where `GRAVITY_G` is 1.0 if readings are in g, or ~9.81 if in m/s^2).
        *   Store these biases (3 floats) and scales (3 floats) for *each* accelerometer (ICM20948, KX134) in EEPROM, similar to the magnetometer calibration (using different EEPROM addresses and a different magic number).
        *   Modify `ICM_20948_read()` to apply these corrections:
            `corrected_ax = (raw_ax_icm - icm_accel_bias_x) * icm_accel_scale_x;` (and similarly for Y, Z).
            Update `icm_accel[]` with these corrected values.
        *   Modify the point where `kx134_accel` data is read to apply its specific corrections.
    *   **Evaluation:**
        *   After calibration, place the device flat (Z-axis up). The corrected Z-axis reading for both sensors should be close to +1.0g (or +9.81 m/s^2). X and Y axes should be close to 0.0g.
        *   Tilt the device so +X is up. Corrected X-axis should be +1.0g, Y and Z near 0.0g. Test other orientations.
        *   Log raw vs. corrected values to verify.

### C. Gyroscope Bias Calibration

**Importance:** Minimizes rotational drift in the Madgwick filter.

**Current Method:** Online bias estimation (`gyroBias` updated in `ICM_20948_read()`).

**Step-by-Step Tuning & Evaluation:**

1.  **Implement Initial Static Bias Calibration:**
    *   **Action:**
        *   Create a function (e.g., `calibrate_gyro_initial_bias()`) that is called once at startup or via command before flight.
        *   Instruct the user to keep the device perfectly still for 30-60 seconds.
        *   During this time, average the raw gyroscope readings (`myICM.gyrX/Y/Z()`, converted to rad/s).
        *   Store these initial average values into the `gyroBias[3]` array.
        *   These can also be saved to EEPROM and loaded, so this static calibration doesn't need to run every single boot, but can be triggered when needed.
    *   **Evaluation:**
        *   Before this, observe `gyroBias` values printed during `enableICMRawDebug`. They might start at 0 and slowly converge.
        *   After implementing static calibration, the initial `gyroBias` values should be non-zero and reflect the actual sensor bias at startup. The subsequent online estimation will then start from a much better baseline.
        *   With the device stationary, the calibrated gyro readings (`icm_gyro[i] - gyroBias[i]`) should be very close to zero.

2.  **Tune `MADGWICK_GYRO_BIAS_LEARN_RATE` (in `config.h`):**
    *   **Variable:** `MADGWICK_GYRO_BIAS_LEARN_RATE` (current `0.0005f`).
    *   **Action & Evaluation:**
        *   If, after initial static calibration, the `gyroBias` values *still* drift significantly while the device is known to be stationary for long periods, you *could* slightly increase this rate.
        *   If the `gyroBias` values change erratically during motion (meaning the filter is incorrectly attributing motion to bias), *decrease* this rate.
        *   Generally, with a good initial static calibration, this learning rate can be kept relatively small to only track slow temperature-induced changes.

## Part 2: Attitude and Heading Reference System (AHRS) Tuning - Madgwick Filter

This filter (`MadgwickAHRSupdateMARG`) fuses calibrated accelerometer, gyroscope, and magnetometer data to estimate orientation.

**Step-by-Step Tuning & Evaluation:**

1.  **Tune `beta` parameters (in `config.h`):**
    *   **Variables:**
        *   `MADGWICK_BETA_INIT` (current `0.05f`): Initial beta for filter convergence.
        *   `MADGWICK_BETA_STATIONARY` (current `0.02f`): Beta when stationary.
        *   `MADGWICK_BETA_MOTION` (current `0.1f`): Beta when moving.
    *   **Action & Evaluation (General Approach):**
        *   Ensure `enableICMRawDebug` is true to see Euler angles and `isStationary` state.
        *   **Stationary Performance (`MADGWICK_BETA_STATIONARY`):**
            *   With the device stationary, observe the Euler angles (Roll, Pitch, Yaw). They should be stable and not drifting.
            *   If they drift (especially yaw, after magnetometer cal), it points to issues in sensor calibration primarily.
            *   If they are noisy/jittery, try slightly *decreasing* `MADGWICK_BETA_STATIONARY`. This trusts the (hopefully stable) gyro integration more over potentially noisy accel/mag data when stationary.
            *   If slow to settle or has minor residual drift, a tiny increase might help, but prioritize calibration.
        *   **Motion Performance (`MADGWICK_BETA_MOTION`):**
            *   Perform various motions: slow rotations, quick snaps.
            *   If orientation (Euler angles) is sluggish or lags behind actual motion, try slightly *increasing* `MADGWICK_BETA_MOTION`.
            *   If orientation is too sensitive, overshoots, or seems to "wobble" excessively during/after motion (especially if roll/pitch are affected by linear acceleration), try *decreasing* `MADGWICK_BETA_MOTION`. This often indicates the filter is relying too heavily on accelerometer data which, during motion, includes non-gravity accelerations.
        *   `MADGWICK_BETA_INIT` influences how quickly the filter initially converges from its starting orientation (identity quaternion) to a stable solution. Usually, defaults are fine.

2.  **Tune Motion Detection Thresholds (in `icm_20948_functions.cpp`):**
    *   **Variables:**
        *   `GYRO_THRESHOLD` (current `0.03` rad/s)
        *   `ACCEL_VARIANCE_THRESHOLD` (current `0.005` g-force variance)
        *   `STATE_CHANGE_THRESHOLD` (current `3` samples for state change)
    *   **Action & Evaluation:**
        *   Observe the `isStationary` flag in the `enableICMRawDebug` output.
        *   It should correctly transition: `true` when still, `false` when moving.
        *   If it declares "motion" too easily (e.g., from minor vibrations when it should be stationary), slightly *increase* `GYRO_THRESHOLD` and/or `ACCEL_VARIANCE_THRESHOLD`.
        *   If it's too sluggish and stays "stationary" during gentle actual motion, slightly *decrease* these thresholds.
        *   `STATE_CHANGE_THRESHOLD` adds hysteresis. Increase for more "debouncing" of the stationary/motion state, decrease for faster transitions.

## Part 3: Unscented Kalman Filter (UKF) Tuning - Vertical State Estimation

The UKF (`ukf.cpp`) estimates vertical position, velocity, and acceleration using accelerometer data.

**CRITICAL PRE-REQUISITE: Implement Gravity Compensation for UKF Accelerometer Input**

*   **Problem:** The UKF currently uses body-frame Z-axis acceleration. Tilts will corrupt this with gravity components.
*   **Action:**
    1.  In `ProcessUKF()` (in `src/ukf.cpp`):
        *   Before calling `ukf_filter.processAccel(...)`, get the current orientation quaternion (`icm_q0, icm_q1, icm_q2, icm_q3`).
        *   For each Z-axis accelerometer input (calibrated `kx_accel_z_ms2` and `icm_accel_z_ms2`):
            *   Represent it as a vector in the body frame: e.g., `accel_body = [0, 0, sensor_accel_z]`.
            *   Rotate this `accel_body` vector to the world frame using the quaternion. You'll need a quaternion rotation function: `accel_world = rotate_vector_by_quaternion(accel_body, quaternion)`.
            *   The Z-component of `accel_world` is the vertical acceleration in the world frame.
            *   Subtract gravity from this Z-component: `linear_vertical_accel = accel_world_z - GRAVITY_CONSTANT` (e.g., `9.80665f` m/s^2).
        *   Pass these `linear_vertical_accel` values (one from KX134, one from ICM20948) to `ukf_filter.processAccel()`.
    2.  The `updateState` method in `UKF::updateState` will then use this `linear_vertical_accel`.
*   **Evaluation:**
    *   With the device stationary and tilted at various angles, the `linear_vertical_accel` fed to the UKF should be close to 0.0 m/s^2.
    *   The `ukf_acceleration_output` should also be near 0.0 m/s^2 when stationary, regardless of tilt.
    *   If `ukf_acceleration_output` still shows significant non-zero values when tilted and stationary, there's an issue in the gravity compensation or the AHRS orientation.

**Step-by-Step UKF Tuning (after gravity compensation):**

1.  **Tune Process Noise (`Q_` matrix in `UKF::UKF()` constructor in `ukf.cpp`):**
    *   **Variables:**
        *   `Q_[0][0]` (position process noise, current `0.01`)
        *   `Q_[1][1]` (velocity process noise, current `0.1`)
        *   `Q_[2][2]` (acceleration process noise, current `1.0`)
    *   **Action & Evaluation:**
        *   These values represent how much you trust your physics model. Higher Q = less trust in model, more in measurements.
        *   **Start with `Q_[2][2]` (acceleration):**
            *   If `ukf_acceleration_output` is too sluggish and doesn't track actual changes in vertical acceleration (e.g., during a simulated thrust event by quickly lifting the device), *increase* `Q_[2][2]`.
            *   If `ukf_acceleration_output` is very noisy and overreacts to sensor noise (even after calibration and gravity compensation), *decrease* `Q_[2][2]`.
        *   **Tune `Q_[1][1]` (velocity) and `Q_[0][0]` (position) next:**
            *   These are often smaller than `Q_[2][2]`.
            *   If integrated velocity or position drift excessively or are too noisy, adjust these. Generally, tune them after `Q_[2][2]` seems reasonable.
            *   Increase if outputs are sluggish, decrease if noisy.
        *   **Relative Magnitudes:** Expect `Q_accel > Q_vel > Q_pos`.
        *   **Evaluation Data:** Log `ukf_position_output`, `ukf_velocity_output`, `ukf_acceleration_output` alongside raw (gravity compensated) accelerometer data. Plot them. Does the UKF output follow the trend of the input smoothly?

2.  **Tune Measurement Noise (`R_` in `UKF::UKF()` constructor `ukf.cpp`):**
    *   **Variables:**
        *   The `updateState` averages two sensor inputs and uses `R_[0][0]` (current `0.5`) as the noise for this average.
    *   **Action & Evaluation:**
        *   `R` represents the variance of the (gravity-compensated, world-frame) vertical acceleration measurement you provide to the UKF.
        *   **Determine R empirically:**
            1.  While the device is stationary (after all calibrations and gravity compensation), collect a statistically significant number of `linear_vertical_accel` samples that would be fed to the UKF.
            2.  Calculate the variance of these samples. This variance is a good starting point for `R_[0][0]` (if you continue to average the two inputs before the UKF update).
        *   **Tuning:**
            *   If `R_[0][0]` is too small: The UKF trusts measurements too much, potentially making `ukf_acceleration_output` (and subsequently velocity/position) noisy if the measurements themselves have some residual noise.
            *   If `R_[0][0]` is too large: The UKF trusts measurements too little, leading to sluggish response and relying more on its (potentially drifting) model.
        *   This value interacts with `Q_`. If you increase `Q` (trust model less), you might need to ensure `R` accurately reflects measurement noise.

3.  **Review UKF Simplifications:**
    *   **Action:** Note the comments in `UKF::generateSigmaPoints` (re: Cholesky decomposition) and `UKF::updateState` (re: only updating acceleration part of state and P).
    *   **Recommendation:** For optimal performance, especially if drift persists, consider implementing a full Cholesky decomposition for sigma point generation and a full state/covariance update in `updateState`. This is a more advanced change.

## Part 4: Targeting a Direction (e.g., Straight Up)

This uses the PID controllers in `guidance_control.cpp` and AHRS output.

**Step-by-Step Configuration & Tuning:**

1.  **Set Target Orientation for "Straight Up":**
    *   **Action:**
        *   In your flight logic (e.g., in `update_guidance_targets` in `flight_logic.cpp`, or wherever guidance targets are set for active control phases):
            *   Call `guidance_set_target_orientation_euler(0.0f, 0.0f, initial_yaw_target_rad);`
                *   `target_roll_rad = 0.0f;` (for wings level)
                *   `target_pitch_rad = 0.0f;` (for nose vertical, assuming standard aircraft conventions where 0 pitch is level)
                *   `initial_yaw_target_rad` can be the yaw captured at launch if you just want to go straight up along the launch rail direction. If you need a specific compass heading, this would be your target absolute yaw.
    *   **Evaluation:**
        *   Log the target roll, pitch, and yaw angles alongside the current roll, pitch, yaw from the AHRS. Verify the targets are being set as expected.

2.  **Tune PID Controller Gains (in `config.h`):**
    *   **Variables:**
        *   `PID_ROLL_KP, KI, KD`
        *   `PID_PITCH_KP, KI, KD`
        *   `PID_YAW_KP, KI, KD`
        *   `PID_INTEGRAL_LIMIT_ROLL, _PITCH, _YAW`
        *   `PID_OUTPUT_MIN, _MAX` (ensure these map reasonably to your actuator capabilities)
    *   **Tuning Process (Iterative, for each axis - e.g., Pitch for vertical flight):**
        1.  **Safety First:** If testing with actuators, ensure the setup is safe and secure. Consider mechanical limits or reduced power initially.
        2.  **P-Term (`Kp`):**
            *   Set `Ki` and `Kd` to 0 for the axis being tuned.
            *   Start with a small `Kp`.
            *   Observe behavior: Manually tilt the rocket off the target pitch (0 degrees). Does the actuator try to correct?
            *   Gradually increase `Kp`. The response should become faster and stronger.
            *   If the system starts to oscillate around the setpoint (0 degrees pitch), `Kp` is too high. Reduce it by 30-50% from the value that caused oscillation.
            *   **Evaluation:** System should try to return to target pitch. It might be slow or have steady-state error.
        3.  **D-Term (`Kd`):**
            *   With `Kp` set, start with a small `Kd`.
            *   `Kd` adds damping. It should reduce overshoot and oscillations seen with `Kp` alone.
            *   Gradually increase `Kd`. Too much `Kd` can make the response sluggish or react excessively to noise in the angular rate.
            *   **Evaluation:** Oscillations should dampen. Response should be quicker to settle without much overshoot.
        4.  **I-Term (`Ki`):**
            *   With `Kp` and `Kd` set, start with a very small `Ki`.
            *   The I-term works to eliminate steady-state error (e.g., if the rocket consistently holds a pitch of 0.5 degrees instead of 0 due to an imbalance or aerodynamic effect).
            *   Gradually increase `Ki`. Too much `Ki` can cause slow oscillations or significant overshoot ("integral windup").
            *   Ensure `PID_INTEGRAL_LIMIT_PITCH` is set to a reasonable value (e.g., 0.3 to 0.7 if output is -1 to 1) to prevent excessive integral buildup.
            *   **Evaluation:** The rocket should now settle precisely at the target pitch (0 degrees) over time.
        5.  **Iterate:** Fine-tune P, I, and D. Adjusting one may require re-tuning others. Aim for a response that is quick, stable, with minimal overshoot, and no steady-state error.
        6.  **Axis Priority:** For "straight up," pitch and roll control are primary. Yaw control might be less critical unless a specific heading is required.
    *   **Data Logging for PID Tuning:**
        *   Log: `target_pitch_rad`, `current_pitch_rad` (from AHRS), `pid_pitch_state_g.integral`, `actuator_output_x_g` (pitch actuator command).
        *   Plot these values. This helps visualize errors, controller response, integral behavior, and actuator commands.

## Part 5: Action Items & Key Recommendations Summary

1.  **CRITICAL: Implement Persistent Magnetometer Calibration:**
    *   Save `magBias` and `magScale` to EEPROM after calibration via `ICM_20948_calibrate()`.
    *   Load these values from EEPROM in `ICM_20948_init()`.
2.  **Implement Accelerometer Calibration (ICM20948 & KX134):**
    *   6-point tumble calibration routine.
    *   Save bias/scale to EEPROM.
    *   Apply corrections to raw accelerometer data *before* it's used by AHRS or UKF.
3.  **Implement Initial Static Gyro Bias Calibration:**
    *   Capture average gyro readings when stationary at startup.
    *   Initialize `gyroBias` with these values (consider EEPROM storage).
4.  **CRITICAL: Implement Gravity Compensation for UKF Input:**
    *   In `ProcessUKF()`, transform body-frame Z-axis accelerations (calibrated) to world-frame.
    *   Subtract the gravity vector component from the world-frame vertical acceleration.
    *   Feed this *linear net vertical acceleration* to the UKF.
5.  **Systematically Tune Madgwick Parameters:** `beta` values and motion detection thresholds, using calibrated sensor data.
6.  **Systematically Tune UKF Parameters:** `Q` and `R` matrices, using calibrated and gravity-compensated accelerometer inputs.
7.  **Review UKF Implementation Details:**
    *   Consider implementing full Cholesky decomposition for sigma points.
    *   Consider implementing a full state and covariance update in `UKF::updateState` rather than the simplified version. (More advanced).
8.  **Configure Guidance for "Straight Up":** Set target roll and pitch to 0.0 radians.
9.  **Systematically Tune PID Controllers:** Kp, Ki, Kd for roll, pitch, and yaw axes, using extensive logging and iterative testing.
10. **Test Incrementally:** Calibrate sensors first, then AHRS, then UKF, then guidance. Verify each step.
11. **Leverage Debug Logs:** Use `enableSensorDebug`, `enableICMRawDebug`, and add more detailed logging for AHRS outputs, UKF states, and PID controller internal states during tuning. Plotting this data is invaluable.

By following these steps, you should be able to significantly reduce sensor drift, improve state estimation accuracy, and achieve more reliable flight control. 