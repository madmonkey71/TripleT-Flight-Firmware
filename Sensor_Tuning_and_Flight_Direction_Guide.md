# Comprehensive Guide: Sensor Tuning, Drift Mitigation, and Flight Direction Control

This guide provides step-by-step instructions to calibrate sensors, tune the Attitude and Heading Reference System (AHRS), and configure the flight control system for targeted flight, such as straight up.

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

**Importance:** Reduces errors in attitude estimation (Madgwick).

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

**Current Method:** Static bias calibration (`gyroBias` determined by `ICM_20948_calibrate_gyro_bias()` at startup). Online estimation has been disabled.

**Step-by-Step Tuning & Evaluation:**

1.  **Ensure Effective Static Bias Calibration:**
    *   **Action:**
        *   The function `ICM_20948_calibrate_gyro_bias()` is called during `ICM_20948_calibrate()` at startup.
        *   Ensure the device is kept *perfectly still* during this phase of initialization.
        *   The default of 2000 samples over ~2 seconds is generally good. If issues persist, consider increasing samples, but stillness is key.
    *   **Evaluation:**
        *   Observe `gyroBias` values printed during `enableICMRawDebug` (if enabled). They should be non-zero and reflect the actual sensor bias at startup and remain constant.
        *   With the device stationary, the corrected gyro readings (`icm_gyro[i] - gyroBias[i]`) should be very close to zero.

2.  **Review `MADGWICK_GYRO_BIAS_LEARN_RATE` (in `config.h`):**
    *   **Variable:** `MADGWICK_GYRO_BIAS_LEARN_RATE` (current `0.002f`).
    *   **Action & Evaluation:**
        *   Since the problematic online gyro bias estimation in `ICM_20948_read()` (which used this learning rate) has been commented out, this constant is currently not directly used by that block of code.
        *   If a new, stable online gyro bias compensation is implemented in the future, this learning rate would be relevant again.
        *   For now, primary reliance is on the quality of the static calibration.

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
            *   With the device stationary, observe the Euler angles (Roll, Pitch, Yaw). They should be stable and not drifting significantly (some minor noise is expected).
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
        *   `ACCEL_VARIANCE_THRESHOLD` (current `0.008f` g-force variance - note: was 0.005g, check current value in file)
        *   `STATE_CHANGE_THRESHOLD` (current `5` samples for state change - note: was 3, check current value in file)
    *   **Action & Evaluation:**
        *   Observe the `isStationary` flag in the `enableICMRawDebug` output.
        *   It should correctly transition: `true` when still, `false` when moving.
        *   If it declares "motion" too easily (e.g., from minor vibrations when it should be stationary), slightly *increase* `GYRO_THRESHOLD` and/or `ACCEL_VARIANCE_THRESHOLD`.
        *   If it's too sluggish and stays "stationary" during gentle actual motion, slightly *decrease* these thresholds.
        *   `STATE_CHANGE_THRESHOLD` adds hysteresis. Increase for more "debouncing" of the stationary/motion state, decrease for faster transitions.

## Part 3: Targeting a Direction (e.g., Straight Up)

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

### Example: Attitude Hold Feature

The firmware includes an "Attitude Hold" feature that automatically engages during the `COAST` phase of flight. This feature captures the rocket's current roll, pitch, and yaw orientation at the moment of motor burnout (transition from `BOOST` to `COAST`) and sets these values as the target for the PID controllers. The system then attempts to maintain this captured orientation throughout the `COAST` phase.

This is a practical application of the `guidance_set_target_orientation_euler()` function and relies on well-tuned PID controllers (as described below) and accurate AHRS data (from Parts 1 and 2) to perform effectively.

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

## Part 4: Action Items & Key Recommendations Summary

1.  **CRITICAL: Implement Persistent Magnetometer Calibration:**
    *   Save `magBias` and `magScale` to EEPROM after calibration via `ICM_20948_calibrate()`.
    *   Load these values from EEPROM in `ICM_20948_init()`.
2.  **Implement Accelerometer Calibration (ICM20948 & KX134):**
    *   6-point tumble calibration routine.
    *   Save bias/scale to EEPROM.
    *   Apply corrections to raw accelerometer data *before* it's used by AHRS.
3.  **Ensure Effective Static Gyro Bias Calibration:**
    *   Confirm the device is perfectly still during `ICM_20948_calibrate_gyro_bias()` at startup.
    *   Verify that the `gyroBias` values are stable and correctly reflect the sensor's resting state.

(Note: The Unscented Kalman Filter (UKF) previously discussed for vertical state estimation has been deprecated and backed up as it was not actively used.) 