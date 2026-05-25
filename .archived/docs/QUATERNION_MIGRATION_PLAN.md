# Quaternion Migration Plan

## 1. Problem Statement
The current GNC (Guidance, Navigation, and Control) system relies on Euler angles (Roll, Pitch, Yaw) for state estimation and integration in `kalman_filter.cpp`.
**Issues:**
*   **Gimbal Lock:** At +/- 90 degrees pitch, Roll and Yaw become indistinguishable.
*   **Singularities:** Math breakdowns at steep angles common in rocketry.
*   **Accuracy:** Integration of Euler rates is an approximation (`dRoll != gyro_x`).

## 2. Mathematical Changes

### 2.1. State Vector
Current: `[Roll, Pitch, Yaw]`
New: `[q0, q1, q2, q3]` (Quaternion)

### 2.2. Prediction Step (Integration)
Instead of adding `gyro * dt` to angles, we integrate the quaternion:
```cpp
// 0.5 * Omega * q
float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

q0 += qDot0 * dt;
q1 += qDot1 * dt;
...
normalizeQuaternion(q0, q1, q2, q3);
```

### 2.3. Update Step (Correction)
The Kalman correction step (`K * (z - Hx)`) needs to be updated.
*   **Accelerometer:** Measures gravity vector. We predict gravity in body frame using `q` and compare with measured accel.
*   **Magnetometer:** Measures magnetic field vector. Similar prediction and comparison.
*   *Note:* An Error-State Kalman Filter (ESKF) is standard for quaternions, tracking "error angles" rather than the quaternion directly in the covariance matrix to avoid constraint issues.

## 3. Code Impact Analysis

### 3.1. `kalman_filter.cpp`
*   **High Impact:** Needs complete rewrite of `kalman_predict` and `kalman_update`.
*   **API Change:** `kalman_get_orientation` should return Quaternions first, with a helper for Euler angles for legacy logging.

### 3.2. `guidance_control.cpp`
*   **Medium Impact:** PID controllers often work on Euler errors. We can either:
    1.  Convert Target Q and Current Q to Euler and use existing PIDs (easiest, still risks gimbal lock).
    2.  Implement Quaternion-based control (calculating error quaternion `q_err = q_target * q_current_inv`).

### 3.3. `TripleT_Flight_Firmware.cpp` & Logging
*   **Low Impact:** The `LogData` struct already supports quaternions (`q0`..`q3`). We just need to ensure `g_kalmanRoll` etc. are populated for the CSV log, or deprecated.

## 4. Implementation Steps

1.  **Preparation:**
    *   Implement `Quaternion` struct and math helpers (multiply, normalize, conjugate) in `utility_functions.h`.
    *   Write unit tests for these helpers.

2.  **Filter Rewrite:**
    *   Create `kalman_filter_quat.cpp` (parallel development).
    *   Implement the prediction step (gyro integration).
    *   Implement a simplified Gradient Descent (Madgwick-like) or ESKF update step. *Note: Full ESKF is complex; a Complementary Filter or Madgwick on Quaternions might be sufficient and simpler than a full Quaternion EKF.*

3.  **Integration:**
    *   Update `flight_logic.cpp` to call the new filter.
    *   Update `guidance_control.cpp` to accept quaternions.

4.  **Verification:**
    *   Compare `kalman_filter_quat` output vs `kalman_filter` (Euler) using recorded flight data in a unit test.

## 5. Recommendation
Start by implementing the **integration step** (Section 2.2) immediately to fix the rate-to-angle error, even if you map back to Euler angles for the update step temporarily. This provides the biggest accuracy win for the least effort.
