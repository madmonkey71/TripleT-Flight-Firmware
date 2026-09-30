#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <Arduino.h> // For millis(), fabs, sqrt, etc.
#include <math.h>    // For trigonometric functions like atan2, sin, cos

// Kalman filter function declarations

/**
 * @brief Initializes the Kalman filter with initial orientation estimates.
 * @param initial_roll The initial roll angle in radians.
 * @param initial_pitch The initial pitch angle in radians.
 * @param initial_yaw The initial yaw angle in radians.
 */
void kalman_init(float initial_roll, float initial_pitch, float initial_yaw);

/**
 * @brief Predicts the next state based on gyroscope data.
 * @param gyro_x Angular velocity around X-axis in radians/second.
 * @param gyro_y Angular velocity around Y-axis in radians/second.
 * @param gyro_z Angular velocity around Z-axis in radians/second.
 * @param dt Time step in seconds since the last prediction.
 */
void kalman_predict(float gyro_x, float gyro_y, float gyro_z, float dt);

/**
 * @brief Updates roll/pitch from the accelerometer, treating it as a gravity (tilt) reference.
 *
 * The update is GATED (audit #9): it is applied only when the specific-force magnitude is within
 * [KALMAN_ACCEL_GATE_LOW_G, KALMAN_ACCEL_GATE_HIGH_G] and the gyro rate seen by the last
 * kalman_predict() is below KALMAN_ACCEL_GATE_MAX_GYRO_RPS. Otherwise it is skipped, the
 * error covariance keeps growing, and the gyro integration carries the estimate. Non-finite
 * input is rejected.
 *
 * @param accel_x,accel_y,accel_z Specific force in g (only the direction and the magnitude
 *        relative to 1 g matter).
 * @return true if the measurement was applied, false if it was skipped.
 */
bool kalman_update_accel(float accel_x, float accel_y, float accel_z);

/**
 * @brief Updates the state estimate using magnetometer data.
 * @param mag_x Magnetometer data along X-axis.
 * @param mag_y Magnetometer data along Y-axis.
 * @param mag_z Magnetometer data along Z-axis.
 */
void kalman_update_mag(float mag_x, float mag_y, float mag_z);

/**
 * @brief Retrieves the current orientation estimates from the Kalman filter.
 * @param roll Output parameter for the estimated roll angle in radians.
 * @param pitch Output parameter for the estimated pitch angle in radians.
 * @param yaw Output parameter for the estimated yaw angle in radians.
 */
void kalman_get_orientation(float &roll, float &pitch, float &yaw);

/** @brief Error variance of axis 0=roll, 1=pitch, 2=yaw (diagnostics / tests). */
float kalman_get_variance(int axis);

/** @brief Number of accelerometer updates skipped by the gate since kalman_init(). */
unsigned long kalman_accel_updates_skipped();

/*
Internal state variables for the filter will be defined in kalman_filter.cpp.
These would typically include:
- State vector (e.g., [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z])
- Covariance matrix P
- Process noise covariance matrix Q
- Measurement noise covariance matrix R
For a C-style implementation, these are static variables in the .cpp file.
A more advanced implementation might use a struct or class to encapsulate these.
*/

#endif // KALMAN_FILTER_H
