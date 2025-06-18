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
 * @brief Updates the state estimate using accelerometer data.
 *        Magnetometer data can be added later for yaw correction.
 * @param accel_x Acceleration along X-axis in m/s^2.
 * @param accel_y Acceleration along Y-axis in m/s^2.
 * @param accel_z Acceleration along Z-axis in m/s^2.
 */
void kalman_update_accel(float accel_x, float accel_y, float accel_z);

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
