#ifndef UKF_H
#define UKF_H

/**
 * @file ukf.h
 * @brief Unscented Kalman Filter implementation for accelerometer data fusion
 * 
 * This file contains the UKF class which implements an Unscented Kalman Filter
 * to fuse acceleration measurements from KX134 and ICM20948 sensors.
 * The filter tracks position, velocity, and acceleration in the vertical axis.
 */

class UKF {
public:
  /**
   * Constructor
   * Initializes UKF parameters and matrices
   */
  UKF();
  
  /**
   * @brief Process acceleration measurements from both sensors
   * @param kx134_accel_z Vertical acceleration from KX134 sensor (m/s^2)
   * @param icm_accel_z Vertical acceleration from ICM20948 sensor (m/s^2)
   * @param dt Time step in seconds
   */
  void processAccel(float kx134_accel_z, float icm_accel_z, float dt);
  
  /**
   * @brief Initialize the filter state with given values
   * @param position Initial position (m), typically 0
   * @param velocity Initial velocity (m/s), typically 0
   * @param acceleration Initial acceleration (m/s^2), typically from sensors
   */
  void initialize(float position, float velocity, float acceleration);
  
  /**
   * @brief Get current estimated position
   * @return Position in meters
   */
  float getPosition() { return x_[0]; }
  
  /**
   * @brief Get current estimated velocity
   * @return Velocity in meters per second
   */
  float getVelocity() { return x_[1]; }
  
  /**
   * @brief Get current estimated acceleration
   * @return Acceleration in meters per second squared
   */
  float getAcceleration() { return x_[2]; }

private:
  // State vector [position, velocity, acceleration]
  float x_[3];
  
  // State covariance matrix
  float P_[3][3];
  
  // Process noise covariance
  float Q_[3][3];
  
  // Measurement noise covariance
  float R_[2][2];
  
  // Sigma points matrix (2*n_x+1, n_x)
  float Xsig_[7][3];
  
  // Weights for sigma points
  float weights_[11];
  
  // UKF parameters
  float alpha_;
  float beta_;
  float kappa_;
  float lambda_;
  
  // State dimensions
  int n_x_;   // Number of states
  int n_aug_; // Augmented state dimension
  
  // Initialization flag
  bool is_initialized_;
  
  // Timestamp for delta time calculation
  static unsigned long lastTimestamp_;
  
  /**
   * @brief Generate sigma points around current state estimate
   */
  void generateSigmaPoints();
  
  /**
   * @brief Predict state mean and covariance
   * @param dt Time step in seconds
   */
  void predictMeanAndCovariance(float dt);
  
  /**
   * @brief Update state with measurements
   * @param kx134_accel_z Vertical acceleration from KX134 sensor
   * @param icm_accel_z Vertical acceleration from ICM20948 sensor
   */
  void updateState(float kx134_accel_z, float icm_accel_z);
  
  /**
   * @brief Platform-independent implementation of time in milliseconds
   * @return Current time in milliseconds
   */
  static unsigned long getTimeMillis();
};

#endif /* UKF_H */ 