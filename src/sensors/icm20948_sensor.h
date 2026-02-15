#ifndef ICM20948_SENSOR_H
#define ICM20948_SENSOR_H

#include "imu_interface.h"
#include "../icm_20948_functions.h"

// ============================================================================
// ICM-20948 SENSOR ADAPTER
//
// Wraps the existing ICM-20948 functionality and provides IMUInterface
// implementation. This allows the ICM-20948 to be swapped with other sensors.
//
// Physical Range: ±16G (configurable via setAccelScale)
// ============================================================================

class ICM20948Sensor : public IMUInterface {
public:
  // Initialize the sensor using existing ICM_20948_init()
  bool begin() override {
    ICM_20948_init();
    return true;  // TODO: Add actual health check to ICM_20948_init
  }

  // Check sensor health
  bool isHealthy() override {
    // TODO: Implement actual health check - verify FIFO, data rates, etc
    return true;
  }

  const char* getErrorMessage() override {
    // TODO: Implement error tracking in ICM-20948 driver
    return "ICM-20948 OK";
  }

  // Read latest sensor data
  bool read() override {
    // Call existing driver read function
    // Note: This function updates global variables: icm_accel, icm_gyro, icm_mag, icm_temp
    ICM_20948_read();
    return true;  // TODO: Add actual error checking
  }

  // ========================================================================
  // ACCELERATION DATA - Convert from g to m/s²
  // ========================================================================

  float getAccelX() override {
    // Convert from g to m/s² (multiply by 9.81)
    return icm_accel[0] * 9.81f;
  }

  float getAccelY() override {
    return icm_accel[1] * 9.81f;
  }

  float getAccelZ() override {
    return icm_accel[2] * 9.81f;
  }

  float getAccelMagnitude() override {
    // Calculate magnitude: sqrt(x² + y² + z²) in m/s²
    float ax = icm_accel[0] * 9.81f;
    float ay = icm_accel[1] * 9.81f;
    float az = icm_accel[2] * 9.81f;
    return sqrt(ax*ax + ay*ay + az*az);
  }

  // ========================================================================
  // ROTATION DATA - Already in rad/s (convert to deg/s if needed)
  // ========================================================================

  float getGyroX() override {
    // Convert from rad/s to deg/s (multiply by 180/π ≈ 57.2958)
    return icm_gyro[0] * 57.2958f;
  }

  float getGyroY() override {
    return icm_gyro[1] * 57.2958f;
  }

  float getGyroZ() override {
    return icm_gyro[2] * 57.2958f;
  }

  // ========================================================================
  // MAGNETIC DATA - Already in microTesla
  // ========================================================================

  float getMagX() override {
    return icm_mag[0];
  }

  float getMagY() override {
    return icm_mag[1];
  }

  float getMagZ() override {
    return icm_mag[2];
  }

  // ========================================================================
  // ORIENTATION - Quaternion (w, x, y, z)
  // ========================================================================

  void getQuaternion(float& qw, float& qx, float& qy, float& qz) override {
    // Get quaternion from existing Madgwick filter
    qw = icm_q0;
    qx = icm_q1;
    qy = icm_q2;
    qz = icm_q3;
  }

  // ========================================================================
  // TEMPERATURE
  // ========================================================================

  float getTemperature() override {
    return icm_temp;
  }

  // ========================================================================
  // CONFIGURATION
  // ========================================================================

  void setAccelScale(uint16_t g_range) override {
    // TODO: Implement actual scale setting in ICM-20948 driver
    // Valid ranges: 2, 4, 8, 16 G
    // This requires updating the sensor configuration and recalibration
  }

  void setGyroScale(uint16_t dps_range) override {
    // TODO: Implement actual scale setting in ICM-20948 driver
    // Valid ranges: 250, 500, 1000, 2000 DPS
  }

  // ========================================================================
  // CALIBRATION
  // ========================================================================

  bool calibrate() override {
    // Use existing calibration function
    // ICM_20948_calibrate_gyro_bias(samples, delay_ms) is available
    ICM_20948_calibrate_gyro_bias(500, 10);  // 500 samples, 10ms apart
    return true;
  }
};

#endif // ICM20948_SENSOR_H
