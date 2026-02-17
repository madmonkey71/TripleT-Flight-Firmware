#ifndef KX134_SENSOR_H
#define KX134_SENSOR_H

#include "imu_interface.h"
#include "../kx134_functions.h"

// ============================================================================
// KX134 SENSOR ADAPTER (High-G Accelerometer)
//
// Wraps the KX134 high-G accelerometer and provides IMUInterface.
// Used as backup when ICM-20948 saturates at high accelerations.
//
// Physical Range: ±64G (much higher than ICM-20948's ±16G)
// Note: This sensor only provides acceleration, not gyro or mag data.
// ============================================================================

class KX134Sensor : public IMUInterface {
public:
  bool begin() override {
    return kx134_init();
  }

  bool isHealthy() override {
    // TODO: Implement actual health check for KX134
    return true;
  }

  const char* getErrorMessage() override {
    return "KX134 OK";
  }

  bool read() override {
    kx134_read();
    return true;  // TODO: Add actual error checking
  }

  // ========================================================================
  // ACCELERATION DATA - Convert from g to m/s²
  // ========================================================================

  float getAccelX() override {
    return kx134_accel[0] * 9.81f;
  }

  float getAccelY() override {
    return kx134_accel[1] * 9.81f;
  }

  float getAccelZ() override {
    return kx134_accel[2] * 9.81f;
  }

  float getAccelMagnitude() override {
    float ax = kx134_accel[0] * 9.81f;
    float ay = kx134_accel[1] * 9.81f;
    float az = kx134_accel[2] * 9.81f;
    return sqrt(ax*ax + ay*ay + az*az);
  }

  // ========================================================================
  // ROTATION DATA - Not available on KX134 (return 0)
  // ========================================================================

  float getGyroX() override { return 0.0f; }
  float getGyroY() override { return 0.0f; }
  float getGyroZ() override { return 0.0f; }

  // ========================================================================
  // MAGNETIC DATA - Not available on KX134 (return 0)
  // ========================================================================

  float getMagX() override { return 0.0f; }
  float getMagY() override { return 0.0f; }
  float getMagZ() override { return 0.0f; }

  // ========================================================================
  // ORIENTATION - Not available on KX134
  // Return identity quaternion (no rotation)
  // ========================================================================

  void getQuaternion(float& qw, float& qx, float& qy, float& qz) override {
    // Identity quaternion (no rotation) - KX134 has no orientation data
    qw = 1.0f;
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;
  }

  // ========================================================================
  // TEMPERATURE - Not available on KX134
  // ========================================================================

  float getTemperature() override {
    return 0.0f;  // Not available
  }

  // ========================================================================
  // CONFIGURATION
  // ========================================================================

  void setAccelScale(uint16_t g_range) override {
    // KX134 supports ±8G and ±64G ranges
    // TODO: Implement actual scale setting
  }

  void setGyroScale(uint16_t dps_range) override {
    // KX134 has no gyro, no-op
  }

  // ========================================================================
  // CALIBRATION
  // ========================================================================

  bool calibrate() override {
    // KX134 doesn't require calibration (static accelerometer)
    return true;
  }
};

#endif // KX134_SENSOR_H
