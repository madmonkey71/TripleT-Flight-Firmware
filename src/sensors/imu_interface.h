#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include <Arduino.h>
#include <stdint.h>

// ============================================================================
// IMU INTERFACE - Abstract base class for all accelerometer/gyro sensors
//
// This interface allows interchangeable sensor implementations:
// - ICM-20948 (primary ±16G accelerometer)
// - KX134 (high-G ±64G accelerometer)
// - BNO085 (alternative with built-in sensor fusion)
//
// All sensors must implement this interface to be used in flight logic.
// ============================================================================

class IMUInterface {
public:
  virtual ~IMUInterface() = default;

  // ========================================================================
  // INITIALIZATION & HEALTH
  // ========================================================================

  // Initialize the IMU sensor (I2C/SPI, calibration, etc)
  // Returns true if initialization successful, false otherwise
  virtual bool begin() = 0;

  // Check if sensor is currently healthy and providing valid data
  // Called regularly to detect sensor failures
  virtual bool isHealthy() = 0;

  // Get descriptive error message if sensor is unhealthy
  // Example: "SPI communication timeout", "Invalid temperature reading", etc
  virtual const char* getErrorMessage() = 0;

  // ========================================================================
  // DATA READING
  // ========================================================================

  // Read latest sensor data (accel, gyro, temp, etc)
  // Returns true if read successful, false if sensor error
  virtual bool read() = 0;

  // ========================================================================
  // ACCELERATION DATA (in m/s²)
  // ========================================================================

  // Get X-axis acceleration (positive direction TBD by sensor orientation)
  virtual float getAccelX() = 0;

  // Get Y-axis acceleration
  virtual float getAccelY() = 0;

  // Get Z-axis acceleration
  virtual float getAccelZ() = 0;

  // Get combined acceleration magnitude
  // Calculated as: sqrt(accelX² + accelY² + accelZ²)
  // Used for launch detection and high-G events
  virtual float getAccelMagnitude() = 0;

  // ========================================================================
  // ROTATION DATA (in degrees per second)
  // ========================================================================

  // Get X-axis rotation rate (pitch)
  virtual float getGyroX() = 0;

  // Get Y-axis rotation rate (roll)
  virtual float getGyroY() = 0;

  // Get Z-axis rotation rate (yaw)
  virtual float getGyroZ() = 0;

  // ========================================================================
  // MAGNETIC DATA (in microTesla) - Optional, may return 0 if not available
  // ========================================================================

  // Get X-axis magnetic field
  virtual float getMagX() = 0;

  // Get Y-axis magnetic field
  virtual float getMagY() = 0;

  // Get Z-axis magnetic field
  virtual float getMagZ() = 0;

  // ========================================================================
  // ORIENTATION (PRIMARY METHOD - Used for Kalman filter)
  // ========================================================================

  // Get quaternion representation of current orientation
  // Quaternion format: (qw, qx, qy, qz) where w is scalar part
  // Must be normalized: sqrt(qw² + qx² + qy² + qz²) = 1
  //
  // This is the primary method for feeding orientation to the Kalman filter.
  // Some sensors (like BNO085) provide quaternion directly from hardware fusion.
  // Others (ICM-20948) require external gyro integration.
  virtual void getQuaternion(float& qw, float& qx, float& qy, float& qz) = 0;

  // ========================================================================
  // TEMPERATURE
  // ========================================================================

  // Get sensor die temperature (Celsius)
  // Used for temperature compensation in some sensors
  virtual float getTemperature() = 0;

  // ========================================================================
  // CONFIGURATION
  // ========================================================================

  // Set accelerometer full-scale range (in ±G)
  // Common values: 2, 4, 8, 16, 32, 64 depending on sensor
  // Not all sensors support all ranges
  virtual void setAccelScale(uint16_t g_range) = 0;

  // Set gyroscope full-scale range (in ±DPS - degrees per second)
  // Common values: 250, 500, 1000, 2000 depending on sensor
  virtual void setGyroScale(uint16_t dps_range) = 0;

  // ========================================================================
  // CALIBRATION (Optional - may be no-op for some sensors)
  // ========================================================================

  // Perform sensor-specific calibration routine
  // For example: ICM-20948 gyro offset calibration
  // Returns true if calibration successful
  virtual bool calibrate() = 0;
};

#endif // IMU_INTERFACE_H
