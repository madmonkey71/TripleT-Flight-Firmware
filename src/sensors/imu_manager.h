#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include "imu_interface.h"

// ============================================================================
// IMU MANAGER - Redundancy and Automatic Sensor Failover
//
// Manages dual-sensor setup:
// - Primary: ICM-20948 (normal flight conditions, ±16G)
// - Backup:  KX134 (high-G events, ±64G) OR BNO085 (alternative sensor)
//
// Automatically switches to healthy sensor if primary fails.
// Returns data from whichever sensor is most healthy.
// ============================================================================

class IMUManager {
public:
  // Initialize both sensors
  bool begin(IMUInterface* primary, IMUInterface* backup) {
    primary_sensor = primary;
    backup_sensor = backup;

    bool primary_ok = false;
    bool backup_ok = false;

    if (primary_sensor) {
      primary_ok = primary_sensor->begin();
      primary_healthy = primary_ok;
    }

    if (backup_sensor) {
      backup_ok = backup_sensor->begin();
      backup_healthy = backup_ok;
    }

    return primary_ok || backup_ok;  // At least one must work
  }

  // Read from primary sensor, fallback to backup if primary fails
  bool read() {
    if (!primary_sensor && !backup_sensor) {
      return false;  // No sensors available
    }

    // Try primary first
    if (primary_sensor && primary_healthy) {
      bool primary_read_ok = primary_sensor->read();
      primary_healthy = primary_read_ok;

      if (primary_read_ok) {
        return true;  // Primary working, use it
      }

      // Primary failed, mark as unhealthy
      primary_healthy = false;
    }

    // Try backup
    if (backup_sensor && backup_healthy) {
      bool backup_read_ok = backup_sensor->read();
      backup_healthy = backup_read_ok;

      if (backup_read_ok) {
        return true;  // Fallback to backup working
      }

      // Backup also failed
      backup_healthy = false;
    }

    return false;  // Both sensors failed
  }

  // ========================================================================
  // DATA ACCESS - Returns from whichever sensor is healthy
  // ========================================================================

  float getAccelX() {
    if (primary_sensor && primary_healthy) {
      return primary_sensor->getAccelX();
    }
    if (backup_sensor && backup_healthy) {
      return backup_sensor->getAccelX();
    }
    return 0.0f;
  }

  float getAccelY() {
    if (primary_sensor && primary_healthy) {
      return primary_sensor->getAccelY();
    }
    if (backup_sensor && backup_healthy) {
      return backup_sensor->getAccelY();
    }
    return 0.0f;
  }

  float getAccelZ() {
    if (primary_sensor && primary_healthy) {
      return primary_sensor->getAccelZ();
    }
    if (backup_sensor && backup_healthy) {
      return backup_sensor->getAccelZ();
    }
    return 0.0f;
  }

  float getAccelMagnitude() {
    if (primary_sensor && primary_healthy) {
      return primary_sensor->getAccelMagnitude();
    }
    if (backup_sensor && backup_healthy) {
      return backup_sensor->getAccelMagnitude();
    }
    return 0.0f;
  }

  float getGyroX() {
    if (primary_sensor && primary_healthy) {
      return primary_sensor->getGyroX();
    }
    return 0.0f;  // Backup (KX134) doesn't have gyro
  }

  float getGyroY() {
    if (primary_sensor && primary_healthy) {
      return primary_sensor->getGyroY();
    }
    return 0.0f;
  }

  float getGyroZ() {
    if (primary_sensor && primary_healthy) {
      return primary_sensor->getGyroZ();
    }
    return 0.0f;
  }

  float getMagX() {
    if (primary_sensor && primary_healthy) {
      return primary_sensor->getMagX();
    }
    return 0.0f;
  }

  float getMagY() {
    if (primary_sensor && primary_healthy) {
      return primary_sensor->getMagY();
    }
    return 0.0f;
  }

  float getMagZ() {
    if (primary_sensor && primary_healthy) {
      return primary_sensor->getMagZ();
    }
    return 0.0f;
  }

  void getQuaternion(float& qw, float& qx, float& qy, float& qz) {
    if (primary_sensor && primary_healthy) {
      primary_sensor->getQuaternion(qw, qx, qy, qz);
      return;
    }
    // Backup sensor (KX134) returns identity quaternion if no gyro
    if (backup_sensor) {
      backup_sensor->getQuaternion(qw, qx, qy, qz);
      return;
    }
    // Default to identity quaternion
    qw = 1.0f;
    qx = 0.0f;
    qy = 0.0f;
    qz = 0.0f;
  }

  float getTemperature() {
    if (primary_sensor && primary_healthy) {
      return primary_sensor->getTemperature();
    }
    return 0.0f;  // Backup doesn't have temp sensor
  }

  // ========================================================================
  // HEALTH STATUS
  // ========================================================================

  bool isPrimaryHealthy() const {
    return primary_healthy;
  }

  bool isBackupHealthy() const {
    return backup_healthy;
  }

  bool isAnyHealthy() const {
    return primary_healthy || backup_healthy;
  }

  // Get which sensor is currently providing data
  const char* getActiveSensor() {
    if (primary_sensor && primary_healthy) {
      return "Primary (ICM-20948)";
    }
    if (backup_sensor && backup_healthy) {
      return "Backup (KX134/BNO085)";
    }
    return "No sensor active";
  }

  // Get error message from active sensor
  const char* getErrorMessage() {
    if (primary_sensor && primary_healthy) {
      return primary_sensor->getErrorMessage();
    }
    if (backup_sensor && backup_healthy) {
      return backup_sensor->getErrorMessage();
    }
    return "All sensors unhealthy";
  }

private:
  IMUInterface* primary_sensor = nullptr;
  IMUInterface* backup_sensor = nullptr;
  bool primary_healthy = false;
  bool backup_healthy = false;
};

#endif // IMU_MANAGER_H
