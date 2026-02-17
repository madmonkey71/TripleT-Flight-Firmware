#ifndef SENSOR_FACTORY_H
#define SENSOR_FACTORY_H

#include "imu_interface.h"
#include "icm20948_sensor.h"
#include "kx134_sensor.h"
#include "bno085_sensor.h"
#include "imu_manager.h"

// ============================================================================
// SENSOR FACTORY - Creates sensor instances with compile-time selection
//
// Use compile-time configuration in platformio.ini or config.h to select
// which sensors to use:
// - Default: ICM-20948 (primary) + KX134 (high-G backup)
// - Alternative: BNO085 (if evaluating different hardware)
// ============================================================================

class SensorFactory {
public:
  // Create primary IMU sensor (normal flight conditions)
  // Default: ICM-20948 (±16G)
  static IMUInterface* createPrimarySensor() {
#ifdef USE_BNO085_VARIANT
    // Alternative: BNO085 instead of ICM-20948
    return new BNO085Sensor();
#else
    // Default: ICM-20948
    return new ICM20948Sensor();
#endif
  }

  // Create backup IMU sensor (high-G or alternative)
  // Default: KX134 (±64G high-G accelerometer)
  // Can be replaced with BNO085 for evaluation
  static IMUInterface* createBackupSensor() {
#ifdef USE_BONO85_BACKUP
    // Use BNO085 as backup instead of KX134
    return new BNO085Sensor();
#else
    // Default: KX134 high-G accelerometer
    return new KX134Sensor();
#endif
  }

  // Create the dual-sensor IMU manager for automatic failover
  // Uses static allocation to prevent heap fragmentation on embedded systems
  static IMUManager* createIMUManager() {
    static IMUManager manager;
    static IMUInterface* primary = createPrimarySensor();
    static IMUInterface* backup = createBackupSensor();
    manager.begin(primary, backup);
    return &manager;
  }

  // Create just primary sensor (if you don't want redundancy)
  static IMUInterface* createPrimarySensorOnly() {
    IMUInterface* sensor = createPrimarySensor();
    if (sensor) {
      sensor->begin();
    }
    return sensor;
  }
};

// ============================================================================
// GLOBAL SENSOR INSTANCES (Initialize in main setup)
// ============================================================================

// Global pointer to the IMU manager (main interface for flight logic)
extern IMUManager* g_imu_manager;

// Optional: Individual sensor pointers if needed
extern IMUInterface* g_primary_sensor;
extern IMUInterface* g_backup_sensor;

// ============================================================================
// INITIALIZATION FUNCTION (Call from main setup())
// ============================================================================

inline void initSensors() {
  g_imu_manager = SensorFactory::createIMUManager();
  // Individual sensors are created inside the manager
  // but pointers can be retrieved if needed for diagnostics
}

#endif // SENSOR_FACTORY_H
