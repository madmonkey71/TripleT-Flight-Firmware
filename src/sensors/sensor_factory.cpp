#include "sensor_factory.h"

// ============================================================================
// GLOBAL SENSOR INSTANCES
// ============================================================================

IMUManager*    g_imu_manager   = nullptr;
IMUInterface*  g_primary_sensor = nullptr;
IMUInterface*  g_backup_sensor  = nullptr;

// Note: initSensors() is defined inline in sensor_factory.h
// This ensures it's only defined once when the header is included
