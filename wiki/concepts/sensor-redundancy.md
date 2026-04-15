---
title: Sensor Redundancy & IMU Interface
type: concept
tags: [sensors, imu, redundancy, failover, abstraction]
created: 2026-04-15
updated: 2026-04-15
related_files: [src/sensors/imu_interface.h, src/sensors/imu_manager.h, src/sensors/icm20948_sensor.h, src/sensors/kx134_sensor.h, src/sensors/sensor_factory.h]
---

All motion sensors implement a common `IMUInterface` (20 pure-virtual methods). `IMUManager` wraps two sensors (primary + backup) and provides automatic failover.

## IMUInterface

```cpp
// src/sensors/imu_interface.h
class IMUInterface {
public:
  virtual bool init() = 0;
  virtual bool read() = 0;
  virtual float getAccelX() = 0;  // m/s²
  virtual float getAccelY() = 0;
  virtual float getAccelZ() = 0;
  virtual float getGyroX() = 0;   // rad/s
  virtual float getGyroY() = 0;
  virtual float getGyroZ() = 0;
  virtual float getMagX() = 0;    // μT
  // ... (20 methods total including health/temp)
};
```

## Sensor Implementations

| Adapter | Sensor | Role | Range |
|---------|--------|------|-------|
| `ICM20948Sensor` | SparkFun ICM-20948 | Primary IMU | ±16G, 6-DoF + Mag |
| `KX134Sensor` | SparkFun KX134 | Backup / High-G | ±64G accel only |
| `BNO085Sensor` | BNO085 | Stub (future eval) | — |

Adapters call existing C driver functions (e.g., `ICM_20948_read()`) and expose them through the interface — no replacement of existing drivers.

## IMUManager Redundancy Logic

```cpp
// src/sensors/imu_manager.h
bool IMUManager::read() {
  if (primary_healthy && primary->read()) return true;
  if (backup_healthy && backup->read()) return true;
  return false;  // total sensor failure
}
```

Failover is transparent to `flight_logic.cpp` — it always calls `IMUManager::read()`.

## Compile-Time Sensor Selection

```ini
; platformio.ini build_flags:
-DUSE_BNO085_VARIANT       ; BNO085 as primary instead of ICM-20948
-DUSE_BONO85_BACKUP        ; BNO085 as backup instead of KX134
```

## KX134 Notes

- Returns identity quaternion (accel-only, no gyro)
- Acceptable: backup role is high-G accel readings only
- Switching threshold: when primary IMU reports > ~16G, KX134 data preferred

## Data Units Convention

All adapters normalize to SI units at the interface boundary:
- Acceleration: m/s² (adapter multiplies raw g values by 9.81)
- Angular rates: rad/s
- Magnetometer: μT
