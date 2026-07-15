---
title: Sensor Redundancy & IMU Interface
type: concept
tags: [sensors, imu, redundancy, failover, abstraction]
created: 2026-04-15
updated: 2026-07-02
related_files: [src/TripleT_Flight_Firmware.cpp, src/utility_functions.cpp, src/sensors/imu_interface.h, src/sensors/imu_manager.h, src/sensors/icm20948_sensor.h, src/sensors/kx134_sensor.h, src/sensors/sensor_factory.h]
---

Acceleration redundancy in the flight build is an **inline high-G fallback**, not a managed failover layer: the ICM-20948 (±16 g) is the primary source, and the KX134 (±64 g) takes over when the ICM saturates. An object-oriented sensor abstraction (`IMUInterface` / `IMUManager` / `SensorFactory`) exists under `src/sensors/` but is **dormant — never instantiated by the flight build**.

## Live Failover Mechanism

Two code paths provide the redundancy that actually runs:

- **Kalman accel source switch** (`src/TripleT_Flight_Firmware.cpp:929-955`): each IMU poll computes the ICM accel magnitude; if it exceeds 16 g (sensor saturation), the KX134 sample feeds `kalman_update_accel()` instead.
- **`get_accel_magnitude()`** (`src/utility_functions.cpp`): prefers the KX134 when available, falling back to the ICM — used by liftoff (`BOOST_ACCEL_THRESHOLD`), burnout, and landing detection.

Health gating is `isSensorSuiteHealthy()`: at least one working IMU is required for `ARMED` through `LANDED` — see [[concepts/system-robustness]].

## Dormant OO Abstraction (`src/sensors/`)

The following stack is scaffolding for a future managed-failover design. `g_imu_manager` / `initSensors()` are referenced only within `src/sensors/` itself; no flight code constructs any of these classes.

### IMUInterface

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

### Sensor Adapters

| Adapter | Sensor | Intended role | Range |
|---------|--------|---------------|-------|
| `ICM20948Sensor` | SparkFun ICM-20948 | Primary IMU | ±16G, 6-DoF + Mag |
| `KX134Sensor` | SparkFun KX134 | Backup / High-G | ±64G accel only |
| `BNO085Sensor` | BNO085 | Stub (future eval) | — |

Adapters wrap the existing C driver functions (e.g., `ICM_20948_read()`) — no replacement of the live drivers.

### IMUManager Failover Logic (not wired in)

```cpp
// src/sensors/imu_manager.h
bool IMUManager::read() {
  if (primary_healthy && primary->read()) return true;
  if (backup_healthy && backup->read()) return true;
  return false;  // total sensor failure
}
```

The design intent was failover transparent to `flight_logic.cpp`; as compiled, flight logic never calls `IMUManager` — it reads the driver globals directly.

### Compile-Time Sensor Selection (not wired in)

```ini
; intended platformio.ini build_flags:
-DUSE_BNO085_VARIANT       ; BNO085 as primary instead of ICM-20948
-DUSE_BONO85_BACKUP        ; BNO085 as backup instead of KX134 (note: misspelled in source)
```

No build environment defines either flag, and `BNO085Sensor` is a stub whose methods all return false/0.

## KX134 Notes

- Returns identity quaternion (accel-only, no gyro)
- Acceptable: backup role is high-G accel readings only
- Switching threshold: when the ICM reports > 16 g, KX134 data is used for the Kalman accel update

## Data Units Convention

The **live** driver globals use sensor-native units: acceleration in g, angular rates in rad/s, magnetometer in µT. The SI normalisation described below applies only to the dormant adapters:

- Acceleration: m/s² (adapter multiplies raw g values by 9.81)
- Angular rates: rad/s
- Magnetometer: μT
