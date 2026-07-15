---
title: Kalman Filter — AHRS Orientation Estimation
type: concept
tags: [kalman, ahrs, orientation, sensor-fusion]
created: 2026-04-15
updated: 2026-07-02
related_files: [src/kalman_filter.cpp, src/kalman_filter.h, src/ukf.cpp, .archived/docs/QUATERNION_MIGRATION_PLAN.md]
---

Custom Kalman filter that fuses gyroscope, accelerometer, and magnetometer data to estimate orientation. Outputs both a quaternion and Euler angles (the quaternion is converted from Euler at log time). Replaced the deprecated Madgwick complementary filter. Runs at ~10 Hz (the IMU poll cadence). This filter is **attitude-only** — barometer and GPS are not fused, and no altitude/velocity estimator runs in the live build.

**Current representation**: Euler angles (with quaternion also output for logging). Attitude estimates degrade near ±90° pitch (gimbal lock). Avoid sustained pitch excursions above ±80° until the quaternion migration (see below) ships.

## State Vector

3 decoupled scalar states: `[roll, pitch, yaw]`, each with its own scalar covariance (diagonal-only `P_diag[3]`).

There are **no gyro-bias states**: gyro bias is handled upstream as a fixed calibration constant captured by the `calibrate_gyro` command and subtracted from the raw gyro before the predict step — the filter does not estimate drift online.

## API

```cpp
// Initialize with known starting orientation
void kalman_init(float roll_rad, float pitch_rad, float yaw_rad);

// Gyro prediction step — call each IMU read cycle
void kalman_predict(float gx, float gy, float gz, float dt_sec);

// Accelerometer correction — call when accel data available
void kalman_update_accel(float ax, float ay, float az);  // live code passes g; atan2 ratios make units cancel

// Magnetometer correction — call when mag data available
// (tilt-compensated heading; yaw innovation is wrap-normalized at ±π)
void kalman_update_mag(float mx, float my, float mz);

// Read current estimate
void kalman_get_orientation(float& roll, float& pitch, float& yaw);  // radians
```

## Covariance & Noise Parameters

- **P** — Per-axis scalar covariance (`P_diag[3]`, initial 1.0), updated each step
- **Q** — Process noise, **hard-coded** `Q_angle = 0.001` in `kalman_filter.cpp`
- **R** — Measurement noise, **hard-coded** `R_accel = 0.03`, `R_mag = 0.03`

There are no config hooks yet — tuning Q and R means editing `kalman_filter.cpp` and rebuilding. High Q trusts measurement more; high R trusts model prediction more.

## Data Flow

```
ICM-20948 gyro (bias-corrected) → kalman_predict()  →┐
ICM/KX134 accel → kalman_update_accel() →┤→ [roll, pitch, yaw]
ICM-20948 mag → kalman_update_mag()  →┘         ↓
                                        guidance_control.cpp
                                        LogData.q0/q1/q2/q3
                                        LogData.euler_roll/pitch/yaw
```

(The accel source switches to the KX134 when the ICM magnitude exceeds 16 g.)

## Output in LogData

```cpp
float q0, q1, q2, q3;              // Orientation quaternion (w,x,y,z)
float euler_roll, euler_pitch, euler_yaw;  // Radians
float gyro_bias_x, gyro_bias_y, gyro_bias_z;  // Calibration constants from calibrate_gyro (not filter states)
```

## Planned: Quaternion Migration

`.archived/docs/QUATERNION_MIGRATION_PLAN.md` outlines replacing the Euler-angle predict/update with a direct quaternion-state filter (q0..q3). Motivation:

- Eliminate gimbal lock at ±90° pitch (high-angle guided flight becomes safe).
- More accurate rate-to-angle integration.
- Simpler linearisation around the current quaternion.

`LogData` already carries q0..q3, so the logging schema is unaffected. A separate `src/ukf.cpp` exists but is not the production filter — treat as experimental. Migration is tracked in [[queries/roadmap-2026]].

## Related

- [[concepts/sensor-redundancy]] — where the sensor data comes from
- [[concepts/calibration]] — gyro bias / mag cal prerequisites
- [[entities/guidance-control]] — consumer of the orientation estimate
- [[concepts/data-logging]] — LogData fields populated by the filter
