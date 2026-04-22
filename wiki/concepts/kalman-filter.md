---
title: Kalman Filter — AHRS Orientation Estimation
type: concept
tags: [kalman, ahrs, orientation, sensor-fusion]
created: 2026-04-15
updated: 2026-04-15
related_files: [src/kalman_filter.cpp, src/kalman_filter.h]
---

Custom Kalman filter that fuses gyroscope and accelerometer data to estimate orientation as quaternion + Euler angles. Replaced the deprecated Madgwick complementary filter.

## State Vector

6-element state: `[roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]`

The bias terms allow the filter to estimate and correct systematic gyro drift over time.

## API

```cpp
// Initialize with known starting orientation
void kalman_init(float roll_rad, float pitch_rad, float yaw_rad);

// Gyro prediction step — call each IMU read cycle
void kalman_predict(float gx, float gy, float gz, float dt_sec);

// Accelerometer correction — call when accel data available
void kalman_update_accel(float ax, float ay, float az);  // m/s²

// Magnetometer correction — call when mag data available
void kalman_update_mag(float mx, float my, float mz);

// Read current estimate
void kalman_get_orientation(float& roll, float& pitch, float& yaw);  // radians
```

## Covariance Matrices

- **P** — State covariance (uncertainty), updated each step
- **Q** — Process noise (model uncertainty, tunable)
- **R** — Measurement noise (sensor uncertainty, tunable)

Tuning Q and R is the key calibration step. High Q trusts measurement more; high R trusts model prediction more.

## Data Flow

```
ICM-20948 gyro → kalman_predict()  →┐
ICM-20948 accel → kalman_update_accel() →┤→ [roll, pitch, yaw, biases]
ICM-20948 mag → kalman_update_mag()  →┘         ↓
                                        guidance_control.cpp
                                        LogData.q0/q1/q2/q3
                                        LogData.euler_roll/pitch/yaw
```

## Output in LogData

```cpp
float q0, q1, q2, q3;              // Orientation quaternion (w,x,y,z)
float euler_roll, euler_pitch, euler_yaw;  // Radians
float gyro_bias_x, gyro_bias_y, gyro_bias_z;  // Estimated biases
```
