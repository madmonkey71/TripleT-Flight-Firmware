#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include <Arduino.h>
#include "utility_functions.h"
#include "icm_20948_functions.h"

// Kalman filter states for attitude estimation
extern float kf_roll, kf_pitch, kf_yaw;       // Filtered attitude angles in radians
extern float kf_q0, kf_q1, kf_q2, kf_q3;      // Filtered quaternion values

// Function declarations
void initSensorFusion();
void updateSensorFusion();
void printSensorFusionData();

// Convert quaternion to Euler angles (roll, pitch, yaw)
void quaternionToEuler(float q0, float q1, float q2, float q3, float& roll, float& pitch, float& yaw);

// Convert Euler angles to quaternion
void eulerToQuaternion(float roll, float pitch, float yaw, float& q0, float& q1, float& q2, float& q3);

#endif // SENSOR_FUSION_H 