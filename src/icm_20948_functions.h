#ifndef ICM_20948_FUNCTIONS_H
#define ICM_20948_FUNCTIONS_H

#include <Arduino.h>
#include "utility_functions.h" // Include this to get variable declarations
#include "ICM_20948.h"

// Sensor status variables
// extern bool isStationary; // This was a previous attempt, definition is in .cpp
extern float sampleFreq;  // Sample frequency in Hz
extern uint32_t lastUpdateTime; // Time of last sensor update

// Function declarations
void ICM_20948_init();
void ICM_20948_read();
void ICM_20948_print();
// void ICM_20948_calibrate(); // REMOVED as unused
// void ICM_20948_get_calibrated_gyro(float out_gyro[3]); // REMOVED as unused
void ICM_20948_calibrate_gyro_bias(int num_samples, int delay_ms); // Static gyro bias calibration
bool icm_20948_get_mag(float* mag);
// void icm_20948_get_gyro(float* gyro); // REMOVED as unused
void icm_20948_get_accel(float* accel); // Restoring

// Functions for magnetometer calibration persistence
bool icm_20948_save_calibration();
bool icm_20948_load_calibration();
void ICM_20948_calibrate_mag_interactive();

// Global variable declarations
extern ICM_20948_I2C myICM;

// External variables for ICM-20948 data
extern float icm_accel[3];  // Accelerometer data (g)
extern float icm_gyro[3];   // Gyroscope data (raw, in rad/s from Madgwick update)
extern float icm_mag[3];    // Magnetometer data (uT)
extern float icm_temp;      // Temperature in degrees C
extern bool icm_data_available;  // Flag to indicate if data is ready
extern uint16_t icm_data_header; // FIFO header for checking packet types
extern bool isStationary;       // Defined in .cpp, used by main firmware

// Madgwick filter outputs
extern float icm_q0, icm_q1, icm_q2, icm_q3;  // Quaternion components (w,x,y,z)
extern float gyroBias[3];                   // Estimated gyroscope bias (rad/s)

// Helper function to convert quaternion to Euler angles is in utility_functions.h
// void convertQuaternionToEuler(float q0, float q1, float q2, float q3, float &roll, float &pitch, float &yaw);

#endif // ICM_20948_FUNCTIONS_H