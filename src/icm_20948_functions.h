#ifndef ICM_20948_FUNCTIONS_H
#define ICM_20948_FUNCTIONS_H

#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"

// External declarations for sensor data
extern float icm_accel[3];  // Accelerometer data (x, y, z)
extern float icm_gyro[3];   // Gyroscope data (x, y, z)
extern float icm_mag[3];    // Magnetometer data (x, y, z)
extern bool icm_data_available;
extern double icm_q1, icm_q2, icm_q3; // Quaternion data
extern uint16_t icm_data_header; // To store what data types are available

// Function declarations
void ICM_20948_init();
void ICM_20948_read();
void ICM_20948_print();
void printScaledAGMT(ICM_20948_I2C *sensor);
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);
void printPaddedInt16b(int16_t val);
void printRawAGMT(ICM_20948_AGMT_t agmt);

#endif // ICM_20948_FUNCTIONS_H 