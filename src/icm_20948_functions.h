#ifndef ICM_20948_FUNCTIONS_H
#define ICM_20948_FUNCTIONS_H

#include <Arduino.h>
#include "utility_functions.h" // Include this to get variable declarations
#include "ICM_20948.h"

// Sensor status variables
extern bool isStationary;
extern float sampleFreq;  // Sample frequency in Hz
extern uint32_t lastUpdateTime; // Time of last sensor update

// Function declarations
void ICM_20948_init();
void ICM_20948_read();
void ICM_20948_print();
void ICM_20948_calibrate();
bool ICM_20948_isReady();


// Global variable declarations
extern ICM_20948_I2C myICM;

// External variables for ICM-20948 data
extern float icm_accel[3];  // Accelerometer data (g)
extern float icm_gyro[3];   // Gyroscope data (deg/s) 
extern float icm_mag[3];    // Magnetometer data (uT)
extern float icm_temp;      // Temperature in degrees C
extern bool icm_data_available;  // Flag to indicate if data is ready
extern bool icm20948_ready;      // Flag to indicate if ICM is ready
extern uint16_t icm_data_header; // FIFO header for checking packet types

#endif // ICM_20948_FUNCTIONS_H 