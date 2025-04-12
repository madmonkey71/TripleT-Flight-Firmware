#ifndef ICM_20948_FUNCTIONS_H
#define ICM_20948_FUNCTIONS_H

#include <Arduino.h>
#include "utility_functions.h" // Include this to get variable declarations
#include "ICM_20948.h"

// Sensor status variables (not declared in utility_functions.h)
extern bool isStationary;
extern float sampleFreq;  // Sample frequency in Hz
extern uint32_t lastUpdateTime; // Time of last sensor update

// Magnetometer calibration
extern float magBias[3];  // Hard iron calibration
extern float magScale[3]; // Soft iron calibration

// Function declarations
void ICM_20948_init();
void ICM_20948_read();
void ICM_20948_print();
void ICM_20948_calibrate();
void detectStationary();
void calculateOrientation();
void normalizeQuaternion(float &q0, float &q1, float &q2, float &q3);
void applyMagnetometerCalibration();

// Global variable declarations
extern ICM_20948_I2C myICM;

#endif // ICM_20948_FUNCTIONS_H 