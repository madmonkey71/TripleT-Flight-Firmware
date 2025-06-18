#ifndef MS5611_FUNCTIONS_H
#define MS5611_FUNCTIONS_H

#include <Arduino.h>
#include <MS5611.h>
#include "constants.h"  // For STANDARD_SEA_LEVEL_PRESSURE

// External variables
extern MS5611 ms5611Sensor;
extern float pressure;
extern float temperature;
extern float baro_altitude_offset;  // Calibration offset between GPS and barometric altitude
extern bool baroCalibrated;  // Flag to track if calibration has been performed
extern bool ms5611_initialized_ok; // Flag to track if the sensor was initialized successfully

// Function declarations
void ms5611_init();
int ms5611_read();
float ms5611_get_altitude(float seaLevelPressure = STANDARD_SEA_LEVEL_PRESSURE);
bool ms5611_calibrate_with_gps(uint32_t timeout_ms = 60000);  // New function for one-time calibration

#endif // MS5611_FUNCTIONS_H 