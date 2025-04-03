#ifndef MS5611_FUNCTIONS_H
#define MS5611_FUNCTIONS_H

#include <Arduino.h>
#include "MS5611.h"

// Standard sea level pressure in hPa (1013.25 hPa)
#define STANDARD_SEA_LEVEL_PRESSURE 1013.25

extern MS5611 ms5611Sensor;
extern float pressure;
extern float temperature;
extern float baro_altitude_offset;  // Calibration offset between GPS and barometric altitude
extern bool baro_calibration_done;  // Flag to track if calibration has been performed

int ms5611_read();
void ms5611_init();
void ms5611_print();
float ms5611_get_altitude(float seaLevelPressure = STANDARD_SEA_LEVEL_PRESSURE);
bool ms5611_calibrate_with_gps(uint32_t timeout_ms = 60000);  // New function for one-time calibration

#endif // MS5611_FUNCTIONS_H 