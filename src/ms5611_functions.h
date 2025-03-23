#ifndef MS5611_FUNCTIONS_H
#define MS5611_FUNCTIONS_H

#include <Arduino.h>
#include "MS5611.h"

extern MS5611 ms5611Sensor;

int ms5611_read();
void ms5611_init();
void ms5611_print();
float ms5611_get_altitude(float seaLevelPressure = 1013.25); // Default sea level pressure in hPa

#endif // MS5611_FUNCTIONS_H 