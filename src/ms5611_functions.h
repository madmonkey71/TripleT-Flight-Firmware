#ifndef MS5611_FUNCTIONS_H
#define MS5611_FUNCTIONS_H

#include <Arduino.h>
#include "MS5611.h"

// Standard sea level pressure in hPa (1013.25 hPa)
#define STANDARD_SEA_LEVEL_PRESSURE 1013.25

extern MS5611 ms5611Sensor;
extern float pressure;
extern float temperature;

int ms5611_read();
void ms5611_init();
void ms5611_print();
float ms5611_get_altitude(float seaLevelPressure = STANDARD_SEA_LEVEL_PRESSURE);

#endif // MS5611_FUNCTIONS_H 