#ifndef GPS_FUNCTIONS_H
#define GPS_FUNCTIONS_H

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// Function declarations
void gps_init();
void gps_read();
void gps_print();
bool checkGPSConnection();

#endif // GPS_FUNCTIONS_H 