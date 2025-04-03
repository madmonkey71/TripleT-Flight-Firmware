#ifndef GPS_FUNCTIONS_H
#define GPS_FUNCTIONS_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// External GPS object declaration
extern SFE_UBLOX_GNSS myGNSS;

// External GPS data variables
extern long lastTime;
extern byte GPS_fixType;
extern byte SIV;
extern long GPS_latitude;
extern long GPS_longitude;
extern long GPS_altitude;
extern long GPS_altitudeMSL;
extern long GPS_speed;
extern long GPS_heading;
extern int pDOP;
extern byte RTK;

// Function declarations
extern void gps_init();
extern void gps_read();
extern void gps_print();
extern bool checkGPSConnection();

#endif // GPS_FUNCTIONS_H 