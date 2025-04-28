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

// Add direct access to cached GPS accuracy values
extern uint16_t GPS_hAcc;
extern uint16_t GPS_vAcc;

// GPS time variables with default values for Jan 1, 2000
extern int GPS_year;
extern byte GPS_month;
extern byte GPS_day;
extern byte GPS_hour;
extern byte GPS_minute;
extern byte GPS_second;
extern bool GPS_time_valid;

// Function declarations
extern void gps_init();
extern bool gps_read();
extern void gps_print();
extern bool checkGPSConnection();

// Function to get GPS date/time safely (uses defaults if no valid time)
extern void getGPSDateTime(int& year, byte& month, byte& day, byte& hour, byte& minute, byte& second);

// New functions to get cached GPS accuracy values
extern uint16_t getGPSHorizontalAccuracy();
extern uint16_t getGPSVerticalAccuracy();

// New function to control GPS debugging
void setGPSDebugging(bool enable);

#endif // GPS_FUNCTIONS_H 