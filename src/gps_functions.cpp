// GPS Functions implementation for TripleT_Flight_Firmware
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "gps_config.h"
#include "gps_functions.h"

// Define the global variables declared as extern in the header
SFE_UBLOX_GNSS myGNSS; // Use default constructor
long lastTime = 0;
byte GPS_fixType = 0;
byte SIV = 0;
long GPS_latitude = 0;
long GPS_longitude = 0;
long GPS_altitude = 0;
long GPS_altitudeMSL = 0;
long GPS_speed = 0;
long GPS_heading = 0;
int pDOP = 0;
byte RTK = 0;

// Make cached accuracy values directly accessible global variables
uint16_t GPS_hAcc = 0;
uint16_t GPS_vAcc = 0;
unsigned long last_accuracy_update = 0;
#define GPS_ACCURACY_UPDATE_INTERVAL 30000 // Only update accuracy every 30 seconds

// GPS time variables with default values for Jan 1, 2000
int GPS_year = 2000;
byte GPS_month = 1;
byte GPS_day = 1;
byte GPS_hour = 0;
byte GPS_minute = 0;
byte GPS_second = 0;
bool GPS_time_valid = false;

// Add reference to debug flag
extern volatile bool enableGPSDebug;

// Create a single persistent NullStream that doesn't output anything
class NullStream : public Stream {
public:
  int available() { return 0; }
  int read() { return -1; }
  int peek() { return -1; }
  void flush() {}
  size_t write(uint8_t) { return 1; }
};

static NullStream nullStream;

// Single function to control GPS debugging state
void setGPSDebugging(bool enable) {
  // Set multiple times to ensure it takes effect
  if (enable) {
    myGNSS.enableDebugging(Serial, true);
  } else {
    // Call multiple times with delays to ensure it's disabled
    myGNSS.enableDebugging(nullStream, false);
    delay(10);
    myGNSS.enableDebugging(nullStream, false);
    delay(10);
    myGNSS.enableDebugging(nullStream, false);
  }
}

void gps_init() {
  Serial.println(F("Initializing GPS..."));
  
  // Initialize GPS time variables with default values
  GPS_year = 2000;
  GPS_month = 1;
  GPS_day = 1;
  GPS_hour = 0;
  GPS_minute = 0;
  GPS_second = 0;
  GPS_time_valid = false;
  
  // Initialize accuracy values
  GPS_hAcc = 0;
  GPS_vAcc = 0;
  last_accuracy_update = 0;
  
  // Initialize GPS connection
  Wire.begin();
  
  // Simple message - debug status will be shown by the setGPSDebugging function
  Serial.println(F("GPS init..."));
  
  // Ensure debugging is set to the desired state before any GPS operations
  setGPSDebugging(enableGPSDebug);
  
  // Direct initialization without checking connection
  myGNSS.begin(Wire);
  
  // Configure the GPS module
  Serial.print(F("Configuring GPS..."));
  
  // Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setI2COutput(COM_TYPE_UBX);
  
  // Configure in a single block
  bool configSuccess = true;
  configSuccess &= myGNSS.setAutoPVT(true);
  configSuccess &= myGNSS.setNavigationFrequency(5);
  configSuccess &= myGNSS.enableMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_I2C, 1);
  configSuccess &= myGNSS.enableMessage(UBX_CLASS_NAV, UBX_NAV_SAT, COM_PORT_I2C, 5);
  configSuccess &= myGNSS.enableMessage(UBX_CLASS_NAV, UBX_NAV_STATUS, COM_PORT_I2C, 1);
  configSuccess &= myGNSS.enableMessage(UBX_CLASS_NAV, UBX_NAV_DOP, COM_PORT_I2C, 1);
  
  Serial.println(configSuccess ? F("OK") : F("failed"));
  
  // Ensure debugging is still in the proper state after initialization
  setGPSDebugging(enableGPSDebug);
}

// Original checkGPSConnection function was redundant - replaced with simpler version
bool checkGPSConnection() {
  // Ensure debug settings are consistent
  setGPSDebugging(enableGPSDebug);
  
  // Just verify we can get data from the GPS
  byte rate = myGNSS.getNavigationFrequency();
  return (rate > 0);
}

bool gps_read() {
  // TIMING DEBUG - Start
  unsigned long gps_start_time = millis();
  
  // Update GPS data if available
  bool pvt_result = myGNSS.getPVT();
  
  if (pvt_result) {
    // Get basic positioning data
    GPS_latitude = myGNSS.getLatitude();
    GPS_longitude = myGNSS.getLongitude();
    GPS_altitude = myGNSS.getAltitude();       // Get altitude above ellipsoid
    GPS_altitudeMSL = myGNSS.getAltitudeMSL(); // Get altitude above mean sea level
    GPS_speed = myGNSS.getGroundSpeed();
    GPS_heading = myGNSS.getHeading();         // Get heading
    GPS_fixType = myGNSS.getFixType();
    SIV = myGNSS.getSIV();
    pDOP = myGNSS.getPDOP();                   // Get position dilution of precision
    RTK = myGNSS.getCarrierSolutionType();     // Get RTK solution status
    
    // Update accuracy values periodically (every 30 seconds)
    // We are only updating these values every 30 seconds as it is a very slow operation (2+ seconds)
    if (GPS_fixType >= 2 && (millis() - last_accuracy_update > GPS_ACCURACY_UPDATE_INTERVAL)) {
      // Get accuracy values
      GPS_hAcc = myGNSS.getHorizontalAccuracy();
      GPS_vAcc = myGNSS.getVerticalAccuracy();
    }
    
    // Update GPS time variables if we have a valid fix
    if (GPS_fixType > 0) {
      GPS_year = myGNSS.getYear();
      GPS_month = myGNSS.getMonth();
      GPS_day = myGNSS.getDay();
      GPS_hour = myGNSS.getHour();
      GPS_minute = myGNSS.getMinute();
      GPS_second = myGNSS.getSecond();
      GPS_time_valid = true;
    }
    // Debug print if enabled
    if (enableGPSDebug) {
      gps_print();
    }
    return true;
  }
  
  // TIMING DEBUG - Not available case
  unsigned long gps_end_time = millis();
  if (gps_end_time - gps_start_time > 100) {
    Serial.print(F("SLOW GPS-nodata: "));
    Serial.print(gps_end_time - gps_start_time);
    Serial.println(F("ms"));
  }
  
  return false;
}

// Functions to get the cached accuracy values
uint16_t getGPSHorizontalAccuracy() {
  return GPS_hAcc;
}

uint16_t getGPSVerticalAccuracy() {
  return GPS_vAcc;
}

void gps_print() {
  Serial.println("GPS Data:");
  Serial.print("  Fix type: ");
  Serial.print(GPS_fixType);
  Serial.print(" | Satellites: ");
  Serial.println(SIV);
  
  Serial.print("  Lat: ");
  Serial.print(GPS_latitude / 10000000.0, 7);
  Serial.print(" | Lon: ");
  Serial.print(GPS_longitude / 10000000.0, 7);
  Serial.print(" | Alt: ");
  Serial.print(GPS_altitude / 1000.0);
  Serial.println(" m");
  
  Serial.print("  Alt MSL: ");
  Serial.print(GPS_altitudeMSL / 1000.0);
  Serial.print(" m | Heading: ");
  Serial.print(GPS_heading / 100000.0, 2);
  Serial.println(" deg");
  
  Serial.print("  Speed: ");
  Serial.print(GPS_speed / 1000.0);
  Serial.print(" m/s | pDOP: ");
  Serial.print(pDOP / 100.0, 2);
  
  Serial.print(" | RTK: ");
  switch(RTK) {
    case 0: Serial.println("None"); break;
    case 1: Serial.println("Float"); break;
    case 2: Serial.println("Fixed"); break;
    default: Serial.println(RTK); break;
  }
  
  Serial.print("  Accuracy: H: ");
  Serial.print(GPS_hAcc / 1000.0);
  Serial.print(" m, V: ");
  Serial.print(GPS_vAcc / 1000.0);
  Serial.println(" m");
  
  if (GPS_time_valid) {
    Serial.print("  Time: ");
    Serial.print(GPS_year);
    Serial.print("-");
    if (GPS_month < 10) Serial.print("0");
    Serial.print(GPS_month);
    Serial.print("-");
    if (GPS_day < 10) Serial.print("0");
    Serial.print(GPS_day);
    Serial.print(" ");
    if (GPS_hour < 10) Serial.print("0");
    Serial.print(GPS_hour);
    Serial.print(":");
    if (GPS_minute < 10) Serial.print("0");
    Serial.print(GPS_minute);
    Serial.print(":");
    if (GPS_second < 10) Serial.print("0");
    Serial.println(GPS_second);
  } else {
    Serial.println("  Time: Not yet valid");
  }
  
  Serial.println();
}

// Function to get GPS date/time safely (uses defaults if no valid time)
void getGPSDateTime(int& year, byte& month, byte& day, byte& hour, byte& minute, byte& second) {
  // Return the current GPS time values (defaults to Jan 1, 2000 if no valid fix)
  year = GPS_year;
  month = GPS_month;
  day = GPS_day;
  hour = GPS_hour;
  minute = GPS_minute;
  second = GPS_second;
}

