// GPS Functions implementation for TripleT_Flight_Firmware
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "gps_config.h"
#include "gps_functions.h"

// Define the global variables declared as extern in the header
SFE_UBLOX_GNSS myGNSS;
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

// Add reference to debug flag
extern bool enableGPSDebug;

bool checkGPSConnection() {
  // Check if we can communicate with the GPS module
  Wire.begin();

  if (myGNSS.begin(Wire) == true) {
      // Successfully connected to GPS
    if (enableGPSDebug) Serial.println(F("GPS module connection verified"));
    byte rate = myGNSS.getNavigationFrequency();
    
    if (rate > 0) {
      // We got a response
      if (enableGPSDebug) {
        Serial.print(F("GPS communication success! Nav rate: "));
        Serial.println(rate);
      }
      return true;
    } else {
      if (enableGPSDebug) Serial.println(F("WARNING: No response from GPS when querying navigation rate."));
      return false;
    }
  } else {
      // Failed to connect to GPS
      if (enableGPSDebug) Serial.println(F("Could not connect to GPS module - check wiring"));
      return false;
  }
}

void gps_init() {
  if (enableGPSDebug) Serial.println(F("Initializing GPS module..."));
  
  // Try to connect to the GPS module - be explicit that we're using I2C
  if (myGNSS.begin(Wire) == false) {
    if (enableGPSDebug) Serial.println(F("u-blox GPS not detected at default I2C address. Please check wiring."));
    return;
  }

  // Enable debug messages only if GPS debug is enabled
  if (enableGPSDebug) {
    myGNSS.enableDebugging(Serial, true);
    
    // Get the protocol version
    Serial.print(F("Protocol version: "));
    Serial.println(myGNSS.getProtocolVersion(), HEX);
  } else {
    myGNSS.enableDebugging(Serial, false);
  }

  // Configure the GPS module
  bool pvtSuccess = myGNSS.setAutoPVT(true);
  if (!pvtSuccess && enableGPSDebug) {
    Serial.println(F("Failed to configure PVT"));
  }

  bool rateSuccess = myGNSS.setNavigationFrequency(5);
  if (!rateSuccess && enableGPSDebug) {
    Serial.println(F("Failed to set navigation frequency"));
  }

  // Set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.setI2COutput(COM_TYPE_UBX);

  // Enable the messages we want to receive
  myGNSS.enableMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_I2C, 1); // Position, velocity, time - 1Hz
  myGNSS.enableMessage(UBX_CLASS_NAV, UBX_NAV_SAT, COM_PORT_I2C, 5); // Satellite information - every 5 seconds
  myGNSS.enableMessage(UBX_CLASS_NAV, UBX_NAV_STATUS, COM_PORT_I2C, 1); // Receiver status - 1Hz
  myGNSS.enableMessage(UBX_CLASS_NAV, UBX_NAV_DOP, COM_PORT_I2C, 1); // Dilution of precision - 1Hz

  // Save the configuration
  bool saveSuccess = myGNSS.saveConfiguration();
  if (!saveSuccess && enableGPSDebug) {
    Serial.println(F("Failed to save configuration"));
  }

  if (enableGPSDebug) Serial.println(F("GPS module initialized successfully"));
}

void gps_read() {
  // Get the latest PVT data
  boolean pvtSuccess = myGNSS.getPVT();
  
  if (pvtSuccess) {
    // Get the fix type
    byte fixType = myGNSS.getFixType();
    GPS_fixType = fixType;

    // Get the number of satellites in view
    SIV = myGNSS.getSIV();

    // Get the position data
    GPS_latitude = myGNSS.getLatitude();
    GPS_longitude = myGNSS.getLongitude();
    GPS_altitude = myGNSS.getAltitude();
    GPS_altitudeMSL = myGNSS.getAltitudeMSL();

    // Get the velocity data
    GPS_speed = myGNSS.getGroundSpeed();
    GPS_heading = myGNSS.getHeading();

    // Get the DOP and RTK data
    pDOP = myGNSS.getPDOP();
    RTK = myGNSS.getCarrierSolutionType();

    // Update the last time we got data
    lastTime = millis();
  }
}

void gps_print() {
  // Only print if GPS debug is enabled
  if (!enableGPSDebug) return;
  
  // Print the date and time
  Serial.print(myGNSS.getYear()); Serial.print(F("-"));
  Serial.print(myGNSS.getMonth()); Serial.print(F("-"));
  Serial.print(myGNSS.getDay()); Serial.print(F(" "));
  Serial.print(myGNSS.getHour()); Serial.print(F(":"));
  Serial.print(myGNSS.getMinute()); Serial.print(F(":"));
  Serial.println(myGNSS.getSecond());

  // Print the fix type
  Serial.print(F("Fix type: "));
  switch (GPS_fixType) {
    case 0: Serial.print(F("No fix")); break;
    case 1: Serial.print(F("Dead reckoning")); break;
    case 2: Serial.print(F("2D")); break;
    case 3: Serial.print(F("3D")); break;
    case 4: Serial.print(F("GNSS + Dead reckoning")); break;
    case 5: Serial.print(F("Time only")); break;
    default: Serial.print(F("Unknown")); break;
  }

  // Print the number of satellites
  Serial.print(F(" | SIV: "));
  Serial.print(SIV);

  // Print the position
  Serial.print(F(" | Lat: "));
  Serial.print(GPS_latitude / 10000000.0, 7);
  Serial.print(F("° "));
  Serial.print(GPS_latitude < 0 ? F("S") : F("N"));
  
  Serial.print(F(" | Long: "));
  Serial.print(GPS_longitude / 10000000.0, 7);
  Serial.print(F("° "));
  Serial.print(GPS_longitude < 0 ? F("W") : F("E"));
  
  // Print the altitude
  Serial.print(F(" | Alt: "));
  Serial.print(GPS_altitude / 1000.0, 3);
  Serial.print(F(" m"));

  Serial.print(F(" | AltMSL: "));
  Serial.print(GPS_altitudeMSL / 1000.0, 3);
  Serial.print(F(" m"));

  // Print the velocity
  Serial.print(F(" | Speed: "));
  Serial.print(GPS_speed / 1000.0, 3);
  Serial.print(F(" m/s"));

  Serial.print(F(" | Heading: "));
  Serial.print(GPS_heading / 100000.0, 2);
  Serial.print(F("°"));

  // Print the DOP
  Serial.print(F(" | pDOP: "));
  Serial.print(pDOP / 100.0, 2);

  // Print the RTK status
  Serial.print(F(" | RTK: "));
  switch (RTK) {
    case 0: Serial.println(F("No RTK")); break;
    case 1: Serial.println(F("Float RTK")); break;
    case 2: Serial.println(F("Fixed RTK")); break;
    default: Serial.println(F("Unknown")); break;
  }
}

