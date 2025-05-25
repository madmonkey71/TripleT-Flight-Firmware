// GPS Functions implementation for TripleT_Flight_Firmware
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include "gps_config.h"
#include "gps_functions.h"

// Forward declarations - these are defined in the main file but used here
SFE_UBLOX_GNSS myGPS;
long lastTime;
byte GPS_fixType;
byte SIV;
long GPS_latitude;
long GPS_longitude;
long GPS_altitude;
long GPS_altitudeMSL;
long GPS_speed;
long GPS_heading;
int pDOP;
byte RTK;

void gps_init() {
  Serial.println(F("Initializing GPS module..."));
  
  // Try to connect to the GPS module - be explicit that we're using I2C
  Wire.begin();
  
  // Try multiple times to connect
  int attempts = 0;
  const int maxAttempts = 5;
  
  while (attempts < maxAttempts) {
    attempts++;
    
    Serial.print(F("Connection attempt ")); 
    Serial.print(attempts);
    Serial.print(F(" of ")); 
    Serial.println(maxAttempts);
    
    if (myGPS.begin(Wire) == true) {
      Serial.println(F("GPS module found!"));
      break;
    } else if (attempts < maxAttempts) {
      Serial.println(F("Connection failed. Retrying in 1 second..."));
      delay(1000);
    } else {
      Serial.println(F("GPS module not detected after multiple attempts. Check wiring."));
      return;
    }
  }
  
  // Enable debugging based on preprocessor settings
#if GPS_DEBUG_ENABLED
  #if GPS_DEBUG_VERBOSE
    myGPS.enableDebugging(Serial, true); // Enable debug messages with pretty printing
    Serial.println(F("GPS VERBOSE debugging enabled - all UBX messages will be shown on serial"));
  #else
    myGPS.enableDebugging(Serial, false); // Enable debug messages without pretty printing
    Serial.println(F("GPS debugging enabled - essential UBX messages will be shown on serial"));
  #endif
#endif
  
  // Configure the GPS module
  Serial.println(F("Configuring GPS module..."));
  
  // Print current configuration
#if GPS_DEBUG_ENABLED
  Serial.println(F("Current GPS Configuration:"));
  Serial.print(F("Protocol version: 0x"));
  Serial.println(myGPS.getProtocolVersion(), HEX);
#endif
  
  // Enable automatic message updates - critical for reliable data
  bool pvtSuccess = myGPS.setAutoPVT(true);
#if GPS_DEBUG_ENABLED
  Serial.print(F("Auto PVT enabled: "));
  Serial.println(pvtSuccess ? F("SUCCESS") : F("FAILED"));
#endif
  
  // Set navigation rate to 5Hz for more frequent updates
  bool rateSuccess = myGPS.setNavigationFrequency(5);
#if GPS_DEBUG_ENABLED
  Serial.print(F("Navigation frequency set to 5Hz: "));
  Serial.println(rateSuccess ? F("SUCCESS") : F("FAILED"));
#endif
  
  // Configure message output
#if GPS_DEBUG_ENABLED
  // Enable all messages for debugging purposes
  Serial.println(F("Enabling detailed message output..."));
  myGPS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); // Enable both UBX and NMEA messages for debugging
  
  // Enable specific UBX messages for detailed debugging
  myGPS.enableMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_I2C, 1); // Position, velocity, time - 1Hz
  myGPS.enableMessage(UBX_CLASS_NAV, UBX_NAV_SAT, COM_PORT_I2C, 5); // Satellite information - every 5 seconds
  myGPS.enableMessage(UBX_CLASS_NAV, UBX_NAV_STATUS, COM_PORT_I2C, 1); // Receiver status - 1Hz
  myGPS.enableMessage(UBX_CLASS_NAV, UBX_NAV_DOP, COM_PORT_I2C, 1); // Dilution of precision - The information in the current block is extremely detailed. Here is how this information can be used for a very detailed understanding of the sensor's data:
  
  Serial.println(F("Enhanced UBX messages enabled for debugging"));
#else
  // For normal operation, use only UBX protocol for efficiency
  myGPS.setI2COutput(COM_TYPE_UBX);
  
  // Just enable essential messages
  myGPS.enableMessage(UBX_CLASS_NAV, UBX_NAV_PVT, COM_PORT_I2C, 1);
#endif
  
  // Wait for a moment to allow settings to take effect
  delay(500);
  
  // Check satellite visibility and signal strength
  if (myGPS.getSIV() > 0) {
    Serial.print(F("Detected "));
    Serial.print(myGPS.getSIV());
    Serial.println(F(" satellites in view"));
    
#if GPS_DEBUG_ENABLED
    // Display satellite details
    Serial.println(F("Satellite details will be shown in debug output"));
#endif
  } else {
    Serial.println(F("No satellites detected yet. This is normal during initial startup."));
    Serial.println(F("A good view of the sky will improve fix acquisition time."));
  }
  
#if GPS_DEBUG_ENABLED
  // Query extended status information
  byte gpsFixOk = myGPS.getGnssFixOk();
  Serial.print(F("GPS Fix OK: "));
  Serial.println(gpsFixOk ? F("Yes") : F("No"));
#endif
  
  // Save settings to flash and battery-backed RAM
  bool saveSuccess = myGPS.saveConfiguration();
#if GPS_DEBUG_ENABLED
  Serial.print(F("GPS configuration saved: "));
  Serial.println(saveSuccess ? F("SUCCESS") : F("FAILED"));
  
  Serial.println(F("GPS module initialized and configured with enhanced debugging."));
#else
  Serial.println(F("GPS module initialized and configured."));
#endif
}

void gps_read() {
  // Query the GPS module only when due based on GPS_POLL_INTERVAL
  if (millis() - lastTime > 200)
  {
    lastTime = millis(); // Update the timer
    
#if GPS_DEBUG_ENABLED
    // Try to get a fresh PVT solution with detailed debug information
    Serial.println(F("\n========== GPS RAW DATA UPDATE =========="));
#endif
    
    boolean pvtSuccess = myGPS.getPVT();
    
#if GPS_DEBUG_ENABLED
    Serial.print(F("getPVT command result: ")); 
    Serial.println(pvtSuccess ? F("SUCCESS") : F("FAILED"));
    
    // Enhanced debug output
    Serial.print(F("GPS data available: "));
    Serial.println(pvtSuccess ? F("Yes") : F("No"));
#endif
    
    // Check for valid fix - only update variables with valid data
    byte fixType = myGPS.getFixType();

#if GPS_DEBUG_ENABLED
    Serial.print(F("Fix type: ")); 
    Serial.print(fixType);
    Serial.print(F(" ("));
    switch(fixType) {
      case 0: Serial.print(F("No fix")); break;
      case 1: Serial.print(F("Dead reckoning")); break;
      case 2: Serial.print(F("2D fix")); break;
      case 3: Serial.print(F("3D fix")); break;
      case 4: Serial.print(F("GNSS + Dead reckoning")); break;
      default: Serial.print(F("Unknown")); break;
    }
    Serial.println(F(")"));
    
    // Get more detailed status flags
    byte gpsFixOk = myGPS.getGnssFixOk();
    byte diffSoln = myGPS.getDiffSoln();
    byte carrSoln = myGPS.getCarrierSolutionType();
    Serial.print(F("GPS Fix OK: ")); Serial.println(gpsFixOk ? F("Yes") : F("No"));
    Serial.print(F("Differential corrections: ")); Serial.println(diffSoln ? F("Applied") : F("None"));
    Serial.print(F("RTK Solution: "));
    switch(carrSoln) {
      case 0: Serial.println(F("None")); break;
      case 1: Serial.println(F("Float")); break;
      case 2: Serial.println(F("Fixed")); break;
      default: Serial.println(carrSoln); break;
    }
#endif
    
    // Update global variables with the data
    GPS_fixType = fixType;
    SIV = myGPS.getSIV();
    
#if GPS_DEBUG_ENABLED
    Serial.print(F("Satellites: ")); Serial.println(SIV);
    
    // Display details about each satellite
    Serial.println(F("Satellite data will be shown in debug output"));
    
    // Get DOP values
    Serial.print(F("Position DOP: ")); Serial.println(myGPS.getPDOP() / 100.0, 2);
#endif
    
    // Only update position info if we have at least a 2D fix
    if (fixType >= 2) {
      // Scale factors from degrees to degrees * 10^-7 (standard format)
      // Report both the raw values and the scaled values for debugging
      long rawLat = myGPS.getLatitude();
      long rawLon = myGPS.getLongitude();
      long rawAlt = myGPS.getAltitude();
      long rawAltMSL = myGPS.getAltitudeMSL();
      long rawSpeed = myGPS.getGroundSpeed();
      long rawHeading = myGPS.getHeading();
      
      GPS_latitude = rawLat;
      GPS_longitude = rawLon;
      GPS_altitude = rawAlt;
      GPS_altitudeMSL = rawAltMSL;
      GPS_speed = rawSpeed;
      GPS_heading = rawHeading;
      pDOP = myGPS.getPDOP();
      RTK = myGPS.getCarrierSolutionType();
      
#if GPS_DEBUG_ENABLED
      // Detailed debug print of the results with both raw values and human-readable conversions
      Serial.println(F("Position data (raw and converted):"));
      Serial.print(F("Lat: ")); Serial.print(rawLat); 
      Serial.print(F(" raw (")); Serial.print(rawLat / 10000000.0, 6); Serial.println(F("°)"));
      
      Serial.print(F("Lon: ")); Serial.print(rawLon);
      Serial.print(F(" raw (")); Serial.print(rawLon / 10000000.0, 6); Serial.println(F("°)"));
      
      Serial.print(F("Alt: ")); Serial.print(rawAlt);
      Serial.print(F(" raw (")); Serial.print(rawAlt / 1000.0, 1); Serial.println(F(" m)"));
      
      Serial.print(F("AltMSL: ")); Serial.print(rawAltMSL);
      Serial.print(F(" raw (")); Serial.print(rawAltMSL / 1000.0, 1); Serial.println(F(" m)"));
      
      Serial.print(F("Speed: ")); Serial.print(rawSpeed);
      Serial.print(F(" raw (")); Serial.print(rawSpeed * 0.0036, 1); Serial.println(F(" km/h)"));
      
      Serial.print(F("Heading: ")); Serial.print(rawHeading);
      Serial.print(F(" raw (")); Serial.print(rawHeading / 100000.0, 1); Serial.println(F("°)"));
      
      // Additional time information
      Serial.println(F("Time information:"));
      Serial.print(F("UTC Date & Time: "));
      Serial.print(myGPS.getYear()); Serial.print(F("-"));
      Serial.print(myGPS.getMonth()); Serial.print(F("-"));
      Serial.print(myGPS.getDay()); Serial.print(F(" "));
      Serial.print(myGPS.getHour()); Serial.print(F(":"));
      Serial.print(myGPS.getMinute()); Serial.print(F(":"));
      Serial.println(myGPS.getSecond());
      
      Serial.print(F("Valid UTC time: ")); 
      Serial.println(myGPS.getTimeValid() ? F("Yes") : F("No"));
      Serial.print(F("Valid UTC date: ")); 
      Serial.println(myGPS.getDateValid() ? F("Yes") : F("No"));
#endif
    } 
#if GPS_DEBUG_ENABLED
    else {
      Serial.println(F("No valid position fix - position data unavailable"));
    }
    
    Serial.println(F("======================================="));
#endif
  }
}
void gps_print(){
    Serial.println(F("\n----- GPS Data -----"));
    
    // Show fix information first (most important for debugging)
    Serial.println(F("GPS Fix Type: "));
    Serial.print(F("  "));
    switch(GPS_fixType) {
      case 0: Serial.print(F("No Fix")); break;
      case 1: Serial.print(F("DR")); break;
      case 2: Serial.print(F("2D Fix")); break;
      case 3: Serial.print(F("3D Fix")); break;
      case 4: Serial.print(F("GNSS+DR")); break;
      default: Serial.print(GPS_fixType); break;
    }    
    Serial.print(F("Satellites: "));
    Serial.println(SIV);
    
    Serial.print(F("Positional DOP: "));
    Serial.println(pDOP / 100.0, 2);
    
    if (GPS_fixType >= 2) {
      Serial.print(F("  Position: "));
      Serial.print(GPS_latitude / 10000000.0, 6);
      Serial.print(F(", "));
      Serial.print(GPS_longitude / 10000000.0, 6);
      Serial.print(F(" | Altitude: "));
      Serial.print(GPS_altitude / 1000.0, 1);
      Serial.println(F("m"));
      
      Serial.print(F("AltitudeMSL: "));
      Serial.print(GPS_altitudeMSL);
      Serial.print(F(" mm ("));
      Serial.print(GPS_altitudeMSL / 1000.0, 1);
      Serial.println(F(" m)"));

      Serial.print(F("Heading: "));
      Serial.print(GPS_heading);
      Serial.print(F(" (degrees * 10^-5) = "));
      Serial.print(GPS_heading / 100000.0, 1);
      Serial.println(F(" degrees"));
    
      Serial.print(F("RTK: "));
      Serial.print(RTK);
      if (RTK == 1) Serial.println(F(" - Float RTK solution"));
      else if (RTK == 2) Serial.println(F(" - Fixed RTK solution"));
      else Serial.println(F(" - No RTK solution"));
  
      Serial.print(F("  Speed: "));
      Serial.print(GPS_speed * 0.0036, 1);
  }
    Serial.println(F("--------------------"));
}
