// TripleT Flight Firmware
// Current Version: v0.48
// Current State: Beta
// Last Updated: 18/06/2025
// **Notes**
// This code started out life as a remake of the Blip Test Code from Joe Barnard @ BPS.Space
// Nothing remains of the original code but that's where the concept originated.
// His code taught me a lot and you should go support his content !!
// I also learned bits from @LabRatMatt from YouTube and instructables
// It has been heavily modifed from the original as I've chosen the Teensy 4.0 and different sensors.

// Moved Commentary to the README.md file

// 22:04:03.459 -> Device found at address 0x1F  (unknown chip) assumed to be the GPS
// 22:04:03.459 -> Device found at address 0x42  (PCA9685) maybe the KX134 ?
// 22:04:03.459 -> Device found at address 0x69  (MPU6050,MPU9050,MPU9250,ITG3701,L3G4200D)
// 22:04:03.459 -> Device found at address 0x77  (BMP085,BMA180,BMP280,MS5611)

// Include all the needed libraries.
#include <Arduino.h>
#include <Wire.h>

// WS2812 LED Library
#include <Adafruit_NeoPixel.h>
// An SPI library for later functions
#include <SPI.h>
// Load the libraries for the SDCard
#include <SdFat.h>
#include <sdios.h>
#include <SerialFlash.h>
// Library for controlling PWM Servo's
#include <PWMServo.h>
// Set the version number
#define TRIPLET_FLIGHT_VERSION 0.48

// Define the board type - Teensy 4.1 only
#ifndef BOARD_TEENSY41
#if defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41)
#define BOARD_TEENSY41
#endif
#endif

// Now include the GPS and sensor functions
#include "constants.h"        // Include constants first to avoid macro conflicts
#include "gps_config.h"
#include "gps_functions.h"  // Include GPS functions header
#include "ms5611_functions.h"
#include "utility_functions.h"    // Include utility functions header
#include "data_structures.h"  // Include our data structures
#include "icm_20948_functions.h"  // Include ICM-20948 functions
#include "kx134_functions.h"  // Include KX134 functions
#include "log_format_definition.h" // For LOG_COLUMNS and LOG_COLUMN_COUNT
#include "guidance_control.h" // For guidance and control functions
#include "flight_logic.h"     // For update_guidance_targets()
#include "config.h"          // For pin definitions and other config
#include "state_management.h" // For recoverFromPowerLoss()
#include "kalman_filter.h"   // For Kalman filter functions
#include "sensor_fusion.h"   // For sensor fusion logic

// Define variables declared as extern in utility_functions.h
String FileDateString = "";
String LogDataString = "";
unsigned long currentTime = 0;  // For timestamp
bool baroCalibrated = false;  // For barometric calibration status
float launchAltitude = 0.0f;
float maxAltitudeReached = 0.0f;
float currentAltitude = 0.0f;
bool kx134_initialized_ok = false;
bool icm20948_ready = false;

// Global variable for dynamic main deployment altitude (Task Step 2)
float g_main_deploy_altitude_m_agl = 0.0f;

const char* BOARD_NAME = "Teensy 4.1";

// Orientation Filter Selection
bool useMadgwickFilter = !KALMAN_FILTER_ACTIVE_BY_DEFAULT;
bool useKalmanFilter = KALMAN_FILTER_ACTIVE_BY_DEFAULT;
float kalmanRoll = 0.0f;
float kalmanPitch = 0.0f;
float kalmanYaw = 0.0f;
bool usingKX134ForKalman = false; // Initialize to false, default to ICM for Kalman

// External declarations for sensor data
extern SFE_UBLOX_GNSS myGNSS;  // GPS object
extern float pressure;
extern float temperature;
// extern float kx134_accel[3];

// Define sensor objects
SparkFun_KX134 kx134Accel;  // Add KX134 accelerometer object definition

// Servo objects for actuators
#include <PWMServo.h> // Ensure it's included (already there)
PWMServo servo_pitch;
PWMServo servo_roll;
PWMServo servo_yaw;

// Storage configuration
// #define SD_CARD_MIN_FREE_SPACE 1024 * 1024  // 1MB minimum free space - REMOVED, defined in config.h
#define EXTERNAL_FLASH_MIN_FREE_SPACE 1024 * 1024  // 1MB minimum free space

// Storage variables
SdFat SD;
bool sdCardAvailable = false;
bool sdCardPresent = false;   // Physical presence of SD card
bool sdCardMounted = false;   // Whether SD card is mounted
bool loggingEnabled = true;  // Whether logging is enabled

// Flight State Variables
FlightState currentFlightState = STARTUP;
FlightState previousFlightState = STARTUP;
unsigned long stateEntryTime = 0; // To store the time when the current state was entered

uint64_t availableSpace = 0;  // Available space on SD card in bytes
bool flashAvailable = false;
FsFile root;
FsFile entry;
FsFile LogDataFile;
FsFile IdleDataFile;

// Global storage variables
uint64_t totalSpace;
uint64_t usedSpace;

// Sequence number for logging
static unsigned long logSequenceNumber = 0;

// Additional global variables for data logging
const int FLASH_CHIP_SELECT = 5; // Choose an appropriate pin for flash CS
char logFileName[32] = ""; // To store the current log file name

// Guidance Control Update Interval
const unsigned long GUIDANCE_UPDATE_INTERVAL_MS = 20; // 50Hz control loop

// Debug control variables
bool enableDetailedOutput = false;
bool enableSystemDebug = false;      // For system messages
// enableSensorDebug is declared as volatile below
bool enableIMUDebug = false;         // For IMU data
// enableGPSDebug is declared as volatile below
bool enableBaroDebug = false;        // For barometer data
bool enableStorageDebug = false;     // For SD card and storage operations
bool enableStatusSummary = false;    // Flag to control status summary output
bool enableICMRawDebug = true;     // Flag to control ICM raw data debug output (ON by default for tuning)
bool displayMode = false;           // Toggle for compact vs detailed display
bool enableSerialCSV = false;  // Global variable to control serial CSV output

// Debug control
volatile bool detailedOutput = false;   // Add detailed data to be printed to console
volatile bool enableSensorDebug = true; // Enable detailed sensor-related debug output
volatile bool enableGPSDebug = false;   // Disable GPS library debug output completely

// Function to check available storage space
void checkStorageSpace() {
  if (sdCardAvailable && SD.vol()) {
    uint64_t freeSpace = SD.vol()->freeClusterCount() * SD.vol()->bytesPerCluster();
    availableSpace = freeSpace; // Update global variable
    
    if (freeSpace < SD_CARD_MIN_FREE_SPACE) {
      Serial.println(F("WARNING: SD card space low!"));
      Serial.print(F("Available space: "));
      Serial.print(freeSpace / (1024ULL * 1024ULL));
      Serial.println(F(" MB"));
    }
  }
}

// Function to create a new log file with timestamp in name
bool createNewLogFile() {
  // Make sure the SD card is available
  if (!sdCardAvailable) {
    Serial.println(F("SD card not available, cannot create log file"));
      return false;
    }
    
  // Close any previously open log file
  if (LogDataFile) {
    LogDataFile.flush();
    LogDataFile.close();
  }
  
  // Create a unique filename based on timestamp
  char fileName[64];
  
  // If GPS has fix, use date/time for filename
  if (myGNSS.getFixType() > 0) {
    // Get date and time safely (will use Jan 1, 2000 as default if no valid time)
    int year;
    byte month, day, hour, minute, second;
    getGPSDateTime(year, month, day, hour, minute, second);
    
    sprintf(fileName, "DATA_%04d%02d%02d_%02d%02d%02d.csv", 
      year, month, day, hour, minute, second);
  } else {
    // No GPS fix, use millis()
    sprintf(fileName, "LOG_%lu.csv", millis());
  }
  
  Serial.print(F("Creating log file: "));
  Serial.println(fileName);
  
  // Open the file
  if (!LogDataFile.open(fileName, O_RDWR | O_CREAT | O_EXCL)) {
    Serial.println(F("Error: Failed to create new log file. SD card might be full, unformatted, or corrupted. Please check the SD card."));
    loggingEnabled = false; // Ensure logging is marked as disabled
    return false;
  }
  
  // Write CSV header by iterating through LOG_COLUMNS
  // This ensures the header is always synchronized with the LogData struct and logDataToString()
  char headerBuffer[768]; // Generous buffer for the header string
  int currentOffset = 0;
  for (size_t i = 0; i < LOG_COLUMN_COUNT; ++i) {
    int written = snprintf(headerBuffer + currentOffset, sizeof(headerBuffer) - currentOffset,
                             "%s%s", LOG_COLUMNS[i].name, (i < LOG_COLUMN_COUNT - 1) ? "," : "");
    if (written > 0) {
      currentOffset += written;
    } else {
      // Handle snprintf error, though unlikely with sufficient buffer
      Serial.println(F("Error writing header buffer"));
      break;
    }
    if (currentOffset >= (int)(sizeof(headerBuffer) -1)) { // -1 for null terminator, cast to int
        Serial.println(F("Header buffer overflow!"));
        break;
    }
  }
  LogDataFile.println(headerBuffer);
  
  // Flush to ensure header is written
  LogDataFile.flush();
  
  // Copy filename to global variable
  strcpy(logFileName, fileName);
  
  loggingEnabled = true;
  Serial.println(F("Log file created successfully"));
  
  return true;
}

// Function to safely close all files
void closeAllFiles() {
  if (LogDataFile) {
    Serial.println(F("Closing log file"));
    LogDataFile.flush(); // Ensure data is written
    LogDataFile.close();
    
    // Reset file object
    LogDataFile = FsFile();
  }
}

// Function to handle system shutdown
void prepareForShutdown() {
  Serial.println(F("Preparing for shutdown..."));
  
  // Close all open files
  closeAllFiles();
  
  // Set LED to indicate shutdown
  pixels.setPixelColor(0, pixels.Color(0, 0, 50)); // Blue during shutdown
  pixels.show();
  
  // Final beep
  tone(BUZZER_PIN, 1000); delay(100); noTone(BUZZER_PIN);
  
  Serial.println(F("System shutdown complete"));
  while (true) {
    delay(5000);
  }
}

void WriteLogData(bool forceLog) {
  static unsigned long lastLogTime = 0;
  
  // Only log at specified intervals or when forced
  if (!forceLog && millis() - lastLogTime < 200) {
    return;
  }
  lastLogTime = millis();
  
  // --- Populate LogData Struct --- 
  LogData logEntry;
  logEntry.seqNum = logSequenceNumber++; // Increment and assign sequence number
  logEntry.timestamp = millis(); // Use current millis() for timestamp
  logEntry.flightState = static_cast<uint8_t>(currentFlightState); // Add current flight state
  logEntry.fixType = GPS_fixType;
  logEntry.sats = SIV;
  logEntry.latitude = GPS_latitude;
  logEntry.longitude = GPS_longitude;
  logEntry.altitude = GPS_altitude;
  logEntry.altitudeMSL = GPS_altitudeMSL;
  // Calculate and add raw and calibrated altitude
  if (pressure > 0) { // Ensure pressure is valid
    logEntry.raw_altitude = 44330.0 * (1.0 - pow(pressure / STANDARD_SEA_LEVEL_PRESSURE, 0.1903));
    if (baroCalibrated) {
      logEntry.calibrated_altitude = logEntry.raw_altitude + baro_altitude_offset;
    } else {
      logEntry.calibrated_altitude = logEntry.raw_altitude; // If not calibrated, log raw altitude as calibrated altitude
    }
  } else {
    logEntry.raw_altitude = 0.0f; // Log 0.0 if pressure is not positive
    logEntry.calibrated_altitude = 0.0f; // Log 0.0 if pressure is not positive
  }
  // --------------------------------
  logEntry.speed = GPS_speed;
  logEntry.heading = GPS_heading;
  logEntry.pDOP = pDOP;
  logEntry.rtk = RTK;
  logEntry.pressure = pressure;
  logEntry.temperature = temperature;
  memcpy(logEntry.kx134_accel, kx134_accel, sizeof(kx134_accel)); // Copy KX134 array
  memcpy(logEntry.icm_accel, icm_accel, sizeof(icm_accel));      // Copy ICM accel array
  memcpy(logEntry.icm_gyro, icm_gyro, sizeof(icm_gyro));        // Copy ICM gyro array
  memcpy(logEntry.icm_mag, icm_mag, sizeof(icm_mag));          // Copy ICM mag array
  logEntry.icm_temp = icm_temp;

  // Populate AHRS Data
  logEntry.q0 = icm_q0;
  logEntry.q1 = icm_q1;
  logEntry.q2 = icm_q2;
  logEntry.q3 = icm_q3;

  if (useKalmanFilter) {
      logEntry.euler_roll = kalmanRoll;   // Assuming kalmanRoll, Pitch, Yaw are in radians
      logEntry.euler_pitch = kalmanPitch;
      logEntry.euler_yaw = kalmanYaw;
  } else { // Default to Madgwick if Kalman is not active
      convertQuaternionToEuler(icm_q0, icm_q1, icm_q2, icm_q3,
                               logEntry.euler_roll, logEntry.euler_pitch, logEntry.euler_yaw);
  }

  logEntry.gyro_bias_x = gyroBias[0]; // Keep logging gyro bias from Madgwick/ICM
  logEntry.gyro_bias_y = gyroBias[1];
  logEntry.gyro_bias_z = gyroBias[2];

  // Populate Guidance Control Data
  // Target Euler angles and PID integrals are static in guidance_control.cpp.
  // Without dedicated getter functions, we log placeholders or last known values if available.
  // For now, logging placeholders (0.0f).
  // A more complete solution would involve adding getters to guidance_control.cpp.
  guidance_get_target_euler_angles(logEntry.target_roll,
                                   logEntry.target_pitch,
                                   logEntry.target_yaw);
  guidance_get_pid_integrals(logEntry.pid_roll_integral,
                             logEntry.pid_pitch_integral,
                             logEntry.pid_yaw_integral);
  
  guidance_get_actuator_outputs(logEntry.actuator_output_roll, logEntry.actuator_output_pitch, logEntry.actuator_output_yaw);

  // Output to serial if enabled
  if (enableSerialCSV) {
    // Convert struct to string and print
    Serial.println(logDataToString(logEntry)); 
  }


  // --- SD Card Logging Logic ---
  static unsigned long lastLogWriteErrorTime = 0;
  static unsigned long lastRateLimitedLogStatusMsgTime = 0;
  const unsigned long LOG_WRITE_RETRY_DELAY_MS = 10000; // 10 seconds
  const unsigned long RATE_LIMITED_LOG_STATUS_INTERVAL_MS = 30000; // 30 seconds

  // If SD card is not available or logging is generally disabled, return early.
  if (!sdCardAvailable || !loggingEnabled) {
    if (millis() - lastRateLimitedLogStatusMsgTime > RATE_LIMITED_LOG_STATUS_INTERVAL_MS) {
      Serial.println(F("INFO: Logging to SD card is currently disabled (SD not available or loggingEnabled=false)."));
      lastRateLimitedLogStatusMsgTime = millis();
    }
    return;
  }

  // Check if the log file is open. If not, try to create/reopen it.
  if (!LogDataFile || !LogDataFile.isOpen()) {
    if (millis() - lastLogWriteErrorTime > LOG_WRITE_RETRY_DELAY_MS) { // Rate limit attempts to reopen
      Serial.println(F("INFO: Log file is not open. Attempting to create a new log file..."));
      if (createNewLogFile()) {
        Serial.println(F("INFO: New log file created successfully."));
      } else {
        Serial.println(F("ERROR: Failed to create a new log file. Logging will be suspended for a while."));
        loggingEnabled = false; // Suspend logging if we can't create a file
        lastLogWriteErrorTime = millis(); // Update timestamp to enforce delay
        return; // Exit if file creation failed
      }
    } else {
      // Not enough time has passed since the last error, return to avoid flooding.
      if (millis() - lastRateLimitedLogStatusMsgTime > RATE_LIMITED_LOG_STATUS_INTERVAL_MS) {
         Serial.println(F("INFO: Logging to SD card is temporarily suspended due to previous file operation failure."));
         lastRateLimitedLogStatusMsgTime = millis();
      }
      return;
    }
  }
  
  // At this point, LogDataFile should be open. Attempt to write the log entry.
  String logString = logDataToString(logEntry);
  if (!LogDataFile.println(logString)) {
    Serial.println(F("ERROR: Failed to write data to log file. Current log file might be closed or corrupted."));
    lastLogWriteErrorTime = millis(); // Record time of write error

    // Attempt to recover: Close current file and try to open a new one.
    if (LogDataFile.isOpen()) {
      LogDataFile.close();
      Serial.println(F("INFO: Closed current log file due to write error."));
    }
    
    // Attempt to create a new log file immediately (createNewLogFile has its own internal logic for success/failure)
    if (createNewLogFile()) {
      Serial.println(F("INFO: Successfully created a new log file. Retrying write for the current log entry."));
      // Retry writing the same log entry to the new file
      if (!LogDataFile.println(logString)) {
        Serial.println(F("ERROR: Failed to write data to the *new* log file. Logging will be suspended."));
        loggingEnabled = false; // Critical failure, disable logging
      } else {
        Serial.println(F("INFO: Successfully wrote data to the new log file after recovery."));
        // Consider flushing immediately after successful recovery write
        LogDataFile.flush();
      }
    } else {
      Serial.println(F("ERROR: Failed to create a new log file after write error. Logging will be suspended."));
      loggingEnabled = false; // Disable logging as recovery failed
    }
    return; // Return whether recovery was successful or not, to avoid normal flush logic on a failed write.
  }
  
  // Flush every 10 writes to reduce card wear while ensuring data is written
  static uint8_t flushCounter = 0;
  if (++flushCounter >= 10) {
    LogDataFile.flush();
    flushCounter = 0;
  }
}

void formatNumber(float input, byte columns, byte places) 
{
  char buffer[20]; // Allocate space to store the formatted number string
  dtostrf(input, columns, places, buffer); // Convert float to string with specified columns and decimal places
  Serial.print(buffer); // Print the formatted number to the serial monitor
}

// Updated printStatusSummary function without VT100 codes
void printStatusSummary() { // Note: `enableStatusSummary` is now toggled by 'j' or 'debug_status_summary'
  // Print status summary regardless of system debug flag, as it's controlled by enableStatusSummary 
  
  // Current time
  unsigned long currentMillis = millis();
  unsigned long seconds = currentMillis / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  Serial.println("\n=== STATUS SUMMARY ===");
  Serial.printf("Time: %02d:%02d:%02d\n", hours % 24, minutes % 60, seconds % 60);
  
  // GPS section
  if (enableGPSDebug) {
    Serial.println("\n--- GPS Status ---");
    if (GPS_fixType > 0) {
      Serial.printf("Position: %.6f, %.6f, Alt: %.2fm, Sat: %d, Fix: %d\n", 
                  GPS_latitude / 10000000.0, GPS_longitude / 10000000.0, GPS_altitude / 1000.0, SIV, GPS_fixType);
      Serial.printf("Speed: %.2f km/h, Course: %.2f°\n", GPS_speed / 1000.0 * 3.6, GPS_heading / 100000.0);
    } else {
      Serial.println("No GPS fix");
    }
  }
  
  // IMU section
  if (enableIMUDebug) {
    Serial.println("\n--- IMU Status ---");
    Serial.printf("Temp: %.1f°C, Motion: %s\n", 
                icm_temp, isStationary ? "STATIONARY" : "MOVING");
  }
  
  // Barometer section
  if (enableBaroDebug) {
    Serial.println("\n--- Barometer ---");
    if (baroCalibrated) {
      // The pressure value is in hPa but the altitude formula expects Pa
      // So we need to multiply by 100 to convert hPa to Pa
      float pressurePa = pressure * 100.0; // Convert from hPa to Pa
      float raw_altitude = 44330.0 * (1.0 - pow(pressurePa / 101325.0, 0.1903));
      float calibrated_altitude = raw_altitude + baro_altitude_offset;
      
      Serial.printf("Pressure: %.2f hPa, Temp: %.2f°C\n", pressure, temperature);
      Serial.printf("Altitude: Raw=%.2fm, Calibrated=%.2fm\n", raw_altitude, calibrated_altitude);
    } else {
      Serial.println("Not calibrated");
    }
  }
  
  // Storage section
  if (enableStorageDebug) {
    Serial.println("\n--- Storage ---");
    Serial.printf("SD Card: %s, Space: %s\n", 
                sdCardPresent ? "Present" : "Not found", 
                sdCardMounted ? String(availableSpace / 1024) + "KB free" : "Not mounted");
    Serial.printf("Logging: %s\n", loggingEnabled ? "Enabled" : "Disabled");
  }
  
  // Debug status
  Serial.println("\n--- Debug Status ---");
  Serial.printf("System: %s, IMU: %s, GPS: %s, Baro: %s\n", 
              enableSystemDebug ? "ON" : "OFF",
              enableIMUDebug ? "ON" : "OFF",
              enableGPSDebug ? "ON" : "OFF",
              enableBaroDebug ? "ON" : "OFF");
  Serial.printf("Storage: %s, ICM Raw: %s\n",
              enableStorageDebug ? "ON" : "OFF",
              enableICMRawDebug ? "ON" : "OFF");
  Serial.printf("Serial CSV: %s\n",
              enableSerialCSV ? "ON" : "OFF");

  Serial.println("=====================\n");
}

// Updated help message with more compact formatting
void printHelpMessage() {
  Serial.println(F("=== TRIPLET FLIGHT COMMAND MENU ==="));
  
  // Debug flags status
  Serial.println(F("\nDebug Flags Status:"));
  Serial.println(F("=== TRIPLET FLIGHT COMMAND MENU ==="));
  Serial.println(F("Current Debug Flag States:"));
  Serial.print(F("  System Debug        (1, debug_system [on|off]): ")); Serial.println(enableSystemDebug ? F("ON") : F("OFF"));
  Serial.print(F("  IMU Debug           (2, debug_imu [on|off]): ")); Serial.println(enableIMUDebug ? F("ON") : F("OFF"));
  Serial.print(F("  GPS Debug           (3, debug_gps [on|off]): ")); Serial.println(enableGPSDebug ? F("ON") : F("OFF")); // Calls setGPSDebugging()
  Serial.print(F("  Barometer Debug     (4, debug_baro [on|off]): ")); Serial.println(enableBaroDebug ? F("ON") : F("OFF"));
  Serial.print(F("  Storage Debug       (5, debug_storage [on|off]): ")); Serial.println(enableStorageDebug ? F("ON") : F("OFF"));
  Serial.print(F("  ICM Raw Debug       (6, debug_icm_raw [on|off]): ")); Serial.println(enableICMRawDebug ? F("ON") : F("OFF"));
  Serial.print(F("  Serial CSV Output   (0, debug_serial_csv [on|off]): ")); Serial.println(enableSerialCSV ? F("ON") : F("OFF"));
  Serial.print(F("  Sensor Detail Debug (sd, debug_sensor_detail [on|off]): ")); Serial.println(enableSensorDebug ? F("ON") : F("OFF"));
  Serial.print(F("  Status Summary      (j, summary, debug_status_summary [on|off]): ")); Serial.println(enableStatusSummary ? F("ON") : F("OFF"));
  Serial.print(F("  Detailed Display    (g, debug_detailed_display [on|off]): ")); Serial.println(displayMode ? F("ON") : F("OFF"));

  Serial.println(F("\nSingle-Key Commands:"));
  Serial.println(F("  0-6: Toggle specific debug flags (see status above)"));
  Serial.println(F("  7  : Attempt to initialize SD card and start/restart logging (also 'start_log')"));
  Serial.println(F("  8  : Check SD card status (presence, mount, space, log file) (also 'sd_status')"));
  Serial.println(F("  9  : Initiate Shutdown"));
  Serial.println(F("  a  : Show this help message (also 'help')"));
  Serial.println(F("  b  : Show system component status (also 'status')"));
  Serial.println(F("  c  : Dump all data (Flash - Not Implemented)"));
  Serial.println(F("  d  : Erase all stored data on flash (Flash - Not Implemented)"));
  Serial.println(F("  e  : List available log files (Flash - Not Implemented)"));
  Serial.println(F("  f  : Show SD card statistics"));
  Serial.println(F("  g  : Toggle detailed display mode"));
  Serial.println(F("  h  : Calibrate barometer (also 'calibrate')"));
  Serial.println(F("  i  : Display current IMU data"));
  Serial.println(F("  j  : Toggle status summary display"));
  
  Serial.println(F("\nMulti-Character Commands:"));
  Serial.println(F("  start_log                  : Attempt to initialize SD card and start/restart logging"));
  Serial.println(F("  sd_status                  : Check SD card status"));
  Serial.println(F("  debug_<flag_name> [on|off] : Set or toggle a debug flag. Examples: 'debug_system on', 'debug_imu off', 'debug_gps'"));
  Serial.println(F("    Known flags: system, imu, gps, baro, storage, icm_raw, serial_csv, sensor_detail, status_summary, detailed_display"));
  Serial.println(F("  debug_all_off              : Disable all common debug flags"));
  Serial.println(F("  help                       : Show this help message"));
  Serial.println(F("  calibrate                  : Attempt barometer calibration"));
  Serial.println(F("  status                     : Show system component status"));
  Serial.println(F("  summary                    : Toggle status summary display"));
  Serial.println(F("  set_orientation_filter [madgwick|kalman] : Selects the active orientation filter."));
  Serial.println(F("  get_orientation_filter                 : Shows the currently active orientation filter."));
  Serial.println(F("\nLegacy Commands (use new versions if possible):"));
  Serial.println(F("  sd                         : Toggle sensor_detail_debug"));
  Serial.println(F("  rd                         : Toggle icm_raw_debug"));
  Serial.println(F("================================="));
  Serial.println(F("  j - Toggle status summary"));
  Serial.println(F("\nExtended commands:"));
  Serial.println(F("  debug_all_off - Disable all debugging output"));
  Serial.println(F("  calibrate - Manually calibrate barometer with GPS"));
  Serial.println(F("  help - Show this help message"));
  Serial.println(F("  arm - Arm the flight computer (transition to ARMED state)"));
  Serial.println(F("  test_error - Simulate sensor error (for testing)"));
  Serial.println(F("  test_watchdog - Simulate watchdog timeout (for testing)"));
  Serial.println(F("  clear_errors - Reset all error flags and return to normal operation"));
}

// Function to print detailed SD Card status
void printSDCardStatus() {
  Serial.println(F("\n--- SD Card Status ---"));
  checkStorageSpace(); // Ensure availableSpace is up-to-date

  Serial.print(F("Physically Present: "));
  Serial.println(sdCardPresent ? F("Yes") : F("No (SDIO initialization failed)"));

  Serial.print(F("Mounted: "));
  Serial.println(sdCardMounted ? F("Yes") : F("No"));

  Serial.print(F("Logging Enabled: "));
  Serial.println(loggingEnabled ? F("Yes") : F("No"));

  if (sdCardMounted) {
    Serial.print(F("Available Space: "));
    if (availableSpace >= 1024 * 1024) { // More than 1MB
      Serial.print(availableSpace / (1024ULL * 1024ULL));
      Serial.println(F(" MB"));
    } else {
      Serial.print(availableSpace / 1024ULL);
      Serial.println(F(" KB"));
    }
    Serial.print(F("Current Log File: "));
    if (strlen(logFileName) > 0 && loggingEnabled) {
      Serial.println(logFileName);
    } else {
      Serial.println(F("None or logging not active"));
    }
  } else {
    Serial.println(F("Available Space: N/A (Card not mounted)"));
    Serial.println(F("Current Log File: N/A (Card not mounted)"));
  }
  Serial.println(F("----------------------"));
}

// Function to attempt to initialize SD card and start/restart logging
void attemptToStartLogging() {
  Serial.println(F("\n--- Attempting to Start Logging ---"));

  // Explicitly re-initialize the SD card. This will also reset status flags.
  Serial.println(F("Step 1: Initializing SD card..."));
  sdCardAvailable = initSDCard(); // initSDCard() updates sdCardPresent, sdCardMounted

  if (sdCardAvailable) {
    Serial.println(F("SUCCESS: SD card initialized."));
    Serial.println(F("Step 2: Attempting to create a new log file..."));
    if (createNewLogFile()) { // createNewLogFile() sets loggingEnabled to true on success
      Serial.print(F("SUCCESS: Logging started. Current log file: "));
      Serial.println(logFileName);
    } else {
      Serial.println(F("ERROR: Failed to create a new log file. Logging remains disabled."));
      // loggingEnabled is already set to false by createNewLogFile() on failure
    }
  } else {
    Serial.println(F("ERROR: Failed to initialize SD card. Logging cannot be started."));
    loggingEnabled = false; // Ensure logging is disabled
    // sdCardPresent and sdCardMounted are handled by initSDCard()
  }
  Serial.println(F("--- Logging Attempt Complete ---"));
}

// Updated storage stats with more concise output
void printStorageStatistics() {
  Serial.println(F("\n----- STORAGE STATISTICS -----"));
  
  // SD Card stats
  Serial.print(F("SD Card: "));
  if (sdCardAvailable) {
    uint32_t usedSpace = 0;
    FsFile root = SD.open("/");
    while (true) {
      FsFile entry = root.openNextFile();
      if (!entry) break;
      usedSpace += entry.size();
      entry.close();
    }
  root.close();
  
    Serial.print(usedSpace / 1024);
    Serial.print(F(" KB used"));
    
    if (strlen(logFileName) > 0) {
      Serial.print(F(" | Log: "));
      Serial.println(logFileName);
    } else {
    Serial.println();
  }
  } else {
    Serial.println(F("Not available"));
  }
  
  Serial.println(F("-----------------------------"));
}

// Helper function to toggle debug flags and print their status
// specificState: -1 = toggle, 0 = off, 1 = on
void toggleDebugFlag(volatile bool& flag, const __FlashStringHelper* name, Stream& output, int specificState = -1) {
  if (specificState == 0) { // Force OFF
    flag = false;
  } else if (specificState == 1) { // Force ON
    flag = true;
  } else { // Toggle
    flag = !flag;
  }
  output.print(name);
  output.print(F(": "));
  output.println(flag ? F("ON") : F("OFF"));

  // Special handling for GPS debug as it calls an external function to control library prints
  // Note: Comparing F() string pointers works because they are compile-time constants.
  if (name == F("GPS debug")) {
      setGPSDebugging(flag);
  }
}

// Function to perform actual calibration logic (extracted for reuse)
void performCalibration() {
                    if (!baroCalibrated) {
        pixels.setPixelColor(0, pixels.Color(50, 0, 50)); // Purple for calibration in progress
                        pixels.show();
        Serial.println(F("Starting barometric calibration with GPS..."));
        Serial.println(F("Waiting for good GPS fix (pDOP < 3.0)..."));
        
        if (ms5611_calibrate_with_gps(30000)) {  // Wait up to 30 seconds for calibration
                            Serial.println(F("Barometric calibration successful!"));
                            baroCalibrated = true;
            pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green for success
                            pixels.show();
                            delay(1000);
                        } else {
                            Serial.println(F("Barometric calibration timed out or failed."));
            pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red for failure
                            pixels.show();
                            delay(1000);
                        }
                    } else {
                        Serial.println(F("Barometric calibration has already been performed."));
    }
}

// Function to print system status (extracted for reuse by 'b' and legacy "status")
void printSystemStatus() {
        Serial.println("System Status:");
        Serial.print("SD Card: ");
        Serial.println(sdCardAvailable ? "Available" : "Not Available");
        Serial.print("External Flash: ");
        Serial.println(flashAvailable ? "Available" : "Not Available");
        Serial.print("GPS: ");
        Serial.println(myGNSS.getPVT() ? "Available" : "Not Available");
        Serial.print("Barometer: ");
        Serial.println(ms5611Sensor.isConnected() ? "Available" : "Not Available");
        Serial.print("Accelerometer: ");
        Serial.println(kx134Accel.dataReady() ? "Available" : "Not Available");
}


void processCommand(String command) {
    command.trim(); // Trim whitespace

    if (command.length() == 1) {
        char cmd = command.charAt(0);
        
        if (cmd >= '0' && cmd <= '9') {
            switch (cmd) {
                case '0': toggleDebugFlag(enableSerialCSV, F("Serial CSV output"), Serial); break;
                case '1': toggleDebugFlag(enableSystemDebug, F("System debug"), Serial); break;
                case '2': toggleDebugFlag(enableIMUDebug, F("IMU debug"), Serial); break;
                case '3': toggleDebugFlag(enableGPSDebug, F("GPS debug"), Serial); break;
                case '4': toggleDebugFlag(enableBaroDebug, F("Barometer debug"), Serial); break;
                case '5': toggleDebugFlag(enableStorageDebug, F("Storage debug"), Serial); break;
                case '6': toggleDebugFlag(enableICMRawDebug, F("ICM raw debug"), Serial); break;
                case '7': attemptToStartLogging(); break;
                case '8': printSDCardStatus(); break;
                case '9': prepareForShutdown(); break;
                default: Serial.println(F("Unknown numeric command.")); break;
            }
            return;
        }
        
        if (cmd >= 'a' && cmd <= 'k') { // Adjusted range if needed, 'k' seems to be the last one.
            switch (cmd) {
                case 'a': printHelpMessage(); break;
                case 'b': printSystemStatus(); break;
                case 'c': // Dump
                    if (flashAvailable) {
                        Serial.println(F("Dump command not supported for external flash."));
      } else {
                        Serial.println(F("External flash not available."));
                    }
                    break;
                case 'd': // Erase flash
                    if (flashAvailable) {
                        Serial.println(F("Erase command not supported for external flash."));
      } else {
                        Serial.println(F("External flash not available."));
                    }
                    break;
                case 'e': // List logs
                    if (flashAvailable) {
                        Serial.println(F("List command not supported for external flash."));
      } else {
                        Serial.println(F("External flash not available."));
                    }
                    break;
                case 'f': printStorageStatistics(); break;
                case 'g': toggleDebugFlag(displayMode, F("Detailed display mode"), Serial); break;
                case 'h': performCalibration(); break;
                case 'i': ICM_20948_print(); break;
                case 'j': toggleDebugFlag(enableStatusSummary, F("Status summary"), Serial); break;
                default: Serial.println(F("Unknown alphabetic command.")); break;
            }
            return;
        }
    }
    
    // Multi-character and legacy commands
    if (command == "help") {
        printHelpMessage();
    } else if (command == "calibrate") {
        performCalibration();
    } else if (command == "arm") {
        if (currentFlightState == PAD_IDLE) {
            currentFlightState = ARMED;
            Serial.println(F("System ARMED. Ready for launch."));
            if (!enableSerialCSV) {
                Serial.println(F("Note: Serial CSV output is off. Web UI will not update until it is enabled ('0')."));
            }
        } else {
            Serial.print(F("Cannot arm. System is not in PAD_IDLE state. Current state: "));
            Serial.println(getStateName(currentFlightState));
        }
    } else if (command == "test_error") {
        Serial.println(F("Simulating sensor error..."));
        // Implement sensor error simulation logic
    } else if (command == "test_watchdog") {
        Serial.println(F("Simulating watchdog timeout..."));
        // Implement watchdog timeout simulation logic
    } else if (command == "clear_errors") {
        Serial.println(F("Clearing error state and resetting to normal operation..."));
        if (currentFlightState == ERROR) {
            // Check if systems are healthy before clearing error
            bool systemHealthy = true; // TODO: Implement proper health check
            // For now, assume system is healthy if we can execute commands
            
            currentFlightState = PAD_IDLE;
            stateEntryTime = millis();
            
            // Update LED to green
            pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green
            pixels.show();
            
            Serial.println(F("Error state cleared. System reset to PAD_IDLE."));
            Serial.println(F("Note: Please verify all systems are functioning correctly."));
            
            // Save the cleared state to EEPROM
            saveStateToEEPROM();
        } else {
            Serial.print(F("System is not in ERROR state. Current state: "));
            Serial.println(getStateName(currentFlightState));
        }
    } else if (command == "summary") { // Legacy "summary"
        toggleDebugFlag(enableStatusSummary, F("Status summary"), Serial);
    } else if (command == "status") { // Legacy "status"
        printSystemStatus();
    } else if (command == "sd_status") {
        printSDCardStatus();
    } else if (command == "start_log") {
        attemptToStartLogging();
    } else if (command == "sd") { // Legacy "sd"
        toggleDebugFlag(enableSensorDebug, F("Sensor detail debug"), Serial);
    } else if (command == "rd") { // Legacy "rd"
        toggleDebugFlag(enableICMRawDebug, F("ICM raw debug"), Serial);
    }
    // Generic debug_xxx [on|off|toggle] commands
    else if (command.startsWith("debug_")) {
        String flagNamePart = command.substring(6); // Remove "debug_" prefix
        int specificState = -1; // Default to toggle
        String flagIdentifier = flagNamePart;

        if (flagNamePart.endsWith(" on")) {
            specificState = 1; // on
            flagIdentifier = flagNamePart.substring(0, flagNamePart.length() - 3);
        } else if (flagNamePart.endsWith(" off")) {
            specificState = 0; // off
            flagIdentifier = flagNamePart.substring(0, flagNamePart.length() - 4);
        }
        flagIdentifier.trim(); // Remove any trailing space

        if (flagIdentifier == "system") toggleDebugFlag(enableSystemDebug, F("System debug"), Serial, specificState);
        else if (flagIdentifier == "imu") toggleDebugFlag(enableIMUDebug, F("IMU debug"), Serial, specificState);
        else if (flagIdentifier == "gps") toggleDebugFlag(enableGPSDebug, F("GPS debug"), Serial, specificState);
        else if (flagIdentifier == "baro") toggleDebugFlag(enableBaroDebug, F("Barometer debug"), Serial, specificState);
        else if (flagIdentifier == "storage") toggleDebugFlag(enableStorageDebug, F("Storage debug"), Serial, specificState);
        else if (flagIdentifier == "icm_raw") toggleDebugFlag(enableICMRawDebug, F("ICM raw debug"), Serial, specificState);
        else if (flagIdentifier == "serial_csv") toggleDebugFlag(enableSerialCSV, F("Serial CSV output"), Serial, specificState);
        else if (flagIdentifier == "sensor_detail") toggleDebugFlag(enableSensorDebug, F("Sensor detail debug"), Serial, specificState);
        else if (flagIdentifier == "status_summary") toggleDebugFlag(enableStatusSummary, F("Status summary"), Serial, specificState);
        else if (flagIdentifier == "detailed_display") toggleDebugFlag(displayMode, F("Detailed display mode"), Serial, specificState);
        else if (flagIdentifier == "all_off") { // Handles "debug_all_off", "debug_all_off off"
             if (specificState == 0 || specificState == -1) { // Only allow "off" or "toggle" to off
                Serial.println(F("Disabling all debug flags:"));
                toggleDebugFlag(enableSerialCSV, F("Serial CSV output"), Serial, 0);
                toggleDebugFlag(enableSystemDebug, F("System debug"), Serial, 0);
                toggleDebugFlag(enableIMUDebug, F("IMU debug"), Serial, 0);
                toggleDebugFlag(enableGPSDebug, F("GPS debug"), Serial, 0);
                toggleDebugFlag(enableBaroDebug, F("Barometer debug"), Serial, 0);
                toggleDebugFlag(enableStorageDebug, F("Storage debug"), Serial, 0);
                toggleDebugFlag(enableICMRawDebug, F("ICM raw debug"), Serial, 0);
                toggleDebugFlag(enableStatusSummary, F("Status summary"), Serial, 0);
                toggleDebugFlag(displayMode, F("Detailed display mode"), Serial, 0);
                toggleDebugFlag(enableSensorDebug, F("Sensor detail debug"), Serial, 0);
                enableDetailedOutput = false; // This was a global not covered by a specific toggle before
                Serial.println(F("Legacy Detailed output (global): OFF"));

            } else { // "debug_all_off on" is not logical
                 Serial.println(F("debug_all_off only supports 'off' or toggle to off."));
            }
      } else {
            Serial.print(F("Unknown debug flag: "));
            Serial.println(flagIdentifier);
        }
    } else if (command.startsWith("set_orientation_filter ")) {
        String filterType = command.substring(23); // Length of "set_orientation_filter "
        filterType.trim();
        if (filterType == "madgwick") {
            useMadgwickFilter = true;
            useKalmanFilter = false;
            Serial.println(F("Orientation filter set to Madgwick."));
            // Optional: if Madgwick needs specific re-initialization, add here
        } else if (filterType == "kalman") {
            if (icm20948_ready) { // Kalman needs ICM to be ready
                useMadgwickFilter = false;
                useKalmanFilter = true;
                // Re-initialize Kalman filter when switching.
                float initial_roll = 0.0f;
                float initial_pitch = 0.0f;
                float initial_yaw = 0.0f;
                // Optionally, use current Madgwick orientation if available:
                // if (!useMadgwickFilter && icm_q0 != 1.0f && (icm_q1 != 0.0f || icm_q2 != 0.0f || icm_q3 != 0.0f)) { // Check if Madgwick has valid non-default quaternion
                //    convertQuaternionToEuler(icm_q0, icm_q1, icm_q2, icm_q3, initial_roll, initial_pitch, initial_yaw);
                // }
                kalman_init(initial_roll, initial_pitch, initial_yaw);
                Serial.println(F("Orientation filter set to Kalman. Initialized/Re-initialized."));
            } else {
                Serial.println(F("Cannot switch to Kalman filter: ICM20948 not ready."));
            }
        } else {
            Serial.print(F("Unknown filter type: "));
            Serial.println(filterType);
        }
    } else if (command == "get_orientation_filter") {
        Serial.print(F("Current orientation filter: "));
        if (useKalmanFilter) {
            Serial.println(F("Kalman"));
        } else if (useMadgwickFilter) { // Madgwick is effectively the default if Kalman is not chosen
            Serial.println(F("Madgwick"));
        } else {
            // This case should ideally not be reached if one is always true.
            // However, if KALMAN_FILTER_ACTIVE_BY_DEFAULT was false, and useMadgwickFilter was also set to false somehow.
            Serial.println(F("None (Madgwick is default if Kalman is false)"));
        }
        Serial.print(F("  useKalmanFilter flag: ")); Serial.println(useKalmanFilter);
        Serial.print(F("  useMadgwickFilter flag: ")); Serial.println(useMadgwickFilter);
    } else if (command == "debug_all_off") { // Direct command for convenience
        Serial.println(F("Disabling all debug flags:"));
        toggleDebugFlag(enableSerialCSV, F("Serial CSV output"), Serial, 0);
        toggleDebugFlag(enableSystemDebug, F("System debug"), Serial, 0);
        toggleDebugFlag(enableIMUDebug, F("IMU debug"), Serial, 0);
        toggleDebugFlag(enableGPSDebug, F("GPS debug"), Serial, 0);
        toggleDebugFlag(enableBaroDebug, F("Barometer debug"), Serial, 0);
        toggleDebugFlag(enableStorageDebug, F("Storage debug"), Serial, 0);
        toggleDebugFlag(enableICMRawDebug, F("ICM raw debug"), Serial, 0);
        toggleDebugFlag(enableStatusSummary, F("Status summary"), Serial, 0);
        toggleDebugFlag(displayMode, F("Detailed display mode"), Serial, 0);
        toggleDebugFlag(enableSensorDebug, F("Sensor detail debug"), Serial, 0);
      enableDetailedOutput = false;
        Serial.println(F("Legacy Detailed output (global): OFF"));
    } else if (command.equalsIgnoreCase("calibrate_mag")) {
        Serial.println(F("Starting interactive magnetometer calibration. Please rotate the device around all axes."));
        ICM_20948_calibrate_mag_interactive();
    } else if (command.equalsIgnoreCase("calibrate_gyro")) {
        Serial.println(F("Starting gyro calibration..."));
        ICM_20948_calibrate_gyro_bias(2000, 1); // Using default parameters
    } else if (command.equalsIgnoreCase("save_mag_cal")) {
        Serial.println(F("Saving magnetometer calibration..."));
        if (icm_20948_save_calibration()) {
            Serial.println(F("Magnetometer calibration saved successfully."));
        } else {
            Serial.println(F("Failed to save magnetometer calibration."));
        }
    }
}

void setup() {
  // Wait for the Serial monitor to be opened.
  Serial.begin(115200);
  delay(500); // Give the serial port time to initialize

  // Initialize NeoPixel first for visual feedback
  pixels.begin(); // Initialize pixels early for feedback during setup
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(20, 0, 0)); // Red during startup
  pixels.setPixelColor(1, pixels.Color(20, 0, 0)); // Red during startup
  pixels.show();

  // Basic hardware init needed for recovery check (e.g. EEPROM access via I2C)
  Wire.begin();
  Wire.setClock(400000); // Ensure I2C is up for EEPROM
  SPI.begin(); // Ensure SPI is up if EEPROM uses it (though typical EEPROM is I2C)

  Serial.println(F("Checking for flight state recovery..."));
  recoverFromPowerLoss(); // This function will update currentFlightState

  Serial.print(F("Flight state after recovery attempt: "));
  Serial.println(getStateName(currentFlightState)); // Assumes getStateName is available via utility_functions.h
  
  // Important: Disable all debugging immediately at startup (unless recovery dictates otherwise, though not typical)
  enableDetailedOutput = false;
  enableSystemDebug = false;
  enableSensorDebug = false;
  enableIMUDebug = false;
  enableGPSDebug = false;
  enableBaroDebug = false;
  enableStorageDebug = false;
  enableICMRawDebug = false;
  displayMode = false;
  
  // Startup Tone (after initial pixel setup and recovery message)
  delay(500);
  tone(BUZZER_PIN, 2000); delay(50); noTone(BUZZER_PIN); delay(75);
  noTone(BUZZER_PIN);
  
  // Change LED to orange to indicate waiting for serial
  pixels.setPixelColor(0, pixels.Color(25, 12, 0)); // Orange during serial init
  pixels.show();
  
  // Wait for serial connection with timeout
  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 5000)) {
    // Wait up to 5 seconds for serial connection
    delay(100);
  }
  
  Serial.print("TripleT Flight Firmware Alpha ");
  Serial.println(TRIPLET_FLIGHT_VERSION);
  Serial.print("Board: ");
  Serial.println(BOARD_NAME);
  
  // Change LED to yellow during initialization
  pixels.setPixelColor(0, pixels.Color(50, 50, 0)); // Yellow during init
  pixels.show();

  // Scan the I2C bus for devices (can be after recovery attempt)
  scan_i2c();

  // Initialize GPS first to get accurate time (essential for logging and potentially calibration)
  Serial.println(F("Initializing GPS module..."));
  enableGPSDebug = false; // Disable GPS debug output
  gps_init();
  Serial.println(F("GPS module initialized"));
  
  // Wait for GPS to have valid date/time if possible
  // Uses a blue "breathing" pattern on the LED to show it's waiting
  Serial.println(F("Waiting for GPS time sync..."));
  bool gpsTimeValid = false;
  unsigned long gpsWaitStart = millis();
  unsigned long lastLedUpdate = 0;
  int brightness = 0;
  int step = 5;
  
  while (!gpsTimeValid && (millis() - gpsWaitStart < 30000)) {  // Wait up to 30 seconds
    // Update GPS data
    gps_read();
    
    // Check if we have a valid year (2025 or later)
    if (myGNSS.getYear() >= 2025) {
      gpsTimeValid = true;
      // Set LED to cyan to indicate valid GPS time
      pixels.setPixelColor(0, pixels.Color(0, 50, 50));
      pixels.show();
      break;
    }
    
    // Update "breathing" LED effect every 50ms
    if (millis() - lastLedUpdate > 50) {
      lastLedUpdate = millis();
      brightness += step;
      if (brightness >= 50) {
        brightness = 50;
        step = -step;
      } else if (brightness <= 0) {
        brightness = 0;
        step = -step;
      }
      pixels.setPixelColor(0, pixels.Color(0, 0, brightness));
      pixels.show();
    }
    
    delay(100);
  }
  
  if (gpsTimeValid) {
    Serial.print(F("GPS time synced: "));
  } else {
    Serial.print(F("GPS time sync timeout. Current time: "));
  }
  
  Serial.print(myGNSS.getYear());
  Serial.print(F("-"));
  Serial.print(myGNSS.getMonth());
  Serial.print(F("-"));
  Serial.print(myGNSS.getDay());
  Serial.print(F(" "));
  Serial.print(myGNSS.getHour());
  Serial.print(F(":"));
  Serial.print(myGNSS.getMinute());
  Serial.print(F(":"));
  Serial.println(myGNSS.getSecond());
  
  // Change LED to purple for other sensor initialization
  pixels.setPixelColor(0, pixels.Color(25, 0, 25));
  pixels.show();

  // Now initialize other sensors
  Serial.println(F("\nInitializing sensors..."));
  if (!kx134_init()) {
    Serial.println(F("WARNING: KX134 accelerometer initialization failed"));
  } else {
    Serial.println(F("KX134 accelerometer initialized"));
  }
  
  // Initialize ICM-20948
  ICM_20948_init();
  Serial.println(F("ICM-20948 initialized."));
  // Attempt to load magnetometer calibration from EEPROM
  if (!icm_20948_load_calibration()) {
      Serial.println(F("Magnetometer calibration not found. Please run 'cal_mag' command."));
  }
  
  ms5611_init();
  Serial.println(F("MS5611 initialized"));
  
  // Change LED to white before storage initialization
  pixels.setPixelColor(0, pixels.Color(25, 25, 25));
  pixels.show();
  
  // Give extra time for SD card to stabilize
  delay(500);
  
#if !DISABLE_SDCARD_LOGGING
  // Initialize storage only if logging is enabled
  Serial.println(F("Initializing storage..."));
  
  // Initialize SD card
  sdCardAvailable = initSDCard();
  Serial.print(F("SD Card: "));
  Serial.println(sdCardAvailable ? F("Available") : F("Not available"));
    
  if (sdCardAvailable) {
    // Only try to create a log file if SD card is available
    Serial.println(F("Creating new log file..."));
    if (createNewLogFile()) {
      Serial.println(F("Data logging ready."));
    } else {
      Serial.println(F("Failed to create log file, logging will be disabled."));
      loggingEnabled = false;
    }
  } else {
    // This case handles if initSDCard() failed even though logging is enabled
    Serial.println(F("WARNING: Storage initialization failed! Logging disabled."));
    loggingEnabled = false;
  }
#else // DISABLE_SDCARD_LOGGING is true
  // Ensure logging is explicitly disabled if the flag is set
  Serial.println(F("SD Card logging disabled by configuration."));
  sdCardAvailable = false;
  loggingEnabled = false;
  sdCardPresent = false;
  sdCardMounted = false;
#endif // !DISABLE_SDCARD_LOGGING
  // Change LED to green to indicate successful initialization (regardless of SD status) - This might be too early, state-dependent now.
  // pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green
  // pixels.show();
  
  // Barometric calibration is now done via command OR if state is CALIBRATION
  if (currentFlightState == CALIBRATION) {
    Serial.println(F("State is CALIBRATION. Attempting barometric calibration if not already done."));
    // performCalibration(); // This function has user interaction/blocking, might need adjustment for setup
                           // For now, assume calibration is initiated here or handled by main loop if state is CALIBRATION
  } else {
    Serial.println(F("Barometric calibration can be initiated via 'calibrate' command if needed."));
  }

  // Initialize Guidance Control System
  guidance_init();
  Serial.println(F("Guidance control system initialized."));

  // Initialize Actuators
  Serial.println(F("Initializing Actuators..."));
  servo_pitch.attach(ACTUATOR_PITCH_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
  servo_roll.attach(ACTUATOR_ROLL_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);
  servo_yaw.attach(ACTUATOR_YAW_PIN, SERVO_MIN_PULSE_WIDTH, SERVO_MAX_PULSE_WIDTH);

  // Set servos to a neutral/default position
  servo_pitch.write(SERVO_DEFAULT_ANGLE);
  servo_roll.write(SERVO_DEFAULT_ANGLE);
  servo_yaw.write(SERVO_DEFAULT_ANGLE);
  Serial.println(F("Actuators initialized and set to default positions."));

  // --- State-based setup adjustments after all hardware init ---

  // Initialize Pyro Pins (as per State Machine.md setup)
  // This should be done regardless of state to ensure pins are in a known safe state.
  pinMode(PYRO_CHANNEL_1, OUTPUT);
  pinMode(PYRO_CHANNEL_2, OUTPUT);
  digitalWrite(PYRO_CHANNEL_1, LOW);
  digitalWrite(PYRO_CHANNEL_2, LOW);
  Serial.println(F("Pyro channels initialized to LOW."));

  // Conditional transition to CALIBRATION or other states
  if (currentFlightState == STARTUP) { // Only transition if starting fresh (recovery didn't set a later state)
    Serial.println(F("Fresh start detected, proceeding to CALIBRATION state."));
    currentFlightState = CALIBRATION;
    stateEntryTime = millis(); 
    pixels.setPixelColor(0, pixels.Color(50, 50, 0)); // Yellow during calibration
    pixels.show();
    // updateLoggingRate(currentFlightState); // To be handled by ProcessFlightState or main loop
    // setStateTimeout(CALIBRATION);         // To be handled by ProcessFlightState or main loop
  } else {
    Serial.print(F("Resuming operation in state: "));
    Serial.println(getStateName(currentFlightState));
    // If resuming directly into a state that expects certain hardware to be active (e.g. pyro channels for deployment states)
    // The actual deployment actions should be part of ProcessFlightState to ensure consistency.
    // For setup(), just ensuring pyro pins are initialized (done above) is sufficient.
  }

  // Final state assignment before loop()
  // This needs to be careful not to override a recovered state if it's valid.
  bool systemHealthy = true; // Placeholder for actual health check. 
                            // TODO: Implement isSensorSuiteHealthy() or similar.
                            // This function should check GPS, IMU, Baro, SD card status.
  
  if (!sdCardAvailable && loggingEnabled) { // Example check: if logging is on but SD fails, system is not healthy.
      Serial.println(F("ERROR: Logging enabled but SD card not available. System unhealthy."));
      systemHealthy = false;
  }
  // Add more checks for critical sensors to set systemHealthy = false if they fail init.
  // For example:
  // if (!ms5611Sensor.isConnected()) systemHealthy = false; // Assuming ms5611_init updates this
  // if (!kx134_initialized_ok && !icm20948_ready) systemHealthy = false; // If no IMU is good. (Need to define these flags based on init funcs)


  if (currentFlightState == CALIBRATION) { 
      if (systemHealthy) {
          Serial.println(F("System healthy, proceeding to PAD_IDLE state."));
          currentFlightState = PAD_IDLE;
      } else {
          Serial.println(F("System unhealthy after calibration attempt, proceeding to ERROR state."));
          currentFlightState = ERROR;
      }
      stateEntryTime = millis();
      // updateLoggingRate(currentFlightState); // Handled by ProcessFlightState
  } else if (currentFlightState == STARTUP && systemHealthy) {
       // This case handles if recovery somehow ended in STARTUP (e.g. EEPROM invalid) but system is now healthy
       Serial.println(F("Fresh start, system healthy, proceeding to PAD_IDLE state."));
       currentFlightState = PAD_IDLE;
       stateEntryTime = millis();
  } else if (currentFlightState == ERROR && systemHealthy) {
      // If we recovered an ERROR state but all systems are now healthy, automatically clear it
      Serial.println(F("ERROR state recovered but all systems are healthy. Automatically clearing error and transitioning to PAD_IDLE."));
      currentFlightState = PAD_IDLE;
      stateEntryTime = millis();
      // Save the cleared state to EEPROM
      saveStateToEEPROM();
  } else if (!systemHealthy && currentFlightState != ERROR) {
      // If system is not healthy and not already in ERROR state (e.g. recovered into a flight state but sensors now fail during setup)
      Serial.println(F("System became unhealthy during setup, transitioning to ERROR state."));
      currentFlightState = ERROR;
      stateEntryTime = millis();
  }
  // If currentFlightState is already a later flight state (e.g., DROGUE_DESCENT) and system is healthy,
  // it will remain in that state. If system becomes unhealthy during its specific setup/checks, it transitions to ERROR.

  Serial.print(F("Setup complete. Initial flight state for loop(): "));
  Serial.println(getStateName(currentFlightState));
  
  // Final LED indication based on state
  if (currentFlightState == PAD_IDLE) pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green
  else if (currentFlightState == ERROR) pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red
  else if (currentFlightState == CALIBRATION) pixels.setPixelColor(0, pixels.Color(50, 50, 0)); // Yellow
  // Other states will be handled by ProcessFlightState's display logic in the main loop.
  pixels.show();
}

void loop() {
  // --- Timekeeping and Sensor Update Flags ---
  static unsigned long lastGPSReadTime = 0;
  static unsigned long lastIMUReadTime = 0;   // For ICM20948 IMU
  static unsigned long lastBaroReadTime = 0;
  static unsigned long lastKalmanUpdateTime = 0; // For Kalman filter dt calculation

  bool sensorsUpdatedThisCycle = false; // Track if any sensor was updated this cycle

  // --- Serial Command Processing ---
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command); // Existing command processor
  }

  // --- Sensor Data Reads ---
  if (millis() - lastGPSReadTime >= GPS_POLL_INTERVAL) {
    lastGPSReadTime = millis();
    gps_read(); // Existing GPS read function
    sensorsUpdatedThisCycle = true;
    // NOTE: Old automatic barometer calibration logic based on pDOP < 300 from loop() should be REMOVED.
    // Calibration is now handled by setup() and the CALIBRATION state in ProcessFlightState.
  }
  if (millis() - lastBaroReadTime >= BARO_POLL_INTERVAL) {
    lastBaroReadTime = millis();
    if (ms5611_read() == MS5611_READ_OK) { // Existing barometer read
        sensorsUpdatedThisCycle = true;
    }
  }
  if (millis() - lastIMUReadTime >= IMU_POLL_INTERVAL) {
    lastIMUReadTime = millis();
    ICM_20948_read(); // Existing ICM read
    sensorsUpdatedThisCycle = true;

    // --- Kalman Filter Processing ---
    if (useKalmanFilter && icm20948_ready) {
        // Sensor switching logic for Kalman filter accelerometer
        float current_accel_for_kalman[3]; // Temporary array to hold accel data for Kalman
        bool kx134_was_used = usingKX134ForKalman; // Store previous state for message toggling

        // Default to ICM20948 accelerometer
        memcpy(current_accel_for_kalman, icm_accel, sizeof(icm_accel));
        usingKX134ForKalman = false;

        // Calculate magnitude of ICM20948 acceleration
        float icm_accel_magnitude = sqrt(icm_accel[0] * icm_accel[0] +
                                         icm_accel[1] * icm_accel[1] +
                                         icm_accel[2] * icm_accel[2]);

        if (icm_accel_magnitude > 16.0f) { // Threshold for switching to KX134 (e.g. >16g)
            if (kx134_initialized_ok) { // Check if KX134 is available and working
                memcpy(current_accel_for_kalman, kx134_accel, sizeof(kx134_accel));
                usingKX134ForKalman = true;
                if (!kx134_was_used && enableIMUDebug) { // Print only on change
                    Serial.println(F("KALMAN: High-G detected. Switched to KX134 for accelerometer data."));
                }
            } else {
                if (enableIMUDebug) { // If KX134 not ok, print warning but continue with ICM (already copied)
                    Serial.println(F("KALMAN: High-G detected, but KX134 not available. Using ICM20948 accel."));
                }
            }
        } else {
            // Already using ICM20948 (default), no change needed for data
            // usingKX134ForKalman is already false
            if (kx134_was_used && enableIMUDebug) { // Print only on change
                Serial.println(F("KALMAN: Low-G detected. Switched back to ICM20948 for accelerometer data."));
            }
        }

        float dt_kalman = 0.0f;
        unsigned long currentTimeMillis = millis(); // Cache current time

        if (lastKalmanUpdateTime > 0) { // Ensure lastKalmanUpdateTime has been initialized after the first run
            dt_kalman = (currentTimeMillis - lastKalmanUpdateTime) / 1000.0f;
        }
        lastKalmanUpdateTime = currentTimeMillis;

        if (dt_kalman > 0.0f && dt_kalman < 1.0f) { // Basic sanity check for dt
            // Gyro data is in icm_gyro (rad/s)
            // Accel data for Kalman is now in current_accel_for_kalman (g's)

            kalman_predict(icm_gyro[0], icm_gyro[1], icm_gyro[2], dt_kalman);
            kalman_update_accel(current_accel_for_kalman[0], current_accel_for_kalman[1], current_accel_for_kalman[2]);
            float mag_data[3];
            icm_20948_get_mag(mag_data);
            kalman_update_mag(mag_data[0], mag_data[1], mag_data[2]);
            kalman_get_orientation(kalmanRoll, kalmanPitch, kalmanYaw);
        }
    }
  }

  // TODO: Add other periodic tasks here as needed
  // - Flight state processing
  // - Actuator updates 
  // - Data logging
  // - Status monitoring
  
  // Note: The rest of the loop function implementation should be added here
  // based on the flight state machine and other system requirements
}
