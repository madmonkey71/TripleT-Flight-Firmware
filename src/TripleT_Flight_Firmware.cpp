// TripleT Flight Firmware
// Current Version: v0.30
// Current State: Alpha
// Last Updated: 24/05/2025
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
#define TRIPLET_FLIGHT_VERSION 0.30

// Define the board type if not defined by platformio.ini
#ifndef BOARD_TEENSY40
#ifndef BOARD_TEENSY41
#if defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41)
#define BOARD_TEENSY41
#elif defined(__IMXRT1062__) && defined(ARDUINO_TEENSY40)
#define BOARD_TEENSY40
#endif
#endif
#endif

// Now include the GPS and sensor functions
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

// Define variables declared as extern in utility_functions.h
String FileDateString;  // For log file naming
String LogDataString;   // For data logging
unsigned long currentTime;  // For timestamp
bool baroCalibrated = false;  // For barometric calibration status

#if defined(BOARD_TEENSY41)
const char* BOARD_NAME = "Teensy 4.1";
#elif defined(BOARD_TEENSY40)
const char* BOARD_NAME = "Teensy 4.0";
#else
const char* BOARD_NAME = "Unknown Board";
#endif

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
bool loggingEnabled = false;  // Whether logging is enabled
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

// Sensor polling intervals (in milliseconds)
#define GPS_POLL_INTERVAL 200       // Poll GPS at 5Hz
#define IMU_POLL_INTERVAL 100       // Poll IMU at 10Hz
// Guidance Control Update Interval
const unsigned long GUIDANCE_UPDATE_INTERVAL_MS = 20; // 50Hz control loop
#define BARO_POLL_INTERVAL 100      // Poll barometer at 10Hz
#define ACCEL_POLL_INTERVAL 100     // Poll accelerometer at 10Hz
#define DISPLAY_INTERVAL 1000       // Update display once per second
#define GPS_CHECK_INTERVAL 10000    // Check GPS connection every 10 seconds
#define STORAGE_CHECK_INTERVAL 30000 // Check storage space every 30 seconds

// Debug control variables
bool enableDetailedOutput = false;
bool enableSystemDebug = false;      // For system messages
// enableSensorDebug is declared as volatile below
bool enableIMUDebug = false;         // For IMU data
// enableGPSDebug is declared as volatile below
bool enableBaroDebug = false;        // For barometer data
bool enableStorageDebug = false;     // For SD card and storage operations
bool enableStatusSummary = false;    // Flag to control status summary output
bool enableICMRawDebug = false;     // Flag to control ICM raw data debug output (OFF by default)
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
  convertQuaternionToEuler(icm_q0, icm_q1, icm_q2, icm_q3, 
                           logEntry.euler_roll, logEntry.euler_pitch, logEntry.euler_yaw);
  logEntry.gyro_bias_x = gyroBias[0];
  logEntry.gyro_bias_y = gyroBias[1];
  logEntry.gyro_bias_z = gyroBias[2];

  // Populate Guidance Control Data
  // Target Euler angles and PID integrals are static in guidance_control.cpp.
  // Without dedicated getter functions, we log placeholders or last known values if available.
  // For now, logging placeholders (0.0f).
  // A more complete solution would involve adding getters to guidance_control.cpp.
  guidance_get_target_euler_angles(logEntry.target_euler_roll,
                                   logEntry.target_euler_pitch,
                                   logEntry.target_euler_yaw);
  guidance_get_pid_integrals(logEntry.pid_integral_roll,
                             logEntry.pid_integral_pitch,
                             logEntry.pid_integral_yaw);
  
  guidance_get_actuator_outputs(logEntry.actuator_x, logEntry.actuator_y, logEntry.actuator_z);

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
  Serial.println(F("\nLegacy Commands (use new versions if possible):"));
  Serial.println(F("  sd                         : Toggle sensor_detail_debug"));
  Serial.println(F("  rd                         : Toggle icm_raw_debug"));
  Serial.println(F("================================="));
}

// Function to print detailed SD Card status
void printSDCardStatus() {
  Serial.println(F("\n--- SD Card Status ---"));
  checkStorageSpace(); // Ensure availableSpace is up-to-date

  Serial.print(F("Physically Present: "));
  Serial.println(sdCardPresent ? F("Yes") : F("No (or SD_DETECT_PIN not defined/functional)"));

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
    } else if (command.length() > 0) { // Changed !command.isEmpty() to command.length() > 0
       Serial.print(F("Unknown command: '"));
       Serial.print(command);
       Serial.println(F("'"));
    }
}

void setup() {
  // Wait for the Serial monitor to be opened.
  Serial.begin(115200);
  delay(500); // Give the serial port time to initialize
  
  // Important: Disable all debugging immediately at startup
  enableDetailedOutput = false;
  enableSystemDebug = false;
  enableSensorDebug = false;
  enableIMUDebug = false;
  enableGPSDebug = false;
  enableBaroDebug = false;
  enableStorageDebug = false;
  enableICMRawDebug = false;
  displayMode = false;
  
  // Initialize NeoPixel first for visual feedback
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(20, 0, 0)); // Red during startup
  pixels.setPixelColor(1, pixels.Color(20, 0, 0)); // Red during startup
  pixels.show();
  
  // Startup Tone
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

  // Setup SPI and I2C based on the selected board
  #if defined(BOARD_TEENSY41)
    // Teensy 4.1 with SDIO doesn't need explicit SPI setup for SD card
    
    // For other SPI devices (if any)
    SPI.begin();
    
    // Standard I2C setup
  Wire.begin();
  Wire.setClock(400000);
  
    Serial.println("Teensy 4.1 SPI and I2C initialized (SDIO mode for SD card)");
  #else
    // Teensy 4.0 and other boards with SPI SD card
    SPI.setCS(SD_CS_PIN);
    SPI.setMOSI(SD_MOSI_PIN);
    SPI.setMISO(SD_MISO_PIN);
    SPI.setSCK(SD_SCK_PIN);
    SPI.begin();
    
    // Standard I2C setup
    Wire.begin();
    Wire.setClock(400000);
    
    Serial.println("Teensy SPI and I2C initialized");
  #endif
  // Scan the I2C bus for devices
  scan_i2c();

  // Initialize GPS first to get accurate time
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
  
  ICM_20948_init();
  Serial.println(F("ICM-20948 initialized"));
  
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
  // Change LED to green to indicate successful initialization (regardless of SD status)
  pixels.setPixelColor(0, pixels.Color(0, 50, 0));
  pixels.show();
  
  // Barometric calibration is now done via command
  Serial.println(F("Use 'calibrate' command to perform barometric calibration with GPS"));

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
}

void loop() {
  static unsigned long lastDisplayTime = 0;
  static unsigned long lastDetailedTime = 0;
  static unsigned long lastGPSReadTime = 0;
  static unsigned long lastIMUReadTime = 0;
  static unsigned long lastBaroReadTime = 0;
  static unsigned long lastAccelReadTime = 0;
  static unsigned long lastGPSCheckTime = 0;
  static unsigned long lastStorageCheckTime = 0;
  static unsigned long lastFlushTime = 0;  // Track when we last flushed data
  static bool sensorsUpdated = false;
  static unsigned long lastGuidanceUpdateTime = 0;
  
  // Check for serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Pass command to our command handler function
    processCommand(command);
  }
  
  // Read GPS data at GPS_POLL_INTERVAL
  if (millis() - lastGPSReadTime >= GPS_POLL_INTERVAL) {
    lastGPSReadTime = millis();
    gps_read();
    sensorsUpdated = true;

    // Check for automatic calibration opportunity
    if (!baroCalibrated && pDOP < 300) {  // pDOP is stored as integer * 100
      static unsigned long lastCalibrationAttempt = 0;
      static int calibrationAttempts = 0;
      static int validReadingsCount = 0;
      static bool readingStable = false;
      
      // Only attempt calibration once every 30 seconds and limit to 3 attempts
      if (millis() - lastCalibrationAttempt > 30000 && calibrationAttempts < 3) {
        // Before starting calibration, ensure we have several good sensor readings
        if (!readingStable) {
          // Check if we have a valid barometer reading
          int result = ms5611_read();
          if (result == MS5611_READ_OK && pressure >= 800 && pressure <= 1100) {
            validReadingsCount++;
            Serial.print(F("Valid barometer reading #"));
            Serial.print(validReadingsCount);
            Serial.print(F(": Pressure = "));
            Serial.print(pressure);
            Serial.println(F(" hPa"));
            
            // Need at least 5 good readings before considering calibration
            if (validReadingsCount >= 5) {
              readingStable = true;
              Serial.println(F("Barometer readings are stable. Ready for calibration."));
            }
          } else {
            // Reset count if we get bad readings
            validReadingsCount = 0;
            Serial.println(F("Invalid barometer reading. Waiting for sensor to stabilize..."));
            delay(200);
            return; // Skip calibration for now
          }
          
          if (!readingStable) {
            return; // Skip calibration until readings are stable
          }
        }
        
        // Now we can attempt calibration
        lastCalibrationAttempt = millis();
        calibrationAttempts++;
        
        Serial.println(F("Good GPS fix detected (pDOP < 3.0), starting automatic barometric calibration..."));
        
        // Change LED to purple to indicate calibration in progress
        pixels.setPixelColor(0, pixels.Color(50, 0, 50));
        pixels.show();
        
        if (ms5611_calibrate_with_gps(30000)) {  // Wait up to 30 seconds for calibration
          Serial.println(F("Automatic barometric calibration successful!"));
          baroCalibrated = true;
          // Change LED to green to indicate success
          pixels.setPixelColor(0, pixels.Color(0, 50, 0));
          pixels.show();
          delay(1000);
        } else {
          Serial.println(F("Automatic barometric calibration timed out or failed."));
          Serial.print(F("Attempt "));
          Serial.print(calibrationAttempts);
          Serial.println(F(" of 3"));
          // Change LED to red to indicate failure
          pixels.setPixelColor(0, pixels.Color(50, 0, 0));
          pixels.show();
          delay(1000);
          
          // Reset stable reading flag to require fresh readings before next attempt
          readingStable = false;
          validReadingsCount = 0;
        }
      }
    }
  }
  
  // Read barometer data at BARO_POLL_INTERVAL
  if (millis() - lastBaroReadTime >= BARO_POLL_INTERVAL) {
    lastBaroReadTime = millis();
    int result = ms5611_read();
    if (result == MS5611_READ_OK) {
      sensorsUpdated = true;
    }
  }
  
  // Read IMU data at 10Hz
  if (millis() - lastIMUReadTime >= 100) {
    lastIMUReadTime = millis();
    ICM_20948_read();
    sensorsUpdated = true;
  }
  
  // Read accelerometer data at 10Hz
  if (millis() - lastAccelReadTime >= 100) {
    lastAccelReadTime = millis();
    kx134_read();
    sensorsUpdated = true;
  }
  
  
  #if DISABLE_SDCARD_LOGGING
  // Logging disabled by configuration, ensure flags are false.
  // The actual sdCardAvailable = false etc. is handled in setup.
  // Removing the early return that prevented serial CSV output.
  // sdCardPresent = false; // These are already set in setup if this macro is defined
  // sdCardMounted = false;
  // sdCardAvailable = false;
  // return; // <--- REMOVED THIS LINE
#endif
  // Log data immediately after any sensor update
  if (sensorsUpdated) {
    WriteLogData(true);
    sensorsUpdated = false;
  }
  
  // Periodically check GPS connection - simplified approach
  if (millis() - lastGPSCheckTime >= GPS_CHECK_INTERVAL) {
    lastGPSCheckTime = millis();
    checkGPSConnection();
  }
  
  // Periodically check storage space
  if (millis() - lastStorageCheckTime >= STORAGE_CHECK_INTERVAL) {
    lastStorageCheckTime = millis();
    checkStorageSpace();
  }
  
  // Periodically flush data to SD card (every 10 seconds)
  if (millis() - lastFlushTime >= 10000) {  // 10 seconds
    lastFlushTime = millis();
    
    // Flush SD card data
    if (sdCardAvailable && LogDataFile) {
      LogDataFile.flush();
      // Only print flush message if storage debugging is enabled
      if (enableStorageDebug) {
        Serial.println(F("SD card data flushed"));
      }
    }
  }
  
  // Print status summary once per second, but only if enabled
  if (enableStatusSummary && millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
    lastDisplayTime = millis();
    printStatusSummary();
  }
  
  // Print detailed data less frequently
  if (displayMode && millis() - lastDetailedTime >= 5000) {
    lastDetailedTime = millis();
    ICM_20948_print();
  }

  // Update dynamic guidance targets based on flight logic
  update_guidance_targets();

  // --- Guidance Control System Update ---
  if (millis() - lastGuidanceUpdateTime >= GUIDANCE_UPDATE_INTERVAL_MS) {
      float deltat_guidance = (float)(millis() - lastGuidanceUpdateTime) / 1000.0f;
      lastGuidanceUpdateTime = millis();

      // 1. Get current orientation (Euler angles)
      float roll_rad, pitch_rad, yaw_rad;
      // Assuming icm_q0, icm_q1, icm_q2, icm_q3 are globally accessible after AHRS update
      // These are extern declared in icm_20948_functions.h and defined in icm_20948_functions.cpp
      convertQuaternionToEuler(icm_q0, icm_q1, icm_q2, icm_q3, roll_rad, pitch_rad, yaw_rad);

      // 2. Get current angular velocities (calibrated)
      float roll_rate_radps, pitch_rate_radps, yaw_rate_radps;
      float calibrated_gyro_data[3];
      ICM_20948_get_calibrated_gyro(calibrated_gyro_data); // Function from icm_20948_functions.cpp
      roll_rate_radps = calibrated_gyro_data[0];  // Assuming X-axis from sensor is roll axis
      pitch_rate_radps = calibrated_gyro_data[1]; // Assuming Y-axis from sensor is pitch axis
      yaw_rate_radps = calibrated_gyro_data[2];   // Assuming Z-axis from sensor is yaw axis
                                              // TODO: Verify axis mapping based on actual sensor orientation on rocket

      // 3. Update guidance controller
      // Target orientation is currently set by guidance_set_target_orientation_euler().
      // For now, it will use the default (0,0,0) or whatever was last set via command.
      guidance_update(roll_rad, pitch_rad, yaw_rad,
                      roll_rate_radps, pitch_rate_radps, yaw_rate_radps,
                      deltat_guidance);

      // 4. Get actuator outputs
      float actuator_x, actuator_y, actuator_z;
      guidance_get_actuator_outputs(actuator_x, actuator_y, actuator_z);

      // 5. Drive Servos
      // Map PID outputs (-1.0 to 1.0 from PID_OUTPUT_MIN/MAX) to servo angles (0 to 180 degrees)
      float pitch_angle = map_float(actuator_x, PID_OUTPUT_MIN, PID_OUTPUT_MAX, 0, 180);
      float roll_angle  = map_float(actuator_y, PID_OUTPUT_MIN, PID_OUTPUT_MAX, 0, 180);
      float yaw_angle   = map_float(actuator_z, PID_OUTPUT_MIN, PID_OUTPUT_MAX, 0, 180);

      // Clamp angles to ensure they are within servo limits (0-180 typical)
      pitch_angle = constrain(pitch_angle, 0, 180);
      roll_angle  = constrain(roll_angle,  0, 180);
      yaw_angle   = constrain(yaw_angle,   0, 180);
      
      // Write to servos
      servo_pitch.write(static_cast<int>(pitch_angle));
      servo_roll.write(static_cast<int>(roll_angle));
      servo_yaw.write(static_cast<int>(yaw_angle));

      // For now, add a debug print if SystemDebug is enabled
      if (enableSystemDebug) { // Using existing SystemDebug flag for this
          static unsigned long lastGuidanceDebugPrintTime = 0;
          if (millis() - lastGuidanceDebugPrintTime > 500) { // Print every 500ms
              lastGuidanceDebugPrintTime = millis();
              Serial.print("Guidance Out - X(Pitch): "); Serial.print(actuator_x, 2);
              Serial.print(" Y(Roll): "); Serial.print(actuator_y, 2);
              Serial.print(" Z(Yaw): "); Serial.println(actuator_z, 2);
              
              Serial.print("Guidance In  - R: "); Serial.print(roll_rad * (180.0f/PI), 1);
              Serial.print(" P: "); Serial.print(pitch_rad * (180.0f/PI), 1);
              Serial.print(" Y: "); Serial.println(yaw_rad * (180.0f/PI), 1);
          }
      }
  }
}



