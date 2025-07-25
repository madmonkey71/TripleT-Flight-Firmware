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
#define TRIPLET_FLIGHT_VERSION 0.51

// Define the board type - Teensy 4.1 only
#ifndef BOARD_TEENSY41
#if defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41)
#define BOARD_TEENSY41
#endif
#endif

// Now include the GPS and sensor functions
#include "constants.h"        // Include constants first to avoid macro conflicts
#include "command_processor.h" // Include command processor
#include "debug_flags.h"      // Include debug flags structure
#include "flight_context.h"   // Include flight context structure
#include "gps_config.h"
#include "gps_functions.h"  // Include GPS functions header
// Forward declare initSDCard if its definition is after its first use (e.g. in setup calling sdCardAvailable = initSDCard())
// bool initSDCard(SdFat& sd_obj, bool& sdCardMounted_out, bool& sdCardPresent_out);
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
// #include "sensor_fusion.h"   // REMOVED as sensor_fusion.h and .cpp were deleted

// Define variables declared as extern in utility_functions.h
String g_FileDateString = ""; // Assuming this should also be global
String g_LogDataString = "";  // Assuming this should also be global
unsigned long g_currentTime = 0;  // For timestamp
bool g_baroCalibrated = false;  // For barometric calibration status
float g_launchAltitude = 0.0f;
float g_maxAltitudeReached = 0.0f;
float g_currentAltitude = 0.0f;
bool g_kx134_initialized_ok = false;
bool g_icm20948_ready = false;
extern bool ms5611_initialized_ok; // Definition is likely in ms5611_functions.cpp, so no 'g_' here

// Global variable for dynamic main deployment altitude (Task Step 2)
float g_main_deploy_altitude_m_agl = 0.0f;

// Global variable for battery voltage
float g_battery_voltage = 0.0f;

// Global variable for last recorded error code
#include "error_codes.h" // Include the error code definitions
ErrorCode_t g_last_error_code = NO_ERROR;

const char* BOARD_NAME = "Teensy 4.1"; // This is a const, often uppercased, g_ prefix is optional by convention

// Orientation Filter Selection - Always use Kalman filter
bool g_useKalmanFilter = true; // Always true - Kalman is the only orientation filter
float g_kalmanRoll = 0.0f;
float g_kalmanPitch = 0.0f;
float g_kalmanYaw = 0.0f;
bool g_usingKX134ForKalman = false; // Initialize to false, default to ICM for Kalman

// External declarations for sensor data
extern float pressure; // Definition likely in sensor_functions.cpp
extern float temperature; // Definition likely in sensor_functions.cpp
// extern float kx134_accel[3]; // This is often a global in the KX134 driver itself.

// Global variable definitions moved to avoid duplicates (already defined above)

// Add debug flag definitions
volatile bool enableGPSDebug = false;
volatile bool enableSensorDebug = false;
volatile bool enableICMRawDebug = false;

// Add variables needed by state_management.cpp
FlightState currentFlightState = STARTUP;  // Alias for g_currentFlightState
float launchAltitude = 0.0f;               // Alias for g_launchAltitude  
float maxAltitudeReached = 0.0f;           // Alias for g_maxAltitudeReached
float currentAltitude = 0.0f;              // Alias for g_currentAltitude

// Add variable needed by ms5611_functions.cpp
bool baroCalibrated = false;               // Alias for g_baroCalibrated

// Define sensor objects
SparkFun_KX134 g_kx134Accel;  // Add KX134 accelerometer object definition

// Servo objects for actuators
#include <PWMServo.h> // Ensure it's included (already there)
PWMServo g_servo_pitch;
PWMServo g_servo_roll;
PWMServo g_servo_yaw;

// NeoPixel object
Adafruit_NeoPixel g_pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// MS5611 sensor object
MS5611 g_ms5611Sensor;

// Storage configuration
// #define SD_CARD_MIN_FREE_SPACE 1024 * 1024  // 1MB minimum free space - REMOVED, defined in config.h
#define EXTERNAL_FLASH_MIN_FREE_SPACE 1024 * 1024  // 1MB minimum free space

// Storage variables
SdFat g_SD;
bool g_sdCardAvailable = false;
bool g_sdCardPresent = false;   // Physical presence of SD card
bool g_sdCardMounted = false;   // Whether SD card is mounted
bool g_loggingEnabled = true;  // Whether logging is enabled

// Flight State Variables
FlightState g_currentFlightState = STARTUP;
FlightState g_previousFlightState = STARTUP;
unsigned long g_stateEntryTime = 0; // To store the time when the current state was entered

uint64_t g_availableSpace = 0;  // Available space on SD card in bytes
bool g_flashAvailable = false;
FsFile g_root; // If these are truly global and not function-local statics
FsFile g_entry; // If these are truly global and not function-local statics
FsFile g_LogDataFile;
FsFile g_IdleDataFile;

// Global storage variables
uint64_t g_totalSpace; // If global
uint64_t g_usedSpace;  // If global

// Sequence number for logging
static unsigned long g_logSequenceNumber = 0; // static global

// Additional global variables for data logging
const int FLASH_CHIP_SELECT = 5; // Choose an appropriate pin for flash CS (usually a const, g_ is optional)
char g_logFileName[32] = ""; // To store the current log file name

// Guidance Control Update Interval
const unsigned long GUIDANCE_UPDATE_INTERVAL_MS = 20; // 50Hz control loop

// Instantiate global debug flags struct
// Forward declare SdFat and FsFile if they are used in function signatures before full definition/include
// class SdFat;
// class FsFile;
// It's generally better to ensure includes are ordered correctly. flight_context.h includes SdFat.h.

DebugFlags g_debugFlags = {
  .enableSerialCSV = false,
  .enableSystemDebug = true, // ON by default for debugging ERROR state issues
  .enableIMUDebug = false,
  .enableGPSDebug = false,
  .enableBaroDebug = false,
  .enableStorageDebug = false,
  .enableICMRawDebug = true, // ON by default for tuning
  .enableStatusSummary = false,
  .displayMode = false,
  .enableSensorDebug = true, // ON by default for tuning (legacy 'sd' command)
  .enableDetailedOutput = false
};

// Function to check available storage space
void checkStorageSpace(SdFat& sd_obj_ref, uint64_t& availableSpace_out_ref) {
  if (g_sdCardAvailable && sd_obj_ref.vol()) {
    uint64_t freeSpace = sd_obj_ref.vol()->freeClusterCount() * sd_obj_ref.vol()->bytesPerCluster();
    availableSpace_out_ref = freeSpace; // Update passed-in reference
    
    if (freeSpace < SD_CARD_MIN_FREE_SPACE) {
      Serial.println(F("WARNING: SD card space low!"));
      Serial.print(F("Available space: "));
      Serial.print(freeSpace / (1024ULL * 1024ULL));
      Serial.println(F(" MB"));
    }
  } else {
    availableSpace_out_ref = 0; // Set to 0 if card not available or no volume
  }
}

// Function to create a new log file with timestamp in name
bool createNewLogFile(SdFat& sd_obj, SFE_UBLOX_GNSS& gnss, FsFile& logFile_obj_ref, char* logFileName_out_buf, size_t logFileName_out_buf_size) {
  // Assuming g_sdCardAvailable is checked by the caller (attemptToStartLogging) before this function is called.

  // Close any previously open log file
  if (logFile_obj_ref) {
    logFile_obj_ref.flush();
    logFile_obj_ref.close();
  }
  
  char local_fileName_buf[64]; // Local buffer for sprintf
  
  // If GPS has fix, use date/time for filename
  if (gnss.getFixType() > 0) {
    int year;
    byte month, day, hour, minute, second;
    // Assuming getGPSDateTime is a global utility or part of gnss object
    // For now, assume getGPSDateTime is a global utility that can access gnss implicitly or takes it.
    // If getGPSDateTime is specific to the global myGNSS, this part needs care.
    // Let's assume getGPSDateTime can work with the passed 'gnss' reference or is general.
    // This might require getGPSDateTime(gnss, year, month, day, hour, minute, second);
    getGPSDateTime(year, month, day, hour, minute, second); // If this uses global myGNSS, it won't use the passed 'gnss'
    
    sprintf(local_fileName_buf, "DATA_%04d%02d%02d_%02d%02d%02d.csv",
      year, month, day, hour, minute, second);
  } else {
    // No GPS fix, use millis()
    sprintf(local_fileName_buf, "LOG_%lu.csv", millis());
  }
  
  Serial.print(F("Creating log file: "));
  Serial.println(local_fileName_buf);
  
  // Open the file using the passed FsFile reference
  if (!logFile_obj_ref.open(local_fileName_buf, O_RDWR | O_CREAT | O_EXCL)) {
    Serial.println(F("Error: Failed to create new log file. SD card might be full, unformatted, or corrupted. Please check the SD card."));
    g_last_error_code = LOG_FILE_CREATE_FAIL;
    // Do NOT modify global loggingEnabled here. Caller (attemptToStartLogging) will do that.
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
  logFile_obj_ref.println(headerBuffer);
  
  // Flush to ensure header is written
  logFile_obj_ref.flush();
  
  // Copy filename to output buffer
  strncpy(logFileName_out_buf, local_fileName_buf, logFileName_out_buf_size - 1);
  logFileName_out_buf[logFileName_out_buf_size - 1] = '\0'; // Ensure null termination
  
  // Do NOT modify global loggingEnabled here. Caller (attemptToStartLogging) will do that.
  Serial.println(F("Log file created successfully"));
  
  return true;
}

// Function to safely close all files
// This function still uses the global LogDataFile. If it needs to be more generic,
// it should take an FsFile reference.
void closeAllFiles(FsFile& logFile_obj_to_close) {
  // Close any open log files
  if (logFile_obj_to_close.isOpen()) {
    logFile_obj_to_close.close();
    Serial.println(F("Log file closed."));
  }
}

// Function to handle system shutdown (MOVED TO command_processor.cpp)
// void prepareForShutdown() {
//   Serial.println(F("Preparing for shutdown..."));
//
//   // Close all open files
//   closeAllFiles();
//
//   // Set LED to indicate shutdown
//   pixels.setPixelColor(0, pixels.Color(0, 0, 50)); // Blue during shutdown
//   pixels.show();
//
//   // Final beep
//   tone(BUZZER_PIN, 1000); delay(100); noTone(BUZZER_PIN);
//
//   Serial.println(F("System shutdown complete"));
//   while (true) {
//     delay(5000);
//   }
// }

void WriteLogData(bool forceLog) {
  static unsigned long lastLogTime = 0;
  
  // Only log at specified intervals or when forced
  if (!forceLog && millis() - lastLogTime < 200) {
    return;
  }
  lastLogTime = millis();
  
  // --- Populate LogData Struct --- 
  LogData logEntry;
  logEntry.seqNum = g_logSequenceNumber++; // Increment and assign sequence number
  logEntry.timestamp = millis(); // Use current millis() for timestamp
  logEntry.flightState = static_cast<uint8_t>(g_currentFlightState); // Add current flight state
  logEntry.fixType = GPS_fixType; // These GPS_ variables are assumed to be populated by gps_read() from myGNSS
  logEntry.sats = SIV;            // and are typically global or static within gps_functions.cpp.
                                  // If they are truly global in *this* file, they need g_ prefix.
                                  // For now, assuming they are from gps_functions.cpp context.
  logEntry.latitude = GPS_latitude;   // Same assumption as above for GPS_fixType
  logEntry.longitude = GPS_longitude; // Same assumption
  logEntry.altitude = GPS_altitude;   // Same assumption
  logEntry.altitudeMSL = GPS_altitudeMSL; // Same assumption
  // Calculate and add raw and calibrated altitude
  if (pressure > 0) { // `pressure` is extern, likely from ms5611_functions.cpp
    logEntry.raw_altitude = 44330.0 * (1.0 - pow(pressure / STANDARD_SEA_LEVEL_PRESSURE, 0.1903));
    if (g_baroCalibrated) {
      logEntry.calibrated_altitude = logEntry.raw_altitude + baro_altitude_offset; // baro_altitude_offset is another global, needs g_ if defined here
    } else {
      logEntry.calibrated_altitude = logEntry.raw_altitude; // If not calibrated, log raw altitude as calibrated altitude
    }
  } else {
    logEntry.raw_altitude = 0.0f; // Log 0.0 if pressure is not positive
    logEntry.calibrated_altitude = 0.0f; // Log 0.0 if pressure is not positive
  }
  // --------------------------------
  logEntry.speed = GPS_speed;         // Same assumption
  logEntry.heading = GPS_heading;     // Same assumption
  logEntry.pDOP = pDOP;               // Same assumption
  logEntry.rtk = RTK;                 // Same assumption
  logEntry.pressure = pressure;       // `pressure` is extern
  logEntry.temperature = temperature; // `temperature` is extern
  memcpy(logEntry.kx134_accel, kx134_accel, sizeof(kx134_accel)); // kx134_accel is global from kx134_functions.cpp (needs g_ if defined here)
  memcpy(logEntry.icm_accel, icm_accel, sizeof(icm_accel));      // icm_accel is global from icm_20948_functions.cpp (needs g_ if defined here)
  memcpy(logEntry.icm_gyro, icm_gyro, sizeof(icm_gyro));        // icm_gyro is global from icm_20948_functions.cpp (needs g_ if defined here)
  memcpy(logEntry.icm_mag, icm_mag, sizeof(icm_mag));          // icm_mag is global from icm_20948_functions.cpp (needs g_ if defined here)
  logEntry.icm_temp = icm_temp;                                 // icm_temp is global from icm_20948_functions.cpp (needs g_ if defined here)

  // Always use Kalman filter orientation data
  logEntry.euler_roll = g_kalmanRoll;
  logEntry.euler_pitch = g_kalmanPitch;
  logEntry.euler_yaw = g_kalmanYaw;
  
  // Convert Kalman Euler angles to quaternion for logging
  convertEulerToQuaternion(g_kalmanRoll, g_kalmanPitch, g_kalmanYaw,
                           logEntry.q0, logEntry.q1, logEntry.q2, logEntry.q3);
  // Get gyro bias from ICM (still useful for diagnostics)
  logEntry.gyro_bias_x = gyroBias[0];
  logEntry.gyro_bias_y = gyroBias[1];
  logEntry.gyro_bias_z = gyroBias[2];

  // Battery Voltage
  logEntry.battery_voltage = g_battery_voltage;

  // Last Error Code
  logEntry.last_error_code = static_cast<uint8_t>(g_last_error_code);

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
  if (g_debugFlags.enableSerialCSV) {
    // Convert struct to string and print
    Serial.println(logDataToString(logEntry)); 
  }


  // --- SD Card Logging Logic ---
  static unsigned long lastLogWriteErrorTime = 0;
  static unsigned long lastRateLimitedLogStatusMsgTime = 0;
  const unsigned long LOG_WRITE_RETRY_DELAY_MS = 10000; // 10 seconds
  const unsigned long RATE_LIMITED_LOG_STATUS_INTERVAL_MS = 30000; // 30 seconds

  // If SD card is not available or logging is generally disabled, return early.
  if (!g_sdCardAvailable || !g_loggingEnabled) {
    if (millis() - lastRateLimitedLogStatusMsgTime > RATE_LIMITED_LOG_STATUS_INTERVAL_MS) {
      Serial.println(F("INFO: Logging to SD card is currently disabled (SD not available or loggingEnabled=false)."));
      lastRateLimitedLogStatusMsgTime = millis();
    }
    return;
  }

  // Check if the log file is open. If not, try to create/reopen it.
  if (!g_LogDataFile || !g_LogDataFile.isOpen()) {
    if (millis() - lastLogWriteErrorTime > LOG_WRITE_RETRY_DELAY_MS) { // Rate limit attempts to reopen
      Serial.println(F("INFO: Log file is not open. Attempting to create a new log file..."));
      // Call the new createNewLogFile, passing global objects/buffers
      if (createNewLogFile(g_SD, myGNSS, g_LogDataFile, g_logFileName, sizeof(g_logFileName))) {
        g_loggingEnabled = true; // Set global on success
        Serial.println(F("INFO: New log file created successfully."));
      } else {
        g_loggingEnabled = false; // Ensure this is false on failure
        Serial.println(F("ERROR: Failed to create a new log file. Logging will be suspended for a while."));
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
  
  // At this point, g_LogDataFile should be open. Attempt to write the log entry.
  String logString = logDataToString(logEntry);
  if (!g_LogDataFile.println(logString)) {
    Serial.println(F("ERROR: Failed to write data to log file. Current log file might be closed or corrupted."));
    g_last_error_code = SD_CARD_WRITE_FAIL;
    lastLogWriteErrorTime = millis(); // Record time of write error

    // Attempt to recover: Close current file and try to open a new one.
    if (g_LogDataFile.isOpen()) {
      g_LogDataFile.close();
      Serial.println(F("INFO: Closed current log file due to write error."));
    }
    
    // Attempt to create a new log file immediately
    if (createNewLogFile(g_SD, myGNSS, g_LogDataFile, g_logFileName, sizeof(g_logFileName))) {
      g_loggingEnabled = true; // Set global on success
      Serial.println(F("INFO: Successfully created a new log file. Retrying write for the current log entry."));
      // Retry writing the same log entry to the new file
      if (!g_LogDataFile.println(logString)) {
        Serial.println(F("ERROR: Failed to write data to the *new* log file. Logging will be suspended."));
        g_loggingEnabled = false; // Critical failure, disable logging
      } else {
        Serial.println(F("INFO: Successfully wrote data to the new log file after recovery."));
        // Consider flushing immediately after successful recovery write
        g_LogDataFile.flush();
      }
    } else {
      g_loggingEnabled = false; // Ensure this is false on failure
      Serial.println(F("ERROR: Failed to create a new log file after write error. Logging will be suspended."));
    }
    return; // Return whether recovery was successful or not, to avoid normal flush logic on a failed write.
  }
  
  // Flush every 10 writes to reduce card wear while ensuring data is written
  static uint8_t flushCounter = 0;
  if (++flushCounter >= 10) {
    g_LogDataFile.flush();
    flushCounter = 0;
  }
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
  if (g_debugFlags.enableGPSDebug) {
    Serial.println("\n--- GPS Status ---");
    if (GPS_fixType > 0) { // Assuming GPS_fixType etc. are updated by gps_read() from myGNSS
      Serial.printf("Position: %.6f, %.6f, Alt: %.2fm, Sat: %d, Fix: %d\n", 
                  GPS_latitude / 10000000.0, GPS_longitude / 10000000.0, GPS_altitude / 1000.0, SIV, GPS_fixType);
      Serial.printf("Speed: %.2f km/h, Course: %.2f°\n", GPS_speed / 1000.0 * 3.6, GPS_heading / 100000.0);
    } else {
      Serial.println("No GPS fix");
    }
  }
  
  // IMU section
  if (g_debugFlags.enableIMUDebug) {
    Serial.println("\n--- IMU Status ---");
    Serial.printf("Temp: %.1f°C, Motion: %s\n", 
                icm_temp, isStationary ? "STATIONARY" : "MOVING"); // Assuming icm_temp and isStationary are from sensor reads
  }
  
  // Barometer section
  if (g_debugFlags.enableBaroDebug) {
    Serial.println("\n--- Barometer ---");
    if (g_baroCalibrated) {
      // The pressure value is in hPa but the altitude formula expects Pa
      // So we need to multiply by 100 to convert hPa to Pa
      float pressurePa = pressure * 100.0; // `pressure` is extern
      float raw_altitude = 44330.0 * (1.0 - pow(pressurePa / 101325.0, 0.1903));
      float calibrated_altitude = raw_altitude + baro_altitude_offset; // baro_altitude_offset is global
      
      Serial.printf("Pressure: %.2f hPa, Temp: %.2f°C\n", pressure, temperature); // pressure, temp are extern
      Serial.printf("Altitude: Raw=%.2fm, Calibrated=%.2fm\n", raw_altitude, calibrated_altitude);
    } else {
      Serial.println("Not calibrated");
    }
  }
  
  // Storage section
  if (g_debugFlags.enableStorageDebug) {
    Serial.println("\n--- Storage ---");
    Serial.printf("SD Card: %s, Space: %s\n", 
                g_sdCardPresent ? "Present" : "Not found",
                g_sdCardMounted ? String(g_availableSpace / 1024) + "KB free" : "Not mounted");
    Serial.printf("Logging: %s\n", g_loggingEnabled ? "Enabled" : "Disabled");
  }
  
  // Debug status
  Serial.println("\n--- Debug Status ---");
  Serial.printf("System: %s, IMU: %s, GPS: %s, Baro: %s\n", 
              g_debugFlags.enableSystemDebug ? "ON" : "OFF",
              g_debugFlags.enableIMUDebug ? "ON" : "OFF",
              g_debugFlags.enableGPSDebug ? "ON" : "OFF",
              g_debugFlags.enableBaroDebug ? "ON" : "OFF");
  Serial.printf("Storage: %s, ICM Raw: %s\n",
              g_debugFlags.enableStorageDebug ? "ON" : "OFF",
              g_debugFlags.enableICMRawDebug ? "ON" : "OFF");
  Serial.printf("Serial CSV: %s\n",
              g_debugFlags.enableSerialCSV ? "ON" : "OFF");

  Serial.println("=====================\n");
}

// Functions moved to command_processor.cpp:
// - printHelpMessage
// - printSDCardStatus
// - attemptToStartLogging
// - printStorageStatistics
// - toggleDebugFlag
// - performCalibration
// - printSystemStatus
// - prepareForShutdown
// - processCommand
// - setOrientationFilter (part of processCommand)
// - getOrientationFilterStatus (part of processCommand)

void setup() {
  // Wait for the Serial monitor to be opened.
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    // Wait for Serial to connect, but don't wait forever
  }

  Serial.println(F("TripleT Flight Firmware Starting..."));
  Serial.print(F("Version: "));
  Serial.println(TRIPLET_FLIGHT_VERSION);
  Serial.print(F("Board: "));
  Serial.println(BOARD_NAME);

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock

  // Initialize NeoPixel
  initNeoPixel(g_pixels);

  // Initialize pyro channels (safety first - ensure they're off)
  pinMode(PYRO_CHANNEL_1, OUTPUT);
  digitalWrite(PYRO_CHANNEL_1, LOW);
  pinMode(PYRO_CHANNEL_2, OUTPUT);
  digitalWrite(PYRO_CHANNEL_2, LOW);

  // Initialize servo objects
  g_servo_pitch.attach(ACTUATOR_PITCH_PIN);
  g_servo_roll.attach(ACTUATOR_ROLL_PIN);
  g_servo_yaw.attach(ACTUATOR_YAW_PIN);

  // Set servos to default positions
  g_servo_pitch.write(SERVO_DEFAULT_ANGLE);
  g_servo_roll.write(SERVO_DEFAULT_ANGLE);
  g_servo_yaw.write(SERVO_DEFAULT_ANGLE);

  // Initialize buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Load flight state from EEPROM (for power-loss recovery)
  recoverFromPowerLoss(); // This sets g_currentFlightState appropriately

  // Initialize SD card
  g_sdCardAvailable = initSDCard(g_SD, g_sdCardMounted, g_sdCardPresent);
  if (g_sdCardAvailable) {
    checkStorageSpace(g_SD, g_availableSpace);
    Serial.println(F("SD Card initialization successful."));
  } else {
    Serial.println(F("SD Card initialization failed."));
  }

  // Initialize GPS
  gps_init(); // This should set up the GPS module

  // Initialize sensors
  ms5611_init(); // This sets ms5611_initialized_ok
  ICM_20948_init(); // This sets g_icm20948_ready
  if (USE_KX134) {
    g_kx134_initialized_ok = kx134_init(); // This sets g_kx134_initialized_ok
  }

  // Initialize Kalman filter
  kalman_init(0.0f, 0.0f, 0.0f); // Initialize with zero initial orientation

  // Initialize guidance system
  guidance_init(); // Reset PID controllers and guidance state

  // Create initial log file
  if (g_sdCardAvailable) {
    createNewLogFile(g_SD, myGNSS, g_LogDataFile, g_logFileName, sizeof(g_logFileName));
  }

  Serial.println(F("Hardware initialization complete."));
  
  // Print initial sensor status
  if (g_debugFlags.enableSystemDebug) {
    Serial.println(F("=== Initial Sensor Status ==="));
    Serial.print(F("MS5611 Barometer: ")); Serial.println(ms5611_initialized_ok ? F("OK") : F("FAILED"));
    Serial.print(F("ICM-20948 IMU: ")); Serial.println(g_icm20948_ready ? F("OK") : F("FAILED"));
    Serial.print(F("KX134 Accelerometer: ")); Serial.println(g_kx134_initialized_ok ? F("OK") : F("FAILED"));
    Serial.print(F("SD Card: ")); Serial.println(g_sdCardAvailable ? F("OK") : F("FAILED"));
    Serial.print(F("Initial Flight State: ")); Serial.println(getStateName(g_currentFlightState));
    Serial.println(F("============================="));
  }

  Serial.println(F("Setup complete. Starting main loop..."));
}

// Function to handle initial state management after setup
void handleInitialStateManagement() {
  static bool initialStateHandled = false;
  if (initialStateHandled) {
    return; // Only run this logic once
  }

  // Check system health using the same criteria as the setup function used to
  bool systemHealthy = true;
  
  if (!g_sdCardAvailable && g_loggingEnabled) {
    Serial.println(F("INIT: Logging enabled but SD card not available. System unhealthy."));
    systemHealthy = false;
  }
  
  if (!g_kx134_initialized_ok && !g_icm20948_ready) {
    Serial.println(F("INIT: No IMU available (both KX134 and ICM20948 failed). System unhealthy."));
    systemHealthy = false;
  }
  
  if (!ms5611_initialized_ok) {
    // Serial.println(F("INIT: MS5611 barometer not initialized. System unhealthy.")); // Keep systemHealthy = true
    // systemHealthy = false; // Allow proceeding to CALIBRATION/PAD_IDLE with a warning
    if (g_debugFlags.enableSystemDebug) { // Still print a warning if debug is enabled
        Serial.println(F("INIT_WARNING: MS5611 barometer not initialized. Functionality will be limited."));
    }
  }

  if (g_debugFlags.enableSystemDebug) {
    Serial.println(F("=== Initial State Management ==="));
    Serial.print(F("System Health: ")); Serial.println(systemHealthy ? F("HEALTHY") : F("UNHEALTHY"));
    Serial.print(F("Current State: ")); Serial.println(getStateName(g_currentFlightState));
  }

  // Handle state transitions based on current state and system health
  if (g_currentFlightState == STARTUP) {
    if (systemHealthy) {
      // Check if barometer is already calibrated to skip CALIBRATION state
      if (g_baroCalibrated) {
        Serial.println(F("Fresh start, system healthy and barometer calibrated, proceeding to PAD_IDLE state."));
        g_currentFlightState = PAD_IDLE;
        g_stateEntryTime = millis();
      } else {
        Serial.println(F("Fresh start, system healthy, proceeding to CALIBRATION state."));
        g_currentFlightState = CALIBRATION;
        g_stateEntryTime = millis();
      }
    } else {
      Serial.println(F("Fresh start but system unhealthy, transitioning to ERROR state."));
      g_last_error_code = STATE_TRANSITION_INVALID_HEALTH; // Or a more specific init error if identifiable here
      g_currentFlightState = ERROR;
      g_stateEntryTime = millis();
      saveStateToEEPROM(); // Save state
      WriteLogData(true);  // Log error immediately
    }
  } else if (g_currentFlightState == ERROR && systemHealthy) {
    // Check if barometer is already calibrated to skip CALIBRATION state
    if (g_baroCalibrated) {
      Serial.println(F("ERROR state recovered, all systems healthy and barometer calibrated, transitioning to PAD_IDLE."));
      g_currentFlightState = PAD_IDLE;
      g_stateEntryTime = millis();
    } else {
      Serial.println(F("ERROR state recovered and all systems are healthy. Automatically clearing error and transitioning to CALIBRATION."));
      g_currentFlightState = CALIBRATION;
      g_stateEntryTime = millis();
    }
    Serial.println(F("ERROR state cleared - starting grace period for health checks"));
    saveStateToEEPROM();
  } else if (!systemHealthy && g_currentFlightState != ERROR) {
    Serial.println(F("System became unhealthy during initialization, transitioning to ERROR state."));
    g_last_error_code = STATE_TRANSITION_INVALID_HEALTH; // Or a more specific init error
    g_currentFlightState = ERROR;
    g_stateEntryTime = millis();
    saveStateToEEPROM(); // Save state
    WriteLogData(true);  // Log error immediately
  }

  if (g_debugFlags.enableSystemDebug) {
    Serial.print(F("Final State: ")); Serial.println(getStateName(g_currentFlightState));
    Serial.println(F("=============================="));
  }

  initialStateHandled = true;
}

void loop() {
  // Handle initial state management (runs once after setup)
  handleInitialStateManagement();

  // --- Timekeeping and Sensor Update Flags ---
  static unsigned long lastGPSReadTime = 0;
  static unsigned long lastIMUReadTime = 0;   // For ICM20948 IMU
  static unsigned long lastBaroReadTime = 0;
  static unsigned long lastKalmanUpdateTime = 0; // For Kalman filter dt calculation
  static unsigned long lastBatteryReadTime = 0; // For battery voltage reading

  bool sensorsUpdatedThisCycle = false; // Track if any sensor was updated this cycle

  // --- Serial Command Processing ---
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Populate SystemStatusContext
    SystemStatusContext statusCtx = {
        g_sdCardAvailable,
        g_sdCardMounted,
        g_sdCardPresent,
        g_loggingEnabled,
        g_logFileName,
        g_availableSpace,
        g_flashAvailable,
        g_baroCalibrated,
        g_icm20948_ready,
        ms5611_initialized_ok, // extern, not prefixed with g_ unless defined here
        g_kx134_initialized_ok,
        &g_useKalmanFilter,
        myGNSS,
        g_ms5611Sensor       // Assuming g_ms5611Sensor is the global instance
    };

    processCommand(command,
                   g_currentFlightState,
                   g_previousFlightState,
                   g_stateEntryTime,
                   statusCtx,
                   g_debugFlags,
                   g_SD,
                   myGNSS,
                   g_LogDataFile,
                   g_logFileName,
                   sizeof(g_logFileName),
                   g_sdCardAvailable,
                   g_loggingEnabled,
                   g_sdCardMounted,
                   g_sdCardPresent,
                   g_availableSpace,
                   g_pixels,
                   g_baroCalibrated,
                   g_ms5611Sensor
                   );
    command = ""; // Clear the command string after processing
  }

  // --- Sensor Data Reads ---
  if (millis() - lastGPSReadTime >= GPS_POLL_INTERVAL) {
    lastGPSReadTime = millis();
    gps_read(); // Existing GPS read function
    sensorsUpdatedThisCycle = true;
    // NOTE: Old automatic barometer calibration logic based on pDOP < 300 from loop() should be REMOVED.
    // Calibration is now handled by setup() and the CALIBRATION state in ProcessFlightState.
  }

  if (millis() - lastBatteryReadTime >= BATTERY_VOLTAGE_READ_INTERVAL_MS) {
    lastBatteryReadTime = millis();
    #if ENABLE_BATTERY_MONITORING == 1
      g_battery_voltage = read_battery_voltage();
      // sensorsUpdatedThisCycle = true; // Optionally mark as updated if voltage is critical for immediate logging
                                      // For now, it will be logged with other sensor data
    #endif
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
    
    // Read KX134 if available
    if (g_kx134_initialized_ok) {
      kx134_read(); // This function should use g_kx134Accel if it's the global object
    }
    
    sensorsUpdatedThisCycle = true;

    // --- Kalman Filter Processing ---
    if (g_useKalmanFilter && g_icm20948_ready) {
        float dt_kalman = 0.0f;
        unsigned long currentTimeMillis = millis(); // Cache current time

        if (lastKalmanUpdateTime > 0) { // Ensure lastKalmanUpdateTime has been initialized after the first run
            dt_kalman = (currentTimeMillis - lastKalmanUpdateTime) / 1000.0f;
        }
        lastKalmanUpdateTime = currentTimeMillis;
        
        // Sensor switching logic for Kalman filter accelerometer
        float current_accel_for_kalman[3]; // Temporary array to hold accel data for Kalman
        bool kx134_was_used = g_usingKX134ForKalman; // Store previous state for message toggling

        // Default to ICM20948 accelerometer
        memcpy(current_accel_for_kalman, icm_accel, sizeof(icm_accel)); // icm_accel is global from icm_functions
        g_usingKX134ForKalman = false;

        // Calculate magnitude of ICM20948 acceleration
        float icm_accel_magnitude = sqrt(icm_accel[0] * icm_accel[0] +
                                         icm_accel[1] * icm_accel[1] +
                                         icm_accel[2] * icm_accel[2]);

        if (icm_accel_magnitude > 16.0f) { // Threshold for switching to KX134 (e.g. >16g)
            if (g_kx134_initialized_ok) { // Check if KX134 is available and working
                memcpy(current_accel_for_kalman, kx134_accel, sizeof(kx134_accel)); // kx134_accel is global from kx134_functions
                g_usingKX134ForKalman = true;
                if (!kx134_was_used && g_debugFlags.enableIMUDebug) { // Print only on change
                    Serial.println(F("KALMAN: High-G detected. Switched to KX134 for accelerometer data."));
                }
            } else {
                if (g_debugFlags.enableIMUDebug) { // If KX134 not ok, print warning but continue with ICM (already copied)
                    Serial.println(F("KALMAN: High-G detected, but KX134 not available. Using ICM20948 accel."));
                }
            }
        } else {
            // Already using ICM20948 (default), no change needed for data
            // g_usingKX134ForKalman is already false
            if (kx134_was_used && g_debugFlags.enableIMUDebug) { // Print only on change
                Serial.println(F("KALMAN: Low-G detected. Switched back to ICM20948 for accelerometer data."));
            }
        }

        if (dt_kalman > 0.0f && dt_kalman < 1.0f) { // Basic sanity check for dt
            float kf_calibrated_gyro[3];
            ICM_20948_get_calibrated_gyro(kf_calibrated_gyro); // Get calibrated gyro data

            // Accel data for Kalman is now in current_accel_for_kalman (g's)

            kalman_predict(kf_calibrated_gyro[0], kf_calibrated_gyro[1], kf_calibrated_gyro[2], dt_kalman);
            kalman_update_accel(current_accel_for_kalman[0], current_accel_for_kalman[1], current_accel_for_kalman[2]);
            float mag_data[3]; // mag_data is local
            icm_20948_get_mag(mag_data); // Populates local mag_data from sensor
            kalman_update_mag(mag_data[0], mag_data[1], mag_data[2]);
            kalman_get_orientation(g_kalmanRoll, g_kalmanPitch, g_kalmanYaw); // Updates global Kalman orientation
        }
    }
  }

  // --- Data Logging ---
  // Call WriteLogData when sensors have been updated
  if (sensorsUpdatedThisCycle) {
    WriteLogData(false); // false = don't force, use normal timing interval
  }

  // --- Guidance Control Update ---
  static unsigned long g_lastGuidanceUpdateTime = 0;
  if (millis() - g_lastGuidanceUpdateTime >= GUIDANCE_UPDATE_INTERVAL_MS) {
      float dt_guidance = (millis() - g_lastGuidanceUpdateTime) / 1000.0f;
      if (dt_guidance <= 0.0f) { // Ensure dt is positive, can happen if millis() wraps or interval is too small
          dt_guidance = 1.0f / (1000.0f / GUIDANCE_UPDATE_INTERVAL_MS); // Use configured rate
      }
      g_lastGuidanceUpdateTime = millis();

      // Assuming g_kalmanRoll, g_kalmanPitch, g_kalmanYaw are updated radians from Kalman filter

      float gu_calibrated_gyro[3];
      ICM_20948_get_calibrated_gyro(gu_calibrated_gyro); // Get calibrated gyro data

      guidance_update(g_kalmanRoll, g_kalmanPitch, g_kalmanYaw,
                      gu_calibrated_gyro[0], gu_calibrated_gyro[1], gu_calibrated_gyro[2],
                      dt_guidance);

      float pitch_command_norm, roll_command_norm, yaw_command_norm; // Normalized (-1 to 1)
      guidance_get_actuator_outputs(pitch_command_norm, roll_command_norm, yaw_command_norm);

      // Map normalized commands to servo angles (degrees)
      // Example: maps -1.0 -> 0 deg, 0.0 -> 90 deg (SERVO_DEFAULT_ANGLE), 1.0 -> 180 deg
      // This assumes a servo travel of 90 degrees in each direction from center.
      // Adjust the multiplier (90.0f) if servo travel is different (e.g., 45.0f for +/- 45 deg travel).
      int pitch_servo_angle = static_cast<int>(pitch_command_norm * 90.0f + SERVO_DEFAULT_ANGLE);
      int roll_servo_angle  = static_cast<int>(roll_command_norm * 90.0f + SERVO_DEFAULT_ANGLE);
      int yaw_servo_angle   = static_cast<int>(yaw_command_norm * 90.0f + SERVO_DEFAULT_ANGLE);

      // Constrain servo angles to typical 0-180 range
      pitch_servo_angle = constrain(pitch_servo_angle, 0, 180);
      roll_servo_angle  = constrain(roll_servo_angle, 0, 180);
      yaw_servo_angle   = constrain(yaw_servo_angle, 0, 180);

      // Only command servos if in an appropriate flight state
      // Example: Active during COAST, DROGUE_DESCENT, MAIN_DESCENT. (Adjust as per actual flight plan)
      // Or, if attitude hold is desired on pad before launch (e.g. ARMED state).
      if ((g_currentFlightState == COAST || g_currentFlightState == DROGUE_DESCENT || g_currentFlightState == MAIN_DESCENT) && !isStationary) {
           // Adding !isStationary to prevent fighting on the ground if state is descent but already landed.
           // isStationary is an extern bool from icm_20948_functions.h
           g_servo_pitch.write(pitch_servo_angle);
           g_servo_roll.write(roll_servo_angle);
           g_servo_yaw.write(yaw_servo_angle);

           if (g_debugFlags.enableSystemDebug) { // Optional: print servo commands
              Serial.print(F("Servo CMDs (P,R,Y): "));
              Serial.print(pitch_servo_angle); Serial.print(F(", "));
              Serial.print(roll_servo_angle); Serial.print(F(", "));
              Serial.println(yaw_servo_angle);
           }
      }
  }

  // --- Flight State Processing ---
  ProcessFlightState(); // Handle flight state machine logic

  // --- Periodic Battery Voltage Printout ---
  #if ENABLE_BATTERY_MONITORING == 1
    static unsigned long lastBatteryPrintTime = 0;
    if (g_debugFlags.enableBatteryDebug && (millis() - lastBatteryPrintTime >= BATTERY_VOLTAGE_READ_INTERVAL_MS)) {
      lastBatteryPrintTime = millis();
      Serial.print(F("Battery Voltage: "));
      Serial.print(g_battery_voltage, 2);
      Serial.println(F(" V"));
    }
  #endif

  // TODO: Add other periodic tasks here as needed
  // - Actuator updates 
  // - Status monitoring
  
  // Note: The rest of the loop function implementation should be added here
  // based on the flight state machine and other system requirements

  // Function to handle initial state management after setup
  handleInitialStateManagement();
}

// Implementation of initSDCard function
bool initSDCard(SdFat& sd_obj, bool& sdCardMounted_out, bool& sdCardPresent_out) {
  Serial.println(F("Initializing SD card..."));
  
  // Reset output parameters
  sdCardMounted_out = false;
  sdCardPresent_out = false;
  
  // Try to initialize the SD card
  if (!sd_obj.begin(SdioConfig(FIFO_SDIO))) {
    Serial.println(F("SD Card initialization failed!"));
    g_last_error_code = SD_CARD_INIT_FAIL; // Or SD_CARD_MOUNT_FAIL depending on interpretation
    return false;
  }
  
  Serial.println(F("SD Card initialized successfully."));
  sdCardPresent_out = true;
  sdCardMounted_out = true;
  
  return true;
}
