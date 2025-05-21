// TripleT Flight Firmware
// Current Version: v0.20
// Current State: Alpha
// Last Updated: 21/05/2025
// **Notes**
// This code started out life as a remake of the Blip Test Code from Joe Barnard @ BPS.Space
// Nothing remains of the original code but that's where the concept originated.
// His code taught me a lot and you should go support his content !!
// I also learned bits from @LabRatMatt from YouTube and instructables
// It has been heavily modifed from the original as I've chosen the Teensy 4.0 and different sensors.

// Getting started with implementing the data collecting from the sensors.
// Once we have all the sensors behaving we'll start looking to do more of the actual rocket fucntions.
// the I2C scanner find the following

// 22:04:03.459 -> Device found at address 0x1F  (unknown chip) assumed to be the GPS
// 22:04:03.459 -> Device found at address 0x42  (PCA9685) maybe the KX134 ?
// 22:04:03.459 -> Device found at address 0x69  (MPU6050,MPU9050,MPU9250,ITG3701,L3G4200D)
// 22:04:03.459 -> Device found at address 0x77  (BMP085,BMA180,BMP280,MS5611)

// Currently working (well partially)
// We can detect the module and pull some data from it.
// SparkFun 9DoF IMU (ICM_20948)
// SparkFun UBlox ZOE-M8Q
// SparkFun KX134 
// MS5611 (is detected not 100% sure on the data just yet)

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
#define TRIPLET_FLIGHT_VERSION 0.20

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

// Define variables declared as extern in utility_functions.h
String FileDateString;  // For log file naming
String LogDataString;   // For data logging
unsigned long currentTime;  // For timestamp
bool baroCalibrated = false;  // For barometric calibration status
const char* BOARD_NAME = "Teensy 4.1";

// External declarations for sensor data
extern SFE_UBLOX_GNSS myGNSS;  // GPS object
extern float pressure;
extern float temperature;
extern float kx134_accel[3];

// Define sensor objects
SparkFun_KX134 kx134Accel;  // Add KX134 accelerometer object definition

// Global variables for ICM_20948 IMU data are defined in icm_20948_functions.cpp

// Define the structure that matches our binary data format
struct LogDataStruct {
  uint32_t timestamp;      // 4 bytes
  uint8_t fixType;        // 1 byte
  uint8_t sats;           // 1 byte
  int32_t latitude;       // 4 bytes
  int32_t longitude;      // 4 bytes
  int32_t altitude;       // 4 bytes
  int32_t altitudeMSL;    // 4 bytes
  int32_t speed;          // 4 bytes
  int32_t heading;        // 4 bytes
  uint16_t pDOP;          // 2 bytes
  uint8_t rtk;            // 1 byte
  float pressure;         // 4 bytes
  float temperature;      // 4 bytes
  float kx134_x;         // 4 bytes
  float kx134_y;         // 4 bytes
  float kx134_z;         // 4 bytes
  float icm_accel[3];    // 12 bytes (x, y, z)
  float icm_gyro[3];     // 12 bytes (x, y, z)
  float icm_mag[3];      // 12 bytes (x, y, z)
  float icm_temp;        // 4 bytes - Temperature from ICM sensor
};

// Storage configuration
#define SD_CARD_MIN_FREE_SPACE 1024 * 1024  // 1MB minimum free space
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
    Serial.println(F("Failed to create log file"));
    return false;
  }
  
  // Write CSV header according to LogData struct in data_structures.h
  // NOTE: Keep this manually synchronized with the LogData struct definition!
  LogDataFile.println(F("SeqNum,Timestamp,FixType,Sats,Lat,Long,Alt,AltMSL,Speed,Heading,pDOP,RTK,Pressure,Temperature,"
                       "KX134_AccelX,KX134_AccelY,KX134_AccelZ,"
                       "ICM_AccelX,ICM_AccelY,ICM_AccelZ,"
                       "ICM_GyroX,ICM_GyroY,ICM_GyroZ,"
                       "ICM_MagX,ICM_MagY,ICM_MagZ,"
                       "ICM_Temp"));
  
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
  // --------------------------------

  // Output to serial if enabled
  if (enableSerialCSV) {
    // Convert struct to string and print
    Serial.println(logDataToString(logEntry)); 
  }


  // If SD card not available, just return
  if (!sdCardAvailable) {
    return;
  }

  // Write to SD card if file is open
  if (!LogDataFile || !LogDataFile.isOpen()) {
    // Try to create new log file if none exists
    if (sdCardAvailable) {
      createNewLogFile(); // Attempt to open/create file
      // Check again if file is now open after attempting creation
      if (!LogDataFile || !LogDataFile.isOpen()) {
        return; // Still couldn't open, exit
      }
    } else {
       return; // SD not available, exit
    } 
  }
  
  // Write data to file as text
  if (!LogDataFile.println(logDataToString(logEntry))) {
    Serial.println(F("Failed to write to log file"));
    // Consider closing/reopening file or other error handling here
    return;
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
void printStatusSummary() {
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
  Serial.print(F("  System Debug: "));
  Serial.println(enableSystemDebug ? F("ON") : F("OFF"));
  Serial.print(F("  IMU Debug: "));
  Serial.println(enableIMUDebug ? F("ON") : F("OFF"));
  Serial.print(F("  GPS Debug: "));
  Serial.println(enableGPSDebug ? F("ON") : F("OFF"));
  Serial.print(F("  Barometer Debug: "));
  Serial.println(enableBaroDebug ? F("ON") : F("OFF"));
  Serial.print(F("  Storage Debug: "));
  Serial.println(enableStorageDebug ? F("ON") : F("OFF"));
  Serial.print(F("  ICM Raw Debug: "));
  Serial.println(enableICMRawDebug ? F("ON") : F("OFF"));
  Serial.print(F("  Serial CSV Output: "));
  Serial.println(enableSerialCSV ? F("ON") : F("OFF"));
  
  // Debug commands (numeric)
  Serial.println(F("\nDebug Commands (Toggle with numbers):"));
  Serial.println(F("  0 - Toggle serial CSV output"));
  Serial.println(F("  1 - Toggle system debug"));
  Serial.println(F("  2 - Toggle IMU debug"));
  Serial.println(F("  3 - Toggle GPS debug"));
  Serial.println(F("  4 - Toggle barometer debug"));
  Serial.println(F("  5 - Toggle storage debug"));
  Serial.println(F("  6 - Toggle ICM raw debug"));
  Serial.println(F("  9 - Initiate Shutdown"));
  Serial.println(F("  0 - Toggle CSV output"));
  
  // Other commands (alphabetic)
  Serial.println(F("\nOther Commands (Use letters):"));
  Serial.println(F("  a - Show this help message"));
  Serial.println(F("  b - Show system status"));
  Serial.println(F("  c - Dump all data"));
  Serial.println(F("  d - Erase all stored data on flash"));
  Serial.println(F("  e - List available log files"));  
  Serial.println(F("  f - Show data statistics"));
  Serial.println(F("  g - Toggle detailed display"));
  Serial.println(F("  h - Calibrate barometer"));
  Serial.println(F("  i - Display IMU data"));
  Serial.println(F("  j - Toggle status summary"));
  
  // Utility commands
  Serial.println(F("\nUtility Commands:"));
  Serial.println(F("  debug_all_off - Disable all debugging"));
  
  Serial.println(F("\nLegacy commands still supported"));
  Serial.println(F("  sd - Toggle sensor debug"));
  Serial.println(F("  rd - Toggle ICM raw debug"));
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

void processCommand(String command) {
    // Single character numeric commands for debug toggles
    if (command.length() == 1) {
        char cmd = command.charAt(0);
        
        // Numeric commands (0-9) for debug toggles
        if (cmd >= '0' && cmd <= '9') {
            switch (cmd) {
                case '0': // Serial CSV output
                    enableSerialCSV = !enableSerialCSV;
                    Serial.print(F("Serial CSV output: "));
                    Serial.println(enableSerialCSV ? F("ON") : F("OFF"));
                    break;
                case '1': // System debug
                    enableSystemDebug = !enableSystemDebug;
                    Serial.print(F("System debug: "));
                    Serial.println(enableSystemDebug ? F("ON") : F("OFF"));
                    break;
                case '2': // IMU debug
                    enableIMUDebug = !enableIMUDebug;
                    Serial.print(F("IMU debug: "));
                    Serial.println(enableIMUDebug ? F("ON") : F("OFF"));
                    break;
                case '3': // GPS debug
                    enableGPSDebug = !enableGPSDebug;
                    Serial.print(F("GPS debug: "));
                    Serial.println(enableGPSDebug ? F("ON") : F("OFF"));
                    
                    // Apply debug setting using our consolidated function
                    setGPSDebugging(enableGPSDebug);
                    break;
                case '4': // Barometer debug
                    enableBaroDebug = !enableBaroDebug;
                    Serial.print(F("Barometer debug: "));
                    Serial.println(enableBaroDebug ? F("ON") : F("OFF"));
                    break;
                case '5': // Storage debug
                    enableStorageDebug = !enableStorageDebug;
                    Serial.print(F("Storage debug: "));
                    Serial.println(enableStorageDebug ? F("ON") : F("OFF"));
                    break;
                case '6': // ICM raw debug
                    enableICMRawDebug = !enableICMRawDebug;
                    Serial.print(F("ICM raw debug: "));
                    Serial.println(enableICMRawDebug ? F("ON") : F("OFF"));
                    break;
                case '9': // ICM raw debug
                    prepareForShutdown();
                    break;
            }
            return;
        }
        
        // Alphabetic commands (a-k) for other functions
        if (cmd >= 'a' && cmd <= 'k') {
            switch (cmd) {
                case 'a': // Help
                    printHelpMessage();
                    break;
                case 'b': // Status
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
                    break;
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
                case 'f': // Stats
                    printStorageStatistics();
                    break;
                case 'g': // Detailed display
                    displayMode = !displayMode;
                    Serial.print(F("Detailed display mode: "));
                    Serial.println(displayMode ? F("ON") : F("OFF"));
                    break;
                case 'h': // Calibrate
                    if (!baroCalibrated) {
                        // Change LED to purple to indicate calibration in progress
                        pixels.setPixelColor(0, pixels.Color(50, 0, 50));
                        pixels.show();

                        // ms5611_calibrate_with_gps();
                        // Serial.println(F("Starting barometric calibration with GPS..."));
                        // Serial.println(F("Waiting for good GPS fix (pDOP < 3.0)..."));
                        
                        if (ms5611_calibrate_with_gps(30000)) {  // Wait up to 60 seconds for calibration
                            Serial.println(F("Barometric calibration successful!"));
                            baroCalibrated = true;
                            // Change LED to green to indicate success
                            pixels.setPixelColor(0, pixels.Color(0, 50, 0));
                            pixels.show();
                            delay(1000);
                        } else {
                            Serial.println(F("Barometric calibration timed out or failed."));
                            // Change LED to red to indicate failure
                            pixels.setPixelColor(0, pixels.Color(50, 0, 0));
                            pixels.show();
                            delay(1000);
                        }
                    } else {
                        Serial.println(F("Barometric calibration has already been performed."));
                    }
                    break;
                case 'i': // IMU data
                    ICM_20948_print();
                    break;
                case 'j': // Status summary
                    enableStatusSummary = !enableStatusSummary;
                    Serial.print(F("Status summary: "));
                    Serial.println(enableStatusSummary ? F("ON") : F("OFF"));
                    break;
            }
            return;
        }
    }
    
    // Legacy command handling for backward compatibility
    if (command == "status") {
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
        Serial.print("Sensor Debug: ");
        Serial.println(enableSensorDebug ? "Enabled" : "Disabled");
        Serial.print("GPS Debug: ");
        Serial.println(enableGPSDebug ? "Enabled" : "Disabled");
        Serial.print("Status Summary: ");
        Serial.println(enableStatusSummary ? "Enabled" : "Disabled");
    } else if (command == "sd") {
        enableSensorDebug = !enableSensorDebug;
        Serial.print("Sensor debug output ");
        Serial.println(enableSensorDebug ? "enabled" : "disabled");
    } else if (command == "rd") {
        enableICMRawDebug = !enableICMRawDebug;
        Serial.print("ICM raw debug output ");
        Serial.println(enableICMRawDebug ? "enabled" : "disabled");
    } else if (command == "summary") {
        enableStatusSummary = !enableStatusSummary;
        Serial.print(F("Status summary: "));
        Serial.println(enableStatusSummary ? F("ENABLED") : F("DISABLED"));
    } else if (command == "calibrate") {
      // Manual calibration command
      if (!baroCalibrated) {
        Serial.println(F("Starting barometric calibration with GPS..."));
        Serial.println(F("Waiting for good GPS fix (pDOP < 3.0)..."));
        
        // Change LED to purple to indicate calibration in progress
        pixels.setPixelColor(0, pixels.Color(50, 0, 50));
        pixels.show();
        
        if (ms5611_calibrate_with_gps(60000)) {  // Wait up to 60 seconds for calibration
          Serial.println(F("Barometric calibration successful!"));
          baroCalibrated = true;
          // Change LED to green to indicate success
          pixels.setPixelColor(0, pixels.Color(0, 50, 0));
          pixels.show();
          delay(1000);
        } else {
          Serial.println(F("Barometric calibration timed out or failed."));
          // Change LED to red to indicate failure
          pixels.setPixelColor(0, pixels.Color(50, 0, 0));
          pixels.show();
          delay(1000);
        }
      } else {
        Serial.println(F("Barometric calibration has already been performed."));
      }
    } else if (command == "help") {
      // Show help message
      printHelpMessage();
    }
    
    // Handle debug commands
    else if (command.startsWith("debug_system")) {
      if (command.indexOf("on") != -1) {
        enableSystemDebug = true;
        Serial.println(F("System debug enabled"));
      } else if (command.indexOf("off") != -1) {
        enableSystemDebug = false;
        Serial.println(F("System debug disabled"));
      } else {
        // Toggle current state
        enableSystemDebug = !enableSystemDebug;
        Serial.print(F("System debug toggled to: "));
        Serial.println(enableSystemDebug ? F("ON") : F("OFF"));
      }
    }
    else if (command.startsWith("debug_imu")) {
      if (command.indexOf("on") != -1) {
        enableIMUDebug = true;
        Serial.println(F("IMU debug enabled"));
      } else if (command.indexOf("off") != -1) {
        enableIMUDebug = false;
        Serial.println(F("IMU debug disabled"));
      } else {
        // Toggle current state
        enableIMUDebug = !enableIMUDebug;
        Serial.print(F("IMU debug toggled to: "));
        Serial.println(enableIMUDebug ? F("ON") : F("OFF"));
      }
    }
    else if (command.startsWith("debug_gps")) {
      if (command.indexOf("on") != -1) {
        enableGPSDebug = true;
        Serial.println(F("GPS debug enabled"));
        // Apply the debug setting using our consolidated function
        setGPSDebugging(true);
      } else if (command.indexOf("off") != -1) {
        enableGPSDebug = false;
        Serial.println(F("GPS debug disabled"));
        // Apply the debug setting using our consolidated function
        setGPSDebugging(false);
      } else {
        // Toggle current state
        enableGPSDebug = !enableGPSDebug;
        Serial.print(F("GPS debug toggled to: "));
        Serial.println(enableGPSDebug ? F("ON") : F("OFF"));
        // Apply the debug setting using our consolidated function
        setGPSDebugging(enableGPSDebug);
      }
    }
    else if (command.startsWith("debug_baro")) {
      if (command.indexOf("on") != -1) {
        enableBaroDebug = true;
        Serial.println(F("Barometer debug enabled"));
      } else if (command.indexOf("off") != -1) {
        enableBaroDebug = false;
        Serial.println(F("Barometer debug disabled"));
      } else {
        // Toggle current state
        enableBaroDebug = !enableBaroDebug;
        Serial.print(F("Barometer debug toggled to: "));
        Serial.println(enableBaroDebug ? F("ON") : F("OFF"));
      }
    }
    else if (command.startsWith("debug_storage")) {
      if (command.indexOf("on") != -1) {
        enableStorageDebug = true;
        Serial.println(F("Storage debug enabled"));
      } else if (command.indexOf("off") != -1) {
        enableStorageDebug = false;
        Serial.println(F("Storage debug disabled"));
      } else {
        // Toggle current state
        enableStorageDebug = !enableStorageDebug;
        Serial.print(F("Storage debug toggled to: "));
        Serial.println(enableStorageDebug ? F("ON") : F("OFF"));
      }
    }
    else if (command.startsWith("debug_icm_raw")) {
      if (command.indexOf("on") != -1) {
        enableICMRawDebug = true;
        Serial.println(F("ICM raw debug enabled"));
      } else if (command.indexOf("off") != -1) {
        enableICMRawDebug = false;
        Serial.println(F("ICM raw debug disabled"));
      } else {
        // Toggle current state
        enableICMRawDebug = !enableICMRawDebug;
        Serial.print(F("ICM raw debug toggled to: "));
        Serial.println(enableICMRawDebug ? F("ON") : F("OFF"));
      }
    }
    else if (command == "debug_all_off") {
      // Disable all debugging at once
      enableDetailedOutput = false;
      enableSystemDebug = false;
      enableSensorDebug = false;
      enableIMUDebug = false;
      enableGPSDebug = false;
      enableBaroDebug = false;
      enableStorageDebug = false;
      enableICMRawDebug = false;
      displayMode = false;
      
      // Make sure GPS module debugging is explicitly disabled using our consolidated function
      setGPSDebugging(false);
      
      Serial.println(F("All debugging disabled"));
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
  // Logging disabled by configuration, ensure flags are false and exit immediately.
  sdCardPresent = false;
  sdCardMounted = false;
  sdCardAvailable = false;
  return false;
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
}



