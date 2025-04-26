// TripleT Flight Firmware
// Current Version: v0.15
// Current State: Alpha
// Last Updated: 16/03/2025
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
#define TRIPLET_FLIGHT_VERSION 0.15

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
extern bool useKX134;  // Flag to track which accelerometer is being used

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

// Timing variables for sensor updates
unsigned long lastGPSUpdate = 0;
unsigned long lastIMUUpdate = 0;
unsigned long lastLogUpdate = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long logInterval = 200; // Default log interval in ms (5Hz)

// Flight state management
enum FlightState {
  IDLE,
  READY,
  LIFTOFF,
  ASCENT,
  APOGEE,
  DESCENT,
  LANDED,
  ERROR
};

FlightState flightState = FlightState::IDLE;

// Forward declarations for flight functions
void ProcessFlightState();
void DetectLiftoff();
void UpdateDisplay();

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
  
  // Write CSV header
  LogDataFile.println(F("Timestamp,FixType,Sats,Lat,Long,Alt,AltMSL,Speed,Heading,pDOP,RTK,Pressure,Temperature,"
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
  
  // Update current time
  currentTime = millis();
  
  // Format data string
  LogDataString = String(currentTime) + "," +
                  String(GPS_fixType) + "," +
                  String(SIV) + "," +
                  String(GPS_latitude / 10000000.0, 6) + "," +
                  String(GPS_longitude / 10000000.0, 6) + "," +
                  String(GPS_altitude / 1000.0, 2) + "," +
                  String(GPS_altitudeMSL / 1000.0, 2) + "," + 
                  String(GPS_speed / 1000.0, 2) + "," +       
                  String(GPS_heading / 100000.0, 2) + "," +   
                  String(pDOP / 100.0, 2) + "," +             
                  String(RTK) + "," +
                  String(pressure, 2) + "," +
                  String(temperature, 2) + "," +
                  String(kx134_accel[0], 4) + "," +
                  String(kx134_accel[1], 4) + "," +
                  String(kx134_accel[2], 4) + "," +
                  String(icm_accel[0], 4) + "," +
                  String(icm_accel[1], 4) + "," +
                  String(icm_accel[2], 4) + "," +
                  String(icm_gyro[0], 4) + "," +
                  String(icm_gyro[1], 4) + "," +
                  String(icm_gyro[2], 4) + "," +
                  String(icm_mag[0], 4) + "," +
                  String(icm_mag[1], 4) + "," +
                  String(icm_mag[2], 4) + "," +
                  String(icm_temp, 2);

  // Output to serial if enabled
  if (enableSerialCSV) {
    Serial.println(LogDataString);
  }

  // If SD card not available, just return
  if (!sdCardAvailable) {
    return;
  }

  // Write to SD card if file is open
  if (!LogDataFile || !LogDataFile.isOpen()) {
    // Try to create new log file if none exists
    if (sdCardAvailable) {
      createNewLogFile();
    }
    return;
  }
  
  // Write data to file
  if (!LogDataFile.println(LogDataString)) {
    Serial.println(F("Failed to write to log file"));
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
  Serial.println(F("\n=== TripleT Flight Commands ==="));
  Serial.println(F("help      - Show this help message"));
  Serial.println(F("status    - Show system status"));
  Serial.println(F("calibrate - Calibrate barometer with GPS"));
  Serial.println(F("storage   - Show storage statistics"));
  Serial.println(F("display   - Toggle status display"));
  Serial.println(F("csv [on|off] - Toggle CSV data output"));
  
  Serial.println(F("\n=== Debug Commands ==="));
  Serial.println(F("debug system [on|off]  - System debug messages"));
  Serial.println(F("debug sensor [on|off]  - Sensor data debug"));
  Serial.println(F("debug gps [on|off]     - GPS data debug"));
  Serial.println(F("debug baro [on|off]    - Barometer data debug"));
  Serial.println(F("debug storage [on|off] - Storage operations debug"));
  Serial.println(F("debug icmraw [on|off]  - ICM raw data debug"));
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

// Add this function declaration near other function declarations

void processCommand(String command) {
  // Process single command
  command.trim();
  if (command.length() == 0) return;

  Serial.print(F("CMD: "));
  Serial.println(command);

  // Process commands
  if (command == "help") {
    printHelpMessage();
  }
  else if (command.startsWith("debug ")) {
    // Debug commands
    String debugMode = command.substring(6);
    
    if (debugMode == "on" || debugMode == "1") {
      enableDetailedOutput = true;
      Serial.println(F("Detailed debug output enabled"));
    }
    else if (debugMode == "off" || debugMode == "0") {
      enableDetailedOutput = false;
      Serial.println(F("Detailed debug output disabled"));
    }
    else if (debugMode == "system") {
      enableSystemDebug = !enableSystemDebug;
      Serial.print(F("System debug: "));
      Serial.println(enableSystemDebug ? F("ON") : F("OFF"));
    }
    else if (debugMode == "sensor") {
      enableSensorDebug = !enableSensorDebug;
      Serial.print(F("Sensor debug: "));
      Serial.println(enableSensorDebug ? F("ON") : F("OFF"));
    }
    else if (debugMode == "imu") {
      enableIMUDebug = !enableIMUDebug;
      Serial.print(F("IMU debug: "));
      Serial.println(enableIMUDebug ? F("ON") : F("OFF"));
    }
    else if (debugMode == "gps") {
      enableGPSDebug = !enableGPSDebug;
      Serial.print(F("GPS debug: "));
      Serial.println(enableGPSDebug ? F("ON") : F("OFF"));
    }
    else if (debugMode == "baro") {
      enableBaroDebug = !enableBaroDebug;
      Serial.print(F("Barometer debug: "));
      Serial.println(enableBaroDebug ? F("ON") : F("OFF"));
    }
    else if (debugMode == "storage") {
      enableStorageDebug = !enableStorageDebug;
      Serial.print(F("Storage debug: "));
      Serial.println(enableStorageDebug ? F("ON") : F("OFF"));
    }
    else if (debugMode == "icm_raw") {
      enableICMRawDebug = !enableICMRawDebug;
      Serial.print(F("ICM-20948 Raw debug: "));
      Serial.println(enableICMRawDebug ? F("ON") : F("OFF"));
    }
    else if (debugMode == "status") {
      enableStatusSummary = !enableStatusSummary;
      Serial.print(F("Status summary: "));
      Serial.println(enableStatusSummary ? F("ON") : F("OFF"));
    }
    else if (debugMode == "all") {
      enableDetailedOutput = true;
      enableSystemDebug = true;
      enableSensorDebug = true;
      enableIMUDebug = true;
      enableGPSDebug = true;
      enableBaroDebug = true;
      enableStorageDebug = true;
      enableICMRawDebug = true;
      enableStatusSummary = true;
      Serial.println(F("All debug options enabled"));
    }
    else if (debugMode == "none") {
      enableDetailedOutput = false;
      enableSystemDebug = false;
      enableSensorDebug = false;
      enableIMUDebug = false;
      enableGPSDebug = false;
      enableBaroDebug = false;
      enableStorageDebug = false;
      enableICMRawDebug = false;
      enableStatusSummary = false;
      Serial.println(F("All debug options disabled"));
    }
    else {
      Serial.println(F("Unknown debug mode. Use: on, off, system, sensor, imu, gps, baro, storage, icm_raw, status, all, none"));
    }
  }
  else if (command == "status") {
    printStatusSummary();
  }
  else if (command == "calibrate") {
    if (gpsHasFix && gpsFixType >= 3) {
      // Only calibrate if we have a 3D fix
      baro_altitude_offset = gpsAltitudeMSL - (44330.0 * (1.0 - pow((pressure * 100.0) / 101325.0, 0.1903)));
      baroCalibrated = true;
      Serial.print(F("Barometer calibrated! Offset: "));
      Serial.print(baro_altitude_offset, 2);
      Serial.println(F("m"));
    } else {
      Serial.println(F("ERROR: Cannot calibrate without GPS 3D fix"));
    }
  }
  else if (command == "storage") {
    printStorageStatistics();
  }
  else if (command == "display") {
    // Toggle display mode
    enableStatusSummary = !enableStatusSummary;
    Serial.print(F("Status display: "));
    Serial.println(enableStatusSummary ? F("ON") : F("OFF"));
  }
  else if (command.startsWith("csv")) {
    String csvMode = command.length() > 4 ? command.substring(4) : "";
    csvMode.trim();
    
    if (csvMode == "on" || csvMode == "1") {
      enableSerialCSV = true;
      Serial.println(F("Serial CSV output enabled"));
    }
    else if (csvMode == "off" || csvMode == "0") {
      enableSerialCSV = false;
      Serial.println(F("Serial CSV output disabled"));
    }
    else {
      // Toggle if no parameter
      enableSerialCSV = !enableSerialCSV;
      Serial.print(F("Serial CSV output: "));
      Serial.println(enableSerialCSV ? F("ON") : F("OFF"));
    }
  }
  else {
    Serial.print(F("Unknown command: "));
    Serial.println(command);
    Serial.println(F("Type 'help' for available commands"));
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
  
  // Initialize storage after GPS is ready
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
    Serial.println(F("WARNING: No storage available for data logging!"));
    loggingEnabled = false;
  }
  
  // Change LED to green to indicate successful initialization
  pixels.setPixelColor(0, pixels.Color(0, 50, 0));
  pixels.show();
  
  // Barometric calibration is now done via command
  Serial.println(F("Use 'calibrate' command to perform barometric calibration with GPS"));
}

void loop() {
  // Check for serial commands
  while (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
  
  // Read sensors at the defined update rates
  currentTime = millis();
  
  // GPS update (1 Hz)
  if (currentTime - lastGPSUpdate >= 1000) {
    gps_read();
    lastGPSUpdate = currentTime;
  }
  
  // IMU and barometer update (~100 Hz)
  if (currentTime - lastIMUUpdate >= 10) {  // 10ms = 100Hz
    // Read barometer
    ms5611_read();
    
    // Read KX134 accelerometer
    kx134_read();
    
    // Read ICM-20948 IMU
    ICM_20948_read();
    
    // Process flight state and events based on current measurements
    ProcessFlightState();
    
    // Check for liftoff detection if in READY state
    if (flightState == FlightState::READY) {
      DetectLiftoff();
    }
    
    // Data logging
    if (loggingEnabled && (currentTime - lastLogUpdate >= logInterval)) {
      WriteLogData(false);
      lastLogUpdate = currentTime;
    }
    
    lastIMUUpdate = currentTime;
  }
  
  // Update display and indicators at 5Hz
  if (currentTime - lastDisplayUpdate >= 200) {
    UpdateDisplay();
    lastDisplayUpdate = currentTime;
  }
}

// Process the current flight state based on sensor readings
void ProcessFlightState() {
  // For now, just a placeholder
  // In a complete implementation, this would transition between states
  // based on sensor readings, time, and events
  
  // Example state transition logic:
  switch(flightState) {
    case FlightState::IDLE:
      // Check if ready for launch
      if (baroCalibrated && GPS_fixType >= 3) {
        flightState = FlightState::READY;
        Serial.println("Entered READY state");
      }
      break;
      
    case FlightState::READY:
      // Transitions to LIFTOFF occur in DetectLiftoff()
      break;
      
    case FlightState::LIFTOFF:
      // Once confirmed liftoff, transition to ASCENT
      flightState = FlightState::ASCENT;
      Serial.println("Entered ASCENT state");
      break;
      
    case FlightState::ASCENT:
      // Monitor for apogee
      // This would use barometer and/or accelerometer data
      break;
      
    case FlightState::APOGEE:
      // Transition to descent after apogee events complete
      flightState = FlightState::DESCENT;
      Serial.println("Entered DESCENT state");
      break;
      
    case FlightState::DESCENT:
      // Check for landing
      break;
      
    case FlightState::LANDED:
      // Final state - log data and prepare for recovery
      break;
      
    case FlightState::ERROR:
      // Handle error conditions
      break;
  }
  }
  
// Detect liftoff based on accelerometer and barometer data
void DetectLiftoff() {
  // Example liftoff detection (simplified)
  float accel_magnitude;
  
  if (useKX134) {
    accel_magnitude = sqrt(
      kx134_accel[0] * kx134_accel[0] + 
      kx134_accel[1] * kx134_accel[1] + 
      kx134_accel[2] * kx134_accel[2]
    );
  } else {
    accel_magnitude = sqrt(
      icm_accel[0] * icm_accel[0] + 
      icm_accel[1] * icm_accel[1] + 
      icm_accel[2] * icm_accel[2]
    );
  }
  
  // Simple threshold for liftoff detection
  if (accel_magnitude > 2.0) { // > 2G acceleration
    flightState = FlightState::LIFTOFF;
    Serial.println("LIFTOFF DETECTED!");
    
    // Could trigger other liftoff-related actions here
  }
}

// Update display and status indicators
void UpdateDisplay() {
  // For now, this is just a placeholder
  // In a full implementation, this would update LEDs, buzzers, etc.
  
  // Example status summary every 5 seconds if enabled
  static unsigned long lastStatusUpdate = 0;
  if (enableStatusSummary && (millis() - lastStatusUpdate > 5000)) {
    printStatusSummary();
    lastStatusUpdate = millis();
  }
}



