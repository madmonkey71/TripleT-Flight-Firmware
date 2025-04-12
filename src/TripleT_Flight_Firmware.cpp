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
};

// Storage configuration
#define SD_CARD_MIN_FREE_SPACE 1024 * 1024  // 1MB minimum free space
#define EXTERNAL_FLASH_MIN_FREE_SPACE 1024 * 1024  // 1MB minimum free space

// Storage variables
SdFat SD;
FsVolume volume;
bool sdCardAvailable = false;
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

// Define global debug control variables (near the top with other global declarations)
bool enableSensorDebug = true;   // Enable sensor data debug output by default
bool enableQuaternionDebug = true;  // Enable quaternion debug output by default

void checkStorageSpace() {
    if (sdCardAvailable) {
        uint64_t freeSpace = SD.vol()->freeClusterCount() * SD.vol()->bytesPerCluster();
        if (freeSpace < SD_CARD_MIN_FREE_SPACE) {
            Serial.println("WARNING: SD card space low!");
        }
    }
}

// Function to create a new log file with timestamp in name
bool createNewLogFile() {
  // Format the filename with date/time if GPS is available
  if (myGNSS.getFixType() != 0) {
    FileDateString = String(myGNSS.getYear()) + 
                    String(myGNSS.getMonth()) + 
                    String(myGNSS.getDay()) + "_" +
                    String(myGNSS.getHour()) + 
                    String(myGNSS.getMinute()) + 
                    String(myGNSS.getSecond());
  } else {
    // If no GPS, use a counter
    static uint32_t fileCounter = 0;
    FileDateString = "LOG_" + String(fileCounter++);
  }

  // Create the full filename
  String fileName = FileDateString + ".csv";
  
  // Try to open the file
  if (!LogDataFile.open(fileName.c_str(), O_RDWR | O_CREAT | O_AT_END)) {
    Serial.println("Failed to create log file: " + fileName);
    return false;
  }

  // Write the CSV header
  LogDataFile.println("Timestamp,FixType,Sats,Lat,Long,Alt,AltMSL,Speed,Heading,pDOP,RTK,Pressure,Temperature,"
                     "KX134_AccelX,KX134_AccelY,KX134_AccelZ,"
                     "ICM_AccelX,ICM_AccelY,ICM_AccelZ,"
                     "ICM_GyroX,ICM_GyroY,ICM_GyroZ,"
                     "ICM_MagX,ICM_MagY,ICM_MagZ");

  // Flush the header to ensure it's written
  LogDataFile.sync();
  
  Serial.println("Created new log file: " + fileName);
  return true;
}

void WriteLogData(bool forceLog) {
  static unsigned long lastLogTime = 0;
  
  // Only log at specified intervals or when forced (after sensor reading)
  if (!forceLog && millis() - lastLogTime < 200) { // Changed from FLASH_LOG_INTERVAL to fixed 200ms
    return;
  }
  lastLogTime = millis();
  
  // Format the data string using global GPS variables
  LogDataString = String(currentTime) + "," +
                  String(GPS_fixType) + "," +
                  String(SIV) + "," +
                  String(GPS_latitude / 10000000.0, 6) + "," +
                  String(GPS_longitude / 10000000.0, 6) + "," +
                  String(GPS_altitude / 1000.0, 2) + "," +
                  String(GPS_altitudeMSL, 2) + "," +
                  String(GPS_speed, 2) + "," +
                  String(GPS_heading, 2) + "," +
                  String(pDOP / 100.0, 2) + "," +  // Convert from int to float
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

  // Write to SD card if available
  if (LogDataFile) {
    if (LogDataFile.println(LogDataString)) {
      // Use periodic sync instead of flush for better performance
      static uint8_t syncCounter = 0;
      if (++syncCounter >= 10) { // Sync every 10 writes
        LogDataFile.sync();
        syncCounter = 0;
      }
    } else {
      Serial.println("Failed to write to log file");
    }
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
  Serial.println(F("\n=== STATUS SUMMARY ==="));
  
  // Current time
  Serial.print(F("Time: "));
  Serial.print(millis() / 1000);
  Serial.print(F("s"));
  
  // GPS
  Serial.print(F("  GPS: "));
  if (GPS_fixType > 0) {
    Serial.print(GPS_fixType);
    Serial.print(F("D fix, "));
    Serial.print(SIV);
    Serial.print(F(" sats"));
    
    // Only print position if we have a fix
    Serial.print(F("Pos: "));
    Serial.print(GPS_latitude / 10000000.0, 6);  // Convert raw integer to decimal degrees
    Serial.print(F(", "));
    Serial.print(GPS_longitude / 10000000.0, 6); // Convert raw integer to decimal degrees
    Serial.print(F(" Alt: "));
    Serial.print(GPS_altitude / 1000.0, 2);  // Convert mm to meters with 2 decimal places
    Serial.println(F("m"));
  } else {
    Serial.println(F("No fix"));
  }
  
  // Barometer data
  if (enableSensorDebug) {
    Serial.print(F("Baro: "));
    Serial.print(pressure, 1);
    Serial.print(F("Pa, "));
    Serial.print(temperature, 1);
    Serial.println(F("°C"));
  
    // Barometer altitude
    if (baroCalibrated) {
      Serial.print(F("Alt: "));
      // Use pressure directly instead of calling undefined function
      float baroAltitude = 44330.0 * (1.0 - pow(pressure / 101325.0, 0.1903));
      Serial.print(baroAltitude, 1);
      Serial.println(F("m"));
    }
  
    // MS5611 Barometer data
    ms5611_print();
    // KX134 Accelerometer data
    kx134_print();
  
    // ICM-20948 IMU data
    Serial.print(F("  ICM-20948: "));
    ICM_20948_print();
  }
  
  // STORAGE STATUS - More compact representation
  Serial.println(F("STORAGE:"));
  Serial.print(F("  SD:"));
  Serial.print(sdCardAvailable ? F("OK") : F("--"));
  Serial.print(F(" | ExtFlash:"));
  Serial.println(flashAvailable ? F("OK") : F("--"));
  
  // Debug status
  Serial.println(F("DEBUG:"));
  Serial.print(F("  Sensor:"));
  Serial.print(enableSensorDebug ? F("ON") : F("OFF"));
  Serial.print(F(" | Quaternion:"));
  Serial.println(enableQuaternionDebug ? F("ON") : F("OFF"));
  
  Serial.println(F("==================================="));
  Serial.println(F("Commands: help, dump, stats, imu, detail, calibrate"));
}

// Updated help message with more compact formatting
void printHelpMessage() {
  Serial.println(F("\n--- AVAILABLE COMMANDS ---"));
  Serial.println(F("dump     - Dump external flash data"));
  Serial.println(F("erase    - Erase all external flash data"));
  Serial.println(F("list     - List all log files"));
  Serial.println(F("stats    - Show storage statistics"));
  Serial.println(F("detail   - Toggle detailed display"));
  Serial.println(F("imu      - Show detailed IMU data"));
  Serial.println(F("calibrate- Perform barometric calibration with GPS"));
  
  // Show current status of debug flags
  Serial.print(F("sensor   - Toggle sensor debug (currently "));
  Serial.print(enableSensorDebug ? F("ON") : F("OFF"));
  Serial.println(F(")"));
  
  Serial.print(F("quat     - Toggle quaternion debug (currently "));
  Serial.print(enableQuaternionDebug ? F("ON") : F("OFF"));
  Serial.println(F(")"));
  
  Serial.println(F("status   - Show system status"));
  Serial.println(F("help     - Show this help message"));
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

void handleCommand(const String& command) {
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
        Serial.print("Quaternion Debug: ");
        Serial.println(enableQuaternionDebug ? "Enabled" : "Disabled");
    } else if (command == "sensor") {
        enableSensorDebug = !enableSensorDebug;
        Serial.print(F("Sensor debug output: "));
        Serial.println(enableSensorDebug ? F("ENABLED") : F("DISABLED"));
    } else if (command == "quat") {
        enableQuaternionDebug = !enableQuaternionDebug;
        Serial.print(F("Quaternion debug output: "));
        Serial.println(enableQuaternionDebug ? F("ENABLED") : F("DISABLED"));
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
}

void setup() {
  // Wait for the Serial monitor to be opened.
  Serial.begin(115200);
  delay(500); // Give the serial port time to initialize
  
  // Initialize NeoPixel first for visual feedback
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red during startup
  pixels.show();
  
  // Startup Tone
  delay(500);
  tone(BUZZER_PIN, 2000); delay(50); noTone(BUZZER_PIN); delay(75);
  noTone(BUZZER_PIN);
  
  // Change LED to orange to indicate waiting for serial
  pixels.setPixelColor(0, pixels.Color(50, 25, 0)); // Orange during serial init
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
  #if defined(BOARD_STM32_THING_PLUS)
    // STM32 specific SPI setup
    SPI.setMOSI(SD_MOSI_PIN);
    SPI.setMISO(SD_MISO_PIN);
    SPI.setSCK(SD_SCK_PIN);
    SPI.begin();
    
    // STM32 specific I2C setup
    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);
  
    Serial.println("STM32 SPI and I2C initialized");
  #else
    // Teensy SPI setup
    SPI.setCS(SD_CS_PIN);
    SPI.setMOSI(SD_MOSI_PIN);
    SPI.setMISO(SD_MISO_PIN);
    SPI.setSCK(SD_SCK_PIN);
    
    // Standard I2C setup
    Wire.begin();
    
    Serial.println("Teensy SPI and I2C initialized");
  #endif
  // Scan the I2C bus for devices
  scan_i2c();

  // Now initialize sensors
  Serial.println(F("\nInitializing sensors..."));
  if (!kx134_init()) {
    Serial.println(F("WARNING: KX134 accelerometer initialization failed"));
  } else {
    Serial.println(F("KX134 accelerometer initialized"));
  }
  
  ICM_20948_init();
  Serial.println(F("ICM-20948 initialized"));
  
  ms5611_init();
  gps_init();
  // Initialize storage first
  Serial.println(F("Initializing storage..."));
  
  // Initialize SD card first since we may need it for flash data transfer
  initSDCard();
  Serial.print(F("SD Card: "));
  Serial.println(sdCardAvailable ? F("Available") : F("Not available"));
    
  if (sdCardAvailable || flashAvailable) {
    Serial.println(F("Creating new log file..."));
    createNewLogFile();
    Serial.println(F("Data logging ready."));
  } else {
    Serial.println(F("WARNING: No storage available for data logging!"));
  }
  
  Serial.print(" GPS Date/Time: ");
  Serial.print(myGNSS.getYear());
  Serial.print("-");
  Serial.print(myGNSS.getMonth());
  Serial.print("-");
  Serial.print(myGNSS.getDay());
  Serial.print(" ");
  Serial.print(myGNSS.getHour());
  Serial.print(":");
  Serial.print(myGNSS.getMinute());
  Serial.print(":");
  Serial.println(myGNSS.getSecond());
  
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
  static bool displayMode = false; // Toggle for compact vs detailed display
  
  // Check for serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "dump") {
      // Dump internal flash data on demand
      if (flashAvailable) {
        Serial.println(F("Dump command not supported for external flash."));
      } else {
        Serial.println(F("External flash not available."));
      }
    } else if (command == "erase") {
      // Erase internal flash data
      if (flashAvailable) {
        Serial.println(F("Erase command not supported for external flash."));
      } else {
        Serial.println(F("External flash not available."));
      }
    } else if (command == "list") {
      // List log files
      if (flashAvailable) {
        Serial.println(F("List command not supported for external flash."));
      } else {
        Serial.println(F("External flash not available."));
      }
    } else if (command == "stats") {
      // Show detailed storage statistics
      printStorageStatistics();
    } else if (command == "detail" || command == "detailed") {
      // Toggle detailed display mode
      displayMode = !displayMode;
      Serial.print(F("Detailed display mode: "));
      Serial.println(displayMode ? F("ON") : F("OFF"));
    } else if (command == "imu") {
      // Show detailed IMU data
      ICM_20948_print();
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
    } else if (command == "sensor") {
        enableSensorDebug = !enableSensorDebug;
        Serial.print(F("Sensor debug output: "));
        Serial.println(enableSensorDebug ? F("ENABLED") : F("DISABLED"));
    } else if (command == "quat") {
        enableQuaternionDebug = !enableQuaternionDebug;
        Serial.print(F("Quaternion debug output: "));
        Serial.println(enableQuaternionDebug ? F("ENABLED") : F("DISABLED"));
    }
  }
  
  // Read GPS data at GPS_POLL_INTERVAL
  if (millis() - lastGPSReadTime >= GPS_POLL_INTERVAL) {
    lastGPSReadTime = millis();
    gps_read();
    sensorsUpdated = true;

    // Check for automatic calibration opportunity
    if (!baroCalibrated && pDOP < 300) {  // pDOP is stored as integer * 100
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
        // Change LED to red to indicate failure
        pixels.setPixelColor(0, pixels.Color(50, 0, 0));
        pixels.show();
        delay(1000);
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
  
  // Read IMU data at IMU_POLL_INTERVAL
  if (millis() - lastIMUReadTime >= IMU_POLL_INTERVAL) {
    lastIMUReadTime = millis();
    ICM_20948_read();
    sensorsUpdated = true;
  }
  
  // Read accelerometer data at ACCEL_POLL_INTERVAL
  if (millis() - lastAccelReadTime >= ACCEL_POLL_INTERVAL) {
    lastAccelReadTime = millis();
    kx134_read();
    sensorsUpdated = true;
  }
  
  // Log data immediately after any sensor update
  if (sensorsUpdated) {
    WriteLogData(true);
    sensorsUpdated = false;
  }
  
  // Periodically check GPS connection
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
      Serial.println(F("SD card data flushed"));
    }
  }
  
  // Print status summary once per second
  if (millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
    lastDisplayTime = millis();
    printStatusSummary();
  }
  
  // Print detailed data less frequently
  if (displayMode && millis() - lastDetailedTime >= 5000) {
    lastDetailedTime = millis();
    ICM_20948_print();
  }
}



