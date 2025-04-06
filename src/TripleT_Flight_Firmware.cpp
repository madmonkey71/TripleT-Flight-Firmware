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


// Save SdFat FILE_READ/WRITE values before including LittleFS
#define SDFAT_FILE_READ O_RDONLY
#define SDFAT_FILE_WRITE (O_RDWR | O_CREAT | O_AT_END)
#undef FILE_READ
#undef FILE_WRITE

// Now include LittleFS
#include <LittleFS.h>

// Now include the GPS and sensor functions
#include "gps_config.h"
#include "gps_functions.h"  // Include GPS functions header
#include "ms5611_functions.h"
#include "kx134_functions.h"
#include "icm_20948_functions.h"  // Include ICM-20948 functions header
#include "utility_functions.h"    // Include utility functions header
#include "data_structures.h"  // Include our data structures

// Define variables declared as extern in utility_functions.h
String FileDateString;  // For log file naming
String LogDataString;   // For data logging
unsigned long currentTime;  // For timestamp
bool baroCalibrated = false;  // For barometric calibration status
const char* BOARD_NAME = "Teensy 4.1";

// Define LittleFS filesystem
LittleFS_Program flashFS;

// External declarations for sensor data
extern SFE_UBLOX_GNSS myGNSS;  // GPS object
extern float pressure;
extern float temperature;
extern float kx134_accel[3];

// Global variables to store ICM_20948 IMU data
float icm_accel[3] = {0};  // Accelerometer data (x, y, z)
float icm_gyro[3] = {0};   // Gyroscope data (x, y, z)
float icm_mag[3] = {0};    // Magnetometer data (x, y, z)
bool icm_data_available = false;
double icm_q1 = 0, icm_q2 = 0, icm_q3 = 0; // Quaternion data
uint16_t icm_data_header = 0; // To store what data types are available

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

// Storage configuration and thresholds
#define STORAGE_FULL_THRESHOLD 90  // Percentage at which storage is considered full
#define SD_CARD_TYPE_SD1 1
#define SD_CARD_TYPE_SD2 2
#define SD_CARD_TYPE_SDHC 3

// Storage variables
SdFat SD;
FsVolume volume;
bool sdCardAvailable = false;
bool flashAvailable = false;
bool internalFlashAvailable = false;
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

// Internal flash memory parameters for Teensy
#define FLASH_LOG_INTERVAL 250  // Log at 4Hz (every 200ms)

// Sensor polling intervals (in milliseconds)
#define GPS_POLL_INTERVAL 200       // Poll GPS at 5Hz
#define IMU_POLL_INTERVAL 100       // Poll IMU at 10Hz
#define BARO_POLL_INTERVAL 100      // Poll barometer at 10Hz
#define ACCEL_POLL_INTERVAL 100     // Poll accelerometer at 10Hz
#define DISPLAY_INTERVAL 1000       // Update display once per second
#define GPS_CHECK_INTERVAL 10000    // Check GPS connection every 10 seconds
#define STORAGE_CHECK_INTERVAL 30000 // Check storage space every 30 seconds

// Add new LittleFS variables
File flashLogFile;
char flashLogPath[32] = "/log.bin";
unsigned long recordCount = 0;

// Forward declarations for flash handling functions
String transferToSDCard();
String silentlyEraseNextFile();
String dumpInternalFlashData();

// Function to initialize the internal flash memory
bool initInternalFlash() {
  Serial.println(F("Initializing internal flash with LittleFS..."));
  
  #if defined(BOARD_STM32_THING_PLUS)
    // STM32-specific LittleFS initialization
    // On STM32, we use a different size for the flash storage
    if (!flashFS.begin(INTERNAL_FLASH_SIZE)) {
      Serial.println(F("Failed to initialize STM32 LittleFS!"));
      return false;
    }
    
    Serial.println(F("STM32 LittleFS initialized successfully"));
  #else
    // Teensy-specific LittleFS initialization
    if (!flashFS.begin(INTERNAL_FLASH_SIZE)) {
      Serial.println(F("Failed to initialize Teensy LittleFS!"));
      return false;
    }
    
    Serial.println(F("Teensy LittleFS initialized successfully"));
  #endif
  
  // Generate a new log file name with timestamp or counter
  if (SIV > 0) { // If we have satellite fix
    sprintf(flashLogPath, "/log_%04d%02d%02d_%02d%02d%02d.bin", 
      myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay(),
      myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond());
  } else {
    // Find an unused log file name
    int fileCount = 0;
    while (fileCount < 9999) {
      sprintf(flashLogPath, "/log_%04d.bin", fileCount);
      if (!flashFS.exists(flashLogPath)) {
        break;
      }
      fileCount++;
    }
  }
  
  // Create and open the log file
  flashLogFile = flashFS.open(flashLogPath, 1); // Use 1 directly instead of FILE_WRITE
  if (!flashLogFile) {
    Serial.println(F("Failed to create log file!"));
    return false;
  }
  
  // Write a header to the file
  char header[32];
  sprintf(header, "TRIPLET_v%.2f_%lu", TRIPLET_FLIGHT_VERSION, millis());
  flashLogFile.write((uint8_t*)header, 32);
  flashLogFile.flush();
  
  Serial.print(F("Internal flash log file created: "));
  Serial.println(flashLogPath);
  
  recordCount = 0;
  return true;
}

// Function to monitor storage space
bool checkStorageSpace() {
  static unsigned long lastCheckTime = 0;
  
  if (millis() - lastCheckTime < 30000) {
    return true;  // Just return true if we're not checking yet
  }
  lastCheckTime = millis();
  
  bool storageOK = true;  // Default to true
  
  // Check SD card free space
  if (sdCardAvailable) {
    uint32_t totalSpace = 0;
    uint32_t usedSpace = 0;
    
    // Get card info using SDFat
    if (!volume.begin(SD.card())) {
      Serial.println("Failed to initialize volume");
      storageOK = false;
    } else {
      totalSpace = volume.clusterCount() * volume.bytesPerCluster();
      usedSpace = (volume.clusterCount() - volume.freeClusterCount()) * volume.bytesPerCluster();
      
      // Count used space
      FsFile root = SD.open("/");
      if (root) {
        while (true) {
          FsFile entry = root.openNextFile();
          if (!entry) {
            break;
          }
          usedSpace += entry.size();
          entry.close();
        }
        root.close();
        
        Serial.print(F("SD Card: "));
        Serial.print(usedSpace / 1024);
        Serial.print(F(" KB used of "));
        Serial.print(totalSpace / 1024);
        Serial.println(F(" KB total"));
        
        if (totalSpace > 0) {
          uint8_t percentUsed = (usedSpace * 100UL) / totalSpace;
          Serial.print(F("SD Card usage: "));
          Serial.print(percentUsed);
          Serial.println(F("%"));
          
          if (percentUsed > STORAGE_FULL_THRESHOLD) {
            storageOK = false;
          }
        }
      }
    }
  }
  
  // Check internal flash space with LittleFS
  if (internalFlashAvailable && flashLogFile) {
    uint64_t totalBytes = INTERNAL_FLASH_SIZE;
    uint64_t usedBytes = 0;
    
    File root = flashFS.open("/");
    if (root && root.isDirectory()) {
      File file = root.openNextFile();
      while (file) {
        if (!file.isDirectory()) {
          usedBytes += file.size();
        }
        file = root.openNextFile();
      }
      root.close();
    }
    
    int percentUsed = (usedBytes * 100) / totalBytes;
    Serial.print(F("Internal flash used: "));
    Serial.print(usedBytes);
    Serial.print(F(" bytes ("));
    Serial.print(percentUsed);
    Serial.println(F("%)"));
    
    // Warn if running low on space
    if (percentUsed > 80) {
      Serial.println(F("WARNING: Internal flash storage is nearly full"));
    }
  }
  
  // Check external flash space if needed
  if (flashAvailable) {
    Serial.println(F("Serial Flash status check (simplified)"));
    // SerialFlash has limited API for checking space
    // This would need to be customized for your specific flash chip
  }
  return storageOK;  // Return the storage status
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
  if (!LogDataFile.open(fileName.c_str(), SDFAT_FILE_WRITE)) {
    Serial.println("Failed to create log file: " + fileName);
    return false;
  }

  // Write the CSV header
  LogDataFile.println("Timestamp,FixType,Sats,Lat,Long,Alt,AltMSL,Speed,Heading,pDOP,RTK,Pressure,Temperature,"
                     "KX134_AccelX,KX134_AccelY,KX134_AccelZ,"
                     "ICM_AccelX,ICM_AccelY,ICM_AccelZ,"
                     "ICM_GyroX,ICM_GyroY,ICM_GyroZ");

  // Flush the header to ensure it's written
  LogDataFile.sync();
  
  Serial.println("Created new log file: " + fileName);
  return true;
}

void WriteLogData(bool forceLog = false) {
  static unsigned long lastLogTime = 0;
  
  // Only log at specified intervals or when forced (after sensor reading)
  if (!forceLog && millis() - lastLogTime < FLASH_LOG_INTERVAL) {
    return;
  }
  lastLogTime = millis();
  
  // Format the data string using global GPS variables
  LogDataString = String(currentTime) + "," +
                  String(GPS_fixType) + "," +
                  String(SIV) + "," +
                  String(GPS_latitude, 7) + "," +
                  String(GPS_longitude, 7) + "," +
                  String(GPS_altitude, 2) + "," +
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
                  String(icm_mag[2], 4);

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

  // Write to Serial Flash
  if (flashAvailable && strlen(logFileName) > 0) {
    SerialFlashFile flashFile = SerialFlash.open(logFileName);
    if (flashFile) {
      // Find the end of the file to append
      flashFile.seek(flashFile.size());
      
      // Append the data
      char newLine[2] = "\n";
      flashFile.write((uint8_t*)LogDataString.c_str(), LogDataString.length());
      flashFile.write(newLine, 1);
      flashFile.close();
    }
  }
}

void formatNumber(float input, byte columns, byte places) 
{
  char buffer[20]; // Allocate space to store the formatted number string
  dtostrf(input, columns, places, buffer); // Convert float to string with specified columns and decimal places
  Serial.print(buffer); // Print the formatted number to the serial monitor
}

// Function to check if valid data exists in internal flash
bool checkInternalFlashData() {
  // Skip if LittleFS is not initialized
  if (!flashFS.begin(INTERNAL_FLASH_SIZE)) {
    return false;
  }
  
  // List files in the root directory to see if there are any logs
  File root = flashFS.open("/");
  if (!root || !root.isDirectory()) {
    return false;
  }
  
  bool hasLogFiles = false;
  File file = root.openNextFile();
  
  while (file) {
    // Check if this looks like a log file
    if (!file.isDirectory() && strstr(file.name(), ".bin") != NULL) {
      Serial.print(F("Found log file: "));
      Serial.print(file.name());
      Serial.print(F(" ("));
      Serial.print(file.size());
      Serial.println(F(" bytes)"));
      hasLogFiles = true;
    }
    file = root.openNextFile();
  }
  
  root.close();
  return hasLogFiles;
}

// Function to dump internal flash data to serial and return the name of the dumped file
String dumpInternalFlashData() {
  Serial.println(F("\n===== INTERNAL FLASH DATA DUMP ====="));
  
  // Skip if LittleFS is not initialized
  if (!flashFS.begin(INTERNAL_FLASH_SIZE)) {
    Serial.println(F("Failed to initialize LittleFS!"));
    return "";
  }
  
  // List all log files
  File root = flashFS.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println(F("Failed to open root directory!"));
    return "";
  }
  
  // Show available log files
  int fileIndex = 0;
  File file = root.openNextFile();
  
  // Store file names for later use
  String fileNames[10]; // Assume max 10 files for simplicity
  int fileCount = 0;
  
  while (file && fileCount < 10) {
    if (!file.isDirectory() && strstr(file.name(), ".bin") != NULL) {
      fileNames[fileCount] = String(file.name());
      Serial.print(fileIndex);
      Serial.print(F(": "));
      Serial.print(file.name());
      Serial.print(F(" ("));
      Serial.print(file.size());
      Serial.println(F(" bytes)"));
      fileIndex++;
      fileCount++;
    }
    file = root.openNextFile();
  }
  
  // If no files found
  if (fileIndex == 0) {
    Serial.println(F("No log files found!"));
    root.close();
    return "";
  }
  
  // Always dump the first file (index 0)
  int selectedFile = 0;
  Serial.print(F("Dumping file: "));
  Serial.println(fileNames[selectedFile]);
  
  // Reopen the root and find the selected file
  root.close();
  root = flashFS.open("/");
  fileIndex = 0;
  file = root.openNextFile();
  
  File logFile;
  String dumpedFileName = "";
  
  while (file) {
    if (!file.isDirectory() && strstr(file.name(), ".bin") != NULL) {
      if (fileIndex == selectedFile) {
        logFile = file;
        dumpedFileName = String(file.name());
        break;
      }
      fileIndex++;
    }
    file = root.openNextFile();
  }
  
  if (!logFile) {
    Serial.println(F("Failed to open selected file!"));
    root.close();
    return "";
  }
  
  // Read the header first (first 32 bytes)
  uint8_t headerBytes[32];
  logFile.read(headerBytes, 32);
  
  Serial.print(F("Header: "));
  for (int i = 0; i < 32; i++) {
    if (headerBytes[i] == 0) break;
    Serial.print((char)headerBytes[i]);
  }
  Serial.println();
  
  // Print column headers for CSV format
  Serial.println(F("Timestamp,FixType,Sats,Lat,Long,Alt,Speed,Pressure,Temp,KX134_AccelX,KX134_AccelY,KX134_AccelZ,ICM_AccelX,ICM_AccelY,ICM_AccelZ,ICM_GyroX,ICM_GyroY,ICM_GyroZ,Checksum"));
  
  // Dump each 46-byte record
  uint8_t recordBytes[46];
  int recordCount = 0;
  
  while (logFile.available() >= 46) {
    logFile.read(recordBytes, 46);
    
    // Extract record data
    
    // Timestamp (4 bytes)
    unsigned long timestamp = 0;
    timestamp |= (unsigned long)recordBytes[0];
    timestamp |= (unsigned long)recordBytes[1] << 8;
    timestamp |= (unsigned long)recordBytes[2] << 16;
    timestamp |= (unsigned long)recordBytes[3] << 24;
    
    // Fix type and satellite count
    byte fixType = recordBytes[4];
    byte satCount = recordBytes[5];
    
    // Latitude (4 bytes)
    long latitude = 0;
    latitude |= (long)recordBytes[6];
    latitude |= (long)recordBytes[7] << 8;
    latitude |= (long)recordBytes[8] << 16;
    latitude |= (long)recordBytes[9] << 24;
    
    // Longitude (4 bytes)
    long longitude = 0;
    longitude |= (long)recordBytes[10];
    longitude |= (long)recordBytes[11] << 8;
    longitude |= (long)recordBytes[12] << 16;
    longitude |= (long)recordBytes[13] << 24;
    
    // Altitude (4 bytes)
    long altitude = 0;
    altitude |= (long)recordBytes[14];
    altitude |= (long)recordBytes[15] << 8;
    altitude |= (long)recordBytes[16] << 16;
    altitude |= (long)recordBytes[17] << 24;
    
    // Speed (2 bytes)
    int16_t speed = 0;
    speed |= (int16_t)recordBytes[18];
    speed |= (int16_t)recordBytes[19] << 8;
    
    // Pressure (2 bytes) - stored as hPa * 10
    int16_t pressure = 0;
    pressure |= (int16_t)recordBytes[20];
    pressure |= (int16_t)recordBytes[21] << 8;
    
    // Temperature (2 bytes) - stored as °C * 100
    int16_t temperature = 0;
    temperature |= (int16_t)recordBytes[22];
    temperature |= (int16_t)recordBytes[23] << 8;
    
    // KX134 Accelerometer values (6 bytes) - stored as g * 1000
    int16_t kx_accelX = 0, kx_accelY = 0, kx_accelZ = 0;
    kx_accelX |= (int16_t)recordBytes[24];
    kx_accelX |= (int16_t)recordBytes[25] << 8;
    kx_accelY |= (int16_t)recordBytes[26];
    kx_accelY |= (int16_t)recordBytes[27] << 8;
    kx_accelZ |= (int16_t)recordBytes[28];
    kx_accelZ |= (int16_t)recordBytes[29] << 8;
    
    // ICM-20948 Accelerometer values (6 bytes) - stored as g * 1000
    int16_t icm_accelX = 0, icm_accelY = 0, icm_accelZ = 0;
    icm_accelX |= (int16_t)recordBytes[30];
    icm_accelX |= (int16_t)recordBytes[31] << 8;
    icm_accelY |= (int16_t)recordBytes[32];
    icm_accelY |= (int16_t)recordBytes[33] << 8;
    icm_accelZ |= (int16_t)recordBytes[34];
    icm_accelZ |= (int16_t)recordBytes[35] << 8;
    
    // ICM-20948 Gyroscope values (6 bytes) - stored as deg/s * 10
    int16_t icm_gyroX = 0, icm_gyroY = 0, icm_gyroZ = 0;
    icm_gyroX |= (int16_t)recordBytes[36];
    icm_gyroX |= (int16_t)recordBytes[37] << 8;
    icm_gyroY |= (int16_t)recordBytes[38];
    icm_gyroY |= (int16_t)recordBytes[39] << 8;
    icm_gyroZ |= (int16_t)recordBytes[40];
    icm_gyroZ |= (int16_t)recordBytes[41] << 8;
    
    // Reserved bytes [42-43] are skipped
    
    // Checksum (2 bytes)
    uint16_t checksum = 0;
    checksum |= (uint16_t)recordBytes[44];
    checksum |= (uint16_t)recordBytes[45] << 8;
    
    // Calculate and verify checksum
    uint16_t calculatedChecksum = 0;
    for (int i = 0; i < 44; i++) {
      calculatedChecksum += recordBytes[i];
    }
    
    // Format and print the record
    char buffer[250];
    sprintf(buffer, "%lu,%d,%d,%ld,%ld,%ld,%d,%.1f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f,%u%s",
            timestamp,
            fixType,
            satCount,
            latitude,
            longitude,
            altitude,
            speed,
            pressure / 10.0,    // Convert back to hPa
            temperature / 100.0, // Convert back to °C
            kx_accelX / 1000.0,  // Convert back to g
            kx_accelY / 1000.0,
            kx_accelZ / 1000.0,
            icm_accelX / 1000.0, // Convert back to g
            icm_accelY / 1000.0,
            icm_accelZ / 1000.0,
            icm_gyroX / 10.0,    // Convert back to deg/s
            icm_gyroY / 10.0,
            icm_gyroZ / 10.0,
            checksum,
            (calculatedChecksum != checksum) ? " (INVALID)" : "");
    
    Serial.println(buffer);
    recordCount++;
    
    // Add a brief delay to avoid overwhelming the serial buffer
    if (recordCount % 10 == 0) {
      delay(10);
    }
  }
  
  logFile.close();
  root.close();
  
  Serial.print(F("Total records: "));
  Serial.println(recordCount);
  Serial.println(F("===== END OF DATA DUMP =====\n"));
  
  return dumpedFileName;
}

// Function to transfer internal flash data to SD card
String transferToSDCard() {
  Serial.println(F("\n===== TRANSFERRING TO SD CARD ====="));
  
  if (!flashFS.begin(INTERNAL_FLASH_SIZE)) {
    Serial.println(F("Failed to initialize LittleFS!"));
    return "";
  }
  
  File root = flashFS.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println(F("Failed to open root directory!"));
    return "";
  }
  
  File file = root.openNextFile();
  String sourceFileName = "";
  
  while (file) {
    if (!file.isDirectory() && strstr(file.name(), ".bin") != NULL) {
      sourceFileName = String(file.name());
      break;
    }
    file = root.openNextFile();
  }
  
  if (sourceFileName.length() == 0) {
    Serial.println(F("No log files found!"));
    root.close();
    return "";
  }
  
  // Open the source file from internal flash
  File sourceFile = flashFS.open(sourceFileName.c_str(), FILE_READ);
  if (!sourceFile) {
    Serial.println(F("Failed to open source file!"));
    root.close();
    return "";
  }
  
  // Change extension from .bin to .csv
  String destFileName = String("FLASH_") + sourceFileName.substring(0, sourceFileName.length() - 4) + ".csv";
  FsFile destFile = SD.open(destFileName.c_str(), O_RDWR | O_CREAT | O_AT_END);
  if (!destFile) {
    Serial.println(F("Failed to create destination file!"));
    sourceFile.close();
    root.close();
    return "";
  }
  
  // Write CSV header
  destFile.println(F("Timestamp,FixType,Sats,Lat,Long,Alt,AltMSL,Speed,Heading,pDOP,RTK,"
                    "Pressure,Temperature,"
                    "KX134_AccelX,KX134_AccelY,KX134_AccelZ,"
                    "ICM_AccelX,ICM_AccelY,ICM_AccelZ,"
                    "ICM_GyroX,ICM_GyroY,ICM_GyroZ"));
  
  const size_t bufferSize = 512;
  uint8_t buffer[bufferSize];
  size_t bytesRead;
  char dataString[320]; // Buffer for CSV line
  
  while ((bytesRead = sourceFile.read(buffer, bufferSize)) > 0) {
    // Process each record in the buffer
    for (size_t i = 0; i < bytesRead; i += sizeof(LogDataStruct)) {
      if (i + sizeof(LogDataStruct) <= bytesRead) {
        LogDataStruct* data = (LogDataStruct*)&buffer[i];
        
        // Format the data as CSV
        int milliseconds = data->timestamp % 1000;
        sprintf(dataString, "%lu,%03d,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                data->timestamp / 1000, milliseconds,
                data->fixType, data->sats,
                data->latitude, data->longitude, data->altitude, data->altitudeMSL,
                data->speed, data->heading, data->pDOP / 100.0, data->rtk,
                data->pressure, data->temperature,
                data->kx134_x, data->kx134_y, data->kx134_z,
                data->icm_accel[0], data->icm_accel[1], data->icm_accel[2],
                data->icm_gyro[0], data->icm_gyro[1], data->icm_gyro[2],
                data->icm_mag[0], data->icm_mag[1], data->icm_mag[2]);
        
        // Write the CSV line to the destination file
        destFile.println(dataString);
      }
    }
  }
  
  destFile.flush();
  destFile.close();
  sourceFile.close();
  root.close();
  
  Serial.print(F("Transferred to SD card as: "));
  Serial.println(destFileName);
  return sourceFileName;
}

// Function to silently erase the next log file
String silentlyEraseNextFile() {
  if (!flashFS.begin(INTERNAL_FLASH_SIZE)) {
    return "";
  }
  
  File root = flashFS.open("/");
  if (!root || !root.isDirectory()) {
    return "";
  }
  
  File file = root.openNextFile();
  String fileName = "";
  
  while (file) {
    if (!file.isDirectory() && strstr(file.name(), ".bin") != NULL) {
      fileName = String(file.name());
      break;
    }
    file = root.openNextFile();
  }
  
  root.close();
  return fileName;
}

// Updated printStatusSummary function without VT100 codes
void printStatusSummary() {
  // Use multiple newlines and separators instead of screen clearing
  Serial.println(F("\n\n\n"));
  Serial.println(F("==================================="));
  Serial.print(F("TripleT Flight Firmware v"));
  Serial.print(TRIPLET_FLIGHT_VERSION);
  Serial.print(F(" | Runtime: "));
  unsigned long runtime = millis() / 1000;
  Serial.print(runtime / 60);
  Serial.print(F("m "));
  Serial.print(runtime % 60);
  Serial.println(F("s"));
  Serial.println(F("==================================="));

  gps_print();
  // SENSORS - Combine atmospheric and IMU data in a compact format
  ms5611_print();
  // KX134 Accelerometer data
  kx134_print();

  // ICM-20948 IMU data
  Serial.print(F("  ICM-20948: "));
  ICM_20948_print();
  
  // STORAGE STATUS - More compact representation
  Serial.println(F("STORAGE:"));
  Serial.print(F("  SD:"));
  Serial.print(sdCardAvailable ? F("OK") : F("--"));
  Serial.print(F(" | ExtFlash:"));
  Serial.print(flashAvailable ? F("OK") : F("--"));
  Serial.print(F(" | IntFlash:"));
  
  if (internalFlashAvailable && flashLogFile) {
    int percentUsed = (flashLogFile.size() * 100) / (INTERNAL_FLASH_SIZE - 4096);
    Serial.print(recordCount);
    Serial.print(F(" recs ("));
    Serial.print(percentUsed);
    Serial.println(F("%)"));
  } else if (internalFlashAvailable) {
    Serial.println(F("Ready"));
  } else {
    Serial.println(F("--"));
  }
  
  Serial.println(F("==================================="));
  Serial.println(F("Commands: help, dump, stats, imu, detail, calibrate"));
}

// Updated help message with more compact formatting
void printHelpMessage() {
  Serial.println(F("\n--- AVAILABLE COMMANDS ---"));
  Serial.println(F("dump   - Dump internal flash data"));
  Serial.println(F("erase  - Erase all internal flash data"));
  Serial.println(F("list   - List all log files"));
  Serial.println(F("stats  - Show storage statistics"));
  Serial.println(F("detail - Toggle detailed display"));
  Serial.println(F("imu    - Show detailed IMU data"));
  Serial.println(F("calibrate - Perform barometric calibration with GPS"));
  Serial.println(F("help   - Show this help message"));
  
  if (internalFlashAvailable && flashLogFile) {
    Serial.print(F("\nFlash: "));
    Serial.print(recordCount);
    Serial.print(F(" records, "));
    
    unsigned long fileSize = flashLogFile.size();
    unsigned long maxSize = INTERNAL_FLASH_SIZE - 4096;
    int percentUsed = (fileSize * 100) / maxSize;
    
    Serial.print(percentUsed);
    Serial.println(F("% used"));
  }
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
  
  // External Flash stats
  Serial.print(F("Ext Flash: "));
  if (flashAvailable) {
    Serial.print(F("Active"));
    if (strlen(logFileName) > 0) {
      Serial.print(F(" | Log: "));
      Serial.println(logFileName);
    } else {
    Serial.println();
  }
  } else {
    Serial.println(F("Not available"));
  }
  
  // Internal Flash stats
  Serial.print(F("Int Flash: "));
  if (internalFlashAvailable) {
    uint64_t totalBytes = INTERNAL_FLASH_SIZE;
    uint64_t usedBytes = 0;
    int fileCount = 0;
    
    File root = flashFS.open("/");
    if (root && root.isDirectory()) {
      File file = root.openNextFile();
      
      while (file) {
        if (!file.isDirectory()) {
          usedBytes += file.size();
          fileCount++;
        }
        file = root.openNextFile();
      }
      root.close();
      
      float percentUsed = (usedBytes * 100.0) / totalBytes;
      
      Serial.print(usedBytes);
      Serial.print(F(" of "));
      Serial.print(totalBytes);
      Serial.print(F(" bytes ("));
      Serial.print(percentUsed, 1);
      Serial.print(F("%) | Files: "));
      Serial.println(fileCount);
      
      if (flashLogFile) {
        Serial.print(F("Current: "));
        Serial.print(flashLogPath);
        Serial.print(F(" ("));
        Serial.print(recordCount);
        Serial.println(F(" records)"));
        
        // Estimate remaining capacity
        if (recordCount > 0) {
          float bytesPerRecord = flashLogFile.size() / (float)recordCount;
          uint64_t remainingBytes = totalBytes - usedBytes;
          uint64_t remainingRecords = remainingBytes / bytesPerRecord;
          float minutesRemaining = (remainingRecords / 5.0) / 60.0; // 5Hz logging rate
          
          Serial.print(F("Est. capacity: ~"));
          Serial.print(remainingRecords);
          Serial.print(F(" more records ("));
          Serial.print(minutesRemaining, 1);
          Serial.println(F(" min)"));
        }
      }
    }
  } else {
    Serial.println(F("Not available"));
  }
  
  Serial.println(F("-----------------------------"));
}

void setup() {
  // Initialize NeoPixel first for visual feedback
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red during startup
  pixels.show();
  
  // Startup Tone
  delay(500);
  tone(BUZZER_PIN, 2000); delay(50); noTone(BUZZER_PIN); delay(75);
  noTone(BUZZER_PIN);

  // Wait for the Serial monitor to be opened.
  Serial.begin(115200);
  delay(500); // Give the serial port time to initialize
  
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
    Wire.setClock(400000);
    
    Serial.println("Teensy SPI and I2C initialized");
  #endif

  // Initialize storage first
  Serial.println(F("Initializing storage..."));
  
  // Initialize SD card first since we may need it for flash data transfer
  sdCardAvailable = initSDCard();
  Serial.print(F("SD Card: "));
  Serial.println(sdCardAvailable ? F("Available") : F("Not available"));
  
  // Initialize internal flash
  Serial.println(F("Checking internal flash memory..."));
  if (!flashFS.begin(INTERNAL_FLASH_SIZE)) {
    Serial.println(F("Failed to initialize LittleFS!"));
    internalFlashAvailable = false;
  } else {
    internalFlashAvailable = true;
    Serial.println(F("Internal flash initialized successfully"));
    
    // Change LED to blue to indicate checking for flash data
    pixels.setPixelColor(0, pixels.Color(0, 0, 50)); // Blue during flash check
    pixels.show();
    
    // Check if there's data in the internal flash and handle it
    handleFlashDataCheck();
  }
  
  // Initialize external flash last
  flashAvailable = initSerialFlash();
  Serial.print(F("External Flash: "));
  Serial.println(flashAvailable ? F("Available") : F("Not available"));

  // Now initialize sensors
  Serial.println(F("\nInitializing sensors..."));
  kxAccel.begin();
  Serial.println(F("KX134 accelerometer initialized"));
  
  // Scan the I2C bus for devices
  scan_i2c();
  ms5611_init();
  gps_init();
  ICM_20948_init();
  kx134_init();
  
  // Only initialize internal flash if LittleFS initialization succeeded earlier
  if (internalFlashAvailable) {
    internalFlashAvailable = initInternalFlash();
  }
  
  if (sdCardAvailable || flashAvailable || internalFlashAvailable) {
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
      if (internalFlashAvailable) {
        if (checkInternalFlashData()) {
          dumpInternalFlashData();
        } else {
          Serial.println(F("No valid data found in internal flash."));
        }
      } else {
        Serial.println(F("Internal flash not available."));
      }
    } else if (command == "erase") {
      // Erase internal flash data
      if (internalFlashAvailable) {
        Serial.println(F("Erasing internal flash data..."));
        if (flashLogFile) {
          flashLogFile.close();
        }
        flashFS.format();
        Serial.println(F("All log files erased."));
        initInternalFlash();
      } else {
        Serial.println(F("Internal flash not available."));
      }
    } else if (command == "list") {
      // List log files
      if (internalFlashAvailable) {
        Serial.println(F("\nLog Files:"));
        File root = flashFS.open("/");
        if (!root || !root.isDirectory()) {
          Serial.println(F("Failed to open root directory!"));
        } else {
          File file = root.openNextFile();
          int fileCount = 0;
          
          while (file) {
            if (!file.isDirectory()) {
              Serial.print(file.name());
              Serial.print(F(" - "));
              Serial.print(file.size());
              Serial.println(F(" bytes"));
              fileCount++;
            }
            file = root.openNextFile();
          }
          
          if (fileCount == 0) {
            Serial.println(F("No files found."));
          }
          
          root.close();
        }
      } else {
        Serial.println(F("Internal flash not available."));
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
    
    // Flush internal flash data
    if (internalFlashAvailable && flashLogFile) {
      flashLogFile.flush();
      Serial.println(F("Internal flash data flushed"));
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



