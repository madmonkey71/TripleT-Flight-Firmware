// TripleT Flight Firmware
// Current Version: v0.30
// Current State: Alpha
// Last Updated: 24/04/2025
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
// Watchdog timer library
#include <Watchdog_t4.h>
#include <EEPROM.h>

// --- Include Core Configuration & Constants ---
#include "config.h"     // <<< ADDED Include for configuration parameters
#include "constants.h"  // Include for shared timing constants

// --- Create the watchdog object ---
#if ENABLE_WATCHDOG
WDT_T4<WDT1> watchdog;
#endif // ENABLE_WATCHDOG

// --- Set the version number ---
#define TRIPLET_FLIGHT_VERSION 0.30

// Additional variables for apogee detection
bool apogeeBackupTimer = false;
// bool useKX134 = true; // <<< REMOVED - Defined in config.h
int descendingCount = 0;

// Global variables for altitude and velocity calculations
float lastAltitude = 0.0;
unsigned long lastAltitudeTime = 0;
float verticalVelocity = 0.0;

// UKF initialization flag and output variables
bool ukfInitialized = false;
float ukf_pos    = 0.0f;
float ukf_vel    = 0.0f;
float ukf_accel  = 0.0f;

// Flight state definition
enum FlightState {
  STARTUP,       // Initial state, hardware initialization
  CALIBRATION,   // Calibration of sensors
  PAD_IDLE,      // On pad, waiting for arm command
  ARMED,         // Armed, waiting for liftoff
  BOOST,         // Motor burning, acceleration > threshold
  COAST,         // Free flight after motor burnout
  APOGEE,        // At peak altitude
  DROGUE_DEPLOY, // Drogue parachute deployment
  DROGUE_DESCENT,// Descending under drogue
  MAIN_DEPLOY,   // Main parachute deployment
  MAIN_DESCENT,  // Descending under main parachute
  LANDED,        // On ground
  RECOVERY,      // Post-flight data transmission
  ERROR          // Error state
};

// Global flight state variable
FlightState flightState = STARTUP;

// Global variable for data logging control
bool log_next_datapoint = false;

// State machine variables
float maxAltitudeReached = 0.0;
float launchAltitude = 0.0;
bool landingDetectedFlag = false;
unsigned long flightStartTime = 0;
unsigned long lastStateChangeTime = 0;
unsigned long boostEndTime = 0;
float baro_altitude_offset = 0.0; // Calibration offset

// Now include the GPS and sensor functions
#include "gps_config.h"
#include "gps_functions.h"  // Include GPS functions header
#include "ms5611_functions.h"
#include "utility_functions.h"    // Include utility functions header
#include "icm_20948_functions.h"  // Include ICM-20948 functions
#include "kx134_functions.h"  // Include KX134 functions
#include "ukf.h"  // Include UKF functions
// #include "constants.h" // <<< REMOVED - Moved include higher up

// Define variables declared as extern in utility_functions.h
String LogDataString;   // For data logging
unsigned long currentTime;  // For timestamp
bool baroCalibrated = false;  // For barometric calibration status
const char* BOARD_NAME = "Teensy 4.1";

// External declarations for sensor data
extern SFE_UBLOX_GNSS myGNSS;  // GPS object
extern float pressure;
extern float temperature;
extern float kx134_accel[3];
extern float icm_accel[3];
extern float icm_gyro[3];
extern float icm_mag[3];
extern float icm_temp;

// Define sensor objects
SparkFun_KX134 kx134Accel;  // Add KX134 accelerometer object definition
UKF ukf;  // Create UKF object for accelerometer data fusion

// Storage configuration
// Use SD_CARD_MIN_FREE_SPACE from utility_functions.h instead
// #define EXTERNAL_FLASH_MIN_FREE_SPACE 1024 * 1024  // <<< REMOVED - Moved to config.h

// Storage variables
SdFat SD;
bool sdCardAvailable = false;
bool sdCardPresent = false;   // Physical presence of SD card
bool sdCardMounted = false;   // Whether SD card is mounted
bool loggingEnabled = false;  // Whether logging is enabled
uint64_t availableSpace = 0;  // Available space on SD card in bytes
bool flashAvailable = false;

FsFile LogDataFile;

// Global storage variables
uint64_t totalSpace;
uint64_t usedSpace;

// Pre-allocated buffers instead of dynamic allocation
char logBufferEntries[MAX_LOG_ENTRIES][MAX_BUFFER_SIZE]; 
uint16_t logBufferLengths[MAX_LOG_ENTRIES];  // Store actual lengths to avoid strlen()
int bufferCount = 0;
unsigned long lastFlushTime = 0;
unsigned long fileOpenTime = 0;
bool fileIsOpen = false; 
// Additional global variables for data logging
// const int FLASH_CHIP_SELECT = 5; // <<< REMOVED - Moved to config.h
char logFileName[32] = ""; // To store the current log file name

// Sensor polling intervals (in milliseconds) - ALL MOVED to constants.h
// ... (defines already removed) ...

// Debug control variables
bool enableDetailedOutput = false;
bool enableSystemDebug = true;       // For system messages
bool enableIMUDebug = false;         // For IMU data
bool enableBaroDebug = false;        // For barometer data
bool enableStorageDebug = false;     // For SD card and storage operations
bool enableStatusSummary = false;    // Flag to control status summary output
bool enableICMRawDebug = false;      // Flag to control ICM raw data debug output (OFF by default)
bool displayMode = false;            // Toggle for compact vs detailed display
bool enableSerialCSV = false;        // Global variable to control serial CSV output (ON by default)

// Debug control
volatile bool detailedOutput = false;   // Add detailed data to be printed to console
volatile bool enableSensorDebug = false; // Enable detailed sensor-related debug output
volatile bool enableGPSDebug = false;   // Disable GPS library debug output completely

// Sensor status tracking
struct SensorStatus {
  bool isWorking;
  unsigned long lastValidReading;
  int failureCount;
  int consecutiveFailures;
};

// Initialize global sensor status variables
SensorStatus barometerStatus = {true, 0, 0, 0};
SensorStatus accelerometerStatus = {true, 0, 0, 0};
SensorStatus gyroscopeStatus = {true, 0, 0, 0};
SensorStatus magnetometerStatus = {true, 0, 0, 0};
SensorStatus gpsStatus = {true, 0, 0, 0};
bool kx134_initialized_ok = false; // Specific flag for KX134 init success

// Sensor ready flags - use functions to check sensor status
#define ms5611_ready ms5611Sensor.isConnected()
#define kx134_accel_ready (kx134_initialized_ok) // Use initialization flag instead
// Remove the macro and use the extern variable defined in ICM file
extern bool icm20948_ready;

// External declarations for ICM status
extern bool ICM_20948_isReady(); 

// State timeout tracking
unsigned long stateEntryTime = 0;

// Flight state storage structure
struct FlightStateData {
  FlightState state;        // Current flight state
  float launchAltitude;     // Stored launch altitude
  float maxAltitude;        // Maximum altitude reached
//  float currentAltitude;    // Current altitude (Removed - less critical to store, recalculate or use last reading)
  unsigned long flightStartTime; // Store flight start time
  unsigned long boostEndTime; // Store boost end time
  unsigned long timestamp;  // Timestamp of last save
  uint16_t signature;       // Validation signature
};

FlightStateData stateData; // Declare the global stateData variable
unsigned long lastStateSave = 0; // Declare and initialize the lastStateSave variable

// Global variable for watchdog status
bool watchdogTriggered = false;

// Forward declarations
void checkSensorStatus();
const char* getStateName(FlightState state); // <<< ADDED FORWARD DECLARATION
bool isSensorSuiteHealthy(FlightState currentState);
float GetAccelMagnitude();
float getBaroAltitude();
bool IsStable();
void detectBoostEnd();
bool detectApogee();
bool detectLanding();
void watchdogCallback();
void setLEDColor(uint8_t p, uint8_t r, uint8_t g, uint8_t b);
void WriteLogData(bool forceLog);
bool createNewLogFile();
void closeAllFiles();
void prepareForShutdown();
void ProcessUKF(); // <<< ADDED Back forward declaration - function defined in ukf.cpp
// EEPROM Functions Forward Declarations
void saveStateToEEPROM();
bool loadStateFromEEPROM();
void recoverFromPowerLoss();
float getUkfPosition(); // Add forward declaration for UKF position getter

// Watchdog timer handler
#if ENABLE_WATCHDOG
void watchdogHandler() {
  watchdogTriggered = true;
  // Emergency recovery actions could be added here
  Serial.println(F("WATCHDOG TRIGGERED - SYSTEM RESET"));
}
#endif // ENABLE_WATCHDOG

// Check and update status of all sensors
void checkSensorStatus() {

  // Check barometer
  static float lastPressure = pressure;
  if (ms5611_ready && fabs(pressure - lastPressure) < BAROMETER_ERROR_THRESHOLD) {
    barometerStatus.isWorking = true;
    barometerStatus.lastValidReading = millis();
    barometerStatus.failureCount = 0;
  } else if (ms5611_ready) {
    // Suspicious reading but sensor is connected
    barometerStatus.failureCount++;
    if (barometerStatus.failureCount > MAX_SENSOR_FAILURES) {
      barometerStatus.isWorking = false;
    }
  } else {
    barometerStatus.isWorking = false;
  }
  lastPressure = pressure;
  
  // Check accelerometer
  static float lastAccelMag = 0;
  float accelMag = GetAccelMagnitude();
  
  // Check if any valid accelerometer is providing reasonable data
  bool kx134_check = kx134_initialized_ok && kx134_accel_ready; // Check KX134 only if initialized
  bool icm_check = icm20948_ready; // Check ICM status
  
  if ((kx134_check || icm_check) && fabs(accelMag - lastAccelMag) < ACCEL_ERROR_THRESHOLD) {
    accelerometerStatus.isWorking = true;
    accelerometerStatus.lastValidReading = millis();
    accelerometerStatus.failureCount = 0;
  } else if (kx134_check || icm_check) { // Check if either sensor is connected, even if reading is suspicious
    // Suspicious reading but sensor is connected
    accelerometerStatus.failureCount++;
    if (accelerometerStatus.failureCount > MAX_SENSOR_FAILURES) {
      accelerometerStatus.isWorking = false;
    }
  } else {
    accelerometerStatus.isWorking = false;
  }
  lastAccelMag = accelMag;
  
  // Check gyroscope
  if (icm20948_ready) {
    gyroscopeStatus.isWorking = true;
    gyroscopeStatus.lastValidReading = millis();
  } else {
    gyroscopeStatus.isWorking = false;
  }
  
  // Check magnetometer
  if (icm20948_ready) {
    magnetometerStatus.isWorking = true;
    magnetometerStatus.lastValidReading = millis();
  } else {
    magnetometerStatus.isWorking = false;
  }
  
  // Check GPS
  if (myGNSS.getPVT()) {
    gpsStatus.isWorking = true;
    gpsStatus.lastValidReading = millis();
  } else if (GPS_fixType >= 3 && SIV >= 4) {
    // We have valid GPS data but getPVT() failed - still consider GPS working
    gpsStatus.isWorking = true;
    gpsStatus.lastValidReading = millis();
  } else if (millis() - gpsStatus.lastValidReading > GPS_TIMEOUT_MS) {
    gpsStatus.isWorking = false;
  }
  
  // Check if system is healthy overall
  if (!isSensorSuiteHealthy(flightState) && flightState != ERROR) {
    // If sensors are unhealthy and we're not already in ERROR state
    Serial.println(F("SENSOR HEALTH CHECK FAILED - ENTERING ERROR STATE"));
    flightState = ERROR;
  }
  
}

// Check if all critical sensors are healthy
bool isSensorSuiteHealthy(FlightState currentState) {
  bool result = false; 

  // Special case: If we're already in ERROR state, check if we can recover
  if (currentState == ERROR) {
    // Minimum requirements to exit ERROR state - consider a subset of sensors
    result = barometerStatus.isWorking && (accelerometerStatus.isWorking || gyroscopeStatus.isWorking);  
    return result;
  }

  // For ground states, we only need GPS and barometer to be working
  if (currentState <= ARMED) {
    result = barometerStatus.isWorking && gpsStatus.isWorking;
  }
  
  // For boost and coast, we need accelerometer, gyro, and barometer
  else if (currentState <= COAST) { // Changed else if to avoid processing multiple conditions
    result = barometerStatus.isWorking && accelerometerStatus.isWorking && gyroscopeStatus.isWorking;
  }
  
  // For apogee and descent, barometer is critical
  else if (currentState <= MAIN_DESCENT) { // Changed else if
    result = barometerStatus.isWorking;
  }
  
  // For landing detection, we need either accelerometer or barometer
  else if (currentState == LANDED) { // Changed else if
    result = barometerStatus.isWorking || accelerometerStatus.isWorking;
  }
  
  // Default case - require all sensors (should only apply to RECOVERY or unknown states now)
  else {
    result = barometerStatus.isWorking && accelerometerStatus.isWorking && 
             gyroscopeStatus.isWorking && gpsStatus.isWorking;
  }
  return result; 
}

// Get state name as string (for logging/debugging)
const char* getStateName(FlightState state) {
  switch (state) {
    case STARTUP: return "STARTUP";
    case CALIBRATION: return "CALIBRATION";
    case PAD_IDLE: return "PAD_IDLE";
    case ARMED: return "ARMED";
    case BOOST: return "BOOST";
    case COAST: return "COAST";
    case APOGEE: return "APOGEE";
    case DROGUE_DEPLOY: return "DROGUE_DEPLOY";
    case DROGUE_DESCENT: return "DROGUE_DESCENT";
    case MAIN_DEPLOY: return "MAIN_DEPLOY";
    case MAIN_DESCENT: return "MAIN_DESCENT";
    case LANDED: return "LANDED";
    case RECOVERY: return "RECOVERY";
    case ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

// Redundant apogee detection
bool detectApogee() {
  float currentAltitude = getBaroAltitude();
  bool apogeeDetected = false;
  
  // Method 1: Barometric detection (primary)
  if (ms5611_ready) {
    // Track maximum altitude
    if (currentAltitude > maxAltitudeReached) {
      maxAltitudeReached = currentAltitude;
      descendingCount = 0;
    }
    else if (currentAltitude < maxAltitudeReached - 1.0) {
      // Altitude is decreasing significantly
      descendingCount++;
      
      // Require multiple consecutive readings to confirm
      if (descendingCount >= APOGEE_CONFIRMATION_COUNT) {
        Serial.println(F("APOGEE DETECTED (barometric)"));
        apogeeDetected = true;
      }
    }
  }
  
  // Method 2: Accelerometer-based detection (backup)
  // Check if vertical acceleration is close to zero (indicating peak)
  if (!apogeeDetected && (kx134_accel_ready || icm20948_ready)) {
    float accel_z = kx134_accel_ready ? kx134_accel[2] : icm_accel[2];
    
    // Counter for near-zero acceleration readings
    static int accelNearZeroCount = 0; 
    const float accel_apogee_threshold = 0.1; // g threshold around zero

    // Check if absolute value of z-acceleration is below threshold
    if (fabs(accel_z) < accel_apogee_threshold) { 
      accelNearZeroCount++;
      // Require multiple consecutive readings near zero G
      if (accelNearZeroCount >= 5) { // Use a count similar to the old check for now
        Serial.println(F("APOGEE DETECTED (accelerometer - near zero G)"));
        apogeeDetected = true;
        accelNearZeroCount = 0; // Reset after detection
      }
    } else {
      // Reset count if acceleration moves away from zero
      accelNearZeroCount = 0;
    }
  }
  
  // Method 3: Time-based detection (last resort)
  if (!apogeeDetected && boostEndTime > 0) {
    // If we know when the boost phase ended, we can estimate apogee
    if (millis() - boostEndTime > BACKUP_APOGEE_TIME) {
      Serial.println(F("APOGEE DETECTED (time-based)"));
      apogeeDetected = true;
    }
  }
  
  return apogeeDetected;
}

// Redundant landing detection
bool detectLanding() {
  static int stableCount = 0;
  bool landingDetected = false;
  
  // Method 1: Accelerometer stability (primary)
  if (kx134_accel_ready || icm20948_ready) {
    float accel_magnitude = GetAccelMagnitude();
    
    // Check if acceleration is close to 1g (just gravity)
    if (accel_magnitude > 0.95 && accel_magnitude < 1.05) {
      stableCount++;
    } else {
      stableCount = 0;
    }
    
    // Require extended stability to confirm landing
    if (stableCount >= LANDING_CONFIRMATION_COUNT) {
      landingDetected = true;
    }
  }
  
  // Method 2: Barometric stability (backup)
  if (!landingDetected && ms5611_ready) {
    static float lastAltitude = 0.0;
    static int altitudeStableCount = 0;
    
    float currentAltitude = getBaroAltitude();
    
    // Check if altitude is stable
    if (fabs(currentAltitude - lastAltitude) < 1.0) {
      altitudeStableCount++;
    } else {
      altitudeStableCount = 0;
    }
    
    lastAltitude = currentAltitude;
    
    // Require extended stability to confirm landing
    if (altitudeStableCount >= LANDING_CONFIRMATION_COUNT) {
      landingDetected = true;
    }
  }
  
  // If landing detected, log it
  if (landingDetected && !landingDetectedFlag) {
    Serial.println(F("LANDING DETECTED"));
    landingDetectedFlag = true;
    // Save state immediately on landing detection
    saveStateToEEPROM();
  }
  
  return landingDetected;
}

// Track boost end for time-based apogee detection
void detectBoostEnd() {
  float accel_magnitude = GetAccelMagnitude();
  
  // When acceleration drops below threshold, record the time
  if (accel_magnitude < COAST_ACCEL_THRESHOLD && boostEndTime == 0) {
    boostEndTime = millis();
    Serial.println(F("BOOST END DETECTED"));
    Serial.print(F("Time since startup: "));
    Serial.print(boostEndTime / 1000.0);
    Serial.println(F(" seconds"));
  }
}

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
    
  // Close any previously open log file - this shouldn't be needed anymore, but keeping for safety
  if (LogDataFile) {
    LogDataFile.close();
    LogDataFile = FsFile(); // Reset the file object
  }
  
  // Create a unique filename based on timestamp
  char fileName[64];
  
  // First try to get valid GPS time
  if (myGNSS.getYear() >= 2023) {
    // Get date and time safely (uses valid GPS time)
    int year;
    byte month, day, hour, minute, second;
    getGPSDateTime(year, month, day, hour, minute, second);
    
    sprintf(fileName, "DATA_%04d%02d%02d_%02d%02d%02d.csv", 
      year, month, day, hour, minute, second);
  } 
  // Fallback 1: Use compile time if available
  else if (__DATE__[0] != '?') {
    // Parse the compile date and time macros
    char monthStr[4];
    int day, year;
    sscanf(__DATE__, "%s %d %d", monthStr, &day, &year);
    
    // Convert month string to number
    int month = 1;
    if (strcmp(monthStr, "Jan") == 0) month = 1;
    else if (strcmp(monthStr, "Feb") == 0) month = 2;
    else if (strcmp(monthStr, "Mar") == 0) month = 3;
    else if (strcmp(monthStr, "Apr") == 0) month = 4;
    else if (strcmp(monthStr, "May") == 0) month = 5;
    else if (strcmp(monthStr, "Jun") == 0) month = 6;
    else if (strcmp(monthStr, "Jul") == 0) month = 7;
    else if (strcmp(monthStr, "Aug") == 0) month = 8;
    else if (strcmp(monthStr, "Sep") == 0) month = 9;
    else if (strcmp(monthStr, "Oct") == 0) month = 10;
    else if (strcmp(monthStr, "Nov") == 0) month = 11;
    else if (strcmp(monthStr, "Dec") == 0) month = 12;
    
    // Parse time
    int hour, minute, second;
    sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &second);
    
    sprintf(fileName, "BUILD_%04d%02d%02d_%02d%02d%02d.csv", 
      year, month, day, hour, minute, second);
  }
  // Fallback 2: Use milliseconds and a random session ID
  else {
    static uint16_t sessionID = random(1000, 9999); // Generate random session ID at startup
    sprintf(fileName, "FALLBACK_S%04d_T%lu.csv", sessionID, millis());
  }
  
  Serial.print(F("Creating log file: "));
  Serial.println(fileName);
  
  // Try to open the file - if it fails, try with an ultra-fallback name
  FsFile tempFile;
  if (!tempFile.open(fileName, O_RDWR | O_CREAT | O_EXCL)) {
    Serial.println(F("Failed to create log file with first name, trying fallback"));
    
    // Ultra-fallback - try a super simple name
    sprintf(fileName, "TEMP_%lu.csv", micros() % 1000000);
    
    if (!tempFile.open(fileName, O_RDWR | O_CREAT | O_EXCL)) {
      Serial.println(F("Failed to create log file with fallback name"));
      return false;
    }
  else {
    tempFile.preAllocate(LOG_PREALLOC_SIZE);
    Serial.println(F("Log file created successfully"));
  }
  }
  
  // Prepare the CSV header string
  String headerString = F("GPSDate(UTC),GPSTime(UTC),FixType,Sats,Lat(deg),Long(deg),Alt(GPS),AltMSL(m),Speed(m/s),Heading(deg),pDOP,RTK,"
                       "Pressure(hPa),Temperature(C),"
                       "KX134_AccelX,KX134_AccelY,KX134_AccelZ,"
                       "ICM_AccelX,ICM_AccelY,ICM_AccelZ,"
                       "ICM_GyroX,ICM_GyroY,ICM_GyroZ,"
                       "ICM_MagX,ICM_MagY,ICM_MagZ,"
                       "ICM_Temp");
  
  // Add UKF columns to header
  headerString += F(",UKF_Position,UKF_Velocity,UKF_Acceleration");
  
  // Add the rest of the header
  headerString += F(",FlightStateCode,FlightStateName,StateEntryTime,StateElapsedTime,"
                    "AltitudeAGL,MaxAltitude,AccelMagnitude,VerticalVelocity,"
                    "FlightDuration,TimeSinceApogee,"
                    "SensorStatusFlags,BatteryVoltage,LoopCount,LogSeqNum,"
                    "SDCardStatus,FreeSpaceMB,"
                    "hAcc(m),vAcc(m)");
  
  // Write the header to the file
  tempFile.println(headerString);
  
  // Close the file - we'll reopen it each time we need to log
  tempFile.close();
  
  // Copy filename to global variable
  strcpy(logFileName, fileName);
  
  loggingEnabled = true;
  Serial.println(F("Log file created successfully"));
  
  // Save state after creating new log file
  saveStateToEEPROM();
  
  return true;
}

// Function to safely close all files
void closeAllFiles() {
  // Check if we have a persistently open file handle
  if (fileIsOpen) {
    Serial.println(F("Closing open log file"));
    LogDataFile.close();
    fileIsOpen = false;
  } else if (LogDataFile) {
    // This case handles any file opened without using our persistent file handling
    Serial.println(F("Closing temporary log file"));
    LogDataFile.close();
  }
  
  // Reset the file object to ensure it's in a clean state
  LogDataFile = FsFile();
  
  // Write any pending buffered data
  if (bufferCount > 0) {
    Serial.print(F("Writing "));
    Serial.print(bufferCount);
    Serial.println(F(" pending log entries before closing"));
    
    // Force a flush with current buffer content
    WriteLogData(true);
  }
  
  // Clear the filename and disable logging
  logFileName[0] = '\0';
  loggingEnabled = false;
  
  // Reset buffer state
  bufferCount = 0;
  lastFlushTime = 0;
  
  Serial.println(F("Logging disabled"));
}

// Function to handle system shutdown
void prepareForShutdown() {
  Serial.println(F("Preparing for shutdown..."));
  
  // Set LED to indicate shutdown is in progress
  setLEDColor(0, 0, 0, 50); // Blue during shutdown
  
  // Only save state to EEPROM if necessary (check if current state differs from saved state)
  if (stateData.state != flightState || 
      stateData.timestamp < millis() - 60000) { // Only save if more than 60s since last save
    saveStateToEEPROM();
  } else {
    Serial.println(F("State recently saved, skipping EEPROM write"));
  }
  
  // Handle open file efficiently
  if (fileIsOpen) {
    // If the file is already open, just close it directly
    LogDataFile.close();
    fileIsOpen = false;
  } else {
    // Otherwise use the standard closeAllFiles routine
    closeAllFiles();
  }
  
  // Brief beep with no delay
  tone(BUZZER_PIN, 1000, 100); // Non-blocking tone
  
  Serial.println(F("System shutdown complete"));
  // Function will return normally, allowing calling code to determine next steps
}

void WriteLogData(bool forceLog) {
  // Check if it's time to log data or forced logging is requested
  static unsigned long lastLogTime = 0;
  static uint16_t logSequenceCounter = 0;
  static uint32_t loopCount = 0; // Counter for loop iterations
  
  // Only log at specified intervals or when forced
  if (!forceLog && millis() - lastLogTime < LOG_INTERVAL) {
    return;
  }
  
  // Update log time
  lastLogTime = millis();
  
  // Update current time
  currentTime = millis();
  
  // Ensure variables are accessible in this scope
  extern float lastAltitude;
  extern unsigned long lastAltitudeTime;
  extern float verticalVelocity;
  extern bool ukfInitialized;
  
  // Calculate vertical velocity
  float currentAltitude = getBaroAltitude();
  if (lastAltitudeTime > 0) {
    float timeElapsed = (currentTime - lastAltitudeTime) / 1000.0; // seconds
    if (timeElapsed > 0) {
      verticalVelocity = (currentAltitude - lastAltitude) / timeElapsed;
    }
  }
  lastAltitude = currentAltitude;
  lastAltitudeTime = currentTime;
  
  // Increment sequence counter for every log entry
  logSequenceCounter++;
  loopCount++; // Increment loop counter
  
  // --- HIGH PERFORMANCE LOG CREATION ---
  char* logBuffer = NULL;
  
  // Select the appropriate buffer slot if we're using buffering
  if (sdCardAvailable && strlen(logFileName) > 0 && bufferCount < MAX_LOG_ENTRIES) {
    logBuffer = logBufferEntries[bufferCount];
  } else {
    // For output to serial only, use a temporary buffer
    static char tempBuffer[MAX_BUFFER_SIZE];
    logBuffer = tempBuffer;
  }
  
  // Fast direct buffer writing - minimize conversions
  char* p = logBuffer;

  // GPS date and time (combined)
  uint32_t gpsDate = 0;
  uint32_t gpsTime = 0;
  if (GPS_time_valid) {
    gpsDate = GPS_year * 10000 + GPS_month * 100 + GPS_day;
    gpsTime = GPS_hour * 10000 + GPS_minute * 100 + GPS_second;
  }
  p += sprintf(p, "%lu,%lu,", gpsDate, gpsTime);
  
  // Build the log string more efficiently with fewer sprintf calls
  // Timestamp and GPS basics (combine fields to reduce function calls)
  p += sprintf(p, "%lu,%d,%d", millis(), GPS_fixType, SIV);
  
  // GPS location
  p += sprintf(p, ",%d.%07d,%d.%07d", 
       (int)(GPS_latitude / 10000000), abs((int)(GPS_latitude % 10000000)),
       (int)(GPS_longitude / 10000000), abs((int)(GPS_longitude % 10000000)));
  
  // GPS altitude and speed - combine related fields
  p += sprintf(p, ",%d.%02d,%d.%02d,%d.%02d,%d.%02d,%u,%d", 
       (int)(GPS_altitude / 1000), abs((int)(GPS_altitude % 1000) / 10),
       (int)(GPS_altitudeMSL / 1000), abs((int)(GPS_altitudeMSL % 1000) / 10),
       (int)(GPS_speed / 1000), abs((int)(GPS_speed % 1000) / 10),
       (int)(GPS_heading / 10000), abs((int)(GPS_heading % 10000) / 100),
       pDOP, RTK);
  
  // Environmental data
  p += sprintf(p, ",%d.%02d,%d.%02d", 
       (int)pressure, abs((int)(pressure * 100) % 100),
       (int)temperature, abs((int)(temperature * 100) % 100));
  
  // Accelerometer data (KX134)
  p += sprintf(p, ",%d.%02d,%d.%02d,%d.%02d", 
       (int)kx134_accel[0], abs((int)(kx134_accel[0] * 100) % 100),
       (int)kx134_accel[1], abs((int)(kx134_accel[1] * 100) % 100),
       (int)kx134_accel[2], abs((int)(kx134_accel[2] * 100) % 100));
  
  // ICM sensor data (combine related fields)
  p += sprintf(p, ",%d.%02d,%d.%02d,%d.%02d", 
       (int)icm_accel[0], abs((int)(icm_accel[0] * 100) % 100),
       (int)icm_accel[1], abs((int)(icm_accel[1] * 100) % 100),
       (int)icm_accel[2], abs((int)(icm_accel[2] * 100) % 100));
  
  p += sprintf(p, ",%d.%02d,%d.%02d,%d.%02d", 
       (int)icm_gyro[0], abs((int)(icm_gyro[0] * 100) % 100),
       (int)icm_gyro[1], abs((int)(icm_gyro[1] * 100) % 100),
       (int)icm_gyro[2], abs((int)(icm_gyro[2] * 100) % 100));
  
  p += sprintf(p, ",%d.%02d,%d.%02d,%d.%02d", 
       (int)icm_mag[0], abs((int)(icm_mag[0] * 100) % 100),
       (int)icm_mag[1], abs((int)(icm_mag[1] * 100) % 100),
       (int)icm_mag[2], abs((int)(icm_mag[2] * 100) % 100));
  
  p += sprintf(p, ",%d.%02d", (int)icm_temp, abs((int)(icm_temp * 100) % 100));
  
  // UKF data (combined)
  if (ukfInitialized) {
    float pos = ukf.getPosition();
    float vel = ukf.getVelocity();
    float acc = ukf.getAcceleration();
    p += sprintf(p, ",%d.%02d,%d.%02d,%d.%02d", 
         (int)pos, abs((int)(pos * 100) % 100),
         (int)vel, abs((int)(vel * 100) % 100),
         (int)acc, abs((int)(acc * 100) % 100));
  } else {
    p += sprintf(p, ",0.00,0.00,0.00");
  }
  
  // Flight state information
  p += sprintf(p, ",%d,%s,%lu,%lu", 
       (int)flightState, getStateName(flightState),
       lastStateChangeTime, millis() - lastStateChangeTime);
  
  // Calculated values (combined)
  float baroAlt = getBaroAltitude() - launchAltitude;
  float accelMag = GetAccelMagnitude();
  p += sprintf(p, ",%d.%02d,%d.%02d,%d.%02d,%d.%02d", 
       (int)baroAlt, abs((int)(baroAlt * 100) % 100),
       (int)maxAltitudeReached, abs((int)(maxAltitudeReached * 100) % 100),
       (int)accelMag, abs((int)(accelMag * 100) % 100),
       (int)verticalVelocity, abs((int)(verticalVelocity * 100) % 100));
  
  // Flight performance metrics (combined)
  if (flightState > ARMED) {
    p += sprintf(p, ",%lu", millis() - stateData.flightStartTime);
    if (flightState > APOGEE) {
      unsigned long apogeeTime = stateData.flightStartTime + EXPECTED_APOGEE_TIME;
      p += sprintf(p, ",%lu", millis() - apogeeTime);
    } else {
      p += sprintf(p, ",0");
    }
  } else {
    p += sprintf(p, ",0,0");
  }
  
  // System health data
  uint8_t sensorFlags = 0;
  sensorFlags |= (barometerStatus.isWorking ? 0x01 : 0);
  sensorFlags |= (accelerometerStatus.isWorking ? 0x02 : 0);
  sensorFlags |= (gyroscopeStatus.isWorking ? 0x04 : 0);
  sensorFlags |= (magnetometerStatus.isWorking ? 0x08 : 0);
  sensorFlags |= (gpsStatus.isWorking ? 0x10 : 0);
  
  // Battery, performance, and storage metrics (combined)
  float batteryVoltage = 0.0; // Placeholder
  p += sprintf(p, ",%u,%d.%02d,%u,%d,%lu,%d.%02d,%d.%02d", 
       sensorFlags,
       (int)batteryVoltage, abs((int)(batteryVoltage * 100) % 100),
       logSequenceCounter,
       sdCardAvailable ? 1 : 0,
       (unsigned long)(availableSpace / (1024ULL * 1024ULL)),
       (int)(GPS_hAcc / 1000), abs((int)((GPS_hAcc % 1000) / 10)),
       (int)(GPS_vAcc / 1000), abs((int)((GPS_vAcc % 1000) / 10)));
  
  // Calculate buffer length - avoid using strlen() later
  size_t bufferLength = p - logBuffer;
  
  // Ensure buffer wasn't overflowed
  if (bufferLength >= MAX_BUFFER_SIZE) {
    Serial.println(F("ERROR: Log buffer overflow!"));
    bufferLength = MAX_BUFFER_SIZE - 1; // Ensure null termination fits
    logBuffer[bufferLength] = '\0';
  }
  
  // --- START MODIFICATION ---
  // Output to Serial *first* if enabled
  if (enableSerialCSV) {
    Serial.println(logBuffer);
  }
  // --- END MODIFICATION ---
  
  // --- START SD CARD LOGGING BLOCK ---
  // Only buffer and write to SD card if NOT disabled
  if (!DISABLE_SDCARD_LOGGING) {
    if (sdCardAvailable && strlen(logFileName) > 0) {
      // Store the actual length for faster processing later (only if buffering)
      if (bufferCount < MAX_LOG_ENTRIES) { // Check if we have space before storing length
          logBufferLengths[bufferCount] = bufferLength;
      } // else: If buffer is full, this log entry might be dropped from SD card but was still printed to serial

      // Add entry to buffer count (only if space available)
      if (bufferCount < MAX_LOG_ENTRIES) {
          bufferCount++;
      } else {
          // Optional: Log buffer full warning (but avoid spamming)
          static unsigned long lastBufferFullWarn = 0;
          if (millis() - lastBufferFullWarn > 1000) {
              Serial.println(F("WARN: Log buffer full, SD card write may lag!"));
              lastBufferFullWarn = millis();
          }
      }

      // Determine if we should flush the buffer
      bool shouldFlush = forceLog ||
                        (millis() - lastFlushTime >= FLUSH_INTERVAL) ||
                        (bufferCount >= MAX_LOG_ENTRIES);

      // Critical flight states should always force a flush for data safety
      if (flightState == BOOST || flightState == APOGEE || flightState == DROGUE_DEPLOY ||
          flightState == MAIN_DEPLOY || flightState == ERROR) {
        shouldFlush = true;
      }

      // Handle file open/close and writing based on activity
      if (shouldFlush && bufferCount > 0) {
        // Open the file if it's not already open
        if (!fileIsOpen) {
          LogDataFile = FsFile(); // Clear any existing handle
          if (LogDataFile.open(logFileName, O_RDWR | O_AT_END)) {
            fileIsOpen = true;
            fileOpenTime = millis();
          } else {
            Serial.print(F("ERROR: Could not open log file: "));
            Serial.println(logFileName);
            bufferCount = 0; // Clear buffer in case of error
            return; // Exit if file open fails
          }
        }

        // Write all buffered entries efficiently
        static char tempNewline[] = "\r\n";
        for (int i = 0; i < bufferCount; i++) {
          LogDataFile.write((uint8_t*)logBufferEntries[i], logBufferLengths[i]);
          LogDataFile.write((uint8_t*)tempNewline, 2);
        }

        // Reset buffer count after writing
        bufferCount = 0;
        lastFlushTime = millis();

        // Only close the file if we've had it open for a while and we're not in a critical state
        bool criticalState = (flightState == BOOST || flightState == APOGEE ||
                              flightState == DROGUE_DEPLOY || flightState == MAIN_DEPLOY);

        if (fileIsOpen && !criticalState && (millis() - fileOpenTime > SD_MAX_OPEN_TIME)) { // SD_MAX_OPEN_TIME needs definition
          LogDataFile.close();
          fileIsOpen = false;
        }
      }
    }
  } // --- END SD CARD LOGGING BLOCK ---
}

void formatNumber(float input, byte columns, byte places) 
{
  char buffer[20]; // Allocate space to store the formatted number string
  dtostrf(input, columns, places, buffer); // Convert float to string with specified columns and decimal places
  Serial.print(buffer); // Print the formatted number to the serial monitor
}

// Main flight state processor
void ProcessFlightState() {
  static FlightState prevState = STARTUP; // Initialize prevState for first check
  
  // Check for state timeouts
  unsigned long stateTime = millis() - stateEntryTime;
  if (stateTime > STATE_TIMEOUT_MS && flightState != STARTUP && flightState != CALIBRATION && 
      flightState != PAD_IDLE && flightState != RECOVERY && flightState != ERROR && flightState != ARMED) {
    // If we're in a critical state and a timeout occurs, transition to ERROR
    Serial.println(F("ERROR: State timeout occurred"));
    flightState = ERROR;
    saveStateToEEPROM(); // Save error state
  }
  
  // Update last state change time if state changed
  if (flightState != prevState) {
    lastStateChangeTime = millis();
    stateEntryTime = millis();
    
    // Log state change
    Serial.print(F("Flight state changed: "));
    Serial.print(getStateName(prevState));
    Serial.print(F(" -> "));
    Serial.println(getStateName(flightState));
    
    // Initialize state
    switch (flightState) {
      // REMOVED STARTUP and CALIBRATION cases - Handled in setup()
      case STARTUP:
      case CALIBRATION:
      case PAD_IDLE:
        // Pad idle state initialization
        setLEDColor(0, 0, 50, 0); // Green
        // Store launch altitude
        launchAltitude = getBaroAltitude();
        break;
      case ARMED:
        // Armed state initialization
        pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red
        pixels.show();
        // Arming beep is handled by the command processor
        break;
      case BOOST:
        // Boost state initialization
        setLEDColor(0, 50, 30, 0); // Orange
        flightStartTime = millis();
        Serial.println(F("LIFTOFF!"));
        // Clear any previous boost end time
        boostEndTime = 0;
        break;
      case COAST:
        // Coast state initialization
        setLEDColor(0, 0, 0, 50); // Blue
        break;
      case APOGEE:
        // Apogee state initialization
        setLEDColor(0, 50, 0, 50); // Purple
        Serial.print(F("Apogee detected at altitude: "));
        Serial.print(getBaroAltitude() - launchAltitude);
        Serial.println(F(" meters AGL"));
        if (BUZZER_OUTPUT) {
          tone(BUZZER_PIN, 3000, 200); // Apogee beep
        }
        break;
      case DROGUE_DEPLOY:
        // Drogue deploy state initialization
        setLEDColor(0, 50, 50, 0); // Yellow
        if (DROGUE_PRESENT) {
          Serial.println(F("Deploying drogue parachute"));
          // Add code to fire drogue parachute pyro channel here
        }
        break;
      case DROGUE_DESCENT:
        // Drogue descent state initialization
        setLEDColor(0, 0, 50, 50); // Cyan
        break;
      
      case MAIN_DEPLOY:
        // Main deploy state initialization
        setLEDColor(0, 50, 25, 0); // Orange
        if (MAIN_PRESENT) {
          Serial.println(F("Deploying main parachute"));
          // Add code to fire main parachute pyro channel here
        }
        break;
      case MAIN_DESCENT:
        // Main descent state initialization
        setLEDColor(0, 0, 30, 0); // Green
        break;
      case LANDED:
        // Landed state initialization
        setLEDColor(0, 0, 50, 0); // Green
        Serial.println(F("Landed safely!"));
        if (BUZZER_OUTPUT) {
          // Play landing melody
          tone(BUZZER_PIN, 2000, 100); delay(150);
          tone(BUZZER_PIN, 2500, 100); delay(150);
          tone(BUZZER_PIN, 3000, 100);
        }
        break;
      case RECOVERY:
        // Recovery state initialization
        setLEDColor(0, 0, 50, 0); // Green
        Serial.println(F("Entering recovery mode"));
        break;
        
      case ERROR:
        // Error state initialization
        setLEDColor(0, 50, 0, 0); // Red
        break;
    }
    
    prevState = flightState;
    // Save state immediately on any state transition
    saveStateToEEPROM();
  }
  
  // STEP 2: State-specific processing
  switch (flightState) {
    case STARTUP:
      break;
    case CALIBRATION:
      break;
    case PAD_IDLE:
      // Check for arm command comes from processCommand
      // Nothing to do here in automatic processing
      break;
    case ARMED:
      // Check for liftoff
      if (kx134_accel_ready || icm20948_ready) {
        float accel_magnitude = GetAccelMagnitude();
        
        // If acceleration exceeds threshold, transition to BOOST
        if (accel_magnitude > BOOST_ACCEL_THRESHOLD) {
          Serial.println(F("LIFTOFF DETECTED"));
          Serial.print(F("Acceleration: "));
          Serial.print(accel_magnitude);
          Serial.println(F(" g"));
          
          flightState = BOOST;
          // Force an immediate log entry
          log_next_datapoint = true;
        }
      }
      break;
      
    case BOOST:
      // Check for motor burnout
      detectBoostEnd();
      
      // If boost end detected, transition to COAST
      if (boostEndTime > 0) {
        flightState = COAST;
      }
      break;
      
    case COAST:
      // Check for apogee
      if (detectApogee()) {
        flightState = APOGEE;
        // Force an immediate log entry
        log_next_datapoint = true;
        // REMOVED: saveStateToEEPROM(); // Save handled by state transition logic above
      }
      break;
      
    case APOGEE:
      // Short delay at apogee for stability
      if (millis() - lastStateChangeTime > 500) {
        flightState = DROGUE_DEPLOY;
      }
      break;
      
    case DROGUE_DEPLOY:
      // Short delay to ensure pyro has fired
      if (millis() - lastStateChangeTime > 1500) {
        flightState = DROGUE_DESCENT;
      }
      break;
      
    case DROGUE_DESCENT: {
      // Check for main deployment altitude
      float altitude = getBaroAltitude();
      float AGL = altitude - launchAltitude;
      
      if (AGL < MAIN_DEPLOY_ALTITUDE) {
        flightState = MAIN_DEPLOY;
        // Force an immediate log entry
        log_next_datapoint = true;
      }
      break;
    }
      
    case MAIN_DEPLOY:
      // Short delay to ensure pyro has fired
      if (millis() - lastStateChangeTime > 1500) {
        flightState = MAIN_DESCENT;
      }
      break;
      
    case MAIN_DESCENT:
      // Check for landing
      if (detectLanding()) {
        flightState = LANDED;
        // Force an immediate log entry
        log_next_datapoint = true;
      }
      break;
      
    case LANDED:
      // Transition to recovery after a short delay
      if (millis() - lastStateChangeTime > 10000) {
        flightState = RECOVERY;
      }
      break;
      
    case RECOVERY:
      // Terminal state, stay here until power off or reset
      // Blink LED to indicate recovery mode
      if ((millis() / 500) % 2) {
        setLEDColor(0, 0, 20, 0); // Green
      } else {
        setLEDColor(0, 0, 0, 0); // Off
      }
      pixels.show();
      break;
      
    case ERROR:
      // Error state, blink red LED
      if ((millis() / 250) % 2) {
        setLEDColor(0, 20, 0, 0); // Red
      } else {
        setLEDColor(0, 0, 0, 0); // Off
      }
      pixels.show();
      
      // Check if system has recovered and can exit ERROR state
      // Check once per second to reduce log spam
      static unsigned long lastErrorRecoveryCheck = 0;
      if (millis() - lastErrorRecoveryCheck > 1000) {
        lastErrorRecoveryCheck = millis();
        
        // Use the special case in isSensorSuiteHealthy() to check if we can exit ERROR
        if (isSensorSuiteHealthy(ERROR)) {
          Serial.println(F("System has recovered from ERROR state. Transitioning to PAD_IDLE."));
          flightState = PAD_IDLE;
          saveStateToEEPROM(); // Save the recovered state immediately
        }
      }
      break;
  }
}

// Updated printStatusSummary function without VT100 codes
void printStatusSummary() {
  Serial.println(F("\n=== STATUS SUMMARY ==="));
  
  // Current time
  unsigned long currentMillis = millis();
  unsigned long seconds = currentMillis / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  Serial.printf("Time: %02d:%02d:%02d\n", hours % 24, minutes % 60, seconds % 60);
  
  // Flight state section
  Serial.println("\n--- Flight State ---");
  Serial.printf("Current State: %s\n", getStateName(flightState));
  if (flightState >= BOOST) {
    // Show flight timers if we're in flight
    Serial.printf("Flight Time: %02d:%02d\n", 
                (millis() - flightStartTime) / 60000, 
                ((millis() - flightStartTime) / 1000) % 60);
                
    if (flightState >= APOGEE) {
      // Show max altitude if we've reached apogee
      Serial.printf("Max Altitude: %.2f m AGL\n", maxAltitudeReached - launchAltitude);
    }
  }
  
  // GPS section
  Serial.println("\n--- GPS Status ---");
  Serial.printf("Fix Type: %d\n", GPS_fixType);
  Serial.printf("Sats: %d\n", SIV);
  Serial.printf("pDOP: %.2f\n", pDOP / 100.0);
  
  // Barometer status
  Serial.println("\n--- Barometer Status ---");
  Serial.printf("Pressure: %.2f hPa\n", pressure);
  Serial.printf("Temperature: %.2f°C\n", temperature);
  Serial.printf("Altitude: %.2f m\n", getBaroAltitude());
  
  // Accelerometer status
  Serial.println("\n--- Accelerometer Status ---");
  float accel_magnitude = GetAccelMagnitude();
  Serial.printf("Acceleration: %.2f g\n", accel_magnitude);
  
  // Gyroscope status
  Serial.println("\n--- Gyroscope Status ---");
  if (gyroscopeStatus.isWorking) {
    Serial.println("Gyroscope: OK");
  } else {
    Serial.println("Gyroscope: FAULT");
  }
  
  // Storage status
  Serial.println("\n--- Storage Status ---");
  Serial.printf("SD Card: %s\n", sdCardAvailable ? "OK" : "UNAVAILABLE");
  Serial.printf("External Flash: %s\n", flashAvailable ? "OK" : "UNAVAILABLE");
  
  // Flight metrics if in flight
  if (flightState >= BOOST) {
    Serial.println("\n--- Flight Metrics ---");
    if (flightState >= BOOST) {
      Serial.printf("Flight Time: %02d:%02d\n", 
                  (millis() - flightStartTime) / 60000, 
                  ((millis() - flightStartTime) / 1000) % 60);
    }
    
    if (boostEndTime > 0) {
      Serial.printf("Boost Duration: %02d:%02d\n", 
                  (boostEndTime - flightStartTime) / 60000, 
                  ((boostEndTime - flightStartTime) / 1000) % 60);
    }
    
    if (flightState >= APOGEE) {
      Serial.printf("Max Altitude: %.2f m AGL\n", maxAltitudeReached - launchAltitude);
      
      if (baroCalibrated) {
        Serial.printf("AGL: %.2f m\n", maxAltitudeReached - launchAltitude);
      }
    }
  }
  
  // System health
  Serial.println("\n--- System Health ---");
  if (isSensorSuiteHealthy(flightState)) {
    Serial.println("System: OK");
  } else {
    Serial.println("System: DEGRADED");
  }
  
  // Watchdog status
  Serial.println("\n--- Watchdog Status ---");
  Serial.println(watchdogTriggered ? F("Watchdog: TRIGGERED") : F("Watchdog: OK"));
  
  Serial.println(F("---------------------------"));
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
  Serial.println(F("  7 - Toggle high-frequency debug"));
  Serial.println(F("  9 - Initiate Shutdown"));
  
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
  Serial.println(F("  status - Display system status"));
  Serial.println(F("  calibrate - Calibrate barometer with GPS"));
  Serial.println(F("  arm - Arm the flight computer"));
  
  // Test commands
  Serial.println(F("\nTest Commands:"));
  Serial.println(F("  test_error - Simulate sensor error and force ERROR state"));
  Serial.println(F("  test_watchdog - Simulate watchdog timeout"));
  Serial.println(F("  clear_errors - Reset all error flags and return to normal operation"));
  Serial.println(F("  status_sensors - Display detailed sensor status information"));
  
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
    Serial.println(F("[DEBUG] Entering processCommand")); // <<< ADDED DEBUG
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
                case '9': // Shutdown
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
                        setLEDColor(0, 50, 0, 50);
                        // ms5611_calibrate_with_gps();
                        // Serial.println(F("Starting barometric calibration with GPS..."));
                        // Serial.println(F("Waiting for good GPS fix (pDOP < 3.0)..."));
                        
                        if (ms5611_calibrate_with_gps(30000)) {  // Wait up to 60 seconds for calibration
                            Serial.println(F("Barometric calibration successful!"));
                            baroCalibrated = true;
                            // Change LED to green to indicate success
                            setLEDColor(0, 0, 50, 0);
                            delay(1000);
                        } else {
                            Serial.println(F("Barometric calibration timed out or failed."));
                            // Change LED to red to indicate failure
                            setLEDColor(0, 50, 0, 0);

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
        setLEDColor(0, 50, 0, 50);
        
        if (ms5611_calibrate_with_gps(60000)) {  // Wait up to 60 seconds for calibration
          Serial.println(F("Barometric calibration successful!"));
          baroCalibrated = true;
          // Change LED to green to indicate success
          setLEDColor(0, 0, 50, 0);
          delay(1000);
        } else {
          Serial.println(F("Barometric calibration timed out or failed."));
          // Change LED to red to indicate failure
          setLEDColor(0, 50, 0, 0);
           delay(1000);
        }
      } else {
        Serial.println(F("Barometric calibration has already been performed."));
      }
    } else if (command == "help") {
      // Show help message
      printHelpMessage();
    } else if (command == "arm") {
      // Arm the flight computer (transition from PAD_IDLE to ARMED)
      if (flightState == PAD_IDLE) {
        if (baroCalibrated) {
          flightState = ARMED;
          Serial.println(F("*** ARMED! ***"));          
        } else {
          Serial.println(F("ERROR: Cannot arm - barometer not calibrated."));
          Serial.println(F("Run 'calibrate' command first."));
        }
      } else {
        Serial.println(F("ERROR: Cannot arm - not in PAD_IDLE state."));
        Serial.println(F("Current state: "));
        Serial.println(getStateName(flightState));
      }
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
    // Error detection and watchdog test commands
    else if (command == "test_error") {
      // Simulate a sensor error by forcing the barometer to be marked as not working
      barometerStatus.isWorking = false;
      barometerStatus.failureCount = MAX_SENSOR_FAILURES + 1;
      Serial.println(F("TEST: Simulating barometer failure"));
      
      // Force sensor health check
      if (!isSensorSuiteHealthy(flightState)) {
        Serial.println(F("TEST: Sensor suite health check failed"));
        flightState = ERROR;
        Serial.println(F("TEST: Entering ERROR state"));
        // Set LED to red
        setLEDColor(0, 255, 0, 0); // Red
      }
    }
    else if (command == "test_watchdog") {
      // Simulate a watchdog timeout by triggering the callback directly
      Serial.println(F("TEST: Simulating watchdog timeout in 3 seconds..."));
      delay(1000);
      Serial.println(F("TEST: 2 seconds..."));
      delay(1000);
      Serial.println(F("TEST: 1 second..."));
      delay(1000);
      
      // Call the watchdog handler directly
      watchdogCallback();
      
      Serial.println(F("TEST: Watchdog callback executed"));
      Serial.print(F("Current state: "));
      Serial.println(getStateName(flightState));
    }
    else if (command == "clear_errors") {
      // Reset all error flags and return to normal operation
      Serial.println(F("Clearing all error flags"));
      
      // Reset sensor statuses
      barometerStatus.isWorking = ms5611Sensor.isConnected();
      barometerStatus.failureCount = 0;
      barometerStatus.consecutiveFailures = 0;
      
      accelerometerStatus.isWorking = (kx134_accel_ready || icm20948_ready);
      accelerometerStatus.failureCount = 0;
      accelerometerStatus.consecutiveFailures = 0;
      
      gyroscopeStatus.isWorking = icm20948_ready;
      gyroscopeStatus.failureCount = 0;
      gyroscopeStatus.consecutiveFailures = 0;
      
      magnetometerStatus.isWorking = icm20948_ready;
      magnetometerStatus.failureCount = 0;
      magnetometerStatus.consecutiveFailures = 0;
      
      gpsStatus.isWorking = myGNSS.getPVT();
      gpsStatus.failureCount = 0;
      gpsStatus.consecutiveFailures = 0;
      
      // If we're in ERROR state, return to PAD_IDLE
      if (flightState == ERROR) {
        flightState = PAD_IDLE;
        Serial.println(F("Returning to PAD_IDLE state from ERROR"));
        setLEDColor(0, 0, 50, 0); // Green
      }
    }
    else if (command == "status_sensors") {
      // Print detailed sensor status information
      Serial.println(F("=== Sensor Status ==="));
      Serial.print(F("Barometer: "));
      Serial.print(barometerStatus.isWorking ? F("OK") : F("FAILED"));
      Serial.print(F(" (Failures: "));
      Serial.print(barometerStatus.failureCount);
      Serial.print(F(", Consecutive: "));
      Serial.print(barometerStatus.consecutiveFailures);
      Serial.println(F(")"));
      
      Serial.print(F("Accelerometer: "));
      Serial.print(accelerometerStatus.isWorking ? F("OK") : F("FAILED"));
      Serial.print(F(" (Failures: "));
      Serial.print(accelerometerStatus.failureCount);
      Serial.print(F(", Consecutive: "));
      Serial.print(accelerometerStatus.consecutiveFailures);
      Serial.println(F(")"));
      
      Serial.print(F("Gyroscope: "));
      Serial.print(gyroscopeStatus.isWorking ? F("OK") : F("FAILED"));
      Serial.print(F(" (Failures: "));
      Serial.print(gyroscopeStatus.failureCount);
      Serial.print(F(", Consecutive: "));
      Serial.print(gyroscopeStatus.consecutiveFailures);
      Serial.println(F(")"));
      
      Serial.print(F("Magnetometer: "));
      Serial.print(magnetometerStatus.isWorking ? F("OK") : F("FAILED"));
      Serial.print(F(" (Failures: "));
      Serial.print(magnetometerStatus.failureCount);
      Serial.print(F(", Consecutive: "));
      Serial.print(magnetometerStatus.consecutiveFailures);
      Serial.println(F(")"));
      
      Serial.print(F("GPS: "));
      Serial.print(gpsStatus.isWorking ? F("OK") : F("FAILED"));
      Serial.print(F(" (Failures: "));
      Serial.print(gpsStatus.failureCount);
      Serial.print(F(", Consecutive: "));
      Serial.print(gpsStatus.consecutiveFailures);
      Serial.println(F(")"));
      
      Serial.print(F("Sensor Suite Health: "));
      Serial.println(isSensorSuiteHealthy(flightState) ? F("HEALTHY") : F("UNHEALTHY"));
      
      Serial.print(F("Current State: "));
      Serial.println(getStateName(flightState));
    }
    else {
      // Unknown command
      Serial.print(F("Unknown command: "));
      Serial.println(command);
      Serial.println(F("Type 'help' for a list of available commands."));
    }
    Serial.println(F("[DEBUG] Exiting processCommand")); // <<< ADDED DEBUG
}

void setup() {
  // Initialize global flight state for clean start
  flightState = STARTUP;
  
  // Initialize Serial
  Serial.begin(115200);
  // Wait a bit for serial, but don't hang indefinitely
  unsigned long serialWaitStart = millis();
  while (!Serial && (millis() - serialWaitStart < 2000)) { delay(100); }
  
  // Initialize random seed from analog noise
  randomSeed(analogRead(0) + micros());
  
  Serial.println(F("\n--- TripleT Flight Firmware Initializing --- "));
  Serial.print(F("Version: ")); Serial.println(TRIPLET_FLIGHT_VERSION);
  Serial.print(F("Board: ")); Serial.println(BOARD_NAME);

  // --- STARTUP Phase --- 
  Serial.println(F("[SETUP] STARTUP Phase"));

  // Initialize NeoPixel for visual feedback
  pixels.begin();
  pixels.clear();
  setLEDColor(0, 20, 0, 0); // Red during startup
  setLEDColor(1, 20, 0, 0); // Red during startup
  
  // Initialize Watchdog
#if ENABLE_WATCHDOG
  WDT_timings_t config;
  config.trigger = 5; // seconds - Reset timeout
  config.timeout = 10; // seconds - Actual timeout before firing callback
  watchdog.begin(config);
  watchdog.callback(watchdogHandler);
  watchdog.feed(); // Initial feed
  Serial.println(F("[SETUP] Watchdog initialized."));
#else
  Serial.println(F("[SETUP] Watchdog DISABLED by config."));
#endif // ENABLE_WATCHDOG

  // Startup Tone
  tone(BUZZER_PIN, 2000); delay(50); noTone(BUZZER_PIN);

  // Check for EEPROM recovery *before* major initializations
  Serial.println(F("[SETUP] Checking for EEPROM state recovery..."));
  recoverFromPowerLoss(); // This might change flightState from STARTUP

  // If recovery changed state, log it and potentially skip some setup if resuming
  if (flightState != STARTUP) {
      Serial.print(F("[SETUP] Resuming from recovered state: ")); 
      Serial.println(getStateName(flightState));
      // Add any specific resume logic here if needed, otherwise proceed
  }

  // watchdog.feed(); // <<< REMOVED redundant feed call - Added #if block below

#if ENABLE_WATCHDOG
  watchdog.feed(); // Feed after potential EEPROM read
#endif // ENABLE_WATCHDOG

  // Initialize I2C and SPI
  Serial.println(F("[SETUP] Initializing I2C & SPI..."));
  #if defined(BOARD_TEENSY41)
    SPI.begin();
    Wire.begin();
    Wire.setClock(200000); // <<< CHANGED from 400000 to 200000
    Serial.println(F("[SETUP] Teensy 4.1 SPI/I2C (SDIO) Initialized at 200kHz.")); // <<< UPDATED LOG
  #else
    // SPI.setCS(SD_CS_PIN); // Pins may vary, ensure defined elsewhere
    // SPI.setMOSI(SD_MOSI_PIN);
    // SPI.setMISO(SD_MISO_PIN);
    // SPI.setSCK(SD_SCK_PIN);
    SPI.begin();
    Wire.begin();
    Wire.setClock(100000); // <<< CHANGED from 400000 to 100000
    Serial.println(F("[SETUP] Teensy 4.0 SPI/I2C Initialized at 100kHz.")); // <<< UPDATED LOG
  #endif
  scan_i2c(); // Scan bus after initialization

#if ENABLE_WATCHDOG
  watchdog.feed();
#endif // ENABLE_WATCHDOG
  
  // Initialize Sensors (check return values)
  Serial.println(F("[SETUP] Initializing Sensors..."));
  setLEDColor(0, 25, 0, 25); // Purple during sensor init
  
  bool kx134_ok = kx134_init();
  if (!kx134_ok) { Serial.println(F("[SETUP] WARNING: KX134 init failed")); }
  else { 
    Serial.println(F("[SETUP] KX134 Initialized.")); 
    // <<< START ADDED DEBUG >>>
    Serial.println(F("[SETUP] Adding 100ms delay after kx134_init..."));
    delay(100); // Add a small delay for sensor stabilization
    
    // NEW APPROACH: Set a timeout for the dataReady check
    Serial.println(F("[SETUP] Attempting kx134Accel.dataReady() check with timeout..."));
    
    // Add global variable for timeout tracking
    unsigned long startTime = millis();
    const unsigned long TIMEOUT = 500; // 500ms timeout
    bool ready = false;
    bool timeoutOccurred = false;

    // Try to poll until timeout
    while (millis() - startTime < TIMEOUT) {
      // Only spend a brief moment trying
      Wire.beginTransmission(0x1F); // KX134 address
      if (Wire.endTransmission() == 0) { // If we can communicate
        ready = false; // Default to false, don't even try dataReady()
        break; // Just exit the loop
      }
      delay(5); // Small delay between attempts
    }
    
    if (millis() - startTime >= TIMEOUT) {
      timeoutOccurred = true;
    }
    
    Serial.print(F("[SETUP] kx134Accel I2C check "));
    Serial.println(timeoutOccurred ? "timed out" : "completed");
    
    // Don't even try to call dataReady() as it appears to be hanging
    ready = false;
    Serial.println(F("[SETUP] Skipping dataReady() check for now"));
    // <<< END ADDED DEBUG >>>
    
    // Set global flag for proper accelerometer selection
    kx134_initialized_ok = true; // Mark as initialized but we'll check dataReady in loop
  }
  accelerometerStatus.isWorking = kx134_ok; // Update status based on init
  
  // Call void init, then check status separately
  ICM_20948_init();
  bool icm_ok = ICM_20948_isReady(); // Check status after init call
  if (!icm_ok) { Serial.println(F("[SETUP] WARNING: ICM-20948 check failed after init")); }
  else { Serial.println(F("[SETUP] ICM-20948 Initialized.")); }
  // Update status based on check
  if (!kx134_ok) { accelerometerStatus.isWorking = icm_ok; } // Use ICM accel if KX134 failed
  gyroscopeStatus.isWorking = icm_ok;
  magnetometerStatus.isWorking = icm_ok;

  // Call void init, then check status separately
  ms5611_init(); 
  bool ms5611_ok = ms5611Sensor.isConnected(); // Check status after init call
  if (!ms5611_ok) { Serial.println(F("[SETUP] WARNING: MS5611 check failed after init")); }
  else { Serial.println(F("[SETUP] MS5611 Initialized.")); }
  barometerStatus.isWorking = ms5611_ok;

  // Call void init, then check status separately
  gps_init(); 
  bool gps_ok = myGNSS.getPVT(); // Check status after init call
  if (!gps_ok) { Serial.println(F("[SETUP] WARNING: GPS check failed after init")); }
  else { Serial.println(F("[SETUP] GPS Initialized.")); }
  gpsStatus.isWorking = gps_ok;

#if ENABLE_WATCHDOG
  watchdog.feed();
#endif // ENABLE_WATCHDOG
  
  // --- CALIBRATION Phase --- 
  // Only perform calibration if starting fresh (not recovering into a flight state)
  if (flightState == STARTUP) {
      Serial.println(F("[SETUP] Entering CALIBRATION Phase"));
      setLEDColor(0, 50, 50, 0); // Yellow during calibration
      
      // Attempt GPS Time Sync and Barometer Calibration
      if (gpsStatus.isWorking && barometerStatus.isWorking) {
          Serial.println(F("[SETUP] Waiting for GPS time sync & 3D Fix (max 60s)..."));
          unsigned long calStart = millis();
          bool gpsCalibrated = false;
          while (millis() - calStart < 60000) {
#if ENABLE_WATCHDOG
              watchdog.feed();
#endif // ENABLE_WATCHDOG
              gps_read(); // Continuously read GPS
              if (myGNSS.getYear() >= 2025 && myGNSS.getFixType() >= 3) { // Check for valid time and fix
                  Serial.println(F("[SETUP] GPS time/fix OK. Attempting baro calibration (max 30s)..."));
                  setLEDColor(0, 0, 50, 50); // Cyan for GPS ok/calibrating
                  if (ms5611_calibrate_with_gps(30000)) { // Try calibrating
                      baroCalibrated = true;
                      Serial.println(F("[SETUP] Barometer calibration SUCCESSFUL."));
                      setLEDColor(0, 0, 50, 0); // Green - calibration done
                  } else {
                      Serial.println(F("[SETUP] WARNING: Barometer calibration failed/timed out."));
                      setLEDColor(0, 50, 0, 0); // Red - calibration failed
                      delay(1000); // Show red briefly
                      setLEDColor(0, 50, 50, 0); // Back to yellow
                  }
                  gpsCalibrated = true;
                  break; // Exit calibration loop
              }
              delay(200); // Short delay
          }
          if (!gpsCalibrated) {
              Serial.println(F("[SETUP] WARNING: GPS time sync/fix timeout during calibration phase."));
              // Baro remains uncalibrated
          }
      } else {
          Serial.println(F("[SETUP] Skipping calibration phase (GPS or Baro init failed)."));
      }
      // Set flightState based on calibration outcomes and overall health
      // If critical sensors failed init OR calibration is required but failed,
      // consider setting state to ERROR. For now, we allow loop() to handle later calibration.
  } 
  // If we recovered into a state like DROGUE_DESCENT, skip calibration phase
  else {
      Serial.println(F("[SETUP] Skipping CALIBRATION Phase due to recovered state."));
  }
  
#if ENABLE_WATCHDOG
  watchdog.feed();
#endif // ENABLE_WATCHDOG
  
  // Initialize Storage
  Serial.println(F("[SETUP] Initializing Storage..."));
  setLEDColor(0, 25, 25, 25); // White during storage init
  delay(500); // Allow time for card stabilization
  
  // Make multiple attempts to initialize SD card
  const int MAX_SD_INIT_ATTEMPTS = 3;
  bool sd_init_success = false;
  
  for (int attempt = 1; attempt <= MAX_SD_INIT_ATTEMPTS; attempt++) {
    Serial.print(F("[SETUP] SD card init attempt "));
    Serial.print(attempt);
    Serial.print(F(" of "));
    Serial.println(MAX_SD_INIT_ATTEMPTS);
    
    if (initSDCard()) {
      sd_init_success = true;
      break;
    }
    
    if (attempt < MAX_SD_INIT_ATTEMPTS) {
      Serial.println(F("[SETUP] Retrying SD card initialization..."));
      delay(1000); // Wait between attempts
    }
  }
  
  sdCardAvailable = sd_init_success;
  Serial.print(F("[SETUP] SD Card Available: ")); Serial.println(sdCardAvailable ? "Yes" : "No");
  
  if (sdCardAvailable) {
      // Try to create log file, with multiple attempts
      bool logFileCreated = false;
      
      // First, try with GPS time if available
      if (createNewLogFile()) {
          Serial.println(F("[SETUP] Log file created successfully."));
          logFileCreated = true;
          loggingEnabled = true;
      } else {
          // If first attempt failed, force creation with fallback naming
          Serial.println(F("[SETUP] First log file creation attempt failed. Trying with forced fallback..."));
          
          // Close any open file
          if (LogDataFile) {
              LogDataFile.flush();
              LogDataFile.close();
          }
          
          // Create a fallback filename
          char fallbackName[32];
          sprintf(fallbackName, "BOOT_%lu.csv", millis());
          
          Serial.print(F("[SETUP] Creating fallback log file: "));
          Serial.println(fallbackName);
          
          // Open the file directly with fallback name
          if (LogDataFile.open(fallbackName, O_RDWR | O_CREAT | O_EXCL)) {
              // Write CSV header
              LogDataFile.println(F("LogSeqNum,LoopCount,Timestamp,FixType,Sats,Lat,Long,Alt,AltMSL,Speed,Heading,pDOP,RTK,"
                                  "GPSDate,GPSTime(UTC),"
                                  "Pressure,Temperature,"
                                  "KX134_AccelX,KX134_AccelY,KX134_AccelZ,"
                                  "ICM_AccelX,ICM_AccelY,ICM_AccelZ,"
                                  "ICM_GyroX,ICM_GyroY,ICM_GyroZ,"
                                  "ICM_MagX,ICM_MagY,ICM_MagZ,"
                                  "ICM_Temp,"
                                  "FlightStateCode,FlightStateName,StateEntryTime,StateElapsedTime,"
                                  "AltitudeAGL,MaxAltitude,AccelMagnitude,VerticalVelocity,"
                                  "FlightDuration,TimeSinceApogee,"
                                  "SensorStatusFlags,BatteryVoltage,"
                                  "SDCardStatus,FreeSpaceMB,"
                                  "hAcc(m),vAcc(m)"));
              
              // Flush to ensure header is written
              LogDataFile.flush();
              
              // Copy filename to global variable
              strcpy(logFileName, fallbackName);
              
              logFileCreated = true;
              loggingEnabled = true;
              
              Serial.println(F("[SETUP] Fallback log file created successfully."));
          } else {
              Serial.println(F("[SETUP] WARNING: Even fallback log file creation failed!"));
              loggingEnabled = false;
          }
      }
      
      // Log a setup message to the file
      if (logFileCreated) {
          // Log setup complete message
          String setupMessage = String(millis()) + ",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
          LogDataFile.println(setupMessage);
          LogDataFile.flush();
      }
  } else {
      Serial.println(F("[SETUP] WARNING: SD Card not available, logging disabled."));
      loggingEnabled = false;
  }

#if ENABLE_WATCHDOG
  watchdog.feed();
#endif // ENABLE_WATCHDOG

  // --- Final Health Check & Set Initial State for Loop --- 
  Serial.println(F("[SETUP] Performing final health check..."));
  bool systemHealthy = isSensorSuiteHealthy(PAD_IDLE); // Check health required for PAD_IDLE
  
  // Set the state for loop() based on recovery and health checks
  // If we recovered into a valid flight/post-flight state, keep it.
  // If we started fresh (state is STARTUP), set based on health.
  if (flightState == STARTUP) { // Only overwrite if we weren't in a recovered state
      if (systemHealthy) {
          flightState = PAD_IDLE;
          Serial.println(F("[SETUP] System OK. Entering PAD_IDLE."));
          setLEDColor(0, 0, 50, 0); // Green - ready
      } else {
          flightState = ERROR;
          Serial.println(F("[SETUP] CRITICAL ERROR during setup. Entering ERROR state."));
          setLEDColor(0, 50, 0, 0); // Red - error
      }
  } else {
       Serial.print(F("[SETUP] Retaining recovered state: ")); Serial.println(getStateName(flightState));
       // Update LED based on recovered state if needed (e.g., green for RECOVERY, red for ERROR)
       if(flightState == ERROR) setLEDColor(0, 50, 0, 0); 
       else if (flightState == RECOVERY) setLEDColor(0, 0, 20, 0);
       // Add other recovered state LED colors if desired
  }
  pixels.show(); 

  // Save the determined initial state
  saveStateToEEPROM();

  Serial.println(F("--- TripleT Flight Firmware Setup Complete --- "));

  // <<< ADDED DEBUG: Print final setup status >>>
  Serial.println(F("[SETUP] Final Status Check:"));
  Serial.print(F("  Final Flight State: ")); Serial.println(getStateName(flightState));
  Serial.print(F("  Barometer Working: ")); Serial.println(barometerStatus.isWorking ? "Yes" : "No");
  Serial.print(F("  Accelerometer Working: ")); Serial.println(accelerometerStatus.isWorking ? "Yes" : "No");
  Serial.print(F("  Gyroscope Working: ")); Serial.println(gyroscopeStatus.isWorking ? "Yes" : "No");
  Serial.print(F("  Magnetometer Working: ")); Serial.println(magnetometerStatus.isWorking ? "Yes" : "No");
  Serial.print(F("  GPS Working: ")); Serial.println(gpsStatus.isWorking ? "Yes" : "No");
  // <<< END ADDED DEBUG >>>

#if ENABLE_WATCHDOG
  watchdog.feed(); // Final feed before loop
#endif // ENABLE_WATCHDOG
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
  static unsigned long lastSensorStatusCheckTime = 0; // Track when we last checked sensor health
  static bool sensorsUpdated = false; // Flag to trigger logging after reads
  
  // Feed the watchdog at the start of every loop
#if ENABLE_WATCHDOG
  watchdog.feed();
#endif // ENABLE_WATCHDOG
  
  // Check for serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // Remove leading/trailing whitespace
    if (command.length() > 0) {
        Serial.print(F("Received command: "));
        Serial.println(command);
        processCommand(command);
    }
  }
  
  // --- Sensor Reading --- 
  // Read GPS data periodically
  if (millis() - lastGPSReadTime >= GPS_POLL_INTERVAL) {
    lastGPSReadTime = millis();
    if (gpsStatus.isWorking) { // Only read if initialized okay
      bool gpsSucess = gps_read();
      if (gpsSucess && GPS_fixType >= 2) {  // Valid 2D or 3D fix
        gpsStatus.isWorking = true;
        gpsStatus.lastValidReading = millis();
        if (gpsStatus.consecutiveFailures > 0) {
          gpsStatus.consecutiveFailures--;
        }
      }
      sensorsUpdated = true;
    }
  }
  
  // Read barometer data periodically
  if (millis() - lastBaroReadTime >= BARO_POLL_INTERVAL) {
    lastBaroReadTime = millis();
    if (barometerStatus.isWorking) {
        int result = ms5611_read();
        if (result == MS5611_READ_OK) {
            sensorsUpdated = true;
        } else {
            // Consider incrementing failure count here or rely on checkSensorStatus
        }
    }
  }
  
  // Read IMU data periodically (Reads ICM Accel/Gyro/Mag)
  if (millis() - lastIMUReadTime >= IMU_POLL_INTERVAL) {
    lastIMUReadTime = millis();
    // Assume if any ICM part is working, we should read it.
    // ICM_20948_read() should update icm20948_ready internally.
    if (gyroscopeStatus.isWorking || magnetometerStatus.isWorking || accelerometerStatus.isWorking || kx134_initialized_ok ) {
        ICM_20948_read();
        sensorsUpdated = true;
    }
  }
  
  // Read High-G Accelerometer data periodically (if used)
  if (USE_KX134 && millis() - lastAccelReadTime >= ACCEL_POLL_INTERVAL) { // <<< FIXED: useKX134 -> USE_KX134
    lastAccelReadTime = millis();
    // We need the KX134 data even if only ICM is checked for UKF readiness
    if (kx134_initialized_ok) { // Check if KX134 init was okay
        kx134_read();
        sensorsUpdated = true;
        // REMOVED UKF processing from here
    }
  }
  
  // --- UKF Processing ---
  ProcessUKF(); // Call the dedicated UKF processing function
  
  // --- Data Logging --- 
  // Log data if any sensor was updated OR if UKF ran
  if (sensorsUpdated || ukfInitialized) { // Log if sensors updated OR UKF processed
    WriteLogData(false); // Let interval timer handle logging
    sensorsUpdated = false;
  }
  
  // --- State Machine Processing ---
  ProcessFlightState();
  
  // --- Safety Check: Ensure logging is active if we're in or past PAD_IDLE --- 
  if (flightState >= PAD_IDLE && !loggingEnabled && sdCardAvailable) {
    // This is a fallback to ensure logging is always enabled when we're in flight states
    Serial.println(F("[LOOP] WARNING: Logging not enabled in flight state! Forcing log file creation..."));
    
    // Force creation of a log file
    if (createNewLogFile()) {
      Serial.println(F("[LOOP] Safety log file created successfully."));
      loggingEnabled = true;
      
      // Log a safety message to mark this event using our new approach
      String setupMessage = String(millis()) + ",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
      // Open, write, and close the file
      FsFile tempFile;
      if (tempFile.open(logFileName, O_RDWR | O_AT_END)) {
        tempFile.println("SAFETY_LOGGING_TRIGGERED");
        tempFile.println(setupMessage);
        tempFile.close();
      }
    }
  }
  
  // Feed watchdog right after state processing (can take time)
#if ENABLE_WATCHDOG
  watchdog.feed(); 
#endif // ENABLE_WATCHDOG
  
  // --- Periodic Checks & Maintenance ---
  // Check sensor health status periodically
  if (millis() - lastSensorStatusCheckTime >= 1000) {
    lastSensorStatusCheckTime = millis();
    checkSensorStatus();
  }
  
  // Periodically check for SD card if not available
  static unsigned long lastSDCardRetryTime = 0;
  if (!sdCardAvailable && (millis() - lastSDCardRetryTime >= 30000)) { // Check every 30 seconds
    lastSDCardRetryTime = millis();
    Serial.println(F("[LOOP] Attempting to recover SD card..."));
    
    if (initSDCard()) {
      sdCardAvailable = true;
      Serial.println(F("[LOOP] SD Card recovered successfully!"));
      
      // Try to create a log file immediately
      if (createNewLogFile()) {
        Serial.println(F("[LOOP] Log file created successfully after SD recovery"));
        loggingEnabled = true;
      } else {
        Serial.println(F("[LOOP] Failed to create log file after SD recovery"));
        loggingEnabled = false;
      }
    } else {
      Serial.println(F("[LOOP] SD Card recovery failed"));
    }
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
  
  // Print status summary once per second, but only if enabled
  if (enableStatusSummary && millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
    lastDisplayTime = millis();
    printStatusSummary();
  }
  
  // Log data to serial at the specified display interval
  static unsigned long lastSerialLogTime = 0;
  if (enableSerialCSV && millis() - lastSerialLogTime >= DISPLAY_INTERVAL) {
    lastSerialLogTime = millis();
    // Let interval timer handle logging, just output to serial here potentially
    WriteLogData(false); 
  }
  
  // Print detailed data less frequently
  if (displayMode && millis() - lastDetailedTime >= 5000) {
    lastDetailedTime = millis();
    ICM_20948_print();
  }
  
  // Reset the watchdog timer at the end of each loop as a safety measure
  // watchdog.feed(); // <<< REMOVED - Feed is at the start of the loop
}

// Check if rocket is stable (no significant motion)
bool IsStable() {
  float accel_magnitude = GetAccelMagnitude();
  // Consider stable if acceleration is close to 1g (gravity only)
  return (accel_magnitude > 0.95 && accel_magnitude < 1.05);
}

// Get current barometric altitude in meters
float getBaroAltitude() {
  if (!ms5611_ready) {
    return 0.0; // Barometer not ready
  }
  
  // The pressure value is in hPa but the altitude formula expects Pa
  // So we need to multiply by 100 to convert hPa to Pa
  float pressurePa = pressure * 100.0;
  
  // Calculate altitude using the barometric formula
  float raw_altitude = 44330.0 * (1.0 - pow(pressurePa / 101325.0, 0.1903));
  
  // Apply calibration offset if calibrated
  if (baroCalibrated) {
    return raw_altitude + baro_altitude_offset;
  } else {
    return raw_altitude;
  }
}

// Helper function to get acceleration magnitude regardless of which accelerometer is available
float GetAccelMagnitude() {
  
  // Check KX134 *only if* it initialized successfully
  if (kx134_initialized_ok) {
      
      // MODIFIED: Safer check with timeout
      unsigned long startTime = millis();
      const unsigned long ACCEL_TIMEOUT = 200; // 200ms timeout
      bool ready = false;
      bool timeoutOccurred = false;
      
      // Try to poll until timeout with safer approach
      while (millis() - startTime < ACCEL_TIMEOUT) {
        // Just check if we can communicate with I2C device
        Wire.beginTransmission(0x1F); // KX134 address
        if (Wire.endTransmission() != 0) { // If we can't communicate
          ready = false;
          break;
        }
        
        // We can communicate, but don't check dataReady() yet
        ready = true;
        break;
      }
      
      if (millis() - startTime >= ACCEL_TIMEOUT) {
        timeoutOccurred = true;
        ready = false;
      }
      
      // Skip the dataReady() call that hangs and assume data is available
      // if communication with the device is successful
      if (ready) {
        return sqrt(kx134_accel[0] * kx134_accel[0] + 
                    kx134_accel[1] * kx134_accel[1] + 
                    kx134_accel[2] * kx134_accel[2]);
      }
  }

  // Fallback to ICM20948 if KX134 didn't init or wasn't ready
  if (icm20948_ready) { // This is an extern bool
    Serial.println(F("[GetAccelMagnitude] Using ICM20948 data")); // <<< ADDED DEBUG
    return sqrt(icm_accel[0] * icm_accel[0] + 
                icm_accel[1] * icm_accel[1] + 
                icm_accel[2] * icm_accel[2]);
  }

  Serial.println(F("[GetAccelMagnitude] No accel available/ready, returning 0.0")); // <<< UPDATED DEBUG
  return 0.0; // No accelerometer available or ready
}

// After the helper functions but before ProcessFlightState
void handleSensorErrors() {
  // Check barometer status
  if (ms5611_ready) {
    if (barometerStatus.consecutiveFailures > 0) barometerStatus.consecutiveFailures--;
  } else {
    barometerStatus.consecutiveFailures++;
    if (barometerStatus.consecutiveFailures >= BAROMETER_ERROR_THRESHOLD) {
      barometerStatus.isWorking = false;
    }
  }
  
  // Check accelerometer status (KX134 if available, otherwise ICM20948)
  if (USE_KX134) { // <<< FIXED: useKX134 -> USE_KX134
    if (kx134_accel_ready) {
      if (accelerometerStatus.consecutiveFailures > 0) accelerometerStatus.consecutiveFailures--;
    } else {
      accelerometerStatus.consecutiveFailures++;
      if (accelerometerStatus.consecutiveFailures >= ACCEL_ERROR_THRESHOLD) {
        accelerometerStatus.isWorking = false;
      }
    }
  } else {
    if (icm20948_ready) {
      if (accelerometerStatus.consecutiveFailures > 0) accelerometerStatus.consecutiveFailures--;
    } else {
      accelerometerStatus.consecutiveFailures++;
      if (accelerometerStatus.consecutiveFailures >= ACCEL_ERROR_THRESHOLD) {
        accelerometerStatus.isWorking = false;
      }
    }
  }
  
  // Check GPS status
  if (millis() - gpsStatus.lastValidReading < GPS_TIMEOUT_MS) {
    if (gpsStatus.consecutiveFailures > 0) gpsStatus.consecutiveFailures--;
  } else {
    // Before marking GPS as not working, check if we have valid fix and satellites data
    // This prevents the system from entering ERROR state when GPS has valid data
    if (GPS_fixType >= 3 && SIV >= 4) {
      // We have a good 3D fix with at least 4 satellites, update lastValidReading
      gpsStatus.isWorking = true;
      gpsStatus.lastValidReading = millis();
      if (gpsStatus.consecutiveFailures > 0) gpsStatus.consecutiveFailures--;
    } else {
      gpsStatus.consecutiveFailures++;
      if (gpsStatus.consecutiveFailures >= GPS_TIMEOUT_MS / 1000) {
        gpsStatus.isWorking = false;
      }
    }
  }
  
  // Handle error state transition if critical sensors fail
  if (flightState != ERROR && 
      ((flightState < BOOST && !barometerStatus.isWorking) || 
       !accelerometerStatus.isWorking)) {
    // Critical sensor failure - transition to ERROR state
    flightState = ERROR;
    setLEDColor(0, 255, 0, 0); // Red for ERROR
  }
}

void watchdogCallback() {
  // This function is called when the watchdog times out
  // In a real implementation, we might try to recover or set error flags
  // For now, we'll just set the flight state to ERROR
  flightState = ERROR;
  setLEDColor(0, 255, 0, 0); // Red for ERROR
}

// Function to set LED color based on state
void setLEDColor(uint8_t p, uint8_t r, uint8_t g, uint8_t b) {
  // Set the color of the LED at pixel index p
  // r, g, b are the red, green, and blue color values (0-255)
  pixels.setPixelColor(p, pixels.Color(r, g, b));
  pixels.show();
}

void checkWatchdog() {
  // Reset the watchdog timer
#if ENABLE_WATCHDOG
  watchdog.feed();
#endif // ENABLE_WATCHDOG
  
  // Check for state timeouts
  unsigned long currentTime = millis();
  
  // For states that shouldn't last too long
  if (flightState == BOOST && lastStateChangeTime > 0) {
    if (currentTime - lastStateChangeTime > 10000) { // 10 seconds max in BOOST
      // Backup transition if boost detection fails
      flightState = COAST;
      lastStateChangeTime = currentTime;
    }
  }
  else if (flightState == COAST && lastStateChangeTime > 0) {
    if (currentTime - lastStateChangeTime > 30000) { // 30 seconds max in COAST
      // Backup transition if apogee detection fails
      flightState = APOGEE;
      lastStateChangeTime = currentTime;
    }
  }
}

// Add these variables near the other global variables
unsigned long armingToneStartTime = 0;
byte armingTonePhase = 0;
bool playingArmingTone = false;

// --- EEPROM Functions Implementation ---

// Save current state to EEPROM
void saveStateToEEPROM() {
  unsigned long currentTime = millis();
  
  // Only save periodically or on critical state changes to reduce wear
  // Critical states for immediate save: APOGEE, DROGUE_DEPLOY, MAIN_DEPLOY, LANDED, ERROR
  bool forceSave = (flightState == APOGEE || flightState == DROGUE_DEPLOY ||
                    flightState == MAIN_DEPLOY || flightState == LANDED || flightState == ERROR);
  
  if (!forceSave && (currentTime - lastStateSave < EEPROM_UPDATE_INTERVAL)) {
    return; // Not time to save yet and not a critical state
  }

  // Update state data structure before saving
  stateData.state = flightState;
  stateData.launchAltitude = launchAltitude; // Assuming launchAltitude is global
  stateData.maxAltitude = maxAltitudeReached; // Assuming maxAltitudeReached is global
  stateData.flightStartTime = flightStartTime; // Assuming flightStartTime is global
  stateData.boostEndTime = boostEndTime; // Assuming boostEndTime is global
  stateData.timestamp = currentTime;
  stateData.signature = EEPROM_SIGNATURE_VALUE;

  // Write the whole structure to EEPROM
  EEPROM.put(EEPROM_STATE_ADDR, stateData);

  lastStateSave = currentTime; // Update the last save time

  if (enableSystemDebug && (forceSave || (currentTime - lastStateSave) < 100)) { // Log forced saves or recent periodic saves
      Serial.print(F("Flight state saved to EEPROM: "));
      Serial.println(getStateName(flightState));
  }
}


// Load state from EEPROM
bool loadStateFromEEPROM() {
  // Read the structure from EEPROM
  EEPROM.get(EEPROM_STATE_ADDR, stateData);

  // Validate data using the signature
  if (stateData.signature != EEPROM_SIGNATURE_VALUE) {
    Serial.println(F("No valid flight state found in EEPROM (invalid signature)"));
    // Optional: Clear invalid data?
    // stateData = {}; // Reset struct
    // stateData.signature = 0;
    // EEPROM.put(EEPROM_STATE_ADDR, stateData);
    return false;
  }

  // Data is valid, print the recovered state
  Serial.println(F("Found valid flight state in EEPROM:"));
  Serial.print(F("  State: ")); Serial.println(getStateName(stateData.state));
  Serial.print(F("  Launch Altitude: ")); Serial.println(stateData.launchAltitude);
  Serial.print(F("  Max Altitude: ")); Serial.println(stateData.maxAltitude);
  Serial.print(F("  Flight Start Time: ")); Serial.println(stateData.flightStartTime);
  Serial.print(F("  Boost End Time: ")); Serial.println(stateData.boostEndTime);
  Serial.print(F("  Timestamp: ")); Serial.println(stateData.timestamp);

  return true;
}

// Recover state after a power loss/reset
void recoverFromPowerLoss() {
  if (!loadStateFromEEPROM()) {
    // No valid data found, start fresh
    flightState = STARTUP; // Ensure we start cleanly
    launchAltitude = 0.0;
    maxAltitudeReached = 0.0;
    flightStartTime = 0;
    boostEndTime = 0;
    Serial.println(F("Starting fresh flight sequence."));
    return;
  }

  // Valid data found, attempt recovery
  Serial.print(F("Attempting recovery from state: "));
  Serial.println(getStateName(stateData.state));

  // Restore global variables from loaded data
  launchAltitude = stateData.launchAltitude;
  maxAltitudeReached = stateData.maxAltitude;
  flightStartTime = stateData.flightStartTime;
  boostEndTime = stateData.boostEndTime;
  
  // Determine the state to resume in
  FlightState recoveredState = stateData.state;

  switch (recoveredState) {
    case STARTUP:
    case CALIBRATION:
    case PAD_IDLE:
      // If reset occurred in these early states, just restart normally
      flightState = STARTUP;
      Serial.println(F("-> Restarting from STARTUP (recovery from early state)"));
      break;

    case ARMED:
      // If reset while armed but not launched, return to PAD_IDLE for safety checks
      flightState = PAD_IDLE;
      Serial.println(F("-> Returning to PAD_IDLE (recovery from ARMED)"));
      break;

    case BOOST:
    case COAST:
      // If reset during ascent, this is dangerous.
      // Safest assumption: We are past apogee and descending.
      // Force DROGUE_DESCENT state. If drogue doesn't exist, logic might need refinement.
      // We *could* try checking current altitude vs maxAltitude, but sensor readings might be unreliable immediately after reset.
      flightState = DROGUE_DESCENT; // Assume descending under drogue
      Serial.println(F("-> Assuming DROGUE_DESCENT (recovery from BOOST/COAST)"));
      // Note: Pyros won't have fired. This state will proceed to check for MAIN deploy altitude.
      // Might need manual intervention or ground command if this happens.
      break;

    case APOGEE:
    case DROGUE_DEPLOY:
      // If reset during/just after apogee or drogue deployment, we should try to ensure drogue is deployed.
      // Go directly to DROGUE_DESCENT. The pyro won't fire again here, but it should have fired before the reset.
      // If hardware allows checking pyro status, that would be better.
      flightState = DROGUE_DESCENT;
      Serial.println(F("-> Ensuring DROGUE_DESCENT (recovery from APOGEE/DROGUE_DEPLOY)"));
      break;

    case DROGUE_DESCENT:
      // If reset during drogue descent, just resume in the same state.
      flightState = DROGUE_DESCENT;
      Serial.println(F("-> Resuming DROGUE_DESCENT"));
      break;

    case MAIN_DEPLOY:
      // If reset during main deployment, ensure we proceed to main descent.
      // Pyro should have fired before reset.
      flightState = MAIN_DESCENT;
      Serial.println(F("-> Ensuring MAIN_DESCENT (recovery from MAIN_DEPLOY)"));
      break;

    case MAIN_DESCENT:
      // If reset during main descent, resume in the same state.
      flightState = MAIN_DESCENT;
      Serial.println(F("-> Resuming MAIN_DESCENT"));
      break;

    case LANDED:
    case RECOVERY:
      // If reset after landing or during recovery, go to RECOVERY state.
      flightState = RECOVERY;
      Serial.println(F("-> Entering RECOVERY (recovery from LANDED/RECOVERY)"));
      break;

    case ERROR:
      // If reset while in ERROR state, remain in ERROR state.
      flightState = ERROR;
      Serial.println(F("-> Remaining in ERROR state (recovery from ERROR)"));
      break;

    default:
      // Unknown state in EEPROM? Treat as error or restart.
      Serial.println(F("-> Unknown state recovered! Returning to STARTUP."));
      flightState = STARTUP;
      break;
  }
  // Note: stateData struct is now loaded, but flightState global variable dictates the actual running state.
  // Global variables like launchAltitude, maxAltitudeReached have been restored.
}

// Global variables for UKF
// bool ukfInitialized = false; // Removed - already declared globally
unsigned long lastUkfProcessTime = 0;
#define UKF_PROCESS_INTERVAL 50  // 20Hz processing rate for UKF

// Global variables for altitude calculation and vertical velocity are already declared at the top
// float lastAltitude = 0.0; // Removed - already declared globally
// unsigned long lastAltitudeTime = 0; // Removed - already declared globally
// float verticalVelocity = 0.0; // Removed - already declared globally

// Function to get the fused vertical velocity from UKF
float getUkfVelocity() {
  // Now reads the global variable updated in the loop
  if (!ukfInitialized) return 0.0;
  return ukf_vel;
}

// Function to get the fused vertical acceleration from UKF
float getUkfAcceleration() {
  // Now reads the global variable updated in the loop
  if (!ukfInitialized) return GetAccelMagnitude() - 1.0; // Fallback remains
  return ukf_accel;
}

// Add this wrapper for position
float getUkfPosition() {
  if (!ukfInitialized) return 0.0;
  return ukf_pos;
}

// Read the KX134 Accelerometer with UKF processing
void ReadKX134Accelerometer() {
  if (!kx134_initialized_ok) return;
  
  // Try to poll KX134 accelerometer
  if (kx134_accel_ready) {
    // Extract vertical axis data (assuming Z is vertical)
    float kx134_accel_z = kx134_accel[2];
    
    // Only process UKF if ICM is also available
    if (icm20948_ready) {
      float icm_accel_z = icm_accel[2];
      
      // Process through UKF at specified intervals
      unsigned long currentMillis = millis();
      if (currentMillis - lastUkfProcessTime >= UKF_PROCESS_INTERVAL) {
        lastUkfProcessTime = currentMillis;
        
        // Process through UKF (time in seconds)
        float dt = UKF_PROCESS_INTERVAL / 1000.0;
        ukf.processAccel(kx134_accel_z, icm_accel_z, dt);
        
        // UKF is now initialized
        if (!ukfInitialized) {
          ukfInitialized = true;
        }
      }
    }
  }
}

