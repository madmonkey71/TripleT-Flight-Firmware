// TripleT Flight Firmware
// Current Version: v0.10
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
#include <SD.h>
#include <SerialFlash.h>
// Library for controlling PWM Servo's
#include <PWMServo.h>
// Add this at the top with the other includes
#include <LittleFS.h>
// Add this near the top where other includes are
#include "gps_config.h"
#include "gps_functions.h"  // Include GPS functions header

// Board Configuration
// Uncomment the board you're using and comment out others
#define BOARD_TEENSY40  // Teensy 4.0
// #define BOARD_STM32_THING_PLUS  // SparkFun STM32 Thing Plus
// Board define is set via build flags in platformio.ini

// Board-specific pin definitions
#ifdef BOARD_STM32_THING_PLUS
  // SparkFun STM32 Thing Plus pin assignments
  #define SD_CS_PIN      10    // SD Card CS pin
  #define SD_MOSI_PIN    11    // SD Card MOSI pin
  #define SD_MISO_PIN    12    // SD Card MISO pin
  #define SD_SCK_PIN     13    // SD Card SCK pin
  #define BUZZER_PIN     9     // Buzzer pin
  #define WLED_PIN       8     // WS2812 LED pin
  #define FLASH_CS_PIN   7     // External flash CS pin
  #define INTERNAL_FLASH_SIZE (1024 * 1024)  // 1MB (STM32F405 has 1MB flash)
  #define I2C_SDA_PIN    20    // I2C SDA pin
  #define I2C_SCL_PIN    21    // I2C SCL pin
#elif defined(BOARD_TEENSY40)
  // Teensy 4.0 pin assignments
  #define SD_CS_PIN 10
  #define SD_MOSI_PIN 11
  #define SD_MISO_PIN 12
  #define SD_SCK_PIN 13
  #define BUZZER_PIN 23
  #define WLED_PIN 7
  #define FLASH_CS_PIN 5
  #define INTERNAL_FLASH_SIZE (512 * 1024)  // 512KB
#endif

// For STM32 boards, include Arduino core headers
#if defined(BOARD_STM32_THING_PLUS) || defined(STM32_THING_PLUS)
  // Include STM32 specific headers
  #include <STM32_GPIO.h>
  // Define Arduino pin numbers for STM32 ports
  #define PIN_PA4 A4
  #define PIN_PA5 A5
  #define PIN_PA6 A6
  #define PIN_PA7 A7
  #define PIN_PB0 2
  #define PIN_PB1 3
  #define PIN_PB3 4
  #define PIN_PB4 5
  #define PIN_PB5 8
  #define PIN_PB6 21
  #define PIN_PB7 20
  #define PIN_PB8 9
  #define PIN_PB12 7
  #define PIN_PA0 A0
  #define PIN_PA1 A1
#endif

// Libraries for all of the sensor hardware
// KX134 Accelerometer
#include <SparkFun_KX13X.h> // Click here to get the library: http://librarymanager/All#SparkFun_KX13X
// Ublox M8Q GPS Module
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
// MS5611 Barometric Pressure Sensor
#include "MS5611.h"
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

// Include the appropriate servo library based on the board
#if defined(USE_STD_SERVO_LIB)
  #include <Servo.h>
#elif defined(USE_TEENSY_SERVO_LIB)
  #include <PWMServo.h>
#else
  #include <PWMServo.h> // Default to PWMServo
#endif

//=============================================================================
// BOARD CONFIGURATION
//=============================================================================
// Board-specific pin definitions and settings
// These are controlled by the build flags in platformio.ini

#if defined(BOARD_STM32_THING_PLUS)
  // SparkFun STM32 Thing Plus pin assignments
  #define BOARD_NAME "SparkFun STM32 Thing Plus"
  // SPI pins - use the standard Arduino constants format for STM32
  #define SD_CS_PIN      10    // SD Card CS pin
  #define SD_MOSI_PIN    11    // SD Card MOSI pin
  #define SD_MISO_PIN    12    // SD Card MISO pin
  #define SD_SCK_PIN     13    // SD Card SCK pin
  // I2C pins
  #define I2C_SDA_PIN    20    // I2C SDA pin
  #define I2C_SCL_PIN    21    // I2C SCL pin
  // GPIO pins
  #define BUZZER_PIN     9     // Buzzer pin
  #define WLED_PIN       8     // WS2812 LED pin
  #define FLASH_CS_PIN   7     // External flash CS pin
  #define PYRO1_PIN      PIN_PB0
  #define PYRO2_PIN      PIN_PB1
  #define PYRO3_PIN      PIN_PB3
  #define PYRO4_PIN      PIN_PB4
  // Servo pins
  #define TVCXpin        PIN_PA0
  #define TVCYpin        PIN_PA1
  // Internal flash settings
  #define INTERNAL_FLASH_SIZE (1024 * 1024)
  // STM32 needs different servo library
  #define USE_STD_SERVO_LIB

#elif defined(BOARD_TEENSY40)
  // Teensy 4.0 pin assignments (original)
  #define BOARD_NAME "Teensy 4.0"
  // SPI pins (handled by Teensy libraries)
  #define SD_CS_PIN 10
  // Standard pins
  #define BUZZER_PIN 23
  #define WLED_PIN 7
  #define FLASH_CS_PIN 5
  #define PYRO1 5
  #define PYRO2 6
  #define PYRO3 7
  #define PYRO4 8
  // Servo pins remain the same
  #define TVCXpin 0
  #define TVCYpin 1
  // Internal flash settings
  #define INTERNAL_FLASH_SIZE (512 * 1024)
  // Keep using Teensy-specific PWMServo
  #define USE_TEENSY_SERVO_LIB

#else
  // Default configuration (Teensy 4.0 compatible)
  #warning "No board defined, using default Teensy 4.0 pin assignments"
  #define BOARD_NAME "Unknown Board (Teensy compatible)"
  #define SD_CS_PIN 10
  #define BUZZER_PIN 23
  #define WLED_PIN 7
  #define FLASH_CS_PIN 5
  #define INTERNAL_FLASH_SIZE (512 * 1024)
  #define USE_TEENSY_SERVO_LIB
#endif

// Set common definitions needed by the code
#define SDchipSelect SD_CS_PIN
//=============================================================================

// Start creating all of the variables needed to pull data or other info
// Separate it out into specific blocks for the various sensors

#define TRIPLET_FLIGHT_VERSION 0.10
// Comment out to restrict roll to ±90deg instead - please read: https://www.nxp.com/docs/en/application-note/AN3461.pdf
// #define RESTRICT_PITCH 

// MS5611 Sensor
// This correctly creates an instance of the MS5611 class with I2C address 0x77
MS5611 ms5611Sensor(0x77);
uint32_t start, stop;

// Sparkfun ZOE-M8Q
// Put all the GPS variables here
// GPS variables are now defined in gps_functions.cpp and declared as extern in gps_functions.h
// ... existing code ...

// SparkFun_KX134 kxAccel;
SparkFun_KX134 kxAccel; // For the KX134, uncomment this and comment line above
outputData kx134AccelData; // Struct for the accelerometer's data
float kx134_x = 0;
float kx134_y = 0;
float kx134_z = 0;

// SparkFun 9DOF
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

// Global variables to store ICM_20948 IMU data
bool icm_data_available = false;
double icm_q1 = 0, icm_q2 = 0, icm_q3 = 0; // Quaternion data
float icm_accel_x = 0, icm_accel_y = 0, icm_accel_z = 0; // Accelerometer data
float icm_gyro_x = 0, icm_gyro_y = 0, icm_gyro_z = 0; // Gyroscope data
float icm_mag_x = 0, icm_mag_y = 0, icm_mag_z = 0; // Compass data
uint16_t icm_data_header = 0; // To store what data types are available

/*If there is no need to calibrate altitude, comment this line*/
#define CALIBRATE_Altitude

// Define all the Teensy Pins are are using
// WLED parameters
#define WLED_PIN 7
#define NUM_LEDS 2

// Create the WLED instance
Adafruit_NeoPixel pixels(NUM_LEDS, WLED_PIN, NEO_GRB + NEO_KHZ800);

//These are the output pins for the pyro channels
#define PYRO1 5
#define PYRO2 6
#define PYRO3 7
#define PYRO4 8

// This is the pin for the BUZZER 
#define BUZZER 23

// Thrust Vector Control Servo Pins
#define TVCXpin 0
#define TVCYpin 1

// Define manual mode button
#define SDchipSelect SD_CS_PIN
#define RESET_BUTTON_PIN NRST
#define MODE_BUTTON_PIN PC15 // the number of the Mode Button Pin

// Initialise the barometric sensor data variables.
float Temperature = 0;
float Pressure = 0;
float baro_altitude = 0;
float seaLevel = 0;

// Setup parameters for the Servo Library
#if defined(USE_STD_SERVO_LIB)
  Servo TVCX;
  Servo TVCY;
#else
  PWMServo TVCX;
  PWMServo TVCY;
#endif

// Used to Test the Servos on startup
int TVCXpos = 105;    // variable to store the servo position
int TVCYpos = 105;    // variable to store the servo position

//Position of servos through the startup function
int TVCXstart = TVCYpos;
int TVCYstart = TVCXpos;

//The amount the servo moves by in the startup function
int Startup_Servo_Offset = 45;

//Ratio between servo gear and tvc mount
float TVCXGearRatio = 8;
float TVCYGearRatio = 8;

// Setup the SD card stuff
Sd2Card card;
SdVolume volume;
SdFile root;
//const int SDchipSelect = 10;

File IdleDataFile; // Data file for Idle data logging.
File LogDataFile; // Data file for Flight data logging.
String LogDataString; // holds the data to be written to the SD card
String FileDateString; // Used for creating unique filenames for every run.
String dataFileName; // Used for building the name of the data file to write.


// Setup some operating variables 
int buttonState = 0;
int MODE = 0;          // Use the flight mode to run different code depending on what mode we are in
int timestamp = 0;     // Create timestamp value
int sampleRate = 20;   // Set specified sampling rate (data points per second) (somewhere between 10-200 is ideal)
int counter = 0;       // Loop Counter for debugging

//"P" Constants
float pidX_p = 0;
float pidY_p = 0;

//"I" Constants
float pidY_i = 0;
float pidX_i = 0;

//"D" Constants
float pidX_d = 0;
float pidY_d = 0;

//PID Gains
double kp = 0.11;
double ki = 0.04;
double kd = 0.025;

//Offsets for tuning 
int servoY_offset = 105;
int servoX_offset = 20;

//Upright Angle of the Flight Computer
int desired_angleX = 0;//servoY
int desired_angleY = 0;//servoX

double dt, currentTime, previousTime;

//Ratio between servo gear and tvc mount
float servoX_gear_ratio = 6;
float servoY_gear_ratio = 6;

int mode0count = 0;
int mode1count = 0;
int mode2count = 0;

uint32_t timer = millis();
int button_pressed = 0;

// Additional global variables for data logging
bool sdCardAvailable = false;
bool flashAvailable = false;
bool internalFlashAvailable = false;
unsigned long lastLogTime = 0;
const int LOG_INTERVAL_MS = 100; // Log every 100ms (10Hz)
const int FLASH_CHIP_SELECT = 5; // Choose an appropriate pin for flash CS
char logFileName[32] = ""; // To store the current log file name

// Internal flash memory parameters for Teensy
#define INTERNAL_FLASH_SIZE (512 * 1024)  // 512KB of internal flash for logging
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
LittleFS_Program flashFS; // Uses program flash memory
File flashLogFile;
char flashLogPath[32] = "/log.bin";
unsigned long recordCount = 0;

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
  flashLogFile = flashFS.open(flashLogPath, FILE_WRITE);
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
void checkStorageSpace() {
  static unsigned long lastCheckTime = 0;
  
  // Only check every 30 seconds to reduce overhead
  if (millis() - lastCheckTime < 30000) {
    return;
  }
  lastCheckTime = millis();
  
  // Check SD card free space
  if (sdCardAvailable) {
    // Get free space in bytes
    uint32_t totalSpace = 0;
    uint32_t usedSpace = 0;
    
    // Get card info to determine total size
    Sd2Card tempCard;
    SdVolume tempVolume;
    
    if (tempCard.init(SPI_HALF_SPEED, SDchipSelect)) {
      if (tempVolume.init(tempCard)) {
        // Card size in blocks (512 bytes per block)
        uint32_t volumeSize = tempVolume.blocksPerCluster() * tempVolume.clusterCount();
        totalSpace = volumeSize * 512UL; // Convert blocks to bytes
      }
    }
    
    // This is a simplified check that assumes FAT32 filesystem
    // For more accurate calculations, additional code would be needed
    File root = SD.open("/");
    while (true) {
      File entry = root.openNextFile();
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
    
    // Calculate percentage if totalSpace is valid
    if (totalSpace > 0) {
      uint8_t percentUsed = (usedSpace * 100UL) / totalSpace;
      Serial.print(F("SD Card usage: "));
      Serial.print(percentUsed);
      Serial.println(F("%"));
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
}

// Function to create a new log file with timestamp in name
void createNewLogFile() {
  // Format: DATA_YYYYMMDD_HHMMSS.CSV
  char fileName[32];
  
  // Try to get date/time from GPS if available
  if (SIV > 0) { // If we have satellite fix
    sprintf(fileName, "DATA_%04d%02d%02d_%02d%02d%02d.CSV", 
      myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay(),
      myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond());
  } else {
    // Fallback to numbered files if no GPS time available
    int fileCount = 0;
    while (fileCount < 9999) {
      sprintf(fileName, "DATA_%04d.CSV", fileCount);
      if (!SD.exists(fileName) && 
          !SerialFlash.exists(fileName)) {
        break;
      }
      fileCount++;
    }
  }
  
  // Copy filename to global variables
  strcpy(logFileName, fileName);
  
  // Create and write header to the SD file
  if (sdCardAvailable) {
    LogDataFile = SD.open(fileName, FILE_WRITE);
    if (LogDataFile) {
      // Write CSV header
      LogDataFile.println(F("Timestamp,FixType,Sats,Lat,Long,Alt,AltMSL,Speed,Heading,pDOP,RTK,"
                          "Pressure,Temperature,"
                          "KX134_AccelX,KX134_AccelY,KX134_AccelZ,"
                          "ICM_AccelX,ICM_AccelY,ICM_AccelZ,"
                          "ICM_GyroX,ICM_GyroY,ICM_GyroZ"));
      LogDataFile.flush();
      LogDataFile.close();
      Serial.print(F("Created SD log file: "));
      Serial.println(fileName);
    } else {
      Serial.println(F("Error creating SD log file!"));
    }
  }
  
  // Create file in SerialFlash if available
  if (flashAvailable) {
    if (SerialFlash.exists(fileName)) {
      SerialFlash.remove(fileName);
    }
    
    // Create file with estimated size (adjust as needed)
    const int fileSize = 512 * 1024; // 512KB
    if (SerialFlash.create(fileName, fileSize)) {
      SerialFlashFile flashFile = SerialFlash.open(fileName);
      if (flashFile) {
        // Write header
        flashFile.write("Timestamp,FixType,Sats,Lat,Long,Alt,AltMSL,Speed,Heading,pDOP,RTK,"
                       "Pressure,Temperature,"
                       "KX134_AccelX,KX134_AccelY,KX134_AccelZ,"
                       "ICM_AccelX,ICM_AccelY,ICM_AccelZ,"
                       "ICM_GyroX,ICM_GyroY,ICM_GyroZ\n", 180);
        flashFile.close();
        Serial.print(F("Created external flash log file: "));
        Serial.println(fileName);
      }
    } else {
      Serial.println(F("Error creating external flash log file!"));
    }
  }
  
  // For internal flash, we rely on initInternalFlash which already initialized the file
  if (internalFlashAvailable) {
    Serial.println(F("Internal flash logging ready"));
  }
}

void WriteLogData(bool forceLog = false) {
  static unsigned long lastInternalLogTime = 0;
  
  // Only log at specified intervals or when forced (after sensor reading)
  if (!forceLog && millis() - lastLogTime < LOG_INTERVAL_MS) {
    return;
  }
  lastLogTime = millis();
  
  // Format the data string for SD card and SerialFlash
  char dataString[320]; // Make sure this is large enough
  int milliseconds = millis() % 1000;
  
  // Format: timestamp,fixType,sats,lat,long,alt,altMSL,speed,heading,pDOP,RTK,pressure,temp,kx_ax,kx_ay,kx_az,icm_ax,icm_ay,icm_az,icm_gx,icm_gy,icm_gz
  sprintf(dataString, "%lu.%03d,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
          millis() / 1000, milliseconds,
          GPS_fixType, SIV, 
          GPS_latitude, GPS_longitude, GPS_altitude, GPS_altitudeMSL, 
          GPS_speed, GPS_heading, pDOP / 100.0, RTK,
          ms5611Sensor.getPressure(), ms5611Sensor.getTemperature(),
          kx134_x, kx134_y, kx134_z,
          icm_accel_x, icm_accel_y, icm_accel_z,
          icm_gyro_x, icm_gyro_y, icm_gyro_z);
  
  // Log to SD card
  if (sdCardAvailable && strlen(logFileName) > 0) {
    LogDataFile = SD.open(logFileName, FILE_WRITE);
    if (LogDataFile) {
      LogDataFile.println(dataString);
      LogDataFile.flush(); // Make sure it's written
      LogDataFile.close();
    }
  }
  
  // Log to Serial Flash
  if (flashAvailable && strlen(logFileName) > 0) {
    SerialFlashFile flashFile = SerialFlash.open(logFileName);
    if (flashFile) {
      // Find the end of the file to append
      flashFile.seek(flashFile.size());
      
      // Append the data
      char newLine[2] = "\n";
      flashFile.write(dataString, strlen(dataString));
      flashFile.write(newLine, 1);
      flashFile.close();
    }
  }
  
  // Log to internal flash (LittleFS) at a reduced rate (5Hz)
  if (internalFlashAvailable && (forceLog || millis() - lastInternalLogTime >= FLASH_LOG_INTERVAL)) {
    lastInternalLogTime = millis();
    
    // Create a 46-byte record per log entry (expanded from 40 to include gyro data)
    uint8_t logRecord[46];
    
    // Record format:
    // [0-3]: Timestamp (32-bit unsigned long)
    // [4-5]: FixType and SIV packed (8 bits each)
    // [6-9]: Latitude (32-bit long)
    // [10-13]: Longitude (32-bit long)
    // [14-17]: Altitude (32-bit long)
    // [18-19]: Speed (16-bit int - mm/s)
    // [20-21]: Pressure (16-bit int - Pa/10)
    // [22-23]: Temperature (16-bit int - °C*100)
    // [24-25]: KX134 AccelX (16-bit int - g*1000)
    // [26-27]: KX134 AccelY (16-bit int - g*1000)
    // [28-29]: KX134 AccelZ (16-bit int - g*1000)
    // [30-31]: ICM_20948 AccelX (16-bit int - g*1000)
    // [32-33]: ICM_20948 AccelY (16-bit int - g*1000)
    // [34-35]: ICM_20948 AccelZ (16-bit int - g*1000)
    // [36-37]: ICM_20948 GyroX (16-bit int - deg/s*10)
    // [38-39]: ICM_20948 GyroY (16-bit int - deg/s*10)
    // [40-41]: ICM_20948 GyroZ (16-bit int - deg/s*10)
    // [42-43]: Reserved for future use
    // [44-45]: Checksum (16-bit)
    
    // Timestamp
    unsigned long timestamp = millis();
    logRecord[0] = timestamp & 0xFF;
    logRecord[1] = (timestamp >> 8) & 0xFF;
    logRecord[2] = (timestamp >> 16) & 0xFF;
    logRecord[3] = (timestamp >> 24) & 0xFF;
    
    // Fix type and SIV
    logRecord[4] = GPS_fixType;
    logRecord[5] = SIV;
    
    // Latitude (convert from degrees*10^-7 to a more compact form if needed)
    logRecord[6] = GPS_latitude & 0xFF;
    logRecord[7] = (GPS_latitude >> 8) & 0xFF;
    logRecord[8] = (GPS_latitude >> 16) & 0xFF;
    logRecord[9] = (GPS_latitude >> 24) & 0xFF;
    
    // Longitude
    logRecord[10] = GPS_longitude & 0xFF;
    logRecord[11] = (GPS_longitude >> 8) & 0xFF;
    logRecord[12] = (GPS_longitude >> 16) & 0xFF;
    logRecord[13] = (GPS_longitude >> 24) & 0xFF;
    
    // Altitude
    logRecord[14] = GPS_altitude & 0xFF;
    logRecord[15] = (GPS_altitude >> 8) & 0xFF;
    logRecord[16] = (GPS_altitude >> 16) & 0xFF;
    logRecord[17] = (GPS_altitude >> 24) & 0xFF;
    
    // Speed (16-bit)
    int16_t speed = constrain(GPS_speed, -32768, 32767);
    logRecord[18] = speed & 0xFF;
    logRecord[19] = (speed >> 8) & 0xFF;
    
    // Pressure (hPa * 10 to keep 1 decimal place)
    int16_t pressure = constrain((int)(ms5611Sensor.getPressure() * 10), -32768, 32767);
    logRecord[20] = pressure & 0xFF;
    logRecord[21] = (pressure >> 8) & 0xFF;
    
    // Temperature (°C * 100 to keep 2 decimal places)
    int16_t temperature = constrain((int)(ms5611Sensor.getTemperature() * 100), -32768, 32767);
    logRecord[22] = temperature & 0xFF;
    logRecord[23] = (temperature >> 8) & 0xFF;
    
    // KX134 Accelerometer data (g * 1000 to keep 3 decimal places)
    int16_t kx_accelX = constrain((int)(kx134_x * 1000), -32768, 32767);
    int16_t kx_accelY = constrain((int)(kx134_y * 1000), -32768, 32767);
    int16_t kx_accelZ = constrain((int)(kx134_z * 1000), -32768, 32767);
    
    logRecord[24] = kx_accelX & 0xFF;
    logRecord[25] = (kx_accelX >> 8) & 0xFF;
    logRecord[26] = kx_accelY & 0xFF;
    logRecord[27] = (kx_accelY >> 8) & 0xFF;
    logRecord[28] = kx_accelZ & 0xFF;
    logRecord[29] = (kx_accelZ >> 8) & 0xFF;
    
    // ICM-20948 Accelerometer data (g * 1000 to keep 3 decimal places)
    int16_t icm_accelX = constrain((int)(icm_accel_x * 1000), -32768, 32767);
    int16_t icm_accelY = constrain((int)(icm_accel_y * 1000), -32768, 32767);
    int16_t icm_accelZ = constrain((int)(icm_accel_z * 1000), -32768, 32767);
    
    logRecord[30] = icm_accelX & 0xFF;
    logRecord[31] = (icm_accelX >> 8) & 0xFF;
    logRecord[32] = icm_accelY & 0xFF;
    logRecord[33] = (icm_accelY >> 8) & 0xFF;
    logRecord[34] = icm_accelZ & 0xFF;
    logRecord[35] = (icm_accelZ >> 8) & 0xFF;
    
    // ICM-20948 Gyroscope data (deg/s * 10 to keep 1 decimal place)
    int16_t icm_gyroX = constrain((int)(icm_gyro_x * 10), -32768, 32767);
    int16_t icm_gyroY = constrain((int)(icm_gyro_y * 10), -32768, 32767);
    int16_t icm_gyroZ = constrain((int)(icm_gyro_z * 10), -32768, 32767);
    
    logRecord[36] = icm_gyroX & 0xFF;
    logRecord[37] = (icm_gyroX >> 8) & 0xFF;
    logRecord[38] = icm_gyroY & 0xFF;
    logRecord[39] = (icm_gyroY >> 8) & 0xFF;
    logRecord[40] = icm_gyroZ & 0xFF;
    logRecord[41] = (icm_gyroZ >> 8) & 0xFF;
    
    // Reserved bytes
    logRecord[42] = 0;
    logRecord[43] = 0;
    
    // Simple checksum (sum of all bytes)
    uint16_t checksum = 0;
    for (int i = 0; i < 44; i++) {
      checksum += logRecord[i];
    }
    logRecord[44] = checksum & 0xFF;
    logRecord[45] = (checksum >> 8) & 0xFF;
    
    // Write to the log file (with updated record size)
    if (flashLogFile) {
      flashLogFile.write(logRecord, 46);
      flashLogFile.flush();
      recordCount++;
      
      // Every 50 records (10 seconds at 5Hz), print status
      if (recordCount % 50 == 0) {
        unsigned long fileSize = flashLogFile.size();
        unsigned long maxSize = INTERNAL_FLASH_SIZE - 4096; // Reserve some space
        int percentUsed = (fileSize * 100) / maxSize;
        
        Serial.print(F("Internal flash log: written "));
        Serial.print(recordCount);
        Serial.print(F(" records, using "));
        Serial.print(fileSize);
        Serial.print(F(" of ~"));
        Serial.print(maxSize);
        Serial.print(F(" bytes ("));
        Serial.print(percentUsed);
        Serial.println(F("%)"));
        
        // If we're getting close to full, warn
        if (percentUsed > 90) {
          Serial.println(F("WARNING: Internal flash log is nearly full!"));
        }
      }
    }
  }
}

void formatNumber(float input, byte columns, byte places) 
{
  char buffer[20]; // Allocate space to store the formatted number string
  dtostrf(input, columns, places, buffer); // Convert float to string with specified columns and decimal places
  Serial.print(buffer); // Print the formatted number to the serial monitor
}

void kx134_init(){
  if (kxAccel.softwareReset())
    Serial.println("Reset.");
  // Give some time for the accelerometer to reset.
  // It needs two, but give it five for good measure.
  delay(50);
  // Many settings for KX13X can only be
  // applied when the accelerometer is powered down.
  // However there are many that can be changed "on-the-fly"
  // check datasheet for more info, or the comments in the
  // "...regs.h" file which specify which can be changed when.
  // So we disable the accelerometer
  kxAccel.enableAccel(false);
  // Do stuff
  // kxAccel.setRange(SFE_KX132_RANGE16G); // 16g Range
  kxAccel.setRange(SFE_KX134_RANGE64G);         // 64g for the KX134
  kxAccel.enableDataEngine(); // Enables the bit that indicates data is ready.
  kxAccel.setOutputDataRate(50); // Default is 50Hz
  // Re-enable the accelerometer
  kxAccel.enableAccel(); 
  kxAccel.begin();
}
void kx134_read(){
  // Query the KX134
  if (kxAccel.dataReady())
  {
    kxAccel.getAccelData(&kx134AccelData);
    kx134_x = kx134AccelData.xData;
    kx134_y = kx134AccelData.yData;
    kx134_z = kx134AccelData.zData;
  }
}
void kx134_print(){
  Serial.print(F("  KX134: X:"));
  Serial.print(kx134_x, 2);
  Serial.print(F("g Y:"));
  Serial.print(kx134_y, 2);
  Serial.print(F("g Z:"));
  Serial.print(kx134_z, 2);
  Serial.println(F("g"));
}
void ICM_20948_init(){
    // Sparkfun 9DOF sensor setup then wait for use input
  Serial.println(F("ICM-20948 Example"));
  // myICM.enableDebugging(); // Comment this line to disable debug messages on Serial
  myICM.begin(Wire, AD0_VAL);
  myICM.enableDebugging();
    // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  bool ICM_20948_initialized = false;
  while (!ICM_20948_initialized)
  {
    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println(F("Trying again..."));
      delay(250);
    }
    else
    {
      ICM_20948_initialized = true;
    }
  Serial.println(F("Device connected!"));

  // Here we are doing a SW reset to make sure the device starts in a known state
  myICM.swReset();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("Software Reset returned: "));
    Serial.println(myICM.statusString());
  }
  delay(250);

  // Now wake the sensor up
  myICM.sleep(false);
  myICM.lowPower(false);

  // The next few configuration functions accept a bit-mask of sensors for which the settings should be applied.

  // Set Gyro and Accelerometer to a particular sample mode
  // options: ICM_20948_Sample_Mode_Continuous
  //          ICM_20948_Sample_Mode_Cycled
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setSampleMode returned: "));
    Serial.println(myICM.statusString());
  }
  
  // Set full scale ranges for both acc and gyr
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

  myFSS.a = gpm16; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                  // gpm2
                  // gpm4
                  // gpm8
                  // gpm16

  myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                    // dps250
                    // dps500
                    // dps1000
                    // dps2000

  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setFullScale returned: "));
    Serial.println(myICM.statusString());
  }

  // Set up Digital Low-Pass Filter configuration
  ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
  myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                  // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                  // acc_d111bw4_n136bw
                                  // acc_d50bw4_n68bw8
                                  // acc_d23bw9_n34bw4
                                  // acc_d11bw5_n17bw
                                  // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                  // acc_d473bw_n499bw

  myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                    // gyr_d196bw6_n229bw8
                                    // gyr_d151bw8_n187bw6
                                    // gyr_d119bw5_n154bw3
                                    // gyr_d51bw2_n73bw3
                                    // gyr_d23bw9_n35bw9
                                    // gyr_d11bw6_n17bw8
                                    // gyr_d5bw7_n8bw9
                                    // gyr_d361bw4_n376bw5

  myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("setDLPcfg returned: "));
    Serial.println(myICM.statusString());
  }

  // Choose whether or not to use DLPF
  // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
  ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, false);
  ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, false);
  Serial.print(F("Enable DLPF for Accelerometer returned: "));
  Serial.println(myICM.statusString(accDLPEnableStat));
  Serial.print(F("Enable DLPF for Gyroscope returned: "));
  Serial.println(myICM.statusString(gyrDLPEnableStat));

  // Choose whether or not to start the magnetometer
  myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    Serial.print(F("startupMagnetometer returned: "));
    Serial.println(myICM.statusString());
  }

  Serial.println();
  Serial.println(F("Configuration complete!"));


  }
  // After successful ICM initialization
  delay(500); // Wait for sensor to stabilize
}
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    Serial.print("-");
  }
  else
  {
    Serial.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      Serial.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    Serial.print(-val, decimals);
  }
  else
  {
    Serial.print(val, decimals);
  }
}
void printScaledAGMT(ICM_20948_I2C *sensor)
{
  Serial.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  Serial.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  Serial.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  Serial.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  Serial.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  Serial.print(" ]");
  Serial.println();
}
void ICM_20948_read(){

  if (myICM.dataReady())
  {
    myICM.getAGMT();              // The values are only updated when you call 'getAGMT'
    //printRawAGMT( myICM.agmt ); // Uncomment this to see the raw values, taken directly from the agmt structure
    //printScaledAGMT(&myICM);      // This function takes into account the scale settings from when the measurement was made to calculate the values with units

    // Convert from mg to g
    icm_accel_x = myICM.accX() / 1000.0;
    icm_accel_y = myICM.accY() / 1000.0;
    icm_accel_z = myICM.accZ() / 1000.0;

    icm_mag_x = myICM.magX(); // Extract the compass data
    icm_mag_y = myICM.magY();
    icm_mag_z = myICM.magZ();

    icm_gyro_x = myICM.gyrX(); // Extract the raw gyro data
    icm_gyro_y = myICM.gyrY();
    icm_gyro_z = myICM.gyrZ();

    delay(30);
  }
  else
  {
    Serial.println("Waiting for data");
    delay(500);
  }

}
int ms5611_read(){
  start = micros();
  int result = ms5611Sensor.read();
  stop = micros();
  
  // Don't print debug messages here - they'll be handled in ms5611_print function
  // Just return the result so the caller can check if needed
  return result;
}
void ms5611_init(){
  Serial.println(__FILE__);
  Serial.print("MS5611_LIB_VERSION: ");
  Serial.println(MS5611_LIB_VERSION);

  if (ms5611Sensor.begin() == true)
  {
    Serial.print("MS5611 found: ");
    Serial.println(ms5611Sensor.getAddress());
  }
  else
  {
    Serial.println("MS5611 not found. halt.");
    delay(500);
  }
  /*
   There are 5 oversampling settings, each corresponding to a different amount of milliseconds
   We are using OSR_HIGH which will take 4.11 millis. OSR_ULTRA_HIGH is roughly double
   Read the Doco if you want to set something different
  */
  ms5611Sensor.setOversampling(OSR_HIGH);
  // ms5611Sensor.setOversampling(OSR_ULTRA_HIGH);
  int result = ms5611_read();
  if (result != MS5611_READ_OK) {
    Serial.print("MS5611 read error during init: ");
    Serial.println(result);
  } else {
    Serial.println("MS5611 successfully initialized and read");
  }
}
// Function to print MS5611 data to serial
void ms5611_print() {
  Serial.print("MS5611 Data\n");
  Serial.print("T: ");
  Serial.print(ms5611Sensor.getTemperature(), 2);
  Serial.print(" P: ");
  Serial.print(ms5611Sensor.getPressure(), 2);
  Serial.println();
}
void scan_i2c() {
  Serial.println(F("\nI2C Scanner"));
  Serial.println(F("Scanning..."));

  byte error, address;
  int nDevices = 0;

  for(address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Wire.endTransmission to see if
    // a device acknowledged the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("I2C device found at address 0x"));
      if (address < 16) 
        Serial.print("0");
      Serial.print(address, HEX);
      
      // Try to identify known devices
      if (address == 0x42) Serial.print(F(" - SparkFun ZOE-M8Q GPS Module"));
      if (address == 0x77) Serial.print(F(" - MS5611 Barometric Pressure Sensor"));
      if (address == 0x69) Serial.print(F(" - ICM-20948 9-DOF IMU"));
      if (address == 0x1F) Serial.print(F(" - SparkFun KX134 Accelerometer"));
      Serial.println();
      nDevices++;
    }
    else if (error == 4) {
      Serial.print(F("Unknown error at address 0x"));
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if (nDevices == 0)
    Serial.println(F("No I2C devices found\n"));
  else
    Serial.println(F("I2C scan complete\n"));
}

bool checkGPSConnection() {
  // Check if we can communicate with the GPS module
  // Query the navigation rate to see if we get a valid response
  byte rate = myGNSS.getNavigationFrequency();
  
  if (rate > 0) {
    // We got a response
    Serial.print(F("GPS communication success! Nav rate: "));
    Serial.println(rate);
    return true;
  } else {
    Serial.println(F("WARNING: No response from GPS when querying navigation rate."));
    return false;
  }
}

// Function to initialize the SD card
bool initSDCard() {
  Serial.println(F("Initializing SD card..."));
  
  #if defined(BOARD_STM32_THING_PLUS)
    // STM32 SD card initialization - may need special handling
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH); // Ensure CS is high initially
    
    // Initialize SD card with explicit parameters
    if (!SD.begin(SD_CS_PIN, SPI_HALF_SPEED)) {
      Serial.println(F("SD Card initialization failed!"));
      return false;
    }
  #else
    // Standard Teensy/Arduino SD card initialization
    if (!SD.begin(SDchipSelect)) {
      Serial.println(F("SD Card initialization failed!"));
      return false;
    }
  #endif
  
  Serial.println(F("SD Card initialized."));
  return true;
}

// Function to initialize the Serial Flash memory
bool initSerialFlash() {
  Serial.println(F("Initializing Serial Flash..."));
  
  #if defined(BOARD_STM32_THING_PLUS)
    // STM32 may need a different approach for external flash
    // SerialFlash library is Teensy-specific
    Serial.println(F("SerialFlash not supported on STM32 yet"));
    Serial.println(F("Using internal LittleFS instead"));
    return false;
  #else
    // Check if the flash memory is present - Teensy-specific SerialFlash
    if (!SerialFlash.begin(FLASH_CS_PIN)) {
      Serial.println(F("Serial Flash initialization failed!"));
      return false;
    }
    
    Serial.println(F("Serial Flash initialized."));
    return true;
  #endif
}
void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    Serial.print(" ");
    if (val < 10000)
    {
      Serial.print("0");
    }
    if (val < 1000)
    {
      Serial.print("0");
    }
    if (val < 100)
    {
      Serial.print("0");
    }
    if (val < 10)
    {
      Serial.print("0");
    }
  }
  else
  {
    Serial.print("-");
    if (abs(val) < 10000)
    {
      Serial.print("0");
    }
    if (abs(val) < 1000)
    {
      Serial.print("0");
    }
    if (abs(val) < 100)
    {
      Serial.print("0");
    }
    if (abs(val) < 10)
    {
      Serial.print("0");
    }
  }
  Serial.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  Serial.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  Serial.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  Serial.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  Serial.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  Serial.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  Serial.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  Serial.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  Serial.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  Serial.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  Serial.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  Serial.print(" ]");
  Serial.println();
}


// Function to print ICM_20948 data to serial
void ICM_20948_print() {
  if (myICM.dataReady())
  {
    //myICM.getAGMT();              // The values are only updated when you call 'getAGMT'
    //printRawAGMT( myICM.agmt ); // Uncomment this to see the raw values, taken directly from the agmt structure
    printScaledAGMT(&myICM);      // This function takes into account the scale settings from when the measurement was made to calculate the values with units
    delay(30);
  }
  else
  {
    Serial.println("Waiting for data");
    delay(500);
  }



  Serial.println(F("-------------------"));
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

// Function to handle the flash data check prompt
void handleFlashDataCheck() {
  bool hasMoreFiles = true;
  
  while (hasMoreFiles) {
    if (!checkInternalFlashData()) {
      // No more files left
      Serial.println(F("No more log files in flash memory."));
      hasMoreFiles = false;
      break;
    }
    
    Serial.println(F("\nFlash data options:"));
    Serial.println(F("1. Dump and delete next log file"));
    Serial.println(F("2. Continue without dumping"));
    Serial.println(F("Enter choice (1-2) [waiting 10 seconds]:"));
    
    // Wait for response with timeout
    unsigned long startTime = millis();
    bool responseReceived = false;
    char response = '0';
    
    while (millis() - startTime < 30000) { // 30 second timeout
      // Blink LED to indicate waiting for user input
      if ((millis() / 500) % 2 == 0) {
        pixels.setPixelColor(0, pixels.Color(50, 0, 50)); // Purple blinking to indicate waiting
      } else {
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      }
      pixels.show();
      
      if (Serial.available()) {
        response = Serial.read();
        responseReceived = true;
        break;
      }
    }
    
    if (!responseReceived || response == '2') {
      // No response or user chose to continue
      Serial.println(F("Continuing without dumping files."));
      pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green for completed
      pixels.show();
      delay(1000);
      break;
    } else if (response == '1') {
      // Dump and delete the next log file
      pixels.setPixelColor(0, pixels.Color(0, 50, 50)); // Cyan for data dumping
      pixels.show();
      
      Serial.println(F("Dumping and deleting next log file..."));
      String dumpedFile = dumpInternalFlashData();
      
      if (dumpedFile.length() > 0) {
        // Delete the file automatically
        pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red for deleting
        pixels.show();
        
        Serial.print(F("Deleting file: "));
        Serial.println(dumpedFile);
        
        if (flashFS.remove(("/" + dumpedFile).c_str())) {
          Serial.println(F("File deleted successfully."));
        } else {
          Serial.println(F("Failed to delete file!"));
        }
        
        delay(1000); // Pause to show deletion indication
      }
      
      // Check if there are more files
      hasMoreFiles = checkInternalFlashData();
      if (!hasMoreFiles) {
        Serial.println(F("No more log files remaining."));
        pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green for completed
        pixels.show();
        delay(1000);
      }
    } else {
      // Invalid response
      Serial.println(F("Invalid choice, continuing without dumping."));
      pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green for completed
      pixels.show();
      delay(1000);
      break;
    }
  }
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
  
  // GPS INFORMATION - Show compact format
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
  Serial.print(F(" | Sats: "));
  Serial.print(SIV);
  Serial.print(F(" | DOP: "));
  Serial.println(pDOP / 100.0, 1);
  
  gps_print();
  // SENSORS - Combine atmospheric and IMU data in a compact format
  Serial.println(F("SENSORS:"));
  Serial.print(F("  Baro: "));
  Serial.print(ms5611Sensor.getPressure(), 1);
  Serial.print(F(" hPa, "));
  Serial.print(ms5611Sensor.getTemperature(), 1);
  Serial.println(F("°C"));
  
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
  Serial.println(F("Commands: help, dump, stats, imu, detail"));
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
    File root = SD.open("/");
    while (true) {
      File entry = root.openNextFile();
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
  
  Serial.println("There will be lots of debug output to serial that will need to be removed later");

  // Initialize sensors
  kxAccel.begin();
  Serial.println(F("KX134 accelerometer initialized"));
  
  // Initialize LittleFS first to check for existing data
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

  // Now initialize everything else
  // Scan the I2C bus for devices
  scan_i2c();
  ms5611_init();
  gps_init();
  ICM_20948_init();
  kx134_init();
  
  // Initialize storage for data logging
  Serial.println(F("Initializing data logging..."));
  sdCardAvailable = initSDCard();
  flashAvailable = initSerialFlash();
  
  // Only initialize internal flash if LittleFS initialization succeeded earlier
  if (internalFlashAvailable) {
    internalFlashAvailable = initInternalFlash();
  }
  
  // Report storage initialization status
  Serial.println(F("\nStorage Status:"));
  Serial.print(F("- SD Card: "));
  Serial.println(sdCardAvailable ? F("Available") : F("Not available"));
  Serial.print(F("- External Flash: "));
  Serial.println(flashAvailable ? F("Available") : F("Not available"));
  Serial.print(F("- Internal Flash: "));
  Serial.println(internalFlashAvailable ? F("Available") : F("Not available"));
  
  if (sdCardAvailable || flashAvailable || internalFlashAvailable) {
    Serial.println(F("Creating new log file..."));
    createNewLogFile();
    Serial.println(F("Data logging ready."));
  } else {
    Serial.println(F("WARNING: No storage available for data logging!"));
  }
  
  Serial.println();
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
  Serial.print(myGNSS.getSecond());

  while (Serial.available()) // Make sure the serial RX buffer is empty
    Serial.read();
  Serial.println(F("Press any key to continue..."));
  
  // Force a read of all sensors immediately
  gps_read();
  ms5611_read();
  ICM_20948_read();
  kx134_read();
  
  // Test output of sensor data
  Serial.println(F("\n===== SENSOR TEST ====="));
  
  // KX134 accelerometer
  Serial.println(F("KX134 Accelerometer data:"));
  Serial.print(F("X: ")); Serial.print(kx134_x, 4); Serial.print(F("g, "));
  Serial.print(F("Y: ")); Serial.print(kx134_y, 4); Serial.print(F("g, "));
  Serial.print(F("Z: ")); Serial.print(kx134_z, 4); Serial.println(F("g"));
  
  // ICM-20948 IMU
  Serial.println(F("ICM-20948 IMU data:"));
  if (icm_data_available) {
    if ((icm_data_header & DMP_header_bitmap_Accel) > 0) {
      Serial.print(F("Accel X: ")); Serial.print(icm_accel_x); Serial.print(F(", "));
      Serial.print(F("Y: ")); Serial.print(icm_accel_y); Serial.print(F(", "));
      Serial.print(F("Z: ")); Serial.println(icm_accel_z);
    } else {
      Serial.println(F("No accelerometer data available"));
    }
    
    if ((icm_data_header & DMP_header_bitmap_Gyro) > 0) {
      Serial.print(F("Gyro X: ")); Serial.print(icm_gyro_x); Serial.print(F(", "));
      Serial.print(F("Y: ")); Serial.print(icm_gyro_y); Serial.print(F(", "));
      Serial.print(F("Z: ")); Serial.println(icm_gyro_z);
    }
    
    if ((icm_data_header & DMP_header_bitmap_Quat6) > 0) {
      Serial.print(F("Quaternion: Q1=")); Serial.print(icm_q1, 3);
      Serial.print(F(", Q2=")); Serial.print(icm_q2, 3);
      Serial.print(F(", Q3=")); Serial.println(icm_q3, 3);
    }
  } else {
    Serial.println(F("No IMU data available"));
  }
  
  // Initialize servos
  Serial.println(F("\nInitializing servos..."));
  #if defined(USE_STD_SERVO_LIB)
    // Standard Arduino Servo library
    TVCX.attach(TVCXpin);
    TVCY.attach(TVCYpin);
    TVCX.write(TVCXpos);
    TVCY.write(TVCYpos);
    Serial.println(F("Standard servo library initialized"));
  #else
    // Teensy PWMServo library
    TVCX.attach(TVCXpin);
    TVCY.attach(TVCYpin);
    TVCX.write(TVCXpos);
    TVCY.write(TVCYpos);
    Serial.println(F("Teensy PWMServo library initialized"));
  #endif
  
  Serial.println(F("======================\n"));
  Serial.println(F("Press any key to continue..."));
  
  // Wait for user input with timeout
  unsigned long startWaitTime = millis();
  while (!Serial.available() && (millis() - startWaitTime < 10000)) {
    // Wait up to 10 seconds for user input
    delay(100);
  }
  
  if (Serial.available()) {
    // Clear any received characters
    while (Serial.available()) {
      Serial.read();
    }
    Serial.println(F("User input received, continuing..."));
  } else {
    Serial.println(F("Timeout waiting for user input, continuing automatically..."));
  }
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


