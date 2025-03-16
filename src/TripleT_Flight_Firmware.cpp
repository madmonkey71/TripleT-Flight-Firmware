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

// Libraries for all of the sensor hardware
// KX134 Accelerometer
#include <SparkFun_KX13X.h> // Click here to get the library: http://librarymanager/All#SparkFun_KX13X
// Ublox M8Q GPS Module
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
// MS5611 Barometric Pressure Sensor
#include "MS5611.h"
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

// Start creating all of the variables needed to pull data or other info
// Separate it out into specific blocks for the various sensors

#define TRIPLET_FLIGHT_VERSION 0.10
// Comment out to restrict roll to ±90deg instead - please read: https://www.nxp.com/docs/en/application-note/AN3461.pdf
// #define RESTRICT_PITCH 

// MS5611 Sensor
// For some reason this throws an error but it still works
// Needs investigation
MS5611 MS5611(0x77);
uint32_t start, stop;

// Sparkfun ZOE-M8Q
// Put all the GPS variables here
SFE_UBLOX_GNSS myGPS;
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.
long GPS_latitude =  0;
long GPS_longitude =  0;
long GPS_altitude =  0;
long GPS_altitudeMSL = 0;
long GPS_speed =  0;
long GPS_heading =  0;
int pDOP =  0;
byte SIV =  0;
byte GPS_fixType =  0;
byte RTK =  0;

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
#define SDchipSelect 10
#define RESET_BUTTON_PIN NRST
#define MODE_BUTTON_PIN PC15 // the number of the Mode Button Pin

// Initialise the barometric sensor data variables.
float Temperature = 0;
float Pressure = 0;
float baro_altitude = 0;
float seaLevel = 0;

//Setup parameters for the Servo Library
PWMServo TVCX;
PWMServo TVCY;

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
#define FLASH_LOG_INTERVAL 200  // Log at 5Hz (every 200ms)

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

// Function to initialize the internal flash memory of the Teensy
bool initInternalFlash() {
  Serial.println(F("Initializing Teensy internal flash with LittleFS..."));
  
  // Initialize LittleFS with 512KB of space
  if (!flashFS.begin(INTERNAL_FLASH_SIZE)) {
    Serial.println(F("Failed to initialize LittleFS!"));
    return false;
  }
  
  // Generate a new log file name with timestamp or counter
  if (SIV > 0) { // If we have satellite fix
    sprintf(flashLogPath, "/log_%04d%02d%02d_%02d%02d%02d.bin", 
      myGPS.getYear(), myGPS.getMonth(), myGPS.getDay(),
      myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond());
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
  
  Serial.print(F("Internal flash initialized with LittleFS, log file: "));
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
    
    Serial.print(F("SD Card used space: "));
    Serial.print(usedSpace / 1024);
    Serial.println(F(" KB"));
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
      myGPS.getYear(), myGPS.getMonth(), myGPS.getDay(),
      myGPS.getHour(), myGPS.getMinute(), myGPS.getSecond());
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
                          "AccelX,AccelY,AccelZ,"
                          "GyroX,GyroY,GyroZ"));
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
                       "AccelX,AccelY,AccelZ,"
                       "GyroX,GyroY,GyroZ\n", 135);
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
  char dataString[256]; // Make sure this is large enough
  int milliseconds = millis() % 1000;
  
  // Format: timestamp,fixType,sats,lat,long,alt,altMSL,speed,heading,pDOP,RTK,pressure,temp,ax,ay,az
  sprintf(dataString, "%lu.%03d,%d,%d,%ld,%ld,%ld,%ld,%ld,%ld,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f",
          millis() / 1000, milliseconds,
          GPS_fixType, SIV, 
          GPS_latitude, GPS_longitude, GPS_altitude, GPS_altitudeMSL, 
          GPS_speed, GPS_heading, pDOP / 100.0, RTK,
          MS5611.getPressure(), MS5611.getTemperature(),
          kx134_x, kx134_y, kx134_z);
  
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
    
    // Create a 32-byte record per log entry (same format as before)
    uint8_t logRecord[32];
    
    // Record format:
    // [0-3]: Timestamp (32-bit unsigned long)
    // [4-5]: FixType and SIV packed (8 bits each)
    // [6-9]: Latitude (32-bit long)
    // [10-13]: Longitude (32-bit long)
    // [14-17]: Altitude (32-bit long)
    // [18-19]: Speed (16-bit int - mm/s)
    // [20-21]: Pressure (16-bit int - Pa/10)
    // [22-23]: Temperature (16-bit int - °C*100)
    // [24-25]: AccelX (16-bit int - g*1000)
    // [26-27]: AccelY (16-bit int - g*1000)
    // [28-29]: AccelZ (16-bit int - g*1000)
    // [30-31]: Checksum (16-bit)
    
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
    int16_t pressure = constrain((int)(MS5611.getPressure() * 10), -32768, 32767);
    logRecord[20] = pressure & 0xFF;
    logRecord[21] = (pressure >> 8) & 0xFF;
    
    // Temperature (°C * 100 to keep 2 decimal places)
    int16_t temperature = constrain((int)(MS5611.getTemperature() * 100), -32768, 32767);
    logRecord[22] = temperature & 0xFF;
    logRecord[23] = (temperature >> 8) & 0xFF;
    
    // Accelerometer data (g * 1000 to keep 3 decimal places)
    int16_t accelX = constrain((int)(kx134_x * 1000), -32768, 32767);
    int16_t accelY = constrain((int)(kx134_y * 1000), -32768, 32767);
    int16_t accelZ = constrain((int)(kx134_z * 1000), -32768, 32767);
    
    logRecord[24] = accelX & 0xFF;
    logRecord[25] = (accelX >> 8) & 0xFF;
    logRecord[26] = accelY & 0xFF;
    logRecord[27] = (accelY >> 8) & 0xFF;
    logRecord[28] = accelZ & 0xFF;
    logRecord[29] = (accelZ >> 8) & 0xFF;
    
    // Simple checksum (sum of all bytes)
    uint16_t checksum = 0;
    for (int i = 0; i < 30; i++) {
      checksum += logRecord[i];
    }
    logRecord[30] = checksum & 0xFF;
    logRecord[31] = (checksum >> 8) & 0xFF;
    
    // Write to the log file
    if (flashLogFile) {
      flashLogFile.write(logRecord, 32);
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
void gps_init() {
  Serial.println(F("Initializing GPS module..."));
  
  // Try to connect to the GPS module - be explicit that we're using I2C
  Wire.begin();
  if (myGPS.begin(Wire) == false) {
    Serial.println(F("GPS module not detected on I2C bus. Please check wiring."));
    // Try with a different I2C address as a fallback
    Serial.println(F("Trying alternate I2C address..."));
    if (myGPS.begin(Wire, 0x42) == false) { // Explicitly try the default address
      Serial.println(F("GPS module still not detected. Check hardware."));
      return;
    }
  }
  
  Serial.println(F("GPS module found!"));
  
  // Configure the GPS module
  myGPS.setNavigationFrequency(10); //Set output to 10 times a second
  Serial.println(F("Navigation frequency set to 10Hz"));
  
  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  Serial.println(F("I2C output set to UBX"));
  
  // Enable automatic updates
  myGPS.setAutoPVT(true);
  Serial.println(F("Auto PVT enabled"));
  
  // Check hardware status
  Serial.println(F("GPS module configured and ready"));
  
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}
void gps_read() {
  // Query the GPS module only every second. Doing it more often will just cause I2C traffic.
  // The module only responds when a new position is available
  if (millis() - lastTime > 200)
  {
    lastTime = millis(); //Update the timer
    
    GPS_latitude = myGPS.getLatitude();
    GPS_longitude = myGPS.getLongitude();
    GPS_altitude = myGPS.getAltitude();
    GPS_altitudeMSL = myGPS.getAltitudeMSL();
    GPS_speed = myGPS.getGroundSpeed();
    GPS_heading = myGPS.getHeading();
    pDOP = myGPS.getPDOP();
    SIV = myGPS.getSIV();
    GPS_fixType = myGPS.getFixType();
    RTK = myGPS.getCarrierSolutionType();
  }
}
void gps_print(){
    Serial.println(F("\n----- GPS Data -----"));
    
    // Show fix information first (most important for debugging)
    Serial.print(F("Fix Type: "));
    if(GPS_fixType == 0) Serial.println(F("No fix - waiting for position"));
    else if(GPS_fixType == 1) Serial.println(F("Dead reckoning"));
    else if(GPS_fixType == 2) Serial.println(F("2D fix"));
    else if(GPS_fixType == 3) Serial.println(F("3D fix - Valid position"));
    else if(GPS_fixType == 4) Serial.println(F("GNSS + Dead reckoning"));
    else Serial.println(GPS_fixType);
    
    Serial.print(F("Satellites: "));
    Serial.println(SIV);
    
    Serial.print(F("Positional DOP: "));
    Serial.println(pDOP / 100.0, 2);
    
    // Print position data
    Serial.print(F("Lat: "));
    Serial.print(GPS_latitude);
    Serial.print(F(" Long: "));
    Serial.print(GPS_longitude);
    Serial.println(F(" (degrees * 10^-7)"));
    
    // For a more human-readable format, optional:
    Serial.print(F("Position: "));
    Serial.print(GPS_latitude / 10000000.0, 6);
    Serial.print(F(", "));
    Serial.println(GPS_longitude / 10000000.0, 6);
    
    Serial.print(F("Altitude: "));
    Serial.print(GPS_altitude);
    Serial.print(F(" mm ("));
    Serial.print(GPS_altitude / 1000.0, 1);
    Serial.println(F(" m)"));
    
    Serial.print(F("AltitudeMSL: "));
    Serial.print(GPS_altitudeMSL);
    Serial.print(F(" mm ("));
    Serial.print(GPS_altitudeMSL / 1000.0, 1);
    Serial.println(F(" m)"));
    
    Serial.print(F("Speed: "));
    Serial.print(GPS_speed);
    Serial.print(F(" mm/s ("));
    Serial.print(GPS_speed * 0.0036, 1); // Convert to km/h
    Serial.println(F(" km/h)"));
    
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
    
    Serial.println(F("--------------------"));
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
  Serial.print("KX134 Data\n");
  Serial.print(" X: ");
  Serial.print(kx134AccelData.xData, 4);
  Serial.print(" Y: ");
  Serial.print(kx134AccelData.yData, 4);
  Serial.print(" Z: ");
  Serial.print(kx134AccelData.zData, 4);
  Serial.println();
}
void ICM_20948_init(){
    // Sparkfun 9DOF sensor setup then wait for use input
  Serial.println(F("ICM-20948 Example"));
  // myICM.enableDebugging(); // Comment this line to disable debug messages on Serial
  myICM.begin(Wire, AD0_VAL);

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

  bool success = true; // Use success to show if the DMP configuration was successful
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);


  // Enable the DMP 32-bit 9-axis quaternion + heading accuracy
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Enable additional sensors / features
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 225Hz ODR rate when DMP is running at 225Hz, value = (225/225) - 1 = 0.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to 225Hz
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to 225Hz
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to 225Hz
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to 225Hz

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
    {
      Serial.println(F("DMP enabled!"));
    }
    else
    {
      Serial.println(F("Enable DMP failed!"));
      Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
      // This has been completed / uncommented
      while (1)
        ; // Do nothing more
    }
  }
}
void ICM_20948_read(){
  // Read data from the ICM20948
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    // Store the data header so we know what data is available
    icm_data_header = data.header;
    icm_data_available = true;

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // Check for GRV data (Quat6)
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      // Scale to +/- 1
      icm_q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      icm_q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      icm_q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
    }

    if ((data.header & DMP_header_bitmap_Accel) > 0) // Check for Accel
    {
      icm_accel_x = (float)data.Raw_Accel.Data.X; // Extract the raw accelerometer data
      icm_accel_y = (float)data.Raw_Accel.Data.Y;
      icm_accel_z = (float)data.Raw_Accel.Data.Z;
    }

    if ((data.header & DMP_header_bitmap_Gyro) > 0) // Check for Gyro
    {
      icm_gyro_x = (float)data.Raw_Gyro.Data.X; // Extract the raw gyro data
      icm_gyro_y = (float)data.Raw_Gyro.Data.Y;
      icm_gyro_z = (float)data.Raw_Gyro.Data.Z;
    }
    
    if ((data.header & DMP_header_bitmap_Compass) > 0) // Check for Compass
    {
      icm_mag_x = (float)data.Compass.Data.X; // Extract the compass data
      icm_mag_y = (float)data.Compass.Data.Y;
      icm_mag_z = (float)data.Compass.Data.Z;
    }
  } else {
    icm_data_available = false;
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(1); // Keep this short!
  }
}
int ms5611_read(){
  start = micros();
  int result = MS5611.read();
  stop = micros();
  
  // Don't print debug messages here - they'll be handled in ms5611_print function
  // Just return the result so the caller can check if needed
  return result;
}
void ms5611_init(){
  Serial.println(__FILE__);
  Serial.print("MS5611_LIB_VERSION: ");
  Serial.println(MS5611_LIB_VERSION);

  if (MS5611.begin() == true)
  {
    Serial.print("MS5611 found: ");
    Serial.println(MS5611.getAddress());
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
  MS5611.setOversampling(OSR_HIGH);
  // MS5611.setOversampling(OSR_ULTRA_HIGH);
  int result = ms5611_read();
  if (result != MS5611_READ_OK) {
    Serial.print("MS5611 read error during init: ");
    Serial.println(result);
  } else {
    Serial.println("MS5611 successfully initialized and read");
  }
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
  byte rate = myGPS.getNavigationFrequency();
  
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
  
  // Check if the card is present and can be initialized
  if (!SD.begin(SDchipSelect)) {
    Serial.println(F("SD Card initialization failed!"));
    return false;
  }
  
  Serial.println(F("SD Card initialized."));
  return true;
}

// Function to initialize the Serial Flash memory
bool initSerialFlash() {
  Serial.println(F("Initializing Serial Flash..."));
  
  // Check if the flash memory is present
  if (!SerialFlash.begin(FLASH_CHIP_SELECT)) {
    Serial.println(F("Serial Flash initialization failed!"));
    return false;
  }
  
  Serial.println(F("Serial Flash initialized."));
  return true;
}

// Function to print MS5611 data to serial
void ms5611_print() {
  Serial.print("MS5611 Data\n");
  Serial.print("T: ");
  Serial.print(MS5611.getTemperature(), 2);
  Serial.print(" P: ");
  Serial.print(MS5611.getPressure(), 2);
  Serial.println();
}

// Function to print ICM_20948 data to serial
void ICM_20948_print() {
  Serial.println(F("\n----- IMU Data -----"));
  
  if (!icm_data_available) {
    Serial.println(F("No IMU data available"));
    return;
  }
  
  Serial.print(F("Data Header: 0x"));
  if (icm_data_header < 0x1000) Serial.print("0");
  if (icm_data_header < 0x100) Serial.print("0");
  if (icm_data_header < 0x10) Serial.print("0");
  Serial.println(icm_data_header, HEX);
  
  if ((icm_data_header & DMP_header_bitmap_Quat6) > 0) {
    Serial.println(F("Quaternion:"));
    Serial.print(F("  Q1: ")); Serial.print(icm_q1, 3);
    Serial.print(F("  Q2: ")); Serial.print(icm_q2, 3);
    Serial.print(F("  Q3: ")); Serial.println(icm_q3, 3);
  }
  
  if ((icm_data_header & DMP_header_bitmap_Accel) > 0) {
    Serial.println(F("Accelerometer (raw):"));
    Serial.print(F("  X: ")); Serial.print(icm_accel_x);
    Serial.print(F("  Y: ")); Serial.print(icm_accel_y);
    Serial.print(F("  Z: ")); Serial.println(icm_accel_z);
  }
  
  if ((icm_data_header & DMP_header_bitmap_Gyro) > 0) {
    Serial.println(F("Gyroscope (raw):"));
    Serial.print(F("  X: ")); Serial.print(icm_gyro_x);
    Serial.print(F("  Y: ")); Serial.print(icm_gyro_y);
    Serial.print(F("  Z: ")); Serial.println(icm_gyro_z);
  }
  
  if ((icm_data_header & DMP_header_bitmap_Compass) > 0) {
    Serial.println(F("Magnetometer (raw):"));
    Serial.print(F("  X: ")); Serial.print(icm_mag_x);
    Serial.print(F("  Y: ")); Serial.print(icm_mag_y);
    Serial.print(F("  Z: ")); Serial.println(icm_mag_z);
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

// Function to dump internal flash data to serial
void dumpInternalFlashData() {
  Serial.println(F("\n===== INTERNAL FLASH DATA DUMP ====="));
  
  // Skip if LittleFS is not initialized
  if (!flashFS.begin(INTERNAL_FLASH_SIZE)) {
    Serial.println(F("Failed to initialize LittleFS!"));
    return;
  }
  
  // List all log files
  File root = flashFS.open("/");
  if (!root || !root.isDirectory()) {
    Serial.println(F("Failed to open root directory!"));
    return;
  }
  
  // Show available log files
  int fileIndex = 0;
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory() && strstr(file.name(), ".bin") != NULL) {
      Serial.print(fileIndex);
      Serial.print(F(": "));
      Serial.print(file.name());
      Serial.print(F(" ("));
      Serial.print(file.size());
      Serial.println(F(" bytes)"));
      fileIndex++;
    }
    file = root.openNextFile();
  }
  
  // If no files found
  if (fileIndex == 0) {
    Serial.println(F("No log files found!"));
    root.close();
    return;
  }
  
  // Ask which file to dump
  Serial.println(F("Enter file number to dump:"));
  unsigned long startTime = millis();
  String input = "";
  
  while (millis() - startTime < 10000) { // 10 second timeout
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        break;
      }
      input += c;
    }
  }
  
  int selectedFile = input.toInt();
  if (selectedFile < 0 || selectedFile >= fileIndex) {
    Serial.println(F("Invalid file number!"));
    root.close();
    return;
  }
  
  // Reopen the root and find the selected file
  root.close();
  root = flashFS.open("/");
  fileIndex = 0;
  file = root.openNextFile();
  
  File logFile;
  while (file) {
    if (!file.isDirectory() && strstr(file.name(), ".bin") != NULL) {
      if (fileIndex == selectedFile) {
        logFile = file;
        break;
      }
      fileIndex++;
    }
    file = root.openNextFile();
  }
  
  if (!logFile) {
    Serial.println(F("Failed to open selected file!"));
    root.close();
    return;
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
  Serial.println(F("Timestamp,FixType,Sats,Lat,Long,Alt,Speed,Pressure,Temp,AccelX,AccelY,AccelZ,Checksum"));
  
  // Dump each 32-byte record
  uint8_t recordBytes[32];
  int recordCount = 0;
  
  while (logFile.available() >= 32) {
    logFile.read(recordBytes, 32);
    
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
    
    // Accelerometer values (6 bytes) - stored as g * 1000
    int16_t accelX = 0, accelY = 0, accelZ = 0;
    accelX |= (int16_t)recordBytes[24];
    accelX |= (int16_t)recordBytes[25] << 8;
    accelY |= (int16_t)recordBytes[26];
    accelY |= (int16_t)recordBytes[27] << 8;
    accelZ |= (int16_t)recordBytes[28];
    accelZ |= (int16_t)recordBytes[29] << 8;
    
    // Checksum (2 bytes)
    uint16_t checksum = 0;
    checksum |= (uint16_t)recordBytes[30];
    checksum |= (uint16_t)recordBytes[31] << 8;
    
    // Calculate and verify checksum
    uint16_t calculatedChecksum = 0;
    for (int i = 0; i < 30; i++) {
      calculatedChecksum += recordBytes[i];
    }
    
    // Format and print the record
    char buffer[150];
    sprintf(buffer, "%lu,%d,%d,%ld,%ld,%ld,%d,%.1f,%.2f,%.3f,%.3f,%.3f,%u%s",
            timestamp,
            fixType,
            satCount,
            latitude,
            longitude,
            altitude,
            speed,
            pressure / 10.0,    // Convert back to hPa
            temperature / 100.0, // Convert back to °C
            accelX / 1000.0,    // Convert back to g
            accelY / 1000.0,
            accelZ / 1000.0,
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
}

// Function to handle the flash data check prompt
void handleFlashDataCheck() {
  if (checkInternalFlashData()) {
    Serial.println(F("Would you like to dump the data? (y/n)"));
    
    // Wait for response with timeout
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) { // 10 second timeout
      if (Serial.available()) {
        char response = Serial.read();
        if (response == 'y' || response == 'Y') {
          dumpInternalFlashData();
          
          // Ask if user wants to erase the data
          Serial.println(F("Would you like to erase all log files? (y/n)"));
          startTime = millis();
          while (millis() - startTime < 10000) { // 10 second timeout
            if (Serial.available()) {
              response = Serial.read();
              if (response == 'y' || response == 'Y') {
                // Format the filesystem
                Serial.println(F("Erasing all log files..."));
                flashFS.format();
                Serial.println(F("All log files erased."));
                break;
              } else if (response == 'n' || response == 'N') {
                Serial.println(F("Log files preserved."));
                break;
              }
            }
          }
          break;
        } else if (response == 'n' || response == 'N') {
          Serial.println(F("Data dump skipped."));
          break;
        }
      }
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
  Serial.println(F("GPS: "));
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
  
  if (GPS_fixType >= 2) {
    Serial.print(F("  Position: "));
    Serial.print(GPS_latitude / 10000000.0, 6);
    Serial.print(F(", "));
    Serial.print(GPS_longitude / 10000000.0, 6);
    Serial.print(F(" | Alt: "));
    Serial.print(GPS_altitude / 1000.0, 1);
    Serial.println(F("m"));
    
    Serial.print(F("  Speed: "));
    Serial.print(GPS_speed * 0.0036, 1);
    Serial.print(F(" km/h | RTK: "));
    Serial.println(RTK == 2 ? F("Fixed") : (RTK == 1 ? F("Float") : F("None")));
  }
  
  // SENSORS - Combine atmospheric and IMU data in a compact format
  Serial.println(F("SENSORS:"));
  Serial.print(F("  Baro: "));
  Serial.print(MS5611.getPressure(), 1);
  Serial.print(F(" hPa, "));
  Serial.print(MS5611.getTemperature(), 1);
  Serial.println(F("°C"));
  
  Serial.print(F("  Accel: X:"));
  Serial.print(kx134_x, 2);
  Serial.print(F("g Y:"));
  Serial.print(kx134_y, 2);
  Serial.print(F("g Z:"));
  Serial.print(kx134_z, 2);
  Serial.println(F("g"));
  
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

// Updated IMU data display function
void printDetailedIMUData() {
  Serial.println(F("\n----- DETAILED IMU DATA -----"));
  
  if (!icm_data_available) {
    Serial.println(F("No IMU data available"));
    return;
  }
  
  // Quaternion
  if ((icm_data_header & DMP_header_bitmap_Quat6) > 0) {
    Serial.print(F("Quat: "));
    Serial.print(F("Q1="));
    Serial.print(icm_q1, 3);
    Serial.print(F(" Q2="));
    Serial.print(icm_q2, 3);
    Serial.print(F(" Q3="));
    Serial.println(icm_q3, 3);
  }
  
  // Accelerometer and Gyroscope on same line
  Serial.print(F("Accel: "));
  if ((icm_data_header & DMP_header_bitmap_Accel) > 0) {
    Serial.print(F("X="));
    Serial.print(icm_accel_x);
    Serial.print(F(" Y="));
    Serial.print(icm_accel_y);
    Serial.print(F(" Z="));
    Serial.print(icm_accel_z);
  } else {
    Serial.print(F("N/A"));
  }
  
  Serial.print(F(" | Gyro: "));
  if ((icm_data_header & DMP_header_bitmap_Gyro) > 0) {
    Serial.print(F("X="));
    Serial.print(icm_gyro_x);
    Serial.print(F(" Y="));
    Serial.print(icm_gyro_y);
    Serial.print(F(" Z="));
    Serial.println(icm_gyro_z);
  } else {
    Serial.println(F("N/A"));
  }
  
  // Magnetometer
  if ((icm_data_header & DMP_header_bitmap_Compass) > 0) {
    Serial.print(F("Mag: X="));
    Serial.print(icm_mag_x);
    Serial.print(F(" Y="));
    Serial.print(icm_mag_y);
    Serial.print(F(" Z="));
    Serial.println(icm_mag_z);
  }
  
  Serial.println(F("-------------------------------"));
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
  // Startup Tone
  delay(500);
  tone(BUZZER, 2000); delay(50); noTone(BUZZER); delay(75);
  noTone(BUZZER);

  // Wait for the Serial monitor to be opened.
  Serial.begin(115200);
  while (!Serial)
    delay(100);

  // Setup the pins FOR TEENSY to talk to the SD Card
  SPI.setCS(10);  // Teensy 4.0 has MOSI on pin 10
  SPI.setMOSI(11);  // Teensy 4.0 has MOSI on pin 11
  SPI.setMISO(12);  // Teensy 4.0 has MOSI on pin 12
  SPI.setSCK(13);  // Teensy 4.0 has SCK on pin 13
  
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);
  
  kxAccel.begin();

  Serial.print("TripleT Flight Firmware Alpha ");
  Serial.println(TRIPLET_FLIGHT_VERSION);
  Serial.println("There will be lots of debug output to serial that will need to be removed later");

  // Initialize LittleFS first to check for existing data
  Serial.println(F("Checking internal flash memory..."));
  if (!flashFS.begin(INTERNAL_FLASH_SIZE)) {
    Serial.println(F("Failed to initialize LittleFS!"));
    internalFlashAvailable = false;
  } else {
    internalFlashAvailable = true;
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
  Serial.print(myGPS.getYear());
  Serial.print("-");
  Serial.print(myGPS.getMonth());
  Serial.print("-");
  Serial.print(myGPS.getDay());
  Serial.print(" ");
  Serial.print(myGPS.getHour());
  Serial.print(":");
  Serial.print(myGPS.getMinute());
  Serial.print(":");
  Serial.print(myGPS.getSecond());

  while (Serial.available()) // Make sure the serial RX buffer is empty
    Serial.read();
  Serial.println(F("Press any key to continue..."));
  while (!Serial.available()); // Wait for the user to press a key (send any serial character)
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
      printDetailedIMUData();
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
    printDetailedIMUData();
  }
}


