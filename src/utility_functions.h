#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <SerialFlash.h>
#include <Adafruit_NeoPixel.h>
#include <SparkFun_KX13X.h>
#include <MS5611.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// External sensor objects
extern SparkFun_KX134 kx134Accel;  // Create instance of the KX134 class
extern MS5611 ms5611Sensor;  // Create instance of the MS5611 class

// Add forward declaration for LittleFS_Program 
class LittleFS_Program;

// External variables
extern SdFat SD;
extern Adafruit_NeoPixel pixels;
extern bool sdCardAvailable;
extern bool sdCardPresent;
extern bool sdCardMounted;
extern bool loggingEnabled;
extern uint64_t availableSpace;
extern bool flashAvailable;
extern String FileDateString;

// External logging variables
extern unsigned long currentTime;
extern bool baroCalibrated;
extern const char* BOARD_NAME;

// External sensor data
extern SFE_UBLOX_GNSS myGNSS;
extern float pressure;
extern float temperature;
extern float kx134_accel[3];
extern float icm_accel[3];
extern float icm_gyro[3];
extern float icm_mag[3];
extern float icm_temp;  // Add ICM temperature variable
extern bool icm_data_available;
extern float icm_q0, icm_q1, icm_q2, icm_q3;
extern uint16_t icm_data_header;

// External board configuration
extern const char* BOARD_NAME;

// Function declarations
bool initSDCard();
void initNeoPixel();
void scan_i2c();
bool createNewLogFile();
void WriteLogData(bool forceLog = false);
void formatNumber(float input, byte columns, byte places);
void printStatusSummary();
void printHelpMessage();
void printStorageStatistics();
void checkStorageSpace();
void handleCommand(const String& command);
void listRootDirectory(); // Function to list files in the root directory

// Debug formatting functions
void printDebugHeader(const char* title);
void printDebugValue(const char* label, float value, int precision = 2);
void printDebugValueWithUnit(const char* label, float value, const char* unit, int precision = 2);
void printDebugPair(const char* label, float value1, float value2, int precision = 2);
void printDebugTriple(const char* label, float value1, float value2, float value3, int precision = 2);
void printDebugQuad(const char* label, float value1, float value2, float value3, float value4, int precision = 4);
void printDebugState(const char* label, const char* state);
void printDebugBoolean(const char* label, bool value);
void printDebugDivider();

// New function declaration
String getFileDateString();

#endif // UTILITY_FUNCTIONS_H 