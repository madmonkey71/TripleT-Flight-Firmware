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

// SD Card Configuration
#ifndef SD_CONFIG
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50))
#endif
#ifndef SD_CS_PIN
#define SD_CS_PIN 10  // CS pin for SD card
#endif
#ifndef SD_DETECT_PIN
#define SD_DETECT_PIN 9  // Card detect pin (if available)
#endif
#ifndef SD_MOSI_PIN
#define SD_MOSI_PIN 11  // MOSI pin for SD card
#endif
#ifndef SD_MISO_PIN
#define SD_MISO_PIN 12  // MISO pin for SD card
#endif
#ifndef SD_SCK_PIN
#define SD_SCK_PIN 13  // SCK pin for SD card
#endif

// Serial Flash Configuration
#ifndef FLASH_CS_PIN
#define FLASH_CS_PIN 6  // CS pin for Serial Flash
#endif

// NeoPixel Configuration
#define NEOPIXEL_PIN 8  // Pin for NeoPixel
#define NEOPIXEL_COUNT 1  // Number of NeoPixels

// Buzzer Configuration
#ifndef BUZZER
#define BUZZER 23  // Pin for buzzer
#endif
#define BUZZER_PIN BUZZER  // Alias for backward compatibility

// External sensor objects
extern SparkFun_KX134 kx134Accel;  // Create instance of the KX134 class
extern MS5611 ms5611Sensor;  // Create instance of the MS5611 class

// Add forward declaration for LittleFS_Program 
class LittleFS_Program;

// External variables
extern SdFat SD;
extern FsVolume volume;
extern Adafruit_NeoPixel pixels;
extern bool sdCardAvailable;
extern bool flashAvailable;
extern String FileDateString;

// External logging variables
extern String LogDataString;
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
extern double icm_q1, icm_q2, icm_q3;
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
void initExternalFlash();
void checkStorageSpace();
void handleCommand(const String& command);

#endif // UTILITY_FUNCTIONS_H 