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
#include <LittleFS.h>

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
#ifndef INTERNAL_FLASH_SIZE
#define INTERNAL_FLASH_SIZE 1024 * 1024  // 1MB internal flash
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
extern LittleFS_Program flashFS;  // Program flash filesystem

// External variables
extern SdFat SD;
extern FsVolume volume;
extern Adafruit_NeoPixel pixels;
extern bool sdCardAvailable;

// External logging variables
extern String FileDateString;
extern String LogDataString;
extern unsigned long currentTime;
extern bool baroCalibrated;

// External board configuration
extern const char* BOARD_NAME;

// Function declarations
bool initSDCard();
bool initSerialFlash();
void initNeoPixel();
void scan_i2c();
void handleFlashDataCheck();

#endif // UTILITY_FUNCTIONS_H 