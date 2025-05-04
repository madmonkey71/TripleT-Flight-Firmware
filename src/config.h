#pragma once

// This file contains central configuration parameters for the TripleT Flight Firmware

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

// --- Master Feature Switches ---
#define ENABLE_WATCHDOG 1                 // Enable/Disable the hardware watchdog timer (1=Enabled, 0=Disabled)

// --- Features & Hardware Presence ---
// Configure parachute presence. At least MAIN must be present.
#define DROGUE_PRESENT 1 // 1 = Present, 0 = Not Present
#define MAIN_PRESENT 1   // 1 = Present, 0 = Not Present

// --- Automatically derive deployment type and check for errors ---
#if MAIN_PRESENT == 0
  // Invalid: Main MUST be present
  #error "Invalid Parachute Configuration: MAIN_PRESENT must be 1."
#elif DROGUE_PRESENT == 1 && MAIN_PRESENT == 1
  // Both Drogue and Main are present
  #define DUAL_DEPLOY 1
  #define SINGLE_DEPLOY 0
#elif DROGUE_PRESENT == 0 && MAIN_PRESENT == 1
  // Only Main is present
  #define DUAL_DEPLOY 0
  #define SINGLE_DEPLOY 1
#else // DROGUE_PRESENT == 1 && MAIN_PRESENT == 0 (Handled by first #if)
  // This case should technically not be reached due to the first #if, but included for completeness
  #error "Invalid Parachute Configuration: Logic error - Main must be present."
#endif

#define BUZZER_OUTPUT 1                   // Enable buzzer output (1=Enabled, 0=Disabled)
#define USE_KX134 1                       // Use KX134 high-g sensor alongside ICM20948 (1=Use, 0=Don't Use)

// --- Pin Definitions ---
#define FLASH_CS_PIN 6                           // CS pin for Serial Flash (if used)
#define NEOPIXEL_PIN 2                           // Pin for NeoPixel
#define BUZZER_PIN 23                            // Pin for buzzer

// --- NeoPixel Configuration ---
#define NEOPIXEL_COUNT 2                         // Number of NeoPixels

// --- Flight Logic Parameters ---
#define MAIN_DEPLOY_ALTITUDE 150.0               // Deploy main parachute at 100m AGL
#define BOOST_ACCEL_THRESHOLD 1.5                // Threshold to detect liftoff (g)
#define COAST_ACCEL_THRESHOLD 0.5                // Threshold to detect end of boost (g)
#define APOGEE_CONFIRMATION_COUNT 3              // Multiple readings required to confirm apogee
#define LANDING_CONFIRMATION_COUNT 10            // Multiple readings required to confirm landing

// --- Sensor Error & Timeout Thresholds ---
#define MAX_SENSOR_FAILURES 3
#define BAROMETER_ERROR_THRESHOLD 10.0           // 10 hPa change between readings is suspicious
#define ACCEL_ERROR_THRESHOLD 10.0               // 10g change between readings is suspicious
#define GPS_TIMEOUT_MS 5000                      // 5 seconds without GPS update is considered a failure

// --- Storage & Logging Configuration ---
// SD Card
#define SD_CARD_MIN_FREE_SPACE 5 * 1024 * 1024   // 50MB minimum free space
#define SD_CACHE_SIZE 8                          // Cache factor for SD operations
#define LOG_PREALLOC_SIZE 5000000                // Pre-allocate 5MB for log file
#define DISABLE_SDCARD_LOGGING true              // Disable SD card logging by default (only for use in testing)

// Logging Buffers (RAM)
#define MAX_LOG_ENTRIES 1                       // Max log entries to buffer in RAM
#define FLUSH_INTERVAL 100                      // Max time (ms) between buffer flushes to SD (if not full)
#define MAX_BUFFER_SIZE 300                     // Max size (bytes) of a single log entry string

// External Flash (if used)
#define EXTERNAL_FLASH_MIN_FREE_SPACE 1024 * 1024  // 1MB minimum free space

// --- EEPROM Configuration ---
#define EEPROM_STATE_ADDR 0                     // EEPROM address for flight state structure
#define EEPROM_SIGNATURE_VALUE 0xABCD           // Signature to validate EEPROM data

// --- SD Card Driver Configuration (Platform Specific) ---

// For Teensy 4.1, use the built-in SD card socket with SDIO mode and optimized settings
#ifndef SD_CONFIG
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#endif
#ifndef SD_BUF_SIZE                           // Buffer size for SdFat library operations
#define SD_BUF_SIZE 65535                     // 16KB buffer for SD card operations
#endif
#ifndef SD_DETECT_PIN // Card detect pin for built-in socket
#define SD_DETECT_PIN 39
#endif

/* For other boards (Teensy 4.0, etc.), use standard SPI with optimized settings
#ifndef SD_CS_PIN
#define SD_CS_PIN 10  // CS pin for SD card
#endif
#ifndef SD_CONFIG
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(50))
#endif
#ifndef SD_BUF_SIZE // Buffer size for SdFat library operations
#define SD_BUF_SIZE 8192  // 8KB buffer for SD card operations on non-Teensy 4.1 boards
#endif
#ifndef SD_DETECT_PIN // Card detect pin (if available)
#define SD_DETECT_PIN 9
#endif
*/
// SPI Pins (only relevant if using SPI mode)
// #ifndef SD_MOSI_PIN
// #define SD_MOSI_PIN 11
// #endif
// #ifndef SD_MISO_PIN
// #define SD_MISO_PIN 12
// #endif
// #ifndef SD_SCK_PIN
// #define SD_SCK_PIN 13
// #endif
