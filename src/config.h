#pragma once

// This file contains central configuration parameters for the TripleT Flight Firmware

// Define the board type - Teensy 4.1 only
#ifndef BOARD_TEENSY41
#if defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41)
#define BOARD_TEENSY41
#endif
#endif

// --- Features & Hardware Presence ---
// Configure parachute presence. At least MAIN must be present.
#define DROGUE_PRESENT true // Set to true if drogue deployment is needed
#define MAIN_PRESENT true   // Set to true if main deployment is needed
#define PYRO_CHANNEL_1 2           // GPIO pin for drogue deployment
#define PYRO_CHANNEL_2 3           // GPIO pin for main deployment

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
#define BUZZER_PIN 9                             // Pin for buzzer

// --- NeoPixel Configuration ---
#define NEOPIXEL_COUNT 2                         // Number of NeoPixels

// --- Flight Logic Parameters ---
// #define MAIN_DEPLOY_ALTITUDE 300   // Deploy main at this height (meters) above ground - Now dynamic, see g_main_deploy_altitude_m_agl
#define MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M 100.0f // Main parachute deployment height above ground level (meters)
#define BOOST_ACCEL_THRESHOLD 2.0f  // Acceleration threshold (in g) to detect liftoff.
#define COAST_ACCEL_THRESHOLD 0.5f  // Acceleration threshold (in g) to detect the end of the boost phase (motor burnout).
#define APOGEE_CONFIRMATION_COUNT 5   // Number of consecutive barometer readings required to confirm apogee.
#define LANDING_CONFIRMATION_COUNT 10 // Number of consecutive readings required to confirm landing.
#define BACKUP_APOGEE_TIME_MS 20000     // Failsafe time in ms after motor burnout to trigger apogee.
#define APOGEE_ACCEL_CONFIRMATION_COUNT 5 // Number of consecutive readings of negative Z-axis acceleration to confirm apogee.
#define APOGEE_GPS_CONFIRMATION_COUNT 3   // Number of consecutive GPS altitude readings showing descent to confirm apogee.

// Redundant Sensing Apogee Detection
#ifndef APOGEE_BARO_DESCENT_THRESHOLD
#define APOGEE_BARO_DESCENT_THRESHOLD 1.0 // Meters change to confirm descent for apogee
#endif
#ifndef APOGEE_ACCEL_THRESHOLD
#define APOGEE_ACCEL_THRESHOLD -0.1     // G value for Z-axis accelerometer apogee detection
#endif
#ifndef APOGEE_ACCEL_SAMPLES
#define APOGEE_ACCEL_SAMPLES 5          // Consecutive samples for accelerometer apogee detection
#endif

// Redundant Sensing Landing Detection
#ifndef LANDING_ACCEL_MIN_G
#define LANDING_ACCEL_MIN_G 0.9f    // Minimum acceleration for landing detection (g)
#endif
#ifndef LANDING_ACCEL_MAX_G
#define LANDING_ACCEL_MAX_G 1.1f    // Maximum acceleration for landing detection (g)
#endif
#ifndef LANDING_CONFIRMATION_TIME_MS
#define LANDING_CONFIRMATION_TIME_MS 2000 // Time in ms of stable conditions to confirm landing
#endif
#ifndef LANDING_ALTITUDE_STABLE_THRESHOLD
#define LANDING_ALTITUDE_STABLE_THRESHOLD 1.0 // Meters altitude change for landing stability
#endif

// --- State Machine Timeouts & Durations ---
#ifndef PYRO_FIRE_DURATION
#define PYRO_FIRE_DURATION 1000 // Milliseconds for pyro channel to be active
#endif
#ifndef LANDED_TIMEOUT_MS
#define LANDED_TIMEOUT_MS 10000 // Milliseconds to stay in LANDED state before RECOVERY
#endif
#ifndef RECOVERY_TIMEOUT_MS
#define RECOVERY_TIMEOUT_MS 300000 // Milliseconds in RECOVERY before auto-shutdown (5 mins)
#endif
#ifndef ERROR_RECOVERY_ATTEMPT_MS
#define ERROR_RECOVERY_ATTEMPT_MS 10000 // Milliseconds in ERROR state before attempting recovery
#endif

// --- Sensor Error & Timeout Thresholds ---
#define MAX_SENSOR_FAILURES 3      // Maximum number of consecutive sensor failures before error state
#define WATCHDOG_TIMEOUT_MS 1000   // Watchdog timer timeout in milliseconds
#define BAROMETER_ERROR_THRESHOLD 10.0  // Barometer error threshold (m) between readings
#define ACCEL_ERROR_THRESHOLD 10.0      // Accelerometer error threshold (g) between readings
#define GPS_TIMEOUT_MS 5000             // GPS timeout in milliseconds

// --- Storage & Logging Configuration ---
// SD Card
#define SD_CARD_MIN_FREE_SPACE 5 * 1024 * 1024   // 50MB minimum free space
#define SD_CACHE_SIZE 8                          // Cache factor for SD operations
#define LOG_PREALLOC_SIZE 5000000                // Pre-allocate 5MB for log file
#define DISABLE_SDCARD_LOGGING false             // Disable SD card logging by default (only for use in testing)

// Logging Buffers (RAM)
#define MAX_LOG_ENTRIES 1                       // Max log entries to buffer in RAM
#define MAX_BUFFER_SIZE 300                     // Max size (bytes) of a single log entry string

// External Flash (if used)
#define EXTERNAL_FLASH_MIN_FREE_SPACE 1024 * 1024  // 1MB minimum free space

// --- EEPROM Configuration ---
#define EEPROM_STATE_ADDR 0                // EEPROM address for flight state (now holds the entire FlightStateData struct)
// #define EEPROM_ALTITUDE_ADDR 4             // REMOVED - Part of FlightStateData struct
// #define EEPROM_TIMESTAMP_ADDR 8            // REMOVED - Part of FlightStateData struct
// #define EEPROM_SIGNATURE_ADDR 12           // REMOVED - Part of FlightStateData struct
#define EEPROM_SIGNATURE_VALUE 0xBEEF      // Signature to validate EEPROM data (16-bit), used within FlightStateData

// --- Madgwick Filter Configuration --- (REMOVED: Madgwick filter no longer supported)

// --- Orientation System Configuration ---
// Determines which orientation system is active at startup.
// Kalman filter is now the only option.
#define KALMAN_FILTER_ACTIVE_BY_DEFAULT true


// --- PID Controller Gains ---

// Roll Axis PID
#define PID_ROLL_KP 1.0f
#define PID_ROLL_KI 0.1f
#define PID_ROLL_KD 0.05f

// Pitch Axis PID
#define PID_PITCH_KP 1.0f
#define PID_PITCH_KI 0.1f
#define PID_PITCH_KD 0.05f

// Yaw Axis PID (e.g., for reaction wheel or differential thrust)
#define PID_YAW_KP 0.8f
#define PID_YAW_KI 0.08f
#define PID_YAW_KD 0.03f

// PID Output Limits (example)
#define PID_OUTPUT_MIN -1.0f // Min actuator command
#define PID_OUTPUT_MAX  1.0f // Max actuator command
#define PID_INTEGRAL_LIMIT_ROLL 0.5f // Anti-windup for roll
#define PID_INTEGRAL_LIMIT_PITCH 0.5f // Anti-windup for pitch
#define PID_INTEGRAL_LIMIT_YAW 0.3f  // Anti-windup for yaw

// --- Actuator Configuration ---
#define ACTUATOR_PITCH_PIN 21 // Teensy pin 21
#define ACTUATOR_ROLL_PIN  23 // Teensy pin 23. Not in hardware design yet
#define ACTUATOR_YAW_PIN   20 // Teensy pin 20
#define SERVO_MIN_PULSE_WIDTH 1000 // Microseconds (adjust as needed)
#define SERVO_MAX_PULSE_WIDTH 2000 // Microseconds (adjust as needed)
#define SERVO_DEFAULT_ANGLE 90     // Default angle for servos (degrees)

// --- SD Card Driver Configuration (Teensy 4.1 Only) ---

// For Teensy 4.1, use the built-in SD card socket with SDIO mode and optimized settings
#ifndef SD_CONFIG
  #define SD_CONFIG SdioConfig(FIFO_SDIO)
#endif
#ifndef SD_BUF_SIZE                           // Buffer size for SdFat library operations
  #define SD_BUF_SIZE 65535                     // 16KB buffer for SD card operations
#endif
// Note: Teensy 4.1 built-in SDIO SD card slot does not use a card detect pin

// State Management Configuration
// EEPROM_UPDATE_INTERVAL is now defined in constants.h

// Sensor Configuration

// --- Magnetometer Calibration Persistence ---
#define MAG_CAL_EEPROM_ADDR 100 // Starting address in EEPROM for mag calibration
#define MAG_CAL_MAGIC_NUMBER 0xBAADF00D // Magic number to validate stored data

// --- Flight State Machine Configuration ---
#define ARMED_TIMEOUT_MS 300000UL // 5 minutes in ARMED state before disarming

#define APOGEE_DELAY 1000           // Milliseconds to wait after apogee before deploying parachute

// --- Sensor Selections and Configurations ---

#define USE_KX134_FOR_LAUNCH_DETECTION  // Use KX134 for launch detection, otherwise use ICM20948

#ifdef USE_KX134_FOR_LAUNCH_DETECTION

#endif

// --- Servo Configuration ---
#define NUM_SERVOS 4          // Total number of servos
