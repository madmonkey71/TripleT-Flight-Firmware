#ifndef UTILITY_FUNCTIONS_H
#define UTILITY_FUNCTIONS_H

#include <Arduino.h> // For Serial, basic types, math functions (indirectly)
#include <Wire.h>    // For scan_i2c
#include <Adafruit_NeoPixel.h> // For initNeoPixel parameter type

// Include LogData definition before it's used in function declarations
#include "data_structures.h" // For LogData, FlightState (if not forward declared)

// SD Card Configuration is now centralized in config.h
// Serial Flash Configuration - Kept if any low-level utilities might use it, otherwise can be removed if specific to a module.
#ifndef FLASH_CS_PIN
#define FLASH_CS_PIN 6  // CS pin for Serial Flash
#endif

// NeoPixel Configuration - Kept if initNeoPixel or other utilities use these defines.
#define NEOPIXEL_PIN 2  // Pin for NeoPixel
#define NEOPIXEL_COUNT 2  // Number of NeoPixels

// External variable declarations REMOVED.
// Forward declarations for sensor object types are no longer needed here as no function signature uses them directly.
// Forward declaration for LittleFS_Program REMOVED (assuming unused).

// Function declarations
// bool initSDCard(); // Moved to command_processor.h as extern, defined in main
void initNeoPixel(Adafruit_NeoPixel& pixels_obj); // Updated signature
void scan_i2c();     // Definition in utility_functions.cpp
// bool createNewLogFile(); // Moved to command_processor.h as extern, defined in main
void WriteLogData(bool forceLog = false); // Defined in main
// void formatNumber(float input, byte columns, byte places); // Removed
// void printStatusSummary(); // Defined in main
// void printHelpMessage(); // Defined in command_processor.cpp
// void printStorageStatistics(); // Defined in command_processor.cpp
// void checkStorageSpace(); // Moved to command_processor.h as extern, defined in main
// void handleCommand(const String& command); // Superseded by processCommand

// void listRootDirectory(); // REMOVED as unused
float get_accel_magnitude(bool kx134_ok, const float* kx_accel, bool icm_ready, const float* icm_accel_data, bool system_debug_enabled); // Restored
String logDataToString(const LogData& data); // Definition in utility_functions.cpp

// Helper function to map a float value from one range to another - REMOVED as unused
// float map_float(float x, float in_min, float in_max, float out_min, float out_max);

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

// Function for saving flight state to EEPROM (defined elsewhere, e.g. main .cpp)
void saveStateToEEPROM();

// Attitude utility functions
void convertQuaternionToEuler(float q0, float q1, float q2, float q3, float& roll, float& pitch, float& yaw);
void convertEulerToQuaternion(float roll, float pitch, float yaw, float& q0, float& q1, float& q2, float& q3);

// Forward declaration for FlightState enum.
// This is needed because the function signature uses FlightState.
// The actual definition of FlightState is expected to be in a place accessible
// by utility_functions.cpp (e.g. by including data_structures.h).
// enum FlightState : uint8_t; // REMOVED as FlightState should be defined in data_structures.h

// Sensor and system health check
bool isSensorSuiteHealthy(FlightState currentState, bool verbose = false);

// Get the string name of a flight state
const char* getStateName(FlightState state);

// Battery Voltage Reading
float read_battery_voltage();

// Error Code to String Conversion
#include "error_codes.h" // Include ErrorCode_t definition
const char* getErrorCodeName(ErrorCode_t code);

#endif // UTILITY_FUNCTIONS_H 