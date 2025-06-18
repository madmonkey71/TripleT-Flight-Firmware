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

// Include LogData definition before it's used in function declarations
#include "data_structures.h"

// SD Card Configuration is now centralized in config.h
// Ensure config.h is included if these definitions are needed, typically via TripleT_Flight_Firmware.cpp including it.

// Serial Flash Configuration
#ifndef FLASH_CS_PIN
#define FLASH_CS_PIN 6  // CS pin for Serial Flash
#endif

// NeoPixel Configuration
#define NEOPIXEL_PIN 2  // Pin for NeoPixel
#define NEOPIXEL_COUNT 2  // Number of NeoPixels

// External sensor objects - These should be included from their respective module headers
// extern SparkFun_KX134 kx134Accel;
// extern MS5611 ms5611Sensor;

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

// Kalman Filter Global Variables
extern bool useMadgwickFilter;
extern bool useKalmanFilter;
extern float kalmanRoll;
extern float kalmanPitch;
extern float kalmanYaw;
extern bool usingKX134ForKalman; // Flag to indicate if KX134 is being used for Kalman accel input

// External logging variables
extern unsigned long currentTime;
extern bool baroCalibrated;
extern const char* BOARD_NAME;
extern bool enableSystemDebug; // Flag for system debug messages

// External sensor data - These should be included from their respective module headers
// extern SFE_UBLOX_GNSS myGNSS;
// extern float pressure;
// extern float temperature;
// extern float kx134_accel[3];
// extern float icm_accel[3];
// extern float icm_gyro[3];
// extern float icm_mag[3];
// extern float icm_temp;
// extern bool icm_data_available;
// extern float icm_q0, icm_q1, icm_q2, icm_q3; // These are not in icm_20948_functions.h as extern
// extern uint16_t icm_data_header;

// External board configuration
// extern const char* BOARD_NAME; // This is defined in main .cpp and also extern here, it's fine.

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
float get_accel_magnitude(); // Calculate acceleration magnitude from available sensors
String logDataToString(const LogData& data); // Convert LogData struct to CSV string

// Helper function to map a float value from one range to another
float map_float(float x, float in_min, float in_max, float out_min, float out_max);

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
// by utility_functions.cpp (e.g. by including TripleT_Flight_Firmware.cpp or a shared header).
// Consider moving FlightState enum to data_structures.h or its own header for better organization.
enum FlightState : uint8_t; // Assuming FlightState is compatible with uint8_t as used in FlightStateData

// Sensor and system health check
bool isSensorSuiteHealthy(FlightState currentState, bool verbose = false);

// Get the string name of a flight state
const char* getStateName(FlightState state);

#endif // UTILITY_FUNCTIONS_H 