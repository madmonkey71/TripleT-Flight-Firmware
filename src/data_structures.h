#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <Arduino.h>
#include <stdint.h>

// Flight State Machine Enum
enum FlightState : uint8_t {
  STARTUP,        // Initial state during power-on
  CALIBRATION,    // Sensor calibration state
  PAD_IDLE,       // On pad waiting for arm command
  ARMED,          // Armed and ready for launch
  BOOST,          // Motor burning, accelerating
  COAST,          // Unpowered flight upward
  APOGEE,         // Peak altitude reached
  DROGUE_DEPLOY,  // Deploying drogue parachute
  DROGUE_DESCENT, // Descending under drogue
  MAIN_DEPLOY,    // Deploying main parachute
  MAIN_DESCENT,   // Descending under main
  LANDED,         // On ground after flight
  RECOVERY,       // Post-flight data collection
  ERROR           // Error condition
};

// Define the structure that matches our binary data format
// NOTE: This struct is the primary definition for logged data.
//       WriteLogData() string construction and createNewLogFile() header
//       MUST be kept manually synchronized with this definition.
typedef struct {
  uint32_t seqNum;         // Log sequence number
  uint32_t timestamp;      // Milliseconds since boot
  uint8_t flightState;    // Current flight state (as uint8_t)
  uint8_t fixType;        // GPS fix type
  uint8_t sats;           // Number of satellites in view (from global SIV)
  int32_t latitude;       // Degrees * 1e7
  int32_t longitude;      // Degrees * 1e7
  int32_t altitude;       // Altitude above ellipsoid (mm)
  int32_t altitudeMSL;    // Altitude above mean sea level (mm)
  float raw_altitude;     // Raw altitude from barometer (m)
  float calibrated_altitude; // Calibrated altitude from barometer (m)
  int32_t speed;          // Ground speed (mm/s)
  int32_t heading;        // Heading of motion (deg * 1e5)
  uint16_t pDOP;          // Position Dilution of Precision (unitless * 100)
  uint8_t rtk;            // RTK fix status
  float pressure;         // Barometric pressure (hPa)
  float temperature;      // Barometric temperature (°C)
  float kx134_accel[3];   // KX134 Accelerometer (X, Y, Z) (g)
  float icm_accel[3];     // ICM Accelerometer (X, Y, Z) (g)
  float icm_gyro[3];      // ICM Gyroscope (X, Y, Z) (rad/s)
  float icm_mag[3];       // ICM Magnetometer (X, Y, Z) (uT)
  float icm_temp;         // ICM Temperature (°C)

  // AHRS Data
  float q0, q1, q2, q3;              // Orientation quaternions (w,x,y,z)
  float euler_roll, euler_pitch, euler_yaw; // Calculated Euler angles (radians)
  float gyro_bias_x, gyro_bias_y, gyro_bias_z; // Estimated gyroscope biases (rad/s)

  // Guidance Control Data (subset for now)
  float target_euler_roll;  // Target Euler roll (radians)
  float target_euler_pitch; // Target Euler pitch (radians)
  float target_euler_yaw;   // Target Euler yaw (radians)
  float pid_integral_roll;  // PID integral for roll
  float pid_integral_pitch; // PID integral for pitch
  float pid_integral_yaw;   // PID integral for yaw
  float actuator_x, actuator_y, actuator_z; // Actuator output commands (-1.0 to 1.0)
} LogData;

#include <Arduino.h> // For FlightState enum if not already included, though it's defined in TripleT_Flight_Firmware.cpp

// Forward declaration of FlightState if its definition is not accessible here
// However, it's better to define FlightState in a shared header if used across multiple .cpp files.
// For now, assuming FlightState definition will be accessible during compilation of state_management.cpp
// If not, this might need adjustment (e.g. #include "TripleT_Flight_Firmware.h" if it existed and contained the enum)
// Or, move FlightState enum to data_structures.h if it makes more sense.
// For this task, we will assume FlightState from TripleT_Flight_Firmware.cpp is usable by state_management.cpp

// Flight state storage structure for EEPROM
struct FlightStateData {
  // FlightState state; // This will be an issue if FlightState is defined in .cpp
  // To resolve, we will use uint8_t to store state, and cast to/from FlightState in functions.
  uint8_t state;            // Current flight state (cast to/from FlightState)
  float launchAltitude;     // Stored launch altitude
  float maxAltitude;        // Maximum altitude reached
  float currentAltitude;    // Current altitude
  unsigned long timestamp;  // Timestamp of last save
  uint16_t signature;       // Validation signature
};

#endif // DATA_STRUCTURES_H 