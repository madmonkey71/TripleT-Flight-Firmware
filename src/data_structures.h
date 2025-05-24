#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <Arduino.h>
#include <stdint.h>

// Define the structure that matches our binary data format
// NOTE: This struct is the primary definition for logged data.
//       WriteLogData() string construction and createNewLogFile() header
//       MUST be kept manually synchronized with this definition.
typedef struct {
  uint32_t seqNum;         // Log sequence number
  uint32_t timestamp;      // Milliseconds since boot
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

#endif // DATA_STRUCTURES_H 