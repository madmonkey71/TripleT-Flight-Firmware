#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <Arduino.h>
#include <stdint.h>

// Define the structure that matches our binary data format
typedef struct {
  uint32_t timestamp;      // 4 bytes
  uint8_t fixType;        // 1 byte
  uint8_t sats;           // 1 byte
  int32_t latitude;       // 4 bytes
  int32_t longitude;      // 4 bytes
  int32_t altitude;       // 4 bytes
  int32_t altitudeMSL;    // 4 bytes
  int32_t speed;          // 4 bytes
  int32_t heading;        // 4 bytes
  uint16_t pDOP;          // 2 bytes
  uint8_t rtk;            // 1 byte
  float pressure;         // 4 bytes
  float temperature;      // 4 bytes
  float kx134_x;         // 4 bytes
  float kx134_y;         // 4 bytes
  float kx134_z;         // 4 bytes
  float icm_accel[3];    // 12 bytes (x, y, z)
  float icm_gyro[3];     // 12 bytes (x, y, z)
  float icm_mag[3];      // 12 bytes (x, y, z)
} LogData;

#endif // DATA_STRUCTURES_H 