#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stddef.h>
#include "data_structures.h"

// Telemetry binary protocol — Teensy -> ESP32 (UART) -> ESP-NOW -> ground.
// See wiki/entities/esp32-telemetry.md for the link topology and the
// rationale for the field choices.
//
// Wire framing (Teensy -> ESP32 over Serial5):
//   [SYNC=0xA5][LEN=sizeof(TelemetryPacket)][... payload ...][CRC8]
//   CRC-8 (poly 0x07, init 0x00) over the payload only.
//
// ESP-NOW payload is the same TelemetryPacket struct, no framing
// (ESP-NOW delivers complete packets atomically up to 250 bytes).

#define TELEMETRY_SYNC_BYTE 0xA5

// All multi-byte integers are little-endian (Teensy and ESP32 are both LE,
// so the struct can be memcpy'd as-is). Floats are not used in the wire
// format — everything is scaled to fixed-point int16/int32 for size and
// determinism.
#pragma pack(push, 1)
typedef struct {
    uint32_t timestamp_ms;     // ms since boot (matches LogData.timestamp)
    uint8_t  flight_state;     // FlightState enum (matches LogData.flightState)
    uint8_t  error_code;       // last ErrorCode_t (matches LogData.last_error_code)
    int32_t  latitude_e7;      // degrees * 1e7
    int32_t  longitude_e7;     // degrees * 1e7
    int32_t  altitude_gps_mm;  // GPS altitude MSL in mm (matches LogData.altitudeMSL)
    int32_t  altitude_baro_mm; // Barometer altitude in mm (LogData.calibrated_altitude * 1000)
    int16_t  accel_x_mg;       // ICM accel, milli-g (g * 1000)
    int16_t  accel_y_mg;
    int16_t  accel_z_mg;
    int16_t  q0_q14;           // Quaternion components, fixed-point with 14 fractional bits
    int16_t  q1_q14;           //   (q * 16384, range ±2 covers any unit quaternion)
    int16_t  q2_q14;
    int16_t  q3_q14;
    uint16_t battery_mv;       // Battery voltage in mV
    uint8_t  stability_flags;  // Bitfield from LogData.stability_flags
    uint8_t  guidance_active;  // 1 if guidance is actively commanding, 0 if disabled
} TelemetryPacket;
#pragma pack(pop)

// Compile-time size guarantee. If the LogData layout changes and the
// scaling fits in a different field width, bump this and recheck.
//   4+1+1+4+4+4+4 (head) + 3*2 (accel) + 4*2 (quat) + 2+1+1 (tail) = 40
#define TELEMETRY_PACKET_SIZE 40
static_assert(sizeof(TelemetryPacket) == TELEMETRY_PACKET_SIZE,
              "TelemetryPacket must be exactly 40 bytes — packed wire format");

// Scaling constants (kept here so packer and unpacker share them).
#define TELEMETRY_ACCEL_SCALE   1000.0f   // m/g  -> milli-g
#define TELEMETRY_QUAT_SCALE    16384.0f  // q14 fixed-point
#define TELEMETRY_BATTERY_SCALE 1000.0f   // V -> mV
#define TELEMETRY_BARO_SCALE    1000.0f   // m -> mm

// Pack a LogData snapshot into a TelemetryPacket. Pure function, no I/O —
// trivially unit-testable on native.
void telemetry_pack(const LogData &src, TelemetryPacket &out);

// CRC-8 over a buffer. Polynomial 0x07, init 0x00 (CRC-8/SMBUS).
uint8_t telemetry_crc8(const uint8_t *data, size_t len);

// Format a framed packet ready to write to UART:
//   buf must be at least (2 + TELEMETRY_PACKET_SIZE + 1) bytes.
// Returns the number of bytes written.
size_t telemetry_frame(const TelemetryPacket &pkt, uint8_t *buf);

#define TELEMETRY_FRAME_SIZE (2 + TELEMETRY_PACKET_SIZE + 1)

#endif // TELEMETRY_H
