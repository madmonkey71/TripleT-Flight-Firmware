#ifndef TELEMETRY_PACKET_H
#define TELEMETRY_PACKET_H

// Wire-format definitions for the TripleT telemetry link.
//
// **MUST match `src/telemetry.h` in the parent project.** If you change one
// you must change the other — the static_assert on packet size will catch
// most accidents, but field-order changes will silently corrupt downstream
// data. Keep both files in sync by hand.
//
// Link topology:
//   Teensy 4.1 (Serial5, framed) -> this ESP32 (UART2) -> ESP-NOW (payload only)

#include <stdint.h>
#include <stddef.h>

#define TELEMETRY_SYNC_BYTE 0xA5
#define TELEMETRY_PACKET_SIZE 40
#define TELEMETRY_FRAME_SIZE (2 + TELEMETRY_PACKET_SIZE + 1)

#pragma pack(push, 1)
typedef struct {
    uint32_t timestamp_ms;
    uint8_t  flight_state;
    uint8_t  error_code;
    int32_t  latitude_e7;
    int32_t  longitude_e7;
    int32_t  altitude_gps_mm;
    int32_t  altitude_baro_mm;
    int16_t  accel_x_mg;
    int16_t  accel_y_mg;
    int16_t  accel_z_mg;
    int16_t  q0_q14;
    int16_t  q1_q14;
    int16_t  q2_q14;
    int16_t  q3_q14;
    uint16_t battery_mv;
    uint8_t  stability_flags;
    uint8_t  guidance_active;
} TelemetryPacket;
#pragma pack(pop)

static_assert(sizeof(TelemetryPacket) == TELEMETRY_PACKET_SIZE,
              "TelemetryPacket must be 40 bytes — must match src/telemetry.h");

// CRC-8/SMBUS (poly 0x07, init 0x00). Inline so each ESP32 firmware is
// header-only for the wire format.
static inline uint8_t telemetry_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

#endif // TELEMETRY_PACKET_H
