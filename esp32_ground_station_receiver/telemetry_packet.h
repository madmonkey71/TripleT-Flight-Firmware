#ifndef TELEMETRY_PACKET_H
#define TELEMETRY_PACKET_H

// Wire-format definitions for the TripleT telemetry link (ground side).
//
// **MUST match `src/telemetry.h` in the parent project and the copy in
// `esp32_telemetry_transmitter/telemetry_packet.h`.** Keep all three in
// sync by hand. The static_assert below will catch size mismatches but
// not field-order ones.

#include <stdint.h>
#include <stddef.h>

#define TELEMETRY_PACKET_SIZE 40

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

#endif // TELEMETRY_PACKET_H
