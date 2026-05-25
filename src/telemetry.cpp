#include "telemetry.h"

#include <math.h>
#include <string.h>

static int16_t clamp_to_int16(float v) {
    if (!isfinite(v)) return 0;
    if (v >  32767.0f) return  32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

void telemetry_pack(const LogData &src, TelemetryPacket &out) {
    memset(&out, 0, sizeof(out));

    out.timestamp_ms     = src.timestamp;
    out.flight_state     = src.flightState;
    out.error_code       = src.last_error_code;
    out.latitude_e7      = src.latitude;
    out.longitude_e7     = src.longitude;
    out.altitude_gps_mm  = src.altitudeMSL;

    float baro_mm = src.calibrated_altitude * TELEMETRY_BARO_SCALE;
    if (!isfinite(baro_mm))         baro_mm = 0.0f;
    else if (baro_mm >  2.0e9f)     baro_mm =  2.0e9f;
    else if (baro_mm < -2.0e9f)     baro_mm = -2.0e9f;
    out.altitude_baro_mm = (int32_t)baro_mm;

    out.accel_x_mg = clamp_to_int16(src.icm_accel[0] * TELEMETRY_ACCEL_SCALE);
    out.accel_y_mg = clamp_to_int16(src.icm_accel[1] * TELEMETRY_ACCEL_SCALE);
    out.accel_z_mg = clamp_to_int16(src.icm_accel[2] * TELEMETRY_ACCEL_SCALE);

    out.q0_q14 = clamp_to_int16(src.q0 * TELEMETRY_QUAT_SCALE);
    out.q1_q14 = clamp_to_int16(src.q1 * TELEMETRY_QUAT_SCALE);
    out.q2_q14 = clamp_to_int16(src.q2 * TELEMETRY_QUAT_SCALE);
    out.q3_q14 = clamp_to_int16(src.q3 * TELEMETRY_QUAT_SCALE);

    float mv = src.battery_voltage * TELEMETRY_BATTERY_SCALE;
    if (!isfinite(mv) || mv < 0.0f) mv = 0.0f;
    if (mv > 65535.0f) mv = 65535.0f;
    out.battery_mv = (uint16_t)mv;

    out.stability_flags = src.stability_flags;
    out.guidance_active = src.guidance_active ? 1 : 0;
}

uint8_t telemetry_crc8(const uint8_t *data, size_t len) {
    // CRC-8/SMBUS: poly 0x07, init 0x00, no reflection, no xor-out.
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

size_t telemetry_frame(const TelemetryPacket &pkt, uint8_t *buf) {
    buf[0] = TELEMETRY_SYNC_BYTE;
    buf[1] = TELEMETRY_PACKET_SIZE;
    memcpy(&buf[2], &pkt, TELEMETRY_PACKET_SIZE);
    buf[2 + TELEMETRY_PACKET_SIZE] = telemetry_crc8(&buf[2], TELEMETRY_PACKET_SIZE);
    return TELEMETRY_FRAME_SIZE;
}
