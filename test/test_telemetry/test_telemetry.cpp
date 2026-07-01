#include <unity.h>
#include <math.h>
#include <string.h>

// Pull in the production telemetry module. ArduinoFake provides Arduino.h
// so data_structures.h compiles natively.
#include "../../src/telemetry.h"
#include "../../src/telemetry.cpp"

// ============================================================================
// TELEMETRY PACKING TESTS
//
// Verifies:
// - Wire-format size and field layout (TelemetryPacket is exactly 38 bytes)
// - Pack scaling for floats (accel, quaternions, battery, baro)
// - NaN / out-of-range handling (must not corrupt downstream values)
// - CRC-8/SMBUS implementation (matches reference vectors)
// - Frame layout: SYNC + LEN + payload + CRC
// ============================================================================

static LogData zero_log_data() {
    LogData d;
    memset(&d, 0, sizeof(d));
    return d;
}

// --- Size / layout ----------------------------------------------------------

void test_packet_size_is_40_bytes(void) {
    TEST_ASSERT_EQUAL(40, (int)sizeof(TelemetryPacket));
    TEST_ASSERT_EQUAL(40, TELEMETRY_PACKET_SIZE);
}

void test_frame_size_is_43_bytes(void) {
    // 1 sync + 1 len + 40 payload + 1 crc = 43
    TEST_ASSERT_EQUAL(43, TELEMETRY_FRAME_SIZE);
}

// --- Basic pack -------------------------------------------------------------

void test_pack_zero_logdata_produces_zero_packet(void) {
    LogData d = zero_log_data();
    TelemetryPacket p;
    telemetry_pack(d, p);
    TEST_ASSERT_EQUAL_UINT32(0, p.timestamp_ms);
    TEST_ASSERT_EQUAL_UINT8(0, p.flight_state);
    TEST_ASSERT_EQUAL_INT32(0, p.latitude_e7);
    TEST_ASSERT_EQUAL_INT16(0, p.accel_x_mg);
    TEST_ASSERT_EQUAL_INT16(0, p.q0_q14);
    TEST_ASSERT_EQUAL_UINT16(0, p.battery_mv);
    TEST_ASSERT_EQUAL_UINT8(0, p.stability_flags);
}

void test_pack_passes_through_integer_fields(void) {
    LogData d = zero_log_data();
    d.timestamp        = 1234567u;
    d.flightState      = 6;          // APOGEE
    d.last_error_code  = 90;         // soft guidance disable
    d.latitude         = 340000000;  // 34.0 deg * 1e7
    d.longitude        = -1180000000;// -118.0 deg * 1e7
    d.altitudeMSL      = 123456;     // mm
    d.stability_flags  = 0b00000101;
    d.guidance_active  = true;

    TelemetryPacket p;
    telemetry_pack(d, p);
    TEST_ASSERT_EQUAL_UINT32(1234567u, p.timestamp_ms);
    TEST_ASSERT_EQUAL_UINT8(6, p.flight_state);
    TEST_ASSERT_EQUAL_UINT8(90, p.error_code);
    TEST_ASSERT_EQUAL_INT32(340000000, p.latitude_e7);
    TEST_ASSERT_EQUAL_INT32(-1180000000, p.longitude_e7);
    TEST_ASSERT_EQUAL_INT32(123456, p.altitude_gps_mm);
    TEST_ASSERT_EQUAL_UINT8(0b00000101, p.stability_flags);
    TEST_ASSERT_EQUAL_UINT8(1, p.guidance_active);
}

void test_pack_scales_accel_to_milligs(void) {
    LogData d = zero_log_data();
    d.icm_accel[0] = 1.0f;   // 1g  -> 1000
    d.icm_accel[1] = -2.5f;  // -2.5g -> -2500
    d.icm_accel[2] = 0.001f; // 0.001g -> 1

    TelemetryPacket p;
    telemetry_pack(d, p);
    TEST_ASSERT_EQUAL_INT16(1000, p.accel_x_mg);
    TEST_ASSERT_EQUAL_INT16(-2500, p.accel_y_mg);
    TEST_ASSERT_EQUAL_INT16(1, p.accel_z_mg);
}

void test_pack_clamps_accel_at_int16_limits(void) {
    LogData d = zero_log_data();
    d.icm_accel[0] = 100.0f;   // 100g -> 100000 -> clamped to 32767
    d.icm_accel[1] = -100.0f;  // -100g -> -100000 -> clamped to -32768
    d.icm_accel[2] = nanf(""); // NaN -> 0

    TelemetryPacket p;
    telemetry_pack(d, p);
    TEST_ASSERT_EQUAL_INT16(32767, p.accel_x_mg);
    TEST_ASSERT_EQUAL_INT16(-32768, p.accel_y_mg);
    TEST_ASSERT_EQUAL_INT16(0, p.accel_z_mg);
}

void test_pack_scales_quaternion_to_q14_fixed_point(void) {
    LogData d = zero_log_data();
    d.q0 = 1.0f;    // -> 16384
    d.q1 = 0.0f;    // -> 0
    d.q2 = 0.5f;    // -> 8192
    d.q3 = -0.5f;   // -> -8192

    TelemetryPacket p;
    telemetry_pack(d, p);
    TEST_ASSERT_EQUAL_INT16(16384, p.q0_q14);
    TEST_ASSERT_EQUAL_INT16(0, p.q1_q14);
    TEST_ASSERT_EQUAL_INT16(8192, p.q2_q14);
    TEST_ASSERT_EQUAL_INT16(-8192, p.q3_q14);
}

void test_pack_handles_battery_voltage(void) {
    LogData d = zero_log_data();
    d.battery_voltage = 11.1f;      // -> 11100 mV
    TelemetryPacket p;
    telemetry_pack(d, p);
    TEST_ASSERT_EQUAL_UINT16(11100, p.battery_mv);

    d.battery_voltage = -1.0f;      // -> clamp to 0
    telemetry_pack(d, p);
    TEST_ASSERT_EQUAL_UINT16(0, p.battery_mv);

    d.battery_voltage = 100.0f;     // 100000 -> clamp to 65535
    telemetry_pack(d, p);
    TEST_ASSERT_EQUAL_UINT16(65535, p.battery_mv);

    d.battery_voltage = nanf("");
    telemetry_pack(d, p);
    TEST_ASSERT_EQUAL_UINT16(0, p.battery_mv);
}

void test_pack_converts_baro_altitude_to_mm(void) {
    LogData d = zero_log_data();
    d.calibrated_altitude = 1234.567f;  // m -> 1234567 mm
    TelemetryPacket p;
    telemetry_pack(d, p);
    TEST_ASSERT_INT_WITHIN(2, 1234567, p.altitude_baro_mm);
}

void test_pack_handles_nan_baro_safely(void) {
    LogData d = zero_log_data();
    d.calibrated_altitude = nanf("");
    TelemetryPacket p;
    telemetry_pack(d, p);
    TEST_ASSERT_EQUAL_INT32(0, p.altitude_baro_mm);
}

// --- CRC ---------------------------------------------------------------------

void test_crc8_of_empty_is_zero(void) {
    TEST_ASSERT_EQUAL_UINT8(0x00, telemetry_crc8(NULL, 0));
}

void test_crc8_of_single_byte(void) {
    // CRC-8/SMBUS reference: crc8(0x00) = 0x00; crc8(0x01) = 0x07; crc8(0xFF) = 0xF3.
    uint8_t b;
    b = 0x00; TEST_ASSERT_EQUAL_UINT8(0x00, telemetry_crc8(&b, 1));
    b = 0x01; TEST_ASSERT_EQUAL_UINT8(0x07, telemetry_crc8(&b, 1));
    b = 0xFF; TEST_ASSERT_EQUAL_UINT8(0xF3, telemetry_crc8(&b, 1));
}

void test_crc8_changes_with_data(void) {
    uint8_t a[] = {0x01, 0x02, 0x03};
    uint8_t b[] = {0x01, 0x02, 0x04};
    TEST_ASSERT_NOT_EQUAL(telemetry_crc8(a, 3), telemetry_crc8(b, 3));
}

// --- Frame layout ------------------------------------------------------------

void test_frame_has_sync_and_length(void) {
    TelemetryPacket p;
    memset(&p, 0, sizeof(p));
    p.timestamp_ms = 0xDEADBEEFu;

    uint8_t buf[TELEMETRY_FRAME_SIZE];
    size_t n = telemetry_frame(p, buf);
    TEST_ASSERT_EQUAL(TELEMETRY_FRAME_SIZE, (int)n);
    TEST_ASSERT_EQUAL_UINT8(TELEMETRY_SYNC_BYTE, buf[0]);
    TEST_ASSERT_EQUAL_UINT8(TELEMETRY_PACKET_SIZE, buf[1]);

    // The payload should round-trip back to the same struct.
    TelemetryPacket round;
    memcpy(&round, &buf[2], TELEMETRY_PACKET_SIZE);
    TEST_ASSERT_EQUAL_UINT32(0xDEADBEEFu, round.timestamp_ms);
}

void test_frame_crc_matches_payload(void) {
    TelemetryPacket p;
    memset(&p, 0, sizeof(p));
    p.timestamp_ms = 42;
    p.flight_state = 3;

    uint8_t buf[TELEMETRY_FRAME_SIZE];
    telemetry_frame(p, buf);

    uint8_t expected = telemetry_crc8(&buf[2], TELEMETRY_PACKET_SIZE);
    TEST_ASSERT_EQUAL_UINT8(expected, buf[TELEMETRY_FRAME_SIZE - 1]);
}

void test_frame_crc_detects_corruption(void) {
    TelemetryPacket p;
    memset(&p, 0, sizeof(p));
    p.battery_mv = 12000;

    uint8_t buf[TELEMETRY_FRAME_SIZE];
    telemetry_frame(p, buf);

    // Flip a payload bit and verify the recomputed CRC no longer matches.
    buf[5] ^= 0x01;
    uint8_t recomputed = telemetry_crc8(&buf[2], TELEMETRY_PACKET_SIZE);
    TEST_ASSERT_NOT_EQUAL(recomputed, buf[TELEMETRY_FRAME_SIZE - 1]);
}

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_packet_size_is_40_bytes);
    RUN_TEST(test_frame_size_is_43_bytes);
    RUN_TEST(test_pack_zero_logdata_produces_zero_packet);
    RUN_TEST(test_pack_passes_through_integer_fields);
    RUN_TEST(test_pack_scales_accel_to_milligs);
    RUN_TEST(test_pack_clamps_accel_at_int16_limits);
    RUN_TEST(test_pack_scales_quaternion_to_q14_fixed_point);
    RUN_TEST(test_pack_handles_battery_voltage);
    RUN_TEST(test_pack_converts_baro_altitude_to_mm);
    RUN_TEST(test_pack_handles_nan_baro_safely);
    RUN_TEST(test_crc8_of_empty_is_zero);
    RUN_TEST(test_crc8_of_single_byte);
    RUN_TEST(test_crc8_changes_with_data);
    RUN_TEST(test_frame_has_sync_and_length);
    RUN_TEST(test_frame_crc_matches_payload);
    RUN_TEST(test_frame_crc_detects_corruption);
    return UNITY_END();
}
