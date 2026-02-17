#include <unity.h>
#include <stdint.h>

// ============================================================================
// GPS VALIDATION UNIT TESTS
//
// Tests GPS data validation: fix type, satellite count, timeout, coordinates.
// ============================================================================

extern "C" {

#define MAX_VALID_FIX_TYPE 5
#define MAX_VALID_SIV 100
#define MIN_VALID_SIV 4
#define GPS_TIMEOUT_MS 5000

bool is_gps_fix_type_valid(uint8_t fix_type) {
  return fix_type <= MAX_VALID_FIX_TYPE;
}

bool is_gps_fix_usable(uint8_t fix_type) {
  return fix_type >= 3;  // 3D fix or better
}

bool is_satellite_count_valid(uint8_t siv) {
  return siv <= MAX_VALID_SIV;
}

bool has_sufficient_satellites(uint8_t siv) {
  return siv >= MIN_VALID_SIV;
}

bool is_gps_data_fresh(unsigned long last_update, unsigned long now) {
  return (now - last_update) < GPS_TIMEOUT_MS;
}

bool are_coordinates_valid(float lat, float lon) {
  return (lat >= -90.0f && lat <= 90.0f && lon >= -180.0f && lon <= 180.0f);
}

void test_valid_gps_fix_accepted(void) {
  TEST_ASSERT_TRUE(is_gps_fix_type_valid(3));
  TEST_ASSERT_TRUE(is_gps_fix_usable(3));
  TEST_ASSERT_TRUE(is_satellite_count_valid(12));
  TEST_ASSERT_TRUE(has_sufficient_satellites(12));
}

void test_invalid_fix_type_rejected(void) {
  TEST_ASSERT_FALSE(is_gps_fix_type_valid(6));
  TEST_ASSERT_FALSE(is_gps_fix_type_valid(255));
}

void test_excessive_satellites_rejected(void) {
  TEST_ASSERT_FALSE(is_satellite_count_valid(150));
}

void test_no_fix_returns_invalid(void) {
  TEST_ASSERT_FALSE(is_gps_fix_usable(0));
  TEST_ASSERT_FALSE(is_gps_fix_usable(1));
  TEST_ASSERT_FALSE(is_gps_fix_usable(2));
}

void test_gps_timeout_handling(void) {
  TEST_ASSERT_TRUE(is_gps_data_fresh(1000, 3000));   // 2s: fresh
  TEST_ASSERT_FALSE(is_gps_data_fresh(1000, 7000));  // 6s: stale
}

void test_gps_coordinates_range_validation(void) {
  TEST_ASSERT_TRUE(are_coordinates_valid(40.7128f, -74.006f));   // NYC
  TEST_ASSERT_TRUE(are_coordinates_valid(0.0f, 0.0f));           // Equator
  TEST_ASSERT_TRUE(are_coordinates_valid(90.0f, 180.0f));        // Limits
  TEST_ASSERT_TRUE(are_coordinates_valid(-90.0f, -180.0f));      // Limits
  TEST_ASSERT_FALSE(are_coordinates_valid(95.0f, 0.0f));         // Bad lat
  TEST_ASSERT_FALSE(are_coordinates_valid(0.0f, 185.0f));        // Bad lon
}

}  // extern "C"
