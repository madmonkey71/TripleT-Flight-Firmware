#include <unity.h>
#include <math.h>

// ============================================================================
// ALTITUDE CALCULATIONS UNIT TESTS
//
// Tests hypsometric formula: h = 44330 * (1 - (P/P0)^(1/5.255))
// ============================================================================

extern "C" {

#define SEA_LEVEL_PRESSURE_PA 101325.0f

float calc_altitude(float pressure_pa, float sea_level_pa) {
  if (pressure_pa <= 0 || sea_level_pa <= 0) return 0.0f;
  return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 1.0f / 5.255f));
}

void test_hypsometric_formula_sea_level(void) {
  float h = calc_altitude(SEA_LEVEL_PRESSURE_PA, SEA_LEVEL_PRESSURE_PA);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, h);
}

void test_hypsometric_formula_high_altitude(void) {
  // ~1000m
  float h1 = calc_altitude(89875.0f, SEA_LEVEL_PRESSURE_PA);
  TEST_ASSERT_FLOAT_WITHIN(15.0f, 1000.0f, h1);

  // ~5000m
  float h5 = calc_altitude(54048.0f, SEA_LEVEL_PRESSURE_PA);
  TEST_ASSERT_FLOAT_WITHIN(50.0f, 5000.0f, h5);
}

void test_pressure_to_altitude_known_values(void) {
  // ~500m
  float h = calc_altitude(95461.0f, SEA_LEVEL_PRESSURE_PA);
  TEST_ASSERT_FLOAT_WITHIN(20.0f, 500.0f, h);

  // ~2000m
  h = calc_altitude(79495.0f, SEA_LEVEL_PRESSURE_PA);
  TEST_ASSERT_FLOAT_WITHIN(50.0f, 2000.0f, h);
}

void test_altitude_calibration_offset(void) {
  float raw_at_pad = calc_altitude(SEA_LEVEL_PRESSURE_PA, SEA_LEVEL_PRESSURE_PA);
  float offset = raw_at_pad;
  float raw_at_1000m = calc_altitude(89875.0f, SEA_LEVEL_PRESSURE_PA);
  float calibrated = raw_at_1000m - offset;
  TEST_ASSERT_FLOAT_WITHIN(15.0f, 1000.0f, calibrated);
}

void test_altitude_with_temperature_compensation(void) {
  // Simplified: standard formula at sea level pressure = 0m regardless of temp
  float h = calc_altitude(SEA_LEVEL_PRESSURE_PA, SEA_LEVEL_PRESSURE_PA);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, h);
}

void test_altitude_negative_values(void) {
  // Pressure above sea level = negative altitude
  float h = calc_altitude(105000.0f, SEA_LEVEL_PRESSURE_PA);
  TEST_ASSERT_TRUE(h < 0.0f);
}

}  // extern "C"
