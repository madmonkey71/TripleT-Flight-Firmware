#include <unity.h>
#include <math.h>

// ============================================================================
// MATH FUNCTIONS UNIT TESTS
//
// Tests physics calculations used in flight logic:
// - Hypsometric formula (barometric altitude)
// - Quaternion math (orientation)
// - Velocity integration (numerical methods)
// - Unit conversions
// ============================================================================

extern "C" {

// ========================================================================
// HYPSOMETRIC FORMULA TESTS
// ========================================================================

// Standard hypsometric formula: h = 44330 * (1 - (P/P0)^(1/5.255))
// Where: P = measured pressure (Pa), P0 = sea level pressure (Pa)

void test_hypsometric_sea_level(void) {
  // At sea level: P = P0 = 101325 Pa
  // Expected altitude: h = 0 m

  const float P0 = 101325.0f;  // Standard sea level pressure Pa
  const float P = 101325.0f;   // At sea level
  const float h = 44330.0f * (1.0f - pow(P / P0, 1.0f / 5.255f));

  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, h);
}

void test_hypsometric_1000_meters(void) {
  // At 1000m: Pressure ≈ 89875 Pa
  // Expected altitude: h ≈ 1000 m

  const float P0 = 101325.0f;
  const float P = 89875.0f;    // ~1000m altitude
  const float h = 44330.0f * (1.0f - pow(P / P0, 1.0f / 5.255f));

  TEST_ASSERT_FLOAT_WITHIN(10.0f, 1000.0f, h);  // ±10m tolerance
}

void test_hypsometric_5000_meters(void) {
  // At 5000m: Pressure ≈ 54048 Pa
  // Expected altitude: h ≈ 5000 m

  const float P0 = 101325.0f;
  const float P = 54048.0f;    // ~5000m altitude
  const float h = 44330.0f * (1.0f - pow(P / P0, 1.0f / 5.255f));

  TEST_ASSERT_FLOAT_WITHIN(50.0f, 5000.0f, h);  // ±50m tolerance
}

void test_hypsometric_10000_meters(void) {
  // At 10000m: Pressure ≈ 26436 Pa
  // Expected altitude: h ≈ 10000 m

  const float P0 = 101325.0f;
  const float P = 26436.0f;    // ~10000m altitude
  const float h = 44330.0f * (1.0f - pow(P / P0, 1.0f / 5.255f));

  TEST_ASSERT_FLOAT_WITHIN(100.0f, 10000.0f, h);  // ±100m tolerance
}

void test_hypsometric_monotonic(void) {
  // As pressure increases (descending), altitude decreases
  const float P0 = 101325.0f;

  float h_prev = 44330.0f * (1.0f - pow(26436.0f / P0, 1.0f / 5.255f));  // 10km
  float h_curr = 44330.0f * (1.0f - pow(54048.0f / P0, 1.0f / 5.255f));  // 5km

  TEST_ASSERT_GREATER_THAN(h_curr, h_prev);  // As pressure increases, h decreases
}

// ========================================================================
// QUATERNION MATH TESTS
// ========================================================================

void test_quaternion_identity(void) {
  // Identity quaternion (no rotation): q = (1, 0, 0, 0)
  float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;

  // Magnitude should be 1
  float mag = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mag);
}

void test_quaternion_normalization(void) {
  // Quaternion (2, 0, 0, 0) should normalize to (1, 0, 0, 0)
  float qw = 2.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;

  float mag = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  qw /= mag;

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, qw);
}

void test_quaternion_conjugate(void) {
  // Conjugate of q = (w, x, y, z) is q* = (w, -x, -y, -z)
  float qw = 1.0f, qx = 0.5f, qy = 0.5f, qz = 0.5f;
  float q_conj_x = -qx;
  float q_conj_y = -qy;
  float q_conj_z = -qz;

  TEST_ASSERT_EQUAL_FLOAT(-0.5f, q_conj_x);
  TEST_ASSERT_EQUAL_FLOAT(-0.5f, q_conj_y);
  TEST_ASSERT_EQUAL_FLOAT(-0.5f, q_conj_z);
}

void test_quaternion_magnitude(void) {
  // Magnitude of q = (0.6, 0.8, 0, 0) should be 1
  float qw = 0.6f, qx = 0.8f, qy = 0.0f, qz = 0.0f;
  float mag = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);

  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, mag);
}

// ========================================================================
// UNIT CONVERSION TESTS
// ========================================================================

void test_gravity_conversion_g_to_mps2(void) {
  // 1 g = 9.81 m/s²
  float accel_g = 1.0f;
  float accel_mps2 = accel_g * 9.81f;

  TEST_ASSERT_FLOAT_WITHIN(0.01f, 9.81f, accel_mps2);
}

void test_gravity_conversion_high_g(void) {
  // 100 g = 981 m/s²
  float accel_g = 100.0f;
  float accel_mps2 = accel_g * 9.81f;

  TEST_ASSERT_FLOAT_WITHIN(1.0f, 981.0f, accel_mps2);
}

void test_pressure_conversion_hpa_to_pa(void) {
  // 1 hPa = 100 Pa
  float pressure_hpa = 1013.25f;  // Sea level
  float pressure_pa = pressure_hpa * 100.0f;

  TEST_ASSERT_FLOAT_WITHIN(1.0f, 101325.0f, pressure_pa);
}

void test_altitude_conversion_mm_to_m(void) {
  // GPS altitude is in mm, convert to m
  long gps_alt_mm = 1500000;  // 1500m in mm
  float alt_m = gps_alt_mm / 1000.0f;

  TEST_ASSERT_FLOAT_WITHIN(0.1f, 1500.0f, alt_m);
}

void test_angular_velocity_rad_to_deg(void) {
  // Gyro in rad/s, convert to deg/s
  // 1 rad = 180/π ≈ 57.3 deg
  float gyro_rad = 1.0f;
  float gyro_deg = gyro_rad * 57.2958f;

  TEST_ASSERT_FLOAT_WITHIN(0.1f, 57.2958f, gyro_deg);
}

// ========================================================================
// VELOCITY INTEGRATION TESTS
// ========================================================================

void test_velocity_integration_constant_accel(void) {
  // v = v0 + a*t
  // With a=10 m/s², t=1s, v0=0: v = 10 m/s

  float v = 0.0f;
  float a = 10.0f;
  float dt = 1.0f;
  v += a * dt;

  TEST_ASSERT_FLOAT_WITHIN(0.1f, 10.0f, v);
}

void test_altitude_integration_constant_vel(void) {
  // h = h0 + v*t
  // With v=10 m/s, t=2s, h0=0: h = 20 m

  float h = 0.0f;
  float v = 10.0f;
  float dt = 2.0f;
  h += v * dt;

  TEST_ASSERT_FLOAT_WITHIN(0.1f, 20.0f, h);
}

void test_altitude_integration_with_drag(void) {
  // Simplified: a = g - (b*v²/m)
  // For realistic rocket: drag increases with velocity

  // Expected: Altitude increases, velocity decreases
  float alt = 0.0f, vel = 100.0f;
  float g = -9.81f;
  float drag_coeff = 0.0001f;
  float dt = 0.1f;

  for (int i = 0; i < 10; i++) {
    float a = g - (drag_coeff * vel * vel);
    vel += a * dt;
    alt += vel * dt;
  }

  TEST_ASSERT_GREATER_THAN(0.0f, alt);  // Still ascending
  TEST_ASSERT_LESS_THAN(100.0f, vel);   // Velocity decreased
}

// ========================================================================
// TRIGONOMETRY TESTS
// ========================================================================

void test_sin_values(void) {
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, sin(0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, sin(M_PI / 2));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, sin(M_PI));
}

void test_cos_values(void) {
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, cos(0));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, cos(M_PI / 2));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, cos(M_PI));
}

void test_atan2_quadrants(void) {
  // atan2(y, x) should work in all quadrants
  float angle_q1 = atan2(1.0f, 1.0f);    // 45 degrees
  float angle_q2 = atan2(1.0f, -1.0f);   // 135 degrees

  TEST_ASSERT_TRUE(angle_q1 > 0 && angle_q1 < M_PI / 2);
  TEST_ASSERT_TRUE(angle_q2 > M_PI / 2 && angle_q2 < M_PI);
}

}  // extern "C"

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_hypsometric_sea_level);
    RUN_TEST(test_hypsometric_1000_meters);
    RUN_TEST(test_hypsometric_5000_meters);
    RUN_TEST(test_hypsometric_10000_meters);
    RUN_TEST(test_hypsometric_monotonic);
    RUN_TEST(test_quaternion_identity);
    RUN_TEST(test_quaternion_normalization);
    RUN_TEST(test_quaternion_conjugate);
    RUN_TEST(test_quaternion_magnitude);
    RUN_TEST(test_gravity_conversion_g_to_mps2);
    RUN_TEST(test_gravity_conversion_high_g);
    RUN_TEST(test_pressure_conversion_hpa_to_pa);
    RUN_TEST(test_altitude_conversion_mm_to_m);
    RUN_TEST(test_angular_velocity_rad_to_deg);
    RUN_TEST(test_velocity_integration_constant_accel);
    RUN_TEST(test_altitude_integration_constant_vel);
    RUN_TEST(test_altitude_integration_with_drag);
    RUN_TEST(test_sin_values);
    RUN_TEST(test_cos_values);
    RUN_TEST(test_atan2_quadrants);
    return UNITY_END();
}
