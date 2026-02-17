#include <unity.h>
#include <math.h>

// ============================================================================
// LANDING DETECTION UNIT TESTS
//
// Tests landing detection logic: altitude stability, velocity near zero,
// acceleration near 1g, and confirmation timing.
// ============================================================================

extern "C" {

#define LANDING_ACCEL_MIN_G 0.9f
#define LANDING_ACCEL_MAX_G 1.1f
#define LANDING_CONFIRMATION_TIME_MS 2000
#define LANDING_ALTITUDE_STABLE_THRESHOLD 1.0f

bool is_altitude_stable(float current, float previous) {
  return fabsf(current - previous) < LANDING_ALTITUDE_STABLE_THRESHOLD;
}

bool is_acceleration_landing_range(float accel_g) {
  return (accel_g >= LANDING_ACCEL_MIN_G && accel_g <= LANDING_ACCEL_MAX_G);
}

bool is_velocity_near_zero(float velocity, float threshold) {
  return fabsf(velocity) < threshold;
}

void test_landing_detected_when_altitude_stable(void) {
  TEST_ASSERT_TRUE(is_altitude_stable(100.0f, 100.1f));
}

void test_landing_not_detected_during_descent(void) {
  TEST_ASSERT_FALSE(is_altitude_stable(500.0f, 400.0f));
}

void test_landing_detection_with_noise(void) {
  TEST_ASSERT_TRUE(is_altitude_stable(100.0f, 100.45f));
}

void test_landing_detection_minimum_time(void) {
  unsigned long stable_start = 0;
  // 1.5s - not enough
  TEST_ASSERT_FALSE((1500UL - stable_start) >= LANDING_CONFIRMATION_TIME_MS);
  // 2.5s - sufficient
  TEST_ASSERT_TRUE((2500UL - stable_start) >= LANDING_CONFIRMATION_TIME_MS);
}

void test_false_landing_rejection_during_coast(void) {
  // Free fall: ~0g, not landing range
  TEST_ASSERT_FALSE(is_acceleration_landing_range(0.1f));
}

void test_landing_from_main_descent(void) {
  TEST_ASSERT_FALSE(is_velocity_near_zero(-5.0f, 0.5f));  // descending
  TEST_ASSERT_TRUE(is_velocity_near_zero(0.1f, 0.5f));    // touchdown
  TEST_ASSERT_TRUE(is_acceleration_landing_range(1.0f));   // on ground
}

void test_landing_altitude_threshold(void) {
  TEST_ASSERT_TRUE(is_altitude_stable(500.0f, 500.99f));   // below threshold
  TEST_ASSERT_FALSE(is_altitude_stable(500.0f, 501.01f));  // above threshold
}

void test_landing_velocity_near_zero(void) {
  TEST_ASSERT_FALSE(is_velocity_near_zero(-3.0f, 0.5f));
  TEST_ASSERT_TRUE(is_velocity_near_zero(0.2f, 0.5f));
}

}  // extern "C"

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_landing_detected_when_altitude_stable);
    RUN_TEST(test_landing_not_detected_during_descent);
    RUN_TEST(test_landing_detection_with_noise);
    RUN_TEST(test_landing_detection_minimum_time);
    RUN_TEST(test_false_landing_rejection_during_coast);
    RUN_TEST(test_landing_from_main_descent);
    RUN_TEST(test_landing_altitude_threshold);
    RUN_TEST(test_landing_velocity_near_zero);
    return UNITY_END();
}
