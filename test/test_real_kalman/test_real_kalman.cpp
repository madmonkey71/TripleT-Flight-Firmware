// Tests that exercise the REAL src/kalman_filter.cpp (not a re-implementation).
#include <unity.h>
#include "../../src/kalman_filter.cpp"

void setUp() { kalman_init(0.0f, 0.0f, 0.0f); }
void tearDown() {}

// Baseline: gyro integration moves the estimate.
void test_real_kalman_predict_integrates_gyro() {
  kalman_predict(1.0f, 0.0f, 0.0f, 0.1f);   // 1 rad/s roll for 0.1 s
  float r, p, y;
  kalman_get_orientation(r, p, y);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.1f, r);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, p);
}

// Baseline: at rest (1 g on +Z) repeated accel updates pull the tilt to level.
void test_real_kalman_accel_update_levels_estimate_at_rest() {
  kalman_init(0.5f, -0.4f, 0.0f);            // start tilted
  for (int i = 0; i < 400; i++) {
    kalman_predict(0, 0, 0, 0.01f);
    kalman_update_accel(0.0f, 0.0f, 1.0f);
  }
  float r, p, y;
  kalman_get_orientation(r, p, y);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, r);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, p);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_real_kalman_predict_integrates_gyro);
  RUN_TEST(test_real_kalman_accel_update_levels_estimate_at_rest);
  return UNITY_END();
}
