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


// ---- audit #9: the accelerometer is only a gravity reference near 1 g ------------------------
// Before the fix kalman_update_accel() ran unconditionally, so 3 g of thrust along the body axis
// (or ~0 g in free fall) was treated as "down" and dragged roll/pitch towards it.
static void orient(float& r, float& p) { float y; kalman_get_orientation(r, p, y); }

void test_real_kalman_thrust_does_not_corrupt_the_estimate() {
  kalman_init(0.3f, 0.2f, 0.0f);            // vehicle is tilted 0.3 / 0.2 rad
  for (int i = 0; i < 300; i++) {           // 3 s of boost: 4 g along +Z (level "gravity" reading)
    kalman_predict(0, 0, 0, 0.01f);
    TEST_ASSERT_FALSE(kalman_update_accel(0.0f, 0.0f, 4.0f));
  }
  float r, p; orient(r, p);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.3f, r);   // untouched: an ungated filter would have flattened it to 0
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, p);
  TEST_ASSERT_EQUAL_UINT32(300, kalman_accel_updates_skipped());
}

void test_real_kalman_free_fall_is_skipped_and_never_produces_nan() {
  kalman_init(0.3f, 0.2f, 0.0f);
  const float freefall[][3] = {{0, 0, 0}, {0.001f, 0, 0}, {0, 0, 0.05f}, {0.02f, -0.03f, 0.01f}};
  for (auto& a : freefall) {
    kalman_predict(0, 0, 0, 0.01f);
    TEST_ASSERT_FALSE(kalman_update_accel(a[0], a[1], a[2]));
  }
  float r, p; orient(r, p);
  TEST_ASSERT_TRUE(isfinite(r) && isfinite(p));
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.3f, r);
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, p);
}

void test_real_kalman_rest_at_one_g_still_converges_and_reports_applied() {
  kalman_init(0.5f, -0.4f, 0.0f);
  bool applied = false;
  for (int i = 0; i < 400; i++) {
    kalman_predict(0, 0, 0, 0.01f);
    applied = kalman_update_accel(0.0f, 0.0f, 1.0f);
  }
  TEST_ASSERT_TRUE(applied);
  float r, p; orient(r, p);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, r);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, p);
}

void test_real_kalman_gate_band_edges() {
  const struct { float mag; bool applied; } cases[] = {
    {0.85f, false}, {0.89f, false}, {0.91f, true}, {1.0f, true}, {1.09f, true}, {1.11f, false}, {1.5f, false}, {16.0f, false}};
  for (auto& c : cases) {
    kalman_init(0.1f, 0.1f, 0.0f);
    kalman_predict(0, 0, 0, 0.01f);
    TEST_ASSERT_EQUAL_MESSAGE(c.applied, kalman_update_accel(0.0f, 0.0f, c.mag), "band edge");
  }
}

void test_real_kalman_fast_rotation_gates_the_update_even_at_one_g() {
  kalman_init(0.3f, 0.2f, 0.0f);
  kalman_predict(6.0f, 0, 0, 0.001f);                    // 6 rad/s, above the gate limit
  TEST_ASSERT_FALSE(kalman_update_accel(0.0f, 0.0f, 1.0f));
  kalman_predict(0.5f, 0, 0, 0.001f);                    // back to a modest rate
  TEST_ASSERT_TRUE(kalman_update_accel(0.0f, 0.0f, 1.0f));
}

void test_real_kalman_variance_grows_while_gated_so_it_reconverges_quickly() {
  kalman_init(0.3f, 0.2f, 0.0f);
  for (int i = 0; i < 300; i++) { kalman_predict(0, 0, 0, 0.01f); kalman_update_accel(0, 0, 1.0f); }
  const float settled = kalman_get_variance(0);
  float prev = settled;
  for (int i = 0; i < 1000; i++) {                       // 10 s of boost: gated
    kalman_predict(0, 0, 0, 0.01f);
    TEST_ASSERT_FALSE(kalman_update_accel(0, 0, 4.0f));
    TEST_ASSERT_TRUE(kalman_get_variance(0) >= prev);    // never shrinks while skipped
    prev = kalman_get_variance(0);
  }
  TEST_ASSERT_TRUE(kalman_get_variance(0) > settled * 5.0f);
  kalman_predict(0, 0, 0, 0.01f);
  TEST_ASSERT_TRUE(kalman_update_accel(0, 0, 1.0f));     // gate reopens (coast ended, at rest)
  TEST_ASSERT_TRUE(kalman_get_variance(0) < prev);
}

void test_real_kalman_atan2_zero_zero_case_is_guarded() {
  // Nose straight up: ay = az = 0, ax = 1 g. Roll is unobservable; pitch is -90 deg. Must stay finite
  // and must not drag roll to atan2(0,0) = 0.
  kalman_init(0.7f, 0.0f, 0.0f);
  kalman_predict(0, 0, 0, 0.01f);
  TEST_ASSERT_TRUE(kalman_update_accel(1.0f, 0.0f, 0.0f));
  float r, p; orient(r, p);
  TEST_ASSERT_TRUE(isfinite(r) && isfinite(p));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.7f, r);              // roll untouched
  TEST_ASSERT_TRUE(p < 0.0f);                            // pitch moved towards -90 deg
}

void test_real_kalman_rejects_non_finite_input() {
  kalman_init(0.3f, 0.2f, 0.0f);
  kalman_predict(0, 0, 0, 0.01f);
  TEST_ASSERT_FALSE(kalman_update_accel(NAN, 0.0f, 1.0f));
  TEST_ASSERT_FALSE(kalman_update_accel(0.0f, INFINITY, 1.0f));
  float r, p; orient(r, p);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.3f, r);
}

// A whole flight profile: pad (1 g) -> boost (5 g) -> coast (0.2 g drag) -> free-fall/descent (~0) -> landed (1 g).
void test_real_kalman_flight_profile_thrust_freefall_rest() {
  kalman_init(0.0f, 0.0f, 0.0f);
  const struct { float g; int n; } phases[] = {{1.0f, 200}, {5.0f, 300}, {0.2f, 500}, {0.05f, 300}, {1.0f, 300}};
  for (auto& ph : phases) {
    for (int i = 0; i < ph.n; i++) {
      kalman_predict(0, 0, 0, 0.01f);
      kalman_update_accel(0.0f, 0.0f, ph.g);
    }
  }
  float r, p; orient(r, p);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, r);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, p);
  TEST_ASSERT_TRUE(kalman_accel_updates_skipped() >= 1100);   // boost + coast + free fall were all skipped
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_real_kalman_predict_integrates_gyro);
  RUN_TEST(test_real_kalman_accel_update_levels_estimate_at_rest);
  RUN_TEST(test_real_kalman_thrust_does_not_corrupt_the_estimate);
  RUN_TEST(test_real_kalman_free_fall_is_skipped_and_never_produces_nan);
  RUN_TEST(test_real_kalman_rest_at_one_g_still_converges_and_reports_applied);
  RUN_TEST(test_real_kalman_gate_band_edges);
  RUN_TEST(test_real_kalman_fast_rotation_gates_the_update_even_at_one_g);
  RUN_TEST(test_real_kalman_variance_grows_while_gated_so_it_reconverges_quickly);
  RUN_TEST(test_real_kalman_atan2_zero_zero_case_is_guarded);
  RUN_TEST(test_real_kalman_rejects_non_finite_input);
  RUN_TEST(test_real_kalman_flight_profile_thrust_freefall_rest);
  return UNITY_END();
}
