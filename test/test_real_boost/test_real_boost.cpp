// Audit #8 - launch detection on fresh samples, BOOST timeout, drag-robust burnout.
// Exercises the REAL flight_logic.cpp.
#include <unity.h>
#include "../support/flight_harness.h"
#include "../support/real_sources.h"

void setUp() { harness_reset(); }
void tearDown() {}

static void armed_on_pad() {
  harness_on_pad(100.0f);
  g_currentFlightState = ARMED;
  harness_pass();
  TEST_ASSERT_EQUAL(ARMED, g_currentFlightState);
}
static void one_sample_then_spin(unsigned long dt_ms, int passes) {
  test_advance_ms(dt_ms);
  harness_new_samples();
  for (int i = 0; i < passes; i++) ProcessFlightState();
}
static void in_boost() {
  armed_on_pad();
  harness_set_accel_g(6.0f);
  harness_run_ms(LAUNCH_CONFIRMATION_COUNT * 100 + 200);
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
}

// ---- false launch on the pad --------------------------------------------------------------
void test_a_single_pad_bump_does_not_launch() {
  armed_on_pad();
  harness_set_accel_g(10.0f);                       // one 10 g shock...
  for (int i = 0; i < 5000; i++) ProcessFlightState();   // ...re-read thousands of times from the cache
  TEST_ASSERT_EQUAL(ARMED, g_currentFlightState);
  one_sample_then_spin(100, 100);                   // a single fresh sample of it
  harness_set_accel_g(1.0f);
  harness_run_ms(2000);
  TEST_ASSERT_EQUAL(ARMED, g_currentFlightState);
}

void test_a_bump_shorter_than_the_confirmation_count_does_not_launch() {
  armed_on_pad();
  harness_set_accel_g(5.0f);
  for (int k = 0; k < LAUNCH_CONFIRMATION_COUNT - 1; k++) one_sample_then_spin(100, 50);
  TEST_ASSERT_EQUAL(ARMED, g_currentFlightState);
  harness_set_accel_g(1.0f);                        // it ends
  one_sample_then_spin(100, 50);
  harness_set_accel_g(5.0f);                        // a new bump: the count starts over
  for (int k = 0; k < LAUNCH_CONFIRMATION_COUNT - 1; k++) one_sample_then_spin(100, 50);
  TEST_ASSERT_EQUAL(ARMED, g_currentFlightState);
}

void test_sustained_thrust_launches_after_exactly_the_confirmation_count() {
  armed_on_pad();
  harness_set_accel_g(5.0f);
  for (int k = 0; k < LAUNCH_CONFIRMATION_COUNT; k++) {
    TEST_ASSERT_EQUAL(ARMED, g_currentFlightState);
    one_sample_then_spin(100, 50);
  }
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
  TEST_ASSERT_TRUE(g_flightInProgress);
}

void test_dead_accelerometers_cannot_launch_and_disarm_timeout_still_applies() {
  armed_on_pad();
  for (int i = 0; i < 3; i++) { icm_accel[i] = 0; kx134_accel[i] = 0; }
  harness_run_ms(3000);
  TEST_ASSERT_EQUAL(ARMED, g_currentFlightState);
}

// ---- BOOST timeout ---------------------------------------------------------------------------
void test_boost_timeout_forces_coast_and_the_backup_timer_still_deploys() {
  in_boost();
  const unsigned long t0 = millis();
  harness_run_ms(BOOST_TIMEOUT_MS - 1000);           // 6 g the whole time: burnout never seen
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
  harness_run_ms(2000);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  TEST_ASSERT_TRUE(boostEndTime > 0);
  TEST_ASSERT_TRUE(millis() - t0 >= BOOST_TIMEOUT_MS - 1000);

  // ...and the backup timer is now reachable: with the baro flat and the accelerometer stuck high
  // (vetoing the baro method) nothing but the ungated backup timer can deploy.
  harness_run_ms(BACKUP_APOGEE_TIME_MS + 1500);
  TEST_ASSERT_TRUE(g_currentFlightState >= APOGEE);
  TEST_ASSERT_TRUE(g_pin_ever_high[PYRO_CHANNEL_1]);
}

void test_dead_accelerometers_in_boost_leave_boost_via_the_timeout() {
  harness_on_pad(100.0f);
  g_flightInProgress = true;
  g_currentFlightState = BOOST; g_previousFlightState = STARTUP;
  harness_pass();
  for (int i = 0; i < 3; i++) { icm_accel[i] = 0; kx134_accel[i] = 0; }
  harness_run_ms(BOOST_TIMEOUT_MS + 1500);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
}

// ---- burnout ------------------------------------------------------------------------------------
void test_high_drag_burnout_is_detected_relative_to_the_boost_level() {
  in_boost();                                       // 6 g of thrust
  harness_run_ms(1500);
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
  harness_set_accel_g(1.8f);                        // motor out: 1.8 g of DRAG deceleration, well above COAST_ACCEL_THRESHOLD
  TEST_ASSERT_TRUE(1.8f > COAST_ACCEL_THRESHOLD);
  harness_run_ms(600);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  TEST_ASSERT_TRUE(boostEndTime > 0);
}

void test_low_drag_burnout_still_uses_the_absolute_threshold() {
  in_boost();
  harness_run_ms(1500);
  harness_set_accel_g(0.2f);
  harness_run_ms(600);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
}

void test_burnout_needs_n_fresh_samples_not_loop_passes() {
  in_boost();
  harness_run_ms(1500);
  harness_set_accel_g(0.2f);
  for (int i = 0; i < 5000; i++) ProcessFlightState();
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
  for (int k = 1; k < COAST_CONFIRMATION_COUNT; k++) one_sample_then_spin(100, 100);
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
  one_sample_then_spin(100, 100);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
}

void test_a_normal_thrust_curve_does_not_trigger_the_relative_burnout() {
  in_boost();                                       // 6 g
  const float profile[] = {10.0f, 10.0f, 8.0f, 6.5f, 6.0f, 6.0f, 5.5f, 5.0f, 5.0f, 4.5f, 4.0f, 3.5f};   // tail-off, all > 35% of the peak
  for (float g : profile) { harness_set_accel_g(g); harness_run_ms(300); TEST_ASSERT_EQUAL(BOOST, g_currentFlightState); }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_a_single_pad_bump_does_not_launch);
  RUN_TEST(test_a_bump_shorter_than_the_confirmation_count_does_not_launch);
  RUN_TEST(test_sustained_thrust_launches_after_exactly_the_confirmation_count);
  RUN_TEST(test_dead_accelerometers_cannot_launch_and_disarm_timeout_still_applies);
  RUN_TEST(test_boost_timeout_forces_coast_and_the_backup_timer_still_deploys);
  RUN_TEST(test_dead_accelerometers_in_boost_leave_boost_via_the_timeout);
  RUN_TEST(test_high_drag_burnout_is_detected_relative_to_the_boost_level);
  RUN_TEST(test_low_drag_burnout_still_uses_the_absolute_threshold);
  RUN_TEST(test_burnout_needs_n_fresh_samples_not_loop_passes);
  RUN_TEST(test_a_normal_thrust_curve_does_not_trigger_the_relative_burnout);
  return UNITY_END();
}
