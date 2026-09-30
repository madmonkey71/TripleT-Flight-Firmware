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
  harness_run_ms(500);                              // the drop must first SETTLE (3 fresh samples) ...
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
  harness_run_ms(600);                              // ... and then be confirmed by COAST_CONFIRMATION_COUNT more
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

// ---- the repo's own motor: Aerotech H125W (flight_simulation_h125w.py) --------------------------------------
// 276 N peak (~18 g) decaying gradually to 0 at 2.77 s. Its tail-off falls below 35 % of the peak ~1 s before
// burnout, so a relative test that ignores "still falling" would end BOOST early on the user's real motor.
struct ThrustPt { float t, n; };
static const ThrustPt kH125w[] = {
  {0.053f, 276}, {0.161f, 241}, {0.270f, 216}, {0.378f, 199}, {0.488f, 189}, {0.597f, 182}, {0.705f, 176},
  {0.814f, 169}, {0.922f, 162}, {1.031f, 154}, {1.141f, 144}, {1.249f, 133}, {1.357f, 123}, {1.466f, 113},
  {1.575f, 102}, {1.684f, 88}, {1.793f, 74}, {1.901f, 60}, {2.009f, 47}, {2.119f, 37}, {2.228f, 30},
  {2.336f, 24}, {2.445f, 19}, {2.553f, 15}, {2.663f, 11}, {2.772f, 0}};
static float h125w_thrust_n(float t) {
  const int n = sizeof(kH125w) / sizeof(kH125w[0]);
  if (t <= kH125w[0].t) return kH125w[0].n;
  if (t >= kH125w[n - 1].t) return 0.0f;
  for (int i = 0; i < n - 1; i++)
    if (t >= kH125w[i].t && t <= kH125w[i + 1].t)
      return kH125w[i].n + (kH125w[i + 1].n - kH125w[i].n) * (t - kH125w[i].t) / (kH125w[i + 1].t - kH125w[i].t);
  return 0.0f;
}
// Specific force (g) the accelerometer reads t seconds after ignition: (thrust - drag) / mass / g.
static float h125w_specific_force_g(float t, float drag_g) {
  const float mass = 1.18f + 0.323f - 0.188f * (t < 2.77f ? t / 2.77f : 1.0f);
  const float f = h125w_thrust_n(t) / mass / 9.81f;
  return f > drag_g ? f : drag_g;                    // after burnout only drag decelerates the vehicle
}

// Fly the H125W profile through the real state machine: returns seconds after ignition when COAST began (or -1).
static float fly_motor(float (*specific_force_g)(float t, float drag_g), float drag_g, float max_s) {
  armed_on_pad();
  harness_set_accel_g(specific_force_g(0.0f, drag_g));
  harness_run_ms(LAUNCH_CONFIRMATION_COUNT * 100 + 100);        // liftoff confirmed
  const float ignition_offset = (LAUNCH_CONFIRMATION_COUNT * 100 + 100) / 1000.0f;   // samples during confirmation were at the peak
  for (float t = ignition_offset; t < max_s; t += 0.1f) {
    harness_set_accel_g(specific_force_g(t, drag_g));
    harness_run_ms(100);
    if (g_currentFlightState == COAST) return t;
  }
  return -1.0f;
}

void test_h125w_tail_off_is_not_mistaken_for_burnout_and_burnout_is_detected_at_its_real_end() {
  const float t_coast = fly_motor(h125w_specific_force_g, 0.15f, 6.0f);   // low-drag 29 mm rocket: ~0.15 g of drag
  TEST_ASSERT_TRUE_MESSAGE(t_coast > 0.0f, "burnout must be detected");
  TEST_ASSERT_TRUE_MESSAGE(t_coast >= 2.6f, "must not declare burnout during the thrust tail-off");
  TEST_ASSERT_TRUE_MESSAGE(t_coast <= 3.6f, "must detect burnout promptly after thrust ends (2.77 s)");
}

void test_h125w_on_a_high_drag_airframe_burns_out_only_after_the_motor_is_actually_out() {
  // Same motor on a draggy airframe: 2.2 g of drag after burnout (never below the 0.5 g absolute threshold).
  const float t_coast = fly_motor(h125w_specific_force_g, 2.2f, 8.0f);
  TEST_ASSERT_TRUE_MESSAGE(t_coast > 0.0f, "high-drag burnout must be detected relative to the boost level");
  TEST_ASSERT_TRUE_MESSAGE(t_coast >= 2.4f, "not during the tail-off");
  TEST_ASSERT_TRUE_MESSAGE(t_coast <= 4.5f, "prompt after the motor is out");
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
  RUN_TEST(test_h125w_tail_off_is_not_mistaken_for_burnout_and_burnout_is_detected_at_its_real_end);
  RUN_TEST(test_h125w_on_a_high_drag_airframe_burns_out_only_after_the_motor_is_actually_out);
  return UNITY_END();
}
