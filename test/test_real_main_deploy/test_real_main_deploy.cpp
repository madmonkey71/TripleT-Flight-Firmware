// Audit #5 - main deploy must be debounced on fresh samples, must still happen when the
// barometer fails (or lies), and the descent states must not persist forever.
// Exercises the REAL flight_logic.cpp.
#include <unity.h>
#include "../support/flight_harness.h"
#include "../support/real_sources.h"

void setUp() { harness_reset(); }
void tearDown() {}

// Under drogue at `alt` (absolute), launch 100 m, drogue fire already completed, max AGL `max_agl`.
static void under_drogue(float alt, float max_agl = 500.0f) {
  harness_on_pad(100.0f);
  g_flightInProgress = true;
  g_pyroFiredMask = PYRO_FIRED_DROGUE;
  g_launchAltitude = 100.0f;
  g_maxAltitudeReached = max_agl;
  g_main_deploy_altitude_m_agl = MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M;   // 100 m AGL => 200 m absolute
  harness_set_baro_alt(alt);
  g_currentFlightState = DROGUE_DESCENT;
  g_previousFlightState = STARTUP;      // let the state-entry actions run
  harness_pass();
  TEST_ASSERT_EQUAL(DROGUE_DESCENT, g_currentFlightState);
}
static bool main_fired() { return g_pin_ever_high[PYRO_CHANNEL_2]; }

// One fresh sample of everything, then spin the loop with nothing changing.
static void one_sample_then_spin(unsigned long dt_ms, int passes) {
  test_advance_ms(dt_ms);
  harness_new_samples();
  for (int i = 0; i < passes; i++) ProcessFlightState();
}

// ---- debounce ---------------------------------------------------------------------
void test_one_cached_low_reading_never_deploys_main() {
  under_drogue(600.0f);
  harness_set_baro_alt(150.0f);                       // 50 m AGL: below the 100 m deploy altitude
  for (int i = 0; i < 5000; i++) ProcessFlightState(); // one cached sample, thousands of passes
  TEST_ASSERT_FALSE(main_fired());
  TEST_ASSERT_EQUAL(DROGUE_DESCENT, g_currentFlightState);
}

void test_a_short_baro_glitch_does_not_deploy_main() {
  under_drogue(600.0f);
  for (int i = 0; i < MAIN_DEPLOY_CONFIRMATION_COUNT - 1; i++) {   // N-1 consecutive low samples...
    harness_set_baro_alt(150.0f);
    one_sample_then_spin(100, 100);
  }
  harness_set_baro_alt(600.0f);                                     // ...then normal again
  one_sample_then_spin(100, 100);
  for (int i = 0; i < MAIN_DEPLOY_CONFIRMATION_COUNT - 1; i++) {   // the count starts over
    harness_set_baro_alt(150.0f);
    one_sample_then_spin(100, 100);
  }
  TEST_ASSERT_FALSE(main_fired());
}

void test_n_consecutive_fresh_low_samples_deploy_main() {
  under_drogue(600.0f);
  harness_set_baro_alt(150.0f);
  for (int i = 0; i < MAIN_DEPLOY_CONFIRMATION_COUNT; i++) one_sample_then_spin(100, 100);
  TEST_ASSERT_TRUE(main_fired());
  harness_run_ms(PYRO_FIRE_DURATION + 300);
  TEST_ASSERT_EQUAL(MAIN_DESCENT, g_currentFlightState);
}

// ---- barometer failure -------------------------------------------------------------
// The barometer stops delivering (stale) while the vehicle falls: main must still deploy.
static void run_without_baro_samples(unsigned long ms) {
  for (unsigned long t = 0; t < ms; t += 10) {
    test_advance_ms(10);
    static unsigned long ph = 0;
    if ((++ph % 10) == 0) harness_new_accel_samples();      // accelerometers alive, baro dead
    ProcessFlightState();
  }
}

void test_dead_barometer_falls_back_to_an_estimated_descent_time() {
  under_drogue(600.0f, 500.0f);    // apogee 500 m AGL: (500-100)/25 m/s*0.8 = 12.8 s
  run_without_baro_samples(12000);
  TEST_ASSERT_FALSE_MESSAGE(main_fired(), "must not fire before the estimated descent time");
  run_without_baro_samples(1500);
  TEST_ASSERT_TRUE(main_fired());
}

void test_dead_barometer_with_unknown_apogee_uses_the_fixed_fallback_time() {
  under_drogue(600.0f, 0.0f);
  run_without_baro_samples(MAIN_DEPLOY_FALLBACK_TIME_MS - 500);
  TEST_ASSERT_FALSE(main_fired());
  run_without_baro_samples(1000);
  TEST_ASSERT_TRUE(main_fired());
}

void test_a_barometer_stuck_high_but_still_delivering_is_caught_by_the_hard_limit() {
  under_drogue(600.0f, 500.0f);    // baro keeps "delivering" 600 m forever
  for (unsigned long t = 0; t < MAIN_DEPLOY_MAX_DROGUE_TIME_MS - 1000; t += 10) {
    test_advance_ms(10);
    static unsigned long ph = 0;
    if ((++ph % 10) == 0) harness_new_samples();
    ProcessFlightState();
  }
  TEST_ASSERT_FALSE(main_fired());
  for (unsigned long t = 0; t < 2000; t += 10) {
    test_advance_ms(10);
    static unsigned long ph2 = 0;
    if ((++ph2 % 10) == 0) harness_new_samples();
    ProcessFlightState();
  }
  TEST_ASSERT_TRUE(main_fired());
}

// ---- states must not persist forever -------------------------------------------------
void test_drogue_descent_progresses_to_landed_on_touchdown_if_main_never_deployed() {
  under_drogue(100.0f, 30.0f);            // already on the ground (launch altitude), low apogee
  g_main_deploy_altitude_m_agl = 0.0f;    // deploy altitude at ground level: the baro gate never opens
  harness_set_baro_alt(100.0f);
  harness_set_accel_g(1.0f);
  isStationary = true;                    // IMU motion detector: at rest
  harness_run_ms(6000);
  TEST_ASSERT_EQUAL(LANDED, g_currentFlightState);
  TEST_ASSERT_FALSE(main_fired());        // touched down with no main: nothing left to fire
}

// Run the loop (10 Hz sensors) until `pred` holds or `limit_ms` passes; returns elapsed ms.
static unsigned long run_until_state_leaves(FlightState st, unsigned long limit_ms) {
  unsigned long t = 0;
  while (t < limit_ms && g_currentFlightState == st) {
    test_advance_ms(100);
    harness_new_samples();
    for (int i = 0; i < 10; i++) ProcessFlightState();
    t += 100;
  }
  return t;
}

// MAIN_DESCENT that never sees touchdown (baro frozen high, accel not at rest) is forced to LANDED.
void test_main_descent_times_out_to_landed() {
  harness_on_pad(100.0f);
  g_flightInProgress = true;
  g_pyroFiredMask = PYRO_FIRED_DROGUE | PYRO_FIRED_MAIN;
  g_launchAltitude = 100.0f;
  harness_set_baro_alt(400.0f);
  harness_set_accel_g(1.0f);
  g_currentFlightState = MAIN_DESCENT;
  g_previousFlightState = STARTUP;
  harness_pass();
  const unsigned long t = run_until_state_leaves(MAIN_DESCENT, DESCENT_STATE_TIMEOUT_MS + 5000);
  TEST_ASSERT_EQUAL(LANDED, g_currentFlightState);
  TEST_ASSERT_TRUE(t >= DESCENT_STATE_TIMEOUT_MS - 1000);
}

// DROGUE_DESCENT with a barometer that never crosses the deploy altitude cannot last forever:
// the hard drogue-time limit deploys main, after which the normal path continues.
void test_drogue_descent_cannot_last_forever() {
  under_drogue(600.0f, 500.0f);
  g_main_deploy_altitude_m_agl = 0.0f;   // baro gate can never trigger
  const unsigned long t = run_until_state_leaves(DROGUE_DESCENT, DESCENT_STATE_TIMEOUT_MS + 5000);
  TEST_ASSERT_TRUE(g_currentFlightState != DROGUE_DESCENT);
  TEST_ASSERT_TRUE(t <= MAIN_DEPLOY_MAX_DROGUE_TIME_MS + 1000);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_one_cached_low_reading_never_deploys_main);
  RUN_TEST(test_a_short_baro_glitch_does_not_deploy_main);
  RUN_TEST(test_n_consecutive_fresh_low_samples_deploy_main);
  RUN_TEST(test_dead_barometer_falls_back_to_an_estimated_descent_time);
  RUN_TEST(test_dead_barometer_with_unknown_apogee_uses_the_fixed_fallback_time);
  RUN_TEST(test_a_barometer_stuck_high_but_still_delivering_is_caught_by_the_hard_limit);
  RUN_TEST(test_drogue_descent_progresses_to_landed_on_touchdown_if_main_never_deployed);
  RUN_TEST(test_main_descent_times_out_to_landed);
  RUN_TEST(test_drogue_descent_cannot_last_forever);
  return UNITY_END();
}
