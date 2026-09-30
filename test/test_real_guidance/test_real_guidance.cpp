// Audit #12 - guidance only steers in COAST (fins centred in the descent states) and its first dt
// is initialised on entry. Exercises the REAL flightGuidanceStep() in flight_logic.cpp.
#include <unity.h>
#include "../support/flight_harness.h"
#include "../support/real_sources.h"

void setUp() { harness_reset(); }
void tearDown() {}

// ---- which states may steer -------------------------------------------------------------------
void test_only_coast_runs_guidance() {
  for (int s = STARTUP; s <= ERROR; s++) {
    harness_reset();
    g_currentFlightState = (FlightState)s;
    float dt = 0;
    bool ran = false;
    for (int i = 0; i < 100; i++) { test_advance_ms(20); ran = ran || flightGuidanceStep(millis(), false, dt); }
    char msg[32]; snprintf(msg, sizeof msg, "state=%s", getStateName((FlightState)s));
    TEST_ASSERT_EQUAL_MESSAGE(s == COAST, ran, msg);
  }
}

void test_guidance_does_not_run_when_disabled_or_stationary() {
  g_currentFlightState = COAST;
  float dt;
  g_guidance_active = false;
  for (int i = 0; i < 50; i++) { test_advance_ms(20); TEST_ASSERT_FALSE(flightGuidanceStep(millis(), false, dt)); }
  g_guidance_active = true;
  for (int i = 0; i < 50; i++) { test_advance_ms(20); TEST_ASSERT_FALSE(flightGuidanceStep(millis(), /*stationary=*/true, dt)); }
  bool ran = false;
  for (int i = 0; i < 50; i++) { test_advance_ms(20); ran = ran || flightGuidanceStep(millis(), false, dt); }
  TEST_ASSERT_TRUE(ran);
}

// ---- fins are centred (once), never steered, after COAST ----------------------------------------
void test_descent_states_centre_the_fins_once_and_never_run_guidance() {
  const FlightState after[] = {APOGEE, DROGUE_DEPLOY, DROGUE_DESCENT, MAIN_DEPLOY, MAIN_DESCENT, LANDED, RECOVERY};
  for (FlightState st : after) {
    harness_reset();
    g_currentFlightState = COAST;               // guidance was running in COAST...
    float dt;
    for (int i = 0; i < 20; i++) { test_advance_ms(20); flightGuidanceStep(millis(), false, dt); }
    TEST_ASSERT_TRUE(g_test_guidance_center_calls == 0);
    g_currentFlightState = st;                  // ...then the vehicle moves on
    for (int i = 0; i < 1000; i++) {
      test_advance_ms(20);
      TEST_ASSERT_FALSE_MESSAGE(flightGuidanceStep(millis(), false, dt), getStateName(st));
    }
    TEST_ASSERT_EQUAL_MESSAGE(1, g_test_guidance_center_calls, getStateName(st));   // centred exactly once
  }
}

void test_disabled_guidance_in_coast_centres_the_fins_once() {
  g_currentFlightState = COAST;
  g_guidance_active = false;
  float dt;
  for (int i = 0; i < 200; i++) { test_advance_ms(20); flightGuidanceStep(millis(), false, dt); }
  TEST_ASSERT_EQUAL(1, g_test_guidance_center_calls);
}

// ---- dt ---------------------------------------------------------------------------------------------
void test_first_guidance_dt_is_the_nominal_interval_not_millis_minus_zero() {
  test_set_ms(1234567);                          // 20 minutes of uptime
  g_currentFlightState = COAST;
  float dt = -1;
  TEST_ASSERT_TRUE(flightGuidanceStep(millis(), false, dt));
  TEST_ASSERT_EQUAL_FLOAT(GUIDANCE_UPDATE_INTERVAL_MS / 1000.0f, dt);   // the old code produced 1234.567 s
  test_advance_ms(GUIDANCE_UPDATE_INTERVAL_MS);
  TEST_ASSERT_TRUE(flightGuidanceStep(millis(), false, dt));
  TEST_ASSERT_EQUAL_FLOAT(GUIDANCE_UPDATE_INTERVAL_MS / 1000.0f, dt);
  test_advance_ms(35);
  TEST_ASSERT_TRUE(flightGuidanceStep(millis(), false, dt));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.035f, dt);
  test_advance_ms(5);
  TEST_ASSERT_FALSE(flightGuidanceStep(millis(), false, dt));           // not yet due
}

void test_a_stalled_loop_does_not_produce_a_huge_dt_and_reentry_reprimes_the_timer() {
  g_currentFlightState = COAST;
  float dt;
  flightGuidanceStep(millis(), false, dt);
  test_advance_ms(3000);                          // loop stalled for 3 s
  TEST_ASSERT_TRUE(flightGuidanceStep(millis(), false, dt));
  TEST_ASSERT_EQUAL_FLOAT(GUIDANCE_MAX_DT_S, dt);
  // leave the running condition (stationary), come back much later: first dt is nominal again
  test_advance_ms(10);
  flightGuidanceStep(millis(), true, dt);
  test_advance_ms(60000);
  TEST_ASSERT_TRUE(flightGuidanceStep(millis(), false, dt));
  TEST_ASSERT_EQUAL_FLOAT(GUIDANCE_UPDATE_INTERVAL_MS / 1000.0f, dt);
}

// ---- through the real state machine ---------------------------------------------------------------------
void test_guidance_runs_in_coast_then_fins_are_centred_when_the_drogue_deploys() {
  harness_on_pad(100.0f);
  g_flightInProgress = true;
  g_launchAltitude = 100.0f;
  g_currentFlightState = COAST; g_previousFlightState = COAST;
  boostEndTime = millis();
  harness_set_accel_g(0.8f);
  harness_ramp_baro(400.0f, 4000);                                     // climbing coast: guidance steering
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  TEST_ASSERT_TRUE(g_test_guidance_run_steps > 100);                   // ~50 Hz for ~4 s
  TEST_ASSERT_EQUAL(0, g_test_guidance_center_calls);
  const int steps_in_coast = g_test_guidance_run_steps;
  harness_run_ms(BACKUP_APOGEE_TIME_MS);                               // apogee by the backup timer, drogue fires...
  harness_run_ms(3000);
  TEST_ASSERT_TRUE(g_currentFlightState >= APOGEE);
  const int after = g_test_guidance_run_steps;
  harness_run_ms(20000);                                               // long drogue/main descent
  TEST_ASSERT_EQUAL(after, g_test_guidance_run_steps);                 // no guidance step after COAST ended
  TEST_ASSERT_TRUE(g_test_guidance_run_steps >= steps_in_coast);
  TEST_ASSERT_EQUAL(1, g_test_guidance_center_calls);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_only_coast_runs_guidance);
  RUN_TEST(test_guidance_does_not_run_when_disabled_or_stationary);
  RUN_TEST(test_descent_states_centre_the_fins_once_and_never_run_guidance);
  RUN_TEST(test_disabled_guidance_in_coast_centres_the_fins_once);
  RUN_TEST(test_first_guidance_dt_is_the_nominal_interval_not_millis_minus_zero);
  RUN_TEST(test_a_stalled_loop_does_not_produce_a_huge_dt_and_reentry_reprimes_the_timer);
  RUN_TEST(test_guidance_runs_in_coast_then_fins_are_centred_when_the_drogue_deploys);
  return UNITY_END();
}
