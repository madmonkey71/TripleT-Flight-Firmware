// Tests that exercise the REAL src/flight_logic.cpp (not a re-implementation).
// See test/support/flight_harness.h for what is faked at the hardware boundary.
#include <unity.h>
#include "../support/flight_harness.h"
#include "../support/real_sources.h"

void setUp() { harness_reset(); }
void tearDown() {}

// Characterisation of the nominal flight walk through the real state machine.
// Guards the behaviour-preserving refactor of ProcessFlightState (statics moved
// into a resettable runtime struct) and doubles as a smoke test of the harness.
void test_real_nominal_flight_walk() {
  harness_on_pad(100.0f);
  TEST_ASSERT_EQUAL(PAD_IDLE, g_currentFlightState);
  TEST_ASSERT_EQUAL_FLOAT(100.0f, g_launchAltitude);

  g_currentFlightState = ARMED;              // what the `arm` command does
  harness_pass();
  TEST_ASSERT_EQUAL(ARMED, g_currentFlightState);

  harness_set_accel_g(4.0f);                 // liftoff
  harness_run_ms(LAUNCH_CONFIRMATION_COUNT * 100 + 100);   // needs N fresh samples (audit #8)
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
  harness_ramp_baro(160.0f, 1000);           // powered ascent

  harness_set_accel_g(0.1f);                 // burnout
  harness_run_ms(500);                       // 3+ fresh samples below the threshold
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);

  harness_set_accel_g(0.8f);
  harness_ramp_baro(600.0f, 6000);           // coast up to apex (600 m)
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  harness_ramp_baro(590.0f, 1000);           // now clearly descending
  harness_run_ms(500);
  TEST_ASSERT_TRUE(g_currentFlightState >= APOGEE);

  harness_run_ms(PYRO_FIRE_DURATION + 100);  // drogue fire window
  TEST_ASSERT_EQUAL(DROGUE_DESCENT, g_currentFlightState);
  TEST_ASSERT_TRUE(g_pin_ever_high[PYRO_CHANNEL_1]);
  TEST_ASSERT_EQUAL(LOW, g_pin_level[PYRO_CHANNEL_1]);

  harness_ramp_baro(150.0f, 4000);           // below main deploy altitude (launch + 100 m)
  harness_run_ms(PYRO_FIRE_DURATION + 300);
  TEST_ASSERT_EQUAL(MAIN_DESCENT, g_currentFlightState);
  TEST_ASSERT_TRUE(g_pin_ever_high[PYRO_CHANNEL_2]);
}

// ---- helpers ---------------------------------------------------------------
static const FlightState kAirborneStates[] = {BOOST, COAST, APOGEE, DROGUE_DEPLOY, DROGUE_DESCENT, MAIN_DEPLOY, MAIN_DESCENT};

// Put the vehicle in `st` mid-flight (flight flag set, entry actions already done).
static void set_airborne(FlightState st, float alt = 400.0f) {
  harness_on_pad(100.0f);
  g_flightInProgress = true;
  g_launchAltitude = 100.0f;
  g_maxAltitudeReached = 500.0f;
  g_main_deploy_altitude_m_agl = MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M;
  harness_set_baro_alt(alt);
  g_currentFlightState = st;
  g_previousFlightState = st;
  g_stateEntryTime = millis();
  if (st == COAST || st == BOOST) boostEndTime = millis();
}
static bool any_pyro_high() { return g_pin_ever_high[PYRO_CHANNEL_1] || g_pin_ever_high[PYRO_CHANNEL_2]; }

// ---- audit #2: ERROR mid-flight ---------------------------------------------
// A sensor-health failure in any airborne state must never leave the flight state
// machine for ERROR (which stops apogee / backup-timer / main-deploy logic).
void test_sensor_health_failure_in_airborne_states_never_enters_error() {
  for (FlightState st : kAirborneStates) {
    harness_reset();
    set_airborne(st);
    g_test_sensors_healthy = false;
    for (int i = 0; i < 1000; i++) {   // 10 s: several health-check intervals
      test_advance_ms(10);
      harness_pass();
      char msg[48]; snprintf(msg, sizeof msg, "started in %s", getStateName(st));
      TEST_ASSERT_NOT_EQUAL_MESSAGE(ERROR, g_currentFlightState, msg);
    }
    TEST_ASSERT_TRUE_MESSAGE(flightIsDegraded(), getStateName(st));
    TEST_ASSERT_FALSE_MESSAGE(g_guidance_active, getStateName(st));            // guidance disabled
    TEST_ASSERT_TRUE_MESSAGE(g_test_guidance_center_calls > 0, getStateName(st)); // fins centred
    TEST_ASSERT_EQUAL_MESSAGE(NEOPIXEL_COUNT ? Adafruit_NeoPixel::Color(255, 165, 0) : 0, g_pixels.color[0], getStateName(st)); // degraded LED
    TEST_ASSERT_EQUAL_MESSAGE(STATE_TRANSITION_INVALID_HEALTH, g_last_error_code, getStateName(st)); // logged
  }
}

// The deployment paths must still run while degraded: the backup apogee timer fires
// the drogue even with the sensor suite reporting unhealthy.
void test_backup_timer_still_deploys_drogue_while_degraded() {
  set_airborne(COAST);
  g_test_sensors_healthy = false;
  harness_run_ms(BACKUP_APOGEE_TIME_MS + 4000);
  TEST_ASSERT_TRUE(g_pin_ever_high[PYRO_CHANNEL_1]);
  TEST_ASSERT_NOT_EQUAL(ERROR, g_currentFlightState);
  TEST_ASSERT_TRUE(g_currentFlightState >= DROGUE_DEPLOY);
}

// BOOST entry with the ICM not ready used to route to ERROR. It must degrade.
void test_boost_with_icm_not_ready_degrades_and_still_reaches_coast() {
  harness_on_pad(100.0f);
  g_currentFlightState = ARMED;
  harness_pass();
  g_icm20948_ready = false;               // the ICM is gone at liftoff (KX134 still works)
  harness_set_accel_g(4.0f);
  harness_run_ms(600);
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
  TEST_ASSERT_FALSE(g_guidance_active);
  TEST_ASSERT_TRUE(flightIsDegraded());
  harness_set_accel_g(0.1f);
  harness_run_ms(600);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
}

// Pre-flight behaviour is unchanged: ERROR is still the response on the pad.
void test_pre_flight_health_failure_still_enters_error() {
  const FlightState pre[] = {CALIBRATION, PAD_IDLE, ARMED};
  for (FlightState st : pre) {
    harness_reset();
    harness_on_pad(100.0f);
    g_currentFlightState = st;
    g_previousFlightState = st;
    g_test_sensors_healthy = false;
    harness_run_ms(3000);
    TEST_ASSERT_EQUAL_MESSAGE(ERROR, g_currentFlightState, getStateName(st));
  }
}

void test_flight_in_progress_flag_blocks_error_from_any_state() {
  TEST_ASSERT_TRUE(flight_error_allowed(PAD_IDLE));
  g_flightInProgress = true;
  for (int s = STARTUP; s <= ERROR; s++) TEST_ASSERT_FALSE(flight_error_allowed((FlightState)s));
}

void test_unknown_state_value_goes_to_error_on_pad_but_recovery_in_flight() {
  harness_on_pad(100.0f);
  g_currentFlightState = (FlightState)99;
  g_previousFlightState = (FlightState)99;
  harness_pass();
  TEST_ASSERT_EQUAL(ERROR, g_currentFlightState);

  harness_reset();
  harness_on_pad(100.0f);
  g_flightInProgress = true;
  g_currentFlightState = (FlightState)99;
  g_previousFlightState = (FlightState)99;
  harness_pass();
  TEST_ASSERT_EQUAL(RECOVERY, g_currentFlightState);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_real_nominal_flight_walk);
  RUN_TEST(test_sensor_health_failure_in_airborne_states_never_enters_error);
  RUN_TEST(test_backup_timer_still_deploys_drogue_while_degraded);
  RUN_TEST(test_boost_with_icm_not_ready_degrades_and_still_reaches_coast);
  RUN_TEST(test_pre_flight_health_failure_still_enters_error);
  RUN_TEST(test_flight_in_progress_flag_blocks_error_from_any_state);
  RUN_TEST(test_unknown_state_value_goes_to_error_on_pad_but_recovery_in_flight);
  return UNITY_END();
}
