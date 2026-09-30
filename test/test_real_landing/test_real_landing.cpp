// Audit #11 - averaged launch altitude, stationarity-based landing detection on fresh samples,
// and no state leaking from one flight to the next. Exercises the REAL flight_logic.cpp.
#include <unity.h>
#include <string>
#include "../support/flight_harness.h"
#include "../support/real_sources.h"

void setUp() { harness_reset(); }
void tearDown() {}

// Deterministic pseudo-random noise in [-1, 1].
static unsigned s_rng = 12345;
static float noise() { s_rng = s_rng * 1664525u + 1013904223u; return ((s_rng >> 8) & 0xFFFF) / 32768.0f - 1.0f; }

// ---- launch altitude average -----------------------------------------------------------------
// One noisy sample used to become the ground reference for the whole flight.
void test_launch_altitude_is_the_average_of_the_pad_samples_not_one_noisy_reading() {
  harness_on_pad(100.9f);                              // the sample taken at PAD_IDLE entry is 0.9 m off the true 100 m
  float worst_single = 0.0f;
  for (int i = 0; i < 60; i++) {                       // 6 s on the pad, +-1 m of baro noise
    const float n = noise();
    if (fabsf(n) > worst_single) worst_single = fabsf(n);
    harness_set_baro_alt(100.0f + n);
    harness_run_ms(100);
  }
  TEST_ASSERT_TRUE(worst_single > 0.7f);                       // the raw samples really are that noisy
  TEST_ASSERT_FLOAT_WITHIN(0.3f, 100.0f, g_launchAltitude);    // the reference is not
}

void test_arming_rezeroes_from_the_average_and_then_freezes_the_reference() {
  harness_on_pad(100.0f);
  for (int i = 0; i < 40; i++) {                       // weather drift on the pad: 100 m -> 104 m over 4 s
    harness_set_baro_alt(100.0f + 4.0f * i / 40.0f + 0.2f * noise());
    harness_run_ms(100);
  }
  const float alt_now = harness_baro_alt();
  g_currentFlightState = ARMED;                        // `arm`
  harness_pass();
  TEST_ASSERT_EQUAL(ARMED, g_currentFlightState);
  // mean of the last 20 samples (~2 s) sits ~1 m below the newest sample, well above the entry-time reading
  TEST_ASSERT_FLOAT_WITHIN(0.6f, alt_now - 1.0f, g_launchAltitude);
  TEST_ASSERT_TRUE(g_launchAltitude > 101.5f);                 // it followed the drift (a stale 100.0 would not)
  TEST_ASSERT_FLOAT_WITHIN(2.5f, MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M, g_main_deploy_altitude_m_agl);

  const float armed_ref = g_launchAltitude;
  harness_set_baro_alt(alt_now + 5.0f);                // more drift after arming
  harness_run_ms(5000);
  TEST_ASSERT_EQUAL_FLOAT(armed_ref, g_launchAltitude);        // frozen while armed
}

// ---- landing ----------------------------------------------------------------------------------------
static void descending_under_main(float alt, float accel_g = 1.0f) {
  harness_on_pad(100.0f);
  g_flightInProgress = true;
  g_pyroFiredMask = PYRO_FIRED_DROGUE | PYRO_FIRED_MAIN;
  g_launchAltitude = 100.0f;
  harness_set_baro_alt(alt);
  harness_set_accel_g(accel_g);
  g_currentFlightState = MAIN_DESCENT;
  g_previousFlightState = STARTUP;       // run the entry actions
  harness_pass();
}

// The old test required |average altitude - launch altitude| < 1 m: a vehicle that lands on a hill
// 60 m above the pad never "landed".
void test_landing_is_detected_by_stationarity_wherever_it_lands() {
  const float landing_alts[] = {160.0f, 100.0f, 40.0f, -20.0f};     // above / at / below the pad elevation
  for (float alt : landing_alts) {
    harness_reset();
    descending_under_main(alt + 20.0f);
    harness_ramp_baro(alt, 2000);                     // final 20 m at 10 m/s
    TEST_ASSERT_EQUAL(MAIN_DESCENT, g_currentFlightState);
    isStationary = true;                              // touchdown: IMU at rest
    const unsigned long t0 = millis();
    harness_run_ms(5000);
    TEST_ASSERT_EQUAL_MESSAGE(LANDED, g_currentFlightState, "landing away from the pad elevation");
    (void)t0;
  }
}

void test_landing_needs_the_window_and_the_confirmation_time_not_an_instant() {
  descending_under_main(160.0f);
  isStationary = true;
  const unsigned long t0 = millis();
  harness_run_ms(1000);
  TEST_ASSERT_EQUAL(MAIN_DESCENT, g_currentFlightState);
  harness_run_ms(LANDING_CONFIRMATION_TIME_MS + 1500);
  TEST_ASSERT_EQUAL(LANDED, g_currentFlightState);
  TEST_ASSERT_TRUE(millis() - t0 >= LANDING_CONFIRMATION_TIME_MS);
}

// ~1 g is also what a steady canopy descent reads; the baro window is what tells them apart.
void test_steady_descent_under_canopy_at_1g_is_not_landing() {
  descending_under_main(1200.0f);
  isStationary = true;                                // even if the IMU happened to look quiet
  for (int i = 0; i < 300; i++) {                     // 30 s at 5 m/s
    harness_set_baro_alt(harness_baro_alt() - 0.5f);
    harness_run_ms(100);
  }
  TEST_ASSERT_EQUAL(MAIN_DESCENT, g_currentFlightState);
}

// A barometer FROZEN at one value (still "delivering") looks stationary; the IMU motion detector
// (swinging under the canopy) must veto it.
void test_frozen_baro_under_a_swinging_canopy_is_not_landing() {
  descending_under_main(500.0f);
  isStationary = false;                               // gyro shows the swing
  harness_run_ms(60000);
  TEST_ASSERT_EQUAL(MAIN_DESCENT, g_currentFlightState);
}

void test_landing_confirmation_must_be_consecutive() {
  descending_under_main(160.0f);
  isStationary = true;
  harness_run_ms(1600);                               // most of the way there
  TEST_ASSERT_EQUAL(MAIN_DESCENT, g_currentFlightState);
  harness_set_accel_g(1.6f);                          // a bump: accelerometer leaves the 1 g band for a sample
  harness_run_ms(100);
  harness_set_accel_g(1.0f);
  harness_run_ms(1500);                               // 1.6 + 1.5 s stationary in total, but NOT consecutive
  TEST_ASSERT_EQUAL_MESSAGE(MAIN_DESCENT, g_currentFlightState, "the timer must restart after the interruption");
  harness_run_ms(2500);
  TEST_ASSERT_EQUAL(LANDED, g_currentFlightState);
}

void test_landing_needs_a_fresh_barometer() {
  descending_under_main(160.0f);
  isStationary = true;
  for (unsigned long t = 0; t < 10000; t += 10) {     // barometer stops delivering entirely
    test_advance_ms(10);
    static unsigned long ph = 0;
    if ((++ph % 10) == 0) harness_new_accel_samples();
    harness_pass();
  }
  TEST_ASSERT_EQUAL(MAIN_DESCENT, g_currentFlightState);   // no proof of stationarity => no landing (the timeout covers it)
}

// ---- state must not leak between flights ------------------------------------------------------------
static void run_cmd_reset_flight_from_recovery() {
  bool baro = g_baroCalibrated;
  Serial.clear();
  handleFlightStateCommand("reset_flight", g_currentFlightState, g_previousFlightState, g_stateEntryTime, baro, true);
  const std::string& out = Serial.out;
  size_t p = out.rfind("reset_flight ");
  TEST_ASSERT_TRUE(p != std::string::npos);
  char buf[40]; snprintf(buf, sizeof buf, "reset_flight %d", atoi(out.c_str() + p + strlen("reset_flight ")));
  handleFlightStateCommand(buf, g_currentFlightState, g_previousFlightState, g_stateEntryTime, baro, true);
}

// Fly a complete nominal flight from PAD_IDLE to RECOVERY; returns the time from liftoff to apogee detection.
static unsigned long fly_full_flight() {
  TEST_ASSERT_EQUAL(PAD_IDLE, g_currentFlightState);
  for (int i = 0; i < 30; i++) { harness_set_baro_alt(100.0f + 0.2f * noise()); harness_run_ms(100); }   // 3 s on the pad
  g_currentFlightState = ARMED; harness_pass();
  harness_set_accel_g(5.0f);
  harness_run_ms(LAUNCH_CONFIRMATION_COUNT * 100 + 100);
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
  const unsigned long t_boost = millis();
  harness_ramp_baro(160.0f, 1000);
  harness_set_accel_g(0.1f); harness_run_ms(500);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  harness_set_accel_g(0.8f);
  harness_ramp_baro(600.0f, 6000);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  for (int i = 0; i < 40 && g_currentFlightState < APOGEE; i++) harness_ramp_baro(harness_baro_alt() - 0.5f, 100);
  TEST_ASSERT_TRUE(g_currentFlightState >= APOGEE);
  const unsigned long apogee_after = millis() - t_boost;
  harness_run_ms(PYRO_FIRE_DURATION + 200);
  TEST_ASSERT_EQUAL(DROGUE_DESCENT, g_currentFlightState);
  harness_ramp_baro(190.0f, 8000);
  harness_run_ms(PYRO_FIRE_DURATION + 500);
  TEST_ASSERT_EQUAL(MAIN_DESCENT, g_currentFlightState);
  harness_set_accel_g(1.0f);
  harness_ramp_baro(103.0f, 3000);
  isStationary = true;
  harness_run_ms(6000);
  TEST_ASSERT_EQUAL(LANDED, g_currentFlightState);
  harness_run_ms(LANDED_TIMEOUT_MS + 500);
  TEST_ASSERT_EQUAL(RECOVERY, g_currentFlightState);
  return apogee_after;
}

void test_a_second_flight_behaves_exactly_like_the_first() {
  harness_on_pad(100.0f);
  const unsigned long first = fly_full_flight();
  TEST_ASSERT_TRUE(g_pin_ever_high[PYRO_CHANNEL_1] && g_pin_ever_high[PYRO_CHANNEL_2]);

  // operator recovers the vehicle: at rest, then reset_flight (the vehicle sits on the pad in this model)
  isStationary = true;
  harness_run_ms(2000);
  run_cmd_reset_flight_from_recovery();
  TEST_ASSERT_EQUAL(PAD_IDLE, g_currentFlightState);
  TEST_ASSERT_FALSE(g_flightInProgress);
  TEST_ASSERT_EQUAL(0, g_pyroFiredMask);

  test_pins_reset();
  isStationary = false;
  harness_set_baro_alt(100.0f);
  const unsigned long second = fly_full_flight();
  TEST_ASSERT_TRUE(g_pin_ever_high[PYRO_CHANNEL_1] && g_pin_ever_high[PYRO_CHANNEL_2]);   // both fire again
  // the same profile is detected at the same time (no counter/timer carried over from flight 1)
  TEST_ASSERT_TRUE(second + 300 >= first && first + 300 >= second);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_launch_altitude_is_the_average_of_the_pad_samples_not_one_noisy_reading);
  RUN_TEST(test_arming_rezeroes_from_the_average_and_then_freezes_the_reference);
  RUN_TEST(test_landing_is_detected_by_stationarity_wherever_it_lands);
  RUN_TEST(test_landing_needs_the_window_and_the_confirmation_time_not_an_instant);
  RUN_TEST(test_steady_descent_under_canopy_at_1g_is_not_landing);
  RUN_TEST(test_frozen_baro_under_a_swinging_canopy_is_not_landing);
  RUN_TEST(test_landing_confirmation_must_be_consecutive);
  RUN_TEST(test_landing_needs_a_fresh_barometer);
  RUN_TEST(test_a_second_flight_behaves_exactly_like_the_first);
  return UNITY_END();
}
