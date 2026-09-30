// Audit #4 - apogee / burnout detection must confirm on FRESH sensor samples, use a
// mounting-independent free-fall test, respect the burnout and transonic gates, and
// cross-check each fast path. Exercises the REAL flight_logic.cpp detectors.
#include <unity.h>
#include "../support/flight_harness.h"
#include "../support/real_sources.h"

void setUp() { harness_reset(); }
void tearDown() {}

// ---- helpers ---------------------------------------------------------------
// Vehicle in COAST at `alt` (absolute), launched from 100 m, burnout "now".
static void coast_at(float alt, float accel_g = 0.8f) {
  harness_on_pad(100.0f);
  g_flightInProgress = true;
  g_launchAltitude = 100.0f;
  g_main_deploy_altitude_m_agl = MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M;
  harness_set_baro_alt(alt);
  harness_set_accel_g(accel_g);
  g_currentFlightState = COAST;
  g_previousFlightState = COAST;
  g_stateEntryTime = millis();
  boostEndTime = millis();
  harness_run_ms(10);
}
// Let the burnout/transonic gates expire with the vehicle hanging at a steady altitude.
static void settle_past_gates() { harness_run_ms(APOGEE_BARO_TRANSONIC_LOCKOUT_MS + 500); }
static bool apogee_reached() { return g_currentFlightState >= APOGEE; }

// Deliver one fresh sample of everything, then hammer the loop `passes` times WITHOUT
// advancing time or the sensors - what the real main loop does between sensor updates.
static void one_sample_then_spin(unsigned long dt_ms, int passes) {
  test_advance_ms(dt_ms);
  harness_new_samples();
  for (int i = 0; i < passes; i++) ProcessFlightState();
}

// ---- fresh-sample confirmation ------------------------------------------------
// The audit's core defect: a "5 consecutive readings" count was satisfied by ONE
// cached sample re-read 5 times within a millisecond.
void test_baro_apogee_needs_five_fresh_samples_not_five_loop_passes() {
  coast_at(400.0f);
  settle_past_gates();
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);

  harness_set_baro_alt(390.0f);              // 10 m below the reference: clearly "descending"
  for (int i = 0; i < 5000; i++) ProcessFlightState();   // thousands of passes, ONE cached sample
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);

  for (int k = 1; k <= APOGEE_CONFIRMATION_COUNT - 1; k++) {
    one_sample_then_spin(100, 200);          // k fresh samples, each re-read 200 times
    TEST_ASSERT_EQUAL_MESSAGE(COAST, g_currentFlightState, "fewer than N fresh samples must not confirm");
  }
  one_sample_then_spin(100, 200);            // the N-th fresh sample
  TEST_ASSERT_TRUE(apogee_reached());
}

void test_gps_apogee_counts_consecutive_fresh_samples_only() {
  g_baroCalibrated = false;                  // GPS is the only altitude source here
  coast_at(400.0f);
  g_baroCalibrated = false;
  GPS_fixType = 3;
  g_test_gps_alt_m = 500.0f;
  harness_run_ms(APOGEE_MIN_TIME_AFTER_BURNOUT_MS + 500);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);

  // Two samples 10 m down, a recovery, two more down: the old code accumulated 4 and fired.
  g_test_gps_alt_m = 490.0f; harness_run_ms(200);
  g_test_gps_alt_m = 500.0f; harness_run_ms(100);
  g_test_gps_alt_m = 490.0f; harness_run_ms(200);
  TEST_ASSERT_EQUAL_MESSAGE(COAST, g_currentFlightState, "non-consecutive GPS counts must not accumulate");

  g_test_gps_alt_m = 490.0f;
  harness_run_ms(APOGEE_GPS_CONFIRMATION_COUNT * 100 + 200);   // consecutive fresh samples
  TEST_ASSERT_TRUE(apogee_reached());
}

void test_burnout_needs_fresh_samples_not_loop_passes() {
  harness_on_pad(100.0f);
  g_flightInProgress = true;
  g_currentFlightState = BOOST; g_previousFlightState = BOOST;
  harness_set_accel_g(4.0f);
  harness_run_ms(200);
  harness_set_accel_g(0.1f);                 // motor out
  for (int i = 0; i < 5000; i++) ProcessFlightState();   // one cached sample only
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
  for (int k = 1; k < COAST_CONFIRMATION_COUNT; k++) {
    one_sample_then_spin(100, 200);
    TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
  }
  one_sample_then_spin(100, 200);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  TEST_ASSERT_TRUE(boostEndTime > 0);
}

void test_dead_accelerometer_does_not_fake_a_burnout() {
  harness_on_pad(100.0f);
  g_flightInProgress = true;
  g_currentFlightState = BOOST; g_previousFlightState = BOOST;
  for (int i = 0; i < 3; i++) { icm_accel[i] = 0; kx134_accel[i] = 0; }   // no data at all
  harness_run_ms(5000);
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);   // (item 8 adds the BOOST timeout that eventually leaves BOOST)
}

// ---- free-fall accelerometer method ---------------------------------------------
// The old test was `icm_accel[2] < 0`: true at 1 g if the IMU is mounted upside-down.
void test_upside_down_imu_at_1g_does_not_trigger_the_accel_method() {
  coast_at(400.0f);
  icm_accel[0] = 0; icm_accel[1] = 0; icm_accel[2] = -1.2f;
  kx134_accel[0] = 0; kx134_accel[1] = 0; kx134_accel[2] = -1.2f;
  harness_run_ms(8000);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
}

void test_free_fall_triggers_after_min_time_and_window_whatever_the_axis() {
  const float dirs[3][3] = {{0, 0, -0.05f}, {0.04f, 0, 0}, {0, -0.03f, 0.03f}};
  for (auto& d : dirs) {
    harness_reset();
    coast_at(400.0f);
    for (int i = 0; i < 3; i++) { icm_accel[i] = d[i]; kx134_accel[i] = d[i]; }
    const unsigned long t0 = millis();
    harness_run_ms(APOGEE_MIN_TIME_AFTER_BURNOUT_MS - 200);
    TEST_ASSERT_EQUAL_MESSAGE(COAST, g_currentFlightState, "must not fire before the min time after burnout");
    harness_run_ms(700);
    TEST_ASSERT_TRUE(apogee_reached());
    TEST_ASSERT_TRUE(millis() - t0 >= APOGEE_MIN_TIME_AFTER_BURNOUT_MS);
  }
}

void test_a_baro_still_climbing_vetoes_the_free_fall_method() {
  coast_at(300.0f);
  icm_accel[2] = 0.05f; kx134_accel[2] = 0.05f;          // looks like free fall...
  harness_ramp_baro(450.0f, 5000);                       // ...but the baro says +30 m/s
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  harness_run_ms(2500);                                  // climb stops: vertical speed -> 0
  TEST_ASSERT_TRUE(apogee_reached());
}

void test_free_fall_without_a_baro_needs_the_longer_window() {
  coast_at(400.0f);
  g_baroCalibrated = false;                              // no corroborating barometer
  icm_accel[2] = 0.05f; kx134_accel[2] = 0.05f;
  const unsigned long t0 = millis();
  harness_run_ms(APOGEE_ACCEL_FREEFALL_WINDOW_NO_BARO_MS - 200);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  harness_run_ms(800);
  TEST_ASSERT_TRUE(apogee_reached());
  TEST_ASSERT_TRUE(millis() - t0 >= APOGEE_ACCEL_FREEFALL_WINDOW_NO_BARO_MS);
}

// ---- transonic lockout ------------------------------------------------------------
void test_transonic_spike_during_lockout_cannot_poison_the_descent_reference() {
  coast_at(200.0f);                                       // AGL 100
  harness_ramp_baro(230.0f, 900);
  harness_set_baro_alt(285.0f); harness_run_ms(300);      // shock-induced false altitude spike (+55 m)
  harness_ramp_baro(240.0f, 100);
  TEST_ASSERT_TRUE(millis() > 0);
  harness_ramp_baro(330.0f, 6000);                        // real climb continues past the spike's height
  TEST_ASSERT_EQUAL_MESSAGE(COAST, g_currentFlightState, "the spike must not be mistaken for the peak");
  harness_ramp_baro(300.0f, 3000);                        // genuine descent from 330 m
  harness_run_ms(1000);
  TEST_ASSERT_TRUE(apogee_reached());
}

void test_baro_method_is_locked_out_right_after_burnout() {
  coast_at(400.0f);
  harness_ramp_baro(380.0f, 1500);                        // "descending" inside the lockout window
  harness_run_ms(1000);                                   // still inside 3 s lockout
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
}

// ---- independent cross-checks -------------------------------------------------------
void test_high_specific_force_vetoes_the_baro_method() {
  coast_at(400.0f, 3.0f);                                 // still decelerating hard (3 g drag)
  settle_past_gates();
  harness_set_baro_alt(385.0f);                           // baro says "descending"
  harness_run_ms(3000);
  TEST_ASSERT_EQUAL_MESSAGE(COAST, g_currentFlightState, "accelerometer contradicts the barometer");
  harness_set_accel_g(0.8f);                              // drag has died away: corroborated
  harness_run_ms(1200);
  TEST_ASSERT_TRUE(apogee_reached());
}

void test_a_baro_still_climbing_vetoes_the_gps_method() {
  coast_at(300.0f);
  GPS_fixType = 3;
  g_test_gps_alt_m = 500.0f;
  harness_run_ms(APOGEE_MIN_TIME_AFTER_BURNOUT_MS + 500);
  g_test_gps_alt_m = 480.0f;                              // GPS glitch says "20 m lower"
  harness_ramp_baro(500.0f, 3000);                        // baro: strong climb
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
}

void test_sensor_methods_are_gated_by_minimum_altitude_gain() {
  coast_at(105.0f);                                       // only 5 m AGL: below APOGEE_MIN_ALTITUDE_GAIN_M
  settle_past_gates();
  harness_set_baro_alt(103.0f);
  icm_accel[2] = 0.05f; kx134_accel[2] = 0.05f;
  harness_run_ms(3000);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
}

// ---- backup timer ----------------------------------------------------------------------
void test_backup_timer_fires_ungated_with_no_working_sensors() {
  coast_at(400.0f);
  g_baroCalibrated = false;
  for (int i = 0; i < 3; i++) { icm_accel[i] = 0; kx134_accel[i] = 0; }
  harness_run_ms(BACKUP_APOGEE_TIME_MS - 200);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  harness_run_ms(400);
  TEST_ASSERT_TRUE(apogee_reached());
}

// ---- happy path ----------------------------------------------------------------------------
void test_nominal_baro_apogee_is_detected_promptly_after_the_peak() {
  coast_at(200.0f);
  harness_ramp_baro(500.0f, 6000);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  const unsigned long t_peak = millis();
  for (int i = 0; i < 33 && !apogee_reached(); i++) {     // ~3 m/s descent, 100 ms slices
    harness_ramp_baro(harness_baro_alt() - 0.3f, 100);
  }
  TEST_ASSERT_TRUE(apogee_reached());
  // 1 m of descent (~330 ms) + N fresh samples (~500 ms) + margin
  TEST_ASSERT_TRUE_MESSAGE(millis() - t_peak < 1500, "detection latency too long");
  TEST_ASSERT_TRUE_MESSAGE(millis() - t_peak >= APOGEE_CONFIRMATION_COUNT * 100 - 100, "cannot beat N sensor periods");
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_baro_apogee_needs_five_fresh_samples_not_five_loop_passes);
  RUN_TEST(test_gps_apogee_counts_consecutive_fresh_samples_only);
  RUN_TEST(test_burnout_needs_fresh_samples_not_loop_passes);
  RUN_TEST(test_dead_accelerometer_does_not_fake_a_burnout);
  RUN_TEST(test_upside_down_imu_at_1g_does_not_trigger_the_accel_method);
  RUN_TEST(test_free_fall_triggers_after_min_time_and_window_whatever_the_axis);
  RUN_TEST(test_a_baro_still_climbing_vetoes_the_free_fall_method);
  RUN_TEST(test_free_fall_without_a_baro_needs_the_longer_window);
  RUN_TEST(test_transonic_spike_during_lockout_cannot_poison_the_descent_reference);
  RUN_TEST(test_baro_method_is_locked_out_right_after_burnout);
  RUN_TEST(test_high_specific_force_vetoes_the_baro_method);
  RUN_TEST(test_a_baro_still_climbing_vetoes_the_gps_method);
  RUN_TEST(test_sensor_methods_are_gated_by_minimum_altitude_gain);
  RUN_TEST(test_backup_timer_fires_ungated_with_no_working_sensors);
  RUN_TEST(test_nominal_baro_apogee_is_detected_promptly_after_the_peak);
  return UNITY_END();
}
