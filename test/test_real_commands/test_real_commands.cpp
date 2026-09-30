// Audit #3 (and later #7, #13): guards on leaving ERROR / dangerous serial commands.
// Exercises the REAL flight_logic.cpp ERROR auto-recovery and flight_commands.cpp.
#include <unity.h>
#include "../support/flight_harness.h"
#include "../support/real_sources.h"

void setUp() { harness_reset(); }
void tearDown() {}

// Vehicle parked in ERROR with a healthy sensor suite (so recovery is possible if allowed).
static void park_in_error(float launch_alt = 100.0f) {
  harness_on_pad(launch_alt);
  g_maxAltitudeReached = 0.0f;
  g_currentFlightState = ERROR;
  g_previousFlightState = ERROR;
  g_last_error_code = STATE_TRANSITION_INVALID_HEALTH;
  g_test_sensors_healthy = true;
}

static bool run_cmd(const char* cmd) {
  bool baro = g_baroCalibrated;
  bool handled = handleFlightStateCommand(cmd, g_currentFlightState, g_previousFlightState, g_stateEntryTime, baro, true);
  g_baroCalibrated = baro;
  return handled;
}

// ---- auto-recovery ----------------------------------------------------------
// The audit's scenario: an airborne vehicle in ERROR whose sensors recover was sent to
// PAD_IDLE, which also reset the launch altitude and max altitude.
void test_error_auto_recovery_is_refused_once_a_flight_is_in_progress() {
  park_in_error(100.0f);
  g_flightInProgress = true;
  g_maxAltitudeReached = 812.0f;
  g_launchAltitude = 100.0f;
  harness_run_ms(20000);
  TEST_ASSERT_EQUAL(ERROR, g_currentFlightState);
  TEST_ASSERT_EQUAL_FLOAT(812.0f, g_maxAltitudeReached);   // not wiped
  TEST_ASSERT_EQUAL_FLOAT(100.0f, g_launchAltitude);
}

void test_error_auto_recovery_is_refused_when_the_baro_shows_the_vehicle_is_airborne() {
  park_in_error(100.0f);                     // flight flag NOT set (e.g. un-armed launch)
  harness_set_baro_alt(100.0f + 250.0f);     // 250 m above the launch altitude
  harness_run_ms(20000);
  TEST_ASSERT_EQUAL(ERROR, g_currentFlightState);
}

void test_error_auto_recovery_still_works_on_the_ground() {
  park_in_error(100.0f);
  harness_run_ms(6000);
  TEST_ASSERT_EQUAL(PAD_IDLE, g_currentFlightState);
  TEST_ASSERT_EQUAL(NO_ERROR, g_last_error_code);
}

void test_provably_on_ground_predicate() {
  harness_on_pad(100.0f);
  TEST_ASSERT_TRUE(flight_is_provably_on_ground());
  harness_set_baro_alt(100.0f + GROUND_AGL_TOLERANCE_M + 1.0f);
  TEST_ASSERT_FALSE(flight_is_provably_on_ground());
  harness_set_baro_alt(100.0f + GROUND_AGL_TOLERANCE_M - 1.0f);
  TEST_ASSERT_TRUE(flight_is_provably_on_ground());
  // A stale barometer cannot disprove it (the flight flag still can).
  harness_set_baro_alt(500.0f);
  test_advance_ms(BARO_STALE_TIMEOUT_MS + 100);
  TEST_ASSERT_TRUE(flight_is_provably_on_ground());
  g_flightInProgress = true;
  TEST_ASSERT_FALSE(flight_is_provably_on_ground());
  g_flightInProgress = false;
  g_currentFlightState = DROGUE_DESCENT;
  TEST_ASSERT_FALSE(flight_is_provably_on_ground());
}

// ---- commands ----------------------------------------------------------------
void test_clear_commands_are_refused_in_flight_and_leave_the_state_alone() {
  const char* cmds[] = {"clear_errors", "clear_to_calibration", "skip_calibration"};
  for (const char* c : cmds) {
    harness_reset();
    park_in_error(100.0f);
    g_flightInProgress = true;
    g_maxAltitudeReached = 812.0f;
    Serial.clear();
    TEST_ASSERT_TRUE(run_cmd(c));
    TEST_ASSERT_EQUAL_MESSAGE(ERROR, g_currentFlightState, c);
    TEST_ASSERT_EQUAL_FLOAT(812.0f, g_maxAltitudeReached);
    TEST_ASSERT_TRUE_MESSAGE(Serial.contains("REFUSED"), c);
  }
}

void test_clear_commands_are_refused_when_baro_shows_airborne_without_flag() {
  const char* cmds[] = {"clear_errors", "clear_to_calibration", "skip_calibration"};
  for (const char* c : cmds) {
    harness_reset();
    park_in_error(100.0f);
    harness_set_baro_alt(400.0f);
    TEST_ASSERT_TRUE(run_cmd(c));
    TEST_ASSERT_EQUAL_MESSAGE(ERROR, g_currentFlightState, c);
  }
}

void test_clear_commands_work_on_the_ground() {
  park_in_error(100.0f);
  TEST_ASSERT_TRUE(run_cmd("clear_errors"));
  TEST_ASSERT_EQUAL(PAD_IDLE, g_currentFlightState);

  harness_reset(); park_in_error(100.0f);
  TEST_ASSERT_TRUE(run_cmd("clear_to_calibration"));
  TEST_ASSERT_EQUAL(CALIBRATION, g_currentFlightState);

  harness_reset(); park_in_error(100.0f);
  TEST_ASSERT_TRUE(run_cmd("skip_calibration"));
  TEST_ASSERT_EQUAL(PAD_IDLE, g_currentFlightState);
}

void test_clear_errors_outside_error_state_is_a_noop_and_unknown_commands_are_not_handled() {
  harness_on_pad(100.0f);
  TEST_ASSERT_TRUE(run_cmd("clear_errors"));
  TEST_ASSERT_EQUAL(PAD_IDLE, g_currentFlightState);
  TEST_ASSERT_FALSE(run_cmd("arm"));
}

// ---- boot: saved ERROR --------------------------------------------------------
void test_boot_with_saved_error_and_flight_flag_never_auto_recovers_to_pad_idle() {
  FlightStateData r; memset(&r, 0, sizeof r);
  r.state = ERROR; r.flightInProgress = 1; r.signature = EEPROM_SIGNATURE_VALUE; r.launchAltitude = 100; r.maxAltitude = 700;
  EEPROM.put(EEPROM_STATE_ADDR, r);
  recoverFromPowerLoss();
  harness_run_ms(10000);
  TEST_ASSERT_EQUAL(RECOVERY, g_currentFlightState);   // pyro-inert; never PAD_IDLE
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_error_auto_recovery_is_refused_once_a_flight_is_in_progress);
  RUN_TEST(test_error_auto_recovery_is_refused_when_the_baro_shows_the_vehicle_is_airborne);
  RUN_TEST(test_error_auto_recovery_still_works_on_the_ground);
  RUN_TEST(test_provably_on_ground_predicate);
  RUN_TEST(test_clear_commands_are_refused_in_flight_and_leave_the_state_alone);
  RUN_TEST(test_clear_commands_are_refused_when_baro_shows_airborne_without_flag);
  RUN_TEST(test_clear_commands_work_on_the_ground);
  RUN_TEST(test_clear_errors_outside_error_state_is_a_noop_and_unknown_commands_are_not_handled);
  RUN_TEST(test_boot_with_saved_error_and_flight_flag_never_auto_recovers_to_pad_idle);
  return UNITY_END();
}
