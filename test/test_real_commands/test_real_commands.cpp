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


// ---- audit #7: reset_flight ---------------------------------------------------------------
// After landing there was no path back to PAD_IDLE: RECOVERY/LANDED are terminal, and the
// (now) persisted flight flag blocks every ERROR clear. reset_flight is the explicit, guarded exit.
#include <fstream>
#include <sstream>

// Extract the confirmation token printed by the last `reset_flight` request (0 if none).
static unsigned printed_token() {
  const std::string& out = Serial.out;
  size_t p = out.rfind("reset_flight ");
  if (p == std::string::npos) return 0;
  return (unsigned)atoi(out.c_str() + p + strlen("reset_flight "));
}

// A landed vehicle at rest: RECOVERY, flight recorded, at 1 g, baro steady (landing spot 60 m above the pad).
static void landed_at_rest(FlightState st = RECOVERY) {
  harness_on_pad(100.0f);
  g_flightInProgress = true;
  g_pyroFiredMask = PYRO_FIRED_DROGUE | PYRO_FIRED_MAIN;
  g_maxAltitudeReached = 812.0f;
  harness_set_baro_alt(160.0f);
  harness_set_accel_g(1.0f);
  g_currentFlightState = st;
  g_previousFlightState = st;
  saveStateToEEPROM();
  harness_run_ms(1500);    // build up >1 s of steady baro samples
}

void test_reset_flight_requires_the_confirmation_token() {
  landed_at_rest();
  Serial.clear();
  TEST_ASSERT_TRUE(run_cmd("reset_flight"));                 // step 1: just issues a token
  TEST_ASSERT_EQUAL(RECOVERY, g_currentFlightState);
  TEST_ASSERT_TRUE(g_flightInProgress);
  const unsigned tok = printed_token();
  TEST_ASSERT_TRUE(tok >= 1000);

  Serial.clear();
  TEST_ASSERT_TRUE(run_cmd("reset_flight 1"));                // wrong token
  TEST_ASSERT_EQUAL(RECOVERY, g_currentFlightState);
  TEST_ASSERT_TRUE(Serial.contains("REFUSED"));

  Serial.clear();
  run_cmd("reset_flight");                                    // (a wrong token re-issues one)
  const unsigned tok2 = printed_token();
  TEST_ASSERT_TRUE(tok2 >= 1000);
  test_advance_ms(RESET_FLIGHT_TOKEN_TIMEOUT_MS + 1000);      // expired
  char buf[40]; snprintf(buf, sizeof buf, "reset_flight %u", tok2);
  Serial.clear();
  run_cmd(buf);
  TEST_ASSERT_EQUAL(RECOVERY, g_currentFlightState);
  TEST_ASSERT_TRUE(Serial.contains("REFUSED"));
  TEST_ASSERT_TRUE(g_flightInProgress);
}

void test_reset_flight_with_the_right_token_clears_the_flight_and_persists_it() {
  landed_at_rest();
  Serial.clear();
  run_cmd("reset_flight");
  const unsigned tok = printed_token();
  char buf[40]; snprintf(buf, sizeof buf, "reset_flight %u", tok);
  Serial.clear();
  TEST_ASSERT_TRUE(run_cmd(buf));
  TEST_ASSERT_EQUAL(PAD_IDLE, g_currentFlightState);
  TEST_ASSERT_FALSE(g_flightInProgress);
  TEST_ASSERT_EQUAL(0, g_pyroFiredMask);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, g_maxAltitudeReached);
  FlightStateData r; EEPROM.get(EEPROM_STATE_ADDR, r);
  TEST_ASSERT_EQUAL(PAD_IDLE, r.state);
  TEST_ASSERT_FALSE(r.flightInProgress);
  TEST_ASSERT_EQUAL(0, r.pyroFiredMask);
  // single use: replaying the token does nothing
  Serial.clear();
  run_cmd(buf);
  TEST_ASSERT_EQUAL(PAD_IDLE, g_currentFlightState);

  // ...and the next boot starts normally instead of resuming/holding the old flight.
  recoverFromPowerLoss();
  TEST_ASSERT_FALSE(recoveryPending());
  TEST_ASSERT_NOT_EQUAL(RECOVERY, g_currentFlightState);
}

void test_reset_flight_works_from_every_ground_state() {
  const FlightState ok[] = {LANDED, RECOVERY, ERROR, PAD_IDLE};
  for (FlightState st : ok) {
    harness_reset();
    landed_at_rest(st);
    Serial.clear();
    run_cmd("reset_flight");
    const unsigned tok = printed_token();
    TEST_ASSERT_TRUE_MESSAGE(tok >= 1000, getStateName(st));
    char buf[40]; snprintf(buf, sizeof buf, "reset_flight %u", tok);
    run_cmd(buf);
    TEST_ASSERT_EQUAL_MESSAGE(PAD_IDLE, g_currentFlightState, getStateName(st));
    TEST_ASSERT_FALSE_MESSAGE(g_flightInProgress, getStateName(st));
  }
}

void test_reset_flight_is_refused_in_every_other_state_and_issues_no_token() {
  const FlightState bad[] = {STARTUP, CALIBRATION, ARMED, BOOST, COAST, APOGEE, DROGUE_DEPLOY,
                             DROGUE_DESCENT, MAIN_DEPLOY, MAIN_DESCENT};
  for (FlightState st : bad) {
    harness_reset();
    landed_at_rest(RECOVERY);
    g_currentFlightState = st;
    g_previousFlightState = st;
    Serial.clear();
    TEST_ASSERT_TRUE(run_cmd("reset_flight"));
    TEST_ASSERT_TRUE_MESSAGE(Serial.contains("REFUSED"), getStateName(st));
    TEST_ASSERT_EQUAL_MESSAGE(0u, printed_token(), getStateName(st));
    TEST_ASSERT_TRUE_MESSAGE(g_flightInProgress, getStateName(st));
  }
}

void test_reset_flight_is_refused_while_the_vehicle_is_moving() {
  landed_at_rest(RECOVERY);
  float alt = 160.0f;                       // baro descending at 20 m/s
  for (int i = 0; i < 20; i++) { alt -= 2.0f; harness_set_baro_alt(alt); harness_run_ms(100); }
  Serial.clear();
  run_cmd("reset_flight");
  TEST_ASSERT_TRUE(Serial.contains("REFUSED"));
  TEST_ASSERT_EQUAL(0u, printed_token());

  harness_reset();
  landed_at_rest(RECOVERY);
  harness_set_accel_g(3.0f);                // being shaken / accelerating
  harness_run_ms(600);
  Serial.clear();
  run_cmd("reset_flight");
  TEST_ASSERT_TRUE(Serial.contains("REFUSED"));
}

void test_reset_flight_is_listed_in_the_help_text() {
  std::string path = __FILE__;
  path = path.substr(0, path.rfind('/')) + "/../../src/command_processor.cpp";
  std::ifstream f(path);
  if (!f.good()) f.open("src/command_processor.cpp");
  TEST_ASSERT_TRUE(f.good());
  std::stringstream ss; ss << f.rdbuf();
  TEST_ASSERT_TRUE(ss.str().find("reset_flight") != std::string::npos);
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
  RUN_TEST(test_reset_flight_requires_the_confirmation_token);
  RUN_TEST(test_reset_flight_with_the_right_token_clears_the_flight_and_persists_it);
  RUN_TEST(test_reset_flight_works_from_every_ground_state);
  RUN_TEST(test_reset_flight_is_refused_in_every_other_state_and_issues_no_token);
  RUN_TEST(test_reset_flight_is_refused_while_the_vehicle_is_moving);
  RUN_TEST(test_reset_flight_is_listed_in_the_help_text);
  return UNITY_END();
}
