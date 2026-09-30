// Audit #1 - stale / recovered EEPROM state must never fire pyros at boot.
// Exercises the REAL state_management.cpp, startup_state.cpp, pyro_control.cpp and
// flight_logic.cpp. Only time, pins, EEPROM, Serial and sensor values are faked.
#include <unity.h>
#include <string>
#include <fstream>
#include <sstream>
#include "../support/flight_harness.h"
#include "../support/real_sources.h"

void setUp() { harness_reset(); }
void tearDown() {}

// ---- helpers ---------------------------------------------------------------
static bool is_flight_state(FlightState s) { return s >= BOOST && s <= MAIN_DESCENT; }

// A saved record as an earlier flight would have left it.
static FlightStateData make_record(FlightState st, uint8_t fired_mask = 0, bool flight_flag = true) {
  FlightStateData r;
  memset(&r, 0, sizeof r);
  r.state = st;
  r.launchAltitude = 100.0f;          // pad was at 100 m
  r.maxAltitude = 600.0f;             // flew to 600 m (500 m AGL)
  r.currentAltitude = 400.0f;
  r.mainDeployAltitudeAgl = 100.0f;
  r.baroAltitudeOffset = 0.0f;
  r.baroCalibrated = 1;
  r.flightInProgress = flight_flag ? 1 : 0;
  r.pyroFiredMask = fired_mask;
  r.resumeCount = 0;
  r.signature = EEPROM_SIGNATURE_VALUE;
  return r;
}
static void write_record(const FlightStateData& r) { EEPROM.put(EEPROM_STATE_ADDR, r); }
static FlightStateData read_record() { FlightStateData r; EEPROM.get(EEPROM_STATE_ADDR, r); return r; }

// Power up with `rec` in EEPROM, then run the real setup-phase recovery and the
// real main-loop passes for `run_ms`, with the barometer at `start_alt` moving at
// `vs` m/s and delivering a fresh sample every 100 ms (10 Hz, like the driver).
static void boot_and_run(const FlightStateData& rec, float start_alt, float vs, unsigned long run_ms, bool baro_ok = true) {
  write_record(rec);
  recoverFromPowerLoss();                       // setup() phase 1
  ms5611_initialized_ok = baro_ok;              // sensors come up
  pressure = baro_ok ? 1000.0f : 0.0f;
  harness_set_accel_g(1.0f);
  float alt = start_alt;
  harness_set_baro_alt(alt);
  for (unsigned long t = 0; t < run_ms; t += 10) {
    test_advance_ms(10);
    alt += vs * 0.01f;
    harness_set_baro_alt(alt);
    if (((t / 10) % 10) == 0) harness_new_samples();
    harness_pass();
  }
}

static bool any_pyro_ever_high() { return g_pin_ever_high[PYRO_CHANNEL_1] || g_pin_ever_high[PYRO_CHANNEL_2]; }

// ---- audit #1: stale EEPROM on the pad -------------------------------------
// Every possible saved state, with and without the flight flag / fired mask, on a
// stationary pad: no pyro pin may ever go HIGH and the vehicle must end in a
// pyro-inert state.
void test_stale_eeprom_in_every_state_never_fires_pyro_on_the_pad() {
  for (int st = STARTUP; st <= ERROR; st++) {
    for (int flag = 0; flag <= 1; flag++) {
      for (int mask = 0; mask <= 3; mask += 3) {
        harness_reset();
        boot_and_run(make_record((FlightState)st, (uint8_t)mask, flag), 100.0f, 0.0f, 30000);
        char msg[96];
        snprintf(msg, sizeof msg, "saved=%s flag=%d mask=%d final=%s", getStateName((FlightState)st), flag, mask,
                 getStateName(g_currentFlightState));
        TEST_ASSERT_FALSE_MESSAGE(any_pyro_ever_high(), msg);
        TEST_ASSERT_FALSE_MESSAGE(is_flight_state(g_currentFlightState), msg);
      }
    }
  }
}

// A stale in-flight record at a launch site whose altitude differs from the saved
// one (so "AGL" looks plausible) must still not resume: the vehicle is stationary.
void test_stale_record_at_a_different_higher_site_does_not_resume() {
  const FlightState in_flight[] = {BOOST, COAST, APOGEE, DROGUE_DEPLOY, DROGUE_DESCENT, MAIN_DEPLOY, MAIN_DESCENT};
  for (FlightState st : in_flight) {
    harness_reset();
    boot_and_run(make_record(st), 250.0f /* 150 m above saved launch alt */, 0.0f, 20000);
    TEST_ASSERT_FALSE_MESSAGE(any_pyro_ever_high(), getStateName(st));
    TEST_ASSERT_EQUAL_MESSAGE(RECOVERY, g_currentFlightState, getStateName(st));
  }
}

// ---- recovery decision table (pure function) --------------------------------
void test_decide_recovery_pre_flight_states() {
  FlightStateData r;
  r = make_record(STARTUP, 0, false);    TEST_ASSERT_EQUAL(STARTUP,  decideRecovery(r, false, 0, 0).state);
  r = make_record(CALIBRATION, 0, false);TEST_ASSERT_EQUAL(STARTUP,  decideRecovery(r, false, 0, 0).state);
  r = make_record(PAD_IDLE, 0, false);   TEST_ASSERT_EQUAL(STARTUP,  decideRecovery(r, false, 0, 0).state);
  r = make_record(ARMED, 0, false);      TEST_ASSERT_EQUAL(PAD_IDLE, decideRecovery(r, false, 0, 0).state);
  r = make_record(LANDED);               TEST_ASSERT_EQUAL(RECOVERY, decideRecovery(r, true, 300, -20).state);
  r = make_record(RECOVERY);             TEST_ASSERT_EQUAL(RECOVERY, decideRecovery(r, true, 300, -20).state);
  r = make_record(ERROR, 0, false);      TEST_ASSERT_EQUAL(ERROR,    decideRecovery(r, false, 0, 0).state);
  r = make_record(ERROR, 0, true);       TEST_ASSERT_EQUAL(RECOVERY, decideRecovery(r, false, 0, 0).state);
  r = make_record(STARTUP); r.state = 200; TEST_ASSERT_EQUAL(STARTUP, decideRecovery(r, true, 300, -20).state);
}

void test_decide_recovery_plausible_in_flight_states_never_restore_a_firing_state() {
  const float alt = 400.0f, vs = -20.0f;   // 300 m AGL and falling: plausible
  struct { FlightState saved; uint8_t mask; FlightState expect; } cases[] = {
    {BOOST, 0, COAST},           {COAST, 0, COAST},
    {APOGEE, 0, APOGEE},         {APOGEE, PYRO_FIRED_DROGUE, DROGUE_DESCENT},
    {DROGUE_DEPLOY, 0, APOGEE},  {DROGUE_DEPLOY, PYRO_FIRED_DROGUE, DROGUE_DESCENT},
    {DROGUE_DESCENT, PYRO_FIRED_DROGUE, DROGUE_DESCENT},
    {MAIN_DEPLOY, PYRO_FIRED_DROGUE, DROGUE_DESCENT},                                   // main not yet fired
    {MAIN_DEPLOY, PYRO_FIRED_DROGUE | PYRO_FIRED_MAIN, MAIN_DESCENT},                   // main already fired
    {MAIN_DESCENT, PYRO_FIRED_DROGUE | PYRO_FIRED_MAIN, MAIN_DESCENT},
  };
  for (auto& c : cases) {
    RecoveryDecision d = decideRecovery(make_record(c.saved, c.mask), true, alt, vs);
    char msg[64];
    snprintf(msg, sizeof msg, "saved=%s mask=%d", getStateName(c.saved), c.mask);
    TEST_ASSERT_TRUE_MESSAGE(d.plausible, msg);
    TEST_ASSERT_EQUAL_MESSAGE(c.expect, d.state, msg);
    TEST_ASSERT_TRUE_MESSAGE(d.state != DROGUE_DEPLOY && d.state != MAIN_DEPLOY, msg);
  }
}

void test_decide_recovery_implausible_evidence_falls_back_to_recovery() {
  const FlightStateData base = make_record(DROGUE_DESCENT, PYRO_FIRED_DROGUE);
  // sanity: the baseline case resumes
  TEST_ASSERT_TRUE(decideRecovery(base, true, 400, -20).resume);

  TEST_ASSERT_EQUAL(RECOVERY, decideRecovery(base, false, 400, -20).state);          // no baro
  TEST_ASSERT_FALSE(decideRecovery(base, false, 400, -20).resume);
  FlightStateData r = base; r.baroCalibrated = 0;
  TEST_ASSERT_EQUAL(RECOVERY, decideRecovery(r, true, 400, -20).state);              // baro reference lost
  TEST_ASSERT_EQUAL(RECOVERY, decideRecovery(base, true, 110, -20).state);           // 10 m AGL: below min
  TEST_ASSERT_EQUAL(RECOVERY, decideRecovery(base, true, 100 + 600 + 400, -20).state); // far above max+margin
  TEST_ASSERT_EQUAL(RECOVERY, decideRecovery(base, true, 400, 0.5f).state);          // stationary
  TEST_ASSERT_EQUAL(RECOVERY, decideRecovery(base, true, 400, NAN).state);           // garbage rate
  TEST_ASSERT_EQUAL(RECOVERY, decideRecovery(base, true, NAN, -20).state);           // garbage altitude
  r = base; r.resumeCount = RECOVERY_MAX_RESUMES;
  TEST_ASSERT_EQUAL(RECOVERY, decideRecovery(r, true, 400, -20).state);              // reset loop: give up
}

// ---- boot flows through the real setup-phase + loop code ---------------------
void test_resume_under_drogue_never_refires_drogue_and_fires_main_at_altitude() {
  // Reset while descending under drogue (drogue fire completed earlier).
  boot_and_run(make_record(DROGUE_DESCENT, PYRO_FIRED_DROGUE), 400.0f, -20.0f, 12000);
  TEST_ASSERT_FALSE_MESSAGE(g_pin_ever_high[PYRO_CHANNEL_1], "drogue must not re-fire");
  TEST_ASSERT_TRUE_MESSAGE(g_pin_ever_high[PYRO_CHANNEL_2], "main should fire on its altitude gate");
  TEST_ASSERT_EQUAL(MAIN_DESCENT, g_currentFlightState);
  TEST_ASSERT_EQUAL(PYRO_FIRED_DROGUE | PYRO_FIRED_MAIN, g_pyroFiredMask);
}

void test_reset_mid_drogue_window_fires_drogue_again_but_only_once() {
  // Saved DROGUE_DEPLOY with the fire window not completed (mask clear): the safe choice
  // is to fire (a second pulse into an already-fired channel is harmless; a missed drogue is not).
  boot_and_run(make_record(DROGUE_DEPLOY, 0), 500.0f, -1.0f * 3.0f, 5000);   // slow descent from apogee
  TEST_ASSERT_TRUE(g_pin_ever_high[PYRO_CHANNEL_1]);
  TEST_ASSERT_EQUAL(1u, g_pin_high_writes[PYRO_CHANNEL_1]);
  TEST_ASSERT_EQUAL(PYRO_FIRED_DROGUE, g_pyroFiredMask & PYRO_FIRED_DROGUE);

  // ... and a second reset after completion must not fire it again.
  FlightStateData saved = read_record();
  harness_reset();
  boot_and_run(saved, 450.0f, -3.0f, 5000);
  TEST_ASSERT_FALSE(g_pin_ever_high[PYRO_CHANNEL_1]);
}

void test_reset_after_both_channels_fired_never_fires_anything() {
  boot_and_run(make_record(MAIN_DEPLOY, PYRO_FIRED_DROGUE | PYRO_FIRED_MAIN), 250.0f, -6.0f, 5000);
  TEST_ASSERT_FALSE(any_pyro_ever_high());
  TEST_ASSERT_EQUAL(MAIN_DESCENT, g_currentFlightState);
}

void test_resume_gives_up_after_repeated_resets() {
  FlightStateData r = make_record(DROGUE_DESCENT, PYRO_FIRED_DROGUE);
  r.resumeCount = RECOVERY_MAX_RESUMES;
  boot_and_run(r, 400.0f, -20.0f, 5000);
  TEST_ASSERT_FALSE(any_pyro_ever_high());
  TEST_ASSERT_EQUAL(RECOVERY, g_currentFlightState);
}

void test_resume_increments_and_persists_resume_count() {
  boot_and_run(make_record(DROGUE_DESCENT, PYRO_FIRED_DROGUE), 400.0f, -20.0f, 3000);
  TEST_ASSERT_EQUAL(1, read_record().resumeCount);
  TEST_ASSERT_TRUE(read_record().flightInProgress);
}

void test_dead_barometer_at_boot_falls_back_to_recovery_after_timeout() {
  boot_and_run(make_record(DROGUE_DESCENT, PYRO_FIRED_DROGUE), 400.0f, -20.0f, 6000, /*baro_ok=*/false);
  TEST_ASSERT_FALSE(any_pyro_ever_high());
  TEST_ASSERT_EQUAL(RECOVERY, g_currentFlightState);
}

void test_while_evidence_is_pending_the_vehicle_stays_inert_in_startup() {
  write_record(make_record(DROGUE_DESCENT, PYRO_FIRED_DROGUE));
  recoverFromPowerLoss();
  TEST_ASSERT_TRUE(recoveryPending());
  TEST_ASSERT_EQUAL(STARTUP, g_currentFlightState);
  ms5611_initialized_ok = true; pressure = 1000.0f;
  harness_set_baro_alt(400);
  harness_new_baro_sample();
  test_advance_ms(10);
  g_test_sensors_healthy = false;          // even an unhealthy suite must not push it into ERROR while pending
  harness_pass();
  TEST_ASSERT_EQUAL(STARTUP, g_currentFlightState);
  TEST_ASSERT_FALSE(any_pyro_ever_high());
}

// audit #2: an unhealthy sensor suite at boot must not turn a resumed flight into ERROR.
void test_resumed_flight_with_unhealthy_sensors_stays_in_flight_state() {
  g_test_sensors_healthy = false;
  g_icm20948_ready = false;
  g_kx134_initialized_ok = false;
  boot_and_run(make_record(DROGUE_DESCENT, PYRO_FIRED_DROGUE), 400.0f, -20.0f, 12000);
  TEST_ASSERT_NOT_EQUAL(ERROR, g_currentFlightState);
  TEST_ASSERT_TRUE(g_pin_ever_high[PYRO_CHANNEL_2]);        // main still deploys at its altitude
  TEST_ASSERT_EQUAL(MAIN_DESCENT, g_currentFlightState);
}

// ---- persistence of the safety flags -------------------------------------------
void test_boost_entry_sets_and_persists_flight_in_progress() {
  harness_on_pad(100.0f);
  g_currentFlightState = ARMED;
  harness_pass();
  TEST_ASSERT_FALSE(g_flightInProgress);
  harness_set_accel_g(4.0f);
  harness_run_ms(LAUNCH_CONFIRMATION_COUNT * 100 + 100);
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
  TEST_ASSERT_TRUE(g_flightInProgress);
  FlightStateData r = read_record();
  TEST_ASSERT_TRUE(r.flightInProgress);
  TEST_ASSERT_EQUAL(BOOST, r.state);
}

void test_fired_flag_is_persisted_the_moment_a_fire_window_completes() {
  harness_on_pad(100.0f);
  g_launchAltitude = 100.0f;
  g_currentFlightState = APOGEE;               // jump straight to the deploy sequence
  harness_pass();
  TEST_ASSERT_EQUAL(DROGUE_DEPLOY, g_currentFlightState);
  TEST_ASSERT_EQUAL(0, read_record().pyroFiredMask & PYRO_FIRED_DROGUE);   // window still open
  harness_run_ms(PYRO_FIRE_DURATION + 50);
  TEST_ASSERT_EQUAL(DROGUE_DESCENT, g_currentFlightState);
  TEST_ASSERT_EQUAL(PYRO_FIRED_DROGUE, read_record().pyroFiredMask & PYRO_FIRED_DROGUE);
  TEST_ASSERT_EQUAL(DROGUE_DESCENT, read_record().state);
}

// ---- pyro pins at power-up -------------------------------------------------------
void test_pyro_init_safe_drives_both_pins_low_and_never_high() {
  g_pin_level[PYRO_CHANNEL_1] = HIGH;     // simulate a floating/latched-high pin at reset
  g_pin_level[PYRO_CHANNEL_2] = HIGH;
  g_pin_ever_high[PYRO_CHANNEL_1] = g_pin_ever_high[PYRO_CHANNEL_2] = false;
  pyro_init_safe();
  TEST_ASSERT_EQUAL(OUTPUT, g_pin_mode[PYRO_CHANNEL_1]);
  TEST_ASSERT_EQUAL(OUTPUT, g_pin_mode[PYRO_CHANNEL_2]);
  TEST_ASSERT_EQUAL(LOW, g_pin_level[PYRO_CHANNEL_1]);
  TEST_ASSERT_EQUAL(LOW, g_pin_level[PYRO_CHANNEL_2]);
  TEST_ASSERT_FALSE(any_pyro_ever_high());
}

// Source-order lint on the REAL setup(): main() cannot be compiled natively, so
// verify that pyro_init_safe() is its very first statement (before Serial.begin,
// the 3 s Serial wait, the watchdog, SD and sensor bring-up).
void test_setup_calls_pyro_init_safe_before_anything_else() {
  std::string path = __FILE__;
  path = path.substr(0, path.rfind('/')) + "/../../src/TripleT_Flight_Firmware.cpp";
  std::ifstream f(path);
  if (!f.good()) f.open("src/TripleT_Flight_Firmware.cpp");
  TEST_ASSERT_TRUE_MESSAGE(f.good(), "cannot open TripleT_Flight_Firmware.cpp");
  std::stringstream ss; ss << f.rdbuf();
  std::string src = ss.str();
  size_t setup = src.find("void setup() {");
  TEST_ASSERT_TRUE(setup != std::string::npos);
  // First non-blank, non-comment line of the body.
  std::istringstream body(src.substr(setup + strlen("void setup() {")));
  std::string line, first;
  while (std::getline(body, line)) {
    size_t a = line.find_first_not_of(" \t\r");
    if (a == std::string::npos) continue;
    if (line.compare(a, 2, "//") == 0) continue;
    first = line.substr(a);
    break;
  }
  TEST_ASSERT_EQUAL_STRING("pyro_init_safe();", first.c_str());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_stale_eeprom_in_every_state_never_fires_pyro_on_the_pad);
  RUN_TEST(test_stale_record_at_a_different_higher_site_does_not_resume);
  RUN_TEST(test_decide_recovery_pre_flight_states);
  RUN_TEST(test_decide_recovery_plausible_in_flight_states_never_restore_a_firing_state);
  RUN_TEST(test_decide_recovery_implausible_evidence_falls_back_to_recovery);
  RUN_TEST(test_resume_under_drogue_never_refires_drogue_and_fires_main_at_altitude);
  RUN_TEST(test_reset_mid_drogue_window_fires_drogue_again_but_only_once);
  RUN_TEST(test_reset_after_both_channels_fired_never_fires_anything);
  RUN_TEST(test_resume_gives_up_after_repeated_resets);
  RUN_TEST(test_resume_increments_and_persists_resume_count);
  RUN_TEST(test_dead_barometer_at_boot_falls_back_to_recovery_after_timeout);
  RUN_TEST(test_while_evidence_is_pending_the_vehicle_stays_inert_in_startup);
  RUN_TEST(test_resumed_flight_with_unhealthy_sensors_stays_in_flight_state);
  RUN_TEST(test_boost_entry_sets_and_persists_flight_in_progress);
  RUN_TEST(test_fired_flag_is_persisted_the_moment_a_fire_window_completes);
  RUN_TEST(test_pyro_init_safe_drives_both_pins_low_and_never_high);
  RUN_TEST(test_setup_calls_pyro_init_safe_before_anything_else);
  return UNITY_END();
}
