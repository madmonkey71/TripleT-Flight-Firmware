// Audit #6 - EEPROM persistence: unthrottled saves on every state change (put-if-changed),
// periodic progress saves in BOOST/COAST, and the recovery table for EVERY saved state,
// including COAST resuming with the backup apogee timer restored from a persisted burnout age.
// Exercises the REAL state_management.cpp / startup_state.cpp / flight_logic.cpp.
#include <unity.h>
#include "../support/flight_harness.h"
#include "../support/real_sources.h"

void setUp() { harness_reset(); }
void tearDown() {}

static FlightStateData read_record() { FlightStateData r; EEPROM.get(EEPROM_STATE_ADDR, r); return r; }
static void write_record(const FlightStateData& r) { EEPROM.put(EEPROM_STATE_ADDR, r); }
static FlightStateData make_record(FlightState st, uint8_t fired_mask = 0) {
  FlightStateData r; memset(&r, 0, sizeof r);
  r.state = st; r.launchAltitude = 100.0f; r.maxAltitude = 600.0f; r.currentAltitude = 400.0f;
  r.mainDeployAltitudeAgl = 100.0f; r.baroCalibrated = 1; r.flightInProgress = 1;
  r.pyroFiredMask = fired_mask; r.signature = EEPROM_SIGNATURE_VALUE;
  return r;
}
// Boot with `rec` in EEPROM and run the real recovery + loop with the baro moving at `vs` from `alt`.
static void boot_and_run(const FlightStateData& rec, float alt, float vs, unsigned long run_ms, float accel_g = 0.8f) {
  write_record(rec);
  recoverFromPowerLoss();
  ms5611_initialized_ok = true; pressure = 1000.0f;
  harness_set_accel_g(accel_g);
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

// ---- saves ---------------------------------------------------------------------------
// The old saveStateToEEPROM() skipped any save within 60 s of the last one unless the state was
// APOGEE/DROGUE_DEPLOY/MAIN_DEPLOY/LANDED, so a quick PAD_IDLE->ARMED->BOOST->COAST left EEPROM stale.
void test_every_state_change_is_saved_immediately() {
  harness_on_pad(100.0f);
  TEST_ASSERT_EQUAL(PAD_IDLE, read_record().state);
  g_currentFlightState = ARMED;                       // what `arm` does (it then saves)
  saveStateToEEPROM();
  TEST_ASSERT_EQUAL(ARMED, read_record().state);
  harness_pass();
  harness_set_accel_g(4.0f);
  harness_run_ms(200);
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);
  TEST_ASSERT_EQUAL(BOOST, read_record().state);      // < 1 s after the previous save
  harness_set_accel_g(0.1f);
  harness_run_ms(500);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  TEST_ASSERT_EQUAL(COAST, read_record().state);      // ... and again
}

void test_save_is_put_if_changed() {
  harness_on_pad(100.0f);
  saveStateToEEPROM();
  const unsigned long puts = EEPROM.put_calls, changed = EEPROM.bytes_changed;
  for (int i = 0; i < 50; i++) { test_advance_ms(100); saveStateToEEPROM(); }   // nothing changed but the clock
  TEST_ASSERT_EQUAL(puts, EEPROM.put_calls);
  TEST_ASSERT_EQUAL(changed, EEPROM.bytes_changed);
  g_maxAltitudeReached = 123.0f;
  saveStateToEEPROM();
  TEST_ASSERT_EQUAL(puts + 1, EEPROM.put_calls);
  TEST_ASSERT_EQUAL_FLOAT(123.0f, read_record().maxAltitude);
}

void test_progress_is_saved_periodically_in_boost_and_coast_only() {
  harness_on_pad(100.0f);
  harness_run_ms(30000);
  for (int i = 0; i < 300; i++) { test_advance_ms(100); saveFlightProgressPeriodic(); }
  const unsigned long idle_puts = EEPROM.put_calls;    // PAD_IDLE: no periodic writes at all
  for (int i = 0; i < 300; i++) { test_advance_ms(100); saveFlightProgressPeriodic(); }
  TEST_ASSERT_EQUAL(idle_puts, EEPROM.put_calls);

  // COAST with a climbing baro: the record follows max altitude within ~1 s, ~1 write per second.
  g_flightInProgress = true;
  g_currentFlightState = COAST; g_previousFlightState = COAST;
  boostEndTime = millis();
  saveStateToEEPROM();
  const unsigned long puts0 = EEPROM.put_calls;
  float alt = 200.0f;
  for (int i = 0; i < 100; i++) {                       // 10 s
    alt += 3.0f;
    harness_set_baro_alt(alt);
    harness_run_ms(100);
    saveFlightProgressPeriodic();
  }
  TEST_ASSERT_TRUE(g_maxAltitudeReached > 250.0f);
  TEST_ASSERT_TRUE(g_maxAltitudeReached - read_record().maxAltitude < 3.0f * 11 + 1);   // <= ~1 s stale
  const unsigned long puts = EEPROM.put_calls - puts0;
  TEST_ASSERT_TRUE_MESSAGE(puts >= 8 && puts <= 12, "about one write per second");
  TEST_ASSERT_TRUE(read_record().burnoutAgeMs > 8000);
}

// ---- recovery: COAST resumes and the backup timer survives the reset ------------------------
// Old behaviour: a reset in BOOST/COAST jumped to DROGUE_DESCENT, skipping the drogue entirely.
void test_reset_in_coast_resumes_coast_and_the_backup_timer_uses_the_persisted_age() {
  FlightStateData r = make_record(COAST);
  r.burnoutAgeMs = 12000;                               // 12 s after burnout at the last save
  const unsigned long t0 = millis();
  write_record(r);
  recoverFromPowerLoss();
  ms5611_initialized_ok = true; pressure = 1000.0f;
  harness_set_accel_g(0.8f);
  // Climbing baro (+5 m/s) so no sensor method can fire; only the backup timer can.
  float alt = 300.0f;
  bool resumed = false;
  unsigned long t_resume = 0;
  for (unsigned long t = 0; t < 20000 && g_currentFlightState <= COAST; t += 10) {
    test_advance_ms(10);
    alt += 0.05f; harness_set_baro_alt(alt);
    if (((t / 10) % 10) == 0) harness_new_samples();
    harness_pass();
    if (!resumed && !recoveryPending() && g_currentFlightState == COAST) { resumed = true; t_resume = millis(); }
  }
  TEST_ASSERT_TRUE(resumed);
  TEST_ASSERT_TRUE(g_currentFlightState >= APOGEE);     // the drogue is NOT skipped
  // Nominal = 20 s after burnout. 12 s already elapsed + 3 s allowance => ~5 s after the resume.
  const unsigned long fired_after = millis() - t_resume;
  TEST_ASSERT_TRUE_MESSAGE(fired_after >= 4500 && fired_after <= 5600, "backup timer restored from persisted burnout age");
  (void)t0;
}

void test_reset_in_boost_resumes_coast_with_only_the_allowance_elapsed() {
  boot_and_run(make_record(BOOST), 300.0f, +5.0f, 3000);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  TEST_ASSERT_FALSE(any_pyro_ever_high());
  const unsigned long since = millis() - boostEndTime;
  TEST_ASSERT_TRUE(since >= RECOVERY_BACKUP_TIMER_ALLOWANCE_MS && since < RECOVERY_BACKUP_TIMER_ALLOWANCE_MS + 3500);
}

void test_coast_resume_then_sensor_apogee_still_fires_the_drogue() {
  FlightStateData r = make_record(COAST); r.burnoutAgeMs = 4000;
  boot_and_run(r, 500.0f, -20.0f, 8000);                // falling from the start: baro method fires
  TEST_ASSERT_TRUE(g_pin_ever_high[PYRO_CHANNEL_1]);
}

// ---- recovery table: every saved state through the real boot path -----------------------------------
// The vehicle is 300 m AGL and falling at 20 m/s (plausible in-flight evidence), 5 s of loop passes.
void test_recovery_table_every_saved_state_with_plausible_flight_evidence() {
  struct Row { FlightState saved; uint8_t mask; bool expect_drogue_fire; bool expect_main_fire; FlightState a; FlightState b; };
  const uint8_t D = PYRO_FIRED_DROGUE, M = PYRO_FIRED_MAIN;
  const Row rows[] = {
    // saved            mask   drogue  main   final state (either of a/b)
    {BOOST,             0,     true,   false, COAST,          DROGUE_DESCENT},   // resumes COAST; baro is falling => apogee, drogue
    {COAST,             0,     true,   false, COAST,          DROGUE_DESCENT},
    {APOGEE,            0,     true,   false, DROGUE_DESCENT, DROGUE_DESCENT},   // drogue had not fired: fires now
    {DROGUE_DEPLOY,     0,     true,   false, DROGUE_DESCENT, DROGUE_DESCENT},
    {DROGUE_DEPLOY,     D,     false,  false, DROGUE_DESCENT, DROGUE_DESCENT},   // window completed before the reset
    {DROGUE_DESCENT,    D,     false,  false, DROGUE_DESCENT, DROGUE_DESCENT},
    {MAIN_DEPLOY,       D,     false,  false, DROGUE_DESCENT, DROGUE_DESCENT},   // main not fired yet, still above deploy alt
    {MAIN_DEPLOY,       D | M, false,  false, MAIN_DESCENT,   MAIN_DESCENT},
    {MAIN_DESCENT,      D | M, false,  false, MAIN_DESCENT,   MAIN_DESCENT},
    {LANDED,            D | M, false,  false, RECOVERY,       RECOVERY},
    {RECOVERY,          D | M, false,  false, RECOVERY,       RECOVERY},
  };
  for (const Row& row : rows) {
    harness_reset();
    boot_and_run(make_record(row.saved, row.mask), 400.0f, -20.0f, 5000, 1.0f);
    char msg[80];
    snprintf(msg, sizeof msg, "saved=%s mask=%d final=%s", getStateName(row.saved), row.mask, getStateName(g_currentFlightState));
    TEST_ASSERT_EQUAL_MESSAGE(row.expect_drogue_fire, g_pin_ever_high[PYRO_CHANNEL_1], msg);
    TEST_ASSERT_EQUAL_MESSAGE(row.expect_main_fire, g_pin_ever_high[PYRO_CHANNEL_2], msg);
    if (row.saved == BOOST || row.saved == COAST) {
      TEST_ASSERT_TRUE_MESSAGE(g_currentFlightState == COAST || g_currentFlightState >= APOGEE, msg);
    } else {
      TEST_ASSERT_TRUE_MESSAGE(g_currentFlightState == row.a || g_currentFlightState == row.b, msg);
    }
  }
}

void test_recovery_table_pre_flight_saved_states() {
  struct Row { FlightState saved; bool allowed_calibration; };
  const FlightState pre[] = {STARTUP, CALIBRATION, PAD_IDLE, ARMED};
  for (FlightState st : pre) {
    harness_reset();
    FlightStateData r = make_record(st); r.flightInProgress = 0;
    boot_and_run(r, 100.0f, 0.0f, 4000, 1.0f);
    char msg[48]; snprintf(msg, sizeof msg, "saved=%s", getStateName(st));
    TEST_ASSERT_FALSE_MESSAGE(any_pyro_ever_high(), msg);
    TEST_ASSERT_TRUE_MESSAGE(g_currentFlightState == PAD_IDLE || g_currentFlightState == CALIBRATION, msg);
    TEST_ASSERT_FALSE_MESSAGE(g_currentFlightState == ARMED, msg);   // never boots armed
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_every_state_change_is_saved_immediately);
  RUN_TEST(test_save_is_put_if_changed);
  RUN_TEST(test_progress_is_saved_periodically_in_boost_and_coast_only);
  RUN_TEST(test_reset_in_coast_resumes_coast_and_the_backup_timer_uses_the_persisted_age);
  RUN_TEST(test_reset_in_boost_resumes_coast_with_only_the_allowance_elapsed);
  RUN_TEST(test_coast_resume_then_sensor_apogee_still_fires_the_drogue);
  RUN_TEST(test_recovery_table_every_saved_state_with_plausible_flight_evidence);
  RUN_TEST(test_recovery_table_pre_flight_saved_states);
  return UNITY_END();
}
