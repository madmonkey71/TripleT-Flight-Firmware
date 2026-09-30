// Audit #10 - one pyro service owns the pins: the fire window always ends after
// PYRO_FIRE_DURATION regardless of the flight state, and no flag can stick between flights.
// Exercises the REAL pyro_control.cpp with the real state machine.
#include <unity.h>
#include "../support/flight_harness.h"
#include "../support/real_sources.h"

void setUp() { harness_reset(); }
void tearDown() {}

static bool level(int ch) { return g_pin_level[ch] == HIGH; }

// The old code raised/lowered the pin INSIDE the DROGUE_DEPLOY case, so if the state changed while the
// window was open nothing ever lowered it: the pin stayed HIGH indefinitely.
void test_the_pin_goes_low_after_the_fire_duration_even_if_the_state_changes_mid_window() {
  const FlightState leave_to[] = {ERROR, RECOVERY, LANDED, PAD_IDLE, MAIN_DESCENT, STARTUP, CALIBRATION};
  for (FlightState other : leave_to) {
    harness_reset();
    harness_on_pad(100.0f);
    g_flightInProgress = true;
    g_currentFlightState = APOGEE;
    harness_pass();
    TEST_ASSERT_EQUAL(DROGUE_DEPLOY, g_currentFlightState);
    harness_run_ms(300);
    TEST_ASSERT_TRUE(level(PYRO_CHANNEL_1));                       // window open
    g_currentFlightState = other;                                  // state changes under the fire window
    g_previousFlightState = other;
    harness_run_ms(PYRO_FIRE_DURATION - 300 - 100);
    TEST_ASSERT_TRUE_MESSAGE(level(PYRO_CHANNEL_1), getStateName(other));   // still inside the window: still on
    harness_run_ms(300);
    TEST_ASSERT_FALSE_MESSAGE(level(PYRO_CHANNEL_1), getStateName(other));  // window over: LOW whatever the state
    harness_run_ms(3000);
    TEST_ASSERT_FALSE_MESSAGE(level(PYRO_CHANNEL_1), getStateName(other));
    TEST_ASSERT_EQUAL_MESSAGE(1u, g_pin_rising_edges[PYRO_CHANNEL_1], getStateName(other));   // no second pulse
    TEST_ASSERT_TRUE_MESSAGE(g_pyroFiredMask & PYRO_FIRED_DROGUE, getStateName(other));       // completion recorded
  }
}

void test_the_fire_window_lasts_exactly_the_configured_duration() {
  harness_on_pad(100.0f);
  g_currentFlightState = APOGEE;
  harness_pass();
  const unsigned long t0 = millis();
  while (level(PYRO_CHANNEL_1) || millis() == t0) { harness_run_ms(10); if (millis() - t0 > 5000) break; }
  const unsigned long window = millis() - t0;
  TEST_ASSERT_TRUE(window >= PYRO_FIRE_DURATION);
  TEST_ASSERT_TRUE(window <= PYRO_FIRE_DURATION + 30);
}

// The old static `drogueHasFired` stayed true if the state left mid-window, so the NEXT flight's
// fire was skipped. There are no such flags now: a second flight (after reset_flight) fires again.
void test_a_second_flight_fires_again_after_an_aborted_first_window_and_reset() {
  harness_on_pad(100.0f);
  g_flightInProgress = true;
  g_currentFlightState = APOGEE; harness_pass();
  harness_run_ms(200);
  g_currentFlightState = RECOVERY; g_previousFlightState = RECOVERY;    // first flight ends mid-window
  harness_run_ms(PYRO_FIRE_DURATION + 500);
  TEST_ASSERT_FALSE(level(PYRO_CHANNEL_1));

  // operator recovers the vehicle and runs reset_flight (clears the fired mask)
  stateManagementResetRuntime();
  flightLogicReset();
  g_pin_rising_edges[PYRO_CHANNEL_1] = 0;
  g_currentFlightState = APOGEE; g_previousFlightState = PAD_IDLE;
  g_flightInProgress = true;
  harness_pass();
  harness_run_ms(200);
  TEST_ASSERT_TRUE(level(PYRO_CHANNEL_1));                     // fires again
  harness_run_ms(PYRO_FIRE_DURATION);
  TEST_ASSERT_FALSE(level(PYRO_CHANNEL_1));
  TEST_ASSERT_EQUAL(1u, g_pin_rising_edges[PYRO_CHANNEL_1]);
}

void test_an_idle_channel_is_actively_driven_low_every_pass() {
  harness_on_pad(100.0f);
  g_pin_level[PYRO_CHANNEL_1] = HIGH;                          // glitch / corrupted output latch
  g_pin_level[PYRO_CHANNEL_2] = HIGH;
  pyro_service();
  TEST_ASSERT_EQUAL(LOW, g_pin_level[PYRO_CHANNEL_1]);
  TEST_ASSERT_EQUAL(LOW, g_pin_level[PYRO_CHANNEL_2]);
}

void test_a_completed_channel_can_never_be_requested_again() {
  g_pyroFiredMask = PYRO_FIRED_DROGUE;
  TEST_ASSERT_FALSE(pyro_request_fire(PYRO_CH_DROGUE));
  TEST_ASSERT_FALSE(g_pin_ever_high[PYRO_CHANNEL_1]);
  TEST_ASSERT_TRUE(pyro_request_fire(PYRO_CH_MAIN));            // the other channel is unaffected
  TEST_ASSERT_TRUE(pyro_is_firing(PYRO_CH_MAIN));
}

void test_a_repeated_request_during_the_window_does_not_extend_or_retrigger_it() {
  TEST_ASSERT_TRUE(pyro_request_fire(PYRO_CH_DROGUE));
  for (int i = 0; i < 50; i++) { test_advance_ms(10); TEST_ASSERT_TRUE(pyro_request_fire(PYRO_CH_DROGUE)); pyro_service(); }
  test_advance_ms(PYRO_FIRE_DURATION);
  pyro_service();
  TEST_ASSERT_FALSE(level(PYRO_CHANNEL_1));
  TEST_ASSERT_EQUAL(1u, g_pin_rising_edges[PYRO_CHANNEL_1]);
  TEST_ASSERT_FALSE(pyro_request_fire(PYRO_CH_DROGUE));         // and it is done for good
}

// Only the DEPLOY sequence fires: none of the other states may drive either pin.
void test_no_state_other_than_the_deploy_sequence_ever_raises_a_pyro_pin() {
  const FlightState quiet[] = {STARTUP, CALIBRATION, PAD_IDLE, ARMED, BOOST, COAST, DROGUE_DESCENT, MAIN_DESCENT, LANDED, RECOVERY, ERROR};
  for (FlightState st : quiet) {
    harness_reset();
    harness_on_pad(100.0f);
    g_flightInProgress = (st >= BOOST);
    g_pyroFiredMask = PYRO_FIRED_DROGUE | PYRO_FIRED_MAIN;      // nothing left to fire in the later states
    harness_set_baro_alt(500.0f);
    g_maxAltitudeReached = 400.0f;
    g_main_deploy_altitude_m_agl = 0.0f;
    g_currentFlightState = st; g_previousFlightState = st;
    if (st == COAST || st == BOOST) boostEndTime = millis();
    harness_run_ms(4000);
    if (st == COAST) continue;                                  // COAST may legitimately proceed to deployment
    TEST_ASSERT_FALSE_MESSAGE(g_pin_ever_high[PYRO_CHANNEL_1] || g_pin_ever_high[PYRO_CHANNEL_2], getStateName(st));
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_the_pin_goes_low_after_the_fire_duration_even_if_the_state_changes_mid_window);
  RUN_TEST(test_the_fire_window_lasts_exactly_the_configured_duration);
  RUN_TEST(test_a_second_flight_fires_again_after_an_aborted_first_window_and_reset);
  RUN_TEST(test_an_idle_channel_is_actively_driven_low_every_pass);
  RUN_TEST(test_a_completed_channel_can_never_be_requested_again);
  RUN_TEST(test_a_repeated_request_during_the_window_does_not_extend_or_retrigger_it);
  RUN_TEST(test_no_state_other_than_the_deploy_sequence_ever_raises_a_pyro_pin);
  return UNITY_END();
}
