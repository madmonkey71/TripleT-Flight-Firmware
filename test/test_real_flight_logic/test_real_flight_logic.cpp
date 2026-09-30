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
  harness_run_ms(100);
  TEST_ASSERT_EQUAL(BOOST, g_currentFlightState);

  harness_set_accel_g(0.1f);                 // burnout
  harness_run_ms(100);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);

  harness_set_baro_alt(600.0f);              // apex
  harness_run_ms(100);
  TEST_ASSERT_EQUAL(COAST, g_currentFlightState);
  harness_set_baro_alt(590.0f);              // now clearly descending
  harness_run_ms(200);
  TEST_ASSERT_TRUE(g_currentFlightState >= APOGEE);

  harness_run_ms(PYRO_FIRE_DURATION + 100);  // drogue fire window
  TEST_ASSERT_EQUAL(DROGUE_DESCENT, g_currentFlightState);
  TEST_ASSERT_TRUE(g_pin_ever_high[PYRO_CHANNEL_1]);
  TEST_ASSERT_EQUAL(LOW, g_pin_level[PYRO_CHANNEL_1]);

  harness_set_baro_alt(150.0f);              // below main deploy altitude (launch + 100 m)
  harness_run_ms(100);
  harness_run_ms(PYRO_FIRE_DURATION + 100);
  TEST_ASSERT_EQUAL(MAIN_DESCENT, g_currentFlightState);
  TEST_ASSERT_TRUE(g_pin_ever_high[PYRO_CHANNEL_2]);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_real_nominal_flight_walk);
  return UNITY_END();
}
