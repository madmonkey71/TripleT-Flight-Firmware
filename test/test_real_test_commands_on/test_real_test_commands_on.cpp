// Audit #13 - bench build (ENABLE_TEST_COMMANDS=1): TEST_FREEZE exists but is refused outside PAD_IDLE.
// The flag is set before anything includes config.h. Exercises the REAL flight_commands.cpp.
#define ENABLE_TEST_COMMANDS 1
#include <unity.h>
#include "../support/flight_harness.h"
#include "../support/real_sources.h"

void setUp() { harness_reset(); }
void tearDown() {}

static bool freeze(FlightState st) {
  g_currentFlightState = st;
  bool baro = false;
  return handleFlightStateCommand("TEST_FREEZE", g_currentFlightState, g_previousFlightState, g_stateEntryTime, baro, true);
}

void test_flag_is_on_in_this_build() { TEST_ASSERT_EQUAL(1, ENABLE_TEST_COMMANDS); }

void test_test_freeze_runs_in_pad_idle() {
  const unsigned long t0 = millis();
  TEST_ASSERT_TRUE(freeze(PAD_IDLE));
  TEST_ASSERT_TRUE(millis() - t0 >= 6000);              // it really did block for 6 s (exceeds the 5 s watchdog)
  TEST_ASSERT_TRUE(Serial.contains("Freezing"));
}

void test_test_freeze_is_refused_in_every_other_state() {
  for (int s = STARTUP; s <= ERROR; s++) {
    if (s == PAD_IDLE) continue;
    harness_reset();
    const unsigned long t0 = millis();
    Serial.clear();
    TEST_ASSERT_TRUE_MESSAGE(freeze((FlightState)s), getStateName((FlightState)s));   // recognised...
    TEST_ASSERT_EQUAL_MESSAGE(t0, millis(), getStateName((FlightState)s));            // ...but never hangs the loop
    TEST_ASSERT_TRUE_MESSAGE(Serial.contains("REFUSED"), getStateName((FlightState)s));
    TEST_ASSERT_EQUAL_MESSAGE(s, g_currentFlightState, getStateName((FlightState)s));
  }
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_flag_is_on_in_this_build);
  RUN_TEST(test_test_freeze_runs_in_pad_idle);
  RUN_TEST(test_test_freeze_is_refused_in_every_other_state);
  return UNITY_END();
}
