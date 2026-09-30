// Audit #13 - default build: TEST_FREEZE does not exist. Exercises the REAL flight_commands.cpp
// compiled with the DEFAULT configuration (ENABLE_TEST_COMMANDS not defined -> 0).
#include <unity.h>
#include <fstream>
#include <sstream>
#include "../support/flight_harness.h"
#include "../support/real_sources.h"

void setUp() { harness_reset(); }
void tearDown() {}

void test_test_commands_are_off_by_default() {
  TEST_ASSERT_EQUAL(0, ENABLE_TEST_COMMANDS);
}

// Not compiled in: the handler does not recognise it (processCommand then reports "Unknown command"),
// so no state - including PAD_IDLE - can be made to hang the loop.
void test_test_freeze_is_not_recognised_and_never_freezes_in_any_state() {
  for (int s = STARTUP; s <= ERROR; s++) {
    harness_reset();
    g_currentFlightState = (FlightState)s;
    bool baro = false;
    const unsigned long t0 = millis();
    TEST_ASSERT_FALSE(handleFlightStateCommand("TEST_FREEZE", g_currentFlightState, g_previousFlightState, g_stateEntryTime, baro, true));
    TEST_ASSERT_FALSE(handleFlightStateCommand("test_freeze", g_currentFlightState, g_previousFlightState, g_stateEntryTime, baro, true));
    TEST_ASSERT_EQUAL(t0, millis());                      // delay(6000) never ran
  }
}

// The command must not survive anywhere else in the flight sources either.
void test_no_delay_6000_freeze_remains_in_command_processor() {
  std::string path = __FILE__;
  path = path.substr(0, path.rfind('/')) + "/../../src/command_processor.cpp";
  std::ifstream f(path);
  if (!f.good()) f.open("src/command_processor.cpp");
  TEST_ASSERT_TRUE(f.good());
  std::stringstream ss; ss << f.rdbuf();
  const std::string src = ss.str();
  TEST_ASSERT_TRUE(src.find("delay(6000)") == std::string::npos);
  TEST_ASSERT_TRUE(src.find("\"TEST_FREEZE\"") == std::string::npos);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_test_commands_are_off_by_default);
  RUN_TEST(test_test_freeze_is_not_recognised_and_never_freezes_in_any_state);
  RUN_TEST(test_no_delay_6000_freeze_remains_in_command_processor);
  return UNITY_END();
}
