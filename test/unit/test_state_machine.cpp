#include <unity.h>
#include "../../src/flight_logic.h"
#include "../../src/state_management.h"
#include "../mocks/mock_sensors.h"

// ============================================================================
// STATE MACHINE UNIT TESTS
//
// Tests the flight state machine transitions to ensure the rocket follows
// the correct state sequence during a simulated flight.
//
// State sequence: STARTUP → CALIBRATION → PAD_IDLE → ARMED → BOOST →
//                 COAST → APOGEE → DROGUE_DEPLOY → DROGUE_DESCENT →
//                 MAIN_DEPLOY → MAIN_DESCENT → LANDED → RECOVERY
// ============================================================================

class TestStateMachine {
public:
  void setUp() {
    // Initialize mock sensor
    mock_sensor.begin();
    mock_sensor.reset();
    mock_sensor.setTemperature(25.0f);
    mock_sensor.setQuaternion(1.0f, 0.0f, 0.0f, 0.0f);
  }

  void tearDown() {
    mock_sensor.reset();
  }

  void test_startup_to_calibration_transition() {
    // Initial state should be STARTUP
    TEST_ASSERT_EQUAL(STARTUP, get_flight_state());

    // Simulate calibration complete
    // TODO: Once calibration logic is integrated, test transition
    // set_flight_state(CALIBRATION);
    // TEST_ASSERT_EQUAL(CALIBRATION, get_flight_state());
  }

  void test_calibration_to_pad_idle_transition() {
    // TODO: Test calibration completion
    // set_flight_state(PAD_IDLE);
    // TEST_ASSERT_EQUAL(PAD_IDLE, get_flight_state());
  }

  void test_pad_idle_to_armed_transition() {
    // TODO: Test arm signal acceptance
    // TEST_ASSERT_EQUAL(PAD_IDLE, get_flight_state());
    // trigger_arm_signal();
    // process_flight_logic();
    // TEST_ASSERT_EQUAL(ARMED, get_flight_state());
  }

  void test_armed_to_boost_transition() {
    // TODO: Test launch detection (acceleration threshold)
    // set_flight_state(ARMED);
    // mock_sensor.setAcceleration(0, 0, 50);  // High acceleration
    // process_flight_logic();
    // TEST_ASSERT_EQUAL(BOOST, get_flight_state());
  }

  void test_boost_to_coast_transition() {
    // TODO: Test apogee detection
    // This is a critical state transition
    // set_flight_state(BOOST);
    // mock_sensor.setAcceleration(0, 0, -5);  // Deceleration
    // process_flight_logic();
    // TEST_ASSERT_EQUAL(COAST, get_flight_state());
  }

  void test_coast_to_apogee_transition() {
    // TODO: Test apogee detection triggers state change
    // set_flight_state(COAST);
    // Simulate deceleration to zero velocity
    // process_flight_logic();
    // TEST_ASSERT_EQUAL(APOGEE, get_flight_state());
  }

  void test_apogee_to_drogue_deploy_transition() {
    // TODO: Test drogue chute deployment at apogee
    // set_flight_state(APOGEE);
    // process_flight_logic();
    // TEST_ASSERT_EQUAL(DROGUE_DEPLOY, get_flight_state());
    // TEST_ASSERT_TRUE(is_drogue_fired());
  }

  void test_drogue_deploy_to_drogue_descent_transition() {
    // TODO: Test immediate transition after firing drogue
    // set_flight_state(DROGUE_DEPLOY);
    // process_flight_logic();
    // TEST_ASSERT_EQUAL(DROGUE_DESCENT, get_flight_state());
  }

  void test_main_deployment_trigger() {
    // TODO: Test main chute deploys at correct altitude
    // Depends on config: MAIN_DEPLOY_ALTITUDE
    // set_flight_state(DROGUE_DESCENT);
    // mock_sensor.setAcceleration(0, 0, -9.81);  // Free fall
    // Simulate descent to main deploy altitude
    // process_flight_logic();
    // TEST_ASSERT_EQUAL(MAIN_DEPLOY, get_flight_state());
    // TEST_ASSERT_TRUE(is_main_fired());
  }

  void test_landing_detection() {
    // TODO: Test landing detection (low velocity + low acceleration)
    // set_flight_state(MAIN_DESCENT);
    // mock_sensor.setAcceleration(0, 0, 0);  // No acceleration
    // Simulate several readings with zero velocity
    // process_flight_logic();
    // TEST_ASSERT_EQUAL(LANDED, get_flight_state());
  }

  void test_state_persistence_on_power_loss() {
    // TODO: Test state is saved to EEPROM
    // set_flight_state(BOOST);
    // save_state_to_eeprom();
    //
    // // Simulate power loss and restart
    // restore_state_from_eeprom();
    // TEST_ASSERT_EQUAL(BOOST, get_flight_state());
  }

  void test_invalid_state_transition_rejected() {
    // TODO: Verify system doesn't allow invalid transitions
    // e.g., BOOST → LANDED should not be allowed
    // set_flight_state(BOOST);
    // TEST_ASSERT_FALSE(can_transition_to(LANDED));
  }

  void test_error_state_entry() {
    // TODO: Test error state is entered on critical failures
    // set_flight_state(BOOST);
    // inject_sensor_failure();
    // process_flight_logic();
    // TEST_ASSERT_EQUAL(ERROR, get_flight_state());
  }

  void test_recovery_state() {
    // TODO: Test final RECOVERY state is reached after landing
    // set_flight_state(LANDED);
    // // Wait timeout period
    // process_flight_logic();
    // TEST_ASSERT_EQUAL(RECOVERY, get_flight_state());
  }

protected:
  MockIMUSensor mock_sensor;
};

// ============================================================================
// TEST RUNNER
// ============================================================================

extern "C" {

void setUp(void) {
  // Run before each test
}

void tearDown(void) {
  // Run after each test
}

// Note: Most tests are stubs because they depend on flight_logic.cpp
// being refactored to use the IMUManager interface. Once Phase 3 is
// complete with proper HAL integration in flight logic, these can be
// fully implemented.

void test_state_machine_startup(void) {
  // Verify system starts in correct state
  // This is a placeholder - actual test depends on architecture
  TEST_PASS();
}

void test_placeholder_unit_test(void) {
  // Placeholder to verify test framework works
  TEST_ASSERT_EQUAL(1, 1);
}

}  // extern "C"
