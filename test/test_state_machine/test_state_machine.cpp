#include <unity.h>
#include <stdint.h>

// ============================================================================
// STATE MACHINE UNIT TESTS
//
// Tests flight state enum definitions and basic state transition logic.
// Full integration tests require hardware (flight_logic.cpp depends on Arduino).
// ============================================================================

extern "C" {

// Replicate the FlightState enum for native testing
typedef enum {
    STARTUP = 0,
    CALIBRATION,
    PAD_IDLE,
    ARMED,
    BOOST,
    COAST,
    APOGEE,
    DROGUE_DEPLOY,
    DROGUE_DESCENT,
    MAIN_DEPLOY,
    MAIN_DESCENT,
    LANDED,
    RECOVERY,
    ERROR
} FlightState;

const char* getStateName(FlightState state) {
    switch (state) {
        case STARTUP: return "STARTUP";
        case CALIBRATION: return "CALIBRATION";
        case PAD_IDLE: return "PAD_IDLE";
        case ARMED: return "ARMED";
        case BOOST: return "BOOST";
        case COAST: return "COAST";
        case APOGEE: return "APOGEE";
        case DROGUE_DEPLOY: return "DROGUE_DEPLOY";
        case DROGUE_DESCENT: return "DROGUE_DESCENT";
        case MAIN_DEPLOY: return "MAIN_DEPLOY";
        case MAIN_DESCENT: return "MAIN_DESCENT";
        case LANDED: return "LANDED";
        case RECOVERY: return "RECOVERY";
        case ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

bool isValidTransition(FlightState from, FlightState to) {
    // Define valid state transitions
    switch (from) {
        case STARTUP: return to == CALIBRATION || to == ERROR;
        case CALIBRATION: return to == PAD_IDLE || to == ERROR;
        case PAD_IDLE: return to == ARMED || to == ERROR;
        case ARMED: return to == BOOST || to == PAD_IDLE || to == ERROR;
        case BOOST: return to == COAST || to == ERROR;
        case COAST: return to == APOGEE || to == ERROR;
        case APOGEE: return to == DROGUE_DEPLOY || to == MAIN_DEPLOY || to == DROGUE_DESCENT || to == ERROR;
        case DROGUE_DEPLOY: return to == DROGUE_DESCENT || to == ERROR;
        case DROGUE_DESCENT: return to == MAIN_DEPLOY || to == LANDED || to == ERROR;
        case MAIN_DEPLOY: return to == MAIN_DESCENT || to == ERROR;
        case MAIN_DESCENT: return to == LANDED || to == ERROR;
        case LANDED: return to == RECOVERY || to == ERROR;
        case RECOVERY: return false; // Terminal state
        case ERROR: return to == PAD_IDLE || to == CALIBRATION; // Recovery from error
        default: return false;
    }
}

void setUp(void) {}
void tearDown(void) {}

void test_state_enum_values(void) {
    TEST_ASSERT_EQUAL(0, STARTUP);
    TEST_ASSERT_EQUAL(13, ERROR);
}

void test_state_names(void) {
    TEST_ASSERT_EQUAL_STRING("STARTUP", getStateName(STARTUP));
    TEST_ASSERT_EQUAL_STRING("BOOST", getStateName(BOOST));
    TEST_ASSERT_EQUAL_STRING("LANDED", getStateName(LANDED));
    TEST_ASSERT_EQUAL_STRING("ERROR", getStateName(ERROR));
}

void test_valid_forward_transitions(void) {
    TEST_ASSERT_TRUE(isValidTransition(STARTUP, CALIBRATION));
    TEST_ASSERT_TRUE(isValidTransition(CALIBRATION, PAD_IDLE));
    TEST_ASSERT_TRUE(isValidTransition(PAD_IDLE, ARMED));
    TEST_ASSERT_TRUE(isValidTransition(ARMED, BOOST));
    TEST_ASSERT_TRUE(isValidTransition(BOOST, COAST));
    TEST_ASSERT_TRUE(isValidTransition(COAST, APOGEE));
    TEST_ASSERT_TRUE(isValidTransition(APOGEE, DROGUE_DEPLOY));
    TEST_ASSERT_TRUE(isValidTransition(DROGUE_DEPLOY, DROGUE_DESCENT));
    TEST_ASSERT_TRUE(isValidTransition(MAIN_DEPLOY, MAIN_DESCENT));
    TEST_ASSERT_TRUE(isValidTransition(MAIN_DESCENT, LANDED));
    TEST_ASSERT_TRUE(isValidTransition(LANDED, RECOVERY));
}

void test_invalid_backward_transitions(void) {
    TEST_ASSERT_FALSE(isValidTransition(COAST, BOOST));
    TEST_ASSERT_FALSE(isValidTransition(LANDED, BOOST));
    TEST_ASSERT_FALSE(isValidTransition(APOGEE, ARMED));
    TEST_ASSERT_FALSE(isValidTransition(RECOVERY, STARTUP));
}

void test_error_transitions(void) {
    // Any state can go to ERROR
    TEST_ASSERT_TRUE(isValidTransition(STARTUP, ERROR));
    TEST_ASSERT_TRUE(isValidTransition(BOOST, ERROR));
    TEST_ASSERT_TRUE(isValidTransition(COAST, ERROR));
    TEST_ASSERT_TRUE(isValidTransition(LANDED, ERROR));

    // ERROR can recover to PAD_IDLE or CALIBRATION
    TEST_ASSERT_TRUE(isValidTransition(ERROR, PAD_IDLE));
    TEST_ASSERT_TRUE(isValidTransition(ERROR, CALIBRATION));
}

void test_apogee_deployment_options(void) {
    // From APOGEE: can go to drogue, main, or descent
    TEST_ASSERT_TRUE(isValidTransition(APOGEE, DROGUE_DEPLOY));
    TEST_ASSERT_TRUE(isValidTransition(APOGEE, MAIN_DEPLOY));
    TEST_ASSERT_TRUE(isValidTransition(APOGEE, DROGUE_DESCENT));
}

}  // extern "C"

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_state_enum_values);
    RUN_TEST(test_state_names);
    RUN_TEST(test_valid_forward_transitions);
    RUN_TEST(test_invalid_backward_transitions);
    RUN_TEST(test_error_transitions);
    RUN_TEST(test_apogee_deployment_options);
    return UNITY_END();
}
