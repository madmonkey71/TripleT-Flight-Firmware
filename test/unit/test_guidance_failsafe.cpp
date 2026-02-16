#include <unity.h>
#include <math.h>
#include <cstring>

// ============================================================================
// GUIDANCE FAILSAFE UNIT TESTS - Phase 6.2
//
// Tests for three-level failsafe escalation system:
// - Failsafe activation/deactivation
// - Gain reduction (gradual from 100% to 30%)
// - Passive mode engagement (center servos)
// - Three-level escalation timing
// - Recovery behavior
// - Diagnostic output validation
// ============================================================================

extern "C" {

// Mock configuration
#define GUIDANCE_FAILSAFE_LEVEL1_MS 1000
#define GUIDANCE_FAILSAFE_LEVEL2_MS 2000
#define GUIDANCE_FAILSAFE_LEVEL3_MS 5000
#define GUIDANCE_FAILSAFE_MIN_GAIN 0.3f

// Mock Failsafe State for testing
struct MockFailsafeState {
    bool failsafe_active = false;
    uint32_t failsafe_start_ms = 0;
    float gain_reduction_factor = 1.0f;
    bool passive_mode_active = false;
    uint8_t escalation_level = 0;
};

// Mock GuidanceFailsafe for testing
class MockGuidanceFailsafe {
private:
    MockFailsafeState state;
    bool stability_violated = false;
    uint32_t violation_start_ms = 0;

public:
    MockGuidanceFailsafe() {
        reset_failsafe();
    }

    void reset_failsafe() {
        state.failsafe_active = false;
        state.failsafe_start_ms = 0;
        state.gain_reduction_factor = 1.0f;
        state.passive_mode_active = false;
        state.escalation_level = 0;
        stability_violated = false;
        violation_start_ms = 0;
    }

    void trigger_stability_violation(uint32_t current_time_ms) {
        stability_violated = true;
        if (violation_start_ms == 0) {
            violation_start_ms = current_time_ms;
        }
    }

    void clear_stability_violation() {
        stability_violated = false;
        violation_start_ms = 0;
    }

    bool check_failsafe(uint32_t current_time_ms) {
        if (!stability_violated) {
            // Stability restored
            if (state.failsafe_active && state.gain_reduction_factor < 1.0f) {
                // Recover gains at 5% per 100ms = 50% per second
                state.gain_reduction_factor += 0.05f;
                if (state.gain_reduction_factor > 1.0f) {
                    state.gain_reduction_factor = 1.0f;
                }
            }

            if (state.escalation_level < 2 && state.gain_reduction_factor >= 1.0f) {
                state.failsafe_active = false;
                state.failsafe_start_ms = 0;
                state.escalation_level = 0;
            }
            return false;
        }

        // Stability compromised
        if (!state.failsafe_active) {
            state.failsafe_active = true;
            state.failsafe_start_ms = current_time_ms;
            state.gain_reduction_factor = 1.0f;
            state.escalation_level = 0;
        }

        uint32_t failsafe_duration = current_time_ms - state.failsafe_start_ms;

        // Level 1: Reduce gains
        if (failsafe_duration >= GUIDANCE_FAILSAFE_LEVEL1_MS && state.escalation_level == 0) {
            state.escalation_level = 1;
        }

        if (state.escalation_level == 1) {
            if (state.gain_reduction_factor > GUIDANCE_FAILSAFE_MIN_GAIN) {
                state.gain_reduction_factor -= 0.02f;
                if (state.gain_reduction_factor < GUIDANCE_FAILSAFE_MIN_GAIN) {
                    state.gain_reduction_factor = GUIDANCE_FAILSAFE_MIN_GAIN;
                }
            }
        }

        // Level 2: Passive mode
        if (failsafe_duration >= GUIDANCE_FAILSAFE_LEVEL2_MS && state.escalation_level < 2) {
            state.escalation_level = 2;
            state.passive_mode_active = true;
        }

        // Level 3: ERROR state
        if (failsafe_duration >= GUIDANCE_FAILSAFE_LEVEL3_MS && state.escalation_level < 3) {
            state.escalation_level = 3;
            return true;  // Signal to transition to ERROR
        }

        return state.failsafe_active && state.escalation_level < 3;
    }

    // Getter methods
    float get_gain_factor() const { return state.gain_reduction_factor; }
    bool is_passive_mode() const { return state.passive_mode_active; }
    bool is_active() const { return state.failsafe_active; }
    uint8_t get_level() const { return state.escalation_level; }
};

// ========================================================================
// FAILSAFE ACTIVATION TESTS
// ========================================================================

void test_failsafe_inactive_on_init(void) {
    MockGuidanceFailsafe failsafe;

    bool active = failsafe.is_active();
    TEST_ASSERT_FALSE(active);
}

void test_failsafe_activates_on_violation(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    failsafe.trigger_stability_violation(now_ms);
    failsafe.check_failsafe(now_ms);

    bool active = failsafe.is_active();
    TEST_ASSERT_TRUE(active);
}

void test_failsafe_deactivates_on_recovery(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    // Activate failsafe
    failsafe.trigger_stability_violation(now_ms);
    failsafe.check_failsafe(now_ms);
    TEST_ASSERT_TRUE(failsafe.is_active());

    // Let it run a bit (gain reduction)
    for (int i = 0; i < 150; i++) {
        failsafe.check_failsafe(now_ms + i * 100);
    }

    // Clear violation
    failsafe.clear_stability_violation();

    // Recovery should happen
    for (int i = 0; i < 30; i++) {
        failsafe.check_failsafe(now_ms + 15000 + i * 100);
    }

    bool active = failsafe.is_active();
    bool level = failsafe.get_level();
    TEST_ASSERT_FALSE(active);
    TEST_ASSERT_EQUAL_INT(0, level);
}

// ========================================================================
// GAIN REDUCTION TESTS
// ========================================================================

void test_gain_factor_initial_unity(void) {
    MockGuidanceFailsafe failsafe;

    float gain = failsafe.get_gain_factor();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, gain);
}

void test_gain_factor_remains_unity_no_violation(void) {
    MockGuidanceFailsafe failsafe;

    for (int i = 0; i < 100; i++) {
        failsafe.check_failsafe(1000 + i * 100);
    }

    float gain = failsafe.get_gain_factor();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, gain);
}

void test_gain_reduction_starts_at_level1(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    failsafe.trigger_stability_violation(now_ms);

    // Check before level 1
    failsafe.check_failsafe(now_ms + 500);
    float gain_before = failsafe.get_gain_factor();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, gain_before);

    // Check at level 1
    failsafe.check_failsafe(now_ms + 1000);
    failsafe.check_failsafe(now_ms + 1100);
    float gain_at_level1 = failsafe.get_gain_factor();
    TEST_ASSERT_TRUE(gain_at_level1 < 1.0f);
}

void test_gain_reduction_gradual(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    failsafe.trigger_stability_violation(now_ms);

    // Reach level 1
    failsafe.check_failsafe(now_ms + 1000);

    // Sample gain reduction over time
    float gains[10];
    for (int i = 0; i < 10; i++) {
        failsafe.check_failsafe(now_ms + 1000 + (i + 1) * 100);
        gains[i] = failsafe.get_gain_factor();
    }

    // Gains should be decreasing
    for (int i = 1; i < 10; i++) {
        TEST_ASSERT_TRUE(gains[i] <= gains[i - 1]);
    }
}

void test_gain_reduction_minimum_limit(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    failsafe.trigger_stability_violation(now_ms);

    // Run for long time to reach minimum
    for (int i = 0; i < 500; i++) {
        failsafe.check_failsafe(now_ms + 1000 + i * 100);
    }

    float gain = failsafe.get_gain_factor();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, GUIDANCE_FAILSAFE_MIN_GAIN, gain);
    TEST_ASSERT_TRUE(gain >= GUIDANCE_FAILSAFE_MIN_GAIN);
}

void test_gain_recovery_after_violation_clears(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    failsafe.trigger_stability_violation(now_ms);

    // Reduce gains
    for (int i = 0; i < 100; i++) {
        failsafe.check_failsafe(now_ms + 1000 + i * 100);
    }

    float gain_reduced = failsafe.get_gain_factor();
    TEST_ASSERT_TRUE(gain_reduced < 1.0f);

    // Clear violation
    failsafe.clear_stability_violation();

    // Check recovery
    failsafe.check_failsafe(now_ms + 10000);
    float gain_after = failsafe.get_gain_factor();

    TEST_ASSERT_TRUE(gain_after > gain_reduced);
}

// ========================================================================
// ESCALATION LEVEL TESTS
// ========================================================================

void test_escalation_level_0_initial(void) {
    MockGuidanceFailsafe failsafe;

    uint8_t level = failsafe.get_level();
    TEST_ASSERT_EQUAL_INT(0, level);
}

void test_escalation_to_level1_at_1000ms(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    failsafe.trigger_stability_violation(now_ms);
    failsafe.check_failsafe(now_ms + 500);
    uint8_t level_before = failsafe.get_level();
    TEST_ASSERT_EQUAL_INT(0, level_before);

    failsafe.check_failsafe(now_ms + 1000);
    uint8_t level_at = failsafe.get_level();
    TEST_ASSERT_EQUAL_INT(1, level_at);
}

void test_escalation_to_level2_at_2000ms(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    failsafe.trigger_stability_violation(now_ms);

    failsafe.check_failsafe(now_ms + 1000);
    uint8_t level_before = failsafe.get_level();
    TEST_ASSERT_EQUAL_INT(1, level_before);

    failsafe.check_failsafe(now_ms + 2000);
    uint8_t level_at = failsafe.get_level();
    TEST_ASSERT_EQUAL_INT(2, level_at);
}

void test_escalation_to_level3_at_5000ms(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    failsafe.trigger_stability_violation(now_ms);

    failsafe.check_failsafe(now_ms + 2000);
    uint8_t level_before = failsafe.get_level();
    TEST_ASSERT_EQUAL_INT(2, level_before);

    bool error_triggered = failsafe.check_failsafe(now_ms + 5000);
    uint8_t level_at = failsafe.get_level();
    TEST_ASSERT_EQUAL_INT(3, level_at);
    TEST_ASSERT_TRUE(error_triggered);
}

// ========================================================================
// PASSIVE MODE TESTS
// ========================================================================

void test_passive_mode_inactive_initially(void) {
    MockGuidanceFailsafe failsafe;

    bool passive = failsafe.is_passive_mode();
    TEST_ASSERT_FALSE(passive);
}

void test_passive_mode_activates_at_level2(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    failsafe.trigger_stability_violation(now_ms);

    failsafe.check_failsafe(now_ms + 1500);
    bool passive_before = failsafe.is_passive_mode();
    TEST_ASSERT_FALSE(passive_before);

    failsafe.check_failsafe(now_ms + 2000);
    bool passive_at = failsafe.is_passive_mode();
    TEST_ASSERT_TRUE(passive_at);
}

void test_passive_mode_stays_active(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    failsafe.trigger_stability_violation(now_ms);

    failsafe.check_failsafe(now_ms + 2000);
    bool passive_after_level2 = failsafe.is_passive_mode();
    TEST_ASSERT_TRUE(passive_after_level2);

    // Should remain active through level 3
    failsafe.check_failsafe(now_ms + 5000);
    bool passive_after_level3 = failsafe.is_passive_mode();
    TEST_ASSERT_TRUE(passive_after_level3);
}

// ========================================================================
// RECOVERY BEHAVIOR TESTS
// ========================================================================

void test_recovery_restores_gains(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    // Trigger violation and reach level 2
    failsafe.trigger_stability_violation(now_ms);
    for (int i = 0; i < 100; i++) {
        failsafe.check_failsafe(now_ms + 1000 + i * 100);
    }

    float gain_at_level2 = failsafe.get_gain_factor();
    TEST_ASSERT_TRUE(gain_at_level2 < 1.0f);

    // Clear violation and let recovery happen
    failsafe.clear_stability_violation();
    for (int i = 0; i < 30; i++) {
        failsafe.check_failsafe(now_ms + 11000 + i * 100);
    }

    float gain_after_recovery = failsafe.get_gain_factor();
    TEST_ASSERT_TRUE(gain_after_recovery > gain_at_level2);
}

void test_recovery_returns_to_unity_gain(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    failsafe.trigger_stability_violation(now_ms);
    for (int i = 0; i < 100; i++) {
        failsafe.check_failsafe(now_ms + 1000 + i * 100);
    }

    failsafe.clear_stability_violation();
    for (int i = 0; i < 100; i++) {
        failsafe.check_failsafe(now_ms + 11000 + i * 100);
    }

    float gain = failsafe.get_gain_factor();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, gain);
}

void test_recovery_only_in_level_0_or_1(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    failsafe.trigger_stability_violation(now_ms);
    failsafe.check_failsafe(now_ms + 2000);  // Reach level 2
    TEST_ASSERT_EQUAL_INT(2, failsafe.get_level());

    failsafe.clear_stability_violation();
    failsafe.check_failsafe(now_ms + 3000);

    // Level should not go back to 0
    uint8_t level = failsafe.get_level();
    TEST_ASSERT_EQUAL_INT(2, level);
}

// ========================================================================
// TIMING PRECISION TESTS
// ========================================================================

void test_level_transitions_at_exact_boundaries(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t start_ms = 0;

    failsafe.trigger_stability_violation(start_ms);

    // Test level 1 boundary
    failsafe.check_failsafe(start_ms + 999);  // Just before
    TEST_ASSERT_EQUAL_INT(0, failsafe.get_level());

    failsafe.check_failsafe(start_ms + 1000);  // At boundary
    TEST_ASSERT_EQUAL_INT(1, failsafe.get_level());

    // Test level 2 boundary
    failsafe.check_failsafe(start_ms + 1999);  // Just before
    TEST_ASSERT_EQUAL_INT(1, failsafe.get_level());

    failsafe.check_failsafe(start_ms + 2000);  // At boundary
    TEST_ASSERT_EQUAL_INT(2, failsafe.get_level());
}

void test_rapid_failsafe_checks_not_double_triggered(void) {
    MockGuidanceFailsafe failsafe;
    uint32_t now_ms = 1000;

    failsafe.trigger_stability_violation(now_ms);

    failsafe.check_failsafe(now_ms);
    uint8_t level1 = failsafe.get_level();

    // Check many times at same time
    for (int i = 0; i < 100; i++) {
        failsafe.check_failsafe(now_ms + 1000);
    }

    uint8_t level2 = failsafe.get_level();
    TEST_ASSERT_EQUAL_INT(level1, level2);  // Should match, not double-trigger
}

} // extern "C"
