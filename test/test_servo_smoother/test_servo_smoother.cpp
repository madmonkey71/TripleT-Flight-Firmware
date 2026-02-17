#include <unity.h>
#include <math.h>
#include <cstring>

// ============================================================================
// SERVO SMOOTHER UNIT TESTS - Phase 6.2
//
// Tests for servo command smoothing and filtering:
// - Rate limiting (prevents excessive servo velocity)
// - Deadband filtering (eliminates small jitter)
// - Low-pass filtering (reduces high-frequency noise)
// - Batch processing (all three axes)
// - Filter type selection
// - Edge cases and numerical stability
// ============================================================================

extern "C" {

// Mock configuration for testing
#define SERVO_RATE_LIMIT_DPS 10.0f
#define SERVO_DEADBAND_DEG 0.5f
#define SERVO_LOWPASS_CUTOFF_HZ 2.0f

const float PI = 3.14159265359f;
const float TWO_PI = 2.0f * PI;

// Mock ServoSmoother for testing
class MockServoSmoother {
public:
    enum FilterType {
        RATE_LIMIT_ONLY = 0,
        DEADBAND_ONLY = 1,
        LOWPASS_ONLY = 2,
        RATE_LIMIT_THEN_LOWPASS = 3,
        FULL_FILTERING = 4
    };

    struct RateLimitParams {
        float max_rate_per_ms[3] = {0.1f, 0.1f, 0.15f};  // deg/ms
    } rate_limit;

    struct DeadbandParams {
        float deadband_deg = SERVO_DEADBAND_DEG;
    } deadband;

    struct LowPassParams {
        float cutoff_hz = SERVO_LOWPASS_CUTOFF_HZ;
        float prev_output[3] = {0.0f, 0.0f, 0.0f};
    } lowpass;

    FilterType active_filter = FULL_FILTERING;

    float apply_rate_limit(float desired, float current, float time_delta_ms, uint8_t axis_id) {
        if (time_delta_ms == 0) return desired;

        float max_change = rate_limit.max_rate_per_ms[axis_id] * time_delta_ms;
        float change = desired - current;

        if (change > max_change) {
            return current + max_change;
        } else if (change < -max_change) {
            return current - max_change;
        }
        return desired;
    }

    float apply_deadband(float command) {
        if (fabsf(command) < deadband.deadband_deg) {
            return 0.0f;
        }
        return command;
    }

    float apply_lowpass(float command, uint32_t time_delta_ms, uint8_t axis_id) {
        if (time_delta_ms == 0) return command;

        float alpha = calculate_alpha(time_delta_ms);
        alpha = fminf(alpha, 1.0f);

        float filtered = alpha * command + (1.0f - alpha) * lowpass.prev_output[axis_id];
        lowpass.prev_output[axis_id] = filtered;

        return filtered;
    }

    float calculate_alpha(uint32_t time_delta_ms) {
        if (lowpass.cutoff_hz <= 0.0f || time_delta_ms == 0) return 0.0f;

        float dt = time_delta_ms / 1000.0f;
        return TWO_PI * lowpass.cutoff_hz * dt;
    }

    float smooth(float desired_angle, float current_angle, uint32_t time_delta_ms, uint8_t axis_id) {
        float result = desired_angle;

        if (active_filter == RATE_LIMIT_ONLY || active_filter == RATE_LIMIT_THEN_LOWPASS ||
            active_filter == FULL_FILTERING) {
            result = apply_rate_limit(desired_angle, current_angle, time_delta_ms, axis_id);
        }

        if (active_filter == DEADBAND_ONLY || active_filter == FULL_FILTERING) {
            result = apply_deadband(result);
        }

        if (active_filter == LOWPASS_ONLY || active_filter == RATE_LIMIT_THEN_LOWPASS ||
            active_filter == FULL_FILTERING) {
            result = apply_lowpass(result, time_delta_ms, axis_id);
        }

        return result;
    }

    void reset() {
        memset(lowpass.prev_output, 0, sizeof(lowpass.prev_output));
    }
};

// ========================================================================
// RATE LIMITING TESTS
// ========================================================================

void test_rate_limit_small_change(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::RATE_LIMIT_ONLY;

    // Small change: 0.1 deg/ms * 10ms = 1 deg max change
    float result = smoother.smooth(0.5f, 0.0f, 10, 0);  // 0.5 deg change in 10ms

    // Should allow full 0.5 deg change since 0.5 < 1.0 max
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.5f, result);
}

void test_rate_limit_large_change_clamped(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::RATE_LIMIT_ONLY;

    // Large change exceeds rate limit (100 deg change in 10ms)
    // Max allowed: 0.1 deg/ms * 10ms = 1 deg
    float result = smoother.smooth(100.0f, 0.0f, 10, 0);

    // Should clamp to current + max_change = 0 + 1 = 1 deg
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, result);
}

void test_rate_limit_negative_change(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::RATE_LIMIT_ONLY;

    // Large negative change
    float result = smoother.smooth(-100.0f, 0.0f, 10, 0);

    // Should clamp to current - max_change = 0 - 1 = -1 deg
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -1.0f, result);
}

void test_rate_limit_zero_time_delta(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::RATE_LIMIT_ONLY;

    // Zero time delta should return desired value unchanged
    float result = smoother.smooth(45.0f, 0.0f, 0, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 45.0f, result);
}

void test_rate_limit_yaw_axis_faster(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::RATE_LIMIT_ONLY;

    // Yaw axis should have higher rate limit (0.15 deg/ms vs 0.1 deg/ms for pitch/roll)
    float pitch_result = smoother.smooth(20.0f, 0.0f, 100, 0);  // pitch axis: clamped to 10

    smoother.reset();
    float yaw_result = smoother.smooth(20.0f, 0.0f, 100, 2);    // yaw axis: clamped to 15

    // Yaw should allow more change (0.15 * 100 = 15 deg vs 0.1 * 100 = 10 deg)
    TEST_ASSERT_TRUE(yaw_result > pitch_result);
}

// ========================================================================
// DEADBAND FILTERING TESTS
// ========================================================================

void test_deadband_small_command_zeroed(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::DEADBAND_ONLY;

    // Command within deadband (0.5 deg)
    float result = smoother.smooth(0.3f, 0.0f, 10, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, result);
}

void test_deadband_at_threshold(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::DEADBAND_ONLY;

    // Command at exactly deadband threshold (0.5 is NOT < 0.5, so it passes through)
    float result = smoother.smooth(0.5f, 0.0f, 10, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, result);
}

void test_deadband_above_threshold(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::DEADBAND_ONLY;

    // Command just above deadband threshold
    float result = smoother.smooth(0.6f, 0.0f, 10, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.6f, result);
}

void test_deadband_negative_command(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::DEADBAND_ONLY;

    // Negative command within deadband
    float result = smoother.smooth(-0.3f, 0.0f, 10, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, result);
}

// ========================================================================
// LOW-PASS FILTERING TESTS
// ========================================================================

void test_lowpass_first_sample(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::LOWPASS_ONLY;
    smoother.reset();

    // First sample: output should be close to input
    float result = smoother.smooth(10.0f, 0.0f, 100, 0);

    // Alpha = TWO_PI * 2 * 0.1 ≈ 1.256, so output ≈ 1.256 * 10 + 0 * 0 = 12.56
    // (clamped to 1.0 in actual implementation)
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 10.0f, result);
}

void test_lowpass_convergence(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::LOWPASS_ONLY;
    smoother.reset();

    // Apply constant input multiple times
    float value = 10.0f;
    for (int i = 0; i < 20; i++) {
        value = smoother.smooth(10.0f, 0.0f, 100, 0);
    }

    // After many iterations with same input, should converge to input
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 10.0f, value);
}

void test_lowpass_step_response(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::LOWPASS_ONLY;
    smoother.reset();

    // Step change from 0 to 10 with small time step so alpha < 1
    float output1 = smoother.smooth(10.0f, 0.0f, 10, 0);  // 10ms

    // Repeated same input
    float output2 = smoother.smooth(10.0f, 0.0f, 10, 0);

    // First output should be partial (alpha < 1), second should converge further
    TEST_ASSERT_TRUE(output1 < 10.0f);  // Not fully converged
    TEST_ASSERT_TRUE(output2 > output1);  // Getting closer
}

void test_lowpass_zero_time_delta(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::LOWPASS_ONLY;
    smoother.reset();

    // Zero time delta should return input unchanged
    float result = smoother.smooth(5.0f, 0.0f, 0, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f, result);
}

// ========================================================================
// COMBINED FILTER TESTS
// ========================================================================

void test_full_filtering_large_step(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::FULL_FILTERING;
    smoother.reset();

    // Large command change: rate limiting should kick in
    // Then deadband applied, then low-pass
    float result = smoother.smooth(100.0f, 0.0f, 10, 0);

    // Rate limit clamps to 1 deg, deadband doesn't affect it, low-pass reduces it
    // Result should be less than 1 but greater than 0
    TEST_ASSERT_TRUE(result >= 0.0f);
    TEST_ASSERT_TRUE(result <= 1.0f);
}

void test_full_filtering_small_jitter(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::FULL_FILTERING;
    smoother.reset();

    // Small jitter commands (within deadband)
    float result1 = smoother.smooth(0.2f, 0.0f, 10, 0);   // Within deadband
    float result2 = smoother.smooth(-0.2f, 0.0f, 10, 0);  // Within deadband
    float result3 = smoother.smooth(0.3f, 0.0f, 10, 0);   // Within deadband

    // All should be zeroed by deadband, so final result near zero
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, result3);
}

void test_full_filtering_nominal_input(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::FULL_FILTERING;
    smoother.reset();

    // Nominal servo command: 5 degrees in 50ms
    float result = smoother.smooth(5.0f, 0.0f, 50, 0);

    // Should be below 5 due to filtering, but still significant
    TEST_ASSERT_TRUE(result > 0.5f);
    TEST_ASSERT_TRUE(result < 5.0f);
}

// ========================================================================
// EDGE CASES
// ========================================================================

void test_zero_input_zero_output(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::FULL_FILTERING;
    smoother.reset();

    float result = smoother.smooth(0.0f, 0.0f, 100, 0);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, result);
}

void test_very_large_time_delta(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::RATE_LIMIT_ONLY;

    // Very large time delta: rate limit should allow large changes
    float result = smoother.smooth(50.0f, 0.0f, 10000, 0);  // 10 seconds

    // Max change = 0.1 deg/ms * 10000ms = 1000 deg
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 50.0f, result);
}

void test_alternating_inputs(void) {
    MockServoSmoother smoother;
    smoother.active_filter = MockServoSmoother::RATE_LIMIT_ONLY;

    // Alternating between +10 and -10
    float result1 = smoother.smooth(10.0f, 0.0f, 10, 0);   // 0 → 1
    float result2 = smoother.smooth(-10.0f, result1, 10, 0); // 1 → 0
    float result3 = smoother.smooth(10.0f, result2, 10, 0);  // 0 → 1

    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, result1);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, result2);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, result3);
}

} // extern "C"

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_rate_limit_small_change);
    RUN_TEST(test_rate_limit_large_change_clamped);
    RUN_TEST(test_rate_limit_negative_change);
    RUN_TEST(test_rate_limit_zero_time_delta);
    RUN_TEST(test_rate_limit_yaw_axis_faster);
    RUN_TEST(test_deadband_small_command_zeroed);
    RUN_TEST(test_deadband_at_threshold);
    RUN_TEST(test_deadband_above_threshold);
    RUN_TEST(test_deadband_negative_command);
    RUN_TEST(test_lowpass_first_sample);
    RUN_TEST(test_lowpass_convergence);
    RUN_TEST(test_lowpass_step_response);
    RUN_TEST(test_lowpass_zero_time_delta);
    RUN_TEST(test_full_filtering_large_step);
    RUN_TEST(test_full_filtering_small_jitter);
    RUN_TEST(test_full_filtering_nominal_input);
    RUN_TEST(test_zero_input_zero_output);
    RUN_TEST(test_very_large_time_delta);
    RUN_TEST(test_alternating_inputs);
    return UNITY_END();
}
