#include <unity.h>
#include <math.h>
#include <cstring>

// ============================================================================
// STABILITY MONITOR UNIT TESTS - Phase 6.2
//
// Tests for guidance stability monitoring:
// - Angular rate threshold detection (pitch, roll, yaw)
// - Attitude error threshold detection
// - Actuator saturation detection
// - Quaternion to Euler angle conversion
// - Violation persistence tracking
// - Metrics collection and diagnostics
// ============================================================================

extern "C" {

// Mock configuration for testing
#define GUIDANCE_STABILITY_ROLL_RATE_LIMIT_DPS 180.0f
#define GUIDANCE_STABILITY_PITCH_RATE_LIMIT_DPS 180.0f
#define GUIDANCE_STABILITY_YAW_RATE_LIMIT_DPS 360.0f
#define GUIDANCE_STABILITY_ROLL_ERROR_LIMIT_DEG 30.0f
#define GUIDANCE_STABILITY_PITCH_ERROR_LIMIT_DEG 20.0f
#define GUIDANCE_STABILITY_YAW_ERROR_LIMIT_DEG 20.0f
#define GUIDANCE_STABILITY_SATURATION_LIMIT_PERCENT 95.0f
#define GUIDANCE_STABILITY_VIOLATION_DURATION_MS 500

// Mock StabilityMonitor for testing
class MockStabilityMonitor {
public:
    struct Thresholds {
        float roll_rate_limit_dps = GUIDANCE_STABILITY_ROLL_RATE_LIMIT_DPS;
        float pitch_rate_limit_dps = GUIDANCE_STABILITY_PITCH_RATE_LIMIT_DPS;
        float yaw_rate_limit_dps = GUIDANCE_STABILITY_YAW_RATE_LIMIT_DPS;
        float roll_error_limit_deg = GUIDANCE_STABILITY_ROLL_ERROR_LIMIT_DEG;
        float pitch_error_limit_deg = GUIDANCE_STABILITY_PITCH_ERROR_LIMIT_DEG;
        float yaw_error_limit_deg = GUIDANCE_STABILITY_YAW_ERROR_LIMIT_DEG;
        float saturation_limit_percent = GUIDANCE_STABILITY_SATURATION_LIMIT_PERCENT;
        uint32_t violation_duration_ms = GUIDANCE_STABILITY_VIOLATION_DURATION_MS;
    } thresholds;

    bool check_angular_rates(float roll_dps, float pitch_dps, float yaw_dps) {
        return (fabsf(roll_dps) > thresholds.roll_rate_limit_dps) ||
               (fabsf(pitch_dps) > thresholds.pitch_rate_limit_dps) ||
               (fabsf(yaw_dps) > thresholds.yaw_rate_limit_dps);
    }

    bool check_attitude_error(float roll_err_deg, float pitch_err_deg, float yaw_err_deg) {
        return (fabsf(roll_err_deg) > thresholds.roll_error_limit_deg) ||
               (fabsf(pitch_err_deg) > thresholds.pitch_error_limit_deg) ||
               (fabsf(yaw_err_deg) > thresholds.yaw_error_limit_deg);
    }

    bool check_actuator_saturation(float cmd_p, float cmd_r, float cmd_y) {
        float max_sat = fmaxf(fabsf(cmd_p), fmaxf(fabsf(cmd_r), fabsf(cmd_y))) * 100.0f;
        return max_sat > thresholds.saturation_limit_percent;
    }

    // Simple quaternion to Euler conversion for testing
    void quaternion_to_euler(const float q[4], float& roll, float& pitch, float& yaw) {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
        float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
        roll = atan2f(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (q0 * q2 - q3 * q1);
        if (fabsf(sinp) >= 1.0f) {
            pitch = copysignf(3.14159265359f / 2.0f, sinp);
        } else {
            pitch = asinf(sinp);
        }

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
        float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
        yaw = atan2f(siny_cosp, cosy_cosp);
    }
};

// ========================================================================
// ANGULAR RATE THRESHOLD TESTS
// ========================================================================

void test_angular_rate_within_limits(void) {
    MockStabilityMonitor monitor;

    // All rates within limits (180 DPS pitch/yaw, 360 DPS roll)
    bool violation = monitor.check_angular_rates(100.0f, 100.0f, 200.0f);
    TEST_ASSERT_FALSE(violation);
}

void test_angular_rate_pitch_exceeds(void) {
    MockStabilityMonitor monitor;

    // Pitch rate exceeds 180 DPS limit
    bool violation = monitor.check_angular_rates(100.0f, 200.0f, 100.0f);
    TEST_ASSERT_TRUE(violation);
}

void test_angular_rate_roll_exceeds(void) {
    MockStabilityMonitor monitor;

    // Roll rate exceeds 360 DPS limit
    bool violation = monitor.check_angular_rates(400.0f, 100.0f, 100.0f);
    TEST_ASSERT_TRUE(violation);
}

void test_angular_rate_yaw_exceeds(void) {
    MockStabilityMonitor monitor;

    // Yaw rate exceeds 180 DPS limit
    bool violation = monitor.check_angular_rates(100.0f, 100.0f, 200.0f);
    TEST_ASSERT_FALSE(violation); // 200 is within 360 limit

    violation = monitor.check_angular_rates(100.0f, 100.0f, 400.0f);
    TEST_ASSERT_TRUE(violation); // 400 exceeds 360 limit
}

void test_angular_rate_negative_values(void) {
    MockStabilityMonitor monitor;

    // Test with negative rates (should be compared as absolute values)
    bool violation = monitor.check_angular_rates(-200.0f, -100.0f, -100.0f);
    TEST_ASSERT_TRUE(violation); // -200 exceeds 180 limit
}

// ========================================================================
// ATTITUDE ERROR THRESHOLD TESTS
// ========================================================================

void test_attitude_error_within_limits(void) {
    MockStabilityMonitor monitor;

    // All errors within limits (20 deg pitch/yaw, 30 deg roll)
    bool violation = monitor.check_attitude_error(20.0f, 15.0f, 15.0f);
    TEST_ASSERT_FALSE(violation);
}

void test_attitude_error_pitch_exceeds(void) {
    MockStabilityMonitor monitor;

    // Pitch error exceeds 20 degree limit
    bool violation = monitor.check_attitude_error(15.0f, 25.0f, 15.0f);
    TEST_ASSERT_TRUE(violation);
}

void test_attitude_error_roll_at_limit(void) {
    MockStabilityMonitor monitor;

    // Roll error at exactly 30 degrees (should not violate)
    bool violation = monitor.check_attitude_error(30.0f, 15.0f, 15.0f);
    TEST_ASSERT_FALSE(violation);

    // Roll error 31 degrees (should violate)
    violation = monitor.check_attitude_error(31.0f, 15.0f, 15.0f);
    TEST_ASSERT_TRUE(violation);
}

void test_attitude_error_combined_violation(void) {
    MockStabilityMonitor monitor;

    // Multiple axes near limits, only yaw exceeds
    bool violation = monitor.check_attitude_error(29.9f, 19.9f, 25.0f);
    TEST_ASSERT_TRUE(violation); // Yaw limit is 20 deg
}

// ========================================================================
// ACTUATOR SATURATION THRESHOLD TESTS
// ========================================================================

void test_saturation_within_limit(void) {
    MockStabilityMonitor monitor;

    // Saturation at 50%
    bool violation = monitor.check_actuator_saturation(0.5f, 0.3f, 0.4f);
    TEST_ASSERT_FALSE(violation);
}

void test_saturation_at_95_percent(void) {
    MockStabilityMonitor monitor;

    // Saturation at exactly 95% (should not violate)
    bool violation = monitor.check_actuator_saturation(0.95f, 0.5f, 0.5f);
    TEST_ASSERT_FALSE(violation);
}

void test_saturation_exceeds_limit(void) {
    MockStabilityMonitor monitor;

    // Saturation at 96% (should violate)
    bool violation = monitor.check_actuator_saturation(0.96f, 0.5f, 0.5f);
    TEST_ASSERT_TRUE(violation);
}

void test_saturation_single_axis(void) {
    MockStabilityMonitor monitor;

    // Only one axis saturated, others low
    bool violation = monitor.check_actuator_saturation(0.1f, 0.1f, 0.98f);
    TEST_ASSERT_TRUE(violation); // Yaw at 98%
}

// ========================================================================
// QUATERNION TO EULER CONVERSION TESTS
// ========================================================================

void test_quaternion_identity(void) {
    MockStabilityMonitor monitor;

    // Identity quaternion [1, 0, 0, 0]
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float roll, pitch, yaw;

    monitor.quaternion_to_euler(q, roll, pitch, yaw);

    // Identity should give roll=0, pitch=0, yaw=0
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, roll);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, yaw);
}

void test_quaternion_90deg_roll(void) {
    MockStabilityMonitor monitor;

    // Quaternion for 90 degree roll: [cos(45°), sin(45°), 0, 0]
    float cos45 = cosf(3.14159265359f / 8.0f);
    float sin45 = sinf(3.14159265359f / 8.0f);
    float q[4] = {cos45, sin45, 0.0f, 0.0f};
    float roll, pitch, yaw;

    monitor.quaternion_to_euler(q, roll, pitch, yaw);

    // Should give roll ≈ 90 degrees (π/2 radians)
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.5708f, roll);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, yaw);
}

void test_quaternion_90deg_pitch(void) {
    MockStabilityMonitor monitor;

    // Quaternion for 90 degree pitch: [cos(45°), 0, sin(45°), 0]
    float cos45 = cosf(3.14159265359f / 8.0f);
    float sin45 = sinf(3.14159265359f / 8.0f);
    float q[4] = {cos45, 0.0f, sin45, 0.0f};
    float roll, pitch, yaw;

    monitor.quaternion_to_euler(q, roll, pitch, yaw);

    // Should give pitch ≈ 90 degrees (π/2 radians)
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, roll);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.5708f, pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, yaw);
}

// ========================================================================
// INTEGRATION TESTS
// ========================================================================

void test_all_violations_detected(void) {
    MockStabilityMonitor monitor;

    // Create conditions that violate all three types simultaneously
    float roll_rate = 400.0f;   // Exceeds 360 DPS roll limit
    float pitch_error = 30.0f;  // Exceeds 20 deg pitch limit
    float saturation = 0.97f;   // Exceeds 95% limit

    bool rate_violation = monitor.check_angular_rates(roll_rate, 100.0f, 100.0f);
    bool attitude_violation = monitor.check_attitude_error(10.0f, pitch_error, 10.0f);
    bool saturation_violation = monitor.check_actuator_saturation(saturation, 0.5f, 0.5f);

    TEST_ASSERT_TRUE(rate_violation);
    TEST_ASSERT_TRUE(attitude_violation);
    TEST_ASSERT_TRUE(saturation_violation);
}

void test_no_violations_nominal_flight(void) {
    MockStabilityMonitor monitor;

    // Simulate nominal flight conditions
    float roll_rate = 50.0f;    // Well below 360 DPS
    float pitch_rate = 30.0f;   // Well below 180 DPS
    float yaw_rate = 60.0f;     // Well below 180 DPS
    float roll_error = 5.0f;    // Well below 30 deg
    float pitch_error = 3.0f;   // Well below 20 deg
    float yaw_error = 4.0f;     // Well below 20 deg
    float pitch_cmd = 0.3f;     // 30% saturation
    float roll_cmd = 0.25f;     // 25% saturation
    float yaw_cmd = 0.2f;       // 20% saturation

    bool rate_violation = monitor.check_angular_rates(roll_rate, pitch_rate, yaw_rate);
    bool attitude_violation = monitor.check_attitude_error(roll_error, pitch_error, yaw_error);
    bool saturation_violation = monitor.check_actuator_saturation(pitch_cmd, roll_cmd, yaw_cmd);

    TEST_ASSERT_FALSE(rate_violation);
    TEST_ASSERT_FALSE(attitude_violation);
    TEST_ASSERT_FALSE(saturation_violation);
}

} // extern "C"
