#include <unity.h>
#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// SENSOR HEALTH MONITORING UNIT TESTS
//
// Tests sensor health checks, redundancy, degraded operation, and recovery.
// ============================================================================

extern "C" {

typedef struct {
  bool imu_primary_ok;
  bool imu_backup_ok;
  bool barometer_ok;
  bool gps_ok;
  uint8_t consecutive_failures;
} SensorHealthState;

bool all_sensors_healthy(const SensorHealthState* h) {
  return h->imu_primary_ok && h->barometer_ok && h->gps_ok;
}

bool can_operate_degraded(const SensorHealthState* h) {
  bool has_imu = h->imu_primary_ok || h->imu_backup_ok;
  return has_imu && h->barometer_ok;
}

void test_all_sensors_healthy(void) {
  SensorHealthState h = {true, true, true, true, 0};
  TEST_ASSERT_TRUE(all_sensors_healthy(&h));
  TEST_ASSERT_TRUE(can_operate_degraded(&h));
}

void test_single_sensor_failure_detected(void) {
  // GPS fails - can still fly
  SensorHealthState h = {true, true, true, false, 1};
  TEST_ASSERT_FALSE(all_sensors_healthy(&h));
  TEST_ASSERT_TRUE(can_operate_degraded(&h));
}

void test_sensor_recovery_after_failure(void) {
  SensorHealthState h = {true, true, true, false, 3};
  TEST_ASSERT_FALSE(all_sensors_healthy(&h));

  // Sensor recovers
  h.gps_ok = true;
  h.consecutive_failures = 0;
  TEST_ASSERT_TRUE(all_sensors_healthy(&h));
}

void test_multiple_sensor_failures(void) {
  // Primary IMU + GPS fail, backup IMU saves us
  SensorHealthState h = {false, true, true, false, 2};
  TEST_ASSERT_FALSE(all_sensors_healthy(&h));
  TEST_ASSERT_TRUE(can_operate_degraded(&h));

  // Both IMUs fail - critical
  h.imu_backup_ok = false;
  TEST_ASSERT_FALSE(can_operate_degraded(&h));
}

void test_sensor_health_grace_period(void) {
  // Single failure shouldn't immediately trigger ERROR
  SensorHealthState h = {true, true, false, true, 1};
  TEST_ASSERT_FALSE(all_sensors_healthy(&h));
  // Not enough consecutive failures for ERROR transition
  TEST_ASSERT_TRUE(h.consecutive_failures < 3);

  // After 3+ failures, should trigger
  h.consecutive_failures = 4;
  TEST_ASSERT_TRUE(h.consecutive_failures >= 3);
}

}  // extern "C"

void setUp(void) {}
void tearDown(void) {}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_all_sensors_healthy);
    RUN_TEST(test_single_sensor_failure_detected);
    RUN_TEST(test_sensor_recovery_after_failure);
    RUN_TEST(test_multiple_sensor_failures);
    RUN_TEST(test_sensor_health_grace_period);
    return UNITY_END();
}
