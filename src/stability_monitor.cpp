#include "stability_monitor.h"
#include "config.h"
#include <cstring>

// Constants (avoid conflicts with Arduino macro definitions)
static const float PI_CONST_SM = 3.14159265359f;
static const float DEG_TO_RAD_SM = PI_CONST_SM / 180.0f;
static const float RAD_TO_DEG_SM = 180.0f / PI_CONST_SM;

StabilityMonitor::StabilityMonitor()
    : violation_start_time_ms(0), in_violation(false) {
  memset(&current_metrics, 0, sizeof(current_metrics));
  current_metrics.violation_type = NULL;
}

void StabilityMonitor::init() {
  // Load thresholds from config.h
  thresholds.roll_rate_limit_dps = GUIDANCE_STABILITY_ROLL_RATE_LIMIT_DPS;
  thresholds.pitch_rate_limit_dps = GUIDANCE_STABILITY_PITCH_RATE_LIMIT_DPS;
  thresholds.yaw_rate_limit_dps = GUIDANCE_STABILITY_YAW_RATE_LIMIT_DPS;

  thresholds.roll_error_limit_deg = GUIDANCE_STABILITY_ROLL_ERROR_LIMIT_DEG;
  thresholds.pitch_error_limit_deg = GUIDANCE_STABILITY_PITCH_ERROR_LIMIT_DEG;
  thresholds.yaw_error_limit_deg = GUIDANCE_STABILITY_YAW_ERROR_LIMIT_DEG;

  thresholds.saturation_limit_percent = GUIDANCE_STABILITY_SATURATION_LIMIT_PERCENT;
  thresholds.violation_duration_ms = GUIDANCE_STABILITY_VIOLATION_DURATION_MS;

  Serial.println(F("[StabilityMonitor] Initialized with config thresholds"));
}

void StabilityMonitor::update(const float current_quat[4],
                              const float desired_quat[4],
                              const float gyro_rate_radps[3],
                              float cmd_pitch,
                              float cmd_roll,
                              float cmd_yaw,
                              uint32_t current_time_ms) {
  // Convert quaternions to Euler angles
  float current_roll, current_pitch, current_yaw;
  float desired_roll, desired_pitch, desired_yaw;

  quaternion_to_euler(current_quat, current_roll, current_pitch, current_yaw);
  quaternion_to_euler(desired_quat, desired_roll, desired_pitch, desired_yaw);

  // Store rates in degrees per second
  current_metrics.roll_rate_dps = gyro_rate_radps[0] * RAD_TO_DEG_SM;
  current_metrics.pitch_rate_dps = gyro_rate_radps[1] * RAD_TO_DEG_SM;
  current_metrics.yaw_rate_dps = gyro_rate_radps[2] * RAD_TO_DEG_SM;

  // Calculate attitude errors (differences)
  current_metrics.roll_error_deg = normalize_angle_deg(desired_roll - current_roll);
  current_metrics.pitch_error_deg = normalize_angle_deg(desired_pitch - current_pitch);
  current_metrics.yaw_error_deg = normalize_angle_deg(desired_yaw - current_yaw);

  // Store individual saturation percentages
  current_metrics.pitch_saturation_percent = fabsf(cmd_pitch) * 100.0f;
  current_metrics.roll_saturation_percent = fabsf(cmd_roll) * 100.0f;
  current_metrics.yaw_saturation_percent = fabsf(cmd_yaw) * 100.0f;

  // Overall saturation is max of all axes
  current_metrics.actuator_saturation_percent =
      fmaxf(current_metrics.pitch_saturation_percent,
      fmaxf(current_metrics.roll_saturation_percent,
            current_metrics.yaw_saturation_percent));

  // Check thresholds
  current_metrics.violation_bitmask = 0;
  current_metrics.violation_type = NULL;
  bool rate_violation = check_angular_rates(current_metrics.roll_rate_dps,
                                           current_metrics.pitch_rate_dps,
                                           current_metrics.yaw_rate_dps);
  if (rate_violation) {
    current_metrics.violation_bitmask |= 0b001;
    current_metrics.violation_type = "RATE";
  }

  bool attitude_violation = check_attitude_error(current_metrics.roll_error_deg,
                                               current_metrics.pitch_error_deg,
                                               current_metrics.yaw_error_deg);
  if (attitude_violation) {
    current_metrics.violation_bitmask |= 0b010;
    if (current_metrics.violation_type == NULL) {
      current_metrics.violation_type = "ATTITUDE";
    }
  }

  bool saturation_violation = check_actuator_saturation(cmd_pitch, cmd_roll, cmd_yaw);
  if (saturation_violation) {
    current_metrics.violation_bitmask |= 0b100;
    if (current_metrics.violation_type == NULL) {
      current_metrics.violation_type = "SATURATION";
    }
  }

  // Update overall stability state
  update_violation_state(current_time_ms);
}

StabilityMonitor::StabilityMetrics StabilityMonitor::getMetrics() const {
  return current_metrics;
}

bool StabilityMonitor::isStabilityViolation() const {
  return in_violation;
}

bool StabilityMonitor::isViolationPersistent() const {
  if (!in_violation) return false;
  return (violation_start_time_ms > 0) &&
         ((millis() - violation_start_time_ms) >= thresholds.violation_duration_ms);
}

uint32_t StabilityMonitor::getViolationDuration() const {
  if (!in_violation || violation_start_time_ms == 0) return 0;
  return millis() - violation_start_time_ms;
}

void StabilityMonitor::resetViolation() {
  in_violation = false;
  violation_start_time_ms = 0;
  current_metrics.violation_type = NULL;
  current_metrics.violation_bitmask = 0;
}

void StabilityMonitor::printDiagnostics() const {
  Serial.println(F("=== STABILITY MONITOR DIAGNOSTICS ==="));
  Serial.print(F("State: "));
  Serial.println(in_violation ? F("VIOLATION") : F("OK"));
  Serial.print(F("Duration: "));
  Serial.print(getViolationDuration());
  Serial.println(F(" ms"));

  Serial.println(F("Angular Rates (DPS):"));
  Serial.print(F("  Roll:  "));
  Serial.print(current_metrics.roll_rate_dps, 2);
  Serial.print(F(" / "));
  Serial.println(thresholds.roll_rate_limit_dps);

  Serial.print(F("  Pitch: "));
  Serial.print(current_metrics.pitch_rate_dps, 2);
  Serial.print(F(" / "));
  Serial.println(thresholds.pitch_rate_limit_dps);

  Serial.print(F("  Yaw:   "));
  Serial.print(current_metrics.yaw_rate_dps, 2);
  Serial.print(F(" / "));
  Serial.println(thresholds.yaw_rate_limit_dps);

  Serial.println(F("Attitude Errors (DEG):"));
  Serial.print(F("  Roll:  "));
  Serial.print(current_metrics.roll_error_deg, 2);
  Serial.print(F(" / "));
  Serial.println(thresholds.roll_error_limit_deg);

  Serial.print(F("  Pitch: "));
  Serial.print(current_metrics.pitch_error_deg, 2);
  Serial.print(F(" / "));
  Serial.println(thresholds.pitch_error_limit_deg);

  Serial.print(F("  Yaw:   "));
  Serial.print(current_metrics.yaw_error_deg, 2);
  Serial.print(F(" / "));
  Serial.println(thresholds.yaw_error_limit_deg);

  Serial.println(F("Saturation (%):"));
  Serial.print(F("  Overall: "));
  Serial.print(current_metrics.actuator_saturation_percent, 1);
  Serial.print(F(" / "));
  Serial.println(thresholds.saturation_limit_percent);

  Serial.print(F("  Pitch: "));
  Serial.print(current_metrics.pitch_saturation_percent, 1);
  Serial.print(F(", Roll: "));
  Serial.print(current_metrics.roll_saturation_percent, 1);
  Serial.print(F(", Yaw: "));
  Serial.println(current_metrics.yaw_saturation_percent, 1);

  Serial.print(F("Violation Type: "));
  if (current_metrics.violation_type) {
    Serial.println(current_metrics.violation_type);
  } else {
    Serial.println(F("NONE"));
  }
  Serial.println(F("====================================="));
}

// Private helper methods

void StabilityMonitor::quaternion_to_euler(const float q[4],
                                          float& roll,
                                          float& pitch,
                                          float& yaw) const {
  // q = [q0, q1, q2, q3] = [w, x, y, z]
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

  // Roll (x-axis rotation)
  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  roll = atan2f(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (fabsf(sinp) >= 1.0f) {
    pitch = copysignf(PI / 2.0f, sinp);
  } else {
    pitch = asinf(sinp);
  }

  // Yaw (z-axis rotation)
  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  yaw = atan2f(siny_cosp, cosy_cosp);
}

float StabilityMonitor::rad_to_deg(float rad) const {
  return rad * RAD_TO_DEG_SM;
}

float StabilityMonitor::normalize_angle_deg(float deg) const {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

bool StabilityMonitor::check_angular_rates(float roll_dps, float pitch_dps, float yaw_dps) {
  return (fabsf(roll_dps) > thresholds.roll_rate_limit_dps) ||
         (fabsf(pitch_dps) > thresholds.pitch_rate_limit_dps) ||
         (fabsf(yaw_dps) > thresholds.yaw_rate_limit_dps);
}

bool StabilityMonitor::check_attitude_error(float roll_err_deg, float pitch_err_deg, float yaw_err_deg) {
  return (fabsf(roll_err_deg) > thresholds.roll_error_limit_deg) ||
         (fabsf(pitch_err_deg) > thresholds.pitch_error_limit_deg) ||
         (fabsf(yaw_err_deg) > thresholds.yaw_error_limit_deg);
}

bool StabilityMonitor::check_actuator_saturation(float cmd_p, float cmd_r, float cmd_y) {
  float max_sat = fmaxf(fabsf(cmd_p), fmaxf(fabsf(cmd_r), fabsf(cmd_y))) * 100.0f;
  return max_sat > thresholds.saturation_limit_percent;
}

void StabilityMonitor::update_violation_state(uint32_t current_time_ms) {
  if (current_metrics.violation_bitmask != 0) {
    // Violation detected
    if (!in_violation) {
      // First frame of violation
      in_violation = true;
      violation_start_time_ms = current_time_ms;
    }
  } else {
    // No violation
    if (in_violation) {
      // Violation cleared
      in_violation = false;
      violation_start_time_ms = 0;
    }
  }
}
