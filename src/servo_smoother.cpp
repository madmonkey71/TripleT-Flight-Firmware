#include "servo_smoother.h"
#include "config.h"
#include <cstring>
#include <cmath>

static const float PI_CONST = 3.14159265359f;
static const float TWO_PI_CONST = 2.0f * PI_CONST;

ServoSmoother::ServoSmoother() : active_filter(FULL_FILTERING) {
  memset(last_output, 0, sizeof(last_output));
  memset(lowpass.prev_output, 0, sizeof(lowpass.prev_output));
  lowpass.cutoff_hz = 2.0f;
  lowpass.alpha = 0.0f;

  // Default rate limits (10 deg/100ms)
  rate_limit.max_rate_per_ms[0] = 0.1f;  // pitch
  rate_limit.max_rate_per_ms[1] = 0.1f;  // roll
  rate_limit.max_rate_per_ms[2] = 0.15f; // yaw (higher)

  // Default deadband
  deadband.deadband_deg = 0.5f;
}

void ServoSmoother::init(FilterType filter_type) {
  active_filter = filter_type;

  // Load parameters from config.h
  rate_limit.max_rate_per_ms[0] = SERVO_RATE_LIMIT_DPS / 100.0f;
  rate_limit.max_rate_per_ms[1] = SERVO_RATE_LIMIT_DPS / 100.0f;
  rate_limit.max_rate_per_ms[2] = (SERVO_RATE_LIMIT_DPS * 1.5f) / 100.0f; // Yaw faster

  deadband.deadband_deg = SERVO_DEADBAND_DEG;
  lowpass.cutoff_hz = SERVO_LOWPASS_CUTOFF_HZ;

  Serial.print(F("[ServoSmoother] Initialized with filter type: "));
  Serial.println((int)filter_type);
}

float ServoSmoother::smooth(float desired_angle,
                            float current_angle,
                            uint32_t time_delta_ms,
                            uint8_t axis_id) {
  float result = desired_angle;

  // Apply filters in sequence based on filter type
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

  // Store as last output
  last_output[axis_id] = result;
  return result;
}

void ServoSmoother::smoothBatch(const float desired[3],
                                const float current[3],
                                uint32_t time_delta_ms,
                                float output[3]) {
  for (int i = 0; i < 3; i++) {
    output[i] = smooth(desired[i], current[i], time_delta_ms, i);
  }
}

void ServoSmoother::reset() {
  memset(last_output, 0, sizeof(last_output));
  memset(lowpass.prev_output, 0, sizeof(lowpass.prev_output));
  Serial.println(F("[ServoSmoother] Filters reset"));
}

float ServoSmoother::getLastOutput(uint8_t axis_id) const {
  if (axis_id < 3) {
    return last_output[axis_id];
  }
  return 0.0f;
}

void ServoSmoother::printParameters() const {
  Serial.println(F("=== SERVO SMOOTHER PARAMETERS ==="));
  Serial.print(F("Filter Type: "));
  Serial.println((int)active_filter);

  Serial.println(F("Rate Limits (deg/ms):"));
  Serial.print(F("  Pitch: "));
  Serial.println(rate_limit.max_rate_per_ms[0], 4);
  Serial.print(F("  Roll:  "));
  Serial.println(rate_limit.max_rate_per_ms[1], 4);
  Serial.print(F("  Yaw:   "));
  Serial.println(rate_limit.max_rate_per_ms[2], 4);

  Serial.print(F("Deadband: "));
  Serial.print(deadband.deadband_deg, 2);
  Serial.println(F(" degrees"));

  Serial.print(F("Low-Pass Cutoff: "));
  Serial.print(lowpass.cutoff_hz, 1);
  Serial.println(F(" Hz"));
  Serial.println(F("=================================="));
}

// Private helper methods

float ServoSmoother::apply_rate_limit(float desired,
                                      float current,
                                      float time_delta_ms,
                                      uint8_t axis_id) {
  if (time_delta_ms == 0) return desired;

  float max_change = rate_limit.max_rate_per_ms[axis_id] * time_delta_ms;
  float change = desired - current;

  // Clamp change to max rate
  if (change > max_change) {
    return current + max_change;
  } else if (change < -max_change) {
    return current - max_change;
  }

  return desired;
}

float ServoSmoother::apply_deadband(float command) {
  // If command is within deadband of zero, return zero
  if (fabsf(command) < deadband.deadband_deg) {
    return 0.0f;
  }
  return command;
}

float ServoSmoother::apply_lowpass(float command,
                                   uint32_t time_delta_ms,
                                   uint8_t axis_id) {
  if (time_delta_ms == 0) return command;

  // Calculate alpha (smoothing factor) for this time step
  float alpha = calculate_alpha(time_delta_ms);
  alpha = fminf(alpha, 1.0f);  // Clamp to [0, 1]

  // First-order IIR: y = alpha * x + (1 - alpha) * y_prev
  float filtered = alpha * command + (1.0f - alpha) * lowpass.prev_output[axis_id];

  // Store for next iteration
  lowpass.prev_output[axis_id] = filtered;

  return filtered;
}

float ServoSmoother::clamp_angle(float angle, float min_ang, float max_ang) const {
  if (angle < min_ang) return min_ang;
  if (angle > max_ang) return max_ang;
  return angle;
}

float ServoSmoother::calculate_alpha(uint32_t time_delta_ms) {
  // Alpha = (2π × cutoff_hz × dt) / 1000
  // dt is in seconds, so divide time_delta_ms by 1000
  if (lowpass.cutoff_hz <= 0.0f || time_delta_ms == 0) return 0.0f;

  float dt = time_delta_ms / 1000.0f;
  return TWO_PI_CONST * lowpass.cutoff_hz * dt;
}
