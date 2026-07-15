#ifndef STABILITY_MONITOR_H
#define STABILITY_MONITOR_H

#include <Arduino.h>
#include <stdint.h>
#include <cmath>

/**
 * @brief Real-time stability monitoring for guidance control system
 *
 * Monitors angular rates, attitude errors, and actuator saturation
 * against configurable thresholds. Used to detect guidance instability
 * and trigger failsafe mechanisms.
 */
class StabilityMonitor {
public:
  /**
   * @brief Stability metrics returned by update()
   */
  struct StabilityMetrics {
    // Angular Rates (degrees per second)
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;

    // Attitude Errors vs Desired (degrees)
    float roll_error_deg;
    float pitch_error_deg;
    float yaw_error_deg;

    // Actuator State (0-100%)
    float actuator_saturation_percent;  // Max of all axes
    float pitch_saturation_percent;
    float roll_saturation_percent;
    float yaw_saturation_percent;

    // Overall Status
    bool is_stable;
    const char* violation_type;  // NULL if stable, else "RATE", "ATTITUDE", "SATURATION"
    uint8_t violation_bitmask;   // Bit 0=rate, 1=attitude, 2=saturation
  };

  // Constructor
  StabilityMonitor();

  /**
   * @brief Initialize with threshold values from config.h
   */
  void init();

  /**
   * @brief Update stability assessment
   *
   * @param current_quat Quaternion from Kalman filter [q0, q1, q2, q3] (normalized)
   * @param desired_quat Desired orientation quaternion [q0, q1, q2, q3] (normalized)
   * @param gyro_rate_radps [roll_rate, pitch_rate, yaw_rate] in rad/s from gyroscope
   * @param cmd_pitch Normalized actuator command [-1.0, 1.0]
   * @param cmd_roll  Normalized actuator command [-1.0, 1.0]
   * @param cmd_yaw   Normalized actuator command [-1.0, 1.0]
   * @param current_time_ms System time in milliseconds
   */
  void update(const float current_quat[4],
              const float desired_quat[4],
              const float gyro_rate_radps[3],
              float cmd_pitch,
              float cmd_roll,
              float cmd_yaw,
              uint32_t current_time_ms);

  /**
   * @brief Get current metrics
   * @return Latest StabilityMetrics
   */
  StabilityMetrics getMetrics() const;

  /**
   * @brief Check if currently in violation
   * @return true if any threshold exceeded, false otherwise
   */
  bool isStabilityViolation() const;

  /**
   * @brief Check if violation duration exceeded persistence threshold
   * Prevents oscillation by requiring violations to persist for 500ms+ (default)
   * @return true if violation has persisted long enough to trigger action
   */
  bool isViolationPersistent() const;

  /**
   * @brief Get time in current violation (milliseconds)
   * @return Duration of current violation, or 0 if not violating
   */
  uint32_t getViolationDuration() const;

  /**
   * @brief Reset violation timer and status
   * Call when entering new flight state
   */
  void resetViolation();

  /**
   * @brief Print diagnostic information to serial
   * Shows current metrics and threshold comparisons
   */
  void printDiagnostics() const;

private:
  // Internal state
  StabilityMetrics current_metrics;
  uint32_t violation_start_time_ms;
  bool in_violation;

  // Thresholds (loaded from config.h during init())
  struct {
    float roll_rate_limit_dps;         // Default: 180 DPS
    float pitch_rate_limit_dps;        // Default: 180 DPS
    float yaw_rate_limit_dps;          // Default: 360 DPS

    float roll_error_limit_deg;        // Default: 30 degrees
    float pitch_error_limit_deg;       // Default: 20 degrees
    float yaw_error_limit_deg;         // Default: 20 degrees

    float saturation_limit_percent;    // Default: 95%
    uint32_t violation_duration_ms;    // Default: 500 ms

  } thresholds;

  // Private helper methods
  /**
   * @brief Convert quaternion to Euler angles
   *
   * @param q Input quaternion [q0, q1, q2, q3] (w, x, y, z)
   * @param[out] roll Roll angle in radians
   * @param[out] pitch Pitch angle in radians
   * @param[out] yaw Yaw angle in radians
   */
  void quaternion_to_euler(const float q[4], float& roll, float& pitch, float& yaw) const;

  /**
   * @brief Convert radians to degrees
   */
  float rad_to_deg(float rad) const;

  /**
   * @brief Normalize angle to [-180, 180] degrees
   */
  float normalize_angle_deg(float deg) const;

  /**
   * @brief Check angular rate thresholds
   * @return true if any rate exceeds limit
   */
  bool check_angular_rates(float roll_dps, float pitch_dps, float yaw_dps);

  /**
   * @brief Check attitude error thresholds
   * @return true if any error exceeds limit
   */
  bool check_attitude_error(float roll_err_deg, float pitch_err_deg, float yaw_err_deg);

  /**
   * @brief Check actuator saturation
   * @return true if saturation exceeds limit
   */
  bool check_actuator_saturation(float cmd_p, float cmd_r, float cmd_y);

  /**
   * @brief Update violation state based on current thresholds
   * Manages violation_start_time_ms and in_violation flag
   */
  void update_violation_state(uint32_t current_time_ms);
};

#endif // STABILITY_MONITOR_H
