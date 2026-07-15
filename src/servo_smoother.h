#ifndef SERVO_SMOOTHER_H
#define SERVO_SMOOTHER_H

#include <Arduino.h>
#include <stdint.h>

/**
 * @brief Servo response smoothing and filtering
 *
 * Provides rate limiting, deadband filtering, and low-pass filtering
 * to reduce servo jitter and prevent control instability.
 *
 * Three filtering modes:
 * 1. Rate Limiting: Max 10°/100ms per axis (prevents jerky movements)
 * 2. Deadband: Ignore commands < 0.5° to reduce noise
 * 3. Low-Pass Filter: 2Hz cutoff for signal smoothing
 */
class ServoSmoother {
public:
  /**
   * @brief Filtering mode enumeration
   */
  enum FilterType {
    RATE_LIMIT_ONLY,         // 0: Only rate limiting
    DEADBAND_ONLY,           // 1: Only deadband
    LOWPASS_ONLY,            // 2: Only low-pass filter
    RATE_LIMIT_THEN_LOWPASS, // 3: Rate limit, then filter
    FULL_FILTERING           // 4: All three applied in sequence
  };

  // Constructor
  ServoSmoother();

  /**
   * @brief Initialize filter parameters from config.h
   *
   * @param filter_type Which filters to apply (default: FULL_FILTERING)
   */
  void init(FilterType filter_type = FULL_FILTERING);

  /**
   * @brief Apply smoothing to single servo command
   *
   * @param desired_angle Desired servo angle (degrees, typically 0-180)
   * @param current_angle Current servo angle from last iteration (degrees)
   * @param time_delta_ms Time since last call (milliseconds)
   * @param axis_id 0=pitch, 1=roll, 2=yaw (for individual parameter tuning if needed)
   * @return Smoothed command angle (degrees)
   */
  float smooth(float desired_angle,
               float current_angle,
               uint32_t time_delta_ms,
               uint8_t axis_id = 0);

  /**
   * @brief Batch smooth all three axes at once
   *
   * More efficient than calling smooth() three times
   *
   * @param desired Array of [pitch, roll, yaw] desired angles
   * @param current Array of [pitch, roll, yaw] current angles
   * @param time_delta_ms Time delta since last call
   * @param[out] output Array of [pitch, roll, yaw] smoothed outputs
   */
  void smoothBatch(const float desired[3],
                   const float current[3],
                   uint32_t time_delta_ms,
                   float output[3]);

  /**
   * @brief Reset all filters
   * Call when entering new flight state
   */
  void reset();

  /**
   * @brief Get last output for given axis
   *
   * @param axis_id 0=pitch, 1=roll, 2=yaw
   * @return Last smoothed output angle
   */
  float getLastOutput(uint8_t axis_id) const;

  /**
   * @brief Print filter parameters to serial
   */
  void printParameters() const;

private:
  FilterType active_filter;

  // Current outputs (used in rate limiting and feedback)
  float last_output[3];

  // Low-pass filter state (first-order IIR)
  struct {
    float prev_output[3];  // Previous filter output
    float cutoff_hz;       // Filter cutoff frequency (Hz)
    float alpha;           // Smoothing factor (0.0 to 1.0)
  } lowpass;

  // Rate limiting parameters
  struct {
    float max_rate_per_ms[3];  // Max degrees per millisecond per axis
                               // Pitch: 10.0 deg/100ms = 0.1 deg/ms
                               // Roll:  10.0 deg/100ms = 0.1 deg/ms
                               // Yaw:   15.0 deg/100ms = 0.15 deg/ms
  } rate_limit;

  // Deadband parameters
  struct {
    float deadband_deg;        // Default: 0.5 degrees
  } deadband;

  // Helper functions
  /**
   * @brief Apply rate limiting to command
   */
  float apply_rate_limit(float desired,
                         float current,
                         float time_delta_ms,
                         uint8_t axis_id);

  /**
   * @brief Apply deadband filtering
   */
  float apply_deadband(float command);

  /**
   * @brief Apply low-pass filter (first-order IIR)
   */
  float apply_lowpass(float command,
                      uint32_t time_delta_ms,
                      uint8_t axis_id);

  /**
   * @brief Clamp angle to specified range
   */
  float clamp_angle(float angle,
                    float min_ang,
                    float max_ang) const;

  /**
   * @brief Calculate low-pass filter alpha
   * Alpha = (2π × cutoff_hz × dt) / 1000
   */
  float calculate_alpha(uint32_t time_delta_ms);
};

#endif // SERVO_SMOOTHER_H
