#ifndef APOGEE_DETECTOR_H
#define APOGEE_DETECTOR_H

#include <Arduino.h>
#include "sensors/imu_interface.h"

// ============================================================================
// APOGEE DETECTOR - Multi-Path Detection with 2-of-3 Voting
//
// Detects peak altitude (apogee) using three independent methods:
// 1. Barometric Pressure (Primary) - Most reliable for altitude
// 2. Acceleration (Secondary) - Fast, direct measurement
// 3. GPS Altitude (Tertiary) - When available, validates baro
// 4. Timeout Timer (Failsafe) - Ensures deployment even if sensors fail
//
// Uses 2-of-3 voting to detect apogee:
// - If 2+ methods agree → Apogee detected
// - If only 1 method detects → Waits for confirmation
// - If no methods agree → Uses timeout failsafe
//
// Safety: Multiple independent methods ensure reliability
// ============================================================================

class ApogeeDetector {
public:
  ApogeeDetector(IMUInterface* imu) : imu_sensor(imu) {
    reset();
  }

  // ========================================================================
  // INITIALIZATION & RESET
  // ========================================================================

  void reset() {
    baro_apogee_detected = false;
    accel_apogee_detected = false;
    gps_apogee_detected = false;
    timeout_apogee_triggered = false;

    baro_velocity_mps = 0.0f;
    accel_velocity_mps = 0.0f;
    gps_velocity_mps = 0.0f;

    last_altitude_baro = 0.0f;
    last_altitude_gps = 0.0f;
    last_time_ms = millis();

    baro_zero_crossings = 0;
    accel_zero_crossings = 0;
    gps_zero_crossings = 0;

    apogee_vote_count = 0;
    boost_start_time_ms = 0;
  }

  // ========================================================================
  // PROCESSING
  // ========================================================================

  void update(float current_altitude_baro, float current_altitude_gps,
              uint32_t boost_time_ms) {
    uint32_t now = millis();
    uint32_t dt_ms = now - last_time_ms;
    if (dt_ms == 0) dt_ms = 1;  // Prevent division by zero
    float dt_s = dt_ms / 1000.0f;

    last_time_ms = now;

    // Update boost timer for timeout failsafe
    if (boost_time_ms == 0) {
      boost_start_time_ms = now;
    } else {
      boost_start_time_ms = now - boost_time_ms;
    }

    // ====== Method 1: Barometric Apogee Detection (Primary) ======
    updateBarometricApogee(current_altitude_baro, dt_s);

    // ====== Method 2: Acceleration-based Apogee Detection (Secondary) ======
    if (imu_sensor) {
      float accel_magnitude = imu_sensor->getAccelMagnitude();
      updateAccelApogee(accel_magnitude, dt_s);
    }

    // ====== Method 3: GPS Altitude Apogee Detection (Tertiary) ======
    updateGPSApogee(current_altitude_gps, dt_s);

    // ====== Timeout Failsafe ======
    updateTimeoutApogee(boost_time_ms);

    // ====== Vote Counting ======
    updateVoting();
  }

  // ========================================================================
  // APOGEE DETECTION RESULT
  // ========================================================================

  bool isApogeeDetected() {
    // Apogee is detected if 2+ methods agree
    return apogee_vote_count >= 2;
  }

  const char* getApogeeReason() {
    if (apogee_vote_count >= 2) {
      if (baro_apogee_detected && accel_apogee_detected) {
        return "Baro + Accel";
      }
      if (baro_apogee_detected && gps_apogee_detected) {
        return "Baro + GPS";
      }
      if (accel_apogee_detected && gps_apogee_detected) {
        return "Accel + GPS";
      }
      if (timeout_apogee_triggered) {
        return "Timeout Failsafe";
      }
    }
    return "No apogee";
  }

  // ========================================================================
  // HEALTH & DEBUG INFORMATION
  // ========================================================================

  struct ApogeeStatus {
    bool baro_apogee;
    bool accel_apogee;
    bool gps_apogee;
    bool timeout_apogee;
    int vote_count;
    float baro_velocity;
    float accel_velocity;
    float gps_velocity;
  };

  ApogeeStatus getStatus() {
    return {
      baro_apogee_detected,
      accel_apogee_detected,
      gps_apogee_detected,
      timeout_apogee_triggered,
      apogee_vote_count,
      baro_velocity_mps,
      accel_velocity_mps,
      gps_velocity_mps
    };
  }

private:
  IMUInterface* imu_sensor;

  // Baro method state
  bool baro_apogee_detected = false;
  float baro_velocity_mps = 0.0f;
  float last_altitude_baro = 0.0f;
  int baro_zero_crossings = 0;

  // Acceleration method state
  bool accel_apogee_detected = false;
  float accel_velocity_mps = 0.0f;
  float last_accel_magnitude = 0.0f;
  int accel_zero_crossings = 0;

  // GPS method state
  bool gps_apogee_detected = false;
  float gps_velocity_mps = 0.0f;
  float last_altitude_gps = 0.0f;
  int gps_zero_crossings = 0;

  // Timeout method state
  bool timeout_apogee_triggered = false;
  uint32_t boost_start_time_ms = 0;
  static constexpr uint32_t APOGEE_TIMEOUT_MS = 60000;  // 60 seconds max

  // Voting
  int apogee_vote_count = 0;
  uint32_t last_time_ms = 0;

  // ========================================================================
  // METHOD 1: BAROMETRIC APOGEE DETECTION
  // ========================================================================

  void updateBarometricApogee(float altitude_baro, float dt_s) {
    // Integrate altitude to get velocity
    float altitude_change = altitude_baro - last_altitude_baro;
    baro_velocity_mps = altitude_change / dt_s;
    last_altitude_baro = altitude_baro;

    // Detect zero crossing: positive velocity → negative velocity
    if (baro_velocity_mps < -0.5f) {  // -0.5 m/s hysteresis
      baro_zero_crossings++;

      // Require multiple crossings to confirm apogee (prevent noise)
      if (baro_zero_crossings >= 3) {
        baro_apogee_detected = true;
      }
    } else if (baro_velocity_mps > 0.5f) {
      // Reset if velocity goes positive again
      baro_zero_crossings = 0;
      baro_apogee_detected = false;
    }
  }

  // ========================================================================
  // METHOD 2: ACCELERATION-BASED APOGEE DETECTION
  // ========================================================================

  void updateAccelApogee(float accel_magnitude, float dt_s) {
    if (!imu_sensor || !imu_sensor->isHealthy()) {
      return;
    }

    // Subtract gravity (9.81 m/s²) - at apogee, accel ≈ 0 in vertical
    float vertical_accel = accel_magnitude - 9.81f;

    // Integrate to get velocity
    accel_velocity_mps += vertical_accel * dt_s;

    // Detect zero crossing: positive accel → negative accel
    if (accel_velocity_mps < -0.5f) {  // -0.5 m/s hysteresis
      accel_zero_crossings++;

      if (accel_zero_crossings >= 2) {
        accel_apogee_detected = true;
      }
    } else if (accel_velocity_mps > 0.5f) {
      accel_zero_crossings = 0;
      accel_apogee_detected = false;
    }
  }

  // ========================================================================
  // METHOD 3: GPS ALTITUDE APOGEE DETECTION
  // ========================================================================

  void updateGPSApogee(float altitude_gps, float dt_s) {
    // GPS is only used for validation, not primary detection
    // Only detect apogee if GPS altitude is valid and decreasing
    if (altitude_gps <= 0) {
      return;  // Invalid GPS altitude
    }

    float altitude_change = altitude_gps - last_altitude_gps;
    gps_velocity_mps = altitude_change / dt_s;
    last_altitude_gps = altitude_gps;

    // GPS has lower precision, require stronger threshold
    if (gps_velocity_mps < -2.0f) {  // -2.0 m/s threshold (more conservative)
      gps_zero_crossings++;

      if (gps_zero_crossings >= 2) {
        gps_apogee_detected = true;
      }
    } else if (gps_velocity_mps > 2.0f) {
      gps_zero_crossings = 0;
      gps_apogee_detected = false;
    }
  }

  // ========================================================================
  // TIMEOUT FAILSAFE
  // ========================================================================

  void updateTimeoutApogee(uint32_t boost_time_ms) {
    // If we don't detect apogee within reasonable time, force it
    // Typical max burn time: 15 seconds + coast time: 30 seconds
    if (boost_time_ms > APOGEE_TIMEOUT_MS) {
      timeout_apogee_triggered = true;
    }
  }

  // ========================================================================
  // VOTING MECHANISM
  // ========================================================================

  void updateVoting() {
    // Count votes for apogee detection
    apogee_vote_count = 0;

    if (baro_apogee_detected) apogee_vote_count++;
    if (accel_apogee_detected) apogee_vote_count++;
    if (gps_apogee_detected) apogee_vote_count++;
    if (timeout_apogee_triggered) apogee_vote_count++;

    // Cap at 3 votes (we only care if >= 2)
    if (apogee_vote_count > 3) apogee_vote_count = 3;
  }
};

#endif // APOGEE_DETECTOR_H
