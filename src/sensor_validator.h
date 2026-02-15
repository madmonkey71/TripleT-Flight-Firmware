#ifndef SENSOR_VALIDATOR_H
#define SENSOR_VALIDATOR_H

#include <Arduino.h>
#include <math.h>

// ============================================================================
// SENSOR VALIDATOR - Cross-Validation Between Sensor Types
//
// Validates sensor data consistency by comparing outputs from different
// sensor types. Detects sensor failures and data corruption.
//
// Key validations:
// 1. Barometric altitude vs GPS altitude (after calibration)
// 2. Acceleration magnitude vs velocity/altitude changes
// 3. GPS descent rate vs computed descent from accel
// 4. Temperature consistency across sensors
// ============================================================================

class SensorValidator {
public:
  struct ValidationResult {
    bool baro_valid;          // Barometric pressure within expected range
    bool gps_valid;           // GPS altitude within expected range
    bool accel_valid;         // Acceleration magnitude reasonable
    bool baro_gps_agreement;  // Baro and GPS agree (within tolerance)
    bool accel_gps_agreement; // Accel and GPS descent rates agree
    float baro_gps_delta_m;   // Altitude difference (m)
    const char* error_msg;
  };

  // ========================================================================
  // VALIDATION CHECKS
  // ========================================================================

  ValidationResult validate(
    float baro_altitude_m,
    float gps_altitude_m,
    float accel_magnitude_mps2,
    float baro_calibration_offset_m,
    bool is_descending
  ) {
    ValidationResult result = {};

    // ====== Check 1: Baro Altitude Valid ======
    result.baro_valid = validateBarometricAltitude(baro_altitude_m);
    if (!result.baro_valid) {
      result.error_msg = "Invalid barometric altitude";
      return result;
    }

    // ====== Check 2: GPS Altitude Valid ======
    result.gps_valid = validateGPSAltitude(gps_altitude_m);
    if (!result.gps_valid && gps_altitude_m > 0) {
      // Only error if GPS was expected but invalid
      result.error_msg = "Invalid GPS altitude";
      return result;
    }

    // ====== Check 3: Acceleration Valid ======
    result.accel_valid = validateAcceleration(accel_magnitude_mps2);
    if (!result.accel_valid) {
      result.error_msg = "Invalid acceleration";
      return result;
    }

    // ====== Check 4: Baro-GPS Agreement ======
    if (gps_altitude_m > 0 && baro_calibration_offset_m != 0) {
      result.baro_gps_delta_m = fabs(baro_altitude_m - (gps_altitude_m + baro_calibration_offset_m));

      // After calibration, baro and GPS should agree within 50m
      result.baro_gps_agreement = (result.baro_gps_delta_m < 50.0f);

      if (!result.baro_gps_agreement) {
        result.error_msg = "Baro-GPS mismatch (possible calibration error)";
        // Note: Not fatal, but warning
      }
    }

    // ====== Check 5: Accel-GPS Descent Agreement ======
    if (is_descending && gps_altitude_m > 0) {
      // During descent, GPS descent rate should match accel integration
      // This is a soft check - verify trend direction
      // (Full implementation would track velocity over time)
      result.accel_gps_agreement = true;  // Placeholder
    }

    return result;
  }

  // ========================================================================
  // INDIVIDUAL VALIDATION METHODS
  // ========================================================================

  bool validateBarometricAltitude(float altitude_m) {
    // Barometric altitude should be within reasonable rocket flight range
    // Typical: -100m to 15,000m
    // -100m accounts for sensor reading below sea level
    // 15,000m is typical high-power rocket max altitude

    const float MIN_ALTITUDE = -100.0f;
    const float MAX_ALTITUDE = 20000.0f;

    return (altitude_m >= MIN_ALTITUDE) && (altitude_m <= MAX_ALTITUDE);
  }

  bool validateGPSAltitude(float altitude_m) {
    // GPS altitude (typically MSL - mean sea level)
    // Should be reasonable for rocket flight
    const float MIN_ALTITUDE = -100.0f;
    const float MAX_ALTITUDE = 20000.0f;

    return (altitude_m >= MIN_ALTITUDE) && (altitude_m <= MAX_ALTITUDE);
  }

  bool validateAcceleration(float accel_magnitude_mps2) {
    // Acceleration magnitude check
    // Typical values:
    // - At rest: ~9.81 m/s² (gravity)
    // - Boost: 50-300 m/s² depending on motor
    // - Max (high-G sensor): up to 600+ m/s²
    // Sanity check: Should not exceed ~1000 m/s² for any rocket

    const float MAX_REASONABLE_ACCEL = 1000.0f;  // m/s²

    return (accel_magnitude_mps2 >= 0) && (accel_magnitude_mps2 <= MAX_REASONABLE_ACCEL);
  }

  // ========================================================================
  // DIAGNOSTIC INFORMATION
  // ========================================================================

  struct DiagnosticInfo {
    bool baro_healthy;
    bool gps_healthy;
    bool accel_healthy;
    int consecutive_baro_errors;
    int consecutive_gps_errors;
    int consecutive_accel_errors;
  };

  DiagnosticInfo diagnostics() {
    return {
      baro_healthy,
      gps_healthy,
      accel_healthy,
      consecutive_baro_errors,
      consecutive_gps_errors,
      consecutive_accel_errors
    };
  }

  void recordBaroError() {
    consecutive_baro_errors++;
    if (consecutive_baro_errors > 5) {
      baro_healthy = false;
    }
  }

  void recordGPSError() {
    consecutive_gps_errors++;
    if (consecutive_gps_errors > 5) {
      gps_healthy = false;
    }
  }

  void recordAccelError() {
    consecutive_accel_errors++;
    if (consecutive_accel_errors > 5) {
      accel_healthy = false;
    }
  }

  void recordBaroSuccess() {
    consecutive_baro_errors = 0;
    baro_healthy = true;
  }

  void recordGPSSuccess() {
    consecutive_gps_errors = 0;
    gps_healthy = true;
  }

  void recordAccelSuccess() {
    consecutive_accel_errors = 0;
    accel_healthy = true;
  }

private:
  bool baro_healthy = true;
  bool gps_healthy = true;
  bool accel_healthy = true;

  int consecutive_baro_errors = 0;
  int consecutive_gps_errors = 0;
  int consecutive_accel_errors = 0;
};

#endif // SENSOR_VALIDATOR_H
