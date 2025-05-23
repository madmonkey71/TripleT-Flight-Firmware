#include "flight_logic.h"
#include "config.h"        // For flight parameters and configuration
#include "constants.h"     // For timing constants like BACKUP_APOGEE_TIME
#include "utility_functions.h" // For get_accel_magnitude()
#include "ms5611_functions.h"  // For ms5611_get_altitude()

// --- External Globals Needed ---
// Flight state and parameters are now declared extern in flight_logic.h

// Sensor readiness/data (Check how these are defined in the original file)
extern bool kx134_initialized_ok; // From main file?
#define kx134_accel_ready (kx134_initialized_ok) // Replicate macro definition if needed
extern bool icm20948_ready; // From main file or ICM header?
// ms5611_ready is defined in ms5611_functions.h as ms5611Sensor.isConnected()

// Sensor raw data (Check headers or declare extern)
extern float kx134_accel[3];
extern float icm_accel[3];

// Forward declaration for saveStateToEEPROM if called from detectLanding
void saveStateToEEPROM();


// --- Function Definitions ---

// Redundant apogee detection
bool detectApogee() {
  float currentAltitude = ms5611_get_altitude();
  bool apogeeDetected = false;

  // Method 1: Barometric detection (primary)
  if (ms5611Sensor.isConnected()) {
    // Track maximum altitude
    if (currentAltitude > maxAltitudeReached) {
      maxAltitudeReached = currentAltitude;
      descendingCount = 0;
    }
    else if (currentAltitude < maxAltitudeReached - 1.0) {
      // Altitude is decreasing significantly
      descendingCount++;

      // Require multiple consecutive readings to confirm
      if (descendingCount >= APOGEE_CONFIRMATION_COUNT) {
        Serial.println(F("APOGEE DETECTED (barometric)"));
        apogeeDetected = true;
      }
    }
  }

  // Method 2: Accelerometer-based detection (backup)
  // Check if vertical acceleration is close to zero (indicating peak)
  if (!apogeeDetected && (kx134_accel_ready || icm20948_ready)) {
    float accel_z = kx134_accel_ready ? kx134_accel[2] : icm_accel[2];

    // Counter for near-zero acceleration readings
    static int accelNearZeroCount = 0;
    const float accel_apogee_threshold = 0.1; // g threshold around zero

    // Check if absolute value of z-acceleration is below threshold
    if (fabs(accel_z) < accel_apogee_threshold) {
      accelNearZeroCount++;
      // Require multiple consecutive readings near zero G
      if (accelNearZeroCount >= 5) { // Use a count similar to the old check for now
        Serial.println(F("APOGEE DETECTED (accelerometer - near zero G)"));
        apogeeDetected = true;
        accelNearZeroCount = 0; // Reset after detection
      }
    } else {
      // Reset count if acceleration moves away from zero
      accelNearZeroCount = 0;
    }
  }

  // Method 3: Time-based detection (last resort)
  if (!apogeeDetected && boostEndTime > 0) {
    // If we know when the boost phase ended, we can estimate apogee
    if (millis() - boostEndTime > BACKUP_APOGEE_TIME) {
      Serial.println(F("APOGEE DETECTED (time-based)"));
      apogeeDetected = true;
    }
  }

  return apogeeDetected;
}

// Redundant landing detection
bool detectLanding() {
  static int stableCount = 0;
  bool landingDetected = false;

  // Method 1: Accelerometer stability (primary)
  if (kx134_accel_ready || icm20948_ready) {
    float accel_magnitude = get_accel_magnitude();

    // Check if acceleration is close to 1g (just gravity)
    if (accel_magnitude > 0.95 && accel_magnitude < 1.05) {
      stableCount++;
    } else {
      stableCount = 0;
    }

    // Require extended stability to confirm landing
    if (stableCount >= LANDING_CONFIRMATION_COUNT) {
      landingDetected = true;
    }
  }

  // Method 2: Barometric stability (backup)
  if (!landingDetected && ms5611Sensor.isConnected()) {
    static float lastAltitude = 0.0;
    static int altitudeStableCount = 0;

    float currentAltitude = ms5611_get_altitude();

    // Check if altitude is stable
    if (fabs(currentAltitude - lastAltitude) < 1.0) {
      altitudeStableCount++;
    } else {
      altitudeStableCount = 0;
    }

    lastAltitude = currentAltitude;

    // Require extended stability to confirm landing
    if (altitudeStableCount >= LANDING_CONFIRMATION_COUNT) {
      landingDetected = true;
    }
  }

  // If landing detected, log it and save state
  if (landingDetected && !landingDetectedFlag) {
    Serial.println(F("LANDING DETECTED"));
    landingDetectedFlag = true;
    // Save state immediately on landing detection
    saveStateToEEPROM(); // Assumes saveStateToEEPROM is accessible (declared above)
  }

  return landingDetected;
}

// Track boost end for time-based apogee detection
void detectBoostEnd() {
  float accel_magnitude = get_accel_magnitude();

  // When acceleration drops below threshold, record the time
  if (accel_magnitude < COAST_ACCEL_THRESHOLD && boostEndTime == 0) {
    boostEndTime = millis();
    Serial.println(F("BOOST END DETECTED"));
    Serial.print(F("Time since startup: "));
    Serial.print(boostEndTime / 1000.0);
    Serial.println(F(" seconds"));
  }
}

// Check if rocket is stable (no significant motion)
bool IsStable() {
  float accel_magnitude = get_accel_magnitude();
  // Consider stable if acceleration is close to 1g (gravity only)
  return (accel_magnitude > 0.95 && accel_magnitude < 1.05);
} 