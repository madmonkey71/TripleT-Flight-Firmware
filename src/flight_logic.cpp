#include "flight_logic.h"
#include "config.h"        // For flight parameters and configuration
#include "constants.h"     // For timing constants like BACKUP_APOGEE_TIME
#include "utility_functions.h" // For get_accel_magnitude()
#include "ms5611_functions.h"  // For ms5611_get_altitude()

// Define global flight state variables
unsigned long boostEndTime = 0;
bool landingDetectedFlag = false;

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
  static float maxAltitudeReached = 0.0f;
  static int descendingCount = 0;

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
    if (millis() - boostEndTime > EXPECTED_APOGEE_TIME) {
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

// Ensure includes for the new function are present.
// These might be redundant if already at the top, but ensures they are available.
#include "guidance_control.h" // For guidance_set_target_orientation_euler()
#include "icm_20948_functions.h" // For convertQuaternionToEuler and icm_q0 etc.
#include <Arduino.h>          // For millis() and PI

// Global quaternions are extern in icm_20948_functions.h, so no need for separate extern here.
// For debug prints in update_guidance_targets
extern bool enableSystemDebug; // Ensure this is declared if used by the function for prints

static bool initial_target_set = false;
static float initial_yaw_target = 0.0f; // Store the initial yaw target in radians

void update_guidance_targets() {
    unsigned long currentTime_ms = millis();
    float target_roll_rad = 0.0f;
    float target_pitch_rad = 0.0f;
    float target_yaw_rad = 0.0f;

    if (!initial_target_set) {
        // On the first run, set the initial target yaw to the current yaw.
        // This prevents sudden movement if the system starts at a non-zero yaw.
        float current_roll, current_pitch, current_yaw;
        convertQuaternionToEuler(icm_q0, icm_q1, icm_q2, icm_q3, current_roll, current_pitch, current_yaw);
        initial_yaw_target = current_yaw; // current_yaw is in radians
        
        target_roll_rad = 0.0f;
        target_pitch_rad = 0.0f;
        target_yaw_rad = initial_yaw_target;
        guidance_set_target_orientation_euler(target_roll_rad, target_pitch_rad, target_yaw_rad);
        initial_target_set = true;
        
        if (enableSystemDebug) { 
            Serial.print(F("Initial guidance target set. Yaw (rad): "));
            Serial.println(initial_yaw_target, 4);
        }

    } else {
        // Example: After 30 seconds, change target yaw to initial_yaw + 45 degrees.
        // And after 60 seconds, return to initial_yaw.
        // Maintain current roll and pitch targets (0 degrees).
        if (currentTime_ms > 30000 && currentTime_ms < 60000) { // Active between 30-60s
            target_roll_rad = 0.0f;
            target_pitch_rad = 0.0f;
            target_yaw_rad = initial_yaw_target + (PI / 4.0f); 
        } else if (currentTime_ms >= 60000) { // After 60s, return to initial yaw
            target_roll_rad = 0.0f;
            target_pitch_rad = 0.0f;
            target_yaw_rad = initial_yaw_target;
        }
        else {
            // Before 30 seconds, maintain the initial yaw target
            target_roll_rad = 0.0f;
            target_pitch_rad = 0.0f;
            target_yaw_rad = initial_yaw_target;
        }
        // Set the potentially updated targets
        guidance_set_target_orientation_euler(target_roll_rad, target_pitch_rad, target_yaw_rad);

        // Optional: Print target update if system debug is enabled and target has changed
        // This logic aims to print only when the target phase changes or periodically during an active phase.
        if (enableSystemDebug) {
            static unsigned long lastTargetPrintTimeMs = 0;
            static float lastReportedTargetYaw = -1000.0f; // Initialize to a value that won't match

            bool should_print_now = false;

            if (target_yaw_rad != lastReportedTargetYaw) { // If target actually changed
                should_print_now = true;
            } else if ( (currentTime_ms > 30000) && (currentTime_ms - lastTargetPrintTimeMs > 5000) ) {
                // Or if in an active phase (after 30s), print every 5s regardless of change
                should_print_now = true;
            }

            if (should_print_now && (currentTime_ms - lastTargetPrintTimeMs > 1000 || lastTargetPrintTimeMs > currentTime_ms) ) { // Basic throttle: at most once per second
                lastTargetPrintTimeMs = currentTime_ms;
                lastReportedTargetYaw = target_yaw_rad;
                Serial.print(F("Guidance target update (Time: "));
                Serial.print(currentTime_ms/1000);
                Serial.print(F("s). New Target Yaw (rad): "));
                Serial.println(target_yaw_rad, 4);
            }
        }
    }
}