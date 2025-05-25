#include "flight_logic.h"
#include "config.h"
#include "utility_functions.h"
#include "ms5611_functions.h"
#include "icm_20948_functions.h" // For icm_accel, kx134_accel might be here or in kx134_functions.h
#include "kx134_functions.h"   // For kx134_accel
// #include "Adafruit_NeoPixel.h" // pixels object is externed in utility_functions.h, so direct include not needed here

// --- Global and Static Variable Definitions ---
FlightState currentFlightState = STARTUP;
float launchAltitude = 0.0f;
float maxAltitudeReached = 0.0f;

static unsigned long flightStartTime = 0; // Not used yet, but defined as per plan
static unsigned long lastStateChangeTime = 0;
static int descendingCount_apogee = 0; // Renamed to avoid conflict with local variables in original detectApogee
static float currentFlightAltitude = 0.0f; // Local cache of altitude

// Existing global variables (review if they are still needed or managed by state machine)
unsigned long boostEndTime = 0;
bool landingDetectedFlag = false;


// --- External Globals That Might Be Needed ---
// Sensor readiness/data (Ensure these are correctly externed or available through functions)
// extern bool kx134_initialized_ok; // This should be available from kx134_functions.cpp/h if needed
// extern bool icm20948_ready; // This should be available from icm_20948_functions.cpp/h if needed

// Sensor raw data (Preferably accessed via functions if possible)
// extern float kx134_accel[3]; // Accessed via get_current_accel_magnitude -> get_accel_magnitude
// extern float icm_accel[3];   // Accessed via get_current_accel_magnitude -> get_accel_magnitude

// Forward declaration for saveStateToEEPROM if called from detectLanding
// This should be declared in a header if it's a global utility, or passed as a function pointer.
// For now, assuming it's available or will be handled.
// void saveStateToEEPROM(); // Commenting out, as it's not defined in this context yet.

// --- New State Machine Core Functions ---

void initialize_flight_state_machine() {
  currentFlightState = STARTUP; // Or PAD_IDLE after initial checks in main setup()
  lastStateChangeTime = millis();
  launchAltitude = 0.0f;
  maxAltitudeReached = 0.0f;
  descendingCount_apogee = 0;
  currentFlightAltitude = 0.0f; // Initialize local cache
  Serial.println("Flight State Machine Initialized. State: STARTUP");
}

const char* get_flight_state_name(FlightState state) {
  switch (state) {
    case STARTUP: return "STARTUP";
    case CALIBRATION: return "CALIBRATION";
    case PAD_IDLE: return "PAD_IDLE";
    case ARMED: return "ARMED";
    case BOOST: return "BOOST";
    case COAST: return "COAST";
    case APOGEE: return "APOGEE";
    case DROGUE_DEPLOY: return "DROGUE_DEPLOY";
    case DROGUE_DESCENT: return "DROGUE_DESCENT";
    case MAIN_DEPLOY: return "MAIN_DEPLOY";
    case MAIN_DESCENT: return "MAIN_DESCENT";
    case LANDED: return "LANDED";
    case RECOVERY: return "RECOVERY";
    case ERROR_STATE: return "ERROR_STATE";
    default: return "UNKNOWN_STATE";
  }
}

float get_current_baro_altitude() {
  // This function adapts the existing ms5611_get_altitude()
  // and updates currentFlightAltitude
  // Assuming ms5611_get_altitude() returns calibrated altitude if baroCalibrated is true
  currentFlightAltitude = ms5611_get_altitude();
  return currentFlightAltitude;
}

float get_current_accel_magnitude() {
  // This function adapts the existing get_accel_magnitude() from utility_functions.h
  // get_accel_magnitude() itself handles which accelerometer to use (KX134 or ICM)
  return get_accel_magnitude();
}

void process_flight_state() {
  // TODO: Implement full state machine logic here in the next step.
  // For now, just print the current state periodically for debugging.
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 2000) {
    Serial.print("Current Flight State: ");
    Serial.println(get_flight_state_name(currentFlightState));
    lastPrintTime = millis();
  }
}

// --- Adapted Helper Functions ---

// Redundant apogee detection - ADAPTED
bool detectApogee() {
  float currentAltitude = get_current_baro_altitude(); // ADAPTED
  bool apogeeDetected = false;
  // maxAltitudeReached is now global static
  // descendingCount_apogee is now global static

  // Method 1: Barometric detection (primary)
  if (ms5611Sensor.isConnected()) { // Keep direct sensor check for availability
    if (currentAltitude > maxAltitudeReached) {
      maxAltitudeReached = currentAltitude;
      descendingCount_apogee = 0;
    } else if (currentAltitude < maxAltitudeReached - 1.0) { // Threshold for descent detection
      descendingCount_apogee++;
      if (descendingCount_apogee >= APOGEE_CONFIRMATION_COUNT) {
        Serial.println(F("APOGEE DETECTED (barometric)"));
        apogeeDetected = true;
      }
    }
  }

  // Method 2: Accelerometer-based detection (backup) - STUB for now, logic needs review with new accel getter
  // Original logic relied on direct kx134_accel[2] or icm_accel[2]
  // This needs to be adapted if we only have magnitude or need specific Z-axis component
  // For now, this part is simplified / placeholder
  // if (!apogeeDetected) {
  //   float accel_mag = get_current_accel_magnitude(); // This gives magnitude, not Z component
  //   // Logic based on Z-axis accel would need get_accel_vector() or similar
  // }


  // Method 3: Time-based detection (last resort)
  if (!apogeeDetected && boostEndTime > 0) {
    if (millis() - boostEndTime > BACKUP_APOGEE_TIME) { // ADAPTED to use BACKUP_APOGEE_TIME from config.h
      Serial.println(F("APOGEE DETECTED (time-based backup)"));
      apogeeDetected = true;
    }
  }
  return apogeeDetected;
}

// Redundant landing detection - ADAPTED
bool detectLanding() {
  static int stableCount = 0;
  bool landingDetectedLocal = false; // Use local flag to avoid premature global flag change

  // Method 1: Accelerometer stability (primary)
  float accel_magnitude = get_current_accel_magnitude(); // ADAPTED
  if (accel_magnitude > 0.0) { // Ensure valid reading
    if (accel_magnitude > 0.95 && accel_magnitude < 1.05) { // Close to 1g
      stableCount++;
    } else {
      stableCount = 0;
    }
    if (stableCount >= LANDING_CONFIRMATION_COUNT) {
      landingDetectedLocal = true;
    }
  }

  // Method 2: Barometric stability (backup)
  if (!landingDetectedLocal && ms5611Sensor.isConnected()) {
    static float lastAltitude = -1000.0f; // Initialize to an unlikely value
    static int altitudeStableCount = 0;
    float currentAltitude = get_current_baro_altitude(); // ADAPTED

    if (lastAltitude > -999.0f) { // Check if lastAltitude is initialized
        if (fabs(currentAltitude - lastAltitude) < 1.0) { // Altitude stable within 1m
            altitudeStableCount++;
        } else {
            altitudeStableCount = 0;
        }
    }
    lastAltitude = currentAltitude;

    if (altitudeStableCount >= LANDING_CONFIRMATION_COUNT) {
      landingDetectedLocal = true;
    }
  }

  if (landingDetectedLocal && !landingDetectedFlag) { // If newly detected
    Serial.println(F("LANDING DETECTED"));
    landingDetectedFlag = true; // Set the global flag
    // saveStateToEEPROM(); // Call to save state, ensure it's available
  }
  return landingDetectedLocal; // Return local status for current call
}

// Track boost end for time-based apogee detection - ADAPTED
void detectBoostEnd() {
  if (boostEndTime == 0) { // Only detect once
    float accel_magnitude = get_current_accel_magnitude(); // ADAPTED
    if (accel_magnitude < COAST_ACCEL_THRESHOLD && currentFlightState == BOOST) { // Ensure we are in BOOST state
      boostEndTime = millis();
      Serial.println(F("BOOST END DETECTED"));
      Serial.print(F("Time since startup: "));
      Serial.print(boostEndTime / 1000.0);
      Serial.println(F(" seconds"));
    }
  }
}

// Check if rocket is stable (no significant motion) - ADAPTED
bool IsStable() {
  float accel_magnitude = get_current_accel_magnitude(); // ADAPTED
  return (accel_magnitude > 0.95 && accel_magnitude < 1.05); // Close to 1g
}

// --- Existing update_guidance_targets() function ---
// Ensure includes for the new function are present.
// These might be redundant if already at the top, but ensures they are available.
#include "guidance_control.h" // For guidance_set_target_orientation_euler()
// #include "icm_20948_functions.h" // Already included at the top
// #include <Arduino.h>          // Already included via flight_logic.h

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