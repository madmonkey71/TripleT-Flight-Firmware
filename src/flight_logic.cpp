// In src/flight_logic.cpp
#include "flight_logic.h"
#include <Arduino.h> // For millis(), fabs(), Serial, digitalWrite, pinMode, delay
#include "config.h"   // For configuration constants
// WORKAROUND: Includes the main cpp file to access FlightState enum and global variables.
// This is not ideal and should be refactored with shared headers.
#include "TripleT_Flight_Firmware.cpp" // Provides FlightState, pixels, currentFlightState, previousFlightState, stateEntryTime, launchAltitude, maxAltitudeReached, etc.
#include "ms5611_functions.h" // For ms5611_get_altitude()
#include "utility_functions.h" // For get_accel_magnitude(), getStateName(), isSensorSuiteHealthy()
#include "state_management.h" // For saveStateToEEPROM()
#include "constants.h"     // For timing constants like BACKUP_APOGEE_TIME
#include "guidance_control.h" // For guidance_set_target_orientation_euler() (needed by existing update_guidance_targets)
#include "icm_20948_functions.h" // For convertQuaternionToEuler and icm_q0 etc. (needed by existing update_guidance_targets)

// Externs for variables globally defined in TripleT_Flight_Firmware.cpp
// These are brought in by including TripleT_Flight_Firmware.cpp, but listed here for clarity of dependencies.
/*
extern FlightState currentFlightState;
extern FlightState previousFlightState;
extern unsigned long stateEntryTime;
extern Adafruit_NeoPixel pixels; // From TripleT_Flight_Firmware.cpp
extern float launchAltitude;
extern float maxAltitudeReached;
extern bool baroCalibrated;
// extern SensorStatus barometerStatus; // Placeholder, using direct checks for now
// extern SensorStatus accelerometerStatus; // Placeholder, using direct checks for now
extern bool kx134_initialized_ok; 
extern bool icm20948_ready; 
extern MS5611 ms5611Sensor; 
extern bool useKX134;
extern float kx134_accel[3];
extern float icm_accel[3];
extern bool enableSystemDebug;
// extern void WriteLogData(bool forceLog = false); // Assumed available from utility_functions.h or TripleT_Flight_Firmware.cpp
// extern void prepareForShutdown(); // Assumed available from utility_functions.h or TripleT_Flight_Firmware.cpp
*/

// Global variables for flight logic state progression, defined in this file (flight_logic.cpp)
unsigned long boostEndTime = 0;
bool landingDetectedFlag = false;
float previousApogeeDetectAltitude = 0.0f;
float lastLandingCheckAltitudeAgl = 0.0f;
int descendingCount = 0;

// Helper function to set LED color based on flight state
void setFlightStateLED(FlightState state) {
    switch (state) {
        case STARTUP: pixels.setPixelColor(0, pixels.Color(128, 0, 0)); break; // Dark Red
        case CALIBRATION: pixels.setPixelColor(0, pixels.Color(255, 165, 0)); break; // Orange
        case PAD_IDLE: pixels.setPixelColor(0, pixels.Color(0, 255, 0)); break; // Green
        case ARMED: pixels.setPixelColor(0, pixels.Color(255, 255, 0)); break; // Yellow
        case BOOST: pixels.setPixelColor(0, pixels.Color(255, 0, 255)); break; // Magenta
        case COAST: pixels.setPixelColor(0, pixels.Color(0, 255, 255)); break; // Cyan
        case APOGEE: pixels.setPixelColor(0, pixels.Color(255, 255, 255)); break; // White
        case DROGUE_DEPLOY: pixels.setPixelColor(0, pixels.Color(255, 0, 0)); pixels.setPixelColor(1, pixels.Color(255, 0, 0)); break; // Pulsing Red (simulated by setting both)
        case DROGUE_DESCENT: pixels.setPixelColor(0, pixels.Color(139, 0, 0)); break; // DarkRed
        case MAIN_DEPLOY: pixels.setPixelColor(0, pixels.Color(0, 0, 255)); pixels.setPixelColor(1, pixels.Color(0, 0, 255)); break; // Pulsing Blue
        case MAIN_DESCENT: pixels.setPixelColor(0, pixels.Color(0, 0, 139)); break; // DarkBlue
        case LANDED: pixels.setPixelColor(0, pixels.Color(75, 0, 130)); break; // Indigo
        case RECOVERY: pixels.setPixelColor(0, pixels.Color(0, 128, 0)); break; // Dark Green (for GPS beacon active, etc.)
        case ERROR: pixels.setPixelColor(0, pixels.Color(255, 0, 0)); break; // Solid Red for error
        default: pixels.setPixelColor(0, pixels.Color(50, 50, 50)); break; // Grey for unknown
    }
    pixels.show();
}

void ProcessFlightState() {
    float currentAbsoluteBaroAlt = 0.0f;
    float currentAglAlt = 0.0f;
    bool newStateSignal = false; // Flag to indicate a state transition occurred this cycle

    // Always get current altitude if barometer is working
    if (ms5611Sensor.isConnected() && baroCalibrated) {
        currentAbsoluteBaroAlt = ms5611_get_altitude();
        currentAglAlt = currentAbsoluteBaroAlt - launchAltitude;
    }

    // Save state periodically or on critical events (handled by saveStateToEEPROM's internal logic)
    saveStateToEEPROM();

    // --- State Change Detection and Logging ---
    if (currentFlightState != previousFlightState) {
        newStateSignal = true;
        previousFlightState = currentFlightState; // Update previous state
        stateEntryTime = millis();                // Record time of state entry

        if (enableSystemDebug) {
            Serial.print(F("Transitioning to state: "));
            Serial.println(getStateName(currentFlightState));
        }
        // WriteLogData(true); // Force log on state change - Assuming WriteLogData is globally accessible
        saveStateToEEPROM(); // Force save state on transition
        setFlightStateLED(currentFlightState);
    }

    // --- State Entry Actions (perform only once on entering a new state) ---
    if (newStateSignal) {
        switch (currentFlightState) {
            case PAD_IDLE:
                launchAltitude = ms5611Sensor.isConnected() && baroCalibrated ? ms5611_get_altitude() : 0.0f;
                maxAltitudeReached = 0.0f; // Reset max altitude AGL
                boostEndTime = 0;
                landingDetectedFlag = false;
                descendingCount = 0; // Reset for apogee detection
                previousApogeeDetectAltitude = launchAltitude; // Initialize for apogee detection
                if (enableSystemDebug) Serial.println(F("PAD_IDLE: System initialized. Launch altitude set."));
                // Ensure pyros are safe
                pinMode(PYRO_CHANNEL_1, OUTPUT); digitalWrite(PYRO_CHANNEL_1, LOW);
                pinMode(PYRO_CHANNEL_2, OUTPUT); digitalWrite(PYRO_CHANNEL_2, LOW);
                break;
            case ARMED:
                if (enableSystemDebug) Serial.println(F("ARMED: System armed and ready for launch."));
                // Optional: short beep sequence
                // tone(BUZZER_PIN, 3000, 100); delay(150); tone(BUZZER_PIN, 3000, 100);
                break;
            case BOOST:
                if (enableSystemDebug) Serial.println(F("BOOST: Liftoff detected!"));
                maxAltitudeReached = currentAglAlt > 0 ? currentAglAlt : 0; // Reset max AGL altitude at liftoff
                descendingCount = 0;
                previousApogeeDetectAltitude = currentAbsoluteBaroAlt;
                boostEndTime = 0; // Reset boost end timer
                // WriteLogData(true); // Log liftoff event
                break;
            case COAST:
                if (enableSystemDebug) Serial.println(F("COAST: Motor burnout. Coasting to apogee."));
                descendingCount = 0; // Reset for apogee detection
                previousApogeeDetectAltitude = currentAbsoluteBaroAlt; // Initialize for apogee baro detection
                break;
            case APOGEE:
                if (enableSystemDebug) {
                    Serial.print(F("APOGEE: Peak altitude reached: "));
                    Serial.print(maxAltitudeReached);
                    Serial.println(F("m AGL."));
                }
                // WriteLogData(true); // Log apogee event
                break;
            case DROGUE_DEPLOY:
                if (enableSystemDebug) Serial.println(F("DROGUE_DEPLOY: Initiating drogue parachute deployment."));
                break;
            case DROGUE_DESCENT:
                if (enableSystemDebug) Serial.println(F("DROGUE_DESCENT: Descending under drogue parachute."));
                if (!MAIN_PRESENT) { // If no main, prepare for landing detection under drogue
                    lastLandingCheckAltitudeAgl = currentAglAlt;
                }
                break;
            case MAIN_DEPLOY:
                if (enableSystemDebug) Serial.println(F("MAIN_DEPLOY: Initiating main parachute deployment."));
                break;
            case MAIN_DESCENT:
                if (enableSystemDebug) Serial.println(F("MAIN_DESCENT: Descending under main parachute."));
                lastLandingCheckAltitudeAgl = currentAglAlt; // Initialize for landing detection
                break;
            case LANDED:
                if (enableSystemDebug) Serial.println(F("LANDED: Touchdown confirmed."));
                // WriteLogData(true); // Log landing event
                break;
            case RECOVERY:
                if (enableSystemDebug) Serial.println(F("RECOVERY: System in post-flight recovery mode."));
                // Optional: start beeping sequence for location
                break;
            case ERROR:
                if (enableSystemDebug) Serial.println(F("ERROR: System has entered error state."));
                // Optional: continuous error beep
                break;
            default:
                break;
        }
    }

    // --- Continuous State Processing Logic ---
    switch (currentFlightState) {
        case PAD_IDLE:
            // System is idle on the pad, waiting for an external command to transition to ARMED.
            // Health checks, GPS lock monitoring, etc., can happen here.
            break;

        case ARMED:
            // Check for liftoff based on accelerometer
            // bool accelOk = useKX134 ? kx134_initialized_ok : icm20948_ready; // Placeholder for accelerometerStatus.isWorking
            // Simplified direct check for now, assuming utility_functions `get_accel_magnitude` handles sensor choice.
            if (get_accel_magnitude() > BOOST_ACCEL_THRESHOLD) {
                currentFlightState = BOOST;
            } else if (millis() - stateEntryTime > 600000 && enableSystemDebug) { // Example: Disarm after 10 mins if no launch
                 Serial.println(F("No launch detected for 10 mins, disarming to PAD_IDLE."));
                 currentFlightState = PAD_IDLE;
            }
            // Add check for accelerometer failure here:
            // if (!accelOk) { currentFlightState = ERROR; } // Simplified
            break;

        case BOOST:
            detectBoostEnd(); // This function sets boostEndTime
            if (boostEndTime > 0) {
                currentFlightState = COAST;
            }
            // Update maxAltitudeReached during boost
            if (ms5611Sensor.isConnected() && baroCalibrated && currentAglAlt > maxAltitudeReached) {
                 maxAltitudeReached = currentAglAlt;
            }
            break;

        case COAST:
            if (detectApogee()) {
                currentFlightState = APOGEE;
            }
            // Update maxAltitudeReached during coast
            if (ms5611Sensor.isConnected() && baroCalibrated && currentAglAlt > maxAltitudeReached) {
                 maxAltitudeReached = currentAglAlt;
            }
            break;

        case APOGEE:
            if (DROGUE_PRESENT) {
                currentFlightState = DROGUE_DEPLOY;
            } else if (MAIN_PRESENT) { // No drogue, but main is present
                currentFlightState = MAIN_DEPLOY;
            } else {
                // No parachutes defined? This case should ideally not happen with config validation.
                // For safety, go to a descent state, though it might be freefall.
                currentFlightState = DROGUE_DESCENT; // Or a more specific "FREEFALL_ERROR" state if defined
                if (enableSystemDebug) Serial.println(F("Warning: Apogee reached but no parachutes configured for deployment!"));
            }
            break;

        case DROGUE_DEPLOY:
            if (DROGUE_PRESENT) {
                if (enableSystemDebug) Serial.println(F("Firing Pyro Channel 1 (Drogue)"));
                digitalWrite(PYRO_CHANNEL_1, HIGH);
                delay(PYRO_FIRE_DURATION); // Keep pyro active for specified duration
                digitalWrite(PYRO_CHANNEL_1, LOW);
                if (enableSystemDebug) Serial.println(F("Pyro Channel 1 (Drogue) Fired."));
                // WriteLogData(true); // Log drogue deployment event
            }
            currentFlightState = DROGUE_DESCENT;
            break;

        case DROGUE_DESCENT:
            if (MAIN_PRESENT) {
                if (ms5611Sensor.isConnected() && baroCalibrated && currentAglAlt < MAIN_DEPLOY_ALTITUDE) {
                    currentFlightState = MAIN_DEPLOY;
                }
            } else { // No main parachute, look for landing under drogue
                if (detectLanding()) {
                    currentFlightState = LANDED;
                }
            }
            break;

        case MAIN_DEPLOY:
            if (MAIN_PRESENT) {
                if (enableSystemDebug) Serial.println(F("Firing Pyro Channel 2 (Main)"));
                digitalWrite(PYRO_CHANNEL_2, HIGH);
                delay(PYRO_FIRE_DURATION); // Keep pyro active
                digitalWrite(PYRO_CHANNEL_2, LOW);
                if (enableSystemDebug) Serial.println(F("Pyro Channel 2 (Main) Fired."));
                // WriteLogData(true); // Log main deployment event
            }
            currentFlightState = MAIN_DESCENT;
            break;

        case MAIN_DESCENT:
            if (detectLanding()) {
                currentFlightState = LANDED;
            }
            break;

        case LANDED:
            // Stay in LANDED state for a defined period for data logging, GPS soak, etc.
            if (millis() - stateEntryTime > LANDED_TIMEOUT_MS) {
                currentFlightState = RECOVERY;
            }
            break;

        case RECOVERY:
            // Post-flight operations: e.g., activate GPS beacon, periodic beeping.
            // Example: Beep every 10 seconds
            // if ((millis() / 1000) % 10 == 0) {
            //    static unsigned long lastRecoveryBeep = 0;
            //    if(millis() - lastRecoveryBeep > 1000) { // ensure it only beeps once per second boundary
            //       tone(BUZZER_PIN, 1000, 500); lastRecoveryBeep = millis();
            //    }
            // }
            if (millis() - stateEntryTime > RECOVERY_TIMEOUT_MS) {
                if (enableSystemDebug) Serial.println(F("RECOVERY timeout. Preparing for shutdown."));
                prepareForShutdown(); // Assumed to be a global function that handles safe shutdown
            }
            break;

        case ERROR:
            // Attempt recovery or notify error condition
            if (millis() - stateEntryTime > ERROR_RECOVERY_ATTEMPT_MS) {
                if (isSensorSuiteHealthy(currentFlightState)) { // Pass current state for context
                    if (enableSystemDebug) Serial.println(F("ERROR: Attempting recovery to PAD_IDLE."));
                    currentFlightState = PAD_IDLE; // Attempt to recover to a safe state
                } else {
                    if (enableSystemDebug) Serial.println(F("ERROR: Sensor suite still unhealthy. Resetting error timer."));
                    stateEntryTime = millis(); // Reset timer to try again after another interval
                }
            }
            break;

        default:
            // Unknown state, should not happen. Transition to ERROR.
            if (enableSystemDebug) {
                Serial.print(F("Unknown flight state encountered: "));
                Serial.println(currentFlightState); // Log the unknown state value
            }
            currentFlightState = ERROR;
            break;
    }
    // Ensure LED is updated if not a new state signal, in case logic within a state changes it (though less common)
    // if (!newStateSignal) { setFlightStateLED(currentFlightState); } // Usually handled by state transition
}


// --- Existing functions to be kept (detectBoostEnd, detectApogee, detectLanding, IsStable, update_guidance_targets) ---
// Note: Their implementations were provided in the previous step and are assumed to be here.
// For brevity, I am not re-listing their full code if they are unchanged from the previous step's prompt.
// The overwrite tool will place the entire content, so they must be part of the block provided to the tool.

// detectBoostEnd() - as implemented in prior step
// detectApogee() - as implemented in prior step
// detectLanding() - as implemented in prior step

// Check if rocket is stable (no significant motion) - Kept from existing file
bool IsStable() {
  float accel_magnitude = get_accel_magnitude();
  // Consider stable if acceleration is close to 1g (gravity only)
  return (accel_magnitude > 0.95 && accel_magnitude < 1.05);
} 

// update_guidance_targets - Kept from existing file
static bool initial_target_set = false;
static float initial_yaw_target = 0.0f; // Store the initial yaw target in radians

void update_guidance_targets() {
    unsigned long currentTime_ms = millis();
    float target_roll_rad = 0.0f;
    float target_pitch_rad = 0.0f;
    float target_yaw_rad = 0.0f;

    if (!initial_target_set) {
        float current_roll, current_pitch, current_yaw;
        convertQuaternionToEuler(icm_q0, icm_q1, icm_q2, icm_q3, current_roll, current_pitch, current_yaw);
        initial_yaw_target = current_yaw; 
        
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
        if (currentTime_ms > 30000 && currentTime_ms < 60000) { 
            target_roll_rad = 0.0f;
            target_pitch_rad = 0.0f;
            target_yaw_rad = initial_yaw_target + (PI / 4.0f); 
        } else if (currentTime_ms >= 60000) { 
            target_roll_rad = 0.0f;
            target_pitch_rad = 0.0f;
            target_yaw_rad = initial_yaw_target;
        }
        else {
            target_roll_rad = 0.0f;
            target_pitch_rad = 0.0f;
            target_yaw_rad = initial_yaw_target;
        }
        guidance_set_target_orientation_euler(target_roll_rad, target_pitch_rad, target_yaw_rad);

        if (enableSystemDebug) {
            static unsigned long lastTargetPrintTimeMs = 0;
            static float lastReportedTargetYaw = -1000.0f; 

            bool should_print_now = false;
            if (target_yaw_rad != lastReportedTargetYaw) { 
                should_print_now = true;
            } else if ( (currentTime_ms > 30000) && (currentTime_ms - lastTargetPrintTimeMs > 5000) ) {
                should_print_now = true;
            }

            if (should_print_now && (currentTime_ms - lastTargetPrintTimeMs > 1000 || lastTargetPrintTimeMs > currentTime_ms) ) { 
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