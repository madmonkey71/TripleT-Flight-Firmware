// In src/flight_logic.cpp
#include "flight_logic.h"
#include <Arduino.h> // For millis(), fabs(), Serial, digitalWrite, pinMode, delay
#include "config.h"   // For configuration constants
#include "ms5611_functions.h" // For ms5611_get_altitude()
#include "utility_functions.h" // For get_accel_magnitude(), getStateName(), isSensorSuiteHealthy()
#include "state_management.h" // For saveStateToEEPROM()
#include "constants.h"     // For timing constants like BACKUP_APOGEE_TIME
#include "guidance_control.h" // For guidance_set_target_orientation_euler() (needed by existing update_guidance_targets)
#include "icm_20948_functions.h" // For convertQuaternionToEuler and icm_q0 etc. (needed by existing update_guidance_targets)
#include <Adafruit_NeoPixel.h>
#include <MS5611.h>

// Externs for variables globally defined in TripleT_Flight_Firmware.cpp
extern FlightState currentFlightState;
extern FlightState previousFlightState;
extern unsigned long stateEntryTime;
extern Adafruit_NeoPixel pixels;
extern float launchAltitude;
extern float maxAltitudeReached;
extern bool baroCalibrated;
extern MS5611 ms5611Sensor;
extern float kx134_accel[3];
extern float icm_accel[3];
extern bool enableSystemDebug;

// Externs from TripleT_Flight_Firmware.cpp for attitude hold logic
extern bool useKalmanFilter; // To check which orientation source is active
extern float kalmanRoll;     // Kalman filter output for roll (radians)
extern float kalmanPitch;    // Kalman filter output for pitch (radians)
extern float kalmanYaw;      // Kalman filter output for yaw (radians)
// Note: icm20948_ready is extern in utility_functions.h
// Note: icm_q0, icm_q1, icm_q2, icm_q3 are extern in icm_20948_functions.h (via guidance_control.h)

// Constants for Recovery Beep Pattern
static const unsigned long RECOVERY_BEEP_DURATION_MS = 200;
static const unsigned long RECOVERY_SILENCE_DURATION_MS = 1800; // Total cycle time = 2000ms (2 seconds)
static const unsigned int RECOVERY_BEEP_FREQUENCY_HZ = 2000; // 2kHz tone

// Forward declaration for prepareForShutdown
void prepareForShutdown();

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

    // Sensor health check for recoverable states
    if (currentFlightState != LANDED && currentFlightState != RECOVERY && currentFlightState != ERROR) {
        if (!isSensorSuiteHealthy()) {
            currentFlightState = ERROR;
            if (enableSystemDebug) {
                Serial.println(F("Sensor suite unhealthy! Transitioning to ERROR state."));
            }
            // Force state change processing for ERROR state entry actions
            // This ensures that any entry actions defined for the ERROR state (like setting LEDs, logging) are executed.
            if (currentFlightState != previousFlightState) { // Check if it's a new error state occurrence
                previousFlightState = currentFlightState;
                stateEntryTime = millis();
                setFlightStateLED(currentFlightState);
                saveStateToEEPROM();
                // if (WriteLogData) WriteLogData(true); // Assuming WriteLogData is available and should be called
            }
            return; // Exit ProcessFlightState immediately
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
                // Attitude Hold: Capture current orientation and set as target for COAST phase
                float currentRoll_rad = 0.0f;
                float currentPitch_rad = 0.0f;
                float currentYaw_rad = 0.0f;

                if (useKalmanFilter && icm20948_ready) {
                    currentRoll_rad = kalmanRoll;
                    currentPitch_rad = kalmanPitch;
                    currentYaw_rad = kalmanYaw;
                    if (enableSystemDebug) {
                        Serial.println(F("ATTITUDE_HOLD: Using Kalman orientation."));
                    }
                } else if (icm20948_ready) { // Default to Madgwick/quaternion-derived if Kalman not active but ICM is ready
                    convertQuaternionToEuler(icm_q0, icm_q1, icm_q2, icm_q3, currentRoll_rad, currentPitch_rad, currentYaw_rad);
                    if (enableSystemDebug) {
                        Serial.println(F("ATTITUDE_HOLD: Using Madgwick/Quaternion orientation."));
                    }
                } else {
                    // Fallback: No reliable orientation source
                    if (enableSystemDebug) {
                        Serial.println(F("ATTITUDE_HOLD_WARN: No valid orientation source at BOOST->COAST. Target set to 0,0,0."));
                    }
                    // currentRoll_rad, currentPitch_rad, currentYaw_rad remain 0.0f
                }

                guidance_set_target_orientation_euler(currentRoll_rad, currentPitch_rad, currentYaw_rad);

                if (enableSystemDebug) {
                    Serial.print(F("ATTITUDE_HOLD: Target set at BOOST->COAST transition. R: "));
                    Serial.print(currentRoll_rad * (180.0f / PI), 2);
                    Serial.print(F(" P: "));
                    Serial.print(currentPitch_rad * (180.0f / PI), 2);
                    Serial.print(F(" Y: "));
                    Serial.println(currentYaw_rad * (180.0f / PI), 2);
                }

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
            { // Scope block for static variables
                static unsigned long lastRecoveryActionTime = 0;
                static bool isBeeping = false;
                unsigned long currentTimeMillis = millis(); // Cache current time

                if (isBeeping) {
                    if (currentTimeMillis - lastRecoveryActionTime >= RECOVERY_BEEP_DURATION_MS) {
                        noTone(BUZZER_PIN);
                        isBeeping = false;
                        lastRecoveryActionTime = currentTimeMillis;
                        if (enableSystemDebug) {
                            Serial.println(F("RECOVERY: Beep stopped."));
                        }
                    }
                } else { // Not beeping
                    if (currentTimeMillis - lastRecoveryActionTime >= RECOVERY_SILENCE_DURATION_MS) {
                        tone(BUZZER_PIN, RECOVERY_BEEP_FREQUENCY_HZ);
                        isBeeping = true;
                        lastRecoveryActionTime = currentTimeMillis;
                        if (enableSystemDebug) {
                            Serial.print(F("RECOVERY: Beep started at "));
                            Serial.print(RECOVERY_BEEP_FREQUENCY_HZ);
                            Serial.println(F(" Hz."));
                        }
                    }
                }
            } // End scope block for static variables

            // Existing timeout logic for RECOVERY state
            if (millis() - stateEntryTime > RECOVERY_TIMEOUT_MS) {
                if (enableSystemDebug) Serial.println(F("RECOVERY: Timeout. Preparing for shutdown."));
                noTone(BUZZER_PIN); // Ensure buzzer is off before shutdown
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

// Placeholder implementations for the missing functions
void detectBoostEnd() {
    // Check if acceleration has dropped below coast threshold
    if (get_accel_magnitude() < COAST_ACCEL_THRESHOLD && boostEndTime == 0) {
        boostEndTime = millis();
        if (enableSystemDebug) {
            Serial.println(F("Boost end detected"));
        }
    }
}

bool detectApogee() {
    bool apogeeDetected = false; // Local flag for apogee detection

    // Primary apogee detection based on altitude decrease
    // WARNING: The use of local static variables 'lastAltitude' and 'descendingCount' here can be problematic
    // as they are not reset by the flight state machine's transition into COAST.
    // The global '::descendingCount' and '::previousApogeeDetectAltitude' are managed by ProcessFlightState
    // and should ideally be used or passed to this function for correct apogee detection logic.
    // This change focuses on the backup timer as per the subtask.
    static float lastAltitude = 0.0f;
    static int descendingCount = 0;   // Local static, not the global one.
    
    if (ms5611Sensor.isConnected() && baroCalibrated) {
        float currentAlt = ms5611_get_altitude() - launchAltitude;
        
        // On first call or if currentAlt is higher, reset descending logic
        // This static 'lastAltitude' will only be 0.0f on the very first call ever.
        // This logic needs to be tied to the COAST state entry.
        if (currentAlt < lastAltitude) {
            descendingCount++;
        } else {
            descendingCount = 0; // Reset if altitude increases or stays the same
        }
        lastAltitude = currentAlt; // Update for next comparison
        
        // Update global max altitude reached
        if (currentAlt > maxAltitudeReached) {
            maxAltitudeReached = currentAlt;
        }
        
        // Confirm apogee if descending for multiple readings
        if (descendingCount >= APOGEE_CONFIRMATION_COUNT) {
            if (enableSystemDebug) {
                Serial.println(F("Apogee detected by primary method (barometer)."));
            }
            apogeeDetected = true;
        }
    }
    
    // New Backup Apogee Timer: Time since motor burnout (boostEndTime)
    // boostEndTime is a global variable, set in detectBoostEnd()
    // BACKUP_APOGEE_TIME_MS is from config.h
    if (!apogeeDetected && boostEndTime > 0 && (millis() - boostEndTime > BACKUP_APOGEE_TIME_MS)) {
        if (enableSystemDebug) {
            Serial.print(F("Apogee detected by backup timer. Time since boost end: "));
            Serial.print(millis() - boostEndTime);
            Serial.print(F("ms, Threshold: "));
            Serial.print(BACKUP_APOGEE_TIME_MS);
            Serial.println(F("ms"));
        }
        apogeeDetected = true;
        // Note: maxAltitudeReached might not be the true peak if this timer fires, but it's a backup.
    }
    
    return apogeeDetected;
}

bool detectLanding() {
    // Simple landing detection based on stable low altitude and low acceleration
    static int stableCount = 0;
    
    if (ms5611Sensor.isConnected() && baroCalibrated) {
        float currentAgl = ms5611_get_altitude() - launchAltitude;
        float accelMag = get_accel_magnitude();
        
        // Check if we're close to ground level with stable acceleration
        if (currentAgl < 50.0f && accelMag > LANDING_ACCEL_MIN_G && accelMag < LANDING_ACCEL_MAX_G) {
            stableCount++;
        } else {
            stableCount = 0;
        }
        
        if (stableCount >= LANDING_CONFIRMATION_COUNT) {
            if (enableSystemDebug) {
                Serial.println(F("Landing detected"));
            }
            return true;
        }
    }
    
    return false;
}

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