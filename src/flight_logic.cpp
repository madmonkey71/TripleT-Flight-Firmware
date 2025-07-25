// In src/flight_logic.cpp
#include "flight_logic.h"
#include <Arduino.h> // For millis(), fabs(), Serial, digitalWrite, pinMode, delay
#include "config.h"   // For configuration constants
#include "ms5611_functions.h" // For ms5611_get_altitude()
#include "utility_functions.h" // For get_accel_magnitude(), getStateName(), isSensorSuiteHealthy()
#include "state_management.h" // For saveStateToEEPROM()
#include "constants.h"     // For timing constants like BACKUP_APOGEE_TIME
#include "guidance_control.h" // For guidance_set_target_orientation_euler()
#include "icm_20948_functions.h" // For convertQuaternionToEuler and icm_q0 etc.
#include "gps_functions.h" // For getGPSAltitude() and getFixType()
#include <Adafruit_NeoPixel.h>
#include <MS5611.h>
#include "debug_flags.h" // For g_debugFlags
#include "kx134_functions.h"
#include "icm_20948_functions.h"

// Externs for variables globally defined in TripleT_Flight_Firmware.cpp
#include "error_codes.h" // For ErrorCode_t
extern ErrorCode_t g_last_error_code; // For accessing last error code
extern FlightState g_currentFlightState;
extern FlightState g_previousFlightState;
extern unsigned long g_stateEntryTime;
extern Adafruit_NeoPixel g_pixels;
extern float g_launchAltitude;
extern float g_maxAltitudeReached;
extern bool g_baroCalibrated;
extern MS5611 g_ms5611Sensor;
// kx134_accel and icm_accel are defined in their respective _functions.cpp files and externed in their .h files.
// flight_logic.cpp includes kx134_functions.h and icm_20948_functions.h, so these externs are not needed here.
// extern float kx134_accel[3];
// extern float icm_accel[3];
extern bool g_kx134_initialized_ok; // This IS defined in TripleT_Flight_Firmware.cpp
extern bool g_icm20948_ready;
extern bool g_useKalmanFilter; // Always true - Kalman is the only orientation filter
extern DebugFlags g_debugFlags; // To access g_debugFlags.enableSystemDebug
extern float g_main_deploy_altitude_m_agl;
extern bool ms5611_initialized_ok; // Declared extern for isSensorSuiteHealthy function

// Externs from TripleT_Flight_Firmware.cpp for attitude hold logic
extern float g_kalmanRoll;
extern float g_kalmanPitch;
extern float g_kalmanYaw;

// Global variables for flight logic state progression, defined in this file
unsigned long boostEndTime = 0;

// Global variables for flight logic state progression, defined in this file (flight_logic.cpp)
bool landingDetectedFlag = false;
float previousApogeeDetectAltitude = 0.0f;
float lastLandingCheckAltitudeAgl = 0.0f;
int descendingCount = 0;
static unsigned long lastStateBroadcastTime = 0; // For broadcasting state when CSV is off

// Helper function to set LED color based on flight state
void setFlightStateLED(FlightState state) {
    switch (state) {
        case STARTUP: g_pixels.setPixelColor(0, g_pixels.Color(128, 0, 0)); break;
        case CALIBRATION: g_pixels.setPixelColor(0, g_pixels.Color(255, 165, 0)); break;
        case PAD_IDLE: g_pixels.setPixelColor(0, g_pixels.Color(0, 255, 0)); break;
        case ARMED: g_pixels.setPixelColor(0, g_pixels.Color(255, 255, 0)); break;
        case BOOST: g_pixels.setPixelColor(0, g_pixels.Color(255, 0, 255)); break;
        case COAST: g_pixels.setPixelColor(0, g_pixels.Color(0, 255, 255)); break;
        case APOGEE: g_pixels.setPixelColor(0, g_pixels.Color(255, 255, 255)); break;
        case DROGUE_DEPLOY: g_pixels.setPixelColor(0, g_pixels.Color(255, 0, 0)); g_pixels.setPixelColor(1, g_pixels.Color(255, 0, 0)); break;
        case DROGUE_DESCENT: g_pixels.setPixelColor(0, g_pixels.Color(139, 0, 0)); break;
        case MAIN_DEPLOY: g_pixels.setPixelColor(0, g_pixels.Color(0, 0, 255)); g_pixels.setPixelColor(1, g_pixels.Color(0, 0, 255)); break;
        case MAIN_DESCENT: g_pixels.setPixelColor(0, g_pixels.Color(0, 0, 139)); break;
        case LANDED: g_pixels.setPixelColor(0, g_pixels.Color(75, 0, 130)); break;
        case RECOVERY: g_pixels.setPixelColor(0, g_pixels.Color(0, 128, 0)); break;
        case ERROR: g_pixels.setPixelColor(0, g_pixels.Color(255, 0, 0)); break;
        default: g_pixels.setPixelColor(0, g_pixels.Color(50, 50, 50)); break;
    }
    g_pixels.show();
}

void ProcessFlightState() {
    float currentAbsoluteBaroAlt = 0.0f;
    float currentAglAlt = 0.0f;
    bool newStateSignal = false;
    static unsigned long lastErrorCheckTime = 0;
    static unsigned long lastErrorClearTime = 0; // Track when errors were last cleared
    const unsigned long errorCheckInterval = 1000; // 1 second
    const unsigned long errorClearGracePeriod = 5000; // 5 seconds grace period after clearing errors
    const unsigned long stateBroadcastInterval = 1000; // 1 second
    unsigned long currentTimeMillis = millis(); // Declare here to avoid case label crossing issues

    // If CSV is OFF, periodically send the current state to the web UI
    if (!g_debugFlags.enableSerialCSV) {
        if (millis() - lastStateBroadcastTime > stateBroadcastInterval) {
            lastStateBroadcastTime = millis();
            // JSON format: {"state_id": 1, "state_name": "PAD_IDLE"}
            Serial.print(F("{\"state_id\":"));
            Serial.print(static_cast<int>(g_currentFlightState));
            Serial.print(F(",\"state_name\":\""));
            Serial.print(getStateName(g_currentFlightState));
            Serial.println(F("\"}"));
        }
    }

    // Defer the health check to avoid immediate re-entry into ERROR state
    // after a manual `clear_errors` command.
    if (g_currentFlightState != LANDED && g_currentFlightState != RECOVERY && g_currentFlightState != ERROR) {
        // Add grace period check - don't run health checks immediately after clearing errors
        bool withinGracePeriod = (millis() - lastErrorClearTime < errorClearGracePeriod);
        
        if (millis() - lastErrorCheckTime > errorCheckInterval && !withinGracePeriod) {
            lastErrorCheckTime = millis();
            if (!isSensorSuiteHealthy(g_currentFlightState)) { // isSensorSuiteHealthy uses g_baroCalibrated, g_icm20948_ready, g_kx134_initialized_ok, myGNSS
                // ALWAYS log detailed sensor status before transitioning to ERROR (regardless of debug flags)
                Serial.println(F("--- CRITICAL: Sensor Suite Health Check Failed ---"));
                Serial.print(F("Current State: "));
                Serial.println(getStateName(g_currentFlightState));
                Serial.print(F("Time since last error clear: "));
                Serial.print((millis() - lastErrorClearTime) / 1000.0, 1);
                Serial.println(F(" seconds"));
                Serial.print(F("Grace period remaining: "));
                Serial.print((errorClearGracePeriod - (millis() - lastErrorClearTime)) / 1000.0, 1);
                Serial.println(F(" seconds"));
                Serial.println(F(""));
                isSensorSuiteHealthy(g_currentFlightState, true); // Call with verbose=true
                Serial.println(F(""));
                Serial.println(F("REASON: Periodic health check failed during normal operation"));
                Serial.println(F("Transitioning to ERROR state..."));
                Serial.println(F("Use 'clear_errors' command to attempt recovery."));
                Serial.println(F("--------------------------------------------------"));
                
                g_last_error_code = STATE_TRANSITION_INVALID_HEALTH; // Set error code
                g_currentFlightState = ERROR;
                g_stateEntryTime = millis(); // Ensure state entry time is updated
                // When we transition to error, we should immediately save and log.
                saveStateToEEPROM();
                WriteLogData(true); // Log immediately with the error code
                setFlightStateLED(g_currentFlightState);
                g_pixels.show(); // Explicitly show error LED
                return; // Avoid further processing this cycle
            } else {
                // Add periodic health status when things are OK
                static unsigned long lastHealthOkTime = 0;
                if (millis() - lastHealthOkTime > 10000) { // Every 10 seconds (reduced frequency)
                    lastHealthOkTime = millis();
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.print(F("Health check OK for state: "));
                        Serial.println(getStateName(g_currentFlightState));
                    }
                }
            }
        } else if (withinGracePeriod && g_debugFlags.enableSystemDebug) {
            // Debug message about grace period
            static unsigned long lastGraceMsg = 0;
            if (millis() - lastGraceMsg > 2000) { // Every 2 seconds during grace period
                lastGraceMsg = millis();
                Serial.print(F("Grace period active: "));
                Serial.print((errorClearGracePeriod - (millis() - lastErrorClearTime)) / 1000.0, 1);
                Serial.println(F(" seconds remaining"));
            }
        }
    } else if (g_currentFlightState == ERROR) {
        // Add automatic error recovery logic - check if system has become healthy
        static unsigned long lastAutoRecoveryCheckTime = 0;
        const unsigned long autoRecoveryCheckInterval = 2000; // Check every 2 seconds
        
        if (millis() - lastAutoRecoveryCheckTime > autoRecoveryCheckInterval) {
            lastAutoRecoveryCheckTime = millis();
            
            // Check if we can recover to PAD_IDLE state
            if (isSensorSuiteHealthy(PAD_IDLE)) {
                Serial.println(F("--- AUTOMATIC ERROR RECOVERY ---"));
                Serial.println(F("System health has been restored. Automatically clearing ERROR state."));
                
                // Determine target state based on barometer calibration status
                if (g_baroCalibrated) {
                    Serial.println(F("All systems healthy and barometer calibrated, transitioning to PAD_IDLE."));
                    g_currentFlightState = PAD_IDLE;
                } else if (ms5611_initialized_ok) {
                    Serial.println(F("Systems healthy but barometer needs calibration, transitioning to CALIBRATION."));
                    g_currentFlightState = CALIBRATION;
                } else {
                    Serial.println(F("Systems partially healthy but barometer not initialized, remaining in ERROR."));
                    // Don't transition out of ERROR if barometer isn't even initialized
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.println(F("Manual intervention may be required for barometer initialization."));
                    }
                    return; // Don't transition out of ERROR
                }
                
                lastErrorClearTime = millis(); // Set grace period for future health checks
                g_stateEntryTime = millis();
                Serial.println(F("ERROR state automatically cleared - starting grace period for health checks"));
                saveStateToEEPROM();
                setFlightStateLED(g_currentFlightState);
                Serial.println(F("--------------------------------"));
                return; // Exit early to avoid the debug message below
            }
        }
        
        // Add periodic debugging for ERROR state (only if we didn't auto-recover)
        if (g_debugFlags.enableSystemDebug) {
            static unsigned long lastErrorDebugTime = 0;
            if (millis() - lastErrorDebugTime > 5000) { // Every 5 seconds (reduced frequency)
                lastErrorDebugTime = millis();
                Serial.println(F("--- Currently in ERROR state ---"));
                Serial.println(F("Use 'clear_errors' command to manually clear if all systems are working."));
                Serial.println(F("Or check sensor health with detailed report:"));
                isSensorSuiteHealthy(PAD_IDLE, true);
                Serial.println(F("--------------------------------"));
            }
        }
    }

    // Record when we transition OUT of ERROR state (for grace period tracking)
    static FlightState lastRecordedState = STARTUP;
    if (lastRecordedState == ERROR && g_currentFlightState != ERROR) {
        lastErrorClearTime = millis();
        Serial.println(F("ERROR state cleared - starting grace period for health checks"));
    }
    lastRecordedState = g_currentFlightState;

    if (g_ms5611Sensor.isConnected() && g_baroCalibrated) {
        currentAbsoluteBaroAlt = ms5611_get_altitude();
        currentAglAlt = currentAbsoluteBaroAlt - g_launchAltitude;
    }

    if (g_currentFlightState != g_previousFlightState) {
        newStateSignal = true;
        g_previousFlightState = g_currentFlightState;
        g_stateEntryTime = millis();

        if (g_debugFlags.enableSystemDebug) {
            Serial.print(F("Transitioning to state: "));
            Serial.println(getStateName(g_currentFlightState));
        }
        WriteLogData(true);
        saveStateToEEPROM();
        setFlightStateLED(g_currentFlightState);
    }

    if (newStateSignal) {
        switch (g_currentFlightState) {
            case CALIBRATION:
                // Initial message when entering CALIBRATION state
                if (g_debugFlags.enableSystemDebug) {
                    Serial.println(F("STATE: Entered CALIBRATION - Waiting for barometer calibration via 'calibrate' command. LED should be Orange."));
                }
                // Actual periodic waiting message and transition logic is in the main switch block below.
                break;
            case PAD_IDLE:
                g_launchAltitude = g_ms5611Sensor.isConnected() && g_baroCalibrated ? ms5611_get_altitude() : 0.0f;
                g_maxAltitudeReached = 0.0f;
                boostEndTime = 0;
                landingDetectedFlag = false;
                descendingCount = 0;
                previousApogeeDetectAltitude = g_launchAltitude;
                if (g_debugFlags.enableSystemDebug) Serial.println(F("PAD_IDLE: System initialized. Launch altitude set."));
                pinMode(PYRO_CHANNEL_1, OUTPUT); digitalWrite(PYRO_CHANNEL_1, LOW);
                pinMode(PYRO_CHANNEL_2, OUTPUT); digitalWrite(PYRO_CHANNEL_2, LOW);
                break;
            case ARMED:
                if (g_debugFlags.enableSystemDebug) Serial.println(F("ARMED: System armed and ready for launch."));
                if (g_baroCalibrated && g_ms5611Sensor.isConnected()) {
                    g_main_deploy_altitude_m_agl = currentAglAlt + MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M;
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.print(F("ARMED: Dynamic main deployment altitude set to: "));
                        Serial.print(g_main_deploy_altitude_m_agl, 2);
                        Serial.println(F(" m AGL"));
                    }
                } else {
                    g_main_deploy_altitude_m_agl = MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M;
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.print(F("ARMED_WARN: Baro not ready. Main deploy altitude defaulting to fixed height: "));
                        Serial.print(g_main_deploy_altitude_m_agl, 2);
                        Serial.println(F(" m above current launch altitude."));
                    }
                }
                break;
            case BOOST:
                if (g_useKalmanFilter && !g_icm20948_ready) {
                    g_currentFlightState = ERROR;
                    if (g_debugFlags.enableSystemDebug) Serial.println(F("ERROR: Cannot detect liftoff; ICM20948 not ready."));
                }
                if (g_debugFlags.enableSystemDebug) Serial.println(F("BOOST: Liftoff detected!"));
                g_maxAltitudeReached = currentAglAlt > 0 ? currentAglAlt : 0;
                boostEndTime = 0;
                break;
            case COAST:
                if (g_debugFlags.enableSystemDebug) Serial.println(F("COAST: Motor burnout. Coasting to apogee."));
                descendingCount = 0;
                previousApogeeDetectAltitude = currentAbsoluteBaroAlt;
                break;
            case APOGEE:
                if (g_debugFlags.enableSystemDebug) {
                    Serial.print(F("APOGEE: Peak altitude reached: "));
                    Serial.print(g_maxAltitudeReached);
                    Serial.println(F("m AGL."));
                }
                break;
            case DROGUE_DEPLOY:
                if (g_debugFlags.enableSystemDebug) Serial.println(F("DROGUE_DEPLOY: Initiating drogue parachute deployment."));
                break;
            case DROGUE_DESCENT:
                if (g_debugFlags.enableSystemDebug) Serial.println(F("DROGUE_DESCENT: Descending under drogue parachute."));
                if (!MAIN_PRESENT) {
                    lastLandingCheckAltitudeAgl = currentAglAlt;
                }
                break;
            case MAIN_DEPLOY:
                if (g_debugFlags.enableSystemDebug) Serial.println(F("MAIN_DEPLOY: Initiating main parachute deployment."));
                break;
            case MAIN_DESCENT:
                if (g_debugFlags.enableSystemDebug) Serial.println(F("MAIN_DESCENT: Descending under main parachute."));
                lastLandingCheckAltitudeAgl = currentAglAlt;
                break;
            case LANDED:
                if (g_debugFlags.enableSystemDebug) Serial.println(F("LANDED: Touchdown confirmed."));
                break;
            case RECOVERY:
                if (g_debugFlags.enableSystemDebug) Serial.println(F("RECOVERY: System in post-flight recovery mode."));
                break;
            case ERROR: {
                // Buzzer Pattern for ERROR state
                static unsigned long lastErrorBuzzerTime = 0;
                static bool errorBeepState = false;
                unsigned long currentTimeMillis_error = millis(); // Use a distinct name to avoid conflict if currentTimeMillis is used elsewhere in a larger scope
                if (currentTimeMillis_error - lastErrorBuzzerTime > 200) { // Fast beeping
                    lastErrorBuzzerTime = currentTimeMillis_error;
                    errorBeepState = !errorBeepState;
                    if (errorBeepState && BUZZER_OUTPUT) {
                        tone(BUZZER_PIN, 2500); // Example error frequency
                    } else if (BUZZER_OUTPUT) {
                        noTone(BUZZER_PIN);
                    }
                }

                // Periodic Error Code Printout
                static unsigned long lastErrorSerialPrintTime = 0;
                // Use g_debugFlags.enableSystemDebug or a new specific flag if desired
                if (g_debugFlags.enableSystemDebug && (currentTimeMillis_error - lastErrorSerialPrintTime > 5000)) { // Print every 5 seconds
                    lastErrorSerialPrintTime = currentTimeMillis_error;
                    Serial.print(F("SYSTEM ERROR STATE - Last Error Code: "));
                    Serial.print(static_cast<int>(g_last_error_code));
                    Serial.print(F(" ("));
                    Serial.print(getErrorCodeName(g_last_error_code)); // Assumes getErrorCodeName is available
                    Serial.println(F(")"));
                    Serial.println(F("Use 'clear_errors' or check 'status_sensors'/'b' command for more info."));
                }
                // Note: Automatic recovery logic is handled in the main part of ProcessFlightState
                // before the switch statement if g_currentFlightState is ERROR.
            }
                break;
            default:
                Serial.print(F("CRITICAL ERROR: Unknown flight state encountered: "));
                Serial.println(static_cast<int>(g_currentFlightState));
                Serial.println(F("Transitioning to ERROR state for safety."));
                g_currentFlightState = ERROR;
                break;
        }
    }

    switch (g_currentFlightState) {
        case CALIBRATION:
            if (g_baroCalibrated) {
                g_currentFlightState = PAD_IDLE;
                if (g_debugFlags.enableSystemDebug) {
                    Serial.println(F("CALIBRATION: Barometer calibrated, transitioning to PAD_IDLE."));
                }
            } else {
                // Periodic message while waiting in CALIBRATION state
                static unsigned long lastCalibWaitMsgTime = 0;
                if (g_debugFlags.enableSystemDebug && (millis() - lastCalibWaitMsgTime > 5000)) { // Print every 5s
                    Serial.println(F("CALIBRATION: Waiting for barometer calibration (use 'calibrate' command)..."));
                    lastCalibWaitMsgTime = millis();
                }
            }
            break;
        case PAD_IDLE:
            // PAD_IDLE is a stable state - no automatic transitions
            // Transitions to ARMED happen via command processor
            break;
        case ARMED:
            if (get_accel_magnitude(g_kx134_initialized_ok, kx134_accel, g_icm20948_ready, icm_accel, g_debugFlags.enableSystemDebug) > BOOST_ACCEL_THRESHOLD) {
                g_currentFlightState = BOOST;
            }
            break;
        case BOOST:
            detectBoostEnd();
            if (boostEndTime > 0) {
                float targetRollRad = 0.0f, targetPitchRad = 0.0f, targetYawRad = 0.0f;
                if (g_useKalmanFilter && g_icm20948_ready) {
                    targetRollRad = g_kalmanRoll;
                    targetPitchRad = g_kalmanPitch;
                    targetYawRad = g_kalmanYaw;
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.println(F("ATT_HOLD: Using Kalman orientation for target at BOOST->COAST."));
                    }
                } else if (g_icm20948_ready) {
                    convertQuaternionToEuler(icm_q0, icm_q1, icm_q2, icm_q3, targetRollRad, targetPitchRad, targetYawRad);
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.println(F("ATT_HOLD: Using ICM quaternion conversion for target at BOOST->COAST."));
                    }
                } else {
                    targetRollRad = 0.0f;
                    targetPitchRad = 0.0f;
                    targetYawRad = 0.0f;
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.println(F("ATT_HOLD_WARN: No valid orientation source at BOOST->COAST. Setting target to 0,0,0."));
                    }
                }
                guidance_set_target_orientation_euler(targetRollRad, targetPitchRad, targetYawRad);
                if (g_debugFlags.enableSystemDebug) {
                    Serial.print(F("ATT_HOLD: Set target orientation R:")); Serial.print(targetRollRad * (180.0f/PI), 2);
                    Serial.print(F(" P:")); Serial.print(targetPitchRad * (180.0f/PI), 2);
                    Serial.print(F(" Y:")); Serial.println(targetYawRad * (180.0f/PI), 2);
                }
                g_currentFlightState = COAST;
            }
            if (g_ms5611Sensor.isConnected() && g_baroCalibrated && currentAglAlt > g_maxAltitudeReached) {
                 g_maxAltitudeReached = currentAglAlt;
            }
            break;
        case COAST:
            if (detectApogee()) {
                g_currentFlightState = APOGEE;
            }
            if (g_ms5611Sensor.isConnected() && g_baroCalibrated && currentAglAlt > g_maxAltitudeReached) {
                 g_maxAltitudeReached = currentAglAlt;
            }
            break;
        case APOGEE:
            if (DROGUE_PRESENT) {
                g_currentFlightState = DROGUE_DEPLOY;
            } else if (MAIN_PRESENT) {
                g_currentFlightState = MAIN_DEPLOY;
            } else {
                g_currentFlightState = DROGUE_DESCENT;
                if (g_debugFlags.enableSystemDebug) Serial.println(F("Warning: Apogee reached but no parachutes configured!"));
            }
            break;
        case DROGUE_DEPLOY:
            if (DROGUE_PRESENT) {
                if (g_debugFlags.enableSystemDebug) Serial.println(F("Firing Pyro Channel 1 (Drogue)"));
                digitalWrite(PYRO_CHANNEL_1, HIGH);
                delay(PYRO_FIRE_DURATION);
                digitalWrite(PYRO_CHANNEL_1, LOW);
                if (g_debugFlags.enableSystemDebug) Serial.println(F("Pyro Channel 1 (Drogue) Fired."));
            }
            g_currentFlightState = DROGUE_DESCENT;
            break;
        case DROGUE_DESCENT:
            if (MAIN_PRESENT) {
                if (g_ms5611Sensor.isConnected() && g_baroCalibrated && currentAglAlt < g_main_deploy_altitude_m_agl) {
                    g_currentFlightState = MAIN_DEPLOY;
                }
            } else {
                if (detectLanding()) {
                    g_currentFlightState = LANDED;
                }
            }
            break;
        case MAIN_DEPLOY:
            if (MAIN_PRESENT) {
                if (g_debugFlags.enableSystemDebug) Serial.println(F("Firing Pyro Channel 2 (Main)"));
                digitalWrite(PYRO_CHANNEL_2, HIGH);
                delay(PYRO_FIRE_DURATION);
                digitalWrite(PYRO_CHANNEL_2, LOW);
                if (g_debugFlags.enableSystemDebug) Serial.println(F("Pyro Channel 2 (Main) Fired."));
            }
            g_currentFlightState = MAIN_DESCENT;
            break;
        case MAIN_DESCENT:
            if (detectLanding()) {
                g_currentFlightState = LANDED;
            }
            break;
        case LANDED:
            if (millis() - g_stateEntryTime > LANDED_TIMEOUT_MS) {
                g_currentFlightState = RECOVERY;
            }
            break;
        case RECOVERY:
            {
                // Declare all variables at the beginning of the case block
                static unsigned long recoveryBuzzerPatternStartTime = 0;
                static int recoveryBuzzerPatternStep = 0;
                static unsigned long recoveryLedStrobeStartTime = 0;
                static bool isLedStrobeOn = false;
                static unsigned long lastGpsBeaconTime = 0;
                
                // SOS Pattern: ... --- ... (S O S)
                // S: Dot Dot Dot
                // O: Dash Dash Dash

                // --- Buzzer Logic ---
                if (BUZZER_OUTPUT) {
                    // Initialize start time if entering state or pattern completed
                    if (newStateSignal || recoveryBuzzerPatternStep == 0) {
                        recoveryBuzzerPatternStartTime = currentTimeMillis;
                        recoveryBuzzerPatternStep = 1;
                        noTone(BUZZER_PIN); // Ensure tone is off initially
                    }

                    unsigned long timeInBuzzerPattern = currentTimeMillis - recoveryBuzzerPatternStartTime;

                    // (Keep existing SOS switch statement here, but use timeInBuzzerPattern and recoveryBuzzerPatternStartTime)
                    // For brevity, assuming the SOS pattern logic from previous step is here,
                    // just changing variable names for clarity if needed.
                    // Example for case 1:
                    // case 1: // Start S - Dot 1
                    //     tone(BUZZER_PIN, RECOVERY_BEACON_FREQUENCY_HZ);
                    //     if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_DOT_MS) {
                    //         noTone(BUZZER_PIN);
                    //         recoveryBuzzerPatternStartTime = currentTimeMillis;
                    //         recoveryBuzzerPatternStep = 2;
                    //     }
                    //     break;
                    // ... rest of SOS cases ...
                    // Ensure variables used are recoveryBuzzerPatternStep, recoveryBuzzerPatternStartTime, timeInBuzzerPattern
                    switch (recoveryBuzzerPatternStep) {
                    // S
                    case 1: // Start S - Dot 1
                        tone(BUZZER_PIN, RECOVERY_BEACON_FREQUENCY_HZ);
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_DOT_MS) {
                            noTone(BUZZER_PIN);
                            recoveryBuzzerPatternStartTime = currentTimeMillis; // Reset timer for pause
                            recoveryBuzzerPatternStep = 2;
                        }
                        break;
                    case 2: // Pause after Dot 1
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_SYMBOL_PAUSE_MS) {
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 3;
                        }
                        break;
                    case 3: // Dot 2
                        tone(BUZZER_PIN, RECOVERY_BEACON_FREQUENCY_HZ);
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_DOT_MS) {
                            noTone(BUZZER_PIN);
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 4;
                        }
                        break;
                    case 4: // Pause after Dot 2
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_SYMBOL_PAUSE_MS) {
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 5;
                        }
                        break;
                    case 5: // Dot 3
                        tone(BUZZER_PIN, RECOVERY_BEACON_FREQUENCY_HZ);
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_DOT_MS) {
                            noTone(BUZZER_PIN);
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 6;
                        }
                        break;
                    case 6: // Pause after S (before O)
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_LETTER_PAUSE_MS) {
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 7;
                        }
                        break;

                    // O
                    case 7: // Start O - Dash 1
                        tone(BUZZER_PIN, RECOVERY_BEACON_FREQUENCY_HZ);
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_DASH_MS) {
                            noTone(BUZZER_PIN);
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 8;
                        }
                        break;
                    case 8: // Pause after Dash 1
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_SYMBOL_PAUSE_MS) {
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 9;
                        }
                        break;
                    case 9: // Dash 2
                        tone(BUZZER_PIN, RECOVERY_BEACON_FREQUENCY_HZ);
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_DASH_MS) {
                            noTone(BUZZER_PIN);
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 10;
                        }
                        break;
                    case 10: // Pause after Dash 2
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_SYMBOL_PAUSE_MS) {
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 11;
                        }
                        break;
                    case 11: // Dash 3
                        tone(BUZZER_PIN, RECOVERY_BEACON_FREQUENCY_HZ);
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_DASH_MS) {
                            noTone(BUZZER_PIN);
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 12;
                        }
                        break;
                    case 12: // Pause after O (before S)
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_LETTER_PAUSE_MS) {
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 13;
                        }
                        break;

                    // S (again)
                    case 13: // Start S - Dot 1
                        tone(BUZZER_PIN, RECOVERY_BEACON_FREQUENCY_HZ);
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_DOT_MS) {
                            noTone(BUZZER_PIN);
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 14;
                        }
                        break;
                    case 14: // Pause after Dot 1
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_SYMBOL_PAUSE_MS) {
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 15;
                        }
                        break;
                    case 15: // Dot 2
                        tone(BUZZER_PIN, RECOVERY_BEACON_FREQUENCY_HZ);
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_DOT_MS) {
                            noTone(BUZZER_PIN);
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 16;
                        }
                        break;
                    case 16: // Pause after Dot 2
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_SYMBOL_PAUSE_MS) {
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 17;
                        }
                        break;
                    case 17: // Dot 3
                        tone(BUZZER_PIN, RECOVERY_BEACON_FREQUENCY_HZ);
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_DOT_MS) {
                            noTone(BUZZER_PIN);
                            recoveryBuzzerPatternStartTime = currentTimeMillis;
                            recoveryBuzzerPatternStep = 18;
                        }
                        break;
                    case 18: // Pause after full SOS (word pause)
                        if (timeInBuzzerPattern >= RECOVERY_BEACON_SOS_WORD_PAUSE_MS) {
                            recoveryBuzzerPatternStep = 0; // Reset to restart pattern
                        }
                        break;
                }
            } // Close the if (BUZZER_OUTPUT) block

                // --- LED Strobe Logic ---
                // Initialize/reset LED strobe timing when first entering RECOVERY state (using newStateSignal)
                // or if recoveryLedStrobeStartTime hasn't been set yet (e.g., if buzzer was off and newStateSignal was missed)
                if (newStateSignal || recoveryLedStrobeStartTime == 0) {
                    recoveryLedStrobeStartTime = currentTimeMillis; // currentTimeMillis is already defined above for buzzer
                    isLedStrobeOn = false;
                    // Ensure LEDs are off at the start of the strobe cycle within RECOVERY
                    g_pixels.setPixelColor(0, g_pixels.Color(0,0,0));
                    if (NEOPIXEL_COUNT > 1) g_pixels.setPixelColor(1, g_pixels.Color(0,0,0));
                    g_pixels.show();
                }

                unsigned long timeInLedCycle = currentTimeMillis - recoveryLedStrobeStartTime;

                if (isLedStrobeOn) { // LED is currently ON
                    if (timeInLedCycle >= RECOVERY_STROBE_ON_MS) {
                        // Time to turn OFF
                        g_pixels.setPixelColor(0, g_pixels.Color(0,0,0));
                        if (NEOPIXEL_COUNT > 1) g_pixels.setPixelColor(1, g_pixels.Color(0,0,0));
                        g_pixels.show();
                        isLedStrobeOn = false;
                        recoveryLedStrobeStartTime = currentTimeMillis; // Reset timer for the OFF period
                    }
                } else { // LED is currently OFF
                    if (timeInLedCycle >= RECOVERY_STROBE_OFF_MS) {
                        // Time to turn ON
                        g_pixels.setPixelColor(0, g_pixels.Color(RECOVERY_STROBE_R, RECOVERY_STROBE_G, RECOVERY_STROBE_B));
                        if (NEOPIXEL_COUNT > 1) g_pixels.setPixelColor(1, g_pixels.Color(RECOVERY_STROBE_R, RECOVERY_STROBE_G, RECOVERY_STROBE_B));
                        // Consider RECOVERY_STROBE_BRIGHTNESS. If it's different from global, it should be set here.
                        // For now, assuming global brightness is acceptable or RECOVERY_STROBE_BRIGHTNESS matches it.
                        // If specific brightness needed: g_pixels.setBrightness(RECOVERY_STROBE_BRIGHTNESS);
                        g_pixels.show();
                        // If brightness was changed: g_pixels.setBrightness(global_brightness_variable); // Restore
                        isLedStrobeOn = true;
                        recoveryLedStrobeStartTime = currentTimeMillis; // Reset timer for the ON period
                    }
                }

                // --- GPS Beacon Serial Output Logic ---
                if (currentTimeMillis - lastGpsBeaconTime >= RECOVERY_GPS_BEACON_INTERVAL_MS) {
                    lastGpsBeaconTime = currentTimeMillis;
                    if (getFixType() > 0) { // Check for a valid GPS fix
                        Serial.print(F("GPS_BEACON: Lat="));
                        Serial.print(GPS_latitude / 10000000.0, 7);
                        Serial.print(F(", Lon="));
                        Serial.print(GPS_longitude / 10000000.0, 7);
                        Serial.print(F(", AltMSL="));
                        Serial.print(GPS_altitudeMSL / 1000.0, 2);
                        Serial.print(F("m, Sats="));
                        Serial.println(SIV);
                    } else {
                        Serial.println(F("GPS_BEACON: No valid GPS fix for beacon."));
                    }
                }
            }
            break;
        case ERROR:
            // ERROR state is stable - no automatic transitions
            // Recovery happens via clear_errors command or handleInitialStateManagement
            break;
        default:
            Serial.print(F("CRITICAL ERROR: Unknown flight state encountered: "));
            Serial.println(static_cast<int>(g_currentFlightState));
            Serial.println(F("Transitioning to ERROR state for safety."));
            g_currentFlightState = ERROR;
            break;
    }
}

void detectBoostEnd() {
    if (g_currentFlightState != BOOST) return;

    if (get_accel_magnitude(g_kx134_initialized_ok, kx134_accel, g_icm20948_ready, icm_accel, g_debugFlags.enableSystemDebug) < COAST_ACCEL_THRESHOLD) {
        boostEndTime = millis();
        g_currentFlightState = COAST;
    }
}

bool detectApogee() {
    bool apogeeDetected = false;

    // Method 1: Barometric Detection (Primary)
    static int baro_descending_count = 0;
    if (g_ms5611Sensor.isConnected() && g_baroCalibrated) {
        float currentBaroAlt = ms5611_get_altitude();
        if (currentBaroAlt < g_maxAltitudeReached) {
            baro_descending_count++;
        } else {
            baro_descending_count = 0;
        }

        if (baro_descending_count >= APOGEE_CONFIRMATION_COUNT) {
            if (g_debugFlags.enableSystemDebug) Serial.println(F("APOGEE DETECTED (Barometer)"));
            apogeeDetected = true;
        }
    }

    // Method 2: Accelerometer Detection (Secondary)
    if (!apogeeDetected && g_icm20948_ready) {
        static int accel_negative_count = 0;
        if (icm_accel[2] < 0.0f) {
            accel_negative_count++;
        } else {
            accel_negative_count = 0;
        }

        if (accel_negative_count >= APOGEE_ACCEL_CONFIRMATION_COUNT) {
            if (g_debugFlags.enableSystemDebug) Serial.println(F("APOGEE DETECTED (Accelerometer)"));
            apogeeDetected = true;
        }
    }

    // Method 3: GPS Altitude Detection (Tertiary)
    if (!apogeeDetected && getFixType() > 0) {
        static int gps_descending_count = 0;
        static float maxGpsAltitude = 0.0f;
        float currentGpsAlt = getGPSAltitude();

        if (currentGpsAlt > maxGpsAltitude) {
            maxGpsAltitude = currentGpsAlt;
            gps_descending_count = 0;
        } else if (currentGpsAlt < maxGpsAltitude - 5.0) {
            gps_descending_count++;
        }

        if (gps_descending_count >= APOGEE_GPS_CONFIRMATION_COUNT) {
            if (g_debugFlags.enableSystemDebug) Serial.println(F("APOGEE DETECTED (GPS)"));
            apogeeDetected = true;
        }
    }

    // Method 4: Backup Timer (Failsafe)
    if (!apogeeDetected && boostEndTime > 0) {
        if (millis() - boostEndTime > BACKUP_APOGEE_TIME_MS) {
            if (g_debugFlags.enableSystemDebug) Serial.println(F("APOGEE DETECTED (Backup Timer)"));
            apogeeDetected = true;
        }
    }

    return apogeeDetected;
}

bool detectLanding() {
    if (g_currentFlightState != DROGUE_DESCENT && g_currentFlightState != MAIN_DESCENT) return false;

    // Use a moving average of altitude to smooth out readings
    const int numReadings = 10;
    static float altReadings[numReadings];
    static int readIndex = 0;
    static float total = 0;
    static bool firstRun = true;
    if (firstRun) {
        for (int i = 0; i < numReadings; i++) altReadings[i] = 0;
        firstRun = false;
    }

    total -= altReadings[readIndex];
    altReadings[readIndex] = ms5611_get_altitude();
    total += altReadings[readIndex];
    readIndex = (readIndex + 1) % numReadings;
    float avgAlt = total / numReadings;

    // Check for landing conditions
    if (fabs(avgAlt - g_launchAltitude) < LANDING_ALTITUDE_STABLE_THRESHOLD) {
        float accelMag = get_accel_magnitude(g_kx134_initialized_ok, kx134_accel, g_icm20948_ready, icm_accel, g_debugFlags.enableSystemDebug);
        if (accelMag >= LANDING_ACCEL_MIN_G && accelMag <= LANDING_ACCEL_MAX_G) {
            static unsigned long firstLandedTime = 0;
            if (firstLandedTime == 0) firstLandedTime = millis();
            if (millis() - firstLandedTime >= LANDING_CONFIRMATION_TIME_MS) {
                return true;
            }
        }
    } else {
        // Reset landing timer if altitude condition is not met
        // firstLandedTime = 0;
    }

    return false;
}

// Placeholder/test implementation for guidance target updates - REMOVED as unused
// void update_guidance_targets() {
//     static bool initial_target_set = false;
//     static float initial_yaw_target = 0.0f; // Store the initial yaw target in radians
//
//     if (!initial_target_set) {
//         float current_roll, current_pitch, current_yaw;
//         convertQuaternionToEuler(icm_q0, icm_q1, icm_q2, icm_q3, current_roll, current_pitch, current_yaw);
//         initial_yaw_target = current_yaw;
//         guidance_set_target_orientation_euler(0.0f, 0.0f, initial_yaw_target);
//         initial_target_set = true;
//     }
//     // Note: The time-based target changing logic has been removed for simplification.
//     // The target is now set once at initialization and held.
// }