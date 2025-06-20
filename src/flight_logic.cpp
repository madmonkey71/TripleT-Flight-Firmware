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

// Externs for variables globally defined in TripleT_Flight_Firmware.cpp
extern FlightState g_currentFlightState;
extern FlightState g_previousFlightState;
extern unsigned long g_stateEntryTime;
extern Adafruit_NeoPixel g_pixels; // Assuming g_pixels is the new name in main
extern float g_launchAltitude;
extern float g_maxAltitudeReached;
extern bool g_baroCalibrated;
extern MS5611 g_ms5611Sensor; // Assuming g_ms5611Sensor is the new name in main
// kx134_accel and icm_accel are often externed in their respective function .h files
// If kx134_functions.h and icm_20948_functions.h provide these, local externs are not needed.
// Assuming they are provided by those headers for now.
extern float kx134_accel[3]; // Retaining if not provided by kx134_functions.h as extern
extern float icm_accel[3];   // Retaining if not provided by icm_20948_functions.h as extern
extern bool g_icm20948_ready;
extern bool g_useKalmanFilter;
// extern bool enableSystemDebug; // Replaced by g_debugFlags
extern DebugFlags g_debugFlags; // To access g_debugFlags.enableSystemDebug
extern float g_main_deploy_altitude_m_agl; // Already correctly prefixed

// Externs from TripleT_Flight_Firmware.cpp for attitude hold logic
extern float g_kalmanRoll;
extern float g_kalmanPitch;
extern float g_kalmanYaw;

// Global variables for flight logic state progression, defined in this file
unsigned long boostEndTime = 0;

// Constants for Recovery Beep Pattern
static const unsigned long RECOVERY_BEEP_DURATION_MS = 200;
static const unsigned long RECOVERY_SILENCE_DURATION_MS = 1800; // Total cycle time = 2000ms (2 seconds)
static const unsigned int RECOVERY_BEEP_FREQUENCY_HZ = 2000; // 2kHz tone

// Global variables for flight logic state progression, defined in this file (flight_logic.cpp)
bool landingDetectedFlag = false;
float previousApogeeDetectAltitude = 0.0f;
float lastLandingCheckAltitudeAgl = 0.0f;
int descendingCount = 0;

// Helper function to set LED color based on flight state
void setFlightStateLED(FlightState state) {
    switch (state) {
        case STARTUP: g_pixels.setPixelColor(0, g_pixels.Color(128, 0, 0)); break; // Dark Red
        case CALIBRATION: g_pixels.setPixelColor(0, g_pixels.Color(255, 165, 0)); break; // Orange
        case PAD_IDLE: g_pixels.setPixelColor(0, g_pixels.Color(0, 255, 0)); break; // Green
        case ARMED: g_pixels.setPixelColor(0, g_pixels.Color(255, 255, 0)); break; // Yellow
        case BOOST: g_pixels.setPixelColor(0, g_pixels.Color(255, 0, 255)); break; // Magenta
        case COAST: g_pixels.setPixelColor(0, g_pixels.Color(0, 255, 255)); break; // Cyan
        case APOGEE: g_pixels.setPixelColor(0, g_pixels.Color(255, 255, 255)); break; // White
        case DROGUE_DEPLOY: g_pixels.setPixelColor(0, g_pixels.Color(255, 0, 0)); g_pixels.setPixelColor(1, g_pixels.Color(255, 0, 0)); break; // Pulsing Red (simulated by setting both)
        case DROGUE_DESCENT: g_pixels.setPixelColor(0, g_pixels.Color(139, 0, 0)); break; // DarkRed
        case MAIN_DEPLOY: g_pixels.setPixelColor(0, g_pixels.Color(0, 0, 255)); g_pixels.setPixelColor(1, g_pixels.Color(0, 0, 255)); break; // Pulsing Blue
        case MAIN_DESCENT: g_pixels.setPixelColor(0, g_pixels.Color(0, 0, 139)); break; // DarkBlue
        case LANDED: g_pixels.setPixelColor(0, g_pixels.Color(75, 0, 130)); break; // Indigo
        case RECOVERY: g_pixels.setPixelColor(0, g_pixels.Color(0, 128, 0)); break; // Dark Green (for GPS beacon active, etc.)
        case ERROR: g_pixels.setPixelColor(0, g_pixels.Color(255, 0, 0)); break; // Solid Red for error
        default: g_pixels.setPixelColor(0, g_pixels.Color(50, 50, 50)); break; // Grey for unknown
    }
    g_pixels.show();
}

void ProcessFlightState() {
    float currentAbsoluteBaroAlt = 0.0f;
    float currentAglAlt = 0.0f;
    bool newStateSignal = false;
    static unsigned long lastErrorCheckTime = 0;
    const unsigned long errorCheckInterval = 1000; // 1 second

    // Defer the health check to avoid immediate re-entry into ERROR state
    // after a manual `clear_errors` command.
    if (g_currentFlightState != LANDED && g_currentFlightState != RECOVERY && g_currentFlightState != ERROR) {
        if (millis() - lastErrorCheckTime > errorCheckInterval) {
            lastErrorCheckTime = millis();
            if (!isSensorSuiteHealthy(g_currentFlightState)) {
                // Log detailed sensor status before transitioning to ERROR
                if (g_debugFlags.enableSystemDebug) {
                    Serial.println(F("--- Sensor Suite Health Report (Pre-ERROR) ---"));
                    isSensorSuiteHealthy(g_currentFlightState, true); // Call with verbose=true
                    Serial.println(F("---------------------------------------------"));
                }
                g_currentFlightState = ERROR;
                if (g_debugFlags.enableSystemDebug) {
                    Serial.println(F("Sensor suite unhealthy, transitioning to ERROR state."));
                }
                // When we transition to error, we should immediately save and return
                // to avoid any other logic processing in this loop cycle.
                saveStateToEEPROM(); // Uses g_currentFlightState
                setFlightStateLED(g_currentFlightState);
                return;
            } else {
                // Add periodic health status when things are OK
                static unsigned long lastHealthOkTime = 0;
                if (millis() - lastHealthOkTime > 5000) { // Every 5 seconds
                    lastHealthOkTime = millis();
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.print(F("Health check OK for state: "));
                        Serial.println(getStateName(g_currentFlightState));
                    }
                }
            }
        }
    } else if (g_currentFlightState == ERROR && g_debugFlags.enableSystemDebug) {
        // Add periodic debugging for ERROR state
        static unsigned long lastErrorDebugTime = 0;
        if (millis() - lastErrorDebugTime > 2000) { // Every 2 seconds
            lastErrorDebugTime = millis();
            Serial.println(F("--- Currently in ERROR state ---"));
            Serial.println(F("Use 'clear_errors' command to manually clear if all systems are working."));
            Serial.println(F("Or check sensor health with detailed report:"));
            isSensorSuiteHealthy(PAD_IDLE, true); // Show what PAD_IDLE health check would find
            Serial.println(F("--------------------------------"));
        }
    }

    if (g_ms5611Sensor.isConnected() && g_baroCalibrated) {
        currentAbsoluteBaroAlt = ms5611_get_altitude(); // Uses global ms5611Sensor
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
        WriteLogData(true); // Uses g_currentFlightState and other globals
        saveStateToEEPROM(); // Uses g_currentFlightState
        setFlightStateLED(g_currentFlightState);
    }

    if (newStateSignal) {
        switch (g_currentFlightState) {
            case PAD_IDLE:
                g_launchAltitude = g_ms5611Sensor.isConnected() && g_baroCalibrated ? ms5611_get_altitude() : 0.0f;
                g_maxAltitudeReached = 0.0f;
                boostEndTime = 0;
                landingDetectedFlag = false;
                descendingCount = 0; // Reset for apogee detection
                previousApogeeDetectAltitude = g_launchAltitude; // Initialize for apogee detection
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
                descendingCount = 0; // Reset for apogee detection
                previousApogeeDetectAltitude = currentAbsoluteBaroAlt; // Initialize for apogee baro detection
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
                if (!MAIN_PRESENT) { // If no main, prepare for landing detection under drogue
                    lastLandingCheckAltitudeAgl = currentAglAlt;
                }
                break;
            case MAIN_DEPLOY:
                if (g_debugFlags.enableSystemDebug) Serial.println(F("MAIN_DEPLOY: Initiating main parachute deployment."));
                break;
            case MAIN_DESCENT:
                if (g_debugFlags.enableSystemDebug) Serial.println(F("MAIN_DESCENT: Descending under main parachute."));
                lastLandingCheckAltitudeAgl = currentAglAlt; // Initialize for landing detection
                break;
            case LANDED:
                if (g_debugFlags.enableSystemDebug) Serial.println(F("LANDED: Touchdown confirmed."));
                break;
            case RECOVERY:
                if (g_debugFlags.enableSystemDebug) Serial.println(F("RECOVERY: System in post-flight recovery mode."));
                break;
            case ERROR:
                if (g_debugFlags.enableSystemDebug) Serial.println(F("ERROR: System has entered error state."));
                break;
            default:
                break;
        }
    }

    switch (g_currentFlightState) {
        case ARMED:
            if (get_accel_magnitude(g_kx134_initialized_ok, kx134_accel, g_icm20948_ready, icm_accel, g_debugFlags.enableSystemDebug) > BOOST_ACCEL_THRESHOLD) {
                g_currentFlightState = BOOST;
            }
            break;
        case BOOST:
            detectBoostEnd(); // Uses get_accel_magnitude internally
            if (boostEndTime > 0) {
                // Attitude Hold: Capture current orientation and set as target for COAST phase
                float targetRollRad = 0.0f, targetPitchRad = 0.0f, targetYawRad = 0.0f;
                if (g_useKalmanFilter && g_icm20948_ready) {
                    targetRollRad = g_kalmanRoll;
                    targetPitchRad = g_kalmanPitch;
                    targetYawRad = g_kalmanYaw;
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.println(F("ATT_HOLD: Using Kalman orientation for target at BOOST->COAST."));
                    }
                } else if (g_icm20948_ready) { // Default to Madgwick/ICM if Kalman not active but ICM is ready
                    convertQuaternionToEuler(icm_q0, icm_q1, icm_q2, icm_q3, targetRollRad, targetPitchRad, targetYawRad); // icm_q variables are extern from icm_20948_functions.h
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.println(F("ATT_HOLD: Using Madgwick/ICM quaternion conversion for target at BOOST->COAST."));
                    }
                } else {
                    // Fallback: No reliable orientation data, set targets to zero.
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
            if (detectApogee()) { // Uses get_accel_magnitude internally
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
                if (detectLanding()) { // Uses get_accel_magnitude internally
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
            if (detectLanding()) { // Uses get_accel_magnitude internally
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
                static unsigned long lastRecoveryActionTime = 0;
                static bool isBeeping = false;
                unsigned long currentTimeMillis = millis();

                if (isBeeping) {
                    if (currentTimeMillis - lastRecoveryActionTime >= RECOVERY_BEEP_DURATION_MS) {
                        noTone(BUZZER_PIN);
                        isBeeping = false;
                        lastRecoveryActionTime = currentTimeMillis;
                    }
                } else {
                    if (currentTimeMillis - lastRecoveryActionTime >= RECOVERY_SILENCE_DURATION_MS) {
                        tone(BUZZER_PIN, RECOVERY_BEEP_FREQUENCY_HZ);
                        isBeeping = true;
                        lastRecoveryActionTime = currentTimeMillis;
                    }
                }
            }
            break;
        case ERROR:
            if (millis() - g_stateEntryTime > ERROR_RECOVERY_ATTEMPT_MS) {
                if (isSensorSuiteHealthy(g_currentFlightState)) {
                    g_currentFlightState = PAD_IDLE;
                } else {
                    g_stateEntryTime = millis();
                }
            }
            break;
        default:
            g_currentFlightState = ERROR;
            break;
    }
}

void detectBoostEnd() {
    if (get_accel_magnitude(g_kx134_initialized_ok, kx134_accel, g_icm20948_ready, icm_accel, g_debugFlags.enableSystemDebug) < COAST_ACCEL_THRESHOLD) {
        boostEndTime = millis();
        if (g_debugFlags.enableSystemDebug) Serial.println(F("BOOST_END detected by accelerometer."));
    }
}

bool detectApogee() {
    bool apogeeDetected = false;

    // Method 1: Barometric Detection (Primary)
    static int baro_descending_count = 0;
    if (g_ms5611Sensor.isConnected() && g_baroCalibrated) {
        float currentBaroAlt = ms5611_get_altitude(); // Uses global g_ms5611Sensor
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
        if (icm_accel[2] < 0.0f) { // Assuming Z-axis is vertical; icm_accel from icm_20948_functions.h
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
    if (!apogeeDetected && getFixType() > 0) { // getFixType from gps_functions.h
        static int gps_descending_count = 0;
        static float maxGpsAltitude = 0.0f;
        float currentGpsAlt = getGPSAltitude(); // from gps_functions.h

        if (currentGpsAlt > maxGpsAltitude) {
            maxGpsAltitude = currentGpsAlt;
            gps_descending_count = 0;
        } else if (currentGpsAlt < maxGpsAltitude - 5.0) { // Hysteresis for noise
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
    static unsigned long firstStableTime = 0;

    if (g_ms5611Sensor.isConnected() && g_baroCalibrated) {
        float currentAgl = ms5611_get_altitude() - g_launchAltitude; // Uses global g_ms5611Sensor
        float accelMag = get_accel_magnitude(g_kx134_initialized_ok, kx134_accel, g_icm20948_ready, icm_accel, g_debugFlags.enableSystemDebug);

        // Check if we're close to ground with low, stable acceleration
        if (currentAgl < 50.0f && accelMag > LANDING_ACCEL_MIN_G && accelMag < LANDING_ACCEL_MAX_G) {
            if (firstStableTime == 0) {
                firstStableTime = millis();
            }
        } else {
            firstStableTime = 0;
        }
        
        // Confirm landing if stable conditions persist
        if (firstStableTime > 0 && millis() - firstStableTime > LANDING_CONFIRMATION_TIME_MS) {
            if (g_debugFlags.enableSystemDebug) {
                Serial.println(F("Landing detected"));
            }
            return true;
        }
    }
    return false;
}

// Placeholder/test implementation for guidance target updates
void update_guidance_targets() {
    static bool initial_target_set = false;
    static float initial_yaw_target = 0.0f; // Store the initial yaw target in radians

    if (!initial_target_set) {
        float current_roll, current_pitch, current_yaw;
        convertQuaternionToEuler(icm_q0, icm_q1, icm_q2, icm_q3, current_roll, current_pitch, current_yaw); // icm_q vars are extern from icm_20948_functions.h
        initial_yaw_target = current_yaw;
        guidance_set_target_orientation_euler(0.0f, 0.0f, initial_yaw_target);
        initial_target_set = true;
    }
    // Note: The time-based target changing logic has been removed for simplification.
    // The target is now set once at initialization and held.
}