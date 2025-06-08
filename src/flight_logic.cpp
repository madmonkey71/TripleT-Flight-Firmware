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
extern bool icm20948_ready;
extern bool useKalmanFilter;
extern bool enableSystemDebug;
extern float g_main_deploy_altitude_m_agl;

// Externs from TripleT_Flight_Firmware.cpp for attitude hold logic
extern float kalmanRoll;
extern float kalmanPitch;
extern float kalmanYaw;

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
    bool newStateSignal = false;

    if (ms5611Sensor.isConnected() && baroCalibrated) {
        currentAbsoluteBaroAlt = ms5611_get_altitude();
        currentAglAlt = currentAbsoluteBaroAlt - launchAltitude;
    }

    saveStateToEEPROM();

    if (currentFlightState != LANDED && currentFlightState != RECOVERY && currentFlightState != ERROR) {
        if (!isSensorSuiteHealthy(currentFlightState)) {
            currentFlightState = ERROR;
            if (enableSystemDebug) {
                Serial.println(F("Sensor suite unhealthy, transitioning to ERROR state."));
            }
            return;
        }
    }

    if (currentFlightState != previousFlightState) {
        newStateSignal = true;
        previousFlightState = currentFlightState;
        stateEntryTime = millis();

        if (enableSystemDebug) {
            Serial.print(F("Transitioning to state: "));
            Serial.println(getStateName(currentFlightState));
        }
        WriteLogData(true);
        saveStateToEEPROM();
        setFlightStateLED(currentFlightState);
    }

    if (newStateSignal) {
        switch (currentFlightState) {
            case PAD_IDLE:
                launchAltitude = ms5611Sensor.isConnected() && baroCalibrated ? ms5611_get_altitude() : 0.0f;
                maxAltitudeReached = 0.0f;
                boostEndTime = 0;
                landingDetectedFlag = false;
                descendingCount = 0; // Reset for apogee detection
                previousApogeeDetectAltitude = launchAltitude; // Initialize for apogee detection
                if (enableSystemDebug) Serial.println(F("PAD_IDLE: System initialized. Launch altitude set."));
                pinMode(PYRO_CHANNEL_1, OUTPUT); digitalWrite(PYRO_CHANNEL_1, LOW);
                pinMode(PYRO_CHANNEL_2, OUTPUT); digitalWrite(PYRO_CHANNEL_2, LOW);
                break;
            case ARMED:
                if (enableSystemDebug) Serial.println(F("ARMED: System armed and ready for launch."));
                if (baroCalibrated && ms5611Sensor.isConnected()) {
                    g_main_deploy_altitude_m_agl = currentAglAlt + MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M;
                    if (enableSystemDebug) {
                        Serial.print(F("ARMED: Dynamic main deployment altitude set to: "));
                        Serial.print(g_main_deploy_altitude_m_agl, 2);
                        Serial.println(F(" m AGL"));
                    }
                } else {
                    g_main_deploy_altitude_m_agl = MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M;
                    if (enableSystemDebug) {
                        Serial.print(F("ARMED_WARN: Baro not ready. Main deploy altitude defaulting to fixed height: "));
                        Serial.print(g_main_deploy_altitude_m_agl, 2);
                        Serial.println(F(" m above current launch altitude."));
                    }
                }
                break;
            case BOOST:
                if (useKalmanFilter && !icm20948_ready) {
                    currentFlightState = ERROR;
                    if (enableSystemDebug) Serial.println(F("ERROR: Cannot detect liftoff; ICM20948 not ready."));
                }
                if (enableSystemDebug) Serial.println(F("BOOST: Liftoff detected!"));
                maxAltitudeReached = currentAglAlt > 0 ? currentAglAlt : 0;
                boostEndTime = 0;
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
                break;
            case RECOVERY:
                if (enableSystemDebug) Serial.println(F("RECOVERY: System in post-flight recovery mode."));
                break;
            case ERROR:
                if (enableSystemDebug) Serial.println(F("ERROR: System has entered error state."));
                break;
            default:
                break;
        }
    }

    switch (currentFlightState) {
        case ARMED:
            if (get_accel_magnitude() > BOOST_ACCEL_THRESHOLD) {
                currentFlightState = BOOST;
            }
            break;
        case BOOST:
            detectBoostEnd();
            if (boostEndTime > 0) {
                currentFlightState = COAST;
            }
            if (ms5611Sensor.isConnected() && baroCalibrated && currentAglAlt > maxAltitudeReached) {
                 maxAltitudeReached = currentAglAlt;
            }
            break;
        case COAST:
            if (detectApogee()) {
                currentFlightState = APOGEE;
            }
            if (ms5611Sensor.isConnected() && baroCalibrated && currentAglAlt > maxAltitudeReached) {
                 maxAltitudeReached = currentAglAlt;
            }
            break;
        case APOGEE:
            if (DROGUE_PRESENT) {
                currentFlightState = DROGUE_DEPLOY;
            } else if (MAIN_PRESENT) {
                currentFlightState = MAIN_DEPLOY;
            } else {
                currentFlightState = DROGUE_DESCENT;
                if (enableSystemDebug) Serial.println(F("Warning: Apogee reached but no parachutes configured!"));
            }
            break;
        case DROGUE_DEPLOY:
            if (DROGUE_PRESENT) {
                if (enableSystemDebug) Serial.println(F("Firing Pyro Channel 1 (Drogue)"));
                digitalWrite(PYRO_CHANNEL_1, HIGH);
                delay(PYRO_FIRE_DURATION);
                digitalWrite(PYRO_CHANNEL_1, LOW);
                if (enableSystemDebug) Serial.println(F("Pyro Channel 1 (Drogue) Fired."));
            }
            currentFlightState = DROGUE_DESCENT;
            break;
        case DROGUE_DESCENT:
            if (MAIN_PRESENT) {
                if (ms5611Sensor.isConnected() && baroCalibrated && currentAglAlt < g_main_deploy_altitude_m_agl) {
                    currentFlightState = MAIN_DEPLOY;
                }
            } else {
                if (detectLanding()) {
                    currentFlightState = LANDED;
                }
            }
            break;
        case MAIN_DEPLOY:
            if (MAIN_PRESENT) {
                if (enableSystemDebug) Serial.println(F("Firing Pyro Channel 2 (Main)"));
                digitalWrite(PYRO_CHANNEL_2, HIGH);
                delay(PYRO_FIRE_DURATION);
                digitalWrite(PYRO_CHANNEL_2, LOW);
                if (enableSystemDebug) Serial.println(F("Pyro Channel 2 (Main) Fired."));
            }
            currentFlightState = MAIN_DESCENT;
            break;
        case MAIN_DESCENT:
            if (detectLanding()) {
                currentFlightState = LANDED;
            }
            break;
        case LANDED:
            if (millis() - stateEntryTime > LANDED_TIMEOUT_MS) {
                currentFlightState = RECOVERY;
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
            if (millis() - stateEntryTime > ERROR_RECOVERY_ATTEMPT_MS) {
                if (isSensorSuiteHealthy(currentFlightState)) {
                    currentFlightState = PAD_IDLE;
                } else {
                    stateEntryTime = millis();
                }
            }
            break;
        default:
            currentFlightState = ERROR;
            break;
    }
}

void detectBoostEnd() {
    if (get_accel_magnitude() < COAST_ACCEL_THRESHOLD) {
        boostEndTime = millis();
        if (enableSystemDebug) Serial.println(F("BOOST_END detected by accelerometer."));
    }
}

bool detectApogee() {
    bool apogeeDetected = false;

    // Method 1: Barometric Detection (Primary)
    static int baro_descending_count = 0;
    if (ms5611Sensor.isConnected() && baroCalibrated) {
        float currentBaroAlt = ms5611_get_altitude();
        if (currentBaroAlt < maxAltitudeReached) {
            baro_descending_count++;
        } else {
            baro_descending_count = 0;
        }

        if (baro_descending_count >= APOGEE_CONFIRMATION_COUNT) {
            if (enableSystemDebug) Serial.println(F("APOGEE DETECTED (Barometer)"));
            apogeeDetected = true;
        }
    }

    // Method 2: Accelerometer Detection (Secondary)
    if (!apogeeDetected && icm20948_ready) {
        static int accel_negative_count = 0;
        if (icm_accel[2] < 0.0f) { // Assuming Z-axis is vertical
            accel_negative_count++;
        } else {
            accel_negative_count = 0;
        }

        if (accel_negative_count >= APOGEE_ACCEL_CONFIRMATION_COUNT) {
            if (enableSystemDebug) Serial.println(F("APOGEE DETECTED (Accelerometer)"));
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
        } else if (currentGpsAlt < maxGpsAltitude - 5.0) { // Hysteresis for noise
            gps_descending_count++;
        }

        if (gps_descending_count >= APOGEE_GPS_CONFIRMATION_COUNT) {
            if (enableSystemDebug) Serial.println(F("APOGEE DETECTED (GPS)"));
            apogeeDetected = true;
        }
    }

    // Method 4: Backup Timer (Failsafe)
    if (!apogeeDetected && boostEndTime > 0) {
        if (millis() - boostEndTime > BACKUP_APOGEE_TIME_MS) {
            if (enableSystemDebug) Serial.println(F("APOGEE DETECTED (Backup Timer)"));
            apogeeDetected = true;
        }
    }

    return apogeeDetected;
}

bool detectLanding() {
    static unsigned long firstStableTime = 0;

    if (ms5611Sensor.isConnected() && baroCalibrated) {
        float currentAgl = ms5611_get_altitude() - launchAltitude;
        float accelMag = get_accel_magnitude();

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
            if (enableSystemDebug) {
                Serial.println(F("Landing detected"));
            }
            return true;
        }
    }
    return false;
}