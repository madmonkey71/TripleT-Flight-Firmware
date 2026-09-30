// In src/flight_logic.cpp
#include "flight_logic.h"
#include <Arduino.h> // For millis(), fabs(), Serial, digitalWrite, pinMode, delay
#include "config.h"   // For configuration constants
#include "ms5611_functions.h" // For ms5611_get_altitude()
#include "utility_functions.h" // For get_accel_magnitude(), getStateName(), isSensorSuiteHealthy()
#include "state_management.h" // For saveStateToEEPROM(), pyro-fired mask, flight-in-progress flag
#include "constants.h"     // For timing constants like BACKUP_APOGEE_TIME
#if ENABLE_GUIDANCE == 1
#include "guidance_control.h" // For guidance functions and g_guidance_active
#endif
#include "icm_20948_functions.h" // For convertQuaternionToEuler and icm_q0 etc.
#include "gps_functions.h" // For getGPSAltitude() and getFixType()
#include <Adafruit_NeoPixel.h>
#include <MS5611.h>
#include "debug_flags.h" // For g_debugFlags
#include "sensor_samples.h" // Fresh-sample sequence numbers / timestamps
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
extern float g_currentAltitude;
extern bool g_baroCalibrated;
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
#if ENABLE_GUIDANCE == 1
extern float g_kalmanRollRate;
extern float g_kalmanPitchRate;
extern float g_kalmanYawRate;
#endif

// Global variables for flight logic state progression, defined in this file
unsigned long boostEndTime = 0;

// Global variables for flight logic state progression, defined in this file (flight_logic.cpp)
bool landingDetectedFlag = false;
float previousApogeeDetectAltitude = 0.0f;
float lastLandingCheckAltitudeAgl = 0.0f;
int descendingCount = 0;
static unsigned long lastStateBroadcastTime = 0; // For broadcasting state when CSV is off

// Stability metrics tracking for logging
static float max_pitch_rate_current_state_rps = 0.0f;
static float max_roll_rate_current_state_rps = 0.0f;
static float max_yaw_rate_current_state_rps = 0.0f;
static float max_pitch_att_err_current_state_rad = 0.0f;
static float max_roll_att_err_current_state_rad = 0.0f;
static float max_yaw_att_err_current_state_rad = 0.0f;
static uint8_t current_stability_flags = 0; // Bitfield: 1=Rate, 2=Att, 4=Sat

// ---------------------------------------------------------------------------
// Resettable runtime state
// ---------------------------------------------------------------------------
// All flight-logic bookkeeping that used to live in function-local `static`
// variables is gathered here so it can be reset explicitly between flights
// (PAD_IDLE entry, post-recovery resume) and between unit tests. Function-local
// statics could not be reset, so state from one flight leaked into the next.
struct FlightRuntime {
    // Health monitor / error auto-recovery timers
    unsigned long lastErrorCheckTime = 0;
    unsigned long lastErrorClearTime = 0;      // Track when errors were last cleared
    unsigned long lastHealthOkTime = 0;
    unsigned long lastGraceMsg = 0;
    unsigned long lastAutoRecoveryCheckTime = 0;
    unsigned long lastErrorDebugTime = 0;
    FlightState   lastRecordedState = STARTUP;
    // CALIBRATION auto-calibration bookkeeping
    unsigned long lastCalibWaitMsgTime = 0;
    bool          autoCalibAttempted = false;
    // COAST entry bookkeeping
    FlightState   lastCoastState = STARTUP;
    bool          coastCountersReset = false;
    // Pyro fire-window flags (see DROGUE_DEPLOY / MAIN_DEPLOY)
    bool          drogueHasFired = false;
    bool          mainHasFired = false;
    // g_launchAltitude is only a trustworthy ground reference once PAD_IDLE was entered (or a flight resumed)
    bool          launchAltValid = false;
    // In-flight sensor degradation (audit #2)
    bool          degraded = false;
    unsigned long lastDegradeLogMs = 0;
    // Main deploy debounce (audit #5)
    FreshCounter  mainGate;
    bool          mainFallbackLogged = false;
    // Detector state (all confirmation counters advance on FRESH samples only)
    FreshCounter  coastConfirm;
    FreshCounter  apogeeBaro;
    float         apogeeBaroRef = 0.0f;        // peak AGL since the transonic lockout ended
    bool          apogeeBaroRefValid = false;
    FreshCounter  apogeeAccel;
    unsigned long freefallStartMs = 0;         // 0 = not currently in free fall
    FreshCounter  apogeeGps;
    float         maxGpsAlt = 0.0f;
    bool          gpsAltValid = false;
    BaroTrack     baroTrack;                   // recent fresh baro samples (vertical rate etc.)
    // Landing detector (moving average of altitude + confirmation timer)
    static const int kLandingReadings = 10;
    float         landingAlt[kLandingReadings] = {0};
    int           landingIndex = 0;
    float         landingTotal = 0.0f;
    bool          landingFirstRun = true;
    unsigned long landingFirstLandedTime = 0;
};
static FlightRuntime g_rt;


// Helper function to convert radians to degrees for logging max values
static inline float rad_to_deg_local(float rad) {
    return rad * (180.0f / M_PI);
}

// Helper function to reset stability metrics for logging at state changes
static void reset_max_stability_metrics() {
    max_pitch_rate_current_state_rps = 0.0f;
    max_roll_rate_current_state_rps = 0.0f;
    max_yaw_rate_current_state_rps = 0.0f;
    max_pitch_att_err_current_state_rad = 0.0f;
    max_roll_att_err_current_state_rad = 0.0f;
    max_yaw_att_err_current_state_rad = 0.0f;
    current_stability_flags = 0; // Reset flags as well
}


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

// ---------------------------------------------------------------------------
// ERROR-state policy (audit #2)
// ---------------------------------------------------------------------------
// The ERROR state stops the state machine's deployment logic (apogee, backup timer,
// main deploy). That is acceptable on the pad, where nothing must deploy, and
// unacceptable in the air. So:
//   * ERROR may only be entered from pre-flight states (and never once a flight is
//     in progress);
//   * a sensor-health failure in flight DEGRADES the vehicle instead: log it, show
//     the degraded LED, disable guidance - and keep running every deployment path.
bool flight_is_airborne_state(FlightState s) { return s >= BOOST && s <= MAIN_DESCENT; }

bool flight_error_allowed(FlightState s) {
    if (g_flightInProgress) return false;
    return s == STARTUP || s == CALIBRATION || s == PAD_IDLE || s == ARMED;
}

bool flightIsDegraded() { return g_rt.degraded; }

// True if the barometer has produced a fresh sample recently.
static bool baroDataFresh() {
    return g_baroSample.seq > 0 && (millis() - g_baroSample.lastMs) <= BARO_STALE_TIMEOUT_MS;
}

void flightSetLaunchAltitude(float alt_m) {
    g_launchAltitude = alt_m;
    g_rt.launchAltValid = true;
}

// audit #3: leaving ERROR (auto-recovery, clear_errors, clear_to_calibration,
// skip_calibration) sends the vehicle to PAD_IDLE/CALIBRATION, which resets the launch
// altitude and max altitude and re-enables arming. That is only acceptable if the
// vehicle is PROVABLY on the ground and NEVER flew:
//   * the persisted flight-in-progress flag is clear (set at BOOST, survives resets),
//   * the state is not an airborne one, and
//   * if the barometer is calibrated, has a ground reference and is delivering fresh
//     samples, the vehicle is within GROUND_AGL_TOLERANCE_M of the launch altitude
//     (this catches an un-armed launch while parked in ERROR).
bool flight_is_provably_on_ground() {
    if (g_flightInProgress) return false;
    if (flight_is_airborne_state(g_currentFlightState)) return false;
    if (g_baroCalibrated && g_rt.launchAltValid && baroDataFresh()) {
        const float agl = ms5611_get_altitude() - g_launchAltitude;
        if (fabsf(agl) > GROUND_AGL_TOLERANCE_M) return false;
    }
    return true;
}

static void flightDegrade(ErrorCode_t code, const char* reason) {
    const bool firstTime = !g_rt.degraded;
    g_rt.degraded = true;
    g_last_error_code = code; // visible in the log's last_error_code column
    if (firstTime || millis() - g_rt.lastDegradeLogMs > 10000) {
        g_rt.lastDegradeLogMs = millis();
        Serial.println(F("=== FLIGHT DEGRADED (state machine keeps running) ==="));
        Serial.print(F("State: "));
        Serial.println(getStateName(g_currentFlightState));
        Serial.print(F("Reason: "));
        Serial.println(reason);
        Serial.println(F("Action: guidance disabled; apogee / backup timer / main deploy unaffected"));
        Serial.println(F("====================================================="));
    }
    #if ENABLE_GUIDANCE == 1
    if (g_guidance_active) {
        g_guidance_active = false;   // never re-enabled mid-flight
        guidance_center_servos();    // fins to neutral
    }
    #endif
    g_pixels.setPixelColor(0, g_pixels.Color(255, 165, 0)); // orange = degraded
    g_pixels.show();
    if (firstTime) WriteLogData(true);
}

// A state value outside the enum: the vehicle cannot know where it is. Before flight
// that is an ERROR; once a flight has begun, ERROR would stop deployment logic, so
// fall to the pyro-inert RECOVERY state instead.
static void flightHandleUnknownState() {
    Serial.print(F("CRITICAL ERROR: Unknown flight state encountered: "));
    Serial.println(static_cast<int>(g_currentFlightState));
    if (g_flightInProgress) {
        Serial.println(F("Flight in progress: transitioning to RECOVERY (never ERROR in flight)."));
        g_currentFlightState = RECOVERY;
    } else {
        Serial.println(F("Transitioning to ERROR state for safety."));
        g_currentFlightState = ERROR;
    }
}

void ProcessFlightState() {
    // audit #1: an in-flight saved state is still awaiting barometer evidence. Do
    // nothing (in particular: no health-check ERROR, no pyro) until it is settled.
    if (recoveryPending()) return;

    float currentAbsoluteBaroAlt = 0.0f;
    float currentAglAlt = 0.0f;
    bool newStateSignal = false;
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
        bool withinGracePeriod = (millis() - g_rt.lastErrorClearTime < errorClearGracePeriod);
        
        if (millis() - g_rt.lastErrorCheckTime > errorCheckInterval && !withinGracePeriod) {
            g_rt.lastErrorCheckTime = millis();
            const bool suiteHealthy = isSensorSuiteHealthy(g_currentFlightState); // uses g_baroCalibrated, g_icm20948_ready, g_kx134_initialized_ok, myGNSS
            if (!suiteHealthy && !flight_error_allowed(g_currentFlightState)) {
                // audit #2: airborne - degrade, never leave the flight state machine.
                if (!g_rt.degraded) {
                    Serial.println(F("--- CRITICAL: Sensor Suite Health Check Failed IN FLIGHT ---"));
                    isSensorSuiteHealthy(g_currentFlightState, true); // detailed report, once per episode
                }
                flightDegrade(STATE_TRANSITION_INVALID_HEALTH, "periodic sensor health check failed");
            } else if (!suiteHealthy) {
                // ALWAYS log detailed sensor status before transitioning to ERROR (regardless of debug flags)
                Serial.println(F("--- CRITICAL: Sensor Suite Health Check Failed ---"));
                Serial.print(F("Current State: "));
                Serial.println(getStateName(g_currentFlightState));
                Serial.print(F("Time since last error clear: "));
                Serial.print((millis() - g_rt.lastErrorClearTime) / 1000.0, 1);
                Serial.println(F(" seconds"));
                Serial.print(F("Grace period remaining: "));
                Serial.print((errorClearGracePeriod - (millis() - g_rt.lastErrorClearTime)) / 1000.0, 1);
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
                // Populate stability flags if this error was due to stability
                if (g_last_error_code == GUIDANCE_STABILITY_FAIL) {
                    // The stability_status_g in guidance_control would have the specifics
                    // For now, just a general flag. More detailed flags could be set here
                    // based on which check failed if guidance_check_stability provided more info.
                    current_stability_flags |= 0b001; // General stability fail flag for logging
                }
                WriteLogData(true); // Log immediately with the error code
                setFlightStateLED(g_currentFlightState);
                g_pixels.show(); // Explicitly show error LED
                return; // Avoid further processing this cycle
            } else {
                g_rt.degraded = false; // healthy again (guidance stays off once disabled)
                // Add periodic health status when things are OK
                if (millis() - g_rt.lastHealthOkTime > 10000) { // Every 10 seconds (reduced frequency)
                    g_rt.lastHealthOkTime = millis();
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.print(F("Health check OK for state: "));
                        Serial.println(getStateName(g_currentFlightState));
                    }
                }
            }
        } else if (withinGracePeriod && g_debugFlags.enableSystemDebug) {
            // Debug message about grace period
            if (millis() - g_rt.lastGraceMsg > 2000) { // Every 2 seconds during grace period
                g_rt.lastGraceMsg = millis();
                Serial.print(F("Grace period active: "));
                Serial.print((errorClearGracePeriod - (millis() - g_rt.lastErrorClearTime)) / 1000.0, 1);
                Serial.println(F(" seconds remaining"));
            }
        }
    } else if (g_currentFlightState == ERROR) {
        // Add automatic error recovery logic - check if system has become healthy
        const unsigned long autoRecoveryCheckInterval = 2000; // Check every 2 seconds
        
        if (millis() - g_rt.lastAutoRecoveryCheckTime > autoRecoveryCheckInterval) {
            g_rt.lastAutoRecoveryCheckTime = millis();
            
            // Check if we can recover to PAD_IDLE state. audit #3: only when the vehicle is
            // provably on the ground and never flew - never re-arm a vehicle that may be airborne.
            if (isSensorSuiteHealthy(PAD_IDLE) && flight_is_provably_on_ground()) {
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
                
                g_rt.lastErrorClearTime = millis(); // Set grace period for future health checks
                g_stateEntryTime = millis();
                g_last_error_code = NO_ERROR; // Clear the latched error now that health is restored
                Serial.println(F("ERROR state automatically cleared - starting grace period for health checks"));
                saveStateToEEPROM();
                setFlightStateLED(g_currentFlightState);
                Serial.println(F("--------------------------------"));
                return; // Exit early to avoid the debug message below
            }
        }
        
        if (isSensorSuiteHealthy(PAD_IDLE) && !flight_is_provably_on_ground() &&
            g_debugFlags.enableSystemDebug && millis() - g_rt.lastErrorDebugTime > 5000) {
            Serial.println(F("ERROR auto-recovery REFUSED: flight in progress or vehicle not provably on the ground. Use 'reset_flight' once landed."));
        }

        // Add periodic debugging for ERROR state (only if we didn't auto-recover)
        if (g_debugFlags.enableSystemDebug) {
            if (millis() - g_rt.lastErrorDebugTime > 5000) { // Every 5 seconds (reduced frequency)
                g_rt.lastErrorDebugTime = millis();
                Serial.println(F("--- Currently in ERROR state ---"));
                Serial.println(F("Use 'clear_errors' command to manually clear if all systems are working."));
                Serial.println(F("Or check sensor health with detailed report:"));
                isSensorSuiteHealthy(PAD_IDLE, true);
                Serial.println(F("--------------------------------"));
            }
        }
    }

    // Record when we transition OUT of ERROR state (for grace period tracking)
    if (g_rt.lastRecordedState == ERROR && g_currentFlightState != ERROR) {
        g_rt.lastErrorClearTime = millis();
        Serial.println(F("ERROR state cleared - starting grace period for health checks"));
    }
    g_rt.lastRecordedState = g_currentFlightState;

    // audit #4/#5: the barometer is "usable" if calibrated AND delivering fresh samples. This
    // replaces g_ms5611Sensor.isConnected(), which pinged the I2C bus on every loop pass and
    // could not tell a stale cached value from a live one.
    const bool baroUsable = g_baroCalibrated && baroDataFresh();
    if (baroUsable) {
        currentAbsoluteBaroAlt = ms5611_get_altitude();
        currentAglAlt = currentAbsoluteBaroAlt - g_launchAltitude;
        g_currentAltitude = currentAbsoluteBaroAlt; // keep the EEPROM snapshot's altitude field meaningful
        g_rt.baroTrack.push(g_baroSample.seq, millis(), currentAbsoluteBaroAlt); // fresh samples only
    }

    if (g_currentFlightState != g_previousFlightState) {
        newStateSignal = true;
        g_previousFlightState = g_currentFlightState;
        g_stateEntryTime = millis();
        reset_max_stability_metrics(); // Reset here for any state change
        #if ENABLE_GUIDANCE == 1
        guidance_reset_stability_status(); // Also reset guidance internal stability timers
        #endif

        if (g_debugFlags.enableSystemDebug) {
            Serial.print(F("Transitioning to state: "));
            Serial.println(getStateName(g_currentFlightState));
        }
        // Populate LogData with max stability metrics from the *previous* state before resetting.
        // This means currentLogData needs to be populated *before* reset_max_stability_metrics if we want to log them.
        // However, WriteLogData is called *after* this block.
        // So, the stability metrics logged will be the fresh (zeroed) ones for the new state's first log entry.
        // This is acceptable. Max values will be captured on subsequent logs within the new state.
        WriteLogData(true);
        saveStateToEEPROM();
        setFlightStateLED(g_currentFlightState);
    }

    // Populate stability metrics for the current log entry BEFORE WriteLogData is called in the main loop.
    // These are instantaneous values or max-so-far values for the *current* processing cycle.
    // The LogData struct expects 'max_..._so_far' which implies these are accumulated.
    // The reset_max_stability_metrics() call above clears them on state change.
    // We need to update them during BOOST/COAST before WriteLogData happens.
    // Let's assume currentLogData is populated elsewhere with instantaneous rates/errors,
    // and we update the 'max_..._so_far' fields here based on those.

    // For logging, we'll update the 'max_..._so_far' fields in currentLogData
    // This should ideally be done where currentLogData is populated, right before WriteLogData()
    // For now, we will assume that currentLogData.euler_roll, .euler_pitch, .euler_yaw,
    // .icm_gyro[0,1,2] are fresh for this cycle.

    if (g_currentFlightState == BOOST || g_currentFlightState == COAST) {
        #if ENABLE_GUIDANCE == 1
        if (fabs(g_kalmanPitchRate) > fabs(max_pitch_rate_current_state_rps)) max_pitch_rate_current_state_rps = g_kalmanPitchRate;
        if (fabs(g_kalmanRollRate) > fabs(max_roll_rate_current_state_rps)) max_roll_rate_current_state_rps = g_kalmanRollRate;
        if (fabs(g_kalmanYawRate) > fabs(max_yaw_rate_current_state_rps)) max_yaw_rate_current_state_rps = g_kalmanYawRate;

        float temp_target_roll, temp_target_pitch, temp_target_yaw;
        guidance_get_target_euler_angles(temp_target_roll, temp_target_pitch, temp_target_yaw);

        float pitch_err = fabs(temp_target_pitch - g_kalmanPitch);
        float roll_err = fabs(temp_target_roll - g_kalmanRoll);
        float yaw_err = fabs(temp_target_yaw - g_kalmanYaw);
        // Normalize yaw error for max tracking
        while (yaw_err > M_PI) yaw_err -= 2.0f * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;
        yaw_err = fabs(yaw_err);

        if (pitch_err > max_pitch_att_err_current_state_rad) max_pitch_att_err_current_state_rad = pitch_err;
        if (roll_err > max_roll_att_err_current_state_rad) max_roll_att_err_current_state_rad = roll_err;
        if (yaw_err > max_yaw_att_err_current_state_rad) max_yaw_att_err_current_state_rad = yaw_err;
        #endif // ENABLE_GUIDANCE
    }
    // current_stability_flags is updated if a stability error occurs.
    // The max_..._current_state_rps etc. are updated above. These will be used
    // by the main loop in TripleT_Flight_Firmware.cpp to populate currentLogData.

    if (newStateSignal) {
        switch (g_currentFlightState) {
            case STARTUP:
                break;
            case CALIBRATION:
                // Initial message when entering CALIBRATION state
                if (g_debugFlags.enableSystemDebug) {
                    Serial.println(F("STATE: Entered CALIBRATION - Waiting for barometer calibration via 'calibrate' command. LED should be Orange."));
                }
                // Actual periodic waiting message and transition logic is in the main switch block below.
                break;
            case PAD_IDLE:
                flightSetLaunchAltitude(baroUsable ? ms5611_get_altitude() : 0.0f);
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
                if (baroUsable) {
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
                // Reset stability monitoring for the upcoming flight
                reset_max_stability_metrics();
                #if ENABLE_GUIDANCE == 1
                guidance_reset_stability_status();
                #endif
                break;
            case BOOST:
                // audit #1/#3: from liftoff until an explicit reset_flight the vehicle counts
                // as "in flight". Persist immediately so a reset a moment later still knows.
                g_flightInProgress = true;
                saveStateToEEPROM();
                if (g_useKalmanFilter && !g_icm20948_ready) {
                    // audit #2: this used to send a launched vehicle to ERROR, which stops apogee,
                    // backup-timer and main-deploy logic. Guidance needs the ICM; deployment does not.
                    flightDegrade(SENSOR_INIT_FAIL_ICM20948, "ICM20948 not ready at liftoff (guidance disabled)");
                }
                if (g_debugFlags.enableSystemDebug) Serial.println(F("BOOST: Liftoff detected!"));
                g_maxAltitudeReached = currentAglAlt > 0 ? currentAglAlt : 0;
                boostEndTime = 0; // Reset boostEndTime, it's set by detectBoostEnd
                // reset_max_stability_metrics(); // Already done when transitioning to ARMED, and again from ARMED to BOOST
                // guidance_reset_stability_status();
                break;
            case COAST: {
                if (g_debugFlags.enableSystemDebug) Serial.println(F("COAST: Motor burnout. Coasting to apogee."));
                descendingCount = 0;
                previousApogeeDetectAltitude = currentAbsoluteBaroAlt; // Capture altitude at start of coast for some apogee logic

                #if ENABLE_GUIDANCE == 1
                {
                    // Set attitude hold target based on orientation at motor burnout (end of BOOST)
                    float targetRollRad = 0.0f, targetPitchRad = 0.0f, targetYawRad = 0.0f;
                if (g_useKalmanFilter && g_icm20948_ready) {
                    targetRollRad = g_kalmanRoll;    // These are current values at transition
                    targetPitchRad = g_kalmanPitch;
                    targetYawRad = g_kalmanYaw;
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.println(F("ATT_HOLD: Using Kalman orientation for target at COAST entry."));
                    }
                } else if (g_icm20948_ready) { // Fallback if Kalman somehow not primary
                    convertQuaternionToEuler(icm_q0, icm_q1, icm_q2, icm_q3, targetRollRad, targetPitchRad, targetYawRad);
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.println(F("ATT_HOLD: Using ICM quaternion for target at COAST entry."));
                    }
                } else {
                    if (g_debugFlags.enableSystemDebug) {
                        Serial.println(F("ATT_HOLD_WARN: No valid orientation source at COAST entry. Target is 0,0,0."));
                    }
                }
                guidance_set_target_orientation_euler(targetRollRad, targetPitchRad, targetYawRad);
                if (g_debugFlags.enableSystemDebug) {
                    Serial.print(F("ATT_HOLD: Set target R:")); Serial.print(targetRollRad * (180.0f/PI), 2);
                    Serial.print(F(" P:")); Serial.print(targetPitchRad * (180.0f/PI), 2);
                    Serial.print(F(" Y:")); Serial.println(targetYawRad * (180.0f/PI), 2);
                }
                }
                #else
                if (g_debugFlags.enableSystemDebug) {
                    Serial.println(F("COAST: Guidance disabled - passive flight mode"));
                }
                #endif
                break;
            }
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
                g_rt.mainGate.reset();          // audit #5: debounce restarts on entry
                g_rt.mainFallbackLogged = false;
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
                flightHandleUnknownState();
                break;
        }
    }

    switch (g_currentFlightState) {
        case STARTUP:
            // Waiting for handleInitialStateManagement() to pick the first real state.
            break;
        case CALIBRATION:
            if (g_baroCalibrated) {
                g_currentFlightState = PAD_IDLE;
                if (g_debugFlags.enableSystemDebug) {
                    Serial.println(F("CALIBRATION: Barometer calibrated, transitioning to PAD_IDLE."));
                }
            } else {
                // Auto-calibrate when GPS fix becomes available (non-blocking)

                unsigned long timeInCalibration = millis() - g_stateEntryTime;

                // Try auto-calibration if GPS has a good fix
                if (!g_rt.autoCalibAttempted && GPS_fixType >= 3 && pDOP < 300 && ms5611_initialized_ok) {
                    g_rt.autoCalibAttempted = true;
                    Serial.println(F("CALIBRATION: GPS fix acquired, attempting auto-calibration..."));

                    // Read fresh pressure
                    int result = ms5611_read();
                    if (result == MS5611_READ_OK && pressure >= 700.0f && pressure <= 1200.0f && GPS_altitude != 0) {
                        float current_pressure_Pa = pressure * 100.0;
                        float sea_level_Pa = STANDARD_SEA_LEVEL_PRESSURE * 100.0;
                        float raw_altitude = 44330.0 * (1.0 - pow(current_pressure_Pa / sea_level_Pa, 0.190295));
                        baro_altitude_offset = (GPS_altitude / 1000.0f) - raw_altitude;
                        baro_calibration_done = true;
                        g_baroCalibrated = true;

                        Serial.print(F("CALIBRATION: Auto-calibration successful! GPS Alt="));
                        Serial.print(GPS_altitude / 1000.0f);
                        Serial.print(F("m, Baro Raw="));
                        Serial.print(raw_altitude);
                        Serial.print(F("m, Offset="));
                        Serial.print(baro_altitude_offset);
                        Serial.println(F("m"));
                    } else {
                        g_rt.autoCalibAttempted = false; // Retry on next loop if reading failed
                        if (g_debugFlags.enableSystemDebug) {
                            Serial.println(F("CALIBRATION: Auto-calibration reading failed, will retry..."));
                        }
                    }
                }

                // Timeout fallback: calibrate without GPS after CALIBRATION_AUTO_TIMEOUT_MS
                if (!g_baroCalibrated && timeInCalibration > CALIBRATION_AUTO_TIMEOUT_MS) {
                    Serial.println(F("CALIBRATION: Timeout reached, performing fallback calibration without GPS."));
                    Serial.println(F("CALIBRATION: Using raw barometric altitude (offset = 0). Altitude may be less accurate."));
                    baro_altitude_offset = 0.0f;
                    baro_calibration_done = true;
                    g_baroCalibrated = true;
                }

                // Periodic status message
                if (!g_baroCalibrated && (millis() - g_rt.lastCalibWaitMsgTime > 5000)) {
                    g_rt.lastCalibWaitMsgTime = millis();
                    unsigned long remaining = 0;
                    if (timeInCalibration < CALIBRATION_AUTO_TIMEOUT_MS) {
                        remaining = (CALIBRATION_AUTO_TIMEOUT_MS - timeInCalibration) / 1000;
                    }
                    Serial.print(F("CALIBRATION: Waiting for GPS fix (type="));
                    Serial.print(GPS_fixType);
                    Serial.print(F(", pDOP="));
                    Serial.print(pDOP / 100.0, 2);
                    Serial.print(F("). Auto-fallback in "));
                    Serial.print(remaining);
                    Serial.println(F("s. Use 'calibrate' for manual or 'skip_calibration' to skip."));
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
                // reset_max_stability_metrics(); // Already done in newStateSignal for ARMED
                // guidance_reset_stability_status(); // Already done in newStateSignal for ARMED
            } else if (millis() - g_stateEntryTime > ARMED_TIMEOUT_MS) {
                // Safety auto-disarm: ARMED_TIMEOUT_MS was defined in config.h
                // but never wired in — the vehicle previously stayed armed
                // indefinitely. Revert to PAD_IDLE; the operator can re-arm.
                Serial.println(F("ARMED timeout expired with no launch detected - auto-disarming to PAD_IDLE."));
                g_currentFlightState = PAD_IDLE;
            }
            break;
        case BOOST:
            #if ENABLE_GUIDANCE == 1
            { // Scope for act_x, act_y, act_z
                float act_x, act_y, act_z; // x=pitch, y=roll, z=yaw (from guidance_get_actuator_outputs)
                guidance_get_actuator_outputs(act_x, act_y, act_z);

                // Call stability check: current R,P,Y, R_rate,P_rate,Y_rate, cmd_pitch, cmd_yaw, cmd_roll
                guidance_check_stability(g_kalmanRoll, g_kalmanPitch, g_kalmanYaw,
                                         g_kalmanRollRate, g_kalmanPitchRate, g_kalmanYawRate,
                                         act_x, act_z, act_y, // Map to: pitch_cmd, yaw_cmd, roll_cmd
                                         millis());

                if (guidance_is_stability_compromised()) {
                    Serial.println(F("WARNING: Guidance stability compromised during BOOST - disabling guidance"));
                    guidance_log_stability_diagnostics(); // Log which check failed
                    g_guidance_active = false; // Disable guidance mid-flight
                    guidance_center_servos(); // Set all fins to neutral position
                    guidance_reset_stability_status(); // Clear flags for potential re-enable later
                    // Change LED to orange (degraded mode) if NeoPixel is available
                    extern Adafruit_NeoPixel g_pixels;
                    g_pixels.setPixelColor(0, g_pixels.Color(255, 165, 0)); // Orange = degraded mode
                    g_pixels.show();
                    Serial.println(F("=== GUIDANCE SYSTEM DISABLED ==="));
                    Serial.println(F("Reason: Stability compromised"));
                    Serial.println(F("Action: Fins centered, passive flight mode"));
                    Serial.println(F("Impact: Apogee detection and parachute deployment unaffected"));
                    Serial.println(F("================================"));
                    // Continue flight - do NOT transition to ERROR
                }
            }
            #endif

            if (baroUsable && currentAglAlt > g_maxAltitudeReached) {
                 g_maxAltitudeReached = currentAglAlt;
            }
            detectBoostEnd(); // This function internally sets g_currentFlightState = COAST if burnout detected
            break;

        case COAST:
            // Reset apogee detection counters on first entry to COAST state
            // This prevents false apogee detection from stale counter values on flight reuse
            {
                if (g_rt.lastCoastState != COAST && !g_rt.coastCountersReset) {
                    // First entry into COAST - reset all apogee detection static counters
                    resetApogeeDetectionCounters();
                    g_rt.coastCountersReset = true;
                }

                g_rt.lastCoastState = g_currentFlightState;

                // Reset flag when leaving COAST so it triggers again on next COAST entry
                if (g_currentFlightState != COAST) {
                    g_rt.coastCountersReset = false;
                }
            }

            #if ENABLE_GUIDANCE == 1
            { // Scope for act_x_c, act_y_c, act_z_c
                float act_x_c, act_y_c, act_z_c; // x=pitch, y=roll, z=yaw
                guidance_get_actuator_outputs(act_x_c, act_y_c, act_z_c);

                guidance_check_stability(g_kalmanRoll, g_kalmanPitch, g_kalmanYaw,
                                         g_kalmanRollRate, g_kalmanPitchRate, g_kalmanYawRate,
                                         act_x_c, act_z_c, act_y_c, // Map to: pitch_cmd, yaw_cmd, roll_cmd
                                         millis());

                if (guidance_is_stability_compromised()) {
                    Serial.println(F("WARNING: Guidance stability compromised during COAST - disabling guidance"));
                    guidance_log_stability_diagnostics(); // Log which check failed
                    g_guidance_active = false; // Disable guidance mid-flight
                    guidance_center_servos(); // Set all fins to neutral position
                    guidance_reset_stability_status(); // Clear flags for potential re-enable later
                    // Change LED to orange (degraded mode) if NeoPixel is available
                    extern Adafruit_NeoPixel g_pixels;
                    g_pixels.setPixelColor(0, g_pixels.Color(255, 165, 0)); // Orange = degraded mode
                    g_pixels.show();
                    Serial.println(F("=== GUIDANCE SYSTEM DISABLED ==="));
                    Serial.println(F("Reason: Stability compromised"));
                    Serial.println(F("Action: Fins centered, passive flight mode"));
                    Serial.println(F("Impact: Apogee detection and parachute deployment unaffected"));
                    Serial.println(F("================================"));
                    // Continue flight - do NOT transition to ERROR
                }
            }
            #endif

            // A COAST without a burnout timestamp would leave the backup timer unarmed.
            if (boostEndTime == 0) boostEndTime = millis() > 0 ? millis() : 1;
            if (baroUsable && currentAglAlt > g_maxAltitudeReached) {
                 g_maxAltitudeReached = currentAglAlt;
            }
            if (detectApogee()) {
                g_currentFlightState = APOGEE;
            }
            break;
        case APOGEE:
            if (DROGUE_PRESENT) {
                g_currentFlightState = DROGUE_DEPLOY;
                // Initialize entry time for pyro logic
                g_stateEntryTime = millis(); 
            } else if (MAIN_PRESENT) {
                g_currentFlightState = MAIN_DEPLOY;
                g_stateEntryTime = millis();
            } else {
                g_currentFlightState = DROGUE_DESCENT;
                if (g_debugFlags.enableSystemDebug) Serial.println(F("Warning: Apogee reached but no parachutes configured!"));
            }
            break;
        case DROGUE_DEPLOY: {
            // Non-blocking Pyro Logic
            if (DROGUE_PRESENT) {
                unsigned long timeInState = millis() - g_stateEntryTime;

                if (g_pyroFiredMask & PYRO_FIRED_DROGUE) {
                    // audit #1: this channel already completed its fire window (possibly
                    // before a reset). Never fire it again.
                    digitalWrite(PYRO_CHANNEL_1, LOW);
                    g_rt.drogueHasFired = false;
                    g_currentFlightState = DROGUE_DESCENT;
                    break;
                }

                if (!g_rt.drogueHasFired) {
                    if (g_debugFlags.enableSystemDebug) Serial.println(F("Firing Pyro Channel 1 (Drogue)"));
                    digitalWrite(PYRO_CHANNEL_1, HIGH);
                    g_rt.drogueHasFired = true;
                }

                if (timeInState >= PYRO_FIRE_DURATION) {
                    digitalWrite(PYRO_CHANNEL_1, LOW);
                    if (g_debugFlags.enableSystemDebug) Serial.println(F("Pyro Channel 1 (Drogue) Fired."));
                    g_rt.drogueHasFired = false;
                    g_currentFlightState = DROGUE_DESCENT;
                    // audit #1: record completion BEFORE anything else can reset us, so a
                    // reset during descent does not fire this channel again.
                    g_pyroFiredMask |= PYRO_FIRED_DROGUE;
                    saveStateToEEPROM();
                }
            } else {
                 g_rt.drogueHasFired = false;
                 g_currentFlightState = DROGUE_DESCENT;
            }
            break;
        }
        case DROGUE_DESCENT:
            if (MAIN_PRESENT) {
                // audit #5: the main deploy used to be `baro AGL < altitude` on ONE cached value with
                // no debounce and no fallback. Now: N consecutive FRESH samples below the deploy
                // altitude, else time-based fallbacks so a dead or lying barometer cannot strand the vehicle.
                const unsigned long timeInDescent = millis() - g_stateEntryTime;
                bool deployMain = false;
                const char* why = "";
                if (baroUsable) {
                    g_rt.mainGate.feed(g_baroSample.seq, currentAglAlt < g_main_deploy_altitude_m_agl);
                    if (g_rt.mainGate.count >= MAIN_DEPLOY_CONFIRMATION_COUNT) { deployMain = true; why = "baro altitude"; }
                } else {
                    // Barometer unavailable/stale: fall back on an estimated descent time.
                    unsigned long deadline = MAIN_DEPLOY_FALLBACK_TIME_MS;
                    if (g_maxAltitudeReached > g_main_deploy_altitude_m_agl) {
                        const float est_ms = (g_maxAltitudeReached - g_main_deploy_altitude_m_agl) /
                                             MAIN_DEPLOY_ASSUMED_DROGUE_RATE_MPS * 1000.0f * MAIN_DEPLOY_FALLBACK_MARGIN;
                        unsigned long est = est_ms > (float)MAIN_DEPLOY_FALLBACK_MIN_MS ? (unsigned long)est_ms : (unsigned long)MAIN_DEPLOY_FALLBACK_MIN_MS;
                        if (est < deadline) deadline = est;
                    }
                    if (!g_rt.mainFallbackLogged) {
                        g_rt.mainFallbackLogged = true;
                        Serial.print(F("WARNING: barometer unavailable in DROGUE_DESCENT - main deploys by timer in <= "));
                        Serial.print(deadline / 1000.0, 1);
                        Serial.println(F(" s after drogue descent began."));
                    }
                    if (timeInDescent >= deadline) { deployMain = true; why = "fallback timer (no barometer)"; }
                }
                if (!deployMain && timeInDescent >= MAIN_DEPLOY_MAX_DROGUE_TIME_MS) {
                    deployMain = true; why = "maximum drogue time exceeded";
                }
                if (deployMain) {
                    Serial.print(F("MAIN DEPLOY triggered by: "));
                    Serial.println(why);
                    g_currentFlightState = MAIN_DEPLOY;
                    g_stateEntryTime = millis(); // Initialize timer for MAIN_DEPLOY
                } else if (detectLanding()) {
                    // Touched down without main ever deploying (e.g. low apogee): do not sit here forever.
                    g_currentFlightState = LANDED;
                }
            } else {
                if (detectLanding()) {
                    g_currentFlightState = LANDED;
                }
            }
            if (g_currentFlightState == DROGUE_DESCENT && millis() - g_stateEntryTime > DESCENT_STATE_TIMEOUT_MS) {
                g_currentFlightState = LANDED; // last resort: this state must not persist indefinitely
            }
            break;
        case MAIN_DEPLOY: {
            // Non-blocking Pyro Logic
            if (MAIN_PRESENT) {
                 unsigned long timeInState = millis() - g_stateEntryTime;

                 if (g_pyroFiredMask & PYRO_FIRED_MAIN) {
                     // audit #1: already fired (possibly before a reset). Never fire again.
                     digitalWrite(PYRO_CHANNEL_2, LOW);
                     g_rt.mainHasFired = false;
                     g_currentFlightState = MAIN_DESCENT;
                     break;
                 }

                 if (!g_rt.mainHasFired) {
                     if (g_debugFlags.enableSystemDebug) Serial.println(F("Firing Pyro Channel 2 (Main)"));
                     digitalWrite(PYRO_CHANNEL_2, HIGH);
                     g_rt.mainHasFired = true;
                 }

                 if (timeInState >= PYRO_FIRE_DURATION) {
                     digitalWrite(PYRO_CHANNEL_2, LOW);
                     if (g_debugFlags.enableSystemDebug) Serial.println(F("Pyro Channel 2 (Main) Fired."));
                     g_rt.mainHasFired = false;
                     g_currentFlightState = MAIN_DESCENT;
                     // audit #1: persist completion so a reset never re-fires the main.
                     g_pyroFiredMask |= PYRO_FIRED_MAIN;
                     saveStateToEEPROM();
                 }
            } else {
                g_rt.mainHasFired = false;
                g_currentFlightState = MAIN_DESCENT;
            }
            break;
        }
        case MAIN_DESCENT:
            if (detectLanding() || millis() - g_stateEntryTime > DESCENT_STATE_TIMEOUT_MS) {
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
            flightHandleUnknownState();
            break;
    }
}

// ---------------------------------------------------------------------------
// Accelerometer access with freshness (audit #4)
// ---------------------------------------------------------------------------
struct AccelReading {
    bool valid;      // initialised, non-zero and delivering fresh samples
    float mag;       // |specific force| in g (axis / mounting independent)
    uint32_t seq;    // sample sequence number of the source used
};

static float vecMag(const float* v) { return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
static bool vecNonZero(const float* v) { return v[0] != 0.0f || v[1] != 0.0f || v[2] != 0.0f; }

// Choose the accelerometer. preferKx134 selects the high-g part (launch, burnout,
// high-force veto); otherwise the ICM-20948 is preferred for its better low-g
// resolution (free-fall detection). A source is only usable if initialised,
// non-zero and stamped by its driver within ACCEL_STALE_TIMEOUT_MS.
static AccelReading readAccel(bool preferKx134) {
    const unsigned long now = millis();
    AccelReading kx = {false, 0.0f, 0};
    AccelReading icm = {false, 0.0f, 0};
    if (g_kx134_initialized_ok && vecNonZero(kx134_accel) && g_kx134Sample.seq > 0 &&
        (now - g_kx134Sample.lastMs) <= ACCEL_STALE_TIMEOUT_MS) {
        kx = {true, vecMag(kx134_accel), g_kx134Sample.seq};
    }
    if (g_icm20948_ready && vecNonZero(icm_accel) && g_icmSample.seq > 0 &&
        (now - g_icmSample.lastMs) <= ACCEL_STALE_TIMEOUT_MS) {
        icm = {true, vecMag(icm_accel), g_icmSample.seq};
    }
    if (preferKx134) return kx.valid ? kx : icm;
    return icm.valid ? icm : kx;
}

// audit #7: evidence that the vehicle is at rest on the ground, used to authorise reset_flight.
// Unlike flight_is_provably_on_ground() this does not compare against the launch altitude
// (a vehicle can land tens of metres above/below the pad); it uses stationarity instead:
// near-zero barometric vertical speed and ~1 g specific force. When neither sensor can
// speak, only the landing states themselves (LANDED/RECOVERY) are trusted.
bool flight_is_stationary_on_ground() {
    if (flight_is_airborne_state(g_currentFlightState)) return false;
    bool evidence = false;
    if (g_baroCalibrated && baroDataFresh()) {
        float vs = 0.0f;
        if (g_rt.baroTrack.verticalSpeed(10, 8, 700, vs)) {
            if (fabsf(vs) > RESET_FLIGHT_MAX_VERTICAL_SPEED_MPS) return false;
            evidence = true;
        }
    }
    const AccelReading a = readAccel(true);
    if (a.valid) {
        if (a.mag < LANDING_ACCEL_MIN_G || a.mag > LANDING_ACCEL_MAX_G) return false;
        evidence = true;
    }
    if (evidence) return true;
    return g_currentFlightState == LANDED || g_currentFlightState == RECOVERY;
}

void detectBoostEnd() {
    if (g_currentFlightState != BOOST) return;

    // audit #4: count FRESH samples only. Previously this ran on every loop pass against
    // a value cached at 10 Hz, so COAST_CONFIRMATION_COUNT passes took about a millisecond.
    const AccelReading a = readAccel(true);
    if (!a.valid) return; // no fresh data: neither confirm nor reset

    g_rt.coastConfirm.feed(a.seq, a.mag < COAST_ACCEL_THRESHOLD);
    if (g_rt.coastConfirm.count >= COAST_CONFIRMATION_COUNT) {
        boostEndTime = millis();
        g_currentFlightState = COAST;
        g_rt.coastConfirm.reset();
    }
}

// Helper function to reset apogee detection counters
// Called when entering COAST state to prevent false apogee from stale values
void resetApogeeDetectionCounters() {
    g_rt.apogeeBaro.reset();
    g_rt.apogeeBaroRef = 0.0f;
    g_rt.apogeeBaroRefValid = false;
    g_rt.apogeeAccel.reset();
    g_rt.freefallStartMs = 0;
    g_rt.apogeeGps.reset();
    g_rt.maxGpsAlt = 0.0f;
    g_rt.gpsAltValid = false;
}

// Reset every piece of flight-logic bookkeeping that must not leak from one
// flight (or one unit test) into the next: detector counters, confirmation
// timers, landing averager, pyro fire-window flags and health-monitor timers.
// Does NOT touch the persisted flight record (see state_management.cpp).
void flightLogicReset() {
    g_rt = FlightRuntime();
    resetApogeeDetectionCounters();
    boostEndTime = 0;
    landingDetectedFlag = false;
    previousApogeeDetectAltitude = 0.0f;
    lastLandingCheckAltitudeAgl = 0.0f;
    descendingCount = 0;
    lastStateBroadcastTime = 0;
    reset_max_stability_metrics();
}

// ---------------------------------------------------------------------------
// Apogee detection (audit #4)
//
// OR / first-match semantics are kept (baro -> accelerometer -> GPS -> backup timer),
// but no sensor method can fire on its own without:
//   * FRESH-sample confirmation (counts advance only when the sensor produced a new
//     sample, so N counts = N sensor periods),
//   * the common gates: at least APOGEE_MIN_TIME_AFTER_BURNOUT_MS since burnout and
//     (with a working barometer) at least APOGEE_MIN_ALTITUDE_GAIN_M of climb, and
//   * an INDEPENDENT cross-check that cannot be fooled by the same fault:
//       baro  : vetoed while the accelerometer still shows hard deceleration
//               (specific force > APOGEE_HIGH_FORCE_VETO_G) and locked out for
//               APOGEE_BARO_TRANSONIC_LOCKOUT_MS after burnout (its descent
//               reference restarts when the lockout ends);
//       accel : magnitude of specific force below APOGEE_ACCEL_FREEFALL_G (near
//               free fall) sustained for a window, vetoed while the barometer is
//               still climbing faster than APOGEE_CLIMB_VETO_MPS;
//       GPS   : vetoed by either check above.
//   A missing / stale cross-check sensor never vetoes, so a single sensor failure
//   cannot block deployment. The backup timer is ungated: it is the last resort.
// ---------------------------------------------------------------------------
bool detectApogee() {
    const unsigned long now = millis();
    const bool haveBurnout = boostEndTime > 0;
    const unsigned long sinceBurnout = haveBurnout ? now - boostEndTime : 0;
    const bool baroUsable = g_baroCalibrated && baroDataFresh();

    // Cross-check inputs (absent data never vetoes).
    float vs = 0.0f;
    const bool haveVs = baroUsable && g_rt.baroTrack.verticalSpeed(10, 5, 300, vs);
    const bool baroClimbing = haveVs && vs > APOGEE_CLIMB_VETO_MPS;
    const AccelReading hi = readAccel(true);
    const bool highForce = hi.valid && hi.mag > APOGEE_HIGH_FORCE_VETO_G;

    const bool gatesOk = haveBurnout && sinceBurnout >= APOGEE_MIN_TIME_AFTER_BURNOUT_MS &&
                         (!baroUsable || g_maxAltitudeReached >= APOGEE_MIN_ALTITUDE_GAIN_M);

    bool apogeeDetected = false;

    // Method 1: Barometric Detection (Primary)
    // AGL is compared with AGL: g_maxAltitudeReached is tracked in metres above ground
    // level. The descent reference is the peak AGL seen AFTER the transonic lockout.
    if (baroUsable) {
        const float agl = ms5611_get_altitude() - g_launchAltitude;
        if (haveBurnout && sinceBurnout < APOGEE_BARO_TRANSONIC_LOCKOUT_MS) {
            g_rt.apogeeBaro.reset();
            g_rt.apogeeBaroRef = agl;          // reference restarts when the lockout ends
            g_rt.apogeeBaroRefValid = true;
        } else {
            if (!g_rt.apogeeBaroRefValid || agl > g_rt.apogeeBaroRef) {
                g_rt.apogeeBaroRef = agl;
                g_rt.apogeeBaroRefValid = true;
            }
            g_rt.apogeeBaro.feed(g_baroSample.seq, agl < g_rt.apogeeBaroRef - APOGEE_BARO_DESCENT_THRESHOLD);
        }

        if (g_rt.apogeeBaro.count >= APOGEE_CONFIRMATION_COUNT && gatesOk && !highForce) {
            if (g_debugFlags.enableSystemDebug) Serial.println(F("APOGEE DETECTED (Barometer)"));
            apogeeDetected = true;
        }
    }

    // Method 2: Accelerometer free-fall Detection (Secondary)
    if (!apogeeDetected) {
        const AccelReading a = readAccel(false);
        if (a.valid) {
            const bool inFreefall = a.mag < APOGEE_ACCEL_FREEFALL_G;
            if (g_rt.apogeeAccel.feed(a.seq, inFreefall)) {
                if (inFreefall) {
                    if (g_rt.freefallStartMs == 0) g_rt.freefallStartMs = now > 0 ? now : 1;
                } else {
                    g_rt.freefallStartMs = 0;
                }
            }
            const unsigned long window = baroUsable ? APOGEE_ACCEL_FREEFALL_WINDOW_MS
                                                    : APOGEE_ACCEL_FREEFALL_WINDOW_NO_BARO_MS;
            if (g_rt.apogeeAccel.count >= APOGEE_ACCEL_SAMPLES && g_rt.freefallStartMs != 0 &&
                (now - g_rt.freefallStartMs) >= window && gatesOk && !baroClimbing) {
                if (g_debugFlags.enableSystemDebug) Serial.println(F("APOGEE DETECTED (Accelerometer free fall)"));
                apogeeDetected = true;
            }
        }
    }

    // Method 3: GPS Altitude Detection (Tertiary) - needs a 3D fix and fresh PVT
    if (!apogeeDetected) {
        const uint8_t fix = getFixType();
        if ((fix == 3 || fix == 4) && g_gpsSample.seq > 0 && (now - g_gpsSample.lastMs) <= GPS_STALE_TIMEOUT_MS) {
            const bool freshGps = !g_rt.apogeeGps.primed || g_gpsSample.seq != g_rt.apogeeGps.lastSeq;
            if (freshGps) {
                const float gpsAlt = getGPSAltitude();
                if (!g_rt.gpsAltValid || gpsAlt > g_rt.maxGpsAlt) {
                    g_rt.maxGpsAlt = gpsAlt;
                    g_rt.gpsAltValid = true;
                }
                g_rt.apogeeGps.feed(g_gpsSample.seq, gpsAlt < g_rt.maxGpsAlt - APOGEE_GPS_DESCENT_THRESHOLD_M);
            }
            if (g_rt.apogeeGps.count >= APOGEE_GPS_CONFIRMATION_COUNT && gatesOk && !baroClimbing && !highForce) {
                if (g_debugFlags.enableSystemDebug) Serial.println(F("APOGEE DETECTED (GPS)"));
                apogeeDetected = true;
            }
        }
    }

    // Method 4: Backup Timer (Failsafe) - ungated by design
    if (!apogeeDetected && haveBurnout) {
        if (sinceBurnout > BACKUP_APOGEE_TIME_MS) {
            if (g_debugFlags.enableSystemDebug) Serial.println(F("APOGEE DETECTED (Backup Timer)"));
            apogeeDetected = true;
        }
    }

    return apogeeDetected;
}

bool detectLanding() {
    if (g_currentFlightState != DROGUE_DESCENT && g_currentFlightState != MAIN_DESCENT) return false;

    // Use a moving average of altitude to smooth out readings
    const int numReadings = FlightRuntime::kLandingReadings;
    float* altReadings = g_rt.landingAlt;
    int& readIndex = g_rt.landingIndex;
    float& total = g_rt.landingTotal;
    if (g_rt.landingFirstRun) {
        for (int i = 0; i < numReadings; i++) altReadings[i] = 0;
        g_rt.landingFirstRun = false;
    }

    total -= altReadings[readIndex];
    altReadings[readIndex] = ms5611_get_altitude();
    total += altReadings[readIndex];
    readIndex = (readIndex + 1) % numReadings;
    float avgAlt = total / numReadings;

    unsigned long& firstLandedTime = g_rt.landingFirstLandedTime;
    
    // Check for landing conditions
    if (fabs(avgAlt - g_launchAltitude) < LANDING_ALTITUDE_STABLE_THRESHOLD) {
        float accelMag = get_accel_magnitude(g_kx134_initialized_ok, kx134_accel, g_icm20948_ready, icm_accel, g_debugFlags.enableSystemDebug);
        if (accelMag >= LANDING_ACCEL_MIN_G && accelMag <= LANDING_ACCEL_MAX_G) {
            if (firstLandedTime == 0) firstLandedTime = millis();
            if (millis() - firstLandedTime >= LANDING_CONFIRMATION_TIME_MS) {
                return true;
            }
        }
    } else {
        // Reset landing timer if altitude condition is not met
        firstLandedTime = 0;
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