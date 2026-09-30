// Boot-time resolution of the first flight state. Moved out of
// TripleT_Flight_Firmware.cpp so it can be unit tested against the real code.
#include "startup_state.h"
#include <Arduino.h>
#include "config.h"
#include "data_structures.h"
#include "error_codes.h"
#include "debug_flags.h"
#include "state_management.h"
#include "utility_functions.h"
#include "ms5611_functions.h"
#include "sensor_samples.h"
#include "flight_logic.h"

// Globals defined in TripleT_Flight_Firmware.cpp / the sensor drivers.
extern FlightState g_currentFlightState;
extern unsigned long g_stateEntryTime;
extern bool g_baroCalibrated;
extern bool g_sdCardAvailable;
extern bool g_loggingEnabled;
extern bool g_kx134_initialized_ok;
extern bool g_icm20948_ready;
extern ErrorCode_t g_last_error_code;
extern DebugFlags g_debugFlags;

static bool s_initialStateHandled = false;

void handleInitialStateManagementReset() { s_initialStateHandled = false; }

// Function to handle initial state management after setup
void handleInitialStateManagement() {
  if (s_initialStateHandled) {
    return; // Only run this logic once
  }

  // audit #1: sensors are up now, so the in-flight recovery decision (which needs
  // a live barometer reading) can be made. This runs BEFORE any health logic so
  // the restart state is settled first. Until this point the vehicle was in the
  // pyro-inert STARTUP state.
  if (recoveryPending()) {
    const bool baroValid = ms5611_initialized_ok && pressure > 300.0f && pressure < 1200.0f;
    // baro_altitude_offset is still 0 here (nothing restored yet), so this is the raw altitude.
    if (!recoveryEvidenceStep(baroValid, g_baroSample.seq, baroValid ? ms5611_get_altitude() : 0.0f, millis())) {
      return; // still observing the barometer; vehicle stays pyro-inert in STARTUP
    }
  }

  // Check system health using the same criteria as the setup function used to
  bool systemHealthy = true;
  
  if (!g_sdCardAvailable && g_loggingEnabled) {
    Serial.println(F("INIT: Logging enabled but SD card not available. System unhealthy."));
    systemHealthy = false;
  }
  
  if (!g_kx134_initialized_ok && !g_icm20948_ready) {
    Serial.println(F("INIT: No IMU available (both KX134 and ICM20948 failed). System unhealthy."));
    systemHealthy = false;
  }
  
  if (!ms5611_initialized_ok) {
    // Serial.println(F("INIT: MS5611 barometer not initialized. System unhealthy.")); // Keep systemHealthy = true
    // systemHealthy = false; // Allow proceeding to CALIBRATION/PAD_IDLE with a warning
    if (g_debugFlags.enableSystemDebug) { // Still print a warning if debug is enabled
        Serial.println(F("INIT_WARNING: MS5611 barometer not initialized. Functionality will be limited."));
    }
  }

  if (g_debugFlags.enableSystemDebug) {
    Serial.println(F("=== Initial State Management ==="));
    Serial.print(F("System Health: ")); Serial.println(systemHealthy ? F("HEALTHY") : F("UNHEALTHY"));
    Serial.print(F("Current State: ")); Serial.println(getStateName(g_currentFlightState));
  }

  // Handle state transitions based on current state and system health
  if (g_currentFlightState == STARTUP) {
    if (systemHealthy) {
      // Check if barometer is already calibrated to skip CALIBRATION state
      if (g_baroCalibrated) {
        Serial.println(F("Fresh start, system healthy and barometer calibrated, proceeding to PAD_IDLE state."));
        g_currentFlightState = PAD_IDLE;
        g_stateEntryTime = millis();
      } else {
        Serial.println(F("Fresh start, system healthy, proceeding to CALIBRATION state."));
        g_currentFlightState = CALIBRATION;
        g_stateEntryTime = millis();
      }
    } else {
      Serial.println(F("Fresh start but system unhealthy, transitioning to ERROR state."));
      g_last_error_code = STATE_TRANSITION_INVALID_HEALTH; // Or a more specific init error if identifiable here
      g_currentFlightState = ERROR;
      g_stateEntryTime = millis();
      saveStateToEEPROM(); // Save state
      WriteLogData(true);  // Log error immediately
    }
  } else if (g_currentFlightState == ERROR && systemHealthy) {
    // Check if barometer is already calibrated to skip CALIBRATION state
    if (g_baroCalibrated) {
      Serial.println(F("ERROR state recovered, all systems healthy and barometer calibrated, transitioning to PAD_IDLE."));
      g_currentFlightState = PAD_IDLE;
      g_stateEntryTime = millis();
    } else {
      Serial.println(F("ERROR state recovered and all systems are healthy. Automatically clearing error and transitioning to CALIBRATION."));
      g_currentFlightState = CALIBRATION;
      g_stateEntryTime = millis();
    }
    g_last_error_code = NO_ERROR; // Clear the latched error along with the state
    Serial.println(F("ERROR state cleared - starting grace period for health checks"));
    saveStateToEEPROM();
  } else if (!systemHealthy && g_currentFlightState != ERROR && !flight_error_allowed(g_currentFlightState)) {
    // audit #2: a resumed in-flight state must keep running its deployment logic.
    Serial.println(F("System unhealthy during initialization, but a flight is in progress: staying in the current state (degraded)."));
  } else if (!systemHealthy && g_currentFlightState != ERROR) {
    Serial.println(F("System became unhealthy during initialization, transitioning to ERROR state."));
    g_last_error_code = STATE_TRANSITION_INVALID_HEALTH; // Or a more specific init error
    g_currentFlightState = ERROR;
    g_stateEntryTime = millis();
    saveStateToEEPROM(); // Save state
    WriteLogData(true);  // Log error immediately
  }

  if (g_debugFlags.enableSystemDebug) {
    Serial.print(F("Final State: ")); Serial.println(getStateName(g_currentFlightState));
    Serial.println(F("=============================="));
  }

  s_initialStateHandled = true;
}

