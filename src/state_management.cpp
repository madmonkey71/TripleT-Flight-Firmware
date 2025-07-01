#include "state_management.h"
#include <EEPROM.h>
#include "constants.h" // For EEPROM_UPDATE_INTERVAL and other timing constants
#include "config.h" // For EEPROM configuration defines
#include "utility_functions.h" // For getStateName
#include "debug_flags.h" // For g_debugFlags
#include <Arduino.h> // For Serial, millis()

// Global instance of FlightStateData, mirroring State Machine.md
FlightStateData stateData; // This holds the data read from/written to EEPROM
unsigned long lastStateSaveTime = 0; // Renamed from lastStateSave to avoid conflict if State Machine.md meant a global in main .cpp

// Extern variables from TripleT_Flight_Firmware.cpp that are needed by these functions.
extern FlightState currentFlightState;
extern float launchAltitude;       // Assuming this is the ground altitude recorded at PAD_IDLE
extern float maxAltitudeReached;   // Assuming this tracks peak altitude
extern float currentAltitude;      // Current altitude from sensors
extern float g_main_deploy_altitude_m_agl; // Added for saving state
// extern bool enableSystemDebug;     // Replaced by g_debugFlags
extern DebugFlags g_debugFlags;     // Declare g_debugFlags

void saveStateToEEPROM() {
  unsigned long currentTimeMillis = millis();

  // Only save periodically to reduce wear, unless in critical phases
  if (currentTimeMillis - lastStateSaveTime < EEPROM_UPDATE_INTERVAL &&
      currentFlightState != APOGEE && currentFlightState != DROGUE_DEPLOY &&
      currentFlightState != MAIN_DEPLOY && currentFlightState != LANDED) { // Added LANDED as a critical save point
    return;
  }

  // Update state data from global variables
  stateData.state = static_cast<uint8_t>(currentFlightState); // Cast enum to uint8_t
  stateData.launchAltitude = launchAltitude;
  stateData.maxAltitude = maxAltitudeReached;
  stateData.currentAltitude = currentAltitude; // This should be the latest available altitude
  stateData.mainDeployAltitudeAgl = g_main_deploy_altitude_m_agl; // Save new field
  stateData.timestamp = currentTimeMillis;
  stateData.signature = EEPROM_SIGNATURE_VALUE;

  // Write to EEPROM
  EEPROM.put(EEPROM_STATE_ADDR, stateData);

  lastStateSaveTime = currentTimeMillis;

  if (g_debugFlags.enableSystemDebug) {
    Serial.print(F("Flight state saved to EEPROM: "));
    Serial.println(getStateName(currentFlightState));
  }
}

static bool loadStateFromEEPROM() { // Changed to static
  EEPROM.get(EEPROM_STATE_ADDR, stateData);

  if (stateData.signature != EEPROM_SIGNATURE_VALUE) {
    if (g_debugFlags.enableSystemDebug) { // Only print if debug is enabled
        Serial.println(F("No valid flight state found in EEPROM. Initializing with default values."));
    }
    // Optionally initialize stateData to defaults if signature is bad
    stateData.state = static_cast<uint8_t>(STARTUP); // Default to STARTUP
    stateData.launchAltitude = 0.0f;
    stateData.maxAltitude = 0.0f;
    stateData.currentAltitude = 0.0f;
    stateData.mainDeployAltitudeAgl = 0.0f; // Default for new field
    stateData.timestamp = 0;
    // Do not set signature here, as it indicates invalid data
    return false;
  }

  if (g_debugFlags.enableSystemDebug) {
      Serial.println(F("Found valid flight state in EEPROM:"));
      Serial.print(F("State: "));
      Serial.println(getStateName(static_cast<FlightState>(stateData.state)));
      Serial.print(F("Launch altitude: "));
      Serial.println(stateData.launchAltitude);
      Serial.print(F("Max altitude: "));
      Serial.println(stateData.maxAltitude);
      Serial.print(F("Last altitude: "));
      Serial.println(stateData.currentAltitude);
      Serial.print(F("Main Deploy AGL: ")); Serial.println(stateData.mainDeployAltitudeAgl); // Print new field
      Serial.print(F("Timestamp: "));
      Serial.println(stateData.timestamp);
  }
  return true;
}

void recoverFromPowerLoss() {
  if (!loadStateFromEEPROM()) {
    // No valid data, or data is explicitly invalid. Start fresh.
    // currentFlightState is already STARTUP by default.
    // Other variables like launchAltitude, maxAltitudeReached will be initialized in their respective states.
    if (g_debugFlags.enableSystemDebug) {
        Serial.println(F("Proceeding with normal startup sequence."));
    }
    return;
  }

  // Valid data loaded into stateData. Now, restore global state variables.
  FlightState savedStateEnum = static_cast<FlightState>(stateData.state);
  currentFlightState = savedStateEnum; // Restore current flight state

  // Restore other critical variables based on the loaded stateData
  launchAltitude = stateData.launchAltitude;
  maxAltitudeReached = stateData.maxAltitude;
  g_main_deploy_altitude_m_agl = stateData.mainDeployAltitudeAgl; // Restore new field
  // currentAltitude will be updated by sensor readings, but stateData.currentAltitude can be logged or used for initial estimate.

  if (g_debugFlags.enableSystemDebug) {
    Serial.print(F("Attempting recovery. Saved state: "));
    Serial.println(getStateName(savedStateEnum));
  }

  // Handle recovery logic based on the saved state
  switch (savedStateEnum) {
    case STARTUP:
    case CALIBRATION:
    case PAD_IDLE:
      // For these early states, it's safest to restart the sequence from STARTUP.
      // The main setup() will proceed to CALIBRATION and then PAD_IDLE if all checks pass.
      currentFlightState = STARTUP;
      if (g_debugFlags.enableSystemDebug) Serial.println(F("Recovery: Resetting to STARTUP."));
      break;

    case ARMED:
      // Was armed but not launched. Revert to PAD_IDLE for safety checks.
      currentFlightState = PAD_IDLE;
      if (g_debugFlags.enableSystemDebug) Serial.println(F("Recovery: Was ARMED, reverting to PAD_IDLE."));
      break;

    case BOOST:
    case COAST:
      // Power loss during ascent. This is a tricky state.
      // Defaulting to DROGUE_DESCENT is a conservative approach.
      // Apogee detection will run again if still relevant, or it will proceed through descent.
      currentFlightState = DROGUE_DESCENT; // Assume apogee was missed or passed.
      // maxAltitudeReached is already restored from EEPROM.
      if (g_debugFlags.enableSystemDebug) Serial.println(F("Recovery: Was BOOST/COAST, transitioning to DROGUE_DESCENT. Max alt restored."));
      break;

    case APOGEE:
    case DROGUE_DEPLOY:
      // If power lost exactly at apogee or during drogue deployment, ensure drogue deployment logic is triggered.
      // The state machine should handle deploying drogue if it's in DROGUE_DEPLOY state on the next iteration.
      currentFlightState = DROGUE_DEPLOY;
      if (g_debugFlags.enableSystemDebug) Serial.println(F("Recovery: Was APOGEE/DROGUE_DEPLOY, ensuring DROGUE_DEPLOY state."));
      break;

    case DROGUE_DESCENT:
      // Already descending under drogue. Continue in this state.
      // Altitudes (launch, max) are already restored.
      currentFlightState = DROGUE_DESCENT;
      if (g_debugFlags.enableSystemDebug) Serial.println(F("Recovery: Resuming DROGUE_DESCENT. Altitudes restored."));
      break;

    case MAIN_DEPLOY:
      // Similar to drogue, ensure main deployment logic is triggered.
      currentFlightState = MAIN_DEPLOY;
      if (g_debugFlags.enableSystemDebug) Serial.println(F("Recovery: Was MAIN_DEPLOY, ensuring MAIN_DEPLOY state."));
      break;

    case MAIN_DESCENT:
    case LANDED: // If landed, it's safe to go to RECOVERY
    case RECOVERY:
      // For later flight stages, or if already landed/in recovery, proceed to RECOVERY state.
      currentFlightState = RECOVERY;
      if (g_debugFlags.enableSystemDebug) Serial.println(F("Recovery: Was MAIN_DESCENT/LANDED/RECOVERY, transitioning to RECOVERY."));
      break;

    case ERROR:
      // If an error state was saved, remain in ERROR state.
      currentFlightState = ERROR;
      if (g_debugFlags.enableSystemDebug) Serial.println(F("Recovery: Resuming in ERROR state."));
      break;

    default:
      // Unknown state, safest to restart.
      currentFlightState = STARTUP;
      if (g_debugFlags.enableSystemDebug) {
        Serial.print(F("Recovery: Unknown saved state, resetting to STARTUP. State value: "));
        Serial.println(stateData.state);
      }
      break;
  }

  if (g_debugFlags.enableSystemDebug) {
    Serial.print(F("State after recovery logic: "));
    Serial.println(getStateName(currentFlightState));
  }
}
