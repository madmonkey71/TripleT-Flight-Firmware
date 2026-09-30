#include "flight_commands.h"
#include <Arduino.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include "config.h"
#include "error_codes.h"
#include "state_management.h"
#include "utility_functions.h"   // isSensorSuiteHealthy, getStateName
#include "flight_logic.h"        // flight_is_provably_on_ground, flight_is_stationary_on_ground

extern ErrorCode_t g_last_error_code;
extern float g_maxAltitudeReached;
// From ms5611_functions.cpp
extern float baro_altitude_offset;
extern bool baro_calibration_done;

// Print the standard refusal for a command that must not run unless the vehicle
// is provably on the ground and has never flown.
static void printGroundRefusal(const char* command) {
  Serial.print(F("REFUSED: '"));
  Serial.print(command);
  Serial.println(F("' is only allowed when the vehicle is provably on the ground and was never in flight."));
  Serial.println(F("A flight is in progress or the barometer shows the vehicle is not at the launch altitude."));
  Serial.println(F("After landing, use 'reset_flight' (LANDED/RECOVERY/ERROR/PAD_IDLE only)."));
}

// reset_flight two-step confirmation (audit #7)
static unsigned s_resetToken = 0;              // 0 = none issued
static unsigned long s_resetTokenIssuedMs = 0;

// Clear the persisted flight so the vehicle can be re-armed. Only from the ground states.
static bool resetFlightStateAllowed(FlightState s) {
  return s == LANDED || s == RECOVERY || s == ERROR || s == PAD_IDLE;
}

static void handleResetFlight(const char* command,
                              FlightState& currentFlightState_ref,
                              FlightState& previousFlightState_ref,
                              unsigned long& stateEntryTime_ref,
                              bool& baroCalibrated_ref,
                              bool ms5611_initialized_ok) {
  // Guards first, so a token is never even issued in a state where the command cannot run.
  if (!resetFlightStateAllowed(currentFlightState_ref)) {
    Serial.print(F("REFUSED: reset_flight is only allowed in LANDED, RECOVERY, ERROR or PAD_IDLE. Current state: "));
    Serial.println(getStateName(currentFlightState_ref));
    return;
  }
  if (!flight_is_stationary_on_ground()) {
    Serial.println(F("REFUSED: reset_flight requires the vehicle to be at rest on the ground (baro vertical speed ~0 and ~1 g)."));
    return;
  }

  // Optional argument: the confirmation token.
  const char* arg = command + strlen("reset_flight");
  while (*arg == ' ') arg++;

  const unsigned long now = millis();
  const bool tokenLive = s_resetToken != 0 && (now - s_resetTokenIssuedMs) <= RESET_FLIGHT_TOKEN_TIMEOUT_MS;
  if (*arg == '\0' || !tokenLive || (unsigned)atoi(arg) != s_resetToken) {
    if (*arg != '\0') {
      Serial.println(tokenLive ? F("REFUSED: wrong confirmation token.") : F("REFUSED: no live confirmation token (expired or never issued)."));
    }
    // (Re)issue a fresh token.
    s_resetToken = 1000 + (unsigned)((now / 7 + 4321) % 9000);
    s_resetTokenIssuedMs = now;
    Serial.println(F("reset_flight will CLEAR the recorded flight (flight-in-progress flag, pyro-fired flags, max altitude)"));
    Serial.println(F("and return the vehicle to PAD_IDLE. Make sure the vehicle is safe and recovered."));
    Serial.print(F("To confirm within "));
    Serial.print(RESET_FLIGHT_TOKEN_TIMEOUT_MS / 1000);
    Serial.print(F(" s type:  reset_flight "));
    Serial.println(s_resetToken);
    return;
  }
  s_resetToken = 0; // single use

  // ---- execute ----
  const FlightState from = currentFlightState_ref;
  stateManagementResetRuntime();      // flight-in-progress, pyro-fired mask, resume count
  flightLogicReset();                 // detectors, timers, boostEndTime
  g_maxAltitudeReached = 0.0f;
  g_last_error_code = NO_ERROR;

  FlightState target;
  if (baroCalibrated_ref && isSensorSuiteHealthy(PAD_IDLE)) target = PAD_IDLE;
  else if (ms5611_initialized_ok) target = CALIBRATION;
  else target = ERROR; // legal now: the flight flag is clear
  previousFlightState_ref = currentFlightState_ref;
  currentFlightState_ref = target;
  stateEntryTime_ref = millis();
  saveStateToEEPROM();

  Serial.print(F("Flight record cleared (was "));
  Serial.print(getStateName(from));
  Serial.print(F("). System is now in "));
  Serial.println(getStateName(target));
}

bool handleFlightStateCommand(const char* command,
                              FlightState& currentFlightState_ref,
                              FlightState& previousFlightState_ref,
                              unsigned long& stateEntryTime_ref,
                              bool& baroCalibrated_ref,
                              bool ms5611_initialized_ok) {
#if ENABLE_TEST_COMMANDS
    // Bench-only (audit #13): hang the main loop to prove the watchdog resets the vehicle.
    // Compiled out of flight builds; even when present, only allowed on the pad.
    if (strcasecmp(command, "TEST_FREEZE") == 0) {
        if (currentFlightState_ref != PAD_IDLE) {
            Serial.print(F("REFUSED: TEST_FREEZE is only allowed in PAD_IDLE. Current state: "));
            Serial.println(getStateName(currentFlightState_ref));
            return true;
        }
        Serial.println(F("Freezing system for 6 seconds (Watchdog should trigger)..."));
        delay(6000); // Exceeds 5s watchdog timeout
        return true;
    }
#endif

    if (strncasecmp(command, "reset_flight", 12) == 0 && (command[12] == '\0' || command[12] == ' ')) {
        handleResetFlight(command, currentFlightState_ref, previousFlightState_ref, stateEntryTime_ref,
                          baroCalibrated_ref, ms5611_initialized_ok);
        return true;
    }

    if (strcasecmp(command, "clear_errors") == 0) {
        if (currentFlightState_ref != ERROR) {
            Serial.print(F("System is not in ERROR state. Current state: "));
            Serial.println(getStateName(currentFlightState_ref));
            return true;
        }
        if (!flight_is_provably_on_ground()) { printGroundRefusal("clear_errors"); return true; }

        Serial.println(F("Attempting to clear error state..."));

        // First, let's check what specifically is failing
        Serial.println(F("Checking system health for PAD_IDLE state:"));
        bool healthy = isSensorSuiteHealthy(PAD_IDLE, true); // Call with verbose=true to see details

        if (healthy) {
            previousFlightState_ref = currentFlightState_ref;
            currentFlightState_ref = PAD_IDLE;
            stateEntryTime_ref = millis();
            g_last_error_code = NO_ERROR; // Clear the latched error along with the state
            saveStateToEEPROM();
            Serial.println(F("Error state cleared. System reset to PAD_IDLE. Check sensors."));
        } else {
            Serial.println(F("Cannot clear error: System health check for PAD_IDLE failed."));
            Serial.println(F(""));
            Serial.println(F("Troubleshooting steps:"));
            Serial.println(F("1. Check if barometer needs calibration: use 'calibrate' or 'h' command"));
            Serial.println(F("2. Check sensor status: use 'status' or 'b' command"));
            Serial.println(F("3. Verify IMU initialization: at least one of ICM20948 or KX134 must be ready"));
            Serial.println(F("4. If barometer is the issue, try transitioning to CALIBRATION state first"));
            Serial.println(F(""));

            // Offer alternative: clear to CALIBRATION state if barometer is the main issue
            if (!baroCalibrated_ref && ms5611_initialized_ok) {
                Serial.println(F("Alternative: Barometer is initialized but not calibrated."));
                Serial.println(F("Would you like to clear to CALIBRATION state instead? (Type 'clear_to_calibration')"));
            }
        }
        return true;
    }

    if (strcasecmp(command, "clear_to_calibration") == 0) {
        if (currentFlightState_ref != ERROR) {
            Serial.print(F("System is not in ERROR state. Current state: "));
            Serial.println(getStateName(currentFlightState_ref));
            return true;
        }
        if (!flight_is_provably_on_ground()) { printGroundRefusal("clear_to_calibration"); return true; }

        Serial.println(F("Attempting to clear error state to CALIBRATION..."));

        // Check minimal requirements for CALIBRATION state (just barometer initialized)
        if (ms5611_initialized_ok) {
            previousFlightState_ref = currentFlightState_ref;
            currentFlightState_ref = CALIBRATION;
            stateEntryTime_ref = millis();
            g_last_error_code = NO_ERROR; // Clear the latched error along with the state
            saveStateToEEPROM();
            Serial.println(F("Error state cleared. System reset to CALIBRATION state."));
            Serial.println(F("Use 'calibrate' or 'h' command to calibrate barometer with GPS."));
        } else {
            Serial.println(F("Cannot clear to CALIBRATION: Barometer (MS5611) not initialized."));
            Serial.println(F("Check hardware connections and restart system."));
        }
        return true;
    }

    if (strcasecmp(command, "skip_calibration") == 0) {
        if (currentFlightState_ref != CALIBRATION && currentFlightState_ref != ERROR) {
            Serial.print(F("Skip calibration only available in CALIBRATION or ERROR state. Current: "));
            Serial.println(getStateName(currentFlightState_ref));
            return true;
        }
        // audit #3: from ERROR this jumps straight to PAD_IDLE - same proof required.
        if (currentFlightState_ref == ERROR && !flight_is_provably_on_ground()) { printGroundRefusal("skip_calibration"); return true; }

        if (ms5611_initialized_ok) {
            Serial.println(F("Skipping GPS-based calibration."));
            Serial.println(F("Using raw barometric altitude (offset = 0)."));
            Serial.println(F("WARNING: Altitude readings may be less accurate without GPS calibration."));
            baro_altitude_offset = 0.0f;
            baro_calibration_done = true;
            baroCalibrated_ref = true;
            previousFlightState_ref = currentFlightState_ref;
            currentFlightState_ref = PAD_IDLE;
            stateEntryTime_ref = millis();
            saveStateToEEPROM();
            Serial.println(F("Calibration skipped. System transitioned to PAD_IDLE."));
        } else {
            Serial.println(F("ERROR: Cannot skip calibration - barometer (MS5611) not initialized."));
        }
        return true;
    }

    return false;
}
