#include "flight_commands.h"
#include <Arduino.h>
#include <string.h>
#include <strings.h>
#include "config.h"
#include "error_codes.h"
#include "state_management.h"
#include "utility_functions.h"   // isSensorSuiteHealthy, getStateName
#include "flight_logic.h"        // flight_is_provably_on_ground

extern ErrorCode_t g_last_error_code;
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

bool handleFlightStateCommand(const char* command,
                              FlightState& currentFlightState_ref,
                              FlightState& previousFlightState_ref,
                              unsigned long& stateEntryTime_ref,
                              bool& baroCalibrated_ref,
                              bool ms5611_initialized_ok) {
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
