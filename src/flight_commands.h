#ifndef FLIGHT_COMMANDS_H
#define FLIGHT_COMMANDS_H

#include "data_structures.h" // FlightState

// Serial commands that change the flight state (or are dangerous in flight).
// Kept out of command_processor.cpp (which drags in SdFat and friends) so their
// guards can be unit tested against the real code.
//
// Handles (case-insensitive): clear_errors, clear_to_calibration, skip_calibration,
// reset_flight [token] (audit #7: token-confirmed, ground states only, stationary).
// Returns true if `command` was recognised (whether or not it was allowed).
//
// audit #3: leaving ERROR is only permitted when flight_is_provably_on_ground().
bool handleFlightStateCommand(const char* command,
                              FlightState& currentState,
                              FlightState& previousState,
                              unsigned long& stateEntryTime,
                              bool& baroCalibrated,
                              bool ms5611Initialized);

#endif // FLIGHT_COMMANDS_H
