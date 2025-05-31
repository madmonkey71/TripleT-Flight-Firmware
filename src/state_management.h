#ifndef STATE_MANAGEMENT_H
#define STATE_MANAGEMENT_H

#include "data_structures.h" // For FlightStateData
// Forward declare FlightState enum if it's defined in TripleT_Flight_Firmware.cpp
// This is a common practice if the full definition isn't needed in the header.
// However, for function signatures using FlightState, its definition must be known.
// Consider moving FlightState enum to data_structures.h or its own header if widely used.
// For now, we'll include TripleT_Flight_Firmware.cpp in state_management.cpp to get the enum.
// This is not ideal but works for this specific structure.
// A better solution is a dedicated header for FlightState.

// We will need the actual FlightState enum for the functions,
// so we will rely on it being defined where these functions are implemented or included.
// The functions in state_management.cpp will need to know the actual FlightState enum.

bool loadStateFromEEPROM();
void saveStateToEEPROM();
void recoverFromPowerLoss();
// Note: getStateName is already planned for utility_functions.cpp

#endif // STATE_MANAGEMENT_H
