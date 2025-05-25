#pragma once

#include <Arduino.h> // Include Arduino framework for standard types

// FlightState Enum Definition
enum FlightState {
  STARTUP, CALIBRATION, PAD_IDLE, ARMED, BOOST, COAST, APOGEE,
  DROGUE_DEPLOY, DROGUE_DESCENT, MAIN_DEPLOY, MAIN_DESCENT,
  LANDED, RECOVERY, ERROR_STATE // Renamed ERROR to ERROR_STATE to avoid conflict
};

// Global Current Flight State Variable
extern FlightState currentFlightState;

// State Machine Related Variables
extern float launchAltitude; // Altitude at launch, set during BOOST
extern float maxAltitudeReached; // Max altitude recorded, for reference

// Core State Processing Function and Helper Functions
void initialize_flight_state_machine(); // New function to set initial state
void process_flight_state();
const char* get_flight_state_name(FlightState state); // Renamed from getStateName

// Helper detection functions (declarations, implementations will be moved/adapted later)
bool detectApogee(); // Already declared
bool detectLanding(); // Already declared
void detectBoostEnd(); // Already declared
bool IsStable(); // Check if rocket is stable (related to landing)
float get_current_baro_altitude(); // Wrapper/adapter for ms5611_get_altitude
float get_current_accel_magnitude(); // Wrapper/adapter for get_accel_magnitude

void update_guidance_targets(); // Existing function

// Existing global flight state variables (can be reviewed/removed later if managed by state machine)
extern unsigned long boostEndTime;
extern bool landingDetectedFlag;