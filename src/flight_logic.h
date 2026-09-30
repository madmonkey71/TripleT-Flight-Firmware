#ifndef FLIGHT_LOGIC_H
#define FLIGHT_LOGIC_H

#include <Arduino.h> // Include Arduino framework for standard types

// Extern declaration for global dynamic main deployment altitude
extern float g_main_deploy_altitude_m_agl;

// Forward declaration for FlightState enum if its definition is not in a common header yet.
// Assuming FlightState is uint8_t compatible for now.
enum FlightState : uint8_t;

// Function declarations
bool detectApogee();
void resetApogeeDetectionCounters(); // Reset apogee detection counters when entering COAST state
bool flight_is_airborne_state(FlightState s); // BOOST .. MAIN_DESCENT
bool flight_error_allowed(FlightState s);     // ERROR may only be entered from pre-flight states (audit #2)
bool flight_is_provably_on_ground();          // never flew AND (if the baro can tell) near the launch altitude (audit #3)
void flightSetLaunchAltitude(float alt_m);    // set g_launchAltitude and mark it valid as a ground reference
bool flight_is_stationary_on_ground();        // stationary evidence for reset_flight (audit #7)
bool flightIsDegraded();                      // a sensor-health failure is being ridden out in flight
void flightLogicReset();             // Reset all flight-logic bookkeeping (new flight / resume / unit tests)
bool detectLanding();
void detectBoostEnd();
bool IsStable(); // Check if rocket is stable (related to landing) - Kept as it was existing
void ProcessFlightState(); // Main state processing function
// void update_guidance_targets(); // REMOVED as unused

#endif // FLIGHT_LOGIC_H