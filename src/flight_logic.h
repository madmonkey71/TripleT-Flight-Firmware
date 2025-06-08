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
bool detectLanding();
void detectBoostEnd();
bool IsStable(); // Check if rocket is stable (related to landing) - Kept as it was existing
void ProcessFlightState(); // Main state processing function
void update_guidance_targets(); // Kept as it was existing

#endif // FLIGHT_LOGIC_H