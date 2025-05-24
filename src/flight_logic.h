#pragma once

#include <Arduino.h> // Include Arduino framework for standard types

// Forward declarations for flight state detection functions
bool detectApogee();
bool detectLanding();
void detectBoostEnd();
bool IsStable(); // Check if rocket is stable (related to landing)

void update_guidance_targets();