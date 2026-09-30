// Includes the REAL firmware sources under test (one translation unit) followed
// by the harness helpers that need them. Include after flight_harness.h.
#pragma once
#include "../../src/sensor_samples.cpp"
#include "../../src/pyro_control.cpp"
#include "../../src/state_management.cpp"
#include "../../src/startup_state.cpp"
#include "../../src/flight_logic.cpp"
#include "flight_harness_post.h"
