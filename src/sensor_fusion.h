// src/sensor_fusion.h
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "data_structures.h" // For FlightState

// Function to select and return the most appropriate acceleration data
void get_fused_acceleration(float* fused_accel, FlightState currentState);

// Function to get the name of the currently active accelerometer
const char* get_active_accelerometer_name();

#endif // SENSOR_FUSION_H 