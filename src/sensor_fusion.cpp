#include "sensor_fusion.h"
#include "kx134_functions.h"      // For kx134_get_accel()
#include "icm_20948_functions.h"  // For icm_20948_get_accel()
#include "config.h"               // For USE_KX134

// External declaration of sensor data arrays from the main firmware file
extern float kx134_accel[3];
extern float icm_accel[3];
extern bool kx134_initialized_ok;
extern bool icm20948_ready;

// Variable to hold the name of the currently active accelerometer
static const char* active_accelerometer = "ICM-20948";

/**
 * @brief Selects the best accelerometer based on the current flight state and returns its data.
 *
 * This function implements the core sensor fusion logic for acceleration data.
 * - During high-G states (BOOST), it prioritizes the KX134 High-G accelerometer.
 * - During all other states, it uses the more sensitive ICM-20948.
 * It includes fallback logic in case a preferred sensor is not available.
 *
 * @param[out] fused_accel A float array of size 3 to store the selected X, Y, Z acceleration.
 * @param[in] currentState The current FlightState of the rocket.
 */
void get_fused_acceleration(float* fused_accel, FlightState currentState) {
    bool use_kx134 = false;

    // 1. Determine which accelerometer to use based on flight state
    if (currentState == BOOST && USE_KX134 && kx134_initialized_ok) {
        // During boost, the high-g KX134 is the preferred sensor
        use_kx134 = true;
    }

    // 2. Retrieve data from the selected accelerometer with fallback logic
    if (use_kx134) {
        // Preferred sensor for this state is KX134
        kx134_get_accel(fused_accel);
        active_accelerometer = "KX134";
    } else {
        // Default to ICM-20948 for all other states or if KX134 is not available
        if (icm20948_ready) {
            icm_20948_get_accel(fused_accel);
            active_accelerometer = "ICM-20948";
        } else {
            // Fallback if ICM is also not ready (should not happen in normal operation)
            fused_accel[0] = 0.0f;
            fused_accel[1] = 0.0f;
            fused_accel[2] = 0.0f;
            active_accelerometer = "None";
        }
    }
}

/**
 * @brief Returns the name of the accelerometer currently being used by the fusion logic.
 *
 * @return A const char pointer to the name ("KX134", "ICM-20948", or "None").
 */
const char* get_active_accelerometer_name() {
    return active_accelerometer;
} 