#ifndef GUIDANCE_CONTROL_H
#define GUIDANCE_CONTROL_H

// Structure to hold the state of a single PID controller
typedef struct {
    float integral;         // Sum of errors for the I-term
    float previous_error;   // Last error for the D-term
} PIDControllerState;

// Structure to hold stability monitoring status and timers
typedef struct {
    // Angular rate violation timers
    unsigned long pitch_rate_violation_start_ms;
    unsigned long roll_rate_violation_start_ms;
    unsigned long yaw_rate_violation_start_ms;

    // Attitude error violation timers
    unsigned long pitch_attitude_error_violation_start_ms;
    unsigned long roll_attitude_error_violation_start_ms;
    unsigned long yaw_attitude_error_violation_start_ms;

    // Actuator saturation timers
    unsigned long pitch_saturation_violation_start_ms;
    unsigned long yaw_saturation_violation_start_ms;
    unsigned long roll_saturation_violation_start_ms; // If roll actuators exist

    bool stability_compromised; // Flag indicating if a stability limit has been breached
} GuidanceStabilityStatus;


// --- Trajectory Following Declarations ---
// Forward declare Trajectory_t if its definition is in data_structures.h and that's included after this.
// However, it's better to include data_structures.h first if it's small and widely used.
// Assuming data_structures.h (which includes config.h for MAX_TRAJECTORY_WAYPOINTS)
// will be included before guidance_control.h in the .cpp files, or include it here.
#include "data_structures.h" // Contains Trajectory_t and TrajectoryWaypoint_t

// Function Declarations for Trajectory Control
void guidance_load_test_trajectory(); // Loads a hardcoded trajectory for testing
// void guidance_load_trajectory_from_file(const char* filename); // Future: load from SD
void guidance_activate_trajectory(bool activate);
bool guidance_is_trajectory_active();
bool guidance_is_trajectory_loaded();
void guidance_reset_trajectory_state();
uint8_t guidance_get_current_trajectory_target_wp_index();
uint8_t guidance_get_trajectory_num_waypoints();


// --- General Guidance Function Declarations ---

/**
 * @brief Initializes the guidance system and PID controllers.
 * Loads gains from config.h (implicitly, as they will be used in .cpp).
 * Resets PID states.
 */
void guidance_init();

/**
 * @brief Sets the target orientation for the guidance system using Euler angles.
 * @param target_roll_rad Target roll angle in radians.
 * @param target_pitch_rad Target pitch angle in radians.
 * @param target_yaw_rad Target yaw angle in radians (relative to current, or absolute if MARG is working).
 */
void guidance_set_target_orientation_euler(float target_roll_rad, float target_pitch_rad, float target_yaw_rad);

// Alternative: Set target using Quaternions (can be added later if preferred)
// void guidance_set_target_orientation_quaternion(float q0, float q1, float q2, float q3);

/**
 * @brief Updates the PID controllers with current orientation and rates.
 * Calculates new actuator commands based on the error from the target orientation.
 * 
 * @param current_roll_rad Current roll angle in radians.
 * @param current_pitch_rad Current pitch angle in radians.
 * @param current_yaw_rad Current yaw angle in radians.
 * @param current_roll_rate_radps Current roll angular velocity in rad/s.
 * @param current_pitch_rate_radps Current pitch angular velocity in rad/s.
 * @param current_yaw_rate_radps Current yaw angular velocity in rad/s.
 * @param current_lat_e7 Current latitude (degrees * 1e7) from GPS.
 * @param current_lon_e7 Current longitude (degrees * 1e7) from GPS.
 * @param current_alt_msl Current altitude above Mean Sea Level (meters) from GPS/Baro.
 * @param deltat Time step in seconds since the last update.
 */
void guidance_update(float current_roll_rad, float current_pitch_rad, float current_yaw_rad,
                     float current_roll_rate_radps, float current_pitch_rate_radps, float current_yaw_rate_radps,
                     int32_t current_lat_e7, int32_t current_lon_e7, float current_alt_msl,
                     float deltat);

/**
 * @brief Retrieves the calculated actuator outputs.
 * Outputs are typically normalized (e.g., -1.0 to 1.0).
 * 
 * @param-out output_x Command for X-axis actuator (e.g., pitch servo).
 * @param-out output_y Command for Y-axis actuator (e.g., roll servo).
 * @param-out output_z Command for Z-axis actuator (e.g., yaw reaction wheel/thrust).
 */
void guidance_get_actuator_outputs(float& output_x, float& output_y, float& output_z);

/**
 * @brief Retrieves the current target Euler angles.
 * @param-out out_target_roll_rad Target roll angle in radians.
 * @param-out out_target_pitch_rad Target pitch angle in radians.
 * @param-out out_target_yaw_rad Target yaw angle in radians.
 */
void guidance_get_target_euler_angles(float& out_target_roll_rad, float& out_target_pitch_rad, float& out_target_yaw_rad);

/**
 * @brief Retrieves the current PID integral values for each axis.
 * @param-out out_integral_roll PID integral for roll axis.
 * @param-out out_integral_pitch PID integral for pitch axis.
 * @param-out out_integral_yaw PID integral for yaw axis.
 */
void guidance_get_pid_integrals(float& out_integral_roll, float& out_integral_pitch, float& out_integral_yaw);

/**
 * @brief Checks the current flight parameters against stability thresholds.
 * Updates the internal stability status. This function should be called periodically
 * when guidance is active.
 *
 * @param current_roll_rad Current roll angle in radians.
 * @param current_pitch_rad Current pitch angle in radians.
 * @param current_yaw_rad Current yaw angle in radians.
 * @param current_roll_rate_radps Current roll angular velocity in rad/s.
 * @param current_pitch_rate_radps Current pitch angular velocity in rad/s.
 * @param current_yaw_rate_radps Current yaw angular velocity in rad/s.
 * @param actuator_cmd_pitch Commanded pitch actuator output (normalized, before mapping to servo).
 * @param actuator_cmd_yaw Commanded yaw actuator output (normalized).
 * @param actuator_cmd_roll Commanded roll actuator output (normalized).
 * @param current_time_ms Current system time in milliseconds.
 */
void guidance_check_stability(float current_roll_rad, float current_pitch_rad, float current_yaw_rad,
                              float current_roll_rate_radps, float current_pitch_rate_radps, float current_yaw_rate_radps,
                              float actuator_cmd_pitch, float actuator_cmd_yaw, float actuator_cmd_roll,
                              unsigned long current_time_ms);

/**
 * @brief Returns the stability status of the guidance system.
 * @return True if stability is compromised, false otherwise.
 */
bool guidance_is_stability_compromised();

/**
 * @brief Resets the stability compromised flag and timers.
 * Call this when transitioning to a state where stability monitoring should restart,
 * or after a failsafe has been handled.
 */
void guidance_reset_stability_status();


#endif // GUIDANCE_CONTROL_H
