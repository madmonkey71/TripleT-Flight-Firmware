#ifndef GUIDANCE_CONTROL_H
#define GUIDANCE_CONTROL_H

// Structure to hold the state of a single PID controller
typedef struct {
    float integral;         // Sum of errors for the I-term
    float previous_error;   // Last error for the D-term
} PIDControllerState;

// Function Declarations

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
 * @param deltat Time step in seconds since the last update.
 */
void guidance_update(float current_roll_rad, float current_pitch_rad, float current_yaw_rad,
                     float current_roll_rate_radps, float current_pitch_rate_radps, float current_yaw_rate_radps,
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

#endif // GUIDANCE_CONTROL_H
