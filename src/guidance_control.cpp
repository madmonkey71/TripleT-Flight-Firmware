#include "guidance_control.h"
#include "config.h" // For PID gains and limits
#include <math.h>   // For fabs, if needed for integral windup or other math functions

// --- Static Global Variables for Guidance Control ---

// Target orientation (desired state)
static float target_roll_rad_g = 0.0f;
static float target_pitch_rad_g = 0.0f;
static float target_yaw_rad_g = 0.0f; 
// Note: For yaw, this might be a target relative change or an absolute heading.
// For now, assume it's a target angle like roll and pitch.

// PID controller states for each axis
static PIDControllerState pid_roll_state_g;
static PIDControllerState pid_pitch_state_g;
static PIDControllerState pid_yaw_state_g;

// Calculated actuator outputs
static float actuator_output_x_g = 0.0f; // E.g., controls pitch
static float actuator_output_y_g = 0.0f; // E.g., controls roll
static float actuator_output_z_g = 0.0f; // E.g., controls yaw


// --- Function Implementations ---

// Helper function for single-axis PID calculation
static float calculate_pid_output(float current_value, float target_value,
                                  float current_rate, // For D-term on rate, or for feedforward (not used in basic P,I,D_on_error)
                                  PIDControllerState* pid_state,
                                  float Kp, float Ki, float Kd,
                                  float integral_limit, float output_min, float output_max,
                                  float deltat) {
    
    float error = target_value - current_value;

    // Proportional term
    float P_term = Kp * error;

    // Integral term with anti-windup
    pid_state->integral += error * deltat;
    // Clamp integral
    if (pid_state->integral > integral_limit) {
        pid_state->integral = integral_limit;
    } else if (pid_state->integral < -integral_limit) {
        pid_state->integral = -integral_limit;
    }
    float I_term = Ki * pid_state->integral;

    // Derivative term
    // Using derivative of error.
    float derivative = (deltat > 0.0f) ? ((error - pid_state->previous_error) / deltat) : 0.0f;
    float D_term = Kd * derivative;
    
    // Update previous error for next iteration
    pid_state->previous_error = error;

    // Total PID output
    float output = P_term + I_term + D_term;

    // Clamp output
    if (output > output_max) {
        output = output_max;
    } else if (output < output_min) {
        output = output_min;
    }

    return output;
}


/**
 * @brief Initializes the guidance system and PID controllers.
 * Loads gains from config.h (implicitly, as they will be used in .cpp).
 * Resets PID states.
 */
void guidance_init() {
    // Reset PID states
    pid_roll_state_g.integral = 0.0f;
    pid_roll_state_g.previous_error = 0.0f;

    pid_pitch_state_g.integral = 0.0f;
    pid_pitch_state_g.previous_error = 0.0f;

    pid_yaw_state_g.integral = 0.0f;
    pid_yaw_state_g.previous_error = 0.0f;

    // Reset target orientation to zero (e.g., level flight, current heading)
    target_roll_rad_g = 0.0f;
    target_pitch_rad_g = 0.0f;
    target_yaw_rad_g = 0.0f; // Or could be set to current yaw if that's desired on init

    // Reset actuator outputs
    actuator_output_x_g = 0.0f;
    actuator_output_y_g = 0.0f;
    actuator_output_z_g = 0.0f;

    // PID gains are #defined in config.h, so no loading step needed here,
    // they are directly used in guidance_update().
}

/**
 * @brief Sets the target orientation for the guidance system using Euler angles.
 * @param target_roll_rad Target roll angle in radians.
 * @param target_pitch_rad Target pitch angle in radians.
 * @param target_yaw_rad Target yaw angle in radians (relative to current, or absolute if MARG is working).
 */
void guidance_set_target_orientation_euler(float tr_rad, float tp_rad, float ty_rad) {
    target_roll_rad_g = tr_rad;
    target_pitch_rad_g = tp_rad;
    target_yaw_rad_g = ty_rad;
}

/**
 * @brief Retrieves the calculated actuator outputs.
 * Outputs are typically normalized (e.g., -1.0 to 1.0).
 * 
 * @param-out output_x Command for X-axis actuator (e.g., pitch servo).
 * @param-out output_y Command for Y-axis actuator (e.g., roll servo).
 * @param-out output_z Command for Z-axis actuator (e.g., yaw reaction wheel/thrust).
 */
void guidance_get_actuator_outputs(float& out_x, float& out_y, float& out_z) {
    out_x = actuator_output_x_g;
    out_y = actuator_output_y_g;
    out_z = actuator_output_z_g;
}

// Note: The core guidance_update() function, which will contain the PID calculations,
// is not implemented in this sub-task. It will be implemented subsequently.

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
                     float deltat) {

    if (deltat <= 0.0f) { // Safety check for deltat
        // Optionally log an error or handle appropriately
        return; 
    }

    // Roll PID Controller
    // Assuming actuator_output_y_g controls roll (adjust if mapping is different)
    actuator_output_y_g = calculate_pid_output(current_roll_rad, target_roll_rad_g,
                                               current_roll_rate_radps,
                                               &pid_roll_state_g,
                                               PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD,
                                               PID_INTEGRAL_LIMIT_ROLL, PID_OUTPUT_MIN, PID_OUTPUT_MAX,
                                               deltat);

    // Pitch PID Controller
    // Assuming actuator_output_x_g controls pitch
    actuator_output_x_g = calculate_pid_output(current_pitch_rad, target_pitch_rad_g,
                                               current_pitch_rate_radps,
                                               &pid_pitch_state_g,
                                               PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD,
                                               PID_INTEGRAL_LIMIT_PITCH, PID_OUTPUT_MIN, PID_OUTPUT_MAX,
                                               deltat);

    // Yaw PID Controller
    // Assuming actuator_output_z_g controls yaw
    // Note: Yaw error calculation might need refinement for angle wrapping if absolute heading control is used.
    // The helper calculate_pid_output currently uses a simple error = target - current.
    actuator_output_z_g = calculate_pid_output(current_yaw_rad, target_yaw_rad_g,
                                               current_yaw_rate_radps,
                                               &pid_yaw_state_g,
                                               PID_YAW_KP, PID_YAW_KI, PID_YAW_KD,
                                               PID_INTEGRAL_LIMIT_YAW, PID_OUTPUT_MIN, PID_OUTPUT_MAX,
                                               deltat);
}

void guidance_get_target_euler_angles(float& out_target_roll_rad, float& out_target_pitch_rad, float& out_target_yaw_rad) {
    out_target_roll_rad = target_roll_rad_g;
    out_target_pitch_rad = target_pitch_rad_g;
    out_target_yaw_rad = target_yaw_rad_g;
}

void guidance_get_pid_integrals(float& out_integral_roll, float& out_integral_pitch, float& out_integral_yaw) {
    out_integral_roll = pid_roll_state_g.integral;
    out_integral_pitch = pid_pitch_state_g.integral;
    out_integral_yaw = pid_yaw_state_g.integral;
}
