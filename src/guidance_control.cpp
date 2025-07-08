#include "guidance_control.h"
#include "config.h" // For PID gains and limits
#include <math.h>   // For fabs, if needed for integral windup or other math functions

// Forward declarations for GPS utility functions
static float gps_distance_m(int32_t lat1_e7, int32_t lon1_e7, int32_t lat2_e7, int32_t lon2_e7);
static float gps_bearing_rad(int32_t lat1_e7, int32_t lon1_e7, int32_t lat2_e7, int32_t lon2_e7);

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

// Guidance stability status
static GuidanceStabilityStatus stability_status_g;

// Trajectory data
static Trajectory_t g_current_trajectory;
static PIDControllerState pid_xte_state_g;    // For cross-track error or heading to waypoint
static PIDControllerState pid_alt_traj_state_g; // For altitude control along trajectory


// --- Utility Functions ---
// Helper to convert degrees to radians
static inline float deg_to_rad(float deg) {
    return deg * (M_PI / 180.0f);
}

// Helper to convert radians to degrees
static inline float rad_to_deg(float rad) {
    return rad * (180.0f / M_PI);
}


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

    // Initialize stability status
    guidance_reset_stability_status();

    // Initialize trajectory state
    guidance_reset_trajectory_state();
    pid_xte_state_g.integral = 0.0f;
    pid_xte_state_g.previous_error = 0.0f;
    pid_alt_traj_state_g.integral = 0.0f;
    pid_alt_traj_state_g.previous_error = 0.0f;
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
 * If trajectory following is active, it adjusts target_roll/pitch/yaw_rad_g based on trajectory errors.
 * Then, attitude PIDs run based on these (potentially modified) targets.
 * 
 * @param current_roll_rad Current roll angle in radians.
 * @param current_pitch_rad Current pitch angle in radians.
 * @param current_yaw_rad Current yaw angle in radians (typically -PI to PI, North=0, East=PI/2).
 * @param current_roll_rate_radps Current roll angular velocity in rad/s.
 * @param current_pitch_rate_radps Current pitch angular velocity in rad/s.
 * @param current_yaw_rate_radps Current yaw angular velocity in rad/s.
 * @param current_lat_e7 Current latitude (degrees * 1e7).
 * @param current_lon_e7 Current longitude (degrees * 1e7).
 * @param current_alt_msl Current altitude MSL (meters).
 * @param deltat Time step in seconds since the last update.
 */
void guidance_update(float current_roll_rad, float current_pitch_rad, float current_yaw_rad,
                     float current_roll_rate_radps, float current_pitch_rate_radps, float current_yaw_rate_radps,
                     int32_t current_lat_e7, int32_t current_lon_e7, float current_alt_msl,
                     float deltat) {

    if (deltat <= 0.0f) { return; }

    if (guidance_is_trajectory_active() && g_current_trajectory.current_target_wp_index < g_current_trajectory.num_waypoints) {
        TrajectoryWaypoint_t target_wp = g_current_trajectory.waypoints[g_current_trajectory.current_target_wp_index];

        // 1. Calculate distance and bearing to target waypoint
        float distance_to_wp_m = gps_distance_m(current_lat_e7, current_lon_e7, target_wp.latitude, target_wp.longitude);
        float bearing_to_wp_rad = gps_bearing_rad(current_lat_e7, current_lon_e7, target_wp.latitude, target_wp.longitude);

        // 2. Waypoint Switching Logic
        if (distance_to_wp_m < DEFAULT_WAYPOINT_ACCEPTANCE_RADIUS_M) {
            g_current_trajectory.current_target_wp_index++;
            if (g_current_trajectory.current_target_wp_index >= g_current_trajectory.num_waypoints) {
                // Reached end of trajectory
                guidance_activate_trajectory(false); // Deactivate or switch to hold last WP etc.
                // For now, just deactivate. Target attitude will remain as per last calculation.
            } else {
                // Reset PIDs for the new segment/target
                pid_xte_state_g.integral = 0.0f;
                pid_xte_state_g.previous_error = 0.0f;
                pid_alt_traj_state_g.integral = 0.0f;
                pid_alt_traj_state_g.previous_error = 0.0f;
            }
            // Recalculate for the new waypoint if switched, or exit if trajectory ended
            if (!guidance_is_trajectory_active()) {
                // Fall through to standard attitude PID based on last set targets
            } else {
                 target_wp = g_current_trajectory.waypoints[g_current_trajectory.current_target_wp_index];
                 distance_to_wp_m = gps_distance_m(current_lat_e7, current_lon_e7, target_wp.latitude, target_wp.longitude);
                 bearing_to_wp_rad = gps_bearing_rad(current_lat_e7, current_lon_e7, target_wp.latitude, target_wp.longitude);
            }
        }

        if (guidance_is_trajectory_active()) { // Check again, might have been deactivated by reaching end
            // 3. Calculate Heading Error for Yaw Control
            // Error = desired_bearing - current_yaw. Result is -PI to PI.
            float heading_error_rad = bearing_to_wp_rad - current_yaw_rad;
            // Normalize error to -PI to PI range
            while (heading_error_rad > M_PI) heading_error_rad -= 2.0f * M_PI;
            while (heading_error_rad < -M_PI) heading_error_rad += 2.0f * M_PI;

            // Use XTE PID for heading adjustment (conceptually, not true XTE yet)
            // Output of this PID will be a yaw rate command or direct adjustment to target_yaw_rad_g
            // For simplicity, let's make it an adjustment to the current target_yaw_rad_g
            // This is a rate-of-change to the desired yaw.
            // Alternative: Output of PID is the target_yaw itself. Let's try that.
            // The "current_value" for this PID is current_yaw_rad, target is bearing_to_wp_rad.
            // This is not quite right. The PID should output a correction.
            // Let PID output be a yaw rate command, which then integrates to our target_yaw_rad_g
            // OR, simpler: PID output is the *target heading* for the attitude controller.

            // Let the XTE PID directly calculate the desired target_yaw_rad_g for the attitude controller.
            // The "current value" for this PID is current_yaw_rad, "target value" is bearing_to_wp_rad.
            // The PID output will be the new target_yaw_rad_g for the attitude stabilizer.
            // Need to handle angle wrapping carefully if current and target are far apart.
            // The calculate_pid_output function computes error as target - current.
            // We must ensure that this error is normalized to the range -PI to PI.
            float raw_heading_error = bearing_to_wp_rad - current_yaw_rad;
            while (raw_heading_error > M_PI) raw_heading_error -= 2.0f * M_PI;
            while (raw_heading_error < -M_PI) raw_heading_error += 2.0f * M_PI;

            // The PID's job is to output a desired yaw. Its internal error will be (bearing_to_wp_rad - current_yaw_rad)
            // So, the 'current value' to the PID is current_yaw_rad, 'target value' is bearing_to_wp_rad.
            // However, calculate_pid_output takes (current, target).
            // If target and current are angles that can wrap, their simple difference might be > PI.
            // It's often better to feed the *normalized error* (raw_heading_error) to a PID whose setpoint is 0.

            // PID for heading: Input is current heading error, setpoint is 0. Output is yaw adjustment.
            // Let's adjust the PID to output an *adjustment* to the current yaw, or a yaw *rate*.
            // Simpler: target_yaw_rad_g = PID(current=current_yaw, target=bearing_to_wp_rad)
            // This means the PID output itself is the target_yaw for the inner loop.
            // The error calculation within calculate_pid_output needs to be robust to angle wrapping if target and current are angles.
            // The current calculate_pid_output does: float error = target_value - current_value;
            // This is problematic if target=0.1, current=6.1 (close to 2PI). Error = -6, should be ~0.2.
            // So, for angular PIDs, it's better to calculate normalized error first.
            // Then call PID with (current_error, target_error=0, ...)
            // The XTE PID will output the desired target_yaw_rad_g.

            // Let's assume the `calculate_pid_output` is used for linear values for now,
            // and `target_yaw_rad_g` is directly set to `bearing_to_wp_rad`.
            // The attitude PID for yaw should then handle the shortest way to get to this `target_yaw_rad_g`.
            // This was the previous approach, and it's simpler to start.
            // The `yaw_error_for_pid` calculation for the attitude PID already handles normalization.
            target_yaw_rad_g = bearing_to_wp_rad; // Simple: aim directly at waypoint.

            // If we wanted the XTE PID to smooth this or act as an outer loop:
            // Option: XTE PID output is a yaw *rate* command.
            // float target_yaw_rate_from_xte_pid = calculate_pid_output(
            //     raw_heading_error, // Current error
            //     0.0f,              // Target error (setpoint is zero error)
            //     current_yaw_rate_radps,
            //     &pid_xte_state_g,
            //     TRAJ_XTE_PID_KP, TRAJ_XTE_PID_KI, TRAJ_XTE_PID_KD,
            //     TRAJ_XTE_PID_INTEGRAL_LIMIT,
            //     -TRAJ_XTE_PID_OUTPUT_LIMIT, TRAJ_XTE_PID_OUTPUT_LIMIT,
            //     deltat);
            // target_yaw_rad_g = current_yaw_rad + target_yaw_rate_from_xte_pid * deltat; // Integrate rate to get target angle
            // This makes TRAJ_XTE_PID_OUTPUT_LIMIT a rate limit (rad/s).
            // For now, stick to simpler direct bearing.

            // 4. Calculate Altitude Error for Pitch Control
            float altitude_error_m = target_wp.altitude_msl - current_alt_msl;
            // Use Altitude PID to get a desired pitch adjustment
            float desired_pitch_adjustment_rad = calculate_pid_output(
                altitude_error_m,  // Current error in meters
                0.0f,              // Target error (setpoint is zero altitude error)
                0.0f, // current_pitch_rate_radps, // Rate of change of altitude error could be used for D term if available
                &pid_alt_traj_state_g,
                TRAJ_ALT_PID_KP, TRAJ_ALT_PID_KI, TRAJ_ALT_PID_KD,
                TRAJ_ALT_PID_INTEGRAL_LIMIT,
                -TRAJ_ALT_PID_OUTPUT_LIMIT, TRAJ_ALT_PID_OUTPUT_LIMIT, // Output is a pitch adjustment command
                deltat
            );
            // This PID output is a pitch *adjustment*. Add it to a nominal pitch (e.g. 0 for level)
            // or directly use it as target_pitch_rad_g if scaled appropriately.
            // For simplicity, let this PID output the target pitch directly, capped by its output limit.
            target_pitch_rad_g = desired_pitch_adjustment_rad;


            // For trajectory following, we typically want to fly "wings level" unless making a turn.
            // The yaw control will handle turns. So, target_roll_rad_g should be 0.
            // If we want coordinated turns, XTE PID could output a bank angle (roll target).
            // For now, simple:
            target_roll_rad_g = 0.0f;
        }
    }
    // Else (trajectory not active), target_roll/pitch/yaw_rad_g remain as set by external calls
    // like guidance_set_target_orientation_euler() for simple attitude hold.

    // --- Attitude PID Controllers (Always Run) ---
    // These use target_roll_rad_g, target_pitch_rad_g, target_yaw_rad_g,
    // which may have been updated by trajectory logic above, or set by attitude hold commands.

    // Roll PID Controller
    actuator_output_y_g = calculate_pid_output(current_roll_rad, target_roll_rad_g,
                                               current_roll_rate_radps,
                                               &pid_roll_state_g,
                                               PID_ROLL_KP, PID_ROLL_KI, PID_ROLL_KD,
                                               PID_INTEGRAL_LIMIT_ROLL, PID_OUTPUT_MIN, PID_OUTPUT_MAX,
                                               deltat);

    // Pitch PID Controller
    actuator_output_x_g = calculate_pid_output(current_pitch_rad, target_pitch_rad_g,
                                               current_pitch_rate_radps,
                                               &pid_pitch_state_g,
                                               PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD,
                                               PID_INTEGRAL_LIMIT_PITCH, PID_OUTPUT_MIN, PID_OUTPUT_MAX,
                                               deltat);

    // Yaw PID Controller
    // Normalize yaw error for PID: target_yaw_rad_g can be any angle, current_yaw_rad is -PI to PI.
    float yaw_error_for_pid = target_yaw_rad_g - current_yaw_rad;
    while (yaw_error_for_pid > M_PI) yaw_error_for_pid -= 2.0f * M_PI;
    while (yaw_error_for_pid < -M_PI) yaw_error_for_pid += 2.0f * M_PI;
    // The calculate_pid_output function expects (current, target, ...), so error is target - current.
    // We pass current_yaw_rad and target_yaw_rad_g. The helper function calculates error.
    // However, the helper's simple `error = target - current` might not handle wrap-around for yaw correctly if target is far from current.
    // Better to calculate normalized error here and pass it to a modified PID function or adjust.
    // For now, relying on the existing calculate_pid_output which takes current and target.
    // The `target_yaw_rad_g` (bearing) and `current_yaw_rad` are both normalized angles (-PI to PI or 0 to 2PI),
    // so simple subtraction should be okay if their ranges are consistent. Let's assume `gps_bearing_rad`
    // and `current_yaw_rad` are both consistently within -PI to PI.
    actuator_output_z_g = calculate_pid_output(current_yaw_rad, target_yaw_rad_g, // Error = target_yaw_rad_g - current_yaw_rad
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


// --- Stability Monitoring Functions ---

void guidance_reset_stability_status() {
    stability_status_g.pitch_rate_violation_start_ms = 0;
    stability_status_g.roll_rate_violation_start_ms = 0;
    stability_status_g.yaw_rate_violation_start_ms = 0;

    stability_status_g.pitch_attitude_error_violation_start_ms = 0;
    stability_status_g.roll_attitude_error_violation_start_ms = 0;
    stability_status_g.yaw_attitude_error_violation_start_ms = 0;

    stability_status_g.pitch_saturation_violation_start_ms = 0;
    stability_status_g.yaw_saturation_violation_start_ms = 0;
    stability_status_g.roll_saturation_violation_start_ms = 0;

    stability_status_g.stability_compromised = false;
}

bool guidance_is_stability_compromised() {
    return stability_status_g.stability_compromised;
}

// Helper function for checking individual stability criteria
static void check_criterion(float current_value, float threshold, unsigned long current_time_ms,
                            unsigned long& violation_start_ms, bool& is_compromised_flag) {
    if (fabs(current_value) > threshold) {
        if (violation_start_ms == 0) { // First time exceeding threshold
            violation_start_ms = current_time_ms;
        } else { // Already exceeding, check duration
            if (current_time_ms - violation_start_ms >= STABILITY_VIOLATION_DURATION_MS) {
                is_compromised_flag = true;
            }
        }
    } else { // Value is within threshold
        violation_start_ms = 0; // Reset timer
    }
}

void guidance_check_stability(float current_roll_rad, float current_pitch_rad, float current_yaw_rad,
                              float current_roll_rate_radps, float current_pitch_rate_radps, float current_yaw_rate_radps,
                              float actuator_cmd_pitch, float actuator_cmd_yaw, float actuator_cmd_roll, // These are normalized PID outputs (-1 to 1)
                              unsigned long current_time_ms) {

    if (stability_status_g.stability_compromised) {
        return; // Once compromised, stays compromised until reset
    }

    // 1. Angular Rate Checks (convert thresholds from DPS to Rad/s)
    check_criterion(current_pitch_rate_radps, deg_to_rad(STABILITY_MAX_PITCH_RATE_DPS), current_time_ms,
                    stability_status_g.pitch_rate_violation_start_ms, stability_status_g.stability_compromised);
    if (stability_status_g.stability_compromised) return;

    check_criterion(current_roll_rate_radps, deg_to_rad(STABILITY_MAX_ROLL_RATE_DPS), current_time_ms,
                    stability_status_g.roll_rate_violation_start_ms, stability_status_g.stability_compromised);
    if (stability_status_g.stability_compromised) return;

    check_criterion(current_yaw_rate_radps, deg_to_rad(STABILITY_MAX_YAW_RATE_DPS), current_time_ms,
                    stability_status_g.yaw_rate_violation_start_ms, stability_status_g.stability_compromised);
    if (stability_status_g.stability_compromised) return;

    // 2. Attitude Error Checks (ensure target orientation is meaningful, i.e., guidance is active)
    // Error = target - current. Target angles are global (target_roll_rad_g, etc.)
    // This check is only meaningful if guidance is actively trying to hold a target.
    // We assume if this function is called, guidance is active.
    float pitch_error_rad = target_pitch_rad_g - current_pitch_rad;
    // Normalize yaw error (handle wrap-around for angles -pi to pi)
    float yaw_error_rad = target_yaw_rad_g - current_yaw_rad;
    while (yaw_error_rad > M_PI) yaw_error_rad -= 2.0f * M_PI;
    while (yaw_error_rad < -M_PI) yaw_error_rad += 2.0f * M_PI;
    float roll_error_rad = target_roll_rad_g - current_roll_rad;
     while (roll_error_rad > M_PI) roll_error_rad -= 2.0f * M_PI;
    while (roll_error_rad < -M_PI) roll_error_rad += 2.0f * M_PI;


    check_criterion(pitch_error_rad, deg_to_rad(STABILITY_MAX_ATTITUDE_ERROR_PITCH_DEG), current_time_ms,
                    stability_status_g.pitch_attitude_error_violation_start_ms, stability_status_g.stability_compromised);
    if (stability_status_g.stability_compromised) return;

    check_criterion(roll_error_rad, deg_to_rad(STABILITY_MAX_ATTITUDE_ERROR_ROLL_DEG), current_time_ms,
                    stability_status_g.roll_attitude_error_violation_start_ms, stability_status_g.stability_compromised);
    if (stability_status_g.stability_compromised) return;

    check_criterion(yaw_error_rad, deg_to_rad(STABILITY_MAX_ATTITUDE_ERROR_YAW_DEG), current_time_ms,
                    stability_status_g.yaw_attitude_error_violation_start_ms, stability_status_g.stability_compromised);
    if (stability_status_g.stability_compromised) return;

    // 3. Actuator Saturation Checks
    // Actuator commands are normalized (-1.0 to 1.0 by PID_OUTPUT_MIN/MAX).
    // STABILITY_ACTUATOR_SATURATION_LEVEL_PERCENT is 0-100. Convert to 0-1.0.
    float saturation_threshold_normalized = STABILITY_ACTUATOR_SATURATION_LEVEL_PERCENT / 100.0f;

    // Check if absolute command value exceeds threshold (e.g. abs(cmd) > 0.9 if threshold is 90%)
    check_criterion(actuator_cmd_pitch, saturation_threshold_normalized, current_time_ms, // Using actuator_cmd_pitch directly as it's already normalized
                    stability_status_g.pitch_saturation_violation_start_ms, stability_status_g.stability_compromised);
    if (stability_status_g.stability_compromised) return;

    check_criterion(actuator_cmd_yaw, saturation_threshold_normalized, current_time_ms,
                    stability_status_g.yaw_saturation_violation_start_ms, stability_status_g.stability_compromised);
    if (stability_status_g.stability_compromised) return;

    check_criterion(actuator_cmd_roll, saturation_threshold_normalized, current_time_ms,
                    stability_status_g.roll_saturation_violation_start_ms, stability_status_g.stability_compromised);
    // No return here, it's the last check
}


// --- Trajectory Following Functions ---

void guidance_reset_trajectory_state() {
    g_current_trajectory.num_waypoints = 0;
    g_current_trajectory.current_target_wp_index = 0;
    g_current_trajectory.is_active = false;
    g_current_trajectory.is_loaded = false;
    // Optionally clear the waypoints array if needed
    // for (int i = 0; i < MAX_TRAJECTORY_WAYPOINTS; ++i) {
    //   g_current_trajectory.waypoints[i] = {0,0,0.0f};
    // }
    pid_xte_state_g.integral = 0.0f;
    pid_xte_state_g.previous_error = 0.0f;
    pid_alt_traj_state_g.integral = 0.0f;
    pid_alt_traj_state_g.previous_error = 0.0f;
}

// Basic test trajectory for development
void guidance_load_test_trajectory() {
    guidance_reset_trajectory_state(); // Clear any existing trajectory

    // Example: A simple two-point trajectory
    // IMPORTANT: These lat/lon are placeholders. Real values needed for testing.
    // These should be somewhat close to a test launch site.
    // Lat/Lon are in degrees * 1e7
    // Altitude MSL in meters

    // Waypoint 1 (e.g., 100m North, 50m East, 200m MSL Alt from a hypothetical origin)
    g_current_trajectory.waypoints[0].latitude = 340000000; // Approx. 34.0 deg
    g_current_trajectory.waypoints[0].longitude = -1180000000; // Approx -118.0 deg
    g_current_trajectory.waypoints[0].altitude_msl = 200.0f;

    // Waypoint 2 (e.g., 200m North, 150m East, 250m MSL Alt)
    g_current_trajectory.waypoints[1].latitude = 340001000; // Slightly more north
    g_current_trajectory.waypoints[1].longitude = -117999000; // Slightly more east
    g_current_trajectory.waypoints[1].altitude_msl = 250.0f;

    g_current_trajectory.num_waypoints = 2;
    g_current_trajectory.is_loaded = true;
    g_current_trajectory.current_target_wp_index = 0;

    // Serial.println(F("Loaded test trajectory with 2 waypoints."));
}

void guidance_activate_trajectory(bool activate) {
    if (activate && !g_current_trajectory.is_loaded) {
        // Serial.println(F("Cannot activate trajectory: No trajectory loaded."));
        g_current_trajectory.is_active = false;
        return;
    }
    if (activate && g_current_trajectory.num_waypoints == 0) {
        // Serial.println(F("Cannot activate trajectory: Trajectory has 0 waypoints."));
        g_current_trajectory.is_active = false;
        return;
    }
    g_current_trajectory.is_active = activate;
    if (activate) {
        g_current_trajectory.current_target_wp_index = 0; // Start from the beginning when activated
        pid_xte_state_g.integral = 0.0f; // Reset PIDs for trajectory control
        pid_xte_state_g.previous_error = 0.0f;
        pid_alt_traj_state_g.integral = 0.0f;
        pid_alt_traj_state_g.previous_error = 0.0f;
        // Serial.print(F("Trajectory guidance activated. Target WP index: "));
        // Serial.println(g_current_trajectory.current_target_wp_index);
    } else {
        // Serial.println(F("Trajectory guidance deactivated."));
        // When deactivating, we might want to revert to simple attitude hold
        // The main guidance_update will handle this by falling through to attitude PID
    }
}

bool guidance_is_trajectory_active() {
    return g_current_trajectory.is_active && g_current_trajectory.is_loaded && g_current_trajectory.num_waypoints > 0;
}

bool guidance_is_trajectory_loaded() {
    return g_current_trajectory.is_loaded && g_current_trajectory.num_waypoints > 0;
}

uint8_t guidance_get_current_trajectory_target_wp_index() {
    return g_current_trajectory.current_target_wp_index;
}

uint8_t guidance_get_trajectory_num_waypoints() {
    return g_current_trajectory.num_waypoints;
}


// --- Placeholder GPS Math Helper Functions ---
// These need proper implementation based on Haversine or other suitable formulas.

/**
 * @brief Calculates distance in meters between two GPS coordinates.
 * @param lat1_e7 Latitude of point 1 (degrees * 1e7)
 * @param lon1_e7 Longitude of point 1 (degrees * 1e7)
 * @param lat2_e7 Latitude of point 2 (degrees * 1e7)
 * @param lon2_e7 Longitude of point 2 (degrees * 1e7)
 * @return Distance in meters.
 */
static float gps_distance_m(int32_t lat1_e7, int32_t lon1_e7, int32_t lat2_e7, int32_t lon2_e7) {
    // IMPORTANT: Placeholder - Replace with actual Haversine formula
    // Convert to radians for calculation
    float lat1_rad = deg_to_rad(lat1_e7 / 1e7f);
    float lon1_rad = deg_to_rad(lon1_e7 / 1e7f);
    float lat2_rad = deg_to_rad(lat2_e7 / 1e7f);
    float lon2_rad = deg_to_rad(lon2_e7 / 1e7f);

    float dlat = lat2_rad - lat1_rad;
    float dlon = lon2_rad - lon1_rad;

    float a = sinf(dlat / 2.0f) * sinf(dlat / 2.0f) +
              cosf(lat1_rad) * cosf(lat2_rad) *
              sinf(dlon / 2.0f) * sinf(dlon / 2.0f);
    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
    const float EARTH_RADIUS_M = 6371000.0f;
    return EARTH_RADIUS_M * c;
}

/**
 * @brief Calculates bearing in radians from point 1 to point 2.
 * Bearing is East of North (0 rad = North, PI/2 rad = East).
 * @param lat1_e7 Latitude of point 1 (degrees * 1e7)
 * @param lon1_e7 Longitude of point 1 (degrees * 1e7)
 * @param lat2_e7 Latitude of point 2 (degrees * 1e7)
 * @param lon2_e7 Longitude of point 2 (degrees * 1e7)
 * @return Bearing in radians.
 */
static float gps_bearing_rad(int32_t lat1_e7, int32_t lon1_e7, int32_t lat2_e7, int32_t lon2_e7) {
    // IMPORTANT: Placeholder - Replace with actual bearing calculation
    float lat1_rad = deg_to_rad(lat1_e7 / 1e7f);
    float lon1_rad = deg_to_rad(lon1_e7 / 1e7f);
    float lat2_rad = deg_to_rad(lat2_e7 / 1e7f);
    float lon2_rad = deg_to_rad(lon2_e7 / 1e7f);

    float dlon = lon2_rad - lon1_rad;
    float y = sinf(dlon) * cosf(lat2_rad);
    float x = cosf(lat1_rad) * sinf(lat2_rad) -
              sinf(lat1_rad) * cosf(lat2_rad) * cosf(dlon);
    float bearing = atan2f(y, x);
    // atan2f returns -PI to PI. Normalize to 0 to 2PI if needed, or ensure usage is consistent.
    // For yaw target, -PI to PI is fine if current_yaw is also in that range.
    return bearing;
}

// Function to calculate cross-track error (placeholder)
// static float calculate_crosstrack_error_m(int32_t cur_lat_e7, int32_t cur_lon_e7,
//                                          TrajectoryWaypoint_t wp1, TrajectoryWaypoint_t wp2) {
//     // Placeholder
//     return 0.0f;
// }
