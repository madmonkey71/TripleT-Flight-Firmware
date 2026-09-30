#include "kalman_filter.h"
#include <math.h> // For atan2, sqrt, etc.
#include "config.h" // KALMAN_ACCEL_GATE_* (audit #9)

// --- Internal State Variables ---
// Simplified state: roll, pitch, yaw. A more complete filter would also estimate gyro biases.
static float kf_roll = 0.0f;
static float kf_pitch = 0.0f;
static float kf_yaw = 0.0f;

// Error covariance matrix P (3x3). Represents the uncertainty in the state estimate.
// P = [[P_roll_roll, P_roll_pitch, P_roll_yaw],
//      [P_pitch_roll, P_pitch_pitch, P_pitch_yaw],
//      [P_yaw_roll, P_yaw_pitch, P_yaw_yaw]]
// For simplicity, we'll only use the diagonal elements for now.
static float P_diag[3] = {1.0f, 1.0f, 1.0f}; // Initial uncertainty

// Process noise covariance matrix Q (3x3). Represents the uncertainty of the process model.
// Higher values mean the filter trusts the gyro less.
// For simplicity, using diagonal Q.
static float Q_angle[3] = {0.001f, 0.001f, 0.001f}; // Roll, Pitch, Yaw process noise

// Measurement noise covariance matrix R (2x2 for accel-based roll/pitch).
// Represents the uncertainty of the measurements. Higher values mean the filter trusts the accelerometer less.
// R_accel = [[R_accel_roll, 0], [0, R_accel_pitch]]
static float R_accel[2] = {0.03f, 0.03f}; // Measurement noise for roll and pitch from accelerometer

// Measurement noise covariance matrix R (2x2 for magnetometer-based yaw).
// Represents the uncertainty of the measurements. Higher values mean the filter trusts the magnetometer less.
// R_mag = [[R_mag_x, 0], [0, R_mag_y]]
static float R_mag = 0.03f; // Measurement noise for magnetometer-based yaw

// Gate bookkeeping (audit #9)
static float kf_last_gyro_mag = 0.0f;            // |gyro| (rad/s) seen by the most recent kalman_predict()
static unsigned long kf_accel_skipped = 0;       // accelerometer updates rejected by the gate

// --- Kalman Filter Functions ---

void kalman_init(float initial_roll, float initial_pitch, float initial_yaw) {
    kf_roll = initial_roll;
    kf_pitch = initial_pitch;
    kf_yaw = initial_yaw;

    // Initialize covariance matrix P with initial uncertainties
    // Larger values mean more uncertainty in the initial state.
    P_diag[0] = 1.0f; // Uncertainty in roll
    P_diag[1] = 1.0f; // Uncertainty in pitch
    P_diag[2] = 1.0f; // Uncertainty in yaw
    kf_last_gyro_mag = 0.0f;
    kf_accel_skipped = 0;

    // Q and R are set to default values above, but could also be initialized here if needed.
}

void kalman_predict(float gyro_x, float gyro_y, float gyro_z, float dt) {
    // --- Predict next state (simplified Euler integration) ---
    // This is a basic integration. A more advanced filter might use quaternions
    // or a more sophisticated integration scheme.

    // Update roll, pitch, yaw based on gyro inputs
    // Note: This simple model assumes gyro inputs are directly angular rates of Euler angles,
    // which is not entirely accurate for large angles but is often used for simplicity.
    // A common approach:
    //  kf_roll  += dt * (gyro_x + sin(kf_roll) * tan(kf_pitch) * gyro_y + cos(kf_roll) * tan(kf_pitch) * gyro_z);
    //  kf_pitch += dt * (cos(kf_roll) * gyro_y - sin(kf_roll) * gyro_z);
    //  kf_yaw   += dt * (sin(kf_roll) / cos(kf_pitch) * gyro_y + cos(kf_roll) / cos(kf_pitch) * gyro_z);
    // For this initial simplified version, let's use direct integration, assuming gyro provides dRoll, dPitch, dYaw

    kf_last_gyro_mag = sqrtf(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z); // for the accel gate
    kf_roll += gyro_x * dt;
    kf_pitch += gyro_y * dt;
    kf_yaw += gyro_z * dt; // Yaw is simple gyro integration for now

    // Normalize Yaw to +/- PI
    if (kf_yaw > M_PI) kf_yaw -= 2.0f * M_PI;
    if (kf_yaw < -M_PI) kf_yaw += 2.0f * M_PI;

    // --- Update error covariance matrix P ---
    // P_k = A * P_{k-1} * A^T + Q
    // For this simplified model where state transition matrix A is assumed to be identity (or close to it for small dt),
    // and we are only considering diagonal P for simplicity:
    // P_k[i] = P_{k-1}[i] + Q_angle[i] * dt; // Simplified update, Q is effectively a rate of variance increase
    // A more rigorous approach involves the Jacobian of the state transition function.

    P_diag[0] += Q_angle[0] * dt;
    P_diag[1] += Q_angle[1] * dt;
    P_diag[2] += Q_angle[2] * dt; // Yaw uncertainty also grows
}

bool kalman_update_accel(float accel_x, float accel_y, float accel_z) {
    // --- Gate (audit #9) ---
    // The accelerometer measures specific force, which equals gravity only when the vehicle is
    // not accelerating. Under thrust (several g), drag, or in free fall (~0 g) the vector's
    // direction says nothing about "down", and a fast rotation makes the sample unrepresentative
    // of the filter's (gyro-propagated) attitude. Skip the update outside the band and let the
    // covariance keep growing; the predict step already inflates P by Q*dt.
    if (!isfinite(accel_x) || !isfinite(accel_y) || !isfinite(accel_z)) {
        kf_accel_skipped++;
        return false;
    }
    const float mag = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    if (mag < KALMAN_ACCEL_GATE_LOW_G || mag > KALMAN_ACCEL_GATE_HIGH_G ||
        kf_last_gyro_mag > KALMAN_ACCEL_GATE_MAX_GYRO_RPS) {
        kf_accel_skipped++;
        return false;
    }

    // --- Calculate Roll and Pitch from Accelerometer ---
    // These are the "measurements" for the Kalman filter.
    // Pitch: atan2(-ax, sqrt(ay^2 + az^2)) is well defined for any non-zero vector.
    // Roll: atan2(ay, az) is undefined when both are ~0 (nose straight up/down, gimbal lock):
    // in that case only the pitch measurement is applied instead of feeding atan2(0,0) = 0.
    const float yz2 = accel_y * accel_y + accel_z * accel_z;
    const bool rollObservable = yz2 > 1e-6f;
    const float measured_roll = rollObservable ? atan2f(accel_y, accel_z) : kf_roll;
    const float measured_pitch = atan2f(-accel_x, sqrtf(yz2));

    // --- Kalman Gain K calculation (simplified for diagonal P and R) ---
    float K_roll = rollObservable ? P_diag[0] / (P_diag[0] + R_accel[0]) : 0.0f;
    float K_pitch = P_diag[1] / (P_diag[1] + R_accel[1]);

    // --- Update state estimate with measurement ---
    kf_roll = kf_roll + K_roll * (measured_roll - kf_roll);
    kf_pitch = kf_pitch + K_pitch * (measured_pitch - kf_pitch);

    // Yaw is not updated by accelerometer in this simple model.
    // Magnetometer would be needed for yaw correction.

    // --- Update error covariance matrix P ---
    P_diag[0] = (1 - K_roll) * P_diag[0];
    P_diag[1] = (1 - K_pitch) * P_diag[1];
    // P_diag[2] for yaw remains unchanged as yaw is not updated by accelerometer.
    return true;
}

/**
 * @brief Updates the Yaw estimate using magnetometer data.
 * This function corrects for gyro drift around the Z-axis.
 */
void kalman_update_mag(float mag_x, float mag_y, float mag_z) {
    // Tilt-compensate the magnetometer readings
    float mag_x_comp = mag_x * cos(kf_pitch) + mag_y * sin(kf_roll) * sin(kf_pitch) - mag_z * cos(kf_roll) * sin(kf_pitch);
    float mag_y_comp = mag_y * cos(kf_roll) + mag_z * sin(kf_roll);

    // Calculate measured yaw from the compensated magnetometer values
    float measured_yaw = atan2(-mag_y_comp, mag_x_comp);

    // Kalman Gain for Yaw
    float K_yaw = P_diag[2] / (P_diag[2] + R_mag);

    // Update Yaw estimate. The innovation must be wrap-normalized to [-PI, PI]
    // first: near the +/-PI boundary the raw difference can approach +/-2*PI,
    // which previously produced a large spurious yaw transient.
    float yaw_innovation = measured_yaw - kf_yaw;
    if (yaw_innovation > M_PI) yaw_innovation -= 2.0f * M_PI;
    if (yaw_innovation < -M_PI) yaw_innovation += 2.0f * M_PI;
    kf_yaw = kf_yaw + K_yaw * yaw_innovation;

    // Keep the state itself wrapped as well
    if (kf_yaw > M_PI) kf_yaw -= 2.0f * M_PI;
    if (kf_yaw < -M_PI) kf_yaw += 2.0f * M_PI;

    // Update Yaw error covariance
    P_diag[2] = (1 - K_yaw) * P_diag[2];
}

float kalman_get_variance(int axis) { return (axis >= 0 && axis < 3) ? P_diag[axis] : 0.0f; }

unsigned long kalman_accel_updates_skipped() { return kf_accel_skipped; }

void kalman_get_orientation(float &roll, float &pitch, float &yaw) {
    roll = kf_roll;
    pitch = kf_pitch;
    yaw = kf_yaw;
}

/*
Note on Matrix Operations:
The above implementation simplifies matrix operations by assuming diagonal matrices
for P, Q, and R in many places. A full Kalman filter implementation would require
proper matrix multiplication, addition, subtraction, and inversion.
For example:
P_k = A * P_{k-1} * A_transpose + Q
K = P_k_predicted * H_transpose * inv(H * P_k_predicted * H_transpose + R)
x_k = x_k_predicted + K * (measurement - H * x_k_predicted)
P_k = (I - K * H) * P_k_predicted

These operations would typically be handled by a matrix library (e.g., Eigen for C++),
or implemented as helper functions. For an embedded system, lightweight matrix
functions would be defined.

Example (pseudo-code for matrix multiplication):
void matrix_multiply(float* A, float* B, float* C, int rowsA, int colsA, int colsB) {
    for (int i = 0; i < rowsA; ++i) {
        for (int j = 0; j < colsB; ++j) {
            C[i * colsB + j] = 0;
            for (int k = 0; k < colsA; ++k) {
                C[i * colsB + j] += A[i * colsA + k] * B[k * colsB + j];
            }
        }
    }
}
*/
