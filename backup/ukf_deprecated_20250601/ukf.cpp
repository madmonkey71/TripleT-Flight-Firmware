#include "ukf.h"
#include <math.h>
#include "constants.h"
#include "config.h"

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <time.h>
#endif

// --- ADDED Dependencies for ProcessUKF ---
// Definitions
// #define ACCEL_POLL_INTERVAL 100 // <<< REMOVED - Now in constants.h

// External Global Variables from TripleT_Flight_Firmware.cpp
extern bool icm20948_ready;
// extern bool USE_KX134; // <<< REMOVED - USE_KX134 is a macro from config.h, not a variable
extern bool kx134_initialized_ok;
extern float kx134_accel[3];
extern float icm_accel[3];
extern bool enableSystemDebug;
// --- END ADDED Dependencies ---

// Global UKF instance - now declared as extern since it's already defined in TripleT_Flight_Firmware.cpp
// extern UKF ukf; // Moved to ukf.h
extern UKF ukf_filter; // Assuming 'ukf_filter' is the global UKF object
extern bool ukf_initialized_flag; // Assuming this flag is set elsewhere after initialization
extern float ukf_position_output;
extern float ukf_velocity_output;
extern float ukf_acceleration_output;

// Last timestamp for time delta calculation
unsigned long UKF::lastTimestamp_ = 0;

// Get current time in milliseconds - platform independent
unsigned long UKF::getTimeMillis() {
#ifdef ARDUINO
    // On Arduino platforms, use the global millis function
    return millis();
#else
    // On other platforms, use a simple clock implementation
    static bool initialized = false;
    static clock_t start_time;
    
    if (!initialized) {
        start_time = clock();
        initialized = true;
    }
    
    clock_t current = clock();
    return (unsigned long)((double)(current - start_time) / CLOCKS_PER_SEC * 1000.0);
#endif
}

// Constructor - initialize parameters
UKF::UKF() {
    // Initialize dimensions
    n_x_ = 3;  // State dimension [position, velocity, acceleration]
    n_aug_ = 5;  // Augmented state dimension (includes process noise)
    
    // UKF parameters
    alpha_ = 0.3;  // Spread parameter (small alpha means sigma points closer to mean)
    beta_ = 2.0;   // Prior knowledge about distribution (2 is optimal for Gaussian)
    kappa_ = 0.0;  // Secondary scaling parameter
    lambda_ = alpha_ * alpha_ * (n_aug_ + kappa_) - n_aug_;
    
    // Initialize weights
    double weight_0 = lambda_ / (lambda_ + n_aug_);
    weights_[0] = weight_0;
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {
        weights_[i] = 0.5 / (lambda_ + n_aug_);
    }
    
    // Initialize state to zero
    for (int i = 0; i < n_x_; i++) {
        x_[i] = 0.0;
        
        // Initialize diagonal elements of P (uncertainty)
        P_[i][i] = 1.0;
        
        // Zero out non-diagonal elements
        for (int j = 0; j < n_x_; j++) {
            if (i != j) {
                P_[i][j] = 0.0;
            }
        }
    }
    
    // Initialize process noise covariance Q
    // Position process noise
    Q_[0][0] = 0.01;
    Q_[0][1] = 0.0;
    Q_[0][2] = 0.0;
    
    // Velocity process noise
    Q_[1][0] = 0.0;
    Q_[1][1] = 0.1;
    Q_[1][2] = 0.0;
    
    // Acceleration process noise
    Q_[2][0] = 0.0;
    Q_[2][1] = 0.0;
    Q_[2][2] = 1.0;
    
    // Initialize measurement noise covariance R
    // KX134 measurement noise
    R_[0][0] = 0.5;
    R_[0][1] = 0.0;
    
    // ICM20948 measurement noise
    R_[1][0] = 0.0;
    R_[1][1] = 0.8;  // Slightly higher uncertainty for ICM
    
    // Set initialization flag
    is_initialized_ = false;
}

// Initialize the filter with starting values
void UKF::initialize(float position, float velocity, float acceleration) {
    x_[0] = position;
    x_[1] = velocity;
    x_[2] = acceleration;
    
    // Set initialization flag
    is_initialized_ = true;
    
    // Reset timestamp on initialization
    lastTimestamp_ = getTimeMillis();
}

// Generate sigma points around current state
void UKF::generateSigmaPoints() {
    // Create augmented mean state (x_aug = [x, 0, 0]) where zeros are process noise means
    // Note: x_aug is not directly used in this implementation as the sigma points are created directly
    // float x_aug[5] = {0}; // Removed unused variable
    
    // Create augmented covariance matrix
    float P_aug[5][5] = {0};
    
    // Copy P into upper-left of P_aug
    for (int i = 0; i < n_x_; i++) {
        for (int j = 0; j < n_x_; j++) {
            P_aug[i][j] = P_[i][j];
        }
    }
    
    // Add Q to the lower-right of P_aug (process noise covariance)
    P_aug[3][3] = Q_[1][1];  // Process noise for velocity
    P_aug[4][4] = Q_[2][2];  // Process noise for acceleration
    
    // Calculate square root of P_aug
    // Note: This is a simplified approach. For a complete implementation,
    // you would use Cholesky decomposition
    float A[5][5] = {0};
    for (int i = 0; i < n_aug_; i++) {
        A[i][i] = sqrt(P_aug[i][i]);
    }
    
    // Calculate sigma points
    // First sigma point is the mean
    for (int i = 0; i < n_x_; i++) {
        Xsig_[0][i] = x_[i];
    }
    
    // Calculate remaining sigma points
    float term;
    for (int i = 0; i < n_aug_; i++) {
        // For points 1 to n_aug_
        for (int j = 0; j < n_x_; j++) {
            if (i < n_x_) {
                term = sqrt(lambda_ + n_aug_) * A[i][j];
            } else {
                term = 0;
            }
            Xsig_[i+1][j] = x_[j] + term;
        }
        
        // For points n_aug_+1 to 2*n_aug_
        for (int j = 0; j < n_x_; j++) {
            if (i < n_x_) {
                term = sqrt(lambda_ + n_aug_) * A[i][j];
            } else {
                term = 0;
            }
            Xsig_[i+1+n_aug_][j] = x_[j] - term;
        }
    }
}

// Predict mean and covariance
void UKF::predictMeanAndCovariance(float dt) {
    // Generate sigma points
    generateSigmaPoints();
    
    // Simple linear state transition model:
    // position += velocity * dt + 0.5 * acceleration * dt^2
    // velocity += acceleration * dt
    // acceleration stays the same (plus noise)
    
    // Propagate sigma points through state transition function
    for (int i = 0; i < 7; i++) {
        float pos = Xsig_[i][0];
        float vel = Xsig_[i][1];
        float acc = Xsig_[i][2];
        
        // Apply state transition
        Xsig_[i][0] = pos + vel * dt + 0.5f * acc * dt * dt;
        Xsig_[i][1] = vel + acc * dt;
        // Xsig_[i][2] = acc; // Acceleration stays the same
    }
    
    // Calculate predicted mean
    for (int j = 0; j < n_x_; j++) {
        x_[j] = 0.0f;
        for (int i = 0; i < 7; i++) {
            x_[j] += weights_[i] * Xsig_[i][j];
        }
    }
    
    // Calculate predicted covariance
    for (int j = 0; j < n_x_; j++) {
        for (int k = 0; k < n_x_; k++) {
            float sum = 0.0f;
            for (int i = 0; i < 7; i++) {
                float diff_j = Xsig_[i][j] - x_[j];
                float diff_k = Xsig_[i][k] - x_[k];
                sum += weights_[i] * diff_j * diff_k;
            }
            P_[j][k] = sum + Q_[j][k]; // Add process noise
        }
    }
}

// Update step - incorporate accelerometer measurement
void UKF::updateState(float kx134_accel_z, float icm_accel_z) {
    // For simplicity, we'll use the average of both sensor readings
    float z = (kx134_accel_z + icm_accel_z) / 2.0f;
    
    // Predicted measurement mean
    float z_pred = 0.0f;
    for (int i = 0; i < 7; i++) {
        z_pred += weights_[i] * Xsig_[i][2]; // Using the acceleration state
    }
    
    // Innovation (measurement) covariance
    float S = 0.0f;
    for (int i = 0; i < 7; i++) {
        float diff = Xsig_[i][2] - z_pred;
        S += weights_[i] * diff * diff;
    }
    S += R_[0][0]; // Add measurement noise (simplified for single measurement type)
    
    // Calculate Kalman gain K
    float K = 0.0f;
    float T = 0.0f; // Cross-covariance Tc
    for (int i = 0; i < 7; i++) {
        float diff_x = Xsig_[i][2] - x_[2]; // Difference in acceleration state
        float diff_z = Xsig_[i][2] - z_pred;
        T += weights_[i] * diff_x * diff_z;
    }
    K = T / S;
    
    // Update state and covariance
    float z_diff = z - z_pred; // Measurement residual
    for (int i = 0; i < n_x_; i++) {
        // For simplicity, only updating acceleration state with this measurement
        // A more complete implementation would map K to all state variables
        if (i == 2) { // If current state is acceleration
            x_[i] += K * z_diff;
        }
    }
    
    // Update covariance matrix P
    // P = P - K * S * K_transpose
    // For simplicity, only updating the acceleration part of P
    P_[2][2] -= K * S * K;
}

// Process accelerometer data from both sensors
void UKF::processAccel(float kx134_accel_z, float icm_accel_z, float dt) {
    if (!is_initialized_) {
        // If not initialized, use current measurements to initialize
        // Initialize with 0 position, 0 velocity, and current accel
        initialize(0.0f, 0.0f, (kx134_accel_z + icm_accel_z) / 2.0f);
        return; // Exit after initialization on first call
    }
    
    // Prediction step
    predictMeanAndCovariance(dt);
    
    // Update step
    updateState(kx134_accel_z, icm_accel_z);
}

// --- NEW FUNCTION for UKF Processing ---
void ProcessUKF() {
    // Check if sensor data is ready
    if (!icm20948_ready) {
        if (enableSystemDebug) {
            Serial.println(F("ProcessUKF: ICM20948 data not ready."));
        }
        return;
    }

    // Calculate dt (time delta)
    static unsigned long last_accel_poll_time = 0;
    unsigned long current_time = millis();
    if (last_accel_poll_time == 0) {
        last_accel_poll_time = current_time;
        return; // Not enough time passed for a valid dt
    }
    float dt = (current_time - last_accel_poll_time) / 1000.0f;
    last_accel_poll_time = current_time;

    if (dt <= 0) {
        if (enableSystemDebug) {
            Serial.println(F("ProcessUKF: dt is zero or negative. Skipping."));
        }
        return; // dt must be positive
    }
    
    // Get acceleration data (vertical axis - Z)
    // Note: kx134_accel[2] is accel_z from KX134
    // Note: icm_accel[2] is accel_z from ICM20948
    float kx_accel_z_ms2 = 0.0f;
    float icm_accel_z_ms2 = icm_accel[2]; // ICM provides data in m/s^2

#ifdef USE_KX134    
    if (kx134_initialized_ok) {
        kx_accel_z_ms2 = kx134_accel[2]; // KX134 provides data in m/s^2
    } else if (enableSystemDebug) {
        Serial.println(F("ProcessUKF: KX134 not initialized, using only ICM."));
    }
#else
    if (enableSystemDebug) {
        // Serial.println(F("ProcessUKF: KX134 not enabled, using only ICM.")); // Too verbose
    }
#endif

    // Call the UKF process function
    ukf_filter.processAccel(kx_accel_z_ms2, icm_accel_z_ms2, dt);

    // Update global state variables if UKF is initialized
    if (!ukf_initialized_flag) {
        // Initialize UKF on first valid accel data
        // Assuming initial state is 0 position, 0 velocity, and current accel
        ukf_filter.initialize(0.0f, 0.0f, (kx_accel_z_ms2 + icm_accel_z_ms2) / 2.0f);
        ukf_initialized_flag = true; // Set the flag after initialization
        if (enableSystemDebug) Serial.println(F("UKF Initialized."));
    }

    ukf_position_output   = ukf_filter.getPosition();
    ukf_velocity_output   = ukf_filter.getVelocity();
    ukf_acceleration_output = ukf_filter.getAcceleration();

    if (enableSystemDebug) {
        Serial.print(F("UKF Out: P=")); Serial.print(ukf_position_output);
        Serial.print(F(" V=")); Serial.print(ukf_velocity_output);
        Serial.print(F(" A=")); Serial.println(ukf_acceleration_output);
    }
    
    // Handle cases where UKF might not be initialized (e.g., sensor errors on startup)
    // This might be redundant if initialization is handled robustly above.
    // Consider if this block is still needed post-initialization logic.
    /*
      if (!ukfInitialized) {
          ukf_pos = 0.0f;
          ukf_vel = 0.0f;
          ukf_accel = 0.0f;
          if (enableSystemDebug) {
            Serial.println(F("UKF not initialized, output set to 0."));
          }
      }
    */
}
