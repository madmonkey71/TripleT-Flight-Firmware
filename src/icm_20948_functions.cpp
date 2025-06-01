#include "icm_20948_functions.h"
#include <Arduino.h>
#include <Wire.h>
#include "config.h" // For Madgwick filter parameters and AHRS selection
#include "constants.h" // For physical constants like STANDARD_GRAVITY, DEG_TO_RAD
#include "utility_functions.h" // Added to access convertQuaternionToEuler
#include "kalman_filter.h"   // For Kalman filter AHRS

// Add extern declarations for the debug flags
extern bool enableSensorDebug;
extern bool enableICMRawDebug; // New flag to control ICM raw data output

// Initialize ICM_20948 variables
float icm_accel[3] = {0.0f, 0.0f, 0.0f};  // Accelerometer data (g)
float icm_gyro[3] = {0.0f, 0.0f, 0.0f};   // Gyroscope data (deg/s)
float icm_mag[3] = {0.0f, 0.0f, 0.0f};    // Magnetometer data (uT)
float icm_temp = 0.0f;                    // Temperature in degrees C
float icm_q0 = 1.0f, icm_q1 = 0.0f, icm_q2 = 0.0f, icm_q3 = 0.0f;  // Quaternion components (w,x,y,z)
bool icm_data_available = false;          // Flag to indicate if data is ready
uint16_t icm_data_header = 0;             // FIFO header for checking packet types

// ICM-20948 IMU object
ICM_20948_I2C myICM;

// Motion detection
bool isStationary = true;
float accelMagnitudePrev = 0;
float accelVariance = 0;
float gyroMagnitude = 0;
const float GYRO_THRESHOLD = 0.03;      // Reduced from 0.06 to 0.03 rad/s (about 1.7 deg/s) for better sensitivity
const float ACCEL_VARIANCE_THRESHOLD = 0.008f; // Increased from 0.005 for less sensitivity
float sampleFreq = 100.0f;              // Hz
uint32_t lastUpdateTime = 0;            // micros

// Add state transition counters for hysteresis
int stationaryCounter = 0;
int movingCounter = 0;
const int STATE_CHANGE_THRESHOLD = 5;   // Increased from 3 for more stable stationary detection

// Madgwick filter parameters
float beta = MADGWICK_BETA_INIT;       // Initialized from config.h
// const float BETA_STATIONARY = 0.02f; // Now use MADGWICK_BETA_STATIONARY from config.h directly
// const float BETA_MOTION = 0.1f;      // Now use MADGWICK_BETA_MOTION from config.h directly

// Add gyro bias estimation
float gyroBias[3] = {0.0f, 0.0f, 0.0f}; // Estimated gyro bias
// const float GYRO_BIAS_LEARN_RATE = 0.0005f; // Now use MADGWICK_GYRO_BIAS_LEARN_RATE from config.h directly

// Previous quaternion values for drift compensation
float q0_prev = 1.0f, q1_prev = 0.0f, q2_prev = 0.0f, q3_prev = 0.0f;

// Drift compensation factors
const float STATIONARY_DRIFT_COMPENSATION_FACTOR = 0.7f;  // Increased from 0.98f for better stability when stationary
const float MOVING_DRIFT_COMPENSATION_FACTOR = 0.1f;      // Decreased for more responsiveness during motion

// ZUPT (Zero-Velocity Update) implementation
bool stationaryReferenceSet = false;
float stationaryReferenceQ0 = 1.0f, stationaryReferenceQ1 = 0.0f, stationaryReferenceQ2 = 0.0f, stationaryReferenceQ3 = 0.0f;
unsigned long lastZuptTime = 0;
const unsigned long ZUPT_INTERVAL = 30000; // Perform ZUPT every 30 seconds when stationary

// Magnetometer calibration
float magBias[3] = {-371.89521075f, -58.02547425f, -286.3834353f}; // Hard iron correction from calibrate3.py output
float magScale[3][3] = { // Soft iron correction matrix from calibrate3.py output
  {1.06757205f, -0.02487542f,  0.01539642f},
  {-0.02487542f, 1.07100769f, -0.00649950f},
  {0.01539642f, -0.00649950f,  1.13008801f}
};

// Function to update quaternion using 9-axis Madgwick filter
void MadgwickAHRSupdateMARG(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltat, float beta_val) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy; // Removed hz as it was unused
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use global quaternions
    float q0 = icm_q0;
    float q1 = icm_q1;
    float q2 = icm_q2;
    float q3 = icm_q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = 1.0f / sqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient descent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1.0f - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta_val * s0;
        qDot2 -= beta_val * s1;
        qDot3 -= beta_val * s2;
        qDot4 -= beta_val * s3;
    }

    // Integrate rate of change of quaternion
    q0 += qDot1 * deltat;
    q1 += qDot2 * deltat;
    q2 += qDot3 * deltat;
    q3 += qDot4 * deltat;

    // Normalise quaternion
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    // Update global quaternions
    icm_q0 = q0;
    icm_q1 = q1;
    icm_q2 = q2;
    icm_q3 = q3;
}

// Function to perform static gyro bias calibration
void ICM_20948_calibrate_gyro_bias(int num_samples = 2000, int delay_ms = 1) {
    if (enableSensorDebug) {
        Serial.println(F("Starting ICM-20948 Gyro Bias Calibration. Keep the device very still..."));
    }

    float temp_gyro_sum[3] = {0.0f, 0.0f, 0.0f};

    // Ensure sensor is initialized
    if (myICM.status != ICM_20948_Stat_Ok) {
        if (enableSensorDebug) {
            Serial.println(F("ICM-20948 not initialized. Cannot calibrate gyro bias."));
        }
        return;
    }

    // Discard initial readings
    for (int i = 0; i < 100; ++i) {
        if (myICM.dataReady()) {
            myICM.getAGMT(); // Read and discard
        }
        delay(delay_ms);
    }

    int samples_collected = 0;
    for (int i = 0; i < num_samples; ++i) {
        if (myICM.dataReady()) {
            myICM.getAGMT();
            // Gyro data is in dps, convert to rad/s for consistency with Madgwick input and gyroBias storage
            temp_gyro_sum[0] += myICM.gyrX() * DEG_TO_RAD;
            temp_gyro_sum[1] += myICM.gyrY() * DEG_TO_RAD;
            temp_gyro_sum[2] += myICM.gyrZ() * DEG_TO_RAD;
            samples_collected++;
        }
        delay(delay_ms); // Small delay between samples
    }

    if (samples_collected > 0) {
        gyroBias[0] = temp_gyro_sum[0] / samples_collected;
        gyroBias[1] = temp_gyro_sum[1] / samples_collected;
        gyroBias[2] = temp_gyro_sum[2] / samples_collected;

        if (enableSensorDebug) {
            Serial.println(F("Gyro Bias Calibration Complete."));
            Serial.print(F("  Bias X (rad/s): ")); Serial.println(gyroBias[0], 6);
            Serial.print(F("  Bias Y (rad/s): ")); Serial.println(gyroBias[1], 6);
            Serial.print(F("  Bias Z (rad/s): ")); Serial.println(gyroBias[2], 6);
        }
    } else {
        if (enableSensorDebug) {
            Serial.println(F("Gyro Bias Calibration Failed: No samples collected."));
        }
        // Keep existing/default (zero) bias if calibration fails
    }
}

// Initialize ICM-20948 IMU
void ICM_20948_init() {
  // Initialize ICM-20948 with I2C interface
  Wire.begin();
  myICM.begin(Wire, 1); // 1 = ADO high
  
  if (myICM.status != ICM_20948_Stat_Ok) {
    Serial.println("ICM-20948 initialization failed");
    return;
  }
  
  // Configure ICM-20948 using SparkFun library methods
  ICM_20948_Status_e result;
  
  // Set sample rate divider
  result = myICM.setSampleMode(
    (ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), 
    ICM_20948_Sample_Mode_Continuous);
    
  if (result != ICM_20948_Stat_Ok) {
    Serial.println("Failed to set sample mode");
  }
  
  // Configure gyroscope
  ICM_20948_smplrt_t mySmplrt;
  mySmplrt.g = 9; // 1100Hz / (1 + 9) = 110Hz sample rate
  
  result = myICM.setSampleRate(ICM_20948_Internal_Gyr, mySmplrt);
  if (result != ICM_20948_Stat_Ok) {
    Serial.println("Failed to set gyro sample rate");
  }
  
  // Configure gyro settings
  ICM_20948_dlpcfg_t myDLPcfg;
  myDLPcfg.g = 7; // 51.2Hz bandwidth
  
  result = myICM.setDLPFcfg(ICM_20948_Internal_Gyr, myDLPcfg);
  if (result != ICM_20948_Stat_Ok) {
    Serial.println("Failed to set gyro DLPF config");
  }
  
  // Enable the DLPF
  result = myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
  if (result != ICM_20948_Stat_Ok) {
    Serial.println("Failed to enable gyro DLPF");
  }
  
  // Set gyro full scale range to +/- 250 dps
  ICM_20948_fss_t myFSS;
  myFSS.g = 0; // dps250
  
  result = myICM.setFullScale(ICM_20948_Internal_Gyr, myFSS);
  if (result != ICM_20948_Stat_Ok) {
    Serial.println("Failed to set gyro full scale range");
  }
  
  // Configure accelerometer
  mySmplrt.a = 9; // Same 110Hz sample rate for accel
  
  result = myICM.setSampleRate(ICM_20948_Internal_Acc, mySmplrt);
  if (result != ICM_20948_Stat_Ok) {
    Serial.println("Failed to set accel sample rate");
  }
  
  // Configure accel DLPF
  myDLPcfg.a = 7; // 50.4Hz bandwidth
  
  result = myICM.setDLPFcfg(ICM_20948_Internal_Acc, myDLPcfg);
  if (result != ICM_20948_Stat_Ok) {
    Serial.println("Failed to set accel DLPF config");
  }
  
  // Enable the DLPF
  result = myICM.enableDLPF(ICM_20948_Internal_Acc, true);
  if (result != ICM_20948_Stat_Ok) {
    Serial.println("Failed to enable accel DLPF");
  }
  
  // Set accel full scale range to +/- 4g
  myFSS.a = 1; // 4g
  
  result = myICM.setFullScale(ICM_20948_Internal_Acc, myFSS);
  if (result != ICM_20948_Stat_Ok) {
    Serial.println("Failed to set accel full scale range");
  }
  
  // Configure magnetometer
  result = myICM.startupMagnetometer();
  if (result != ICM_20948_Stat_Ok) {
    Serial.println("Failed to start magnetometer");
  }
  
  Serial.println("ICM-20948 initialized successfully");
  
  // Initialize variables
  isStationary = true;
  accelMagnitudePrev = 0;
  accelVariance = 0;
  gyroMagnitude = 0;
  
  lastUpdateTime = micros();

  // Initialize AHRS
#if AHRS_USE_KALMAN
  if (enableSensorDebug) {
    Serial.println(F("Initializing Kalman AHRS..."));
  }
  kalman_init(0.0f, 0.0f, 0.0f); // Initialize with (roll, pitch, yaw) = (0,0,0)
  if (enableSensorDebug) {
    Serial.println(F("Kalman AHRS initialized."));
  }
#else // Default to Madgwick if AHRS_USE_KALMAN is not 1 (i.e., AHRS_USE_MADGWICK is 1)
  if (enableSensorDebug) {
    Serial.println(F("Madgwick AHRS selected (or default). Beta initialized from config.h"));
  }
  // Madgwick beta is already initialized globally: float beta = MADGWICK_BETA_INIT;
  // No specific init function for Madgwick here, quaternion initialized globally.
#endif

  // Initialize lastUpdateTime for deltat calculation
  lastUpdateTime = micros();

  if (enableSensorDebug) {
    Serial.println(F("ICM-20948 Initialized and Configured."));
  }
}

// Apply magnetometer calibration
void applyMagnetometerCalibration() {
  // Apply hard iron correction (offset)
  float hardIronCorrected[3];
  hardIronCorrected[0] = icm_mag[0] - magBias[0];
  hardIronCorrected[1] = icm_mag[1] - magBias[1];
  hardIronCorrected[2] = icm_mag[2] - magBias[2];

  // Apply soft iron correction (matrix multiplication)
  float calibratedMag[3];
  calibratedMag[0] = magScale[0][0] * hardIronCorrected[0] + magScale[0][1] * hardIronCorrected[1] + magScale[0][2] * hardIronCorrected[2];
  calibratedMag[1] = magScale[1][0] * hardIronCorrected[0] + magScale[1][1] * hardIronCorrected[1] + magScale[1][2] * hardIronCorrected[2];
  calibratedMag[2] = magScale[2][0] * hardIronCorrected[0] + magScale[2][1] * hardIronCorrected[1] + magScale[2][2] * hardIronCorrected[2];
  
  // Update the global values
  icm_mag[0] = calibratedMag[0];
  icm_mag[1] = calibratedMag[1];
  icm_mag[2] = calibratedMag[2];
}

// Read data from the ICM-20948 sensor
void ICM_20948_read() {
    if (myICM.dataReady()) {
        myICM.getAGMT(); // Retrieves all data from the sensor
        icm_data_available = true;

        // Calculate deltat
        uint32_t currentTime = micros();
        float deltat = ((float)(currentTime - lastUpdateTime)) / 1000000.0f; // Convert to seconds
        lastUpdateTime = currentTime;
        sampleFreq = 1.0f / deltat; // Update sample frequency

        // Raw data (consider units and scaling as needed)
        float ax_raw = myICM.accX();
        float ay_raw = myICM.accY();
        float az_raw = myICM.accZ();
        float gx_raw = myICM.gyrX();
        float gy_raw = myICM.gyrY();
        float gz_raw = myICM.gyrZ();
        float mx_raw = myICM.magX();
        float my_raw = myICM.magY();
        float mz_raw = myICM.magZ();
        icm_temp = myICM.temp();

        // Store raw values for logging/debugging if needed (already in global icm_accel etc. for some)
        icm_accel[0] = ax_raw; 
        icm_accel[1] = ay_raw; 
        icm_accel[2] = az_raw;
        icm_gyro[0] = gx_raw;  // Assuming these are already in deg/s from library
        icm_gyro[1] = gy_raw;
        icm_gyro[2] = gz_raw;
        // Magnetometer data is stored after calibration below

        // --- Sensor Unit Conversions and Corrections ---
        // Accelerometer: Convert from mG to m/s^2 (if library doesn't do it)
        // SparkFun library getAGMT() provides accel in mg. Convert to g's then to m/s^2.
        // 1 g = 9.80665 m/s^2.
        // Accel data from getAGMT() is in milli-g's. So, (val / 1000.0) * 9.80665
        float ax = ax_raw / 1000.0f * STANDARD_GRAVITY;
        float ay = ay_raw / 1000.0f * STANDARD_GRAVITY;
        float az = az_raw / 1000.0f * STANDARD_GRAVITY;

        // Gyroscope: Convert from dps to rad/s
        float gx_rps = gx_raw * DEG_TO_RAD;
        float gy_rps = gy_raw * DEG_TO_RAD;
        float gz_rps = gz_raw * DEG_TO_RAD;

        // Apply gyro bias correction
        float gx_corrected = gx_rps - gyroBias[0];
        float gy_corrected = gy_rps - gyroBias[1];
        float gz_corrected = gz_rps - gyroBias[2];
        
        // Magnetometer: Apply calibration (hard and soft iron)
        // Raw mag data is in uT (microTesla)
        float mx_cal = mx_raw * magScale[0][0] + my_raw * magScale[0][1] + mz_raw * magScale[0][2] + magBias[0];
        float my_cal = mx_raw * magScale[1][0] + my_raw * magScale[1][1] + mz_raw * magScale[1][2] + magBias[1];
        float mz_cal = mx_raw * magScale[2][0] + my_raw * magScale[2][1] + mz_raw * magScale[2][2] + magBias[2];
        
        // Store calibrated mag data
        icm_mag[0] = mx_cal;
        icm_mag[1] = my_cal;
        icm_mag[2] = mz_cal;

        // Stationary detection logic (simplified, can be expanded)
        // ... (existing stationary detection logic can remain here if needed for beta adjustment) ...
        // Update beta based on stationary state (if Madgwick is used)
        // This logic was here before, ensure it's compatible or adjusted for AHRS_USE_MADGWICK
        float currentAccelMagnitude = sqrt(ax * ax + ay * ay + az * az);
        accelVariance = accelVariance * 0.95f + 0.05f * fabsf(currentAccelMagnitude - accelMagnitudePrev); // Low-pass filter variance
        accelMagnitudePrev = currentAccelMagnitude;
        gyroMagnitude = sqrt(gx_corrected * gx_corrected + gy_corrected * gy_corrected + gz_corrected * gz_corrected);

        if (gyroMagnitude < GYRO_THRESHOLD && accelVariance < ACCEL_VARIANCE_THRESHOLD) {
            stationaryCounter++;
            movingCounter = 0;
            if (stationaryCounter >= STATE_CHANGE_THRESHOLD) {
                isStationary = true;
                stationaryCounter = STATE_CHANGE_THRESHOLD; // Cap counter
            }
        } else {
            movingCounter++;
            stationaryCounter = 0;
            if (movingCounter >= STATE_CHANGE_THRESHOLD) {
                isStationary = false;
                movingCounter = STATE_CHANGE_THRESHOLD; // Cap counter
            }
        }
        
#if AHRS_USE_MADGWICK
        // Update Madgwick beta based on stationary state
        if (isStationary) {
            beta = MADGWICK_BETA_STATIONARY;
        } else {
            beta = MADGWICK_BETA_MOTION;
        }
        
        /* // Temporarily disabling Madgwick AHRS update - RE-ENABLING THIS
        // Call the 9-axis Madgwick update function
        MadgwickAHRSupdateMARG(gx_corrected, gy_corrected, gz_corrected,
                               ax, ay, az,
                               mx_cal, my_cal, mz_cal,
                               deltat, beta);
        */ // End of temporarily disabling Madgwick AHRS update
        // --- Re-enabled Madgwick ---
        MadgwickAHRSupdateMARG(gx_corrected, gy_corrected, gz_corrected,
                               ax, ay, az,
                               mx_cal, my_cal, mz_cal,
                               deltat, beta);
        // Note: icm_q0, icm_q1, icm_q2, icm_q3 are updated globally by MadgwickAHRSupdateMARG

#elif AHRS_USE_KALMAN
        if (enableSensorDebug) {
            // Optional: Print inputs to Kalman if debugging
            // Serial.print("Kalman In: G( "); Serial.print(gx_corrected); Serial.print(", "); Serial.print(gy_corrected); Serial.print(", "); Serial.print(gz_corrected); Serial.print(" ) A( "); Serial.print(ax); Serial.print(", "); Serial.print(ay); Serial.print(", "); Serial.print(az); Serial.print(" ) dt: "); Serial.println(deltat);
        }

        kalman_predict(gx_corrected, gy_corrected, gz_corrected, deltat);
        kalman_update(ax, ay, az); // Magnetometer data for yaw correction can be added to kalman_update later

        float roll_rad, pitch_rad, yaw_rad;
        kalman_get_orientation(roll_rad, pitch_rad, yaw_rad);

        // Convert Euler angles from Kalman filter to quaternion to update global icm_q values
        convertEulerToQuaternion(roll_rad, pitch_rad, yaw_rad, icm_q0, icm_q1, icm_q2, icm_q3);
        
        if (enableSensorDebug) {
            // Optional: Print outputs of Kalman if debugging
            // Serial.print("Kalman Out RPY (rad): "); Serial.print(roll_rad, 4); Serial.print(", "); Serial.print(pitch_rad, 4); Serial.print(", "); Serial.println(yaw_rad, 4);
            // Serial.print("Kalman Out Quat: "); Serial.print(icm_q0, 4); Serial.print(", "); Serial.print(icm_q1, 4); Serial.print(", "); Serial.print(icm_q2, 4); Serial.print(", "); Serial.println(icm_q3, 4);
        }
#endif

        // The online gyro bias estimation was previously commented out here.
        // If re-introducing, ensure it's compatible with the selected AHRS.
        // For now, it remains commented.
        /* 
        if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            // ... (online gyro bias estimation code) ...
        }
        */

        // ZUPT (Zero-Velocity UPdaTe) Logic
        // ... (existing ZUPT logic can remain here) ...

        // Update global icm_accel, icm_gyro, icm_mag with processed (e.g. m/s^2, rad/s, calibrated) values
        // This was a bit inconsistent before, let's clarify:
        // icm_accel should store m/s^2 values
        icm_accel[0] = ax; 
        icm_accel[1] = ay; 
        icm_accel[2] = az;
        // icm_gyro should store rad/s (corrected) values
        icm_gyro[0] = gx_corrected;
        icm_gyro[1] = gy_corrected;
        icm_gyro[2] = gz_corrected;
        // icm_mag is already updated with calibrated values (mx_cal, my_cal, mz_cal)

    } else {
        icm_data_available = false;
    }
}

// Initialize ICM-20948 IMU
void ICM_20948_calibrate() {
    if (enableSensorDebug) {
        Serial.println(F("Starting ICM-20948 Full Calibration Sequence..."));
    }
    // For now, it only calls the gyro bias calibration.
    // Magnetometer calibration is separate and interactive (ICM_20948_calibrate_mag_interactive)
    ICM_20948_calibrate_gyro_bias(); 

    // TODO: Add magnetometer calibration call here if a non-interactive one is developed.

    if (enableSensorDebug) {
        Serial.println(F("ICM-20948 Full Calibration Sequence Finished."));
    }
}

// Print ICM-20948 data to serial
void ICM_20948_print() {
  // Only print if sensor debug is enabled
  if (!enableSensorDebug) return;
  
  Serial.println("ICM-20948 Data:");
  
  Serial.print("  Accelerometer: X:");
  Serial.print(icm_accel[0], 2);
  Serial.print("g Y:");
  Serial.print(icm_accel[1], 2);
  Serial.print("g Z:");
  Serial.print(icm_accel[2], 2);
  Serial.println("g");
  
  Serial.print("  Gyroscope: X:");
  Serial.print(icm_gyro[0] * (180.0/PI), 1);  // Convert to deg/s for display
  Serial.print("°/s Y:");
  Serial.print(icm_gyro[1] * (180.0/PI), 1);
  Serial.print("°/s Z:");
  Serial.print(icm_gyro[2] * (180.0/PI), 1);
  Serial.println("°/s");
  
  Serial.print("  Magnetometer: X:");
  Serial.print(icm_mag[0], 1);
  Serial.print("uT Y:");
  Serial.print(icm_mag[1], 1);
  Serial.print("uT Z:");
  Serial.print(icm_mag[2], 1);
  Serial.println("uT");
  
  Serial.print("  Temperature: ");
  Serial.print(icm_temp, 1);
  Serial.println("°C");

} 

// Function to get calibrated gyroscope data (raw - bias)
void ICM_20948_get_calibrated_gyro(float out_gyro[3]) {
  out_gyro[0] = icm_gyro[0] - gyroBias[0];
  out_gyro[1] = icm_gyro[1] - gyroBias[1];
  out_gyro[2] = icm_gyro[2] - gyroBias[2];
}

// Helper function to convert quaternion to Euler angles (Roll, Pitch, Yaw)
// Roll (x-axis rotation)
// Pitch (y-axis rotation)
// Yaw (z-axis rotation)
// Order of rotation: ZYX
// void convertQuaternionToEuler(float q0, float q1, float q2, float q3, float &roll, float &pitch, float &yaw) {
//     // Roll (x-axis rotation)
//     float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
//     float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
//     roll = atan2(sinr_cosp, cosr_cosp);
// 
//     // Pitch (y-axis rotation)
//     float sinp = 2.0f * (q0 * q2 - q3 * q1);
//     if (fabs(sinp) >= 1)
//         pitch = copysign(PI / 2, sinp); // use 90 degrees if out of range
//     else
//         pitch = asin(sinp);
// 
//     // Yaw (z-axis rotation)
//     float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
//     float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
//     yaw = atan2(siny_cosp, cosy_cosp);
// }