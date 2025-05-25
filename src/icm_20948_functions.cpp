#include "icm_20948_functions.h"
#include <Arduino.h>
#include <Wire.h>
#include "config.h" // For Madgwick filter parameters
#include "utility_functions.h" // Added to access convertQuaternionToEuler

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
const float ACCEL_VARIANCE_THRESHOLD = 0.005; // Reduced from 0.008 to 0.005 for better sensitivity
float sampleFreq = 100.0f;              // Hz
uint32_t lastUpdateTime = 0;            // micros

// Add state transition counters for hysteresis
int stationaryCounter = 0;
int movingCounter = 0;
const int STATE_CHANGE_THRESHOLD = 3;   // Reduced from 5 to 3 for faster state change detection

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
float magBias[3] = {0, 0, 0};           // Hard iron correction
float magScale[3] = {1, 1, 1};          // Soft iron correction

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
}

// Apply magnetometer calibration
void applyMagnetometerCalibration() {
  // Apply hard iron correction (offset) and soft iron correction (scale)
  float calibratedMag[3];
  calibratedMag[0] = (icm_mag[0] - magBias[0]) * magScale[0];
  calibratedMag[1] = (icm_mag[1] - magBias[1]) * magScale[1];
  calibratedMag[2] = (icm_mag[2] - magBias[2]) * magScale[2];
  
  // Update the global values
  icm_mag[0] = calibratedMag[0];
  icm_mag[1] = calibratedMag[1];
  icm_mag[2] = calibratedMag[2];
}

// Read data from the ICM-20948 sensor
void ICM_20948_read() {
  static unsigned long lastErrorPrintTime = 0;
  static unsigned long lastDetailedDebugTime = 0;
  
    if (myICM.dataReady()) {
    myICM.getAGMT();  // Get the latest data
    
    // Apply magnetometer calibration before using the mag data
    applyMagnetometerCalibration(); 

    // Convert accelerometer data from mg to g and store in global array
    icm_accel[0] = myICM.accX() / 1000.0f;
    icm_accel[1] = myICM.accY() / 1000.0f;
    icm_accel[2] = myICM.accZ() / 1000.0f;
    
    // Store gyroscope data in global array (convert to radians for calculations)
    icm_gyro[0] = myICM.gyrX() * DEG_TO_RAD;
    icm_gyro[1] = myICM.gyrY() * DEG_TO_RAD;
    icm_gyro[2] = myICM.gyrZ() * DEG_TO_RAD;
    
    // Store magnetometer data in global array (uT)
    icm_mag[0] = myICM.magX();
    icm_mag[1] = myICM.magY();
    icm_mag[2] = myICM.magZ();

    // Store temperature in global variable (C)
    icm_temp = myICM.temp();

    // --- Motion Detection Logic ---
    uint32_t currentTime = micros();
    float deltat = (lastUpdateTime > 0) ? ((currentTime - lastUpdateTime) / 1000000.0f) : (1.0f / sampleFreq);
    lastUpdateTime = currentTime;
    if (deltat <= 0) deltat = 1.0f / sampleFreq; // Prevent division by zero or negative dt

    // Calculate magnitudes and variance for motion detection
    // Gyro magnitude (rad/s)
    gyroMagnitude = sqrt(icm_gyro[0] * icm_gyro[0] + icm_gyro[1] * icm_gyro[1] + icm_gyro[2] * icm_gyro[2]);

    // Accelerometer variance (difference from previous magnitude, in g)
    // Note: get_accel_magnitude() from utility_functions.h could be used if it's preferred
    // but that function uses kx134 or icm, here we are specifically in ICM context.
    // For simplicity, let's use the raw icm_accel data which is already in g.
    float accelMagnitudeCurrent = sqrt(icm_accel[0] * icm_accel[0] + icm_accel[1] * icm_accel[1] + icm_accel[2] * icm_accel[2]);
    if (accelMagnitudePrev != 0.0f) { // Avoid large variance on first run
        accelVariance = fabs(accelMagnitudeCurrent - accelMagnitudePrev);
    }
    accelMagnitudePrev = accelMagnitudeCurrent;

    if (gyroMagnitude < GYRO_THRESHOLD && accelVariance < ACCEL_VARIANCE_THRESHOLD) {
        stationaryCounter++;
        movingCounter = 0;
        if (stationaryCounter >= STATE_CHANGE_THRESHOLD) {
            isStationary = true;
            stationaryCounter = STATE_CHANGE_THRESHOLD; // Prevent overflow
        }
    } else {
        movingCounter++;
        stationaryCounter = 0;
        if (movingCounter >= STATE_CHANGE_THRESHOLD) {
            isStationary = false;
            movingCounter = STATE_CHANGE_THRESHOLD; // Prevent overflow
        }
    }

    // --- Dynamic Beta Adjustment ---
    if (isStationary) {
        beta = MADGWICK_BETA_STATIONARY;
    } else {
        beta = MADGWICK_BETA_MOTION;
    }
    // --- End Motion Detection & Dynamic Beta ---

    // --- Madgwick AHRS MARG Update ---
    float ax = icm_accel[0], ay = icm_accel[1], az = icm_accel[2];
    float gx_corrected = icm_gyro[0] - gyroBias[0];
    float gy_corrected = icm_gyro[1] - gyroBias[1];
    float gz_corrected = icm_gyro[2] - gyroBias[2];
    float mx_cal = icm_mag[0]; // Already calibrated
    float my_cal = icm_mag[1]; // Already calibrated
    float mz_cal = icm_mag[2]; // Already calibrated

    // Call the 9-axis Madgwick update function
    MadgwickAHRSupdateMARG(gx_corrected, gy_corrected, gz_corrected,
                           ax, ay, az,
                           mx_cal, my_cal, mz_cal,
                           deltat, beta);
    // Note: icm_q0, icm_q1, icm_q2, icm_q3 are updated globally by MadgwickAHRSupdateMARG

    // Gyro bias estimation (original logic, can be refined or integrated into MARG if needed)
    // For now, let's keep the existing gyro bias update logic based on accelerometer feedback
    // as the MARG function above doesn't explicitly include gyro bias updates in this form.
    // This part might need to be removed or adapted if the MARG implementation handles it.
    // However, the provided MARG function does not update gyroBias.
    // The IMU version had this:
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        float recipNorm_acc;
        recipNorm_acc = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        float ax_norm = ax * recipNorm_acc;
        float ay_norm = ay * recipNorm_acc;
        float az_norm = az * recipNorm_acc;

        float hx_g = 2.0f * (icm_q1 * icm_q3 - icm_q0 * icm_q2);
        float hy_g = 2.0f * (icm_q0 * icm_q1 + icm_q2 * icm_q3);
        float hz_g = icm_q0 * icm_q0 - icm_q1 * icm_q1 - icm_q2 * icm_q2 + icm_q3 * icm_q3;

        float ex = (ay_norm * hz_g - az_norm * hy_g);
        float ey = (az_norm * hx_g - ax_norm * hz_g);
        float ez = (ax_norm * hy_g - ay_norm * hx_g);

        if (MADGWICK_GYRO_BIAS_LEARN_RATE > 0.0f) {
            gyroBias[0] += MADGWICK_GYRO_BIAS_LEARN_RATE * ex * deltat;
            gyroBias[1] += MADGWICK_GYRO_BIAS_LEARN_RATE * ey * deltat;
            gyroBias[2] += MADGWICK_GYRO_BIAS_LEARN_RATE * ez * deltat;
        }
    }
    // --- End Madgwick AHRS MARG Update ---
    
    // Print detailed raw sensor data and Madgwick Euler angles every second for debugging
    if (enableICMRawDebug && millis() - lastDetailedDebugTime > 1000) {
      lastDetailedDebugTime = millis();
      
      Serial.println("--- ICM-20948 Raw Data ---");
      Serial.print("Raw Gyro (rad/s): X=");
      Serial.print(icm_gyro[0], 6);
      Serial.print(", Y=");
      Serial.print(icm_gyro[1], 6);
      Serial.print(", Z=");
      Serial.println(icm_gyro[2], 6);
      
      Serial.print("Gyro Bias (rad/s): X=");
      Serial.print(gyroBias[0], 6);
      Serial.print(", Y=");
      Serial.print(gyroBias[1], 6);
      Serial.print(", Z=");
      Serial.println(gyroBias[2], 6);
      
      Serial.print("Gyro Magnitude: ");
      Serial.println(gyroMagnitude, 6);
      
      Serial.print("Stationary: ");
      Serial.print(isStationary ? "YES" : "NO");
      Serial.print(", Counter: ");
      Serial.print(isStationary ? stationaryCounter : movingCounter);
      Serial.print(", Beta: ");
      Serial.println(beta, 4);

      // --- Temporary Madgwick Euler Angle Debug Print ---
      float roll_deg, pitch_deg, yaw_deg;
      convertQuaternionToEuler(icm_q0, icm_q1, icm_q2, icm_q3, roll_deg, pitch_deg, yaw_deg);

      // Convert radians to degrees for printing
      roll_deg *= (180.0f / PI);
      pitch_deg *= (180.0f / PI);
      yaw_deg *= (180.0f / PI);

      Serial.print("Madgwick Euler (deg) - R: "); Serial.print(roll_deg, 1);
      Serial.print(" P: "); Serial.print(pitch_deg, 1);
      Serial.print(" Y: "); Serial.println(yaw_deg, 1);
      // --- End Temporary Madgwick Euler Angle Debug Print ---
    }
      icm_data_available = true;
  } else {
    // Only print error message once every 5 seconds to avoid flooding serial
    if (enableSensorDebug && millis() - lastErrorPrintTime > 5000) {
      lastErrorPrintTime = millis();
      Serial.println("ICM-20948: No new data available");
    }
    icm_data_available = false;
  }
}

// Calibrate the magnetometer
void ICM_20948_calibrate() {
  const int numSamples = 500;
  float magMin[3] = {99999.0f, 99999.0f, 99999.0f};
  float magMax[3] = {-99999.0f, -99999.0f, -99999.0f};
  
  Serial.println("Starting magnetometer calibration. Move the sensor in a figure-8 pattern...");
  
  // Collect calibration data
  for (int i = 0; i < numSamples; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      
      float mx = myICM.magX();
      float my = myICM.magY();
      float mz = myICM.magZ();
      
      // Update min/max values
      magMin[0] = min(magMin[0], mx);
      magMin[1] = min(magMin[1], my);
      magMin[2] = min(magMin[2], mz);
      
      magMax[0] = max(magMax[0], mx);
      magMax[1] = max(magMax[1], my);
      magMax[2] = max(magMax[2], mz);
      
      if (i % 50 == 0) {
        Serial.print("Progress: ");
        Serial.print(i * 100 / numSamples);
        Serial.println("%");
      }
      
      delay(10);
    }
  }
  
  // Calculate bias (hard iron) and scale (soft iron)
  for (int i = 0; i < 3; i++) {
    magBias[i] = (magMax[i] + magMin[i]) / 2.0f;
    magScale[i] = (magMax[i] - magMin[i]) / 2.0f;
  }
  
  // Normalize scale factors
  float avg_scale = (magScale[0] + magScale[1] + magScale[2]) / 3.0f;
  for (int i = 0; i < 3; i++) {
    if (magScale[i] != 0) {
      magScale[i] = avg_scale / magScale[i];
    } else {
      magScale[i] = 1.0f;  // Avoid division by zero
    }
  }
  
  Serial.println("Magnetometer calibration complete!");
  Serial.println("Bias (hard iron):");
  Serial.print("X: "); Serial.print(magBias[0]);
  Serial.print(" Y: "); Serial.print(magBias[1]);
  Serial.print(" Z: "); Serial.println(magBias[2]);
  
  Serial.println("Scale (soft iron):");
  Serial.print("X: "); Serial.print(magScale[0]);
  Serial.print(" Y: "); Serial.print(magScale[1]);
  Serial.print(" Z: "); Serial.println(magScale[2]);
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