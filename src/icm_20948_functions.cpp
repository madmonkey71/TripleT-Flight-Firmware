#include "icm_20948_functions.h"
#include <Arduino.h>
#include <Wire.h>
#include "config.h" // For Madgwick filter parameters

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

    // --- Madgwick AHRS IMU Update (Simplified) ---
    // Local copies for calculation, to make the algorithm match the provided snippet more closely
    float q0 = icm_q0, q1 = icm_q1, q2 = icm_q2, q3 = icm_q3;
    float ax = icm_accel[0], ay = icm_accel[1], az = icm_accel[2];
    
    // Correct gyroscope readings with current bias estimate
    float gx_corrected = icm_gyro[0] - gyroBias[0];
    float gy_corrected = icm_gyro[1] - gyroBias[1];
    float gz_corrected = icm_gyro[2] - gyroBias[2];

    float recipNorm;
    // float s0, s1, s2, s3; // These were for context, not directly used in the simplified snippet
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2; // _2q3 was not used in the simplified snippet's gradient

    // Rate of change of quaternion from gyroscope (using corrected gyro values)
    qDot1 = 0.5f * (-q1 * gx_corrected - q2 * gy_corrected - q3 * gz_corrected);
    qDot2 = 0.5f * ( q0 * gx_corrected + q2 * gz_corrected - q3 * gy_corrected);
    qDot3 = 0.5f * ( q0 * gy_corrected - q1 * gz_corrected + q3 * gx_corrected);
    qDot4 = 0.5f * ( q0 * gz_corrected + q1 * gy_corrected - q2 * gx_corrected);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        // _2q3 = 2.0f * q3; // _2q3 was not used in the provided simplified version's gradient calc

        // Estimated direction of gravity in body frame from quaternion
        // (Note: Original Madgwick uses 2*q1*q3 - 2*q0*q2 for v_x, etc. This is slightly different but matches snippet)
        float hx_g = 2.0f * (q1 * q3 - q0 * q2); // Corresponds to 2*(q1q3 - q0q2) -> (vx in Madgwick paper)
        float hy_g = 2.0f * (q0 * q1 + q2 * q3); // Corresponds to 2*(q0q1 + q2q3) -> (vy in Madgwick paper)
        float hz_g = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // Corresponds to q0^2 - q1^2 - q2^2 + q3^2 -> (vz in Madgwick paper)

        // Error is cross product between estimated direction and measured direction of gravity
        float ex = (ay * hz_g - az * hy_g);
        float ey = (az * hx_g - ax * hz_g);
        float ez = (ax * hy_g - ay * hx_g);

        // Update gyro bias using error terms (if learning is enabled)
        if (MADGWICK_GYRO_BIAS_LEARN_RATE > 0.0f) {
            gyroBias[0] += MADGWICK_GYRO_BIAS_LEARN_RATE * ex * deltat;
            gyroBias[1] += MADGWICK_GYRO_BIAS_LEARN_RATE * ey * deltat;
            gyroBias[2] += MADGWICK_GYRO_BIAS_LEARN_RATE * ez * deltat;
        }
        
        // Apply feedback to quaternion derivative
        // (Original Madgwick paper: qDot1 -= beta * s0_gradient_component, etc. where s are components of J^T * F)
        // The snippet implies a direct subtraction of these error terms.
        // For an IMU (accel+gyro) update, the error term (ex, ey, ez) is used to correct the gyro rates.
        // The gradient descent step involves multiplying these error terms by components of q.
        // Re-interpreting the snippet's intention for qDot correction:
        // s0, s1, s2, s3 are effectively the "gradient" components scaled by beta
        // The snippet seems to be directly applying ex,ey,ez to qDot components.
        // A standard IMU update would look like:
        // s0 = _2q2 * ex - _2q1 * ey; // Simplified, assumes error is perpendicular to estimated gravity
        // s1 = _2q3 * ex + _2q0 * ey - 2.0f * q1 * ez; // This is more complex
        // For the provided snippet, it is:
        qDot1 -= beta * ex; // This is not standard Madgwick gradient descent on qDot directly.
        qDot2 -= beta * ey; // It implies ex, ey, ez are scaled components of angular error to correct gyro integration.
        qDot3 -= beta * ez; // If ex,ey,ez are components of d(omega_err), then this makes sense.
                            // Let's assume the snippet means to adjust the quaternion derivatives this way.
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

    // Update global quaternion variables
    icm_q0 = q0;
    icm_q1 = q1;
    icm_q2 = q2;
    icm_q3 = q3;
    // --- End Madgwick AHRS IMU Update ---
    
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