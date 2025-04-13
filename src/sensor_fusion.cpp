#include "sensor_fusion.h"
#include <math.h>

// Kalman filter state variables
float kf_roll = 0.0f, kf_pitch = 0.0f, kf_yaw = 0.0f;  // Filtered angles
float kf_q0 = 1.0f, kf_q1 = 0.0f, kf_q2 = 0.0f, kf_q3 = 0.0f; // Filtered quaternion

// External reference to the global Euler angles in main firmware
extern float roll, pitch, yaw;

// Kalman filter matrices and parameters
// State covariance matrix
float P[6][6] = {{1, 0, 0, 0, 0, 0},
                 {0, 1, 0, 0, 0, 0},
                 {0, 0, 1, 0, 0, 0},
                 {0, 0, 0, 1, 0, 0},
                 {0, 0, 0, 0, 1, 0},
                 {0, 0, 0, 0, 0, 1}};

// Process noise covariance
float Q[6] = {0.001, 0.001, 0.001, 0.003, 0.003, 0.003};

// Measurement noise covariance
float R[3] = {0.03, 0.03, 0.03};

// State vector (roll, pitch, yaw, bias_x, bias_y, bias_z)
float x[6] = {0, 0, 0, 0, 0, 0};

// Gyro bias estimates
float gyro_bias[3] = {0, 0, 0};

// Time tracking
unsigned long lastFusionUpdateTime = 0;

// External debug control
extern bool enableSensorDebug;
extern bool enableQuaternionDebug;
extern bool enableFusionDebug;
extern bool enableEulerDebug;

// Structure to hold a parameter set for calibration
typedef struct {
  float processNoise[6];  // Q values
  float measurementNoise[3];  // R values
  float driftMetric;  // Lower is better
} CalibrationParams;

// Function to test a parameter set and measure stability
void testParameterSet(CalibrationParams& params, float testDurationMs) {
  // Save original parameters
  float origQ[6], origR[3];
  for (int i = 0; i < 6; i++) origQ[i] = Q[i];
  for (int i = 0; i < 3; i++) origR[i] = R[i];
  
  // Apply test parameters
  for (int i = 0; i < 6; i++) Q[i] = params.processNoise[i];
  for (int i = 0; i < 3; i++) R[i] = params.measurementNoise[i];
  
  // Reset state for testing
  float testQ0 = 1.0f, testQ1 = 0.0f, testQ2 = 0.0f, testQ3 = 0.0f;
  float testState[6] = {0, 0, 0, 0, 0, 0};
  
  if (icm_data_available) {
    // Initialize test state from current accelerometer data
    testState[0] = atan2(icm_accel[1], icm_accel[2]);
    testState[1] = atan2(-icm_accel[0], sqrt(icm_accel[1]*icm_accel[1] + icm_accel[2]*icm_accel[2]));
    eulerToQuaternion(testState[0], testState[1], testState[2], testQ0, testQ1, testQ2, testQ3);
  }
  
  // Variables to track quaternion stability
  float maxVariance = 0.0f;
  float q0Sum = 0.0f, q1Sum = 0.0f, q2Sum = 0.0f, q3Sum = 0.0f;
  float q0SqSum = 0.0f, q1SqSum = 0.0f, q2SqSum = 0.0f, q3SqSum = 0.0f;
  int sampleCount = 0;
  
  // Run the filter for the test duration
  unsigned long testStartTime = millis();
  unsigned long lastUpdateTime = testStartTime;
  
  while (millis() - testStartTime < testDurationMs) {
    // Wait for new sensor data
    if (millis() - lastUpdateTime >= 10) { // 10ms between updates
      lastUpdateTime = millis();
      
      // Run one iteration of filter update
      if (icm_data_available) {
        // Calculate dt in seconds
        float dt = 0.01f; // Fixed 10ms for consistent testing
        
        // Apply filter update (simplified from updateSensorFusion)
        float gyro_x = icm_gyro[0] - testState[3];
        float gyro_y = icm_gyro[1] - testState[4];
        float gyro_z = icm_gyro[2] - testState[5];
        
        // Predict step
        testState[0] += gyro_x * dt;
        testState[1] += gyro_y * dt;
        testState[2] += gyro_z * dt;
        
        // Update step with accelerometer
        float accel_roll = atan2(icm_accel[1], icm_accel[2]);
        float accel_pitch = atan2(-icm_accel[0], sqrt(icm_accel[1]*icm_accel[1] + icm_accel[2]*icm_accel[2]));
        
        // Innovation
        float y[2] = {accel_roll - testState[0], accel_pitch - testState[1]};
        
        // Unwrap angles
        if (y[0] > PI) y[0] -= 2 * PI;
        if (y[0] < -PI) y[0] += 2 * PI;
        if (y[1] > PI) y[1] -= 2 * PI;
        if (y[1] < -PI) y[1] += 2 * PI;
        
        // Simple gain
        float K = 0.1f;
        testState[0] += K * y[0];
        testState[1] += K * y[1];
        
        // Convert to quaternion
        eulerToQuaternion(testState[0], testState[1], testState[2], testQ0, testQ1, testQ2, testQ3);
        
        // Track statistics for variance calculation
        q0Sum += testQ0;
        q1Sum += testQ1;
        q2Sum += testQ2;
        q3Sum += testQ3;
        
        q0SqSum += testQ0 * testQ0;
        q1SqSum += testQ1 * testQ1;
        q2SqSum += testQ2 * testQ2;
        q3SqSum += testQ3 * testQ3;
        
        sampleCount++;
      }
    }
    
    // Allow other processes to run
    yield();
  }
  
  // Calculate variance of quaternion components
  if (sampleCount > 1) {
    float q0Mean = q0Sum / sampleCount;
    float q1Mean = q1Sum / sampleCount;
    float q2Mean = q2Sum / sampleCount;
    float q3Mean = q3Sum / sampleCount;
    
    float q0Var = (q0SqSum / sampleCount) - (q0Mean * q0Mean);
    float q1Var = (q1SqSum / sampleCount) - (q1Mean * q1Mean);
    float q2Var = (q2SqSum / sampleCount) - (q2Mean * q2Mean);
    float q3Var = (q3SqSum / sampleCount) - (q3Mean * q3Mean);
    
    // Overall quaternion stability metric (we want this to be as low as possible)
    params.driftMetric = q0Var + q1Var + q2Var + q3Var;
  } else {
    // Not enough samples
    params.driftMetric = 999.0f;
  }
  
  // Restore original parameters
  for (int i = 0; i < 6; i++) Q[i] = origQ[i];
  for (int i = 0; i < 3; i++) R[i] = origR[i];
}

// Initialize sensor fusion
void initSensorFusion() {
  Serial.println("Starting dynamic sensor fusion calibration...");
  Serial.println("Please keep the device stationary during calibration.");
  
  // Initial delay to ensure device is stable
  delay(1000);
  
  // Initialize Kalman filter state
  kf_roll = 0;
  kf_pitch = 0;
  kf_yaw = 0;
  
  kf_q0 = 1.0f;
  kf_q1 = 0.0f;
  kf_q2 = 0.0f;
  kf_q3 = 0.0f;
  
  lastFusionUpdateTime = millis();
  
  // Initial estimate of Euler angles from accelerometer
  if (icm_data_available) {
    // Use ICM accelerometer for initial angles
    float accel_roll = atan2(icm_accel[1], icm_accel[2]);
    float accel_pitch = atan2(-icm_accel[0], sqrt(icm_accel[1]*icm_accel[1] + icm_accel[2]*icm_accel[2]));
    
    // Initialize state
    x[0] = accel_roll;
    x[1] = accel_pitch;
    x[2] = 0; // Can't determine yaw from accelerometer alone
    
    // Convert to quaternion
    eulerToQuaternion(x[0], x[1], x[2], kf_q0, kf_q1, kf_q2, kf_q3);
  }
  
  // Dynamic calibration process to find optimal parameters
  bool performDynamicCalibration = true;
  
  if (performDynamicCalibration && icm_data_available) {
    Serial.println("Starting dynamic calibration sequence...");
    
    // Define parameter sets to test
    const int NUM_PARAM_SETS = 5;
    CalibrationParams paramSets[NUM_PARAM_SETS];
    
    // Initialize parameter sets with different values to test
    // Set 1: Default (baseline)
    for (int i = 0; i < 6; i++) paramSets[0].processNoise[i] = Q[i];
    for (int i = 0; i < 3; i++) paramSets[0].measurementNoise[i] = R[i];
    
    // Set 2: Lower process noise (more trust in model)
    for (int i = 0; i < 3; i++) paramSets[1].processNoise[i] = 0.0005f; // For angles
    for (int i = 3; i < 6; i++) paramSets[1].processNoise[i] = 0.001f;  // For bias
    for (int i = 0; i < 3; i++) paramSets[1].measurementNoise[i] = 0.05f;
    
    // Set 3: Higher measurement noise (more trust in gyro)
    for (int i = 0; i < 3; i++) paramSets[2].processNoise[i] = 0.001f;
    for (int i = 3; i < 6; i++) paramSets[2].processNoise[i] = 0.003f;
    for (int i = 0; i < 3; i++) paramSets[2].measurementNoise[i] = 0.08f;
    
    // Set 4: Lower measurement noise (more trust in accelerometer)
    for (int i = 0; i < 3; i++) paramSets[3].processNoise[i] = 0.001f;
    for (int i = 3; i < 6; i++) paramSets[3].processNoise[i] = 0.002f;
    for (int i = 0; i < 3; i++) paramSets[3].measurementNoise[i] = 0.01f;
    
    // Set 5: Balanced set with lower gyro bias process noise
    for (int i = 0; i < 3; i++) paramSets[4].processNoise[i] = 0.001f;
    for (int i = 3; i < 6; i++) paramSets[4].processNoise[i] = 0.0005f; // Lower bias drift
    for (int i = 0; i < 3; i++) paramSets[4].measurementNoise[i] = 0.04f;
    
    // Test each parameter set
    const float TEST_DURATION_MS = 1000.0f; // 1 second per test
    for (int i = 0; i < NUM_PARAM_SETS; i++) {
      Serial.print("Testing parameter set ");
      Serial.print(i+1);
      Serial.print("/");
      Serial.print(NUM_PARAM_SETS);
      Serial.print("...");
      
      testParameterSet(paramSets[i], TEST_DURATION_MS);
      
      Serial.print(" Drift metric: ");
      Serial.println(paramSets[i].driftMetric, 6);
    }
    
    // Find the best parameter set (lowest drift metric)
    int bestSetIndex = 0;
    for (int i = 1; i < NUM_PARAM_SETS; i++) {
      if (paramSets[i].driftMetric < paramSets[bestSetIndex].driftMetric) {
        bestSetIndex = i;
      }
    }
    
    // Apply the best parameter set
    for (int i = 0; i < 6; i++) Q[i] = paramSets[bestSetIndex].processNoise[i];
    for (int i = 0; i < 3; i++) R[i] = paramSets[bestSetIndex].measurementNoise[i];
    
    Serial.print("Dynamic calibration complete. Selected parameter set ");
    Serial.print(bestSetIndex + 1);
    Serial.print(" with drift metric ");
    Serial.println(paramSets[bestSetIndex].driftMetric, 6);
    
    // Print the selected parameters
    Serial.println("Selected parameters:");
    Serial.print("Process noise (Q): ");
    for (int i = 0; i < 6; i++) {
      Serial.print(Q[i], 6);
      Serial.print(" ");
    }
    Serial.println();
    Serial.print("Measurement noise (R): ");
    for (int i = 0; i < 3; i++) {
      Serial.print(R[i], 6);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  Serial.println("Sensor fusion initialized with optimized parameters");
}

// Update sensor fusion with new data
void updateSensorFusion() {
  // Only update if new data is available from both sensors
  if (!icm_data_available) {
    return;
  }
  
  // Calculate delta time in seconds
  unsigned long currentTime = millis();
  float dt = (float)(currentTime - lastFusionUpdateTime) / 1000.0f;
  
  // Safety check for dt
  if (dt <= 0.0f || dt > 0.2f) {
    dt = 0.01f; // Default to 10ms if time delta is unreasonable
  }
  
  lastFusionUpdateTime = currentTime;
  
  // 1. PREDICT STEP
  // Get gyro rates (already in rad/s, no need to convert)
  float gyro_x = icm_gyro[0] - x[3]; // Apply estimated bias
  float gyro_y = icm_gyro[1] - x[4];
  float gyro_z = icm_gyro[2] - x[5];
  
  // Predict next state based on gyro data
  float roll_pred = x[0] + gyro_x * dt;
  float pitch_pred = x[1] + gyro_y * dt;
  float yaw_pred = x[2] + gyro_z * dt;
  
  // Update state prediction
  x[0] = roll_pred;
  x[1] = pitch_pred;
  x[2] = yaw_pred;
  
  // Update state covariance matrix
  for (int i = 0; i < 6; i++) {
    P[i][i] += Q[i] * dt;
  }
  
  // 2. UPDATE STEP with accelerometer data
  // Calculate roll and pitch from accelerometer
  float accel_roll, accel_pitch;
  
  // Use KX134 accelerometer if available, otherwise use ICM accelerometer
  if (kx134_accel[0] != 0 || kx134_accel[1] != 0 || kx134_accel[2] != 0) {
    accel_roll = atan2(kx134_accel[1], kx134_accel[2]);
    accel_pitch = atan2(-kx134_accel[0], sqrt(kx134_accel[1]*kx134_accel[1] + kx134_accel[2]*kx134_accel[2]));
  } else {
    accel_roll = atan2(icm_accel[1], icm_accel[2]);
    accel_pitch = atan2(-icm_accel[0], sqrt(icm_accel[1]*icm_accel[1] + icm_accel[2]*icm_accel[2]));
  }
  
  // Measurement vector
  float z[2] = {accel_roll, accel_pitch};
  
  // Innovation (measurement - prediction)
  float y[2] = {z[0] - x[0], z[1] - x[1]};
  
  // Unwrap angles for innovation
  if (y[0] > PI) y[0] -= 2 * PI;
  if (y[0] < -PI) y[0] += 2 * PI;
  if (y[1] > PI) y[1] -= 2 * PI;
  if (y[1] < -PI) y[1] += 2 * PI;
  
  // Innovation covariance
  float S[2][2] = {{P[0][0] + R[0], 0},
                   {0, P[1][1] + R[1]}};
  
  // Kalman gain
  float K[6][2];
  for (int i = 0; i < 6; i++) {
    K[i][0] = P[i][0] / S[0][0];
    K[i][1] = P[i][1] / S[1][1];
  }
  
  // Update state with measurement
  for (int i = 0; i < 6; i++) {
    x[i] += K[i][0] * y[0] + K[i][1] * y[1];
  }
  
  // Update bias drift compensation
  gyro_bias[0] = x[3];
  gyro_bias[1] = x[4];
  gyro_bias[2] = x[5];
  
  // Keep roll and pitch in range (-PI, PI)
  while (x[0] > PI) x[0] -= 2*PI;
  while (x[0] < -PI) x[0] += 2*PI;
  while (x[1] > PI) x[1] -= 2*PI;
  while (x[1] < -PI) x[1] += 2*PI;
  
  // Update yaw with magnetometer if available
  if (icm_mag[0] != 0 || icm_mag[1] != 0 || icm_mag[2] != 0) {
    // Calculate tilt-compensated heading
    float mag_x = icm_mag[0];
    float mag_y = icm_mag[1];
    float mag_z = icm_mag[2];
    
    // Tilt-compensate the magnetic field readings
    float cos_roll = cos(x[0]);
    float sin_roll = sin(x[0]);
    float cos_pitch = cos(x[1]);
    float sin_pitch = sin(x[1]);
    
    float mag_x_comp = mag_x * cos_pitch + mag_z * sin_pitch;
    float mag_y_comp = mag_x * sin_roll * sin_pitch + mag_y * cos_roll - mag_z * sin_roll * cos_pitch;
    
    // Calculate heading
    float heading = atan2(-mag_y_comp, mag_x_comp);
    
    // Apply declination correction if needed
    // float declination = 0.0f; // Set for your location
    // heading += declination;
    
    // Keep heading in range (-PI, PI)
    while (heading > PI) heading -= 2*PI;
    while (heading < -PI) heading += 2*PI;
    
    // Weight for magnetometer fusion (adjust as needed)
    float mag_weight = 0.01f;
    
    // Fuse with current yaw estimate
    float yaw_diff = heading - x[2];
    
    // Ensure we take the shortest path
    if (yaw_diff > PI) yaw_diff -= 2*PI;
    if (yaw_diff < -PI) yaw_diff += 2*PI;
    
    // Apply weighted correction
    x[2] += yaw_diff * mag_weight;
    
    // Keep yaw in range (-PI, PI)
    while (x[2] > PI) x[2] -= 2*PI;
    while (x[2] < -PI) x[2] += 2*PI;
  }
  
  // Store the Kalman filter output
  kf_roll = x[0];
  kf_pitch = x[1];
  kf_yaw = x[2];
  
  // Convert to quaternion representation
  eulerToQuaternion(kf_roll, kf_pitch, kf_yaw, kf_q0, kf_q1, kf_q2, kf_q3);
  
  // Update the global Euler angles (in degrees for display)
  roll = kf_roll * RAD_TO_DEG;
  pitch = kf_pitch * RAD_TO_DEG;
  yaw = kf_yaw * RAD_TO_DEG;
}

// Print sensor fusion data
void printSensorFusionData() {
  Serial.println("--- Sensor Fusion Results ---");
  
  // Print Kalman filter Euler angles if enabled
  if (enableEulerDebug) {
    Serial.print("KF Euler (deg): Roll=");
    Serial.print(kf_roll * RAD_TO_DEG, 2);
    Serial.print(", Pitch=");
    Serial.print(kf_pitch * RAD_TO_DEG, 2);
    Serial.print(", Yaw=");
    Serial.print(kf_yaw * RAD_TO_DEG, 2);
    
    // Print gyro bias estimates
    Serial.print(" | Gyro Bias: ");
    Serial.print(gyro_bias[0], 5);
    Serial.print(", ");
    Serial.print(gyro_bias[1], 5);
    Serial.print(", ");
    Serial.println(gyro_bias[2], 5);
  }
  
  // Print quaternion values if enabled
  if (enableQuaternionDebug) {
    // Print Kalman filter quaternion
    Serial.print("KF Quat: ");
    Serial.print(kf_q0, 4);
    Serial.print(", ");
    Serial.print(kf_q1, 4);
    Serial.print(", ");
    Serial.print(kf_q2, 4);
    Serial.print(", ");
    Serial.println(kf_q3, 4);
    
    // Print original quaternion for comparison
    Serial.print("ICM Quat: ");
    Serial.print(icm_q0, 4);
    Serial.print(", ");
    Serial.print(icm_q1, 4);
    Serial.print(", ");
    Serial.print(icm_q2, 4);
    Serial.print(", ");
    Serial.println(icm_q3, 4);
  }
}

// Convert quaternion to Euler angles
void quaternionToEuler(float q0, float q1, float q2, float q3, float& roll, float& pitch, float& yaw) {
  // Roll (x-axis rotation)
  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  roll = atan2(sinr_cosp, cosr_cosp);
  
  // Pitch (y-axis rotation)
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (fabs(sinp) >= 1.0f)
    pitch = copysign(PI / 2, sinp); // Use 90 degrees if out of range
  else
    pitch = asin(sinp);
  
  // Yaw (z-axis rotation)
  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  yaw = atan2(siny_cosp, cosy_cosp);
}

// Convert Euler angles to quaternion
void eulerToQuaternion(float roll, float pitch, float yaw, float& q0, float& q1, float& q2, float& q3) {
  float cr = cos(roll * 0.5f);
  float sr = sin(roll * 0.5f);
  float cp = cos(pitch * 0.5f);
  float sp = sin(pitch * 0.5f);
  float cy = cos(yaw * 0.5f);
  float sy = sin(yaw * 0.5f);
  
  q0 = cr * cp * cy + sr * sp * sy;
  q1 = sr * cp * cy - cr * sp * sy;
  q2 = cr * sp * cy + sr * cp * sy;
  q3 = cr * cp * sy - sr * sp * cy;
  
  // Normalize quaternion
  float norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  if (norm > 0.0f) {
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
  }
} 