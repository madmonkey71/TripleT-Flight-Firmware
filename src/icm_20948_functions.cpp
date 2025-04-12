#include "icm_20948_functions.h"
#include <Arduino.h>
#include <Wire.h>

// Add extern declarations for the debug flags
extern bool enableSensorDebug;
extern bool enableQuaternionDebug;

// Sensor data
ICM_20948_I2C myICM;
float icm_accel[3] = {0, 0, 0};
float icm_gyro[3] = {0, 0, 0};
float icm_mag[3] = {0, 0, 0};
float icm_temp = 0;
double icm_q0 = 1.0, icm_q1 = 0.0, icm_q2 = 0.0, icm_q3 = 0.0;

// Motion detection
bool isStationary = true;
float accelMagnitudePrev = 0;
float accelVariance = 0;
float gyroMagnitude = 0;
const float GYRO_THRESHOLD = 0.06;      // Increased from 0.03 to 0.06 rad/s
const float ACCEL_VARIANCE_THRESHOLD = 0.008; // Increased from 0.005 to 0.008
float sampleFreq = 100.0f;              // Hz
uint32_t lastUpdateTime = 0;            // micros

// Add state transition counters for hysteresis
int stationaryCounter = 0;
int movingCounter = 0;
const int STATE_CHANGE_THRESHOLD = 5;   // Require multiple consecutive readings to change state

// Madgwick filter parameters
float beta = 0.05f;                     // Reduced from 0.1f to reduce influence of noisy measurements
const float BETA_STATIONARY = 0.02f;    // Lower when stationary for less drift
const float BETA_MOTION = 0.1f;         // Higher when in motion for more responsiveness

// Add gyro bias estimation
float gyroBias[3] = {0.0f, 0.0f, 0.0f}; // Estimated gyro bias
const float GYRO_BIAS_LEARN_RATE = 0.001f; // Rate at which we update the gyro bias estimates

// Previous quaternion values for drift compensation
float q0_prev = 1.0f, q1_prev = 0.0f, q2_prev = 0.0f, q3_prev = 0.0f;
const float DRIFT_COMPENSATION_FACTOR = 0.98f;  // Increased from 0.95f to 0.98f for more stability

// Magnetometer calibration
float magBias[3] = {0, 0, 0};           // Hard iron correction
float magScale[3] = {1, 1, 1};          // Soft iron correction

// Initialize the ICM-20948 sensor
void ICM_20948_init() {
  // Start with a slower I2C clock speed
  Wire.begin();
  Wire.setClock(100000); // Try a slower 100kHz speed instead of 400kHz
  
  Serial.println("Initializing ICM-20948...");
  
  // Reset I2C bus
  Wire.end();
  delay(100);
  Wire.begin();
  delay(100);
  
  // Try both possible addresses with timeout
  bool initialized = false;
  unsigned long startTime = millis();
  const unsigned long timeout = 5000; // 5 second timeout
  
  // Try address 0 first (AD0 pin low)
  Serial.println("Trying ICM-20948 with AD0=0...");
  int attempts = 0;
  while (!initialized && (millis() - startTime < timeout) && attempts < 3) {
    attempts++;
    myICM.begin(Wire, 0); // AD0 pin low (0x68)
    
    Serial.print(F("Initialization returned: "));
    Serial.println(myICM.statusString());
    
    if (myICM.status == ICM_20948_Stat_Ok) {
      Serial.println("Successfully connected to ICM-20948 with AD0=0!");
      initialized = true;
    } else {
      delay(500);
    }
  }
  
  // If not successful, try with address 1
  if (!initialized) {
    Serial.println("Trying ICM-20948 with AD0=1...");
    attempts = 0;
    while (!initialized && (millis() - startTime < timeout) && attempts < 3) {
      attempts++;
      myICM.begin(Wire, 1); // AD0 pin high (0x69)
      
      Serial.print(F("Initialization returned: "));
      Serial.println(myICM.statusString());
      
      if (myICM.status == ICM_20948_Stat_Ok) {
        Serial.println("Successfully connected to ICM-20948 with AD0=1!");
        initialized = true;
      } else {
        delay(500);
      }
    }
  }
  
  // Check if we successfully initialized the sensor
  if (!initialized) {
    Serial.println("WARNING: Could not connect to ICM-20948 after multiple attempts");
    Serial.println("Check wiring, power, and address jumpers on the board");
    return; // Exit the function if initialization failed
  }
  
  // Configure the sensors
  ICM_20948_Status_e status;
  
  // Reset the device to ensure it's in a known state
  status = myICM.swReset();
  if (status != ICM_20948_Stat_Ok) {
    Serial.print(F("Software Reset returned: "));
    Serial.println(myICM.statusString(status));
  }
  delay(50);
  
  // Wake the device up
  status = myICM.sleep(false);
  if (status != ICM_20948_Stat_Ok) {
    Serial.print(F("Sleep returned: "));
    Serial.println(myICM.statusString(status));
  }
  delay(50);
  
  // Set full scale ranges for both sensors
  ICM_20948_fss_t fss;
  fss.a = gpm4;       // +/- 4g
  fss.g = dps1000;    // +/- 1000 dps
  
  status = myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss);
  if (status != ICM_20948_Stat_Ok) {
    Serial.print(F("setFullScale returned: "));
    Serial.println(myICM.statusString(status));
  }
  
  // Set ODR (Output Data Rate)
  ICM_20948_dlpcfg_t dlpcfg;
  dlpcfg.a = acc_d473bw_n499bw;  // Highest bandwidth for accelerometer
  dlpcfg.g = gyr_d361bw4_n376bw5;  // Highest bandwidth for gyroscope
  
  status = myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
  if (status != ICM_20948_Stat_Ok) {
    Serial.print(F("setDLPFcfg returned: "));
    Serial.println(myICM.statusString(status));
  }
  
  // Set sample mode for both sensors
  ICM_20948_smplrt_t smplrt;
  smplrt.g = 9;  // about 100Hz
  smplrt.a = 9;  // about 100Hz
  
  status = myICM.setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), smplrt);
  if (status != ICM_20948_Stat_Ok) {
    Serial.print(F("setSampleRate returned: "));
    Serial.println(myICM.statusString(status));
  }
  
  // Enable the magnetometer
  status = myICM.startupMagnetometer();
  if (status != ICM_20948_Stat_Ok) {
    Serial.print(F("startupMagnetometer returned: "));
    Serial.println(myICM.statusString(status));
  }
  
  Serial.println("ICM-20948 initialized successfully");
  delay(500); // Wait for sensor to stabilize
  
  // Initialize last update time
  lastUpdateTime = micros();
}

// Normalize a quaternion
void normalizeQuaternion(float &q0, float &q1, float &q2, float &q3) {
  float recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
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

// Apply a low-pass filter to sensor data
void applyLowPassFilter(float &current, float newValue, float alpha) {
  current = current * (1.0f - alpha) + newValue * alpha;
}

// Detect if the sensor is stationary
void detectStationary() {
  // Apply low-pass filter to gyro readings to reduce noise
  static float filteredGyro[3] = {0.0f, 0.0f, 0.0f};
  const float gyroAlpha = 0.3f;  // Lower = more filtering
  
  applyLowPassFilter(filteredGyro[0], icm_gyro[0], gyroAlpha);
  applyLowPassFilter(filteredGyro[1], icm_gyro[1], gyroAlpha);
  applyLowPassFilter(filteredGyro[2], icm_gyro[2], gyroAlpha);
  
  // Calculate gyro magnitude from filtered values
  gyroMagnitude = sqrt(sq(filteredGyro[0]) + sq(filteredGyro[1]) + sq(filteredGyro[2]));
  
  // Calculate accel magnitude with low-pass filter
  float accelMagnitude = sqrt(sq(icm_accel[0]) + sq(icm_accel[1]) + sq(icm_accel[2]));
  static float filteredAccelMagnitude = accelMagnitude;
  const float accelAlpha = 0.3f;
  
  applyLowPassFilter(filteredAccelMagnitude, accelMagnitude, accelAlpha);
  
  // Check variance in filtered acceleration magnitude
  accelVariance = abs(filteredAccelMagnitude - accelMagnitudePrev);
  accelMagnitudePrev = filteredAccelMagnitude;
  
  // Increment or reset counters based on current measurements
  if ((gyroMagnitude < GYRO_THRESHOLD) && (accelVariance < ACCEL_VARIANCE_THRESHOLD)) {
    stationaryCounter++;
    movingCounter = 0;
  } else {
    movingCounter++;
    stationaryCounter = 0;
  }
  
  // Determine if stationary based on counters (hysteresis)
  bool wasStationary = isStationary;
  
  // Only transition states after multiple consistent readings
  if (isStationary && movingCounter > STATE_CHANGE_THRESHOLD) {
    isStationary = false;
  } else if (!isStationary && stationaryCounter > STATE_CHANGE_THRESHOLD) {
    isStationary = true;
  }
  
  // Adjust beta based on motion state
  if (isStationary) {
    beta = BETA_STATIONARY; // Lower beta when stationary for less drift
    
    // Update gyro bias estimate when stationary
    if (gyroMagnitude < GYRO_THRESHOLD) {
      gyroBias[0] += GYRO_BIAS_LEARN_RATE * (icm_gyro[0] - gyroBias[0]);
      gyroBias[1] += GYRO_BIAS_LEARN_RATE * (icm_gyro[1] - gyroBias[1]);
      gyroBias[2] += GYRO_BIAS_LEARN_RATE * (icm_gyro[2] - gyroBias[2]);
    }
  } else {
    beta = BETA_MOTION;     // Higher beta when moving for more responsiveness
  }
  
  // Print state change for debugging
  if (wasStationary != isStationary) {
    Serial.print("Motion state changed to: ");
    Serial.println(isStationary ? "STATIONARY" : "MOVING");
    Serial.print("Gyro magnitude: ");
    Serial.print(gyroMagnitude, 5);
    Serial.print(" | Accel variance: ");
    Serial.println(accelVariance, 5);
  }
}

// Calculate orientation using Madgwick filter with drift compensation
void calculateOrientation() {
  // Calculate time delta
  uint32_t now = micros();
  float dt = (now - lastUpdateTime) / 1000000.0f; // Convert to seconds
  lastUpdateTime = now;
  
  // Update sample frequency based on actual time delta
  if (dt > 0) {
    sampleFreq = 1.0f / dt;
  }
  
  // Skip this iteration if sensor data isn't valid or dt is too large/small
  if ((icm_accel[0] == 0.0f && icm_accel[1] == 0.0f && icm_accel[2] == 0.0f) ||
      dt <= 0 || dt > 0.5) { // Skip if dt is unreasonable
    return;
  }
  
  // Compensate for gyro bias
  float biasCompensatedGyro[3];
  biasCompensatedGyro[0] = icm_gyro[0] - gyroBias[0];
  biasCompensatedGyro[1] = icm_gyro[1] - gyroBias[1];
  biasCompensatedGyro[2] = icm_gyro[2] - gyroBias[2];
  
  // Temporary quaternion for calculations
  float q0 = icm_q0, q1 = icm_q1, q2 = icm_q2, q3 = icm_q3;
  
  // Madgwick filter implementation
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
  float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  
  // Rate of change of quaternion from gyroscope (using bias-compensated gyro)
  qDot1 = 0.5f * (-q1 * biasCompensatedGyro[0] - q2 * biasCompensatedGyro[1] - q3 * biasCompensatedGyro[2]);
  qDot2 = 0.5f * (q0 * biasCompensatedGyro[0] + q2 * biasCompensatedGyro[2] - q3 * biasCompensatedGyro[1]);
  qDot3 = 0.5f * (q0 * biasCompensatedGyro[1] - q1 * biasCompensatedGyro[2] + q3 * biasCompensatedGyro[0]);
  qDot4 = 0.5f * (q0 * biasCompensatedGyro[2] + q1 * biasCompensatedGyro[1] - q2 * biasCompensatedGyro[0]);
  
  // If magnetometer measurement is valid, incorporate it
  if (!((icm_mag[0] == 0.0f) && (icm_mag[1] == 0.0f) && (icm_mag[2] == 0.0f))) {
    // Normalize accelerometer measurement
    recipNorm = 1.0f / sqrt(icm_accel[0] * icm_accel[0] + icm_accel[1] * icm_accel[1] + icm_accel[2] * icm_accel[2]);
    icm_accel[0] *= recipNorm;
    icm_accel[1] *= recipNorm;
    icm_accel[2] *= recipNorm;
    
    // Normalize magnetometer measurement
    recipNorm = 1.0f / sqrt(icm_mag[0] * icm_mag[0] + icm_mag[1] * icm_mag[1] + icm_mag[2] * icm_mag[2]);
    icm_mag[0] *= recipNorm;
    icm_mag[1] *= recipNorm;
    icm_mag[2] *= recipNorm;
    
    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * icm_mag[0];
    _2q0my = 2.0f * q0 * icm_mag[1];
    _2q0mz = 2.0f * q0 * icm_mag[2];
    _2q1mx = 2.0f * q1 * icm_mag[0];
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
    hx = icm_mag[0] * q0q0 - _2q0my * q3 + _2q0mz * q2 + icm_mag[0] * q1q1 + _2q1 * icm_mag[1] * q2 + _2q1 * icm_mag[2] * q3 - icm_mag[0] * q2q2 - icm_mag[0] * q3q3;
    hy = _2q0mx * q3 + icm_mag[1] * q0q0 - _2q0mz * q1 + _2q1mx * q2 - icm_mag[1] * q1q1 + icm_mag[1] * q2q2 + _2q2 * icm_mag[2] * q3 - icm_mag[1] * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + icm_mag[2] * q0q0 + _2q1mx * q3 - icm_mag[2] * q1q1 + _2q2 * icm_mag[1] * q3 - icm_mag[2] * q2q2 + icm_mag[2] * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;
    
    // Gradient descent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - icm_accel[0]) + _2q1 * (2.0f * q0q1 + _2q2q3 - icm_accel[1]) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - icm_mag[0]) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - icm_mag[1]) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - icm_mag[2]);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - icm_accel[0]) + _2q0 * (2.0f * q0q1 + _2q2q3 - icm_accel[1]) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - icm_accel[2]) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - icm_mag[0]) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - icm_mag[1]) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - icm_mag[2]);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - icm_accel[0]) + _2q3 * (2.0f * q0q1 + _2q2q3 - icm_accel[1]) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - icm_accel[2]) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - icm_mag[0]) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - icm_mag[1]) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - icm_mag[2]);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - icm_accel[0]) + _2q2 * (2.0f * q0q1 + _2q2q3 - icm_accel[1]) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - icm_mag[0]) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - icm_mag[1]) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - icm_mag[2]);
    
    // Normalize step magnitude
    recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    
    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  } else {
    // Just use accelerometer without magnetometer
    float ax = icm_accel[0];
    float ay = icm_accel[1];
    float az = icm_accel[2];
    
    // Normalize accelerometer measurement
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // Estimated direction of gravity
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    // Error is cross product between estimated and measured direction of gravity
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);
    
    // Apply proportional feedback
    qDot1 -= beta * ex;
    qDot2 -= beta * ey;
    qDot3 -= beta * ez;
  }
  
  // Integrate rate of change of quaternion
  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;
  
  // Normalize quaternion
  normalizeQuaternion(q0, q1, q2, q3);
  
  // Apply drift compensation for both states, but more aggressively when stationary
  float compensationFactor = isStationary ? DRIFT_COMPENSATION_FACTOR : 0.5f;
  
  // Only apply drift compensation if we've initialized previous values
  if (q0_prev != 0.0f || q1_prev != 0.0f || q2_prev != 0.0f || q3_prev != 0.0f) {
    // Weighted average between current and previous quaternion
    q0 = q0 * (1.0f - compensationFactor) + q0_prev * compensationFactor;
    q1 = q1 * (1.0f - compensationFactor) + q1_prev * compensationFactor;
    q2 = q2 * (1.0f - compensationFactor) + q2_prev * compensationFactor;
    q3 = q3 * (1.0f - compensationFactor) + q3_prev * compensationFactor;
    
    // Re-normalize after dampening
    normalizeQuaternion(q0, q1, q2, q3);
  }
  
  // Store current values as previous for next iteration
  q0_prev = q0;
  q1_prev = q1;
  q2_prev = q2;
  q3_prev = q3;
  
  // Update global quaternion values
  icm_q0 = q0;
  icm_q1 = q1;
  icm_q2 = q2;
  icm_q3 = q3;
}

// Read data from the ICM-20948 sensor
void ICM_20948_read() {
  // Check if data is ready
  if (myICM.dataReady()) {
    // Get the data
    myICM.getAGMT();
    
    // Update global variables with new data
    icm_accel[0] = myICM.accX() / 1000.0;  // Convert from mg to g
    icm_accel[1] = myICM.accY() / 1000.0;
    icm_accel[2] = myICM.accZ() / 1000.0;
    icm_gyro[0] = myICM.gyrX() * (PI / 180.0);  // Convert to radians/s
    icm_gyro[1] = myICM.gyrY() * (PI / 180.0);
    icm_gyro[2] = myICM.gyrZ() * (PI / 180.0);
    icm_mag[0] = myICM.magX();
    icm_mag[1] = myICM.magY();
    icm_mag[2] = myICM.magZ();
    icm_temp = myICM.temp();
    
    // Apply magnetometer calibration
    applyMagnetometerCalibration();
    
    // Detect if we're stationary
    detectStationary();
    
    // Calculate orientation
    calculateOrientation();
    
    // Print quaternion values only occasionally to reduce serial traffic
    // and only if quaternion debug is enabled
    static unsigned long lastPrintTime = 0;
    if (enableQuaternionDebug && (millis() - lastPrintTime > 1000)) { // Print only once per second
      lastPrintTime = millis();
      
      Serial.print("Quaternion: ");
      Serial.print(icm_q0, 4);
      Serial.print(", ");
      Serial.print(icm_q1, 4);
      Serial.print(", ");
      Serial.print(icm_q2, 4);
      Serial.print(", ");
      Serial.print(icm_q3, 4);
      Serial.print(" | State: ");
      Serial.print(isStationary ? "STATIONARY" : "MOVING");
      
      // Also print current gyro bias estimates
      Serial.print(" | Bias X:");
      Serial.print(gyroBias[0], 5);
      Serial.print(" Y:");
      Serial.print(gyroBias[1], 5);
      Serial.print(" Z:");
      Serial.println(gyroBias[2], 5);
    }
  } else {
    // Don't print this message every time - it floods the serial port
    // and only print if sensor debug is enabled
    static unsigned long lastErrorTime = 0;
    if (enableSensorDebug && (millis() - lastErrorTime > 5000)) { // Only print every 5 seconds
      lastErrorTime = millis();
      Serial.println("No data ready from ICM-20948");
    }
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
  
  // Only print quaternion data if quaternion debug is enabled
  if (enableQuaternionDebug) {
    Serial.print("  Quaternion: W:");
    Serial.print(icm_q0, 4);
    Serial.print(" X:");
    Serial.print(icm_q1, 4);
    Serial.print(" Y:");
    Serial.print(icm_q2, 4);
    Serial.print(" Z:");
    Serial.print(icm_q3, 4);
    Serial.print(" (");
    Serial.print(isStationary ? "STATIONARY" : "MOVING");
    Serial.println(")");
  }
} 