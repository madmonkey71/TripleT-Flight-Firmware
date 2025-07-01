#include "icm_20948_functions.h"
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include "config.h" 
#include "utility_functions.h" // Added to access convertQuaternionToEuler
#include "ICM_20948.h"
#include "data_structures.h"

extern ErrorCode_t g_last_error_code; // For setting error codes

// Add extern declarations for the debug flags
extern bool enableSensorDebug;
extern bool enableICMRawDebug; // New flag to control ICM raw data output
extern bool g_icm20948_ready; // To set the global ready flag

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

// Add gyro bias estimation
float gyroBias[3] = {0.0f, 0.0f, 0.0f}; // Estimated gyro bias

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
            // Gyro data is in dps, convert to rad/s for consistency with Kalman filter and gyroBias storage
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
  #include "error_codes.h" // For ErrorCode_t

  // Initialize ICM-20948 with I2C interface
  Wire.begin(); // Ensure Wire is initialized, though it's likely done in main setup
  myICM.begin(Wire, 1); // 1 = ADO high
  
  if (myICM.status != ICM_20948_Stat_Ok) {
    Serial.println("ICM-20948 initialization failed");
    g_last_error_code = SENSOR_INIT_FAIL_ICM20948;
    g_icm20948_ready = false; // Ensure this is set if init fails
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
  
  // Set accel full scale range to +/- 16g
  myFSS.a = 3; // 16g
  
  result = myICM.setFullScale(ICM_20948_Internal_Acc, myFSS);
  if (result != ICM_20948_Stat_Ok) {
    Serial.println("Failed to set accel full scale range");
  }
  
  // Configure magnetometer
  result = myICM.startupMagnetometer();
  if (result != ICM_20948_Stat_Ok) {
    Serial.println("Failed to start magnetometer");
  }
  
  // If we get here, all configuration was successful.
  g_icm20948_ready = true;
  Serial.println("ICM-20948 initialization successful.");
  
  // Load existing magnetometer calibration from EEPROM if available
  if(icm_20948_load_calibration()){
      Serial.println("Loaded existing ICM20948 magnetometer calibration from EEPROM.");
  } else {
      Serial.println("No existing ICM20948 magnetometer calibration found in EEPROM.");
  }
  
  // Initialize variables
  isStationary = true;
  accelMagnitudePrev = 0;
  accelVariance = 0;
  gyroMagnitude = 0;
  
  lastUpdateTime = micros();
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
  static unsigned long lastErrorPrintTime = 0;
  static unsigned long lastDetailedDebugTime = 0;
  
    if (myICM.dataReady()) {
    myICM.getAGMT();  // Get the latest data
    
    // Store raw magnetometer data first before calibration
    icm_mag[0] = myICM.magX();
    icm_mag[1] = myICM.magY();
    icm_mag[2] = myICM.magZ();
    
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
    
    // Magnetometer data is already stored and calibrated (icm_mag global updated by applyMagnetometerCalibration)
    // icm_mag[0] = myICM.magX(); // This line is now redundant as it's done before calibration
    // icm_mag[1] = myICM.magY(); // This line is now redundant
    // icm_mag[2] = myICM.magZ(); // This line is now redundant

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
    
    // Print detailed raw sensor data and ICM debug info every second for debugging
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
      Serial.println();


      // Note: ICM quaternion values (icm_q0, icm_q1, icm_q2, icm_q3) are still maintained
      // for compatibility but Kalman filter is the primary orientation source
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

void icm_20948_get_accel(float* accel) {
  memcpy(accel, icm_accel, sizeof(float) * 3);
}
// void icm_20948_get_gyro(float* gyro) { // REMOVED as unused }

// Function to get calibrated gyroscope data (raw - bias)
void ICM_20948_get_calibrated_gyro(float out_gyro[3]) {
  // Assumes icm_gyro contains the latest raw (but rad/s converted) gyro data
  // and gyroBias contains the calibrated bias values from ICM_20948_calibrate_gyro_bias()
  out_gyro[0] = icm_gyro[0] - gyroBias[0];
  out_gyro[1] = icm_gyro[1] - gyroBias[1];
  out_gyro[2] = icm_gyro[2] - gyroBias[2];
}

bool icm_20948_get_mag(float* mag) {
  if (mag != nullptr) {
    memcpy(mag, icm_mag, sizeof(float) * 3);
    return true;
  }
  return false;
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

void ICM_20948_calibrate_mag_interactive() {
    // Implementation of ICM_20948_calibrate_mag_interactive function
}

bool icm_20948_save_calibration() {
    Serial.println(F("Saving magnetometer calibration to EEPROM..."));
    uint32_t magic = MAG_CAL_MAGIC_NUMBER;
    EEPROM.put(MAG_CAL_EEPROM_ADDR, magic);
    EEPROM.put(MAG_CAL_EEPROM_ADDR + sizeof(uint32_t), magBias);
    EEPROM.put(MAG_CAL_EEPROM_ADDR + sizeof(uint32_t) + sizeof(magBias), magScale);
    Serial.println(F("Save complete."));
    return true;
}

bool icm_20948_load_calibration() {
    Serial.println(F("Attempting to load magnetometer calibration from EEPROM..."));
    uint32_t magic;
    EEPROM.get(MAG_CAL_EEPROM_ADDR, magic);

    if (magic == MAG_CAL_MAGIC_NUMBER) {
        Serial.println(F("Valid magic number found. Loading calibration data."));
        EEPROM.get(MAG_CAL_EEPROM_ADDR + sizeof(uint32_t), magBias);
        EEPROM.get(MAG_CAL_EEPROM_ADDR + sizeof(uint32_t) + sizeof(magBias), magScale);

        Serial.print(F("Loaded Bias: "));
        Serial.print(magBias[0]); Serial.print(F(", "));
        Serial.print(magBias[1]); Serial.print(F(", "));
        Serial.println(magBias[2]);
        Serial.print(F("Loaded Scale: "));
        Serial.print(magScale[0][0]); Serial.print(F(", "));
        Serial.print(magScale[0][1]); Serial.print(F(", "));
        Serial.println(magScale[0][2]);
        Serial.print(F("Loaded Scale: "));
        Serial.print(magScale[1][0]); Serial.print(F(", "));
        Serial.print(magScale[1][1]); Serial.print(F(", "));
        Serial.println(magScale[1][2]);
        Serial.print(F("Loaded Scale: "));
        Serial.print(magScale[2][0]); Serial.print(F(", "));
        Serial.print(magScale[2][1]); Serial.print(F(", "));
        Serial.println(magScale[2][2]);
        return true;
    } else {
        Serial.println(F("No valid calibration data found in EEPROM."));
        g_last_error_code = MAG_CALIBRATION_LOAD_FAIL;
        return false;
    }
}