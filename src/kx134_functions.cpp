#include <Arduino.h>
#include "kx134_functions.h" // Click here to get the library: http://librarymanager/All#SparkFun_KX13X

// Global variable definitions
SparkFun_KX134 kxAccel;
outputData kx134AccelData;
float kx134_accel[3] = {0, 0, 0};  // Initialize array to zeros

// Add reference to debug flag
extern bool enableSensorDebug;

#include "error_codes.h" // For ErrorCode_t
extern ErrorCode_t g_last_error_code; // For setting error codes

bool kx134_init() {
    Serial.println("Initializing KX134 accelerometer...");
    
    // First try to begin communication
    if (!kxAccel.begin()) {
        Serial.println("KX134 initialization failed - could not communicate with device");
        g_last_error_code = SENSOR_INIT_FAIL_KX134;
        return false;
    }
    
    // Perform software reset
    if (!kxAccel.softwareReset()) {
        Serial.println("KX134 software reset failed");
        g_last_error_code = SENSOR_INIT_FAIL_KX134; // Could be a more specific error if desired
        return false;
    }
    
    // Wait for reset to complete
    delay(5);
    
    // Disable accelerometer for configuration
    if (!kxAccel.enableAccel(false)) {
        Serial.println("Failed to disable accelerometer for configuration");
        return false;
    }
    
    // Configure range
    if (!kxAccel.setRange(SFE_KX134_RANGE64G)) {
        Serial.println("Failed to set accelerometer range");
        return false;
    }

    // Enable data engine
    if (!kxAccel.enableDataEngine()) {
        Serial.println("Failed to enable data engine");
        return false;
    }
    
    // Re-enable accelerometer
    if (!kxAccel.enableAccel(true)) {
        Serial.println("Failed to re-enable accelerometer");
        return false;
    }
    
    Serial.println("KX134 initialization successful");
    return true;
}

void kx134_read(){
  // Query the KX134
  if (kxAccel.dataReady())
  {
    kxAccel.getAccelData(&kx134AccelData);
    kx134_accel[0] = kx134AccelData.xData;
    kx134_accel[1] = kx134AccelData.yData;
    kx134_accel[2] = kx134AccelData.zData;
  }
}

// void kx134_print(){ // REMOVED as unused
//   // Only print if sensor debug is enabled
//   if (!enableSensorDebug) return;
//
//   Serial.print(F("  KX134: X:"));
//   Serial.print(kx134_accel[0], 2);
//   Serial.print(F("g Y:"));
//   Serial.print(kx134_accel[1], 2);
//   Serial.print(F("g Z:"));
//   Serial.print(kx134_accel[2], 2);
//   Serial.print(F("g"));
// }

void kx134_get_accel(float* accel) {
  memcpy(accel, kx134_accel, sizeof(float) * 3);
}