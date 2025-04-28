#include <Arduino.h>
#include "kx134_functions.h" // Click here to get the library: http://librarymanager/All#SparkFun_KX13X
#include "config.h" // <<< ADDED Include for config parameters (potentially needed later)

// Global variable definitions
SparkFun_KX134 kxAccel;
outputData kx134AccelData;
float kx134_accel[3] = {0, 0, 0};  // Initialize array to zeros

// Add reference to debug flag
extern bool enableSensorDebug;

bool kx134_init() {
    Serial.println("Initializing KX134 accelerometer...");
    
    // First try to begin communication - Let library try default addresses (0x1E, 0x1F)
    if (!kxAccel.begin()) { 
        Serial.println("KX134 initialization failed - could not communicate with device on default addresses");
        return false;
    }
    // Serial.print("KX134 Found at address: 0x"); // Can't access i2cAddr directly
    // Serial.println(kxAccel.i2cAddr, HEX);
    
    // Perform software reset
    if (!kxAccel.softwareReset()) {
        Serial.println("KX134 software reset failed");
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
  // MODIFIED: Skip the dataReady check that's causing hangs
  // if (kxAccel.dataReady())
  // {
    // Always try to read - the library will handle errors internally
    kxAccel.getAccelData(&kx134AccelData);
    kx134_accel[0] = kx134AccelData.xData;
    kx134_accel[1] = kx134AccelData.yData;
    kx134_accel[2] = kx134AccelData.zData;
  // }
}

void kx134_print(){
  // Only print if sensor debug is enabled
  if (!enableSensorDebug) return;
  
  Serial.print(F("  KX134: X:"));
  Serial.print(kx134_accel[0], 2);
  Serial.print(F("g Y:"));
  Serial.print(kx134_accel[1], 2);
  Serial.print(F("g Z:"));
  Serial.print(kx134_accel[2], 2);
  Serial.print(F("g"));
} 