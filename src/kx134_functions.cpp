#include <Arduino.h>
#include "kx134_functions.h" // Click here to get the library: http://librarymanager/All#SparkFun_KX13X

// Global variable definitions
SparkFun_KX134 kxAccel;
outputData kx134AccelData;
float kx134_accel[3] = {0, 0, 0};  // Initialize array to zeros

bool kx134_init() {
    Serial.println("Initializing KX134 accelerometer...");
    
    // First try to begin communication
    if (!kxAccel.begin()) {
        Serial.println("KX134 initialization failed - could not communicate with device");
        return false;
    }
    
    // Perform software reset
    if (!kxAccel.softwareReset()) {
        Serial.println("KX134 software reset failed");
        return false;
    }
    
    // Wait for reset to complete
    delay(50);
    
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
    
    // Configure data rate
    if (!kxAccel.setOutputDataRate(50)) {
        Serial.println("Failed to set output data rate");
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

void kx134_print(){
  Serial.print(F("  KX134: X:"));
  Serial.print(kx134_accel[0], 2);
  Serial.print(F("g Y:"));
  Serial.print(kx134_accel[1], 2);
  Serial.print(F("g Z:"));
  Serial.print(kx134_accel[2], 2);
  Serial.println(F("g"));
} 