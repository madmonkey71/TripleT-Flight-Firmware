#include <Arduino.h>
#include "kx134_functions.h" // Click here to get the library: http://librarymanager/All#SparkFun_KX13X

// Global variable definitions
SparkFun_KX134 kxAccel;
outputData kx134AccelData;
float kx134_accel[3] = {0, 0, 0};  // Initialize array to zeros

void kx134_init(){
  if (kxAccel.softwareReset())
    Serial.println("Reset.");
  // Give some time for the accelerometer to reset.
  // It needs two, but give it five for good measure.
  delay(50);
  // Many settings for KX13X can only be
  // applied when the accelerometer is powered down.
  // However there are many that can be changed "on-the-fly"
  // check datasheet for more info, or the comments in the
  // "...regs.h" file which specify which can be changed when.
  // So we disable the accelerometer
  kxAccel.enableAccel(false);
  // Do stuff
  // kxAccel.setRange(SFE_KX132_RANGE16G); // 16g Range
  kxAccel.setRange(SFE_KX134_RANGE64G);         // 64g for the KX134
  kxAccel.enableDataEngine(); // Enables the bit that indicates data is ready.
  kxAccel.setOutputDataRate(50); // Default is 50Hz
  // Re-enable the accelerometer
  kxAccel.enableAccel(); 
  kxAccel.begin();
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