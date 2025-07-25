#ifndef KX134_FUNCTIONS_H
#define KX134_FUNCTIONS_H

#include <Arduino.h>
#include <SparkFun_KX13X.h>

// Function declarations
bool kx134_init();
void kx134_read();
// void kx134_print(); // REMOVED as unused
// void kx134_calibrate(); // REMOVED as unused
void kx134_get_accel(float* accel); // Restoring

// Global variable declarations
extern SparkFun_KX134 kxAccel;
extern outputData kx134AccelData;
extern float kx134_accel[3];  // Array for x, y, z acceleration values

#endif // KX134_FUNCTIONS_H 