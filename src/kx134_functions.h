#ifndef KX134_FUNCTIONS_H
#define KX134_FUNCTIONS_H

#include <Wire.h>
#include <SparkFun_KX13X.h>

// Global variables
extern SparkFun_KX134 kxAccel;
extern outputData kx134AccelData;
extern float kx134_x;
extern float kx134_y;
extern float kx134_z;

// Function declarations
void kx134_init();
void kx134_read();
void kx134_print();

#endif // KX134_FUNCTIONS_H 