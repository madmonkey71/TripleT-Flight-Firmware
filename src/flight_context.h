#ifndef FLIGHT_CONTEXT_H
#define FLIGHT_CONTEXT_H

#include "data_structures.h" // For FlightState
#include "gps_functions.h"   // For SFE_UBLOX_GNSS
#include "ms5611_functions.h"// For MS5611 object type
#include <Adafruit_NeoPixel.h> // For Adafruit_NeoPixel (needed by functions that will take it directly)
#include <SdFat.h>           // For SdFat (needed by functions that will take it directly)


// This context holds status information primarily read by the command processor,
// and pointers to flags that it can modify.
typedef struct {
    // Status Flags (mostly read-only for command processor)
    bool sdCardAvailable;
    bool sdCardMounted;
    bool sdCardPresent;
    bool loggingEnabled;
    char* logFileName; // Changed from const char*
    uint64_t availableSpace;

    bool flashAvailable;
    bool baroCalibrated; // This will be changed to a pointer if performCalibration is refactored to update it via context
    bool icm20948_ready;
    bool ms5611_initialized_ok;
    bool kx134_initialized_ok;

    // Pointers to allow modification of these specific flags by command_processor
    bool* useMadgwickFilter_ptr;
    bool* useKalmanFilter_ptr;

    // References to global objects needed by some status commands (e.g. printSystemStatus)
    SFE_UBLOX_GNSS& myGNSS_ref;
    MS5611& ms5611Sensor_ref;
    // SparkFun_KX134& kx134Accel_ref; // Not strictly needed by status funcs yet

} SystemStatusContext;

#endif // FLIGHT_CONTEXT_H
