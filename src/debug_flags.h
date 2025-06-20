#ifndef DEBUG_FLAGS_H
#define DEBUG_FLAGS_H

#include <stdint.h> // For bool if not implicitly available via Arduino.h

typedef struct {
    volatile bool enableSerialCSV;
    volatile bool enableSystemDebug;
    volatile bool enableIMUDebug;
    volatile bool enableGPSDebug;
    volatile bool enableBaroDebug;
    volatile bool enableStorageDebug;
    volatile bool enableICMRawDebug;
    volatile bool enableStatusSummary;
    volatile bool displayMode;
    volatile bool enableSensorDebug;    // For 'sd' command, legacy
    volatile bool enableDetailedOutput; // General detailed output not tied to a specific system
} DebugFlags;

#endif // DEBUG_FLAGS_H
