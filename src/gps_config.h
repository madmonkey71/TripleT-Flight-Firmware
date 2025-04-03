#ifndef GPS_CONFIG_H
#define GPS_CONFIG_H

// GPS Debugging Configuration
// --------------------------
// Set GPS_DEBUG_ENABLED to 1 to enable detailed debugging output to the serial monitor
// Set to 0 for normal operation with minimal debugging
#define GPS_DEBUG_ENABLED  0   // Set to 1 to enable GPS debugging output, 0 to disable

// Set GPS_DEBUG_VERBOSE to 1 for even more detailed debugging, including raw UBX messages
// Only takes effect if GPS_DEBUG_ENABLED is also set to 1
#define GPS_DEBUG_VERBOSE  0   // Set to 1 for extremely detailed UBX message debugging

#endif // GPS_CONFIG_H 