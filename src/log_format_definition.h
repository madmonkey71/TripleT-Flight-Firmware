#ifndef LOG_FORMAT_DEFINITION_H
#define LOG_FORMAT_DEFINITION_H

#include "data_structures.h" // For LogData struct definition
#include <stddef.h>         // For offsetof

// Enum to define the type of data for logging, including scaling and precision
typedef enum {
    TYPE_UINT32,                            // uint32_t, format as %lu
    TYPE_UINT8,                             // uint8_t, format as %u
    TYPE_INT32_AS_FLOAT_SCALED_1E7_P6,      // int32_t (degrees * 1e7), format as float with 6 decimal places
    TYPE_INT32_AS_FLOAT_SCALED_1E3_P2,      // int32_t (mm or mm/s), format as float (m or m/s) with 2 decimal places
    TYPE_INT32_AS_FLOAT_SCALED_1E5_P2,      // int32_t (degrees * 1e5), format as float with 2 decimal places
    TYPE_UINT16_AS_FLOAT_SCALED_1E2_P2,     // uint16_t (value * 100), format as float with 2 decimal places
    TYPE_FLOAT_P2,                          // float, format with 2 decimal places
    TYPE_FLOAT_P4                           // float, format with 4 decimal places
} LogFieldType_e;

// Struct to describe each column in the log file
typedef struct {
    const char* name;       // Column header name
    LogFieldType_e type;    // Data type and formatting rule
    size_t offset;          // Offset of the field within LogData struct
} LogColumnDescriptor_t;

// External declaration of the log columns array and its count
// The actual definition will be in a .cpp file
extern const LogColumnDescriptor_t LOG_COLUMNS[];
extern const size_t LOG_COLUMN_COUNT;

#endif // LOG_FORMAT_DEFINITION_H
