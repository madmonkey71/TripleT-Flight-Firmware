#include "log_format_definition.h"
#include "data_structures.h" // For offsetof and LogData
#include <stddef.h>         // For offsetof

// Definition of the log columns array
// This array is the single source of truth for the CSV log format.
// Order in this array defines CSV column order.
const LogColumnDescriptor_t LOG_COLUMNS[] = {
    {"SeqNum", TYPE_UINT32, offsetof(LogData, seqNum)},
    {"Timestamp", TYPE_UINT32, offsetof(LogData, timestamp)},
    {"FixType", TYPE_UINT8, offsetof(LogData, fixType)},
    {"Sats", TYPE_UINT8, offsetof(LogData, sats)},
    {"Lat", TYPE_INT32_AS_FLOAT_SCALED_1E7_P6, offsetof(LogData, latitude)},
    {"Long", TYPE_INT32_AS_FLOAT_SCALED_1E7_P6, offsetof(LogData, longitude)},
    {"Alt", TYPE_INT32_AS_FLOAT_SCALED_1E3_P2, offsetof(LogData, altitude)},
    {"AltMSL", TYPE_INT32_AS_FLOAT_SCALED_1E3_P2, offsetof(LogData, altitudeMSL)},
    {"RawAltitude", TYPE_FLOAT_P2, offsetof(LogData, raw_altitude)},
    {"CalibratedAltitude", TYPE_FLOAT_P2, offsetof(LogData, calibrated_altitude)},
    {"Speed", TYPE_INT32_AS_FLOAT_SCALED_1E3_P2, offsetof(LogData, speed)},
    {"Heading", TYPE_INT32_AS_FLOAT_SCALED_1E5_P2, offsetof(LogData, heading)},
    {"pDOP", TYPE_UINT16_AS_FLOAT_SCALED_1E2_P2, offsetof(LogData, pDOP)},
    {"RTK", TYPE_UINT8, offsetof(LogData, rtk)},
    {"Pressure", TYPE_FLOAT_P2, offsetof(LogData, pressure)},
    {"Temperature", TYPE_FLOAT_P2, offsetof(LogData, temperature)},
    {"KX134_AccelX", TYPE_FLOAT_P4, offsetof(LogData, kx134_accel[0])},
    {"KX134_AccelY", TYPE_FLOAT_P4, offsetof(LogData, kx134_accel[1])},
    {"KX134_AccelZ", TYPE_FLOAT_P4, offsetof(LogData, kx134_accel[2])},
    {"ICM_AccelX", TYPE_FLOAT_P4, offsetof(LogData, icm_accel[0])},
    {"ICM_AccelY", TYPE_FLOAT_P4, offsetof(LogData, icm_accel[1])},
    {"ICM_AccelZ", TYPE_FLOAT_P4, offsetof(LogData, icm_accel[2])},
    {"ICM_GyroX", TYPE_FLOAT_P4, offsetof(LogData, icm_gyro[0])},
    {"ICM_GyroY", TYPE_FLOAT_P4, offsetof(LogData, icm_gyro[1])},
    {"ICM_GyroZ", TYPE_FLOAT_P4, offsetof(LogData, icm_gyro[2])},
    {"ICM_MagX", TYPE_FLOAT_P4, offsetof(LogData, icm_mag[0])},
    {"ICM_MagY", TYPE_FLOAT_P4, offsetof(LogData, icm_mag[1])},
    {"ICM_MagZ", TYPE_FLOAT_P4, offsetof(LogData, icm_mag[2])},
    {"ICM_Temp", TYPE_FLOAT_P2, offsetof(LogData, icm_temp)}
};

// Definition of the log column count
const size_t LOG_COLUMN_COUNT = sizeof(LOG_COLUMNS) / sizeof(LogColumnDescriptor_t);
