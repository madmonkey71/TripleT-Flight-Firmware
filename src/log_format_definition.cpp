#include "log_format_definition.h"
#include "data_structures.h" // For offsetof and LogData
#include <stddef.h>         // For offsetof

// Definition of the log columns array
// This array is the single source of truth for the CSV log format.
// Order in this array defines CSV column order.
const LogColumnDescriptor_t LOG_COLUMNS[] = {
    {"SeqNum", TYPE_UINT32, offsetof(LogData, seqNum)},
    {"Timestamp", TYPE_UINT32, offsetof(LogData, timestamp)},
    {"FlightState", TYPE_UINT8, offsetof(LogData, flightState)},
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
    {"ICM_Temp", TYPE_FLOAT_P2, offsetof(LogData, icm_temp)},

    // AHRS Data
    {"Q0", TYPE_FLOAT_P6_RAD, offsetof(LogData, q0)},
    {"Q1", TYPE_FLOAT_P6_RAD, offsetof(LogData, q1)},
    {"Q2", TYPE_FLOAT_P6_RAD, offsetof(LogData, q2)},
    {"Q3", TYPE_FLOAT_P6_RAD, offsetof(LogData, q3)},
    {"EulerRoll_rad", TYPE_FLOAT_P6_RAD, offsetof(LogData, euler_roll)},
    {"EulerPitch_rad", TYPE_FLOAT_P6_RAD, offsetof(LogData, euler_pitch)},
    {"EulerYaw_rad", TYPE_FLOAT_P6_RAD, offsetof(LogData, euler_yaw)},
    {"GyroBiasX_rps", TYPE_FLOAT_P6_RAD, offsetof(LogData, gyro_bias_x)},
    {"GyroBiasY_rps", TYPE_FLOAT_P6_RAD, offsetof(LogData, gyro_bias_y)},
    {"GyroBiasZ_rps", TYPE_FLOAT_P6_RAD, offsetof(LogData, gyro_bias_z)},

    // Guidance and Control - Target, Integral, and Actuator Outputs
    {"TgtRoll", TYPE_FLOAT_P6_RAD, offsetof(LogData, target_roll)},
    {"TgtPitch", TYPE_FLOAT_P6_RAD, offsetof(LogData, target_pitch)},
    {"TgtYaw", TYPE_FLOAT_P6_RAD, offsetof(LogData, target_yaw)},
    {"PIDIntRoll", TYPE_FLOAT_P4, offsetof(LogData, pid_roll_integral)},
    {"PIDIntPitch", TYPE_FLOAT_P4, offsetof(LogData, pid_pitch_integral)},
    {"PIDIntYaw", TYPE_FLOAT_P4, offsetof(LogData, pid_yaw_integral)},
    {"ActuatorOutRoll", TYPE_FLOAT_P3, offsetof(LogData, actuator_output_roll)},
    {"ActuatorOutPitch", TYPE_FLOAT_P3, offsetof(LogData, actuator_output_pitch)},
    {"ActuatorOutYaw", TYPE_FLOAT_P3, offsetof(LogData, actuator_output_yaw)},
    {"BattVoltage", TYPE_FLOAT_P2, offsetof(LogData, battery_voltage)},
    {"LastErrorCode", TYPE_UINT8, offsetof(LogData, last_error_code)} // Added Last Error Code
};

// Definition of the log column count
const size_t LOG_COLUMN_COUNT = sizeof(LOG_COLUMNS) / sizeof(LogColumnDescriptor_t);
