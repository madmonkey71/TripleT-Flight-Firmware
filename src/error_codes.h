#ifndef ERROR_CODES_H
#define ERROR_CODES_H

#include <stdint.h>

typedef enum : uint8_t {
    NO_ERROR = 0,

    // Sensor Initialization Failures (10-29)
    SENSOR_INIT_FAIL_MS5611 = 10,
    SENSOR_INIT_FAIL_ICM20948 = 11,
    SENSOR_INIT_FAIL_KX134 = 12,
    SENSOR_INIT_FAIL_GPS = 13,

    // Sensor Read/Timeout Failures (30-49)
    // Specific timeouts might be harder to pinpoint to a single code here,
    // often they might result in isSensorSuiteHealthy returning false.
    // These could be used if a direct read failure is detected.
    SENSOR_READ_FAIL_MS5611 = 30,
    SENSOR_READ_FAIL_ICM20948 = 31,
    SENSOR_READ_FAIL_KX134 = 32,
    SENSOR_READ_FAIL_GPS = 33, // e.g., PVT query fails repeatedly

    // SD Card / Logging Failures (50-59)
    SD_CARD_INIT_FAIL = 50,
    SD_CARD_MOUNT_FAIL = 51, // If begin() fails after physical presence detected
    LOG_FILE_CREATE_FAIL = 52,
    SD_CARD_WRITE_FAIL = 53,
    SD_CARD_LOW_SPACE = 54, // Warning, might not be a critical error leading to ERROR state

    // Calibration Failures (60-69)
    BARO_CALIBRATION_FAIL_NO_GPS = 60, // Baro cal requested but GPS unavailable
    BARO_CALIBRATION_FAIL_TIMEOUT = 61, // Baro cal started but timed out waiting for good GPS
    MAG_CALIBRATION_LOAD_FAIL = 62, // Failed to load mag calibration from EEPROM (e.g. bad signature)


    // Flight Logic / State Machine Issues (70-79)
    STATE_TRANSITION_INVALID_HEALTH = 70, // isSensorSuiteHealthy returned false for a critical transition
    ARM_FAIL_HEALTH_CHECK = 71,           // Specifically when 'arm' command fails health check

    // EEPROM Issues (80-89)
    EEPROM_SIGNATURE_INVALID = 80, // EEPROM data for state recovery is invalid
    // EEPROM_WRITE_FAIL (hard to detect reliably without read-back verification)

    // System Level / Unknown (250-255)
    CONFIG_ERROR_MAIN_PARACHUTE = 250, // e.g. MAIN_PRESENT is false

    UNKNOWN_ERROR = 255
} ErrorCode_t;

#endif // ERROR_CODES_H
