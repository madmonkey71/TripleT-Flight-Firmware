#pragma once

#include <cstdint> // for fixed-width types

// --- Timing Intervals (milliseconds) ---

// Logging
static constexpr uint32_t LOG_INTERVAL               = 100;   // Rate for attempting to log data
static constexpr uint32_t SD_MAX_OPEN_TIME           = 500;   // Max time to keep log file open between writes

// Sensor Polling
static constexpr uint32_t GPS_POLL_INTERVAL          = 100;   // Poll GPS at 10Hz
static constexpr uint32_t IMU_POLL_INTERVAL          = 100;   // Poll IMU (ICM) at 10Hz
static constexpr uint32_t BARO_POLL_INTERVAL         = 100;   // Poll barometer (MS5611) at 10Hz
static constexpr uint32_t ACCEL_POLL_INTERVAL        = 100;   // Poll accelerometer (KX134) at 10Hz

// Periodic Checks
static constexpr uint32_t DISPLAY_INTERVAL           = 100;   // Update display/serial CSV at 10Hz (if enabled)
static constexpr uint32_t GPS_CHECK_INTERVAL         = 10000; // Check GPS connection status
static constexpr uint32_t STORAGE_CHECK_INTERVAL     = 30000; // Check storage space
static constexpr uint32_t EEPROM_UPDATE_INTERVAL     = 60000; // Save state to EEPROM

// State Machine & Processing
static constexpr uint32_t STATE_TIMEOUT_MS           = 30000; // Timeout for critical flight states
static constexpr uint32_t UKF_PROCESS_INTERVAL       = 50;    // Process UKF data at 20Hz

// Flight Logic Related (Consider moving to a flight_config.h later if needed)
// BACKUP_APOGEE_TIME has been moved to config.h
static constexpr uint32_t EXPECTED_APOGEE_TIME       = 10000; // Expected time from liftoff to apogee (ms) 

// --- Physical Constants ---
static constexpr float STANDARD_SEA_LEVEL_PRESSURE  = 1013.25f; // Standard sea level pressure in hPa (millibars) 