#include "ms5611_functions.h"
#include "gps_functions.h"  // Include GPS functions for calibration

// Named constants for magic numbers used in this file
static const double HYPSOMETRIC_CONSTANT_A = 44330.0;
static const double HYPSOMETRIC_EXPONENT = 0.190295; // Approx 1/5.255
static const double HPA_TO_PA_CONVERSION_FACTOR = 100.0;
static const double MM_TO_M_CONVERSION_FACTOR = 1000.0;

static const float MIN_VALID_CALIBRATION_PRESSURE_HPA = 700.0f;
static const float MAX_VALID_CALIBRATION_PRESSURE_HPA = 1200.0f;
static const uint8_t MIN_GPS_FIX_TYPE_FOR_CALIBRATION = 3;
static const int MAX_PDOP_FOR_CALIBRATION = 300; // Represents pDOP < 3.0 (value * 100)
static const int MAX_PDOP_FOR_OFFSET_UPDATE = 200; // Represents pDOP < 2.0 (value * 100)

static const uint32_t CALIBRATION_LOOP_DELAY_MS = 200;
static const uint32_t CALIBRATION_ERROR_DELAY_MS = 500;
static const unsigned int CALIBRATION_STATUS_PRINT_INTERVAL = 5; // Print status every N attempts

static const float OFFSET_UPDATE_SMOOTHING_OLD_FACTOR = 0.8f;
static const float OFFSET_UPDATE_SMOOTHING_NEW_FACTOR = 0.2f;

static const uint32_t INIT_ERROR_DELAY_MS = 1000;
static const int INIT_STABILIZATION_READINGS = 5;
static const uint32_t INIT_STABILIZATION_DELAY_MS = 100;
static const float INIT_VALID_PRESSURE_MIN_HPA = 800.0f;
static const float INIT_VALID_PRESSURE_MAX_HPA = 1100.0f;


MS5611 ms5611Sensor(0x77);
float pressure = 0;
float temperature = 0;
float baro_altitude_offset = 0.0f;
bool baro_calibration_done = false;
bool ms5611_initialized_ok = false;

// Declare the global variable from main
extern bool baroCalibrated;
#include "error_codes.h" // For ErrorCode_t
extern ErrorCode_t g_last_error_code; // For setting error codes

// Add reference to debug flag
extern bool enableSensorDebug;
extern bool enableGPSDebug;

int ms5611_read() {
    int result = ms5611Sensor.read();
    if (result == MS5611_READ_OK) {
        pressure = ms5611Sensor.getPressure();
        temperature = ms5611Sensor.getTemperature();
    }
    return result;
}

bool ms5611_calibrate_with_gps(uint32_t timeout_ms) {
    if (baro_calibration_done) {
        return true;  // Already calibrated
    }
    
    Serial.println(F("Starting barometric calibration with GPS..."));
    Serial.print(F("Current pressure: "));
    Serial.print(pressure);
    Serial.print(F(" hPa, GPS fix type: "));
    Serial.print(GPS_fixType);
    Serial.print(F(", pDOP: "));
    Serial.println(pDOP / 100.0, 2);
    
    // Check if pressure is valid (not zero or unreasonable)
    if (pressure < MIN_VALID_CALIBRATION_PRESSURE_HPA || pressure > MAX_VALID_CALIBRATION_PRESSURE_HPA) {
        Serial.print(F("Pressure reading (")); Serial.print(pressure);
        Serial.print(F(" hPa) is invalid for calibration. Must be between "));
        Serial.print(MIN_VALID_CALIBRATION_PRESSURE_HPA); Serial.print(F(" and "));
        Serial.print(MAX_VALID_CALIBRATION_PRESSURE_HPA); Serial.println(F(" hPa"));
        
        // Try to get a fresh reading
        Serial.println(F("Attempting to get fresh pressure reading..."));
        int result = ms5611_read();
        if (result == MS5611_READ_OK) {
            Serial.print(F("New pressure reading: "));
            Serial.print(pressure);
            Serial.println(F(" hPa"));
            // Re-check the new pressure
            if (pressure < MIN_VALID_CALIBRATION_PRESSURE_HPA || pressure > MAX_VALID_CALIBRATION_PRESSURE_HPA) {
                 Serial.println(F("New pressure reading still invalid. Calibration aborted."));
                 g_last_error_code = SENSOR_READ_FAIL_MS5611; // Or a specific BARO_CAL_INVALID_PRESSURE
                 return false;
            }
        } else {
            Serial.print(F("MS5611 read error: "));
            Serial.println(result);
            g_last_error_code = SENSOR_READ_FAIL_MS5611;
            return false;
        }
    }

    uint32_t start_time = millis();
    unsigned int attempts = 0;
    
    while (millis() - start_time < timeout_ms) {
        attempts++;
        gps_read();  // Update GPS data
        
        // Check for good GPS fix and accuracy
        if (GPS_fixType >= MIN_GPS_FIX_TYPE_FOR_CALIBRATION && pDOP < MAX_PDOP_FOR_CALIBRATION) {
            // Get fresh pressure reading for calibration
            int result = ms5611_read();
            if (result != MS5611_READ_OK) {
                Serial.print(F("Failed to read pressure: "));
                Serial.println(result);
                delay(CALIBRATION_ERROR_DELAY_MS);
                continue;
            }
            
            // The pressure is in hPa but formula expects Pa
            float current_pressure_hPa = pressure;  // Use the global pressure value in hPa
            
            // Invalid pressure check
            if (current_pressure_hPa < MIN_VALID_CALIBRATION_PRESSURE_HPA || current_pressure_hPa > MAX_VALID_CALIBRATION_PRESSURE_HPA) {
                Serial.print(F("Invalid pressure reading: "));
                Serial.print(current_pressure_hPa);
                Serial.println(F(" hPa"));
                delay(CALIBRATION_ERROR_DELAY_MS);
                continue;
            }
            
            float current_pressure_Pa = current_pressure_hPa * HPA_TO_PA_CONVERSION_FACTOR; // Convert to Pa
            
            // Check GPS altitude validity
            if (GPS_altitude == 0) { // Assuming 0 is an invalid/default GPS altitude
                Serial.println(F("GPS altitude is zero, waiting for valid altitude..."));
                delay(CALIBRATION_ERROR_DELAY_MS);
                continue;
            }
            
            // Standard sea level pressure in Pa
            float sea_level_Pa = STANDARD_SEA_LEVEL_PRESSURE * HPA_TO_PA_CONVERSION_FACTOR; // Convert from hPa to Pa
            
            float raw_altitude = HYPSOMETRIC_CONSTANT_A * (1.0 - pow(current_pressure_Pa / sea_level_Pa, HYPSOMETRIC_EXPONENT));
            baro_altitude_offset = (GPS_altitude / MM_TO_M_CONVERSION_FACTOR) - raw_altitude;  // Convert GPS altitude from mm to m
            
            Serial.print(F("Calibration attempt #"));
            Serial.print(attempts);
            Serial.println(F(" succeeded!"));
            Serial.print(F("GPS Alt="));
            Serial.print(GPS_altitude / MM_TO_M_CONVERSION_FACTOR);
            Serial.print(F("m, Raw Baro Alt="));
            Serial.print(raw_altitude);
            Serial.print(F("m, Calculated Offset="));
            Serial.print(baro_altitude_offset);
            Serial.println(F("m"));
            
            baro_calibration_done = true;
            baroCalibrated = true;  // Also update the main program's flag
            
            return true;
        } else {
            // If we don't have a good fix yet, provide feedback
            if (attempts % CALIBRATION_STATUS_PRINT_INTERVAL == 0) {  // Only print every N attempts
                Serial.print(F("Waiting for good GPS fix. Current Fix Type: "));
                Serial.print(GPS_fixType);
                Serial.print(F(", pDOP: "));
                Serial.print(pDOP / 100.0, 2); // pDOP is scaled by 100
                Serial.print(F(", Time elapsed: "));
                Serial.print((millis() - start_time) / 1000);
                Serial.println(F("s"));
            }
        }    
        delay(CALIBRATION_LOOP_DELAY_MS);  // Wait before next GPS read
    }
    
    Serial.println(F("Calibration timeout after "));
    Serial.print((millis() - start_time) / 1000);
    Serial.println(F(" seconds."));
    Serial.print(F("Final GPS fix: "));
    Serial.print(GPS_fixType);
    Serial.print(F(", pDOP: "));
    Serial.println(pDOP / 100.0, 2);

    if (GPS_fixType < MIN_GPS_FIX_TYPE_FOR_CALIBRATION || pDOP >= MAX_PDOP_FOR_CALIBRATION) {
        g_last_error_code = BARO_CALIBRATION_FAIL_NO_GPS;
    } else {
        g_last_error_code = BARO_CALIBRATION_FAIL_TIMEOUT; // General timeout if GPS was good but still failed
    }
    
    return false;  // Timeout occurred
}

float ms5611_get_altitude(float seaLevelPressure) {
    // Get the current pressure in hPa
    float current_pressure_hPa = pressure;  // Use the global pressure value
    
    // Convert pressures from hPa to Pa
    float current_pressure_Pa = current_pressure_hPa * HPA_TO_PA_CONVERSION_FACTOR;
    float seaLevelPressure_Pa = seaLevelPressure * HPA_TO_PA_CONVERSION_FACTOR;
    
    // Use the barometric formula: h = 44330 * (1 - (P/P0)^(1/5.255))
    // where P is the measured pressure and P0 is the sea level pressure
    // This formula gives altitude in meters
    float raw_altitude = HYPSOMETRIC_CONSTANT_A * (1.0 - pow(current_pressure_Pa / seaLevelPressure_Pa, HYPSOMETRIC_EXPONENT));
    
    // Apply calibration offset
    return raw_altitude + baro_altitude_offset;
}

// void ms5611_update_calibration() { // REMOVED as unused
//     // Only update calibration if we have a good GPS fix (3D fix or better)
//     // and good DOP (less than 2.0 is considered good)
//     if (GPS_fixType >= MIN_GPS_FIX_TYPE_FOR_CALIBRATION && pDOP < MAX_PDOP_FOR_OFFSET_UPDATE) {  // pDOP is in 0.01 units
//         float gps_altitude = GPS_altitude / MM_TO_M_CONVERSION_FACTOR;  // Convert from mm to meters
//         float baro_altitude = ms5611_get_altitude() - baro_altitude_offset;  // Get raw baro altitude
//
//         // Calculate new offset
//         float new_offset = gps_altitude - baro_altitude;
//
//         // Apply some smoothing to the calibration (80% old value, 20% new value)
//         baro_altitude_offset = OFFSET_UPDATE_SMOOTHING_OLD_FACTOR * baro_altitude_offset + OFFSET_UPDATE_SMOOTHING_NEW_FACTOR * new_offset;
//
//         // Print debug info if GPS debugging is enabled
//         if (enableGPSDebug) {
//             Serial.print(F("Baro Calibration: GPS="));
//             Serial.print(gps_altitude, 1);
//             Serial.print(F("m Baro="));
//             Serial.print(baro_altitude, 1);
//             Serial.print(F("m Offset="));
//             Serial.print(baro_altitude_offset, 1);
//             Serial.println(F("m"));
//         }
//     }
// }

void ms5611_init() {
    Serial.println(F("Initializing MS5611 barometer..."));
    Serial.print(F("MS5611_LIB_VERSION: "));
    Serial.println(MS5611_LIB_VERSION);

    // Try to initialize the sensor
    if (ms5611Sensor.begin() == true) {
        Serial.print(F("MS5611 found at address: "));
        Serial.println(ms5611Sensor.getAddress());
    } else {
        Serial.println(F("ERROR: MS5611 not found! Check connections."));
        g_last_error_code = SENSOR_INIT_FAIL_MS5611;
        ms5611_initialized_ok = false;
        delay(INIT_ERROR_DELAY_MS);
        return;
    }

    // Set high oversampling for better accuracy
    ms5611Sensor.setOversampling(OSR_HIGH);
    
    // Take several readings to stabilize the sensor
    Serial.println(F("Taking initial readings to stabilize MS5611..."));
    bool validReading = false;
    for (int i = 0; i < INIT_STABILIZATION_READINGS; i++) {
        int result = ms5611_read();
        if (result == MS5611_READ_OK) {
            validReading = true;
            Serial.print(F("Reading #"));
            Serial.print(i+1);
            Serial.print(F(": Pressure = "));
            Serial.print(pressure);
            Serial.print(F(" hPa, Temperature = "));
            Serial.print(temperature);
            Serial.println(F(" C"));
        } else {
            Serial.print(F("Read attempt #"));
            Serial.print(i+1);
            Serial.print(F(" failed with error: "));
            Serial.println(result);
        }
        delay(INIT_STABILIZATION_DELAY_MS);
    }
    
    // Check if a valid reading was obtained and pressure is within a reasonable range
    if (!validReading || pressure < INIT_VALID_PRESSURE_MIN_HPA || pressure > INIT_VALID_PRESSURE_MAX_HPA) {
        Serial.print(F("ERROR: MS5611 stabilization failed. Last pressure: "));
        Serial.print(pressure);
        Serial.println(F(" hPa. Check sensor."));
        g_last_error_code = SENSOR_READ_FAIL_MS5611; // Could also be SENSOR_INIT_FAIL if considered part of init
        ms5611_initialized_ok = false;
        return;
    }
    
    Serial.println(F("MS5611 stabilization complete."));
    ms5611_initialized_ok = true;
}

// void ms5611_print() { // REMOVED as unused
//     // Only print if sensor debug is enabled
//     if (!enableSensorDebug) return;
//
//     // Convert pressure from hPa to Pa for altitude calculation
//     float pressure_Pa = pressure * HPA_TO_PA_CONVERSION_FACTOR;
//     float std_pressure_Pa = STANDARD_SEA_LEVEL_PRESSURE * HPA_TO_PA_CONVERSION_FACTOR;
//
//     float raw_altitude = HYPSOMETRIC_CONSTANT_A * (1.0 - pow(pressure_Pa / std_pressure_Pa, HYPSOMETRIC_EXPONENT));
//
//     Serial.print("MS5611 Data");
//     Serial.print(" Baro (hPa): ");
//     Serial.print(pressure, 2);
//     Serial.print(" Temp: ");
//     Serial.print(temperature, 2);
//     Serial.print(" Raw Alt: ");
//     Serial.print(raw_altitude, 2);
//     Serial.print(" Cal'd Alt: ");
//     Serial.print(raw_altitude + baro_altitude_offset, 2);
//     Serial.print("m");
// }
