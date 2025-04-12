#include "ms5611_functions.h"
#include "gps_functions.h"  // Include GPS functions for calibration

MS5611 ms5611Sensor(0x77);
float pressure = 0;
float temperature = 0;
float baro_altitude_offset = 0;  // Initialize calibration offset to 0
bool baro_calibration_done = false;

// Add reference to debug flag
extern bool enableSensorDebug;

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

    uint32_t start_time = millis();
    while (millis() - start_time < timeout_ms) {
        gps_read();  // Update GPS data
        
        // Check for good GPS fix and accuracy
        if (GPS_fixType >= 3 && pDOP < 300) {  // 3.0 * 100 for fixed-point comparison
            float current_pressure = pressure;  // Use the global pressure value
            float raw_altitude = 44330.0 * (1.0 - pow(current_pressure / STANDARD_SEA_LEVEL_PRESSURE, 0.190295));
            baro_altitude_offset = (GPS_altitude / 1000.0) - raw_altitude;  // Convert GPS altitude from mm to m
            baro_calibration_done = true;
            
            #ifdef GPS_DEBUG_ENABLED
            Serial.print("GPS Altitude: "); Serial.print(GPS_altitude);
            Serial.print("m, Baro Altitude: "); Serial.print(raw_altitude);
            Serial.print("m, Calibration Offset: "); Serial.println(baro_altitude_offset);
            #endif
            
            return true;
        }
        
        delay(200);  // Wait before next GPS read
    }
    
    return false;  // Timeout occurred
}

float ms5611_get_altitude(float seaLevelPressure) {
    // Get the current pressure in hPa
    float current_pressure = pressure;  // Use the global pressure value
    
    // Use the barometric formula: h = 44330 * (1 - (P/P0)^(1/5.255))
    // where P is the measured pressure and P0 is the sea level pressure
    // This formula gives altitude in meters
    float raw_altitude = 44330.0 * (1.0 - pow(current_pressure / seaLevelPressure, 0.190295));
    
    // Apply calibration offset
    return raw_altitude + baro_altitude_offset;
}

void ms5611_update_calibration() {
    // Only update calibration if we have a good GPS fix (3D fix or better)
    // and good DOP (less than 2.0 is considered good)
    if (GPS_fixType >= 3 && pDOP < 200) {  // pDOP is in 0.01 units
        float gps_altitude = GPS_altitude / 1000.0;  // Convert from mm to meters
        float baro_altitude = ms5611_get_altitude() - baro_altitude_offset;  // Get raw baro altitude
        
        // Calculate new offset
        float new_offset = gps_altitude - baro_altitude;
        
        // Apply some smoothing to the calibration (80% old value, 20% new value)
        baro_altitude_offset = 0.8 * baro_altitude_offset + 0.2 * new_offset;
        
#if GPS_DEBUG_ENABLED
        Serial.print(F("Baro Calibration: GPS="));
        Serial.print(gps_altitude, 1);
        Serial.print(F("m Baro="));
        Serial.print(baro_altitude, 1);
        Serial.print(F("m Offset="));
        Serial.print(baro_altitude_offset, 1);
        Serial.println(F("m"));
#endif
    }
}

void ms5611_init() {
    Serial.println(__FILE__);
    Serial.print("MS5611_LIB_VERSION: ");
    Serial.println(MS5611_LIB_VERSION);

    if (ms5611Sensor.begin() == true) {
        Serial.print("MS5611 found: ");
        Serial.println(ms5611Sensor.getAddress());
    } else {
        Serial.println("MS5611 not found. halt.");
        delay(500);
    }

    ms5611Sensor.setOversampling(OSR_HIGH);
    int result = ms5611_read();
    if (result != MS5611_READ_OK) {
        Serial.print("MS5611 read error during init: ");
        Serial.println(result);
    } else {
        Serial.println("MS5611 successfully initialized and read");
    }
}

void ms5611_print() {
    // Only print if sensor debug is enabled
    if (!enableSensorDebug) return;
    
    float raw_altitude = 44330.0 * (1.0 - pow(pressure / STANDARD_SEA_LEVEL_PRESSURE, 0.190295));
    
    Serial.print("MS5611 Data");
    Serial.print(" Baro (hPa): ");
    Serial.print(pressure, 2);
    Serial.print(" Temp: ");
    Serial.print(temperature, 2);
    Serial.print(" Raw Alt: ");
    Serial.print(raw_altitude, 2);
    Serial.print(" Cal'd Alt: ");
    Serial.print(raw_altitude + baro_altitude_offset, 2);
    Serial.print("m");
}
