#include "ms5611_functions.h"
#include "gps_functions.h"  // Include GPS functions for calibration

MS5611 ms5611Sensor(0x77);
float pressure = 0;
float temperature = 0;
float baro_altitude_offset = 0.0f;
bool baro_calibration_done = false;

// Declare the global variable from main
extern bool baroCalibrated;

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
    if (pressure < 700 || pressure > 1200) {
        Serial.println(F("Pressure reading is invalid for calibration. Must be between 700-1200 hPa"));
        Serial.print(F("Current value: "));
        Serial.print(pressure);
        Serial.println(F(" hPa"));
        
        // Try to get a fresh reading
        Serial.println(F("Attempting to get fresh pressure reading..."));
        int result = ms5611_read();
        if (result == MS5611_READ_OK) {
            Serial.print(F("New pressure reading: "));
            Serial.print(pressure);
            Serial.println(F(" hPa"));
        } else {
            Serial.print(F("MS5611 read error: "));
            Serial.println(result);
            return false;
        }
    }

    uint32_t start_time = millis();
    unsigned int attempts = 0;
    
    while (millis() - start_time < timeout_ms) {
        attempts++;
        gps_read();  // Update GPS data
        
        // Check for good GPS fix and accuracy
        if (GPS_fixType >= 3 && pDOP < 300) {  // 3.0 * 100 for fixed-point comparison
            // Get fresh pressure reading for calibration
            int result = ms5611_read();
            if (result != MS5611_READ_OK) {
                Serial.print(F("Failed to read pressure: "));
                Serial.println(result);
                delay(500);
                continue;
            }
            
            // The pressure is in hPa but formula expects Pa
            float current_pressure_hPa = pressure;  // Use the global pressure value in hPa
            
            // Invalid pressure check
            if (current_pressure_hPa < 700 || current_pressure_hPa > 1200) {
                Serial.print(F("Invalid pressure reading: "));
                Serial.print(current_pressure_hPa);
                Serial.println(F(" hPa"));
                delay(500);
                continue;
            }
            
            float current_pressure_Pa = current_pressure_hPa * 100.0; // Convert to Pa
            
            // Check GPS altitude validity
            if (GPS_altitude == 0) {
                Serial.println(F("GPS altitude is zero, waiting for valid altitude..."));
                delay(500);
                continue;
            }
            
            // Standard sea level pressure in Pa
            float sea_level_Pa = STANDARD_SEA_LEVEL_PRESSURE * 100.0; // Convert from hPa to Pa
            
            float raw_altitude = 44330.0 * (1.0 - pow(current_pressure_Pa / sea_level_Pa, 0.190295));
            baro_altitude_offset = (GPS_altitude / 1000.0) - raw_altitude;  // Convert GPS altitude from mm to m
            
            Serial.print(F("Calibration attempt #"));
            Serial.print(attempts);
            Serial.println(F(" succeeded!"));
            Serial.print(F("GPS Alt="));
            Serial.print(GPS_altitude / 1000.0);
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
            if (attempts % 5 == 0) {  // Only print every 5 attempts
                Serial.print(F("Waiting for good GPS fix. Current Fix Type: "));
                Serial.print(GPS_fixType);
                Serial.print(F(", pDOP: "));
                Serial.print(pDOP / 100.0, 2);
                Serial.print(F(", Time elapsed: "));
                Serial.print((millis() - start_time) / 1000);
                Serial.println(F("s"));
            }
        }    
        delay(200);  // Wait before next GPS read
    }
    
    Serial.println(F("Calibration timeout after "));
    Serial.print((millis() - start_time) / 1000);
    Serial.println(F(" seconds."));
    Serial.print(F("Final GPS fix: "));
    Serial.print(GPS_fixType);
    Serial.print(F(", pDOP: "));
    Serial.println(pDOP / 100.0, 2);
    
    return false;  // Timeout occurred
}

float ms5611_get_altitude(float seaLevelPressure) {
    // Get the current pressure in hPa
    float current_pressure_hPa = pressure;  // Use the global pressure value
    
    // Convert pressures from hPa to Pa
    float current_pressure_Pa = current_pressure_hPa * 100.0;
    float seaLevelPressure_Pa = seaLevelPressure * 100.0;
    
    // Use the barometric formula: h = 44330 * (1 - (P/P0)^(1/5.255))
    // where P is the measured pressure and P0 is the sea level pressure
    // This formula gives altitude in meters
    float raw_altitude = 44330.0 * (1.0 - pow(current_pressure_Pa / seaLevelPressure_Pa, 0.190295));
    
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
        
        // Print debug info if GPS debugging is enabled
        if (enableGPSDebug) {
            Serial.print(F("Baro Calibration: GPS="));
            Serial.print(gps_altitude, 1);
            Serial.print(F("m Baro="));
            Serial.print(baro_altitude, 1);
            Serial.print(F("m Offset="));
            Serial.print(baro_altitude_offset, 1);
            Serial.println(F("m"));
        }
    }
}

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
        delay(1000);
        return;
    }

    // Set high oversampling for better accuracy
    ms5611Sensor.setOversampling(OSR_HIGH);
    
    // Take several readings to stabilize the sensor
    Serial.println(F("Taking initial readings to stabilize MS5611..."));
    bool validReading = false;
    for (int i = 0; i < 5; i++) {
        int result = ms5611_read();
        if (result == MS5611_READ_OK) {
            validReading = true;
            Serial.print(F("Reading #"));
            Serial.print(i+1);
            Serial.print(F(": Pressure = "));
            Serial.print(pressure);
            Serial.print(F(" hPa, Temperature = "));
            Serial.print(temperature);
            Serial.println(F("°C"));
        } else {
            Serial.print(F("Read attempt #"));
            Serial.print(i+1);
            Serial.print(F(" failed with error: "));
            Serial.println(result);
        }
        delay(100); // Small delay between readings
    }
    
    if (validReading) {
        Serial.println(F("MS5611 successfully initialized!"));
        
        // Check if pressure readings are in valid range
        if (pressure < 800 || pressure > 1100) {
            Serial.println(F("WARNING: Pressure readings outside expected range (800-1100 hPa)"));
            Serial.print(F("Current pressure: "));
            Serial.print(pressure);
            Serial.println(F(" hPa - check sensor calibration"));
        } else {
            Serial.println(F("Pressure readings in valid range."));
        }
    } else {
        Serial.println(F("ERROR: Failed to get valid readings from MS5611"));
    }
}

void ms5611_print() {
    // Only print if sensor debug is enabled
    if (!enableSensorDebug) return;
    
    // Convert pressure from hPa to Pa for altitude calculation
    float pressure_Pa = pressure * 100.0; 
    float std_pressure_Pa = STANDARD_SEA_LEVEL_PRESSURE * 100.0;
    
    float raw_altitude = 44330.0 * (1.0 - pow(pressure_Pa / std_pressure_Pa, 0.190295));
    
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
