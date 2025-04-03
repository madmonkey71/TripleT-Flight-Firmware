#include "ms5611_functions.h"

MS5611 ms5611Sensor(0x77);
float pressure = 0;
float temperature = 0;

int ms5611_read() {
    int result = ms5611Sensor.read();
    if (result == MS5611_READ_OK) {
        pressure = ms5611Sensor.getPressure();
        temperature = ms5611Sensor.getTemperature();
    }
    return result;
}

float ms5611_get_altitude(float seaLevelPressure) {
    // Get the current pressure in hPa
    float pressure = ms5611Sensor.getPressure();
    
    // Use the barometric formula: h = 44330 * (1 - (P/P0)^(1/5.255))
    // where P is the measured pressure and P0 is the sea level pressure
    // This formula gives altitude in meters
    return 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.190295));
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
    Serial.print("MS5611 Data\n");
    Serial.print(" Baro (hPa): ");
    Serial.print(pressure, 2);
    Serial.print(" Temp: ");
    Serial.print(temperature, 2);
    Serial.print(" Alt: ");
    Serial.print(ms5611_get_altitude(), 2);
    Serial.println("m");
}
