#include "utility_functions.h"
#include "config.h" // Ensure config.h is included for SD_CONFIG and other hardware defs
#include "data_structures.h" // Include LogData definition
#include "log_format_definition.h" // For LOG_COLUMNS and LOG_COLUMN_COUNT
#include <cmath> // Include for sqrt()
#include <stdio.h> // For snprintf
#include <SdFat.h>
#include "gps_functions.h"
// Required for dtostrf typically, though often included by Arduino.h
#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_TEENSY)
#include <stdlib.h> 
#else
// For other AVR, dtostrf might be in stdlib.h or avr/dtostrf.h if not in global scope
// This is a common include pattern, but might need adjustment for specific older AVR toolchains
#include <stdlib.h> 
// #include <avr/dtostrf.h> // Fallback for some AVR toolchains
#endif

extern bool ms5611_initialized_ok;
extern bool g_baroCalibrated;
extern bool g_icm20948_ready;
extern bool g_kx134_initialized_ok;
extern SFE_UBLOX_GNSS myGNSS;
extern SdFat g_SD;

// Removed global Adafruit_NeoPixel pixels object definition from here.
// It should be defined once in TripleT_Flight_Firmware.cpp as g_pixels.

void scan_i2c() {
  Serial.println(F("\nI2C Scanner"));
  Serial.println(F("Scanning..."));

  byte error, address;
  int nDevices = 0;

  for(address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Wire.endTransmission to see if
    // a device acknowledged the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print(F("I2C device found at address 0x"));
      if (address < 16) 
        Serial.print("0");
      Serial.print(address, HEX);
      
      // Try to identify known devices
      if (address == 0x42) Serial.print(F(" - SparkFun ZOE-M8Q GPS Module"));
      if (address == 0x77) Serial.print(F(" - MS5611 Barometric Pressure Sensor"));
      if (address == 0x69) Serial.print(F(" - ICM-20948 9-DOF IMU"));
      if (address == 0x1F) Serial.print(F(" - SparkFun KX134 Accelerometer"));
      Serial.println();
      nDevices++;
    }
    else if (error == 4) {
      Serial.print(F("Unknown error at address 0x"));
      if (address < 16) 
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  if (nDevices == 0)
    Serial.println(F("No I2C devices found\n"));
  else
    Serial.println(F("I2C scan complete\n"));
}

// Old initSDCard() definition REMOVED.
// The refactored version bool initSDCard(SdFat& sd_obj, bool& sdCardMounted_out, bool& sdCardPresent_out)
// is (or should be) in TripleT_Flight_Firmware.cpp.

// Function to list all files in the root directory
void listRootDirectory() {
  Serial.println(F("\n--- Files in root directory ---"));
  
  FsFile root = g_SD.open("/");
  if (!root) {
    Serial.println(F("Failed to open root directory"));
    return;
  }
  
  int fileCount = 0;
  uint64_t totalSize = 0;
  
  // Print header
  Serial.println(F("Name                            Size      Date/Time"));
  Serial.println(F("------------------------------------------------------"));
  
  // List all files
  FsFile file;
  while (file.openNext(&root, O_RDONLY)) {
    // Get file name (truncated to 30 chars)
    char fileName[31];
    file.getName(fileName, sizeof(fileName));
    
    // Get file size
    uint32_t fileSize = file.size();
    totalSize += fileSize;
    
    // Print file details
    Serial.print(fileName);
    
    // Pad name for alignment
    int nameLen = strlen(fileName);
    for (int i = nameLen; i < 30; i++) {
      Serial.print(" ");
    }
    
    // Print size (right-aligned)
    Serial.print(" ");
    if (fileSize < 10000) Serial.print(" ");
    if (fileSize < 1000) Serial.print(" ");
    if (fileSize < 100) Serial.print(" ");
    if (fileSize < 10) Serial.print(" ");
    Serial.print(fileSize);
    Serial.print(" bytes");
    
    Serial.println();
    
    file.close();
    fileCount++;
  }
  
  root.close();
  
  // Print summary
  Serial.println(F("------------------------------------------------------"));
  Serial.print(fileCount);
  Serial.print(F(" file(s), Total size: "));
  Serial.print((float)totalSize / 1024.0, 1);
  Serial.println(F(" KB"));
  Serial.println();
}

void initNeoPixel(Adafruit_NeoPixel& pixels_obj) {
  pixels_obj.begin();
  pixels_obj.setBrightness(50);  // Set brightness to 50%
  pixels_obj.setPixelColor(0, pixels_obj.Color(0, 0, 0));  // Turn off the LED
  pixels_obj.show();
}

// Debug formatting functions
void printDebugHeader(const char* title) {
  Serial.println();
  Serial.print(F("===== "));
  Serial.print(title);
  Serial.println(F(" ====="));
}

void printDebugValue(const char* label, float value, int precision) {
  Serial.print(F("  "));
  Serial.print(label);
  Serial.print(F(": "));
  Serial.println(value, precision);
}

void printDebugValueWithUnit(const char* label, float value, const char* unit, int precision) {
  Serial.print(F("  "));
  Serial.print(label);
  Serial.print(F(": "));
  Serial.print(value, precision);
  Serial.print(F(" "));
  Serial.println(unit);
}

void printDebugPair(const char* label, float value1, float value2, int precision) {
  Serial.print(F("  "));
  Serial.print(label);
  Serial.print(F(": "));
  Serial.print(value1, precision);
  Serial.print(F(", "));
  Serial.println(value2, precision);
}

void printDebugTriple(const char* label, float value1, float value2, float value3, int precision) {
  Serial.print(F("  "));
  Serial.print(label);
  Serial.print(F(": "));
  Serial.print(value1, precision);
  Serial.print(F(", "));
  Serial.print(value2, precision);
  Serial.print(F(", "));
  Serial.println(value3, precision);
}

void printDebugQuad(const char* label, float value1, float value2, float value3, float value4, int precision) {
  Serial.print(F("  "));
  Serial.print(label);
  Serial.print(F(": "));
  Serial.print(value1, precision);
  Serial.print(F(", "));
  Serial.print(value2, precision);
  Serial.print(F(", "));
  Serial.print(value3, precision);
  Serial.print(F(", "));
  Serial.println(value4, precision);
}

void printDebugState(const char* label, const char* state) {
  Serial.print(F("  "));
  Serial.print(label);
  Serial.print(F(": "));
  Serial.println(state);
}

void printDebugBoolean(const char* label, bool value) {
  Serial.print(F("  "));
  Serial.print(label);
  Serial.print(F(": "));
  Serial.println(value ? F("ON") : F("OFF"));
}

void printDebugDivider() {
  Serial.println(F("  -------------------"));
}

// Helper function to get acceleration magnitude regardless of which accelerometer is available
float get_accel_magnitude(bool kx134_ok, const float* kx_accel,
                          bool icm_ready, const float* icm_accel_data,
                          bool system_debug_enabled) {

  // Prefer KX134 if it initialized successfully
  if (kx134_ok) {
    // Check for non-zero values to ensure data is likely valid (simple check)
    if (kx_accel && (kx_accel[0] != 0.0 || kx_accel[1] != 0.0 || kx_accel[2] != 0.0)) {
      return sqrt(kx_accel[0] * kx_accel[0] +
                  kx_accel[1] * kx_accel[1] +
                  kx_accel[2] * kx_accel[2]);
    } else {
        if (system_debug_enabled) {
             Serial.println(F("[ACC MAG] KX134 OK but data is zero/null. Falling back."));
        }
    }
  }

  // Fallback to ICM20948 if KX134 didn't init or data was zero/null
  if (icm_ready) {
    // Check for non-zero values (simple check)
    if (icm_accel_data && (icm_accel_data[0] != 0.0 || icm_accel_data[1] != 0.0 || icm_accel_data[2] != 0.0)) {
        return sqrt(icm_accel_data[0] * icm_accel_data[0] +
                    icm_accel_data[1] * icm_accel_data[1] +
                    icm_accel_data[2] * icm_accel_data[2]);
    } else {
        if (system_debug_enabled) {
             Serial.println(F("[ACC MAG] ICM20948 ready but data is zero/null."));
        }
    }
  }
  
  if (system_debug_enabled) {
    Serial.println(F("[ACC MAG] No valid accel data available, returning 0.0"));
  }
  return 0.0; // No accelerometer available or ready or data is zero
}

// Helper function to convert LogData struct to a CSV formatted char array.
// This function is now driven by the LOG_COLUMNS definition.
// Uses a static buffer, so the result should be used or copied immediately.
String logDataToString(const LogData& data) {
    static char buffer[1024]; // Increased buffer size for safety
    int offset = 0;
    char tempFloatBuffer[20]; // Temporary buffer for dtostrf conversions.

    for (size_t i = 0; i < LOG_COLUMN_COUNT; ++i) {
        const LogColumnDescriptor_t* desc = &LOG_COLUMNS[i];
        const void* field_ptr = reinterpret_cast<const char*>(&data) + desc->offset; // Cast to char* for pointer arithmetic
        int written_chars = 0;

        // Ensure buffer has space before writing. Account for potential comma and null terminator.
        // Max typical float string: "-123.123456" (11) + comma (1) + null (1) = 13. tempFloatBuffer is 20.
        // Max typical uint32_t string: "4294967295" (10) + comma (1) + null (1) = 12.
        if (offset >= (int)(sizeof(buffer) - (sizeof(tempFloatBuffer) + 2))) { 
            Serial.println(F("logDataToString buffer nearly full! Skipping remaining fields."));
            break; 
        }

        switch (desc->type) {
            case TYPE_UINT32:
                written_chars = snprintf(buffer + offset, sizeof(buffer) - offset,
                                         "%lu", *static_cast<const uint32_t*>(field_ptr));
                break;
            case TYPE_UINT8:
                written_chars = snprintf(buffer + offset, sizeof(buffer) - offset,
                                         "%u", *static_cast<const uint8_t*>(field_ptr));
                break;
            case TYPE_INT32_AS_FLOAT_SCALED_1E7_P6: // For Latitude/Longitude
                dtostrf(*static_cast<const int32_t*>(field_ptr) / 10000000.0, 1, 6, tempFloatBuffer);
                written_chars = snprintf(buffer + offset, sizeof(buffer) - offset, "%s", tempFloatBuffer);
                break;
            case TYPE_INT32_AS_FLOAT_SCALED_1E3_P2: // For Altitude, AltitudeMSL, Speed
                dtostrf(*static_cast<const int32_t*>(field_ptr) / 1000.0, 1, 2, tempFloatBuffer);
                written_chars = snprintf(buffer + offset, sizeof(buffer) - offset, "%s", tempFloatBuffer);
                break;
            case TYPE_INT32_AS_FLOAT_SCALED_1E5_P2: // For Heading
                dtostrf(*static_cast<const int32_t*>(field_ptr) / 100000.0, 1, 2, tempFloatBuffer);
                written_chars = snprintf(buffer + offset, sizeof(buffer) - offset, "%s", tempFloatBuffer);
                break;
            case TYPE_UINT16_AS_FLOAT_SCALED_1E2_P2: // For pDOP
                dtostrf(*static_cast<const uint16_t*>(field_ptr) / 100.0, 1, 2, tempFloatBuffer);
                written_chars = snprintf(buffer + offset, sizeof(buffer) - offset, "%s", tempFloatBuffer);
                break;
            case TYPE_FLOAT_P2: // For direct floats like pressure, temperature, altitudes from baro, icm_temp
                dtostrf(*static_cast<const float*>(field_ptr), 1, 2, tempFloatBuffer);
                written_chars = snprintf(buffer + offset, sizeof(buffer) - offset, "%s", tempFloatBuffer);
                break;
            case TYPE_FLOAT_P3: // For actuator outputs and similar
                dtostrf(*static_cast<const float*>(field_ptr), 1, 3, tempFloatBuffer);
                written_chars = snprintf(buffer + offset, sizeof(buffer) - offset, "%s", tempFloatBuffer);
                break;
            case TYPE_FLOAT_P4: // For sensor float arrays (accel, gyro, mag)
                dtostrf(*static_cast<const float*>(field_ptr), 1, 4, tempFloatBuffer);
                written_chars = snprintf(buffer + offset, sizeof(buffer) - offset, "%s", tempFloatBuffer);
                break;
            case TYPE_FLOAT_P6_RAD: // For quaternions, euler angles in radians, gyro biases
                dtostrf(*static_cast<const float*>(field_ptr), 1, 6, tempFloatBuffer);
                written_chars = snprintf(buffer + offset, sizeof(buffer) - offset, "%s", tempFloatBuffer);
                break;
            default:
                Serial.print(F("Unknown type in logDataToString: ")); Serial.println(desc->type);
                written_chars = snprintf(buffer + offset, sizeof(buffer) - offset, "ERR");
                break;
        }

        if (written_chars > 0) {
            offset += written_chars;
        } else if (written_chars < 0) {
            // snprintf encoding error or buffer too small from the start
            Serial.println(F("logDataToString snprintf error or buffer too small!"));
            // Ensure buffer is null-terminated safely if an error occurs mid-string
            if(offset < (int)sizeof(buffer)) buffer[offset] = '\0';
            else buffer[sizeof(buffer)-1] = '\0';
            return String(buffer); // Return immediately with what we have
        }


        if (i < LOG_COLUMN_COUNT - 1) {
          if (offset < (int)sizeof(buffer) - 1) { // Check space for comma
            buffer[offset++] = ',';
          } else {
            Serial.println(F("logDataToString buffer overflow (comma)!"));
            break; 
          }
        }
    }
    
    // Final null termination, ensuring it's within bounds
    if(offset < (int)sizeof(buffer)){
      buffer[offset] = '\0'; 
    } else {
      buffer[sizeof(buffer)-1] = '\0'; // Force null termination if at the very end
      if(offset > (int)sizeof(buffer)){ // If offset somehow exceeded buffer size.
        Serial.println(F("logDataToString critical buffer overflow detected!"));
      } else {
         Serial.println(F("logDataToString buffer full, string potentially truncated."));
      }
    }

    return String(buffer);
}

// Helper function to map a float value from one range to another - REMOVED as unused.
// float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
// ... implementation ...
// }

void convertQuaternionToEuler(float q0, float q1, float q2, float q3, float& roll, float& pitch, float& yaw) {
    // Roll (x-axis rotation)
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1.0f)
        pitch = copysign(PI / 2.0f, sinp); // use 90 degrees if out of range, Arduino PI
    else
        pitch = asin(sinp);

    // Yaw (z-axis rotation)
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    yaw = atan2(siny_cosp, cosy_cosp);
}

void convertEulerToQuaternion(float roll, float pitch, float yaw, float& q0, float& q1, float& q2, float& q3) {
    // Compute half angles
    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);
    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);
    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    // Compute quaternion components (Hamilton convention: w, x, y, z)
    q0 = cr * cp * cy + sr * sp * sy; // w component
    q1 = sr * cp * cy - cr * sp * sy; // x component  
    q2 = cr * sp * cy + sr * cp * sy; // y component
    q3 = cr * cp * sy - sr * sp * cy; // z component
}

// ... other functions ...

bool isSensorSuiteHealthy(FlightState currentState, bool verbose) {
    bool healthy = true;
    
    if (verbose) {
        Serial.println(F("=== Sensor Suite Health Check ==="));
        Serial.print(F("Target State: "));
        Serial.println(getStateName(currentState));
        Serial.println(F(""));
    }
    
    // Barometer Health Check (Essential for most states)
    if (!ms5611_initialized_ok) {
        if (currentState == CALIBRATION || currentState == PAD_IDLE) {
            if (verbose) {
                Serial.println(F("⚠️ HEALTH_WARN: MS5611 (Barometer) not initialized."));
                Serial.println(F("   → Functionality will be limited. Arming will be prevented."));
                Serial.println(F("   → Check I2C connections and power to barometer."));
            }
            // Do not set healthy = false for CALIBRATION or PAD_IDLE if baro HW init failed
            // This allows reaching these states for diagnostics or other operations.
        } else { // For ARMED and other flight states, it's a failure
            if (verbose) {
                Serial.println(F("❌ HEALTH_FAIL: MS5611 (Barometer) not initialized. Critical for current state."));
                Serial.println(F("   → Check I2C connections and power to barometer"));
            }
            healthy = false;
        }
    } else if (verbose) {
        Serial.println(F("✓ MS5611 (Barometer) initialized successfully"));
    }
    
    // This check for calibration status remains important,
    // but only if the barometer hardware itself was initialized.
    if (ms5611_initialized_ok && currentState > CALIBRATION && !g_baroCalibrated) {
        if (verbose) {
            Serial.println(F("❌ HEALTH_FAIL: Barometer is initialized but not calibrated."));
            Serial.println(F("   → Use 'calibrate' or 'h' command to calibrate with GPS."));
            Serial.println(F("   → Calibration requires GPS fix (3D fix with 7+ satellites)."));
        }
        healthy = false;
    } else if (!ms5611_initialized_ok && currentState > CALIBRATION && verbose) {
        // If baro HW isn't OK, and we are past CALIBRATION, it implies we are trying to operate
        // in a state that normally requires calibration, but can't calibrate.
        // The !ms5611_initialized_ok check earlier would have handled warnings/errors for the current state.
        Serial.println(F("ℹ️ INFO: Barometer hardware not initialized, so calibration status is not applicable/checked. Arming will be prevented."));
    } else if (ms5611_initialized_ok && currentState > CALIBRATION && g_baroCalibrated && verbose) { // Baro OK, past CALIB, and calibrated
        Serial.println(F("✓ Barometer initialized and calibrated."));
    }
    // Note: if !ms5611_initialized_ok, the earlier check handles the verbose output for baro init status.
    // If ms5611_initialized_ok and currentState <= CALIBRATION, no specific message here about calibration status yet.

    // IMU Health Check (Essential for flight)
    if (currentState >= ARMED && currentState < LANDED) {
        if (!g_icm20948_ready && !g_kx134_initialized_ok) {
            if (verbose) {
                Serial.println(F("❌ HEALTH_FAIL: No primary or secondary IMU is ready for flight."));
                Serial.println(F("   → ICM20948 status: ") + String(g_icm20948_ready ? "Ready" : "Not Ready"));
                Serial.println(F("   → KX134 status: ") + String(g_kx134_initialized_ok ? "Ready" : "Not Ready"));
                Serial.println(F("   → At least one IMU must be working for flight operations"));
            }
            healthy = false;
        } else if (verbose) {
            Serial.println(F("✓ IMU Health Check Passed:"));
            if (g_icm20948_ready) Serial.println(F("   → ICM20948: Ready"));
            if (g_kx134_initialized_ok) Serial.println(F("   → KX134: Ready"));
        }
    } else if (verbose && (currentState >= ARMED && currentState < LANDED)) {
        Serial.println(F("ℹ IMU check skipped (not required for current state)"));
    }
    
    // GPS Health Check (Less critical for flight, more for recovery)
    // We can be more lenient here, but log if it's not available.
    if (verbose) {
        int fixType = myGNSS.getFixType();
        if (fixType == 0) {
            Serial.println(F("⚠ HEALTH_WARN: No GPS fix."));
            Serial.println(F("   → GPS is not critical for basic flight operations"));
            Serial.println(F("   → Required for barometer calibration and recovery operations"));
        } else {
            Serial.println(F("✓ GPS: Fix Type ") + String(fixType) + F(" (") + 
                          String(myGNSS.getSIV()) + F(" satellites)"));
        }
    }
    
    if (verbose) {
        Serial.println(F(""));
        Serial.print(F("Overall Health Status: "));
        Serial.println(healthy ? F("✓ HEALTHY") : F("❌ UNHEALTHY"));
        Serial.println(F("================================"));
    }

    return healthy;
}

const char* getStateName(FlightState state) {
  switch (state) {
    case STARTUP: return "STARTUP";
    case CALIBRATION: return "CALIBRATION";
    case PAD_IDLE: return "PAD_IDLE";
    case ARMED: return "ARMED";
    case BOOST: return "BOOST";
    case COAST: return "COAST";
    case APOGEE: return "APOGEE";
    case DROGUE_DEPLOY: return "DROGUE_DEPLOY";
    case DROGUE_DESCENT: return "DROGUE_DESCENT";
    case MAIN_DEPLOY: return "MAIN_DEPLOY";
    case MAIN_DESCENT: return "MAIN_DESCENT";
    case LANDED: return "LANDED";
    case RECOVERY: return "RECOVERY";
    case ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

// Function to read battery voltage
// Assumes a voltage divider is used: Battery(+) -- R1 -- ADC_PIN -- R2 -- GND
// Vin = ADC_Value * (ADC_REF_VOLTAGE / ADC_RESOLUTION) * ((R1 + R2) / R2)
float read_battery_voltage() {
  #if ENABLE_BATTERY_MONITORING == 1
    int adc_raw = analogRead(BATTERY_VOLTAGE_PIN);
    // Convert ADC reading to voltage at the ADC pin
    float adc_voltage = adc_raw * (ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION);
    // Calculate actual battery voltage using divider formula
    // Vin = Vout * (R1+R2)/R2
    float battery_voltage = adc_voltage * (VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / VOLTAGE_DIVIDER_R2;
    return battery_voltage;
  #else
    return 0.0f; // Return 0 if monitoring is disabled
  #endif
}