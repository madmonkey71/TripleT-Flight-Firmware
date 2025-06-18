#include "utility_functions.h"
#include "config.h" // Ensure config.h is included for SD_CONFIG and other hardware defs
#include "data_structures.h" // Include LogData definition
#include "log_format_definition.h" // For LOG_COLUMNS and LOG_COLUMN_COUNT
#include <cmath> // Include for sqrt()
#include <stdio.h> // For snprintf
// Required for dtostrf typically, though often included by Arduino.h
#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_TEENSY)
#include <stdlib.h> 
#else
// For other AVR, dtostrf might be in stdlib.h or avr/dtostrf.h if not in global scope
// This is a common include pattern, but might need adjustment for specific older AVR toolchains
#include <stdlib.h> 
// #include <avr/dtostrf.h> // Fallback for some AVR toolchains
#endif


// Define the pixels variable
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

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

// Initialize the SD card with detailed error handling and status reporting
bool initSDCard() {
  Serial.println(F("--- SD Card Initialization ---"));

  // Reset SD card flags initially
  sdCardPresent = false;
  sdCardMounted = false;
  sdCardAvailable = false;

  // Step 1: Initialize the SD card library using SDIO
  // Note: Teensy 4.1 built-in SD card slot uses SDIO, no card detect pin needed
  Serial.println(F("Initializing SD library (SD.begin) with SDIO..."));
  if (!SD.begin(SD_CONFIG)) {
    Serial.println(F("ERROR: SD.begin() failed."));
    Serial.println(F(" -> Check card formatting (FAT16/FAT32), card insertion, or try a different SD card."));
    sdCardPresent = false; // If SD.begin fails, likely no card or bad connection
    sdCardMounted = false; // Explicitly set
    return false; // Critical error, cannot proceed
  }
  Serial.println(F("SUCCESS: SD.begin() completed with SDIO."));
  sdCardMounted = true; // Card is mounted
  sdCardPresent = true; // If SD.begin() succeeded, card is present

  // Step 2: Check FAT type
  Serial.println(F("Checking filesystem type (SD.fatType())..."));
  uint8_t fatType = SD.fatType();
  if (fatType == 0) {
    Serial.println(F("ERROR: SD.fatType() returned 0. Filesystem is not FAT16 or FAT32 or card not usable."));
    Serial.println(F(" -> Reformat the card with a supported FAT filesystem."));
    sdCardAvailable = false; // Card is mounted but not usable
    return false; 
  } else {
    Serial.print(F("SUCCESS: Filesystem type is FAT"));
    Serial.println(fatType == FAT_TYPE_EXFAT ? F("EX (EXFAT)") : String(fatType));
    // Note: SdFat typically uses defines like FAT_TYPE_FAT16, FAT_TYPE_FAT32, FAT_TYPE_EXFAT
    // For simplicity, just printing the number or "EXFAT" if detected.
  }

  // Step 3: Check card capacity and free space
  Serial.println(F("Checking card capacity and free space..."));
  
  // Card capacity
  uint32_t sectorCount = SD.card()->sectorCount();
  if (sectorCount == 0) {
    Serial.println(F("ERROR: Failed to get card sector count (SD.card()->sectorCount() returned 0)."));
    Serial.println(F(" -> Card may be corrupted or unreadable."));
    sdCardAvailable = false; // Card is mounted but details are not readable
    return false;
  }
  uint64_t cardCapacity = (uint64_t)sectorCount * 512; // SdFat uses 512 byte sectors
  Serial.print(F("  Card Capacity: "));
  Serial.print(cardCapacity / (1024ULL * 1024ULL));
  Serial.println(F(" MB"));

  // Free space
  uint32_t freeClusterCount = SD.vol()->freeClusterCount();
  uint32_t bytesPerCluster = SD.vol()->bytesPerCluster();
  if (bytesPerCluster == 0) { // Check if bytesPerCluster is valid
      Serial.println(F("ERROR: Failed to get valid cluster size (bytesPerCluster is 0)."));
      Serial.println(F(" -> Filesystem metadata might be corrupted."));
      sdCardAvailable = false;
      return false;
  }
  availableSpace = (uint64_t)freeClusterCount * bytesPerCluster;
  Serial.print(F("  Available Space: "));
  Serial.print(availableSpace / (1024ULL * 1024ULL));
  Serial.println(F(" MB"));

  // All checks passed
  Serial.println(F("SUCCESS: SD card is initialized, mounted, and available."));
  sdCardAvailable = true;
  Serial.println(F("--- SD Card Initialization Complete ---"));
  return true;
}

// Function to list all files in the root directory
void listRootDirectory() {
  Serial.println(F("\n--- Files in root directory ---"));
  
  FsFile root;
  if (!root.open("/")) {
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

void initNeoPixel() {
  pixels.begin();
  pixels.setBrightness(50);  // Set brightness to 50%
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));  // Turn off the LED
  pixels.show();
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
float get_accel_magnitude() {
  // Need access to the status flags and data arrays
  extern bool kx134_initialized_ok; // Assume defined in main .cpp
  extern float kx134_accel[3];      // Assume defined in kx134_functions.cpp
  extern bool icm20948_ready;       // Assume defined in main .cpp or icm_functions? Needs checking.
  extern float icm_accel[3];        // Assume defined in icm_20948_functions.cpp

  // Prefer KX134 if it initialized successfully
  // Note: The original code had complex checks for data readiness/timeouts.
  // Here we simplify based on the initialization flag. More robust checking
  // might be needed depending on how sensor reads are handled elsewhere.
  if (kx134_initialized_ok) {
    // Check for non-zero values to ensure data is likely valid (simple check)
    if (kx134_accel[0] != 0.0 || kx134_accel[1] != 0.0 || kx134_accel[2] != 0.0) {
      return sqrt(kx134_accel[0] * kx134_accel[0] +
                  kx134_accel[1] * kx134_accel[1] +
                  kx134_accel[2] * kx134_accel[2]);
    } else {
        if (enableSystemDebug) { // Only print if debug enabled
             Serial.println(F("[ACC MAG] KX134 initialized but data is zero. Falling back."));
        }
    }
  }

  // Fallback to ICM20948 if KX134 didn't init or data was zero
  if (icm20948_ready) { 
    // Check for non-zero values (simple check)
    if (icm_accel[0] != 0.0 || icm_accel[1] != 0.0 || icm_accel[2] != 0.0) {
        return sqrt(icm_accel[0] * icm_accel[0] +
                    icm_accel[1] * icm_accel[1] +
                    icm_accel[2] * icm_accel[2]);
    } else {
        if (enableSystemDebug) { // Only print if debug enabled
             Serial.println(F("[ACC MAG] ICM20948 ready but data is zero."));
        }
    }
  }
  
  if (enableSystemDebug) { // Only print if debug enabled
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

// Helper function to map a float value from one range to another
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  // Check for division by zero if in_min == in_max
  if (in_min == in_max) {
    return out_min; // Or handle as an error, like returning NaN or a specific error value
  }
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

bool isSensorSuiteHealthy(FlightState currentState) {
  // Placeholder implementation - customize with actual sensor checks
  if (enableSystemDebug && (millis() % 5000 < 20)) { // Print periodically if debug enabled
    Serial.print(F("Placeholder: isSensorSuiteHealthy() called for state "));
    Serial.print(getStateName(currentState)); // Assuming getStateName is available via this file or its includes
    Serial.println(F(", returning true."));
  }
  // Example check (actual checks would be more comprehensive)
  // if (!barometerStatus.isWorking && currentState > CALIBRATION && currentState < LANDED) return false;
  // if (!accelerometerStatus.isWorking && currentState > ARMED && currentState < LANDED) return false;
  return true;
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