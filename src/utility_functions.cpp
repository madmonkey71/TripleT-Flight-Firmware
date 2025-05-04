#include "utility_functions.h"

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

// Initialize the SD card with simpler error handling
bool initSDCard() {
  Serial.println(F("Initializing SD card..."));
  
  // Reset SD card flags
  sdCardPresent = false;
  sdCardMounted = false;
  sdCardAvailable = false;
  
  // Check if card detect pin is defined and check physical presence
  #ifdef SD_DETECT_PIN
    pinMode(SD_DETECT_PIN, INPUT_PULLUP);
    if (digitalRead(SD_DETECT_PIN) == HIGH) {
      Serial.println(F("SD card not physically present (card detect pin is HIGH)"));
      return false;
    } else {
      sdCardPresent = true;
    }
  #endif
  
  // Try to initialize the SD card with configured settings
  if (!SD.begin(SD_CONFIG)) {
    Serial.println(F("SD card initialization failed"));
    return false;
  }

  // Card initialized successfully
  Serial.println(F("SD card initialized successfully"));
  sdCardMounted = true;
  
  // Get volume information directly from SD object
  if (SD.fatType() == 0) {
    Serial.println(F("Failed to get FAT type - card may not be usable"));
    return false;
  }
  
  // Print card capacity and available space
  uint64_t cardCapacity = (uint64_t)SD.card()->sectorCount() * 512;
  Serial.print(F("SD card capacity: "));
  Serial.print(cardCapacity / (1024ULL * 1024ULL));
  Serial.println(F(" MB"));
  
  // Calculate available space
  availableSpace = (uint64_t)SD.vol()->freeClusterCount() * 
                   (uint64_t)SD.vol()->bytesPerCluster();
  Serial.print(F("Available space: "));
  Serial.print(availableSpace / (1024ULL * 1024ULL));
  Serial.println(F(" MB"));
  
  sdCardAvailable = true;
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