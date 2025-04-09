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

// Function to initialize the SD card
bool initSDCard() {
  Serial.println("Initializing SD card...");
  
  // Check if card is physically present if we have a detect pin
  #ifdef SD_DETECT_PIN
    pinMode(SD_DETECT_PIN, INPUT_PULLUP);
    if (digitalRead(SD_DETECT_PIN)) {
      Serial.println("No SD card detected!");
      return false;
    }
  #endif

  // Try to initialize the card
  if (!SD.begin(SD_CONFIG)) {
    if (SD.sdErrorCode()) {
      Serial.println("SD initialization failed!");
      Serial.print("Error: ");
      Serial.println(SD.sdErrorCode());
      
      // Try lower speed as fallback
      Serial.println("Trying lower speed...");
      SdSpiConfig config = SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(25));
      
      if (!SD.begin(config)) {
        Serial.println("SD initialization failed at lower speed!");
        return false;
      }
    }
  }
  
  if (!volume.begin(SD.card())) {
    Serial.println("Failed to initialize volume");
    return false;
  }

  // Try to create a test file to verify filesystem
  FsFile testFile;
  const char* testFileName = "test.txt";
  if (!testFile.open(testFileName, O_RDWR | O_CREAT | O_AT_END)) {
    Serial.println("Failed to create test file!");
    return false;
  }
  
  // Write something to the test file
  if (testFile.println("Test write")) {
    Serial.println("Test file write successful");
  } else {
    Serial.println("Test file write failed!");
    return false;
  }
  testFile.close();
  
  // Clean up test file
  if (!SD.remove(testFileName)) {
    Serial.println("Warning: Could not remove test file");
  }

  // Print some card statistics
  Serial.print("Volume size (MB): ");
  Serial.println((float)SD.card()->sectorCount() * 512.0 / 1000000.0);
  
  Serial.println("SD card initialization successful!");
  return true;
}

// Function to initialize the Serial Flash memory
bool initSerialFlash() {
  Serial.println(F("Initializing Serial Flash..."));
  
  #if defined(BOARD_STM32_THING_PLUS)
    // STM32 may need a different approach for external flash
    // SerialFlash library is Teensy-specific
    Serial.println(F("SerialFlash not supported on STM32 yet"));
    Serial.println(F("Using internal LittleFS instead"));
    return false;
  #else
    // Check if the flash memory is present - Teensy-specific SerialFlash
    if (!SerialFlash.begin(FLASH_CS_PIN)) {
      Serial.println(F("Serial Flash initialization failed!"));
      return false;
    }
    
    Serial.println(F("Serial Flash initialized."));
    return true;
  #endif
}

void handleFlashDataCheck() {
  // Check if there are any files in the internal flash
  File root = flashFS.open("/");
  if (!root) {
    Serial.println(F("Failed to open root directory"));
    return;
  }

  File file = root.openNextFile();
  if (!file) {
    Serial.println(F("No files found in internal flash"));
    root.close();
    return;
  }

  Serial.println(F("Files found in internal flash:"));
  while (file) {
    Serial.print(F("  "));
    Serial.print(file.name());
    Serial.print(F(" - "));
    Serial.print(file.size());
    Serial.println(F(" bytes"));

    // If we have an SD card, try to transfer the file
    if (sdCardAvailable) {
      const char* filename = file.name();
      
      // Create destination filename with a timestamp prefix
      char destPath[256];  // Large buffer for the path
      snprintf(destPath, sizeof(destPath), "/flash_data/%lu_%s", millis(), filename);
      
      // Create the directory if it doesn't exist
      if (!SD.exists("/flash_data")) {
        SD.mkdir("/flash_data");
      }

      // Open the destination file with explicit mode
      FsFile destFile = SD.open(destPath, O_RDWR | O_CREAT | O_AT_END);
      if (!destFile) {
        Serial.print(F("Failed to create destination file: "));
        Serial.println(destPath);
        file = root.openNextFile();
        continue;
      }

      // Reset file position to start
      file.seek(0);
      
      // Copy the file contents with verification
      uint8_t buffer[512];
      size_t bytesRead;
      size_t totalBytes = 0;
      bool transferSuccess = true;
      
      while ((bytesRead = file.read(buffer, sizeof(buffer))) > 0) {
        if (destFile.write(buffer, bytesRead) != bytesRead) {
          Serial.println(F("Write verification failed!"));
          transferSuccess = false;
          break;
        }
        totalBytes += bytesRead;
      }

      // Close the destination file
      destFile.close();

      // Verify file sizes match
      if (transferSuccess && totalBytes == file.size()) {
        // Delete the source file
        flashFS.remove(file.name());
        Serial.print(F("File transferred and verified: "));
        Serial.println(destPath);
      } else {
        Serial.println(F("Transfer verification failed!"));
        // Delete the incomplete destination file
        SD.remove(destPath);
      }
    }

    file = root.openNextFile();
  }

  root.close();
}

void initNeoPixel() {
  pixels.begin();
  pixels.setBrightness(50);  // Set brightness to 50%
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));  // Turn off the LED
  pixels.show();
} 