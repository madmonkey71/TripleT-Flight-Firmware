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

void initNeoPixel() {
  pixels.begin();
  pixels.setBrightness(50);  // Set brightness to 50%
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));  // Turn off the LED
  pixels.show();
} 