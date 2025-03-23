/*
 * GPS Module Test Program
 * For SparkFun ZOE-M8Q GPS Module
 * 
 * This program tests the GPS module by reading data and displaying 
 * it via the serial port. It provides diagnostic information and 
 * continuous position updates.
 */

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // http://librarymanager/All#SparkFun_u-blox_GNSS

// Create a GPS object
SFE_UBLOX_GNSS myGPS;

// GPS variables
long lastTime = 0; // Timer for limiting GPS polling
long latitude = 0;
long longitude = 0;
long altitude = 0;
long speed = 0;
int satellites = 0;
byte fixType = 0;
int pDOP = 0;

// Pin definitions
#define LED_PIN 13 // Built-in LED

void setup() {
  // Initialize LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Start serial connection
  Serial.begin(115200);
  delay(2000); // Give time for the serial monitor to open
  
  Serial.println(F("GPS Module Test Program"));
  Serial.println(F("For SparkFun ZOE-M8Q GPS Module"));
  Serial.println();
  
  // Initialize I2C
  Wire.begin();
  
  // Connect to the GPS module with I2C
  Serial.println(F("Connecting to GPS module..."));
  
  // Try multiple times to connect
  boolean connected = false;
  for (int attempt = 1; attempt <= 5; attempt++) {
    Serial.print(F("Attempt "));
    Serial.print(attempt);
    Serial.print(F(" of 5... "));
    
    if (myGPS.begin(Wire) == true) {
      connected = true;
      Serial.println(F("Success!"));
      break;
    } else {
      Serial.println(F("Failed!"));
      delay(1000);
    }
  }
  
  if (!connected) {
    Serial.println(F("Could not connect to GPS. Please check wiring."));
    // Flash LED to indicate failure
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(300);
      digitalWrite(LED_PIN, LOW);
      delay(300);
    }
  }
  
  // Configure GPS module
  Serial.println(F("GPS found! Configuring..."));
  
  // Enable automatic PVT (position, velocity, time) messages
  myGPS.setAutoPVT(true);
  
  // Set navigation rate to 5Hz
  myGPS.setNavigationFrequency(5);
  
  // Use UBX protocol only (more efficient on I2C)
  myGPS.setI2COutput(COM_TYPE_UBX);
  
  // Save configuration
  myGPS.saveConfiguration();
  
  Serial.println(F("Configuration complete!"));
  Serial.println(F("--------------------------------"));
  Serial.println(F("GPS data will refresh every second"));
  Serial.println();
  
  // Flash LED twice to indicate successful setup
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void loop() {
  // Check for GPS data every 200ms
  if (millis() - lastTime > 200) {
    lastTime = millis();
    
    // Get the latest data from the GPS module
    if (myGPS.getPVT()) {
      // Read GPS data
      fixType = myGPS.getFixType();
      satellites = myGPS.getSIV();
      pDOP = myGPS.getPDOP();
      
      // Read position data if we have a fix
      if (fixType >= 2) {
        // Position and altitude
        latitude = myGPS.getLatitude();
        longitude = myGPS.getLongitude();
        altitude = myGPS.getAltitudeMSL();
        speed = myGPS.getGroundSpeed();
        
        // Turn on LED when we have a fix
        digitalWrite(LED_PIN, HIGH);
      } else {
        // Blink LED when searching for fix
        digitalWrite(LED_PIN, (millis() / 500) % 2);
      }
      
      // Print GPS status
      printGPSData();
    }
  }
  
  // Check for serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    
    // Process commands
    switch (cmd) {
      case 'r':
      case 'R':
        // Reset GPS
        Serial.println(F("Resetting GPS module..."));
        myGPS.hardReset();
        delay(1000);
        break;
        
      case 'c':
      case 'C':
        // Display detailed configuration
        printGPSConfig();
        break;
        
      case 's':
      case 'S':
        // Scan I2C bus
        scanI2C();
        break;
        
      case 'h':
      case 'H':
      case '?':
        // Help menu
        printHelp();
        break;
    }
    
    // Clear any remaining characters
    while (Serial.available()) Serial.read();
  }
}

void printGPSData() {
  // Clear screen with ANSI escape code
  Serial.print(F("\033[2J\033[H"));
  
  Serial.println(F("==== GPS Status ===="));
  
  // Fix type
  Serial.print(F("Fix type: "));
  switch (fixType) {
    case 0: Serial.println(F("No fix")); break;
    case 1: Serial.println(F("Dead reckoning")); break;
    case 2: Serial.println(F("2D fix")); break;
    case 3: Serial.println(F("3D fix")); break;
    case 4: Serial.println(F("GNSS + Dead reckoning")); break;
    default: Serial.println(fixType); break;
  }
  
  // Satellites and signal quality
  Serial.print(F("Satellites: "));
  Serial.println(satellites);
  
  Serial.print(F("Position DOP: "));
  Serial.println(pDOP / 100.0, 2);
  
  // Position information
  if (fixType >= 2) {
    Serial.println(F("\n==== Position ===="));
    
    // Format latitude and longitude as decimal degrees
    Serial.print(F("Lat: "));
    Serial.print(latitude / 10000000.0, 7);
    Serial.println(F("°"));
    
    Serial.print(F("Long: "));
    Serial.print(longitude / 10000000.0, 7);
    Serial.println(F("°"));
    
    Serial.print(F("Altitude: "));
    Serial.print(altitude / 1000.0, 1);
    Serial.println(F(" m"));
    
    Serial.print(F("Speed: "));
    Serial.print(speed * 0.0036, 1); // Convert to km/h
    Serial.println(F(" km/h"));
  }
  
  Serial.println(F("\nPress ? for help menu"));
}

void printGPSConfig() {
  Serial.println(F("\n==== GPS Configuration ===="));
  
  // Display navigation rate
  byte rate = myGPS.getNavigationFrequency();
  Serial.print(F("Navigation rate: "));
  Serial.print(rate);
  Serial.println(F(" Hz"));
  
  // Display power mode
  uint8_t mode = myGPS.getPowerSaveMode();
  Serial.print(F("Power save mode: "));
  Serial.println(mode);
  
  // Display protocol version
  Serial.print(F("UBX protocol version: "));
  Serial.print(myGPS.getProtocolVersionHigh());
  Serial.print(F("."));
  Serial.println(myGPS.getProtocolVersionLow());
  
  // Display module info
  Serial.print(F("Module: u-blox "));
  Serial.println(myGPS.getModuleName());
  
  Serial.println();
}

void scanI2C() {
  Serial.println(F("\n==== I2C Scanner ===="));
  Serial.println(F("Scanning..."));
  
  byte error, address;
  int devices = 0;
  
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print(F("Device found at address 0x"));
      if (address < 16) Serial.print(F("0"));
      Serial.print(address, HEX);
      
      // Try to identify common devices
      if (address == 0x42) Serial.print(F(" - SparkFun ZOE-M8Q GPS"));
      if (address == 0x77) Serial.print(F(" - MS5611/BMP280"));
      if (address == 0x69) Serial.print(F(" - ICM-20948/MPU9250"));
      if (address == 0x1F) Serial.print(F(" - KX134"));
      
      Serial.println();
      devices++;
    }
  }
  
  if (devices == 0) {
    Serial.println(F("No I2C devices found"));
  }
  
  Serial.println(F("Scan complete\n"));
}

void printHelp() {
  Serial.println(F("\n==== Command Help ===="));
  Serial.println(F("r - Reset GPS module"));
  Serial.println(F("c - Show GPS configuration"));
  Serial.println(F("s - Scan I2C bus"));
  Serial.println(F("? or h - Show this help"));
  Serial.println();
} 