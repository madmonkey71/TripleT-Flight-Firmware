/*
 * GPS Module Test Program (Serial/UART Version)
 * For SparkFun ZOE-M8Q GPS Module or compatible UBLOX GPS
 * 
 * This program tests the GPS module using Serial/UART connection,
 * reading NMEA sentences and displaying data via the serial monitor.
 */

#include <Arduino.h>
#include <SoftwareSerial.h>

// Pin definitions
#define GPS_RX_PIN 0  // Use an available digital pin on your board
#define GPS_TX_PIN 1  // Use an available digital pin on your board
#define LED_PIN 13    // Built-in LED

// Create a software serial port
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

// Buffer for incoming data
#define BUFFER_SIZE 128
char buffer[BUFFER_SIZE];
int bufferPos = 0;

// GPS data variables
float latitude = 0;
float longitude = 0;
float altitude = 0;
float speed_kmh = 0;
int satellites = 0;
char fixType = '0';
char timeStr[10] = "00:00:00";
char dateStr[11] = "00/00/0000";

// Timer for display updates
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 1000; // 1 second

void setup() {
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Start hardware serial for debug output
  Serial.begin(115200);
  delay(2000); // Give serial monitor time to open
  
  Serial.println(F("GPS Module Test Program (Serial/UART Version)"));
  Serial.println(F("For SparkFun ZOE-M8Q GPS Module"));
  Serial.println();
  
  // Initialize software serial for GPS
  gpsSerial.begin(9600); // Standard NMEA baud rate
  
  Serial.println(F("GPS serial connection initialized"));
  Serial.println(F("Waiting for GPS data..."));
  Serial.println();
  
  // Flash LED to indicate successful initialization
  for (int i = 0; i < 2; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void loop() {
  // Read data from GPS
  readGPS();
  
  // Update display periodically
  if (millis() - lastDisplayTime >= DISPLAY_INTERVAL) {
    lastDisplayTime = millis();
    printGPSData();
  }
  
  // Check for commands from serial monitor
  if (Serial.available()) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'h':
      case 'H':
      case '?':
        printHelp();
        break;
        
      case 'r':
      case 'R':
        // Reset buffer
        bufferPos = 0;
        memset(buffer, 0, BUFFER_SIZE);
        Serial.println(F("Buffer reset"));
        break;
    }
    
    // Clear any remaining characters
    while (Serial.available()) Serial.read();
  }
}

void readGPS() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    
    // Blink LED to show data reception
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    
    // Add character to buffer
    if (bufferPos < BUFFER_SIZE - 1) {
      buffer[bufferPos++] = c;
      buffer[bufferPos] = '\0'; // Null terminate the string
    }
    
    // If end of line, process the sentence
    if (c == '\n') {
      // Process the NMEA sentence
      if (strstr(buffer, "$GPGGA") || strstr(buffer, "$GNGGA")) {
        // GGA sentence - contains position, altitude, and satellites
        parseGGA(buffer);
      } 
      else if (strstr(buffer, "$GPRMC") || strstr(buffer, "$GNRMC")) {
        // RMC sentence - contains position, speed, date and time
        parseRMC(buffer);
      }
      
      // Reset buffer for next sentence
      bufferPos = 0;
      memset(buffer, 0, BUFFER_SIZE);
    }
  }
}

void parseGGA(const char* sentence) {
  // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
  // Find commas and extract values
  
  // Skip the sentence identifier
  const char* ptr = strchr(sentence, ',') + 1;
  
  // Time
  ptr = strchr(ptr, ',') + 1;
  
  // Latitude
  char latStr[12] = "";
  int i = 0;
  while (*ptr != ',' && i < 11) {
    latStr[i++] = *ptr++;
  }
  latStr[i] = '\0';
  ptr++; // Skip comma
  
  // North/South
  char ns = *ptr;
  ptr += 2; // Skip N/S and comma
  
  // Longitude
  char lonStr[12] = "";
  i = 0;
  while (*ptr != ',' && i < 11) {
    lonStr[i++] = *ptr++;
  }
  lonStr[i] = '\0';
  ptr++; // Skip comma
  
  // East/West
  char ew = *ptr;
  ptr += 2; // Skip E/W and comma
  
  // Fix quality
  fixType = *ptr;
  ptr += 2; // Skip fix quality and comma
  
  // Number of satellites
  char satStr[4] = "";
  i = 0;
  while (*ptr != ',' && i < 3) {
    satStr[i++] = *ptr++;
  }
  satStr[i] = '\0';
  satellites = atoi(satStr);
  ptr++; // Skip comma
  
  // Skip HDOP
  ptr = strchr(ptr, ',') + 1;
  
  // Altitude
  char altStr[10] = "";
  i = 0;
  while (*ptr != ',' && i < 9) {
    altStr[i++] = *ptr++;
  }
  altStr[i] = '\0';
  altitude = atof(altStr);
  
  // Convert NMEA format to decimal degrees
  if (strlen(latStr) > 0 && strlen(lonStr) > 0) {
    // Convert latitude from DDMM.MMMM to decimal degrees
    float latDeg = atof(latStr) / 100.0;
    int latWhole = (int)latDeg;
    float latDec = (latDeg - latWhole) * 100.0 / 60.0;
    latitude = latWhole + latDec;
    
    if (ns == 'S') latitude = -latitude;
    
    // Convert longitude from DDDMM.MMMM to decimal degrees
    float lonDeg = atof(lonStr) / 100.0;
    int lonWhole = (int)lonDeg;
    float lonDec = (lonDeg - lonWhole) * 100.0 / 60.0;
    longitude = lonWhole + lonDec;
    
    if (ew == 'W') longitude = -longitude;
  }
}

void parseRMC(const char* sentence) {
  // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  // Find commas and extract values
  
  // Skip the sentence identifier
  const char* ptr = strchr(sentence, ',') + 1;
  
  // Time
  char timeBuffer[7];
  int i = 0;
  while (*ptr != ',' && i < 6) {
    timeBuffer[i++] = *ptr++;
  }
  timeBuffer[i] = '\0';
  
  // Format time as hh:mm:ss
  if (strlen(timeBuffer) >= 6) {
    snprintf(timeStr, 9, "%c%c:%c%c:%c%c", 
             timeBuffer[0], timeBuffer[1],
             timeBuffer[2], timeBuffer[3],
             timeBuffer[4], timeBuffer[5]);
  }
  ptr++; // Skip comma
  
  // Status (A=active, V=void)
  char status = *ptr;
  ptr += 2; // Skip status and comma
  
  // Skip latitude, N/S, longitude, E/W (already parsed from GGA)
  for (i = 0; i < 4; i++) {
    ptr = strchr(ptr, ',') + 1;
  }
  
  // Speed in knots
  char speedStr[10] = "";
  i = 0;
  while (*ptr != ',' && i < 9) {
    speedStr[i++] = *ptr++;
  }
  speedStr[i] = '\0';
  
  // Convert speed from knots to km/h
  if (strlen(speedStr) > 0) {
    speed_kmh = atof(speedStr) * 1.852;
  }
  ptr++; // Skip comma
  
  // Skip course
  ptr = strchr(ptr, ',') + 1;
  
  // Date (DDMMYY)
  char dateBuffer[7];
  i = 0;
  while (*ptr != ',' && i < 6) {
    dateBuffer[i++] = *ptr++;
  }
  dateBuffer[i] = '\0';
  
  // Format date as DD/MM/YYYY
  if (strlen(dateBuffer) >= 6) {
    snprintf(dateStr, 11, "%c%c/%c%c/20%c%c", 
             dateBuffer[0], dateBuffer[1],
             dateBuffer[2], dateBuffer[3],
             dateBuffer[4], dateBuffer[5]);
  }
}

void printGPSData() {
  // Clear the screen with ANSI escape code
  Serial.print(F("\033[2J\033[H"));
  
  Serial.println(F("==== GPS Status ===="));
  
  // Time and date
  Serial.print(F("Time: "));
  Serial.print(timeStr);
  Serial.print(F("  Date: "));
  Serial.println(dateStr);
  
  // Fix information
  Serial.print(F("Fix: "));
  switch (fixType) {
    case '0': Serial.println(F("No fix")); break;
    case '1': Serial.println(F("GPS fix")); break;
    case '2': Serial.println(F("DGPS fix")); break;
    case '6': Serial.println(F("Estimated fix")); break;
    default: Serial.println(fixType); break;
  }
  
  Serial.print(F("Satellites: "));
  Serial.println(satellites);
  
  // Position information
  if (fixType != '0') {
    Serial.println(F("\n==== Position ===="));
    
    Serial.print(F("Lat: "));
    Serial.print(latitude, 6);
    Serial.println(F("°"));
    
    Serial.print(F("Long: "));
    Serial.print(longitude, 6);
    Serial.println(F("°"));
    
    Serial.print(F("Altitude: "));
    Serial.print(altitude, 1);
    Serial.println(F(" m"));
    
    Serial.print(F("Speed: "));
    Serial.print(speed_kmh, 1);
    Serial.println(F(" km/h"));
  }
  
  Serial.println(F("\nPress ? for help menu"));
}

void printHelp() {
  Serial.println(F("\n==== Command Help ===="));
  Serial.println(F("r - Reset buffer"));
  Serial.println(F("? or h - Show this help"));
  Serial.println();
} 