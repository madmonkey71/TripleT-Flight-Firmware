// GPS Functions implementation for TripleT_Flight_Firmware
#include "gps_functions.h"
#include "config.h"
#include "gps_config.h"
#include <Arduino.h>
#include <SPI.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>

// Define the global variables declared as extern in the header
SFE_UBLOX_GNSS myGNSS; // Use default constructor
long lastTime = 0;
byte GPS_fixType = 0;
byte SIV = 0;
long GPS_latitude = 0;
long GPS_longitude = 0;
long GPS_altitude = 0;
long GPS_altitudeMSL = 0;
long GPS_speed = 0;
long GPS_heading = 0;
int pDOP = 0;
byte RTK = 0;

// GPS time variables with default values for Jan 1, 2000
int GPS_year = 2000;
byte GPS_month = 1;
byte GPS_day = 1;
byte GPS_hour = 0;
byte GPS_minute = 0;
byte GPS_second = 0;
bool GPS_time_valid = false;
bool g_gps_initialized_ok = false;

// Add reference to debug flag
extern volatile bool enableGPSDebug;

#include "error_codes.h"              // For ErrorCode_t
extern ErrorCode_t g_last_error_code; // For setting error codes

// Watchdog reference - must feed during long init sequences
#include <Watchdog_t4.h>
extern WDT_T4<WDT1> wdt;

// Create a single persistent NullStream that doesn't output anything
class NullStream : public Stream {
public:
  int available() { return 0; }
  int read() { return -1; }
  int peek() { return -1; }
  void flush() {}
  size_t write(uint8_t) { return 1; }
};

static NullStream nullStream;

// Single function to control GPS debugging state
void setGPSDebugging(bool enable) {
  // Set multiple times to ensure it takes effect
  if (enable) {
    myGNSS.enableDebugging(Serial, true);
  } else {
    // Call multiple times with delays to ensure it's disabled
    myGNSS.enableDebugging(nullStream, false);
    delay(10);
    myGNSS.enableDebugging(nullStream, false);
    delay(10);
    myGNSS.enableDebugging(nullStream, false);
  }
}

void gps_init() {
  Serial.println(F("Initializing GPS..."));

  // Initialize GPS time variables with default values
  GPS_year = 2000;
  GPS_month = 1;
  GPS_day = 1;
  GPS_hour = 0;
  GPS_minute = 0;
  GPS_second = 0;
  GPS_time_valid = false;

  // Initialize GPS connection
  bool gpsInitOk = false;

  // Feed watchdog before GPS init - begin() can take several seconds polling
  wdt.feed();

#if GPS_USE_SPI
  Serial.println(F("GPS init (SPI)..."));
  SPI.begin();
  setGPSDebugging(enableGPSDebug);
  // Use shorter maxWait (1000ms) to avoid exceeding watchdog timeout
  gpsInitOk = myGNSS.begin(SPI, GPS_SPI_CS_PIN, GPS_SPI_SPEED, 1000);
#else
  Serial.println(F("GPS init (I2C)..."));
  // Note: Wire.begin() is already called in main setup(), but safe to call
  // again
  Wire.begin();
  setGPSDebugging(enableGPSDebug);
  gpsInitOk = myGNSS.begin(Wire, 0x42, 1000);
#endif

  // Feed watchdog after GPS init attempt
  wdt.feed();

  if (!gpsInitOk) {
    Serial.println(
        F("ERROR: GPS module (myGNSS.begin()) failed to initialize!"));
    Serial.println(F("   → Skipping GPS configuration"));
    g_gps_initialized_ok = false;
    g_last_error_code = SENSOR_INIT_FAIL_GPS;
    return;
  }

  g_gps_initialized_ok = true;

  // Configure the GPS module
  Serial.print(F("Configuring GPS..."));

  // Set the port to output UBX only (turn off NMEA noise)
#if GPS_USE_SPI
  myGNSS.setSPIOutput(COM_TYPE_UBX);
  const uint8_t gpsComPort = COM_PORT_SPI;
#else
  myGNSS.setI2COutput(COM_TYPE_UBX);
  const uint8_t gpsComPort = COM_PORT_I2C;
#endif

  wdt.feed();

  // Configure in a single block
  bool configSuccess = true;
  // Rocket flight profile: without this, the u-blox default (portable) model
  // clamps its tracking filters at low dynamics and can reject or lag the
  // solution during boost. AIRBORNE4g supports <4g accel and high climb rates.
  configSuccess &= myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g);
  configSuccess &= myGNSS.setAutoPVT(true);
  configSuccess &= myGNSS.setNavigationFrequency(5);
  configSuccess &=
      myGNSS.enableMessage(UBX_CLASS_NAV, UBX_NAV_PVT, gpsComPort, 1);
  configSuccess &=
      myGNSS.enableMessage(UBX_CLASS_NAV, UBX_NAV_SAT, gpsComPort, 5);
  configSuccess &=
      myGNSS.enableMessage(UBX_CLASS_NAV, UBX_NAV_STATUS, gpsComPort, 1);
  configSuccess &=
      myGNSS.enableMessage(UBX_CLASS_NAV, UBX_NAV_DOP, gpsComPort, 1);

  wdt.feed();

  Serial.println(configSuccess ? F("OK") : F("failed"));

  // Ensure debugging is still in the proper state after initialization
  setGPSDebugging(enableGPSDebug);
}

bool gps_read() {
  // Skip if GPS module failed to initialize - avoid polling dead SPI/I2C bus
  if (!g_gps_initialized_ok) {
    return false;
  }

  // Update GPS data if available
  if (myGNSS.getPVT()) {
    // Get basic positioning data
    GPS_latitude = myGNSS.getLatitude();
    GPS_longitude = myGNSS.getLongitude();
    GPS_altitude = myGNSS.getAltitudeMSL();
    GPS_speed = myGNSS.getGroundSpeed();
    byte rawFixType = myGNSS.getFixType();
    byte rawSIV = myGNSS.getSIV();

    // Validate GPS data - reject corrupted values from bus errors
    if (rawFixType > 5) {
      // Fix type must be 0-5 per u-blox spec; anything else is corrupted
      GPS_fixType = 0;
      SIV = 0;
      return true; // Data was read but is invalid, skip further processing
    }
    GPS_fixType = rawFixType;
    SIV = (rawSIV <= 100) ? rawSIV : 0; // Reject unrealistic satellite counts

    // Populate additional GPS variables for logging and guidance
    GPS_altitudeMSL =
        myGNSS.getAltitudeMSL();       // Altitude Above Mean Sea Level in mm
    GPS_heading = myGNSS.getHeading(); // Heading in degrees * 1e-5
    pDOP = myGNSS.getPDOP(); // Positional Dilution of Precision (pDOP * 100)
    RTK = myGNSS.getCarrierSolutionType(); // 0=None, 1=Float, 2=Fixed

    // Update GPS time variables if we have a valid fix
    if (GPS_fixType > 0) {
      GPS_year = myGNSS.getYear();
      GPS_month = myGNSS.getMonth();
      GPS_day = myGNSS.getDay();
      GPS_hour = myGNSS.getHour();
      GPS_minute = myGNSS.getMinute();
      GPS_second = myGNSS.getSecond();
      GPS_time_valid = true;
    }

    // Debug print if enabled
    if (enableGPSDebug) {
      gps_print();
    }
    return true;
  }
  return false;
}

void gps_print() {
  Serial.println("GPS Data:");
  Serial.print("  Fix type: ");
  Serial.print(GPS_fixType);
  Serial.print(" | Satellites: ");
  Serial.println(SIV);

  Serial.print("  Lat: ");
  Serial.print(GPS_latitude / 10000000.0, 7);
  Serial.print(" | Lon: ");
  Serial.print(GPS_longitude / 10000000.0, 7);
  Serial.print(" | Alt: ");
  Serial.print(GPS_altitude / 1000.0);
  Serial.println(" m");

  Serial.print("  Speed: ");
  Serial.print(GPS_speed / 1000.0);
  Serial.println(" m/s");

  if (GPS_time_valid) {
    Serial.print("  Time: ");
    Serial.print(GPS_year);
    Serial.print("-");
    if (GPS_month < 10)
      Serial.print("0");
    Serial.print(GPS_month);
    Serial.print("-");
    if (GPS_day < 10)
      Serial.print("0");
    Serial.print(GPS_day);
    Serial.print(" ");
    if (GPS_hour < 10)
      Serial.print("0");
    Serial.print(GPS_hour);
    Serial.print(":");
    if (GPS_minute < 10)
      Serial.print("0");
    Serial.print(GPS_minute);
    Serial.print(":");
    if (GPS_second < 10)
      Serial.print("0");
    Serial.println(GPS_second);
  } else {
    Serial.println("  Time: Not yet valid");
  }

  Serial.println();
}

// Function to get GPS date/time safely (uses defaults if no valid time)
void getGPSDateTime(int &year, byte &month, byte &day, byte &hour, byte &minute,
                    byte &second) {
  // Return the current GPS time values (defaults to Jan 1, 2000 if no valid
  // fix)
  year = GPS_year;
  month = GPS_month;
  day = GPS_day;
  hour = GPS_hour;
  minute = GPS_minute;
  second = GPS_second;
}

// Safely get the GPS fix type
uint8_t getFixType() { return GPS_fixType; }

// Safely get the GPS altitude in meters
float getGPSAltitude() {
  // The u-blox modules report altitude in millimeters. Convert to meters.
  return (float)GPS_altitude / 1000.0f;
}
