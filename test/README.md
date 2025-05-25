# GPS Module Test Programs

This repository contains two test programs for the SparkFun ZOE-M8Q GPS module used in the TripleT Flight Firmware. Each program demonstrates a different approach to interfacing with GPS modules.

## Program Options

### 1. GPS_Test.ino
- **Connection Method**: I2C protocol (address 0x42)
- **Dependencies**: 
  - SparkFun u-blox GNSS Arduino Library
  - Wire.h
- **Key Features**:
  - Direct UBX protocol communication
  - 5Hz position update rate
  - Position, velocity, and time data retrieval
  - LED status indication for GPS fix status
  - Interactive serial commands

### 2. GPS_Test_Serial.ino
- **Connection Method**: UART/Serial via SoftwareSerial
- **Dependencies**:
  - SoftwareSerial library (no GPS-specific libraries)
- **Key Features**:
  - Raw NMEA sentence parsing
  - GGA and RMC sentence interpretation
  - NMEA to decimal degree coordinate conversion
  - Visual LED feedback for data reception
  - Automatic screen refresh

## Pin Configurations

### Current Pin Settings:
- **I2C Version**: Uses standard I2C pins for your board (SDA/SCL)
- **Serial Version**: 
  - GPS module TX → Pin 7 (RX)
  - GPS module RX → Pin 8 (TX)
  - *Note: These pins were chosen to avoid conflicts with WLED_PIN (7) and PYRO pins*

### Hardware Considerations:
- The Teensy 4.0 pin configuration differs from the STM32 Thing Plus
- WLED uses pin 7, which may conflict with GPS serial RX
- The GPS module requires 3.3V power (do not use 5V)

## Compilation Instructions

### Using PlatformIO:
1. Navigate to the test directory
2. Edit platformio.ini if needed to match your board
3. Run: `pio run` to compile
4. Run: `pio run -t upload` to upload to your board

### Using Arduino IDE:
1. Open the appropriate .ino file
2. Select your board (Teensy 4.0 or STM32 Thing Plus)
3. Click Verify/Compile
4. Connect your board and click Upload

### Using Command Line:
- **Windows**: Run `compile_gps_test.bat`
- **Linux/Mac**: Run `chmod +x compile_gps_test.sh` then `./compile_gps_test.sh`

## Usage Guide

### Serial Monitor:
- Open at 115200 baud
- The GPS module communicates at 9600 baud (Serial version) or I2C (I2C version)

### LED Indicators:
- **Serial Version**:
  - LED blinks rapidly when receiving data
  - No blinking indicates no data from GPS
- **I2C Version**:
  - Slow blinking: searching for fix
  - Solid on: GPS fix acquired

### Interactive Commands:
- **Serial Version**:
  - `r` - Reset buffer
  - `?` or `h` - Show help menu
- **I2C Version**:
  - `r` - Reset GPS module
  - `c` - Show configuration
  - `s` - Scan I2C bus
  - `?` or `h` - Show help menu

## Troubleshooting

### Common Issues:
1. **No data received**: 
   - Check wiring connections
   - Verify power (3.3V)
   - For Serial version, confirm TX/RX aren't swapped

2. **GPS module not found on I2C**:
   - Run I2C scan (use `s` command in I2C version)
   - Check for address conflicts (ZOE-M8Q should be at 0x42)
   - Verify pull-up resistors on I2C lines

3. **Cannot get a fix**:
   - Ensure GPS has clear sky view
   - First fix may take 30-90 seconds
   - Cold start after long power-off may take several minutes
   - Indoor testing may yield poor or no results

## Extending the Code

- To log data to SD card, see the main firmware's logging functions
- For more advanced settings, explore the `gps_init()` function in the main firmware
- Disable debug messages for production use
- Consider adding power-saving modes for battery operation

## References

- [SparkFun ZOE-M8Q GPS Module Documentation](https://www.sparkfun.com/products/15193)
- [u-blox M8 Receiver Description](https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf)
- [NMEA Reference Manual](https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual-Rev2.1-Dec07.pdf) 