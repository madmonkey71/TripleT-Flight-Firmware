# TripleT Flight Firmware

An eventually comprehensive flight controller firmware for Teensy 4.0/4.1 microcontrollers, designed for model rockets and high-power rocketry applications.

## Project Status
**Current Version**: v0.15 (Alpha)

### Development Status
- ✅ Core sensor integration (GPS, Barometer, IMU, Accelerometer)
- ✅ SD Card data logging
- ✅ Interactive serial interface
- ✅ Basic diagnostic tools
- ✅ Quaternion and Euler angle-based orientation tracking
- ✅ GPS/Barometer calibration
- ✅ Stationary detection
- ✅ Configurable debug outputs
- 🚧 Flight state detection (In Progress)
- 🚧 Apogee Detection (Planned)
- 🚧 Parachute deployment control (Planned)
- 🚧 Enhanced telemetry (Planned)
- 🚧 Thrust vector control (Planned)
- 🚧 Live Transmission of data via radio (Planned)

## Project Lead
**Matthew Thom** - Project Lead and Primary Developer

### AI Assistance
This project utilizes AI assistance for:
- Code documentation
- System architecture design
- Implementation suggestions
- Code review and optimization

## Overview

TripleT Flight Firmware is an open-source flight controller software built for Teensy 4.0/4.1. It provides robust sensor integration, data logging, and a command-driven interface to configure and interact with your flight computer.

## Hardware Requirements

- **Microcontroller**: Teensy 4.0 or 4.1
- **Sensors**:
  - SparkFun ZOE-M8Q GPS Module (I2C address 0x42)
  - MS5611 Barometric Pressure Sensor (I2C address 0x77)
  - SparkFun KX134 Accelerometer (I2C address 0x1F)
  - SparkFun ICM-20948 9-DOF IMU (I2C address 0x69)
- **Storage**:
  - SD Card (connected via SPI on Teensy 4.0, via SDIO on Teensy 4.1)
- **Other Components**:
  - Optional: Buzzer (pin 23)
  - Optional: WS2812 LEDs (pin 7)
  - Optional: Servo motors for TVC (pins 0, 1)
  - Optional: Pyro channels (pins 5, 6, 7, 8)

## Features

- **Multi-sensor Integration**: Combined data from GPS, barometer, accelerometer, and 9-DOF IMU
- **Sensor Fusion**: Kalman filter-based sensor fusion for accurate orientation tracking
- **SD Card Logging**: Comprehensive data logging to SD card with CSV format
- **GPS/Barometer Calibration**: Automatic or manual calibration of barometric altitude based on GPS data
- **Comprehensive Data Collection**:
  - GPS position, altitude, speed, and fix quality
  - Barometric pressure, temperature, and calibrated altitude
  - High-G acceleration measurements
  - Orientation data from IMU (quaternions and Euler angles)
- **Interactive Serial Interface**: Command-driven system for data retrieval and configuration
- **Diagnostic Tools**: I2C scanner, sensor status reporting, and storage space monitoring
- **Configurable Debug Outputs**: Selectively enable/disable debug information for specific sensors

## Installation

1. **Set up PlatformIO Environment**:
   - Install [PlatformIO](https://platformio.org/)
   - Clone this repository
   - Open the project in PlatformIO
   
2. **Required Libraries**:
   - Adafruit_NeoPixel
   - SdFat
   - SerialFlash
   - PWMServo
   - SparkFun_KX13X
   - SparkFun_u-blox_GNSS_Arduino_Library
   - MS5611 library
   - SparkFun ICM-20948 IMU library

3. **Hardware Connections**:
   - Connect sensors via I2C (SDA/SCL)
   - Connect SD card via SPI (pins 10-13) for Teensy 4.0 or SDIO for Teensy 4.1
   - Connect buzzer, servos, and other components to designated pins

4. **Compile and Upload**:
   - Select your target board (teensy40 or teensy41)
   - Compile and upload using PlatformIO

## Usage

### Basic Operation

On startup, the firmware:
1. Initializes all sensors and storage devices
2. Creates a new log file (with timestamp if GPS is available)
3. Begins collecting and logging sensor data

### Serial Commands

The firmware supports both single-character and extended commands:

| Command | Description |
|---------|-------------|
| `1-9`   | Toggle various debug outputs (system, IMU, GPS, barometer, etc.) |
| `a`     | Show help message |
| `b`     | Show system status |
| `f`     | Show storage statistics |
| `g`     | Toggle detailed display mode |
| `h`     | Calibrate barometer with GPS |
| `i`     | Display IMU data |
| `j`     | Toggle status summary |
| `debug_all_off` | Disable all debugging output |
| `calibrate` | Manually calibrate barometer with GPS |
| `help`  | Show help message |

### Debug Options

You can selectively enable/disable various debug outputs:

- System debug
- IMU debug
- GPS debug
- Barometer debug
- Storage debug
- Sensor fusion debug
- Quaternion debug
- Euler angles debug
- ICM raw debug

### Data Logging

Data is logged to the SD card in CSV format with the following information:
- Timestamp
- GPS information (fix type, satellites, position, altitude, speed)
- Barometric data (pressure, temperature)
- Accelerometer readings (X, Y, Z in g)
- IMU data (acceleration, gyro, magnetometer)
- Quaternion orientation values

### Barometer Calibration

The system supports calibration of the barometric altitude using GPS data:
- Automatic calibration when a good GPS fix is available
- Manual calibration via the `h` command or `calibrate` command
- Calibration status and values displayed in the barometer debug output

## Documentation

For detailed documentation, please refer to:
- [System Documentation](docs/TripleT_Flight_Firmware_Documentation.md)

## Future Enhancements

- Flight state detection (launch, apogee, descent)
- Parachute deployment control
- Flight stabilization via thrust vector control implementation
- Enhanced telemetry and wireless data transmission
- Improved power management and battery monitoring
- User-configurable settings saved to flash

## Acknowledgements

This project was inspired by:
- BPS.Space flight computer designs by Joe Barnard
- He makes awesome rocketry content so you should go and support him https://www.youtube.com/@BPSspace

## License
This project is licensed under the GNU General Public License v3.0 (GPL-3.0) - a copyleft license that requires anyone who distributes this code or a derivative work to make the source available under the same terms.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

When contributing:
- Fork the repository
- Create a feature branch
- Make your changes
- Submit a pull request with a clear description of the changes


