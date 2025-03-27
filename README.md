# TripleT Flight Firmware

An eventually comprehensive flight controller firmware for Teensy 4.0 microcontrollers, designed for model rockets and high-power rocketry applications.

## Overview

TripleT Flight Firmware is an open-source flight controller software built for Teensy 4.0. It provides robust sensor integration, multi-target data logging, and a command-driven interface to configure and interact with your flight computer.

**Current Version**: v0.10 (Alpha)

## Hardware Requirements

- **Microcontroller**: Teensy 4.0
- **Sensors**:
  - SparkFun ZOE-M8Q GPS Module (I2C address 0x42)
  - MS5611 Barometric Pressure Sensor (I2C address 0x77)
  - SparkFun KX134 Accelerometer (I2C address 0x1F)
  - SparkFun ICM-20948 9-DOF IMU (I2C address 0x69)
- **Storage**:
  - SD Card (connected via SPI)
  - External Serial Flash (optional)
  - Internal Teensy Flash (512KB dedicated for logging)
- **Other Components**:
  - Optional: Buzzer (pin 23)
  - Optional: WS2812 LEDs (pin 7)
  - Optional: Servo motors for TVC (pins 0, 1)
  - Optional: Pyro channels (pins 5, 6, 7, 8)

## Features

- **Multi-sensor Integration**: Combined data from GPS, barometer, accelerometer, and 9-DOF IMU
- **Triple Redundant Logging**: Data recorded to SD card, external flash, and internal flash memory
- **LittleFS Implementation**: Efficient storage and management of log files in internal flash
- **Comprehensive Data Collection**:
  - GPS position, altitude, speed, and fix quality
  - Atmospheric pressure and temperature
  - High-G acceleration measurements
  - Orientation data from IMU
- **Interactive Serial Interface**: Command-driven system for data retrieval and configuration
- **Diagnostic Tools**: I2C scanner, sensor status reporting, and storage space monitoring
- **Efficient Data Management**: Records at 5-10Hz with adjustable logging rates

## Installation

1. **Set up Arduino Environment**:
   - Install [Arduino IDE](https://www.arduino.cc/en/software)
   - Install [Teensyduino Add-on](https://www.pjrc.com/teensy/teensyduino.html)
   
2. **Required Libraries**:
   - Adafruit_NeoPixel
   - SD
   - SerialFlash
   - PWMServo
   - LittleFS
   - SparkFun_KX13X
   - SparkFun_u-blox_GNSS_Arduino_Library
   - MS5611 library
   - ICM_20948 library

3. **Hardware Connections**:
   - Connect sensors via I2C (SDA/SCL)
   - Connect SD card via SPI (pins 10-13)
   - Connect buzzer, servos, and other components to designated pins

4. **Upload the Firmware**:
   - Open the project in Arduino IDE
   - Select Teensy 4.0 as the board
   - Compile and upload

## Usage

### Basic Operation

On startup, the firmware:
1. Initializes all sensors and storage devices
2. Checks for existing log data in internal flash
3. Creates a new log file (with timestamp if GPS is available)
4. Begins collecting and logging sensor data

### Serial Commands

The firmware supports the following commands via the serial monitor (115200 baud):

| Command | Description |
|---------|-------------|
| `help`  | Displays available commands and current status |
| `dump`  | Dumps internal flash data to serial output in CSV format |
| `erase` | Erases all internal flash log files |
| `list`  | Lists all log files stored in internal flash |
| `stats` | Shows detailed storage statistics for all media |
| `detail`| Toggles detailed display mode for sensor data |
| `imu`   | Shows detailed IMU data |

### Data Logging

Data is logged at the following rates:
- SD Card: 10Hz (every 100ms)
- External Flash: 10Hz (every 100ms)
- Internal Flash: 5Hz (every 200ms)

Log files contain the following data:
- Timestamp
- GPS information (fix type, satellites, position, altitude, speed)
- Barometric data (pressure, temperature)
- Accelerometer readings (X, Y, Z in g)
- IMU data when available

### LED Status Indicators

The firmware supports WS2812 RGB LEDs for status indication (not fully implemented in current version).

## Data Retrieval

After a flight, you can retrieve the data using:

1. **SD Card**: Remove the SD card and read the CSV files directly
2. **Serial Interface**: Use the `dump` command to export internal flash data via serial
3. **Manual Download**: Connect to the Teensy and use the serial commands to extract data

## Future Enhancements

- Flight state detection (launch, apogee, descent)
- Parachute deployment control
- Flight stabilisation via a thrust vector control implementation
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


