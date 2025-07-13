# TripleT Flight Firmware

**Current Version:** v0.51  
**Current State:** Beta (Ready for Controlled Test Flights)  
**Last Updated:** July 2025

## Project Lead
**Matthew Thom** - Project Lead and Primary Developer

## Overview

This firmware is designed for the **Teensy 4.1** microcontroller and provides comprehensive flight control capabilities for model rockets. The system manages all phases of flight from launch detection through recovery, utilizing a Kalman filter for sensor fusion, robust data logging, and extensive safety features.

## Key Features

- **Multi-Phase Flight Management**: Handles all 14 flight phases from pad idle through recovery
- **Advanced Sensor Fusion**: Kalman filter combining data from ICM-20948 IMU, KX134 high-G accelerometer, MS5611 barometer, and u-blox GPS
- **Dual Accelerometer Strategy**: Intelligently uses KX134 for high-G events and ICM-20948 for general flight
- **Redundant Apogee Detection**: Four distinct methods: barometric, accelerometer, GPS, and backup timer
- **Comprehensive Recovery System**: SOS audio beacon, LED strobe patterns, and GPS coordinate transmission
- **Real-time Data Logging**: 62 data points logged to SD card in CSV format
- **Web Interface**: Real-time data visualization with 3D orientation display and live charts
- **Interactive Command Interface**: Rich serial command system for diagnostics and control
- **Robust Error Recovery**: Automatic sensor health monitoring with recovery mechanisms
- **Battery Voltage Monitoring**: Real-time battery status monitoring and logging

## Quick Start

1. **Hardware Setup**: Connect sensors via I2C, GPS via serial, insert SD card
2. **Upload Firmware**: Use PlatformIO with `teensy41` environment
3. **Initialize**: System performs startup checks and enters `PAD_IDLE` state
4. **Web Interface**: Launch `web_interface/run_local_server.sh` for real-time monitoring
5. **Enable CSV Output**: Send `debug_serial_csv on` command to enable data streaming
6. **Arm**: Use `arm` command when ready for flight
7. **Flight**: System automatically manages all flight phases
8. **Recovery**: Follow SOS beacon and LED strobe to locate rocket

## Documentation

- 📋 **[Development Status](docs/DEVELOPMENT_STATUS.md)** - Current features, roadmap, and gap analysis
- 🔧 **[Hardware Requirements](docs/HARDWARE.md)** - Complete hardware setup and pin configuration
- 🚀 **[Flight Operations](docs/FLIGHT_OPERATIONS.md)** - Flight states, operations, and safety procedures
- ⚙️ **[Configuration Guide](docs/CONFIGURATION.md)** - Parameter settings and customization
- 🛠️ **[Installation & Setup](docs/INSTALLATION.md)** - Detailed installation and setup instructions
- 💻 **[Serial Commands](docs/COMMANDS.md)** - Complete command reference
- 📊 **[Data Logging](docs/DATA_LOGGING.md)** - Logging format and analysis
- 🌐 **[Web Interface](web_interface/README.md)** - Real-time data visualization and monitoring
- 🔍 **[Troubleshooting](docs/TROUBLESHOOTING.md)** - Common issues and debugging
- 🏗️ **[Development Guide](docs/DEVELOPMENT.md)** - Architecture and contribution guidelines
- ⚠️ **[Error Codes](docs/ERROR_CODES.md)** - Complete error code reference

## State Machine

The firmware operates on a 14-state flight state machine:

`STARTUP → CALIBRATION → PAD_IDLE → ARMED → BOOST → COAST → APOGEE → DROGUE_DEPLOY → DROGUE_DESCENT → MAIN_DEPLOY → MAIN_DESCENT → LANDED → RECOVERY` (plus ERROR state)

See [Flight Operations](docs/FLIGHT_OPERATIONS.md) for detailed state descriptions.

## Recent Updates (v0.52)

- ✅ **Web Interface Enhancement**: Complete web-based data visualization system
  - Real-time data monitoring with 3D orientation display
  - Live charts for altitude, acceleration, gyroscope, and magnetometer data
  - Serial command interface for remote control
  - Resolved HTTPS redirection issues with multiple server options
  - Updated CSV field mapping to match firmware's 62-field output
- ✅ **Configurable Guidance System**: Added ability to completely disable guidance for passive rockets
  - Set `ENABLE_GUIDANCE 0` for passive rockets without actuators
  - Set `ENABLE_GUIDANCE 1` for guided rockets with servo-controlled fins
  - All other functions (logging, recovery, etc.) work in both modes
- ✅ **Critical Compilation Fixes**: Resolved all compiler errors including missing braces, variable scope issues, and function structure problems
- ✅ **Header Guards Implementation**: Added proper header guards to prevent multiple inclusions
- ✅ **Missing Variable Declarations**: Added missing Kalman rate variables and extern declarations
- ✅ **Function Signature Fixes**: Corrected guidance_update function calls with proper GPS parameters
- ✅ **Switch Statement Fixes**: Resolved variable initialization crossing case labels
- ✅ **GPS Function Access**: Fixed scope issues with GPS utility functions
- ✅ **Successful Build Status**: ✅ **COMPILATION SUCCESSFUL** - All errors resolved, firmware builds without warnings
- ✅ **Enhanced Code Quality**: Improved variable naming consistency and function organization
- ✅ **Recovery System Enhancements**: Complete SOS audio beacon, LED strobe patterns, and GPS coordinate output
- ✅ **Battery Monitoring**: Comprehensive battery voltage monitoring and logging
- ✅ **Error Recovery**: Automatic error recovery with grace period protection
- ✅ **Documentation Reorganization**: Restructured documentation for better navigation and maintenance

## Safety Features

- **Sensor Health Monitoring**: Continuous sensor validation
- **Redundant Apogee Detection**: Four independent detection methods
- **Error Recovery**: Automatic and manual recovery mechanisms
- **State Persistence**: EEPROM-based state saving for power-loss recovery
- **Backup Timers**: Failsafe systems for critical operations
- **Comprehensive Logging**: Detailed flight data for analysis

## Hardware Requirements

**Core Components:**
- Teensy 4.1 microcontroller
- ICM-20948 9-DOF IMU
- KX134 high-G accelerometer
- MS5611 barometric pressure sensor
- u-blox GPS module
- SD card for logging
- NeoPixel LEDs for status
- Buzzer for audio feedback

See [Hardware Requirements](docs/HARDWARE.md) for complete specifications.

## License

This project is licensed under the MIT License.

## Acknowledgments

- Original inspiration from BPS.Space flight computer projects
- SparkFun and Adafruit for sensor libraries
- PJRC (Paul Stoffregen) for the Teensy 4.1 platform
- PlatformIO for the development environment

## Support

For issues, questions, or contributions, please use the GitHub repository's issue tracker and pull request system.

## AI Assistance

This project utilizes AI assistance for code documentation, system architecture design, and project planning.