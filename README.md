# TripleT Flight Firmware

**Current Version:** v0.51  
**Current State:** Beta (Ready for Controlled Test Flights)  
**Last Updated:** July 2025

## Project Lead
**Matthew Thom** - Project Lead and Primary Developer

## Overview

This firmware is designed for the **Teensy 4.1** microcontroller and provides comprehensive flight control capabilities for model rockets. The system manages all phases of flight from launch detection through recovery, utilizing a Kalman filter for sensor fusion, robust data logging, and extensive safety features.

## Key Features

- **Multi-Phase Flight Management**: Handles all 14 flight phases from pad idle through recovery.
- **Advanced Sensor Fusion**: Kalman filter combining data from ICM-20948 IMU, KX134 high-G accelerometer, MS5611 barometer, and u-blox GPS.
- **Dual Accelerometer Strategy**: Intelligently uses KX134 for high-G events and ICM-20948 for general flight.
- **Redundant Apogee Detection**: Four distinct methods: barometric, accelerometer, GPS, and backup timer.
- **Comprehensive Recovery System**: SOS audio beacon, LED strobe patterns, and GPS coordinate transmission (Serial).
- **Persistent Calibration**: Magnetometer calibration data is saved to EEPROM, removing the need to recalibrate before every flight.
- **Real-time Data Logging**: 62 data points logged to SD card in CSV format.
- **Interactive Command Interface**: Rich serial command system for diagnostics and control.

## Project Status & Roadmap

Please refer to `UPDATED_GAP_ANALYSIS_2025.md` for the most up-to-date status of features.

**Major Missing Features:**
1. **Live Telemetry:** The wireless link code is currently a placeholder.
2. **Automated Testing:** Unit tests are not yet implemented.
3. **Quaternion GNC:** Orientation logic currently relies on Euler angles.

## Quick Start

1. **Hardware Setup**: Connect sensors via I2C, GPS via serial, insert SD card.
2. **Upload Firmware**: Use PlatformIO with `teensy41` environment.
3. **Initialize**: System performs startup checks and enters `PAD_IDLE` state.
4. **Arm**: Use `arm` command when ready for flight.
5. **Recovery**: If lost, the rocket will emit an SOS beacon and strobe light.

## Documentation

- 📋 **[Gap Analysis](UPDATED_GAP_ANALYSIS_2025.md)** - Current progress and remaining tasks.
- 🔧 **[Hardware Requirements](docs/HARDWARE.md)** - Complete hardware setup and pin configuration.
- 🚀 **[Flight Operations](docs/FLIGHT_OPERATIONS.md)** - Flight states, operations, and safety procedures.
- ⚙️ **[Configuration Guide](docs/CONFIGURATION.md)** - Parameter settings and customization.
- 💻 **[Serial Commands](docs/COMMANDS.md)** - Complete command reference.

## License

This project is licensed under the MIT License.
