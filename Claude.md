# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build and Development Commands

### PlatformIO Commands
- **Build**: `pio run` or `pio run -e teensy41`
- **Upload**: `pio run -t upload` or `pio run -e teensy41 -t upload`
- **Clean**: `pio run -t clean`
- **Monitor Serial**: `pio device monitor --baud 115200`
- **List Libraries**: `pio lib list`
- **Update Dependencies**: `pio lib update`

### Testing
- **GPS Test**: Use files in `test/` directory
- **Compile GPS Test**: Run `test/compile_gps_test.sh` (Linux/Mac) or `test/compile_gps_test.bat` (Windows)

### Web Interface Development
- **Local Server**: Open `web_interface/index.html` in a modern browser with Web Serial API support
- **Testing**: Use `web_interface/test_message_filtering.html` for data parsing validation

## High-Level Architecture

### Flight State Machine
The system centers around a 13-state flight state machine:
`STARTUP → CALIBRATION → PAD_IDLE → ARMED → BOOST → COAST → APOGEE → DROGUE_DEPLOY → DROGUE_DESCENT → MAIN_DEPLOY → MAIN_DESCENT → LANDED → RECOVERY` (plus ERROR state)

Key architectural patterns:
- **State persistence** in EEPROM for power-loss recovery
- **Automatic error recovery** with sensor health monitoring
- **State-dependent processing** - different sensor/control behaviors per state

### Sensor Fusion Architecture
- **Dual accelerometer setup**: ICM-20948 (±16G) for normal flight, KX134 (±64G) for high-G events
- **Automatic switching** based on acceleration magnitude thresholds
- **Kalman filter** for primary orientation estimation (replacing deprecated Madgwick)
- **Multi-sensor apogee detection**: Barometric + accelerometer + GPS + backup timer

### Data Pipeline
Flow: `Sensors (20-100ms) → Kalman Filter → Flight Logic → Guidance Control → Actuators + Data Logging → Web Interface`

### Module Organization
- **Core modules**: `flight_logic.cpp`, `state_management.cpp`, `guidance_control.cpp`
- **Sensor drivers**: `icm_20948_functions.cpp`, `kx134_functions.cpp`, `ms5611_functions.cpp`, `gps_functions.cpp`
- **I/O systems**: `command_processor.cpp`, `log_format_definition.cpp`
- **Web integration**: `web_interface/` directory with real-time data streaming

### Configuration System
- **Primary config**: `src/config.h` - flight parameters, hardware presence, safety limits
- **Debug control**: `src/debug_flags.h` - granular diagnostic output control
- **Data structures**: `src/data_structures.h` - shared types and LogData struct

### Command System
Serial command processor with text-based commands:
- `arm` - Arms flight computer for launch detection
- `status_sensors` - Detailed sensor health report
- `calibrate` - Manual barometer calibration
- `clear_errors` - Manual error state recovery
- Debug flags `1-9` for different subsystem outputs

### Error Handling
- **Hierarchical detection**: Sensor validation → automatic recovery → manual override
- **Grace periods** prevent error state oscillation
- **Degraded operation** continues with partial sensor failures
- **Multiple recovery paths** via commands and automatic health checks

## Development Guidelines

### When Modifying Flight Logic
1. Update configuration in `src/config.h` if adding parameters
2. Consider impact on all flight states - many behaviors are state-dependent
3. Update `LogData` struct and CSV headers if adding logged data
4. Test state transitions thoroughly, especially error recovery paths

### When Adding Sensors
1. Create dedicated function files following pattern: `[sensor]_functions.cpp/.h`
2. Add health monitoring to `isSensorSuiteHealthy()` function
3. Consider adding to Kalman filter if providing orientation data
4. Update command processor for sensor-specific diagnostics

### When Modifying Web Interface
1. Test data parsing with `test_message_filtering.html`
2. Update `flight_console_data_mapping.json` for new data fields
3. Verify CSV data format matches what data parser expects
4. Consider impact on 3D visualization if changing orientation data

### Critical Safety Considerations
- Any changes to apogee detection logic require extensive testing
- Pyro channel control is safety-critical - verify all state transitions
- Error state recovery must not compromise flight safety
- Backup timer systems should remain as final failsafes

### Hardware Dependencies
- **Target platform**: Teensy 4.1 only (ARM Cortex-M7)
- **Required sensors**: ICM-20948, MS5611, GPS module
- **Optional sensors**: KX134 high-G accelerometer
- **I2C bus**: All sensors except GPS use single I2C bus
- **SD card**: Uses built-in Teensy 4.1 SDIO interface

## Common Development Patterns

### Adding New Serial Commands
1. Add command string to `command_processor.cpp`
2. Implement handler function with `SystemStatusContext` parameter
3. Add help text to command list
4. Test both success and error cases

### Extending Data Logging
1. Modify `LogData` struct in `data_structures.h`
2. Update CSV header creation in `log_format_definition.cpp`
3. Update data collection in main firmware loop
4. Verify web interface data parsing handles new fields

### State-Dependent Features
Many systems activate only in specific states:
- **PID control**: Active only in COAST, DROGUE_DESCENT, MAIN_DESCENT
- **Apogee detection**: Active only in COAST state
- **Sensor switching**: Different thresholds per flight phase
- Check `flight_logic.cpp` for state-specific behaviors

