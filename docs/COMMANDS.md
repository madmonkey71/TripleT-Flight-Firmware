# Serial Commands Reference

**Version:** v0.51

## Overview

The firmware provides a comprehensive serial command interface for system interaction, diagnostics, calibration, and control. Commands are issued via the USB serial connection at 115200 baud.

## Command Categories

### Flight Control Commands

#### `arm`
Arms the flight computer, transitioning from PAD_IDLE to ARMED state.
- **Requirements**: All sensors must pass health checks for ARMED state
- **Response**: Confirmation message or error if health check fails
- **Usage**: Issue when ready for launch detection

#### `clear_errors`
Attempts to clear ERROR state and transition to PAD_IDLE.
- **Requirements**: Sensor suite must be healthy for PAD_IDLE operation
- **Response**: Success message or explanation of remaining issues
- **Usage**: After addressing sensor problems in ERROR state

#### `clear_to_calibration`
Attempts to clear ERROR state and transition to CALIBRATION.
- **Requirements**: Basic sensors operational but barometer needs calibration
- **Response**: Confirmation or error details
- **Usage**: When barometer needs recalibration after error recovery

### Calibration Commands

#### `calibrate` / `h`
Manually triggers barometer calibration using GPS data.
- **Requirements**: GPS fix (Type ≥ 2, pDOP ≤ 5.0)
- **Process**: Establishes ground level reference for altitude calculations
- **Usage**: Use when barometer needs calibration or recalibration

#### `calibrate_mag`
Starts interactive magnetometer calibration routine.
- **Process**: Multi-step calibration requiring physical rotation
- **Instructions**: Follow on-screen prompts for each calibration step
- **Result**: Bias and scale factors calculated for magnetometer
- **Usage**: Initial setup or when magnetic interference suspected

#### `calibrate_gyro`
Performs gyroscope bias calibration while stationary.
- **Requirements**: Keep system perfectly still during calibration
- **Duration**: Several seconds of sampling
- **Result**: Gyroscope bias offsets calculated and applied
- **Usage**: When gyro drift is observed or initial setup

#### `save_mag_cal`
Saves current magnetometer calibration data to EEPROM.
- **Purpose**: Persist calibration across power cycles
- **Storage**: EEPROM address `MAG_CAL_EEPROM_ADDR`
- **Usage**: After successful magnetometer calibration

### Status & Diagnostic Commands

#### `status_sensors` / `status` / `b`
Displays detailed status information for all sensors and system components.
- **Information**: Sensor health, initialization status, current readings
- **Format**: Comprehensive multi-line status report
- **Usage**: Primary diagnostic command for troubleshooting

#### `sd_status` / `f`
Shows SD card status, available space, and current log file.
- **Information**: Card presence, free space, current log file name
- **Format**: Storage status summary
- **Usage**: Verify logging capability before flight

#### `sensor_requirements`
Displays sensor requirements for each flight state.
- **Information**: Which sensors are required for each state
- **Format**: State-by-state requirements table
- **Usage**: Understanding state transition failures

#### `scan_i2c`
Scans the I2C bus and lists detected devices.
- **Process**: Probes all I2C addresses (0x00-0x7F)
- **Output**: List of detected device addresses
- **Usage**: Hardware debugging and sensor detection

### Debug Control Commands

#### `0` / `debug_serial_csv [on|off]`
Toggles continuous CSV data output over serial.
- **Output**: Real-time CSV data matching SD card log format
- **Usage**: Real-time data monitoring and debugging
- **Note**: High data rate - may impact performance

#### `1` / `debug_system [on|off]`
Toggle system-level debug output.
- **Information**: State transitions, system events, critical operations
- **Usage**: Monitoring flight state machine and system behavior

#### `2` / `debug_imu [on|off]`
Toggle IMU debug output.
- **Information**: ICM-20948 data, Kalman filter outputs, orientation
- **Usage**: Sensor fusion debugging and orientation verification

#### `3` / `debug_gps [on|off]`
Toggle GPS debug output.
- **Information**: GPS fix status, coordinates, satellite data
- **Usage**: GPS functionality verification and troubleshooting

#### `4` / `debug_baro [on|off]`
Toggle barometer debug output.
- **Information**: Pressure readings, altitude calculations, calibration
- **Usage**: Altitude measurement debugging

#### `5` / `debug_storage [on|off]`
Toggle storage system debug output.
- **Information**: SD card operations, file creation, logging status
- **Usage**: Data logging troubleshooting

#### `6` / `debug_icm_raw [on|off]`
Toggle raw ICM-20948 sensor data output.
- **Information**: Raw accelerometer, gyroscope, magnetometer readings
- **Usage**: Low-level sensor debugging

#### `debug_battery [on|off]`
Toggles periodic battery voltage display.
- **Information**: Current battery voltage readings
- **Interval**: Based on `BATTERY_VOLTAGE_READ_INTERVAL_MS`
- **Usage**: Battery status monitoring

### Configuration Commands

#### `set_orientation_filter kalman`
Sets the orientation filter type.
- **Options**: Currently only 'kalman' supported
- **Usage**: Future expansion for multiple filter types

#### `get_orientation_filter`
Shows the currently active orientation filter.
- **Output**: Current filter type name
- **Usage**: Verify active filter configuration

### Utility Commands

#### `help` / `a`
Shows the complete list of available commands.
- **Output**: Formatted command reference with descriptions
- **Usage**: Command discovery and syntax reference

#### `start_log` / `7`
Attempts to initialize SD card and start a new log file.
- **Process**: SD card initialization, file creation with timestamp
- **Usage**: Manual logging restart if SD issues occur

#### `summary` / `j`
Toggles the periodic display of a status summary.
- **Information**: Periodic system status updates
- **Interval**: Configurable update rate
- **Usage**: Continuous system monitoring

## Command Input Format

### Basic Syntax
- Commands are case-sensitive
- Most commands have both full names and single-character shortcuts
- Commands with parameters use space separation
- Boolean parameters accept: `on`, `off`, `true`, `false`, `1`, `0`

### Examples
```
arm
calibrate
debug_system on
debug_imu off
status_sensors
help
```

## Response Format

### Success Responses
- Confirmation messages for successful operations
- Data output for status commands
- Acknowledgment for setting changes

### Error Responses
- Clear error descriptions for failed operations
- Specific reasons for command failures
- Suggestions for resolving issues

### Example Session
```
> status_sensors
=== SENSOR STATUS ===
MS5611 Barometer: ✓ Connected, Calibrated
ICM-20948 IMU: ✓ Connected, Operational
KX134 Accel: ✓ Connected, Operational
GPS: ✓ Connected, Fix Type: 3
SD Card: ✓ Ready, 14.2GB free

> calibrate_mag
Starting magnetometer calibration...
Step 1: Rotate slowly around X-axis...
[Follow prompts]

> arm
System armed successfully. Ready for launch detection.
```

## Debug Output Levels

### Level 0 (CSV)
Continuous comma-separated data output matching log file format.

### Level 1 (System)
State transitions, errors, critical system events.

### Level 2 (IMU)
Orientation data, sensor fusion output, calibration status.

### Level 3 (GPS)
GPS fix status, coordinate updates, satellite information.

### Level 4 (Barometer)
Pressure readings, altitude calculations, calibration data.

### Level 5 (Storage)
SD card operations, file I/O, logging status.

### Level 6 (ICM Raw)
Raw sensor readings from ICM-20948 before processing.

## Integration with Web Interface

Commands can be issued through:
- **Direct Serial**: Terminal programs (Arduino IDE, PuTTY, screen)
- **Web Interface**: Browser-based control via Web Serial API
- **Programmatic**: Automated testing and control scripts

## Error Handling

### Invalid Commands
- Unknown commands display error message and suggest `help`
- Malformed parameters prompt for correct syntax

### State Restrictions
- Some commands only available in specific flight states
- Clear error messages when commands unavailable

### Hardware Dependencies
- Commands requiring specific sensors check availability
- Graceful degradation when hardware unavailable

This command interface provides comprehensive control and monitoring capabilities for all aspects of flight computer operation.