# Error Codes Reference

**Version:** v0.51

## Overview

The firmware uses a comprehensive error code system to help diagnose issues. When the system enters an ERROR state, the last recorded error code provides specific information about the failure cause. Error codes are defined in `src/error_codes.h`.

## Error Code Categories

### System Status (0-9)

| Code | Enum Name | Description |
|------|-----------|-------------|
| 0 | `NO_ERROR` | No error recorded - system operating normally |

### Sensor Initialization Failures (10-19)

| Code | Enum Name | Description & Potential Causes |
|------|-----------|--------------------------------|
| 10 | `SENSOR_INIT_FAIL_MS5611` | **MS5611 Barometer initialization failed**<br>• Check I2C wiring (SDA/SCL connections)<br>• Verify 3.3V power supply<br>• Confirm sensor not damaged<br>• Check I2C pull-up resistors (4.7kΩ) |
| 11 | `SENSOR_INIT_FAIL_ICM20948` | **ICM-20948 IMU initialization failed**<br>• Check I2C connections and addressing<br>• Verify power supply stability<br>• Confirm sensor orientation and mounting<br>• Check for I2C address conflicts |
| 12 | `SENSOR_INIT_FAIL_KX134` | **KX134 High-G accelerometer initialization failed**<br>• Verify I2C communication<br>• Check power supply (3.3V)<br>• Confirm sensor presence and health<br>• Validate I2C address configuration |
| 13 | `SENSOR_INIT_FAIL_GPS` | **GPS module initialization failed**<br>• Check I2C or UART connections<br>• Verify GPS module power<br>• Confirm correct communication protocol<br>• Allow time for GPS module startup |

### Sensor Read Failures (30-39)

| Code | Enum Name | Description & Potential Causes |
|------|-----------|--------------------------------|
| 30 | `SENSOR_READ_FAIL_MS5611` | **MS5611 data read failure**<br>• Sensor communication lost<br>• Possible sensor hardware failure<br>• I2C bus issues or interference<br>• Temperature-related sensor failure |
| 31 | `SENSOR_READ_FAIL_ICM20948` | **ICM-20948 data read failure**<br>• IMU communication interrupted<br>• Sensor malfunction during operation<br>• I2C bus corruption or noise<br>• Power supply instability |
| 32 | `SENSOR_READ_FAIL_KX134` | **KX134 data read failure**<br>• High-G accelerometer communication lost<br>• Sensor damage from excessive acceleration<br>• I2C communication failure<br>• Power supply issues |
| 33 | `SENSOR_READ_FAIL_GPS` | **GPS PVT data read failure**<br>• GPS signal lost or weak<br>• Antenna connection problems<br>• GPS module malfunction<br>• Repeated communication timeouts |

### Storage System Failures (50-59)

| Code | Enum Name | Description & Potential Causes |
|------|-----------|--------------------------------|
| 50 | `SD_CARD_INIT_FAIL` | **SD card initialization failed**<br>• No SD card inserted<br>• SD card corrupted or damaged<br>• Incompatible card format (use FAT32)<br>• Card insertion not properly seated |
| 51 | `SD_CARD_MOUNT_FAIL` | **SD card mount failure**<br>• File system corruption<br>• Unsupported file system format<br>• SD card hardware failure<br>• SDIO interface problems |
| 52 | `LOG_FILE_CREATE_FAIL` | **Log file creation failed**<br>• SD card full (no available space)<br>• Write protection enabled<br>• File system corruption<br>• Insufficient permissions |
| 53 | `SD_CARD_WRITE_FAIL` | **Data write failure during logging**<br>• SD card full during operation<br>• Card corruption during write<br>• SDIO communication failure<br>• Power supply instability |
| 54 | `SD_CARD_LOW_SPACE` | **SD card low space warning**<br>• Available space below minimum threshold<br>• Large log files consuming space<br>• Recommend card cleanup or replacement |

### Calibration Failures (60-69)

| Code | Enum Name | Description & Potential Causes |
|------|-----------|--------------------------------|
| 60 | `BARO_CALIBRATION_FAIL_NO_GPS` | **Barometer calibration failed - insufficient GPS**<br>• GPS fix quality too poor (Type < 2)<br>• Position dilution of precision too high (pDOP > 5.0)<br>• GPS antenna obstructed<br>• Insufficient satellite visibility |
| 61 | `BARO_CALIBRATION_FAIL_TIMEOUT` | **Barometer calibration timeout**<br>• GPS fix not acquired within timeout period<br>• Barometer readings unstable<br>• Environmental conditions preventing calibration<br>• Hardware failure during calibration |
| 62 | `MAG_CALIBRATION_LOAD_FAIL` | **Magnetometer calibration load failure**<br>• Invalid EEPROM signature/magic number<br>• EEPROM corruption<br>• First-time use (no previous calibration)<br>• EEPROM hardware failure |

### State Management Failures (70-79)

| Code | Enum Name | Description & Potential Causes |
|------|-----------|--------------------------------|
| 70 | `STATE_TRANSITION_INVALID_HEALTH` | **State transition blocked by health check**<br>• Required sensors not operational for target state<br>• Sensor suite health check failed<br>• Critical sensors offline<br>• System not ready for requested state |
| 71 | `ARM_FAIL_HEALTH_CHECK` | **Arm command failed health check**<br>• One or more sensors not ready for ARMED state<br>• Barometer not calibrated<br>• GPS not available when required<br>• Pyro channel continuity failure |

### EEPROM/Memory Failures (80-89)

| Code | Enum Name | Description & Potential Causes |
|------|-----------|--------------------------------|
| 80 | `EEPROM_SIGNATURE_INVALID` | **EEPROM flight state signature invalid**<br>• First boot (no previous state saved)<br>• EEPROM corruption or wear<br>• Invalid signature - using default state<br>• EEPROM hardware failure |

### Configuration Errors (250-254)

| Code | Enum Name | Description & Potential Causes |
|------|-----------|--------------------------------|
| 250 | `CONFIG_ERROR_MAIN_PARACHUTE` | **Main parachute configuration error**<br>• `MAIN_PRESENT` disabled in configuration<br>• Deployment system configuration mismatch<br>• Hardware configuration inconsistency |

### General Errors (255)

| Code | Enum Name | Description & Potential Causes |
|------|-----------|--------------------------------|
| 255 | `UNKNOWN_ERROR` | **Unspecified error occurred**<br>• Unexpected system condition<br>• Undefined error state<br>• Software bug or edge case<br>• Memory corruption |

## Error Code Usage

### Viewing Error Codes
Error codes can be viewed through:
- **Status Commands**: `status_sensors` shows last error
- **Debug Output**: System debug shows errors as they occur
- **Serial Messages**: Error transitions display codes
- **Log Files**: Errors recorded in CSV logs

### Error Code Context
When an error occurs:
1. **Error Code Set**: `g_last_error_code` updated with specific code
2. **State Transition**: System transitions to ERROR state (if critical)
3. **Debug Output**: Error details printed if debug enabled
4. **Recovery Attempt**: Automatic recovery mechanisms activated

### Interpreting Error Patterns

#### Intermittent Errors (30-39)
- Usually indicate marginal hardware or connections
- Check power supply stability
- Verify I2C connections and pull-ups
- Consider environmental factors (temperature, vibration)

#### Initialization Errors (10-19)
- Often indicate hardware problems or wiring issues
- Use `scan_i2c` to verify sensor detection
- Check power supply voltages
- Verify sensor orientation and mounting

#### Storage Errors (50-59)
- Usually SD card related issues
- Check card format (FAT32), capacity, and quality
- Verify card insertion and connection
- Consider card replacement if persistent

## Troubleshooting by Error Code

### Quick Diagnostic Steps

1. **Check Hardware**: Verify all connections and power
2. **Scan I2C**: Use `scan_i2c` to detect sensors
3. **Check Status**: Use `status_sensors` for comprehensive health
4. **Review Debug**: Enable appropriate debug flags
5. **Clear Errors**: Use `clear_errors` after fixing issues

### Common Resolution Steps

#### For Sensor Errors (10-39):
```
> scan_i2c                  # Check sensor detection
> status_sensors            # Verify sensor health
> debug_system on           # Monitor system behavior
> clear_errors              # Attempt recovery
```

#### For Storage Errors (50-59):
```
> sd_status                 # Check SD card status
> start_log                 # Reinitialize logging
> debug_storage on          # Monitor storage operations
```

#### For Calibration Errors (60-69):
```
> status_sensors            # Check GPS and sensor status
> calibrate                 # Retry calibration
> debug_gps on              # Monitor GPS fix quality
```

## Error Recovery

### Automatic Recovery
The system includes automatic error recovery mechanisms:
- **Health Monitoring**: Continuous sensor health checks
- **Grace Period**: 5-second protection against error oscillation
- **State Recovery**: Automatic transition to appropriate state when healthy

### Manual Recovery
Manual recovery options available:
- **`clear_errors`**: Attempt transition to PAD_IDLE
- **`clear_to_calibration`**: Attempt transition to CALIBRATION
- **Specific Commands**: Address root cause (calibration, etc.)

### Recovery Success Indicators
- **State Transition**: Movement out of ERROR state
- **LED Change**: Status LED changes from red error indication
- **Serial Confirmation**: Success message displayed
- **Normal Operation**: System resumes normal function

This comprehensive error code system enables rapid diagnosis and resolution of system issues during development, testing, and operational use.