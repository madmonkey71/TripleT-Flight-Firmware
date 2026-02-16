# Error Codes Reference

**Version:** v0.51

## Overview

The firmware uses a comprehensive error code system to help diagnose issues and guide operational decisions. Error codes are logged to `g_last_error_code` for diagnostics and CSV output. **Important distinction**: Not all error codes cause the system to enter ERROR state. Error codes fall into two categories:

- **Hard Errors** (Hardware Failures) → Enter ERROR state, abort operations, red LED
- **Soft Errors** (Control/Guidance Failures) → Graceful degradation, continue flight, orange LED

Error codes are defined in `src/error_codes.h`. For details on ERROR state behavior and transitions, see `FLIGHT_STATE_MACHINE.md`.

## Error Categories: Hard vs Soft Errors

### Hard Errors (Triggers ERROR State)
**Hardware Failures** → System enters ERROR state, aborts flight operations, red LED

These indicate critical hardware failures that prevent safe continued operation:
- Sensor initialization/read failures (codes 10-39): Loss of critical flight data
- Storage failures (codes 50-59): Cannot log telemetry
- State management failures (codes 70-71): System cannot transition to required states
- Configuration errors (codes 250-254): Fundamental system misconfiguration

**Recovery**: Requires manual intervention (`clear_errors` command) or hardware repair

### Soft Errors (Graceful Degradation)
**Control/Guidance Failures** → System continues flight operations, orange LED, feature disabled

These indicate control subsystem failures that don't prevent flight but reduce capabilities:
- `GUIDANCE_STABILITY_FAIL` (code 90+): Guidance system unstable, disables g_guidance_active
- Errors logged to `g_last_error_code` for diagnostics
- Accessible in CSV output and `status_sensors` command
- Flight continues under primary state machine logic

**Rationale**: Guidance control is safety-critical but not primary flight control. Disabling it gracefully is safer than aborting flight entirely. The guidance loop can fail without compromising apogee detection, parachute deployment, or ballistic descent.

---

## Error Code Categories

### System Status (0-9)

| Code | Enum Name | Description |
|------|-----------|-------------|
| 0 | `NO_ERROR` | No error recorded - system operating normally |

### Sensor Initialization Failures (10-19)

| Code | Enum Name | Type | Description & Potential Causes |
|------|-----------|------|--------------------------------|
| 10 | `SENSOR_INIT_FAIL_MS5611` | **Hard** | **MS5611 Barometer initialization failed**<br>• Check I2C wiring (SDA/SCL connections)<br>• Verify 3.3V power supply<br>• Confirm sensor not damaged<br>• Check I2C pull-up resistors (4.7kΩ) |
| 11 | `SENSOR_INIT_FAIL_ICM20948` | **Hard** | **ICM-20948 IMU initialization failed**<br>• Check I2C connections and addressing<br>• Verify power supply stability<br>• Confirm sensor orientation and mounting<br>• Check for I2C address conflicts |
| 12 | `SENSOR_INIT_FAIL_KX134` | **Hard** | **KX134 High-G accelerometer initialization failed**<br>• Verify I2C communication<br>• Check power supply (3.3V)<br>• Confirm sensor presence and health<br>• Validate I2C address configuration |
| 13 | `SENSOR_INIT_FAIL_GPS` | **Hard** | **GPS module initialization failed**<br>• Check I2C or UART connections<br>• Verify GPS module power<br>• Confirm correct communication protocol<br>• Allow time for GPS module startup |

### Sensor Read Failures (30-39)

| Code | Enum Name | Type | Description & Potential Causes |
|------|-----------|------|--------------------------------|
| 30 | `SENSOR_READ_FAIL_MS5611` | **Hard** | **MS5611 data read failure**<br>• Sensor communication lost<br>• Possible sensor hardware failure<br>• I2C bus issues or interference<br>• Temperature-related sensor failure |
| 31 | `SENSOR_READ_FAIL_ICM20948` | **Hard** | **ICM-20948 data read failure**<br>• IMU communication interrupted<br>• Sensor malfunction during operation<br>• I2C bus corruption or noise<br>• Power supply instability |
| 32 | `SENSOR_READ_FAIL_KX134` | **Hard** | **KX134 data read failure**<br>• High-G accelerometer communication lost<br>• Sensor damage from excessive acceleration<br>• I2C communication failure<br>• Power supply issues |
| 33 | `SENSOR_READ_FAIL_GPS` | **Hard** | **GPS PVT data read failure**<br>• GPS signal lost or weak<br>• Antenna connection problems<br>• GPS module malfunction<br>• Repeated communication timeouts |

### Storage System Failures (50-59)

| Code | Enum Name | Type | Description & Potential Causes |
|------|-----------|------|--------------------------------|
| 50 | `SD_CARD_INIT_FAIL` | **Hard** | **SD card initialization failed**<br>• No SD card inserted<br>• SD card corrupted or damaged<br>• Incompatible card format (use FAT32)<br>• Card insertion not properly seated |
| 51 | `SD_CARD_MOUNT_FAIL` | **Hard** | **SD card mount failure**<br>• File system corruption<br>• Unsupported file system format<br>• SD card hardware failure<br>• SDIO interface problems |
| 52 | `LOG_FILE_CREATE_FAIL` | **Hard** | **Log file creation failed**<br>• SD card full (no available space)<br>• Write protection enabled<br>• File system corruption<br>• Insufficient permissions |
| 53 | `SD_CARD_WRITE_FAIL` | **Hard** | **Data write failure during logging**<br>• SD card full during operation<br>• Card corruption during write<br>• SDIO communication failure<br>• Power supply instability |
| 54 | `SD_CARD_LOW_SPACE` | **Hard** | **SD card low space warning**<br>• Available space below minimum threshold<br>• Large log files consuming space<br>• Recommend card cleanup or replacement |

### Calibration Failures (60-69)

| Code | Enum Name | Type | Description & Potential Causes |
|------|-----------|------|--------------------------------|
| 60 | `BARO_CALIBRATION_FAIL_NO_GPS` | **Hard** | **Barometer calibration failed - insufficient GPS**<br>• GPS fix quality too poor (Type < 2)<br>• Position dilution of precision too high (pDOP > 5.0)<br>• GPS antenna obstructed<br>• Insufficient satellite visibility |
| 61 | `BARO_CALIBRATION_FAIL_TIMEOUT` | **Hard** | **Barometer calibration timeout**<br>• GPS fix not acquired within timeout period<br>• Barometer readings unstable<br>• Environmental conditions preventing calibration<br>• Hardware failure during calibration |
| 62 | `MAG_CALIBRATION_LOAD_FAIL` | **Hard** | **Magnetometer calibration load failure**<br>• Invalid EEPROM signature/magic number<br>• EEPROM corruption<br>• First-time use (no previous calibration)<br>• EEPROM hardware failure |

### State Management Failures (70-79)

| Code | Enum Name | Type | Description & Potential Causes |
|------|-----------|------|--------------------------------|
| 70 | `STATE_TRANSITION_INVALID_HEALTH` | **Hard** | **State transition blocked by health check**<br>• Required sensors not operational for target state<br>• Sensor suite health check failed<br>• Critical sensors offline<br>• System not ready for requested state |
| 71 | `ARM_FAIL_HEALTH_CHECK` | **Hard** | **Arm command failed health check**<br>• One or more sensors not ready for ARMED state<br>• Barometer not calibrated<br>• GPS not available when required<br>• Pyro channel continuity failure |

### Guidance & Control Failures (90-99)

| Code | Enum Name | Type | Description & Behavior |
|------|-----------|------|--------------------------------|
| 90 | `GUIDANCE_STABILITY_FAIL` | **Soft** | **Guidance control system stability violation**<br>**Behavior**: Sets `g_guidance_active = false`, continues flight<br>**Triggers**: Angular rate or attitude error exceeds limits<br>**Impact**: Disables guidance PID control, relies on state machine apogee detection and parachute deployment<br>**Rationale**: Guidance is enhancement only, not essential for safe flight<br>**Diagnostic**: See `STABILITY_DEBUG_GUIDE.md` for detailed diagnostics<br>**Recovery**: Logged to CSV and `g_last_error_code`, visible in `status_sensors` |

### EEPROM/Memory Failures (80-89)

| Code | Enum Name | Type | Description & Potential Causes |
|------|-----------|------|--------------------------------|
| 80 | `EEPROM_SIGNATURE_INVALID` | **Hard** | **EEPROM flight state signature invalid**<br>• First boot (no previous state saved)<br>• EEPROM corruption or wear<br>• Invalid signature - using default state<br>• EEPROM hardware failure |

### Configuration Errors (250-254)

| Code | Enum Name | Type | Description & Potential Causes |
|------|-----------|------|--------------------------------|
| 250 | `CONFIG_ERROR_MAIN_PARACHUTE` | **Hard** | **Main parachute configuration error**<br>• `MAIN_PRESENT` disabled in configuration<br>• Deployment system configuration mismatch<br>• Hardware configuration inconsistency |

### General Errors (255)

| Code | Enum Name | Type | Description & Potential Causes |
|------|-----------|------|--------------------------------|
| 255 | `UNKNOWN_ERROR` | **Hard** | **Unspecified error occurred**<br>• Unexpected system condition<br>• Undefined error state<br>• Software bug or edge case<br>• Memory corruption |

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
2. **State Response** (depends on error type):
   - **Hard Errors**: System transitions to ERROR state, aborts operations
   - **Soft Errors**: Feature disabled (e.g., `g_guidance_active = false`), flight continues
3. **Debug Output**: Error details printed if debug enabled
4. **Recovery/Logging**: Error logged to CSV for post-flight analysis
5. **Recovery Attempt**: Automatic or manual recovery mechanisms activated (varies by error type)

### Interpreting Error Patterns

#### Hard Error Patterns

**Intermittent Errors (30-39 Sensor Read Failures)**
- Usually indicate marginal hardware or connections
- Check power supply stability
- Verify I2C connections and pull-ups
- Consider environmental factors (temperature, vibration)

**Initialization Errors (10-19)**
- Often indicate hardware problems or wiring issues
- Use `scan_i2c` to verify sensor detection
- Check power supply voltages
- Verify sensor orientation and mounting

**Storage Errors (50-59)**
- Usually SD card related issues
- Check card format (FAT32), capacity, and quality
- Verify card insertion and connection
- Consider card replacement if persistent

#### Soft Error Patterns

**Guidance Stability Failures (Code 90+)**
- Indicates control loop is unstable but flight can continue
- Does NOT prevent apogee detection or parachute deployment
- See `STABILITY_DEBUG_GUIDE.md` for detailed diagnostics of which parameter exceeded
- Can be reviewed post-flight in CSV output
- May indicate high-G events, motor anomalies, or wind conditions
- Not an emergency condition if flight continues normally

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

**For Hard Errors:**
- **`clear_errors`**: Attempt transition to PAD_IDLE (requires sensor health restored)
- **`clear_to_calibration`**: Attempt transition to CALIBRATION (requires sensor health restored)
- **Specific Commands**: Address root cause (calibration, SD card, etc.)

**For Soft Errors (e.g., Guidance Stability):**
- Automatic: System logs error but continues flight
- No manual recovery needed during flight
- Post-flight: Review `g_last_error_code` in CSV logs and serial output
- Next flight: Adjust configuration parameters based on diagnostics

### Recovery Success Indicators
- **State Transition**: Movement out of ERROR state
- **LED Change**: Status LED changes from red error indication
- **Serial Confirmation**: Success message displayed
- **Normal Operation**: System resumes normal function

This comprehensive error code system enables rapid diagnosis and resolution of system issues during development, testing, and operational use.