# Flight Operations

**Version:** v0.51

## Flight State Machine

The firmware operates on a 14-state flight state machine that manages all phases of rocket flight from startup to recovery.

### State Flow Diagram

```
STARTUP → CALIBRATION → PAD_IDLE → ARMED → BOOST → COAST → APOGEE → 
DROGUE_DEPLOY → DROGUE_DESCENT → MAIN_DEPLOY → MAIN_DESCENT → LANDED → RECOVERY

                                  ↓ (on critical error)
                                ERROR
```

## State Descriptions

### 1. STARTUP
**Initial power-on and hardware initialization**
- Hardware component initialization
- Sensor health checks
- EEPROM state recovery (if applicable)
- LED: Dim white
- Duration: ~2-5 seconds

**Requirements:**
- Power-on reset
- Basic hardware functionality

**Transitions:**
- → CALIBRATION: When all sensors initialize successfully
- → ERROR: On critical hardware failure

### 2. CALIBRATION  
**Waiting for barometer calibration with GPS**
- GPS time synchronization
- Barometer ground level calibration
- System readiness checks
- LED: Blue
- Duration: Variable (GPS fix dependent)

**Requirements:**
- GPS fix (Type ≥ 2, pDOP ≤ 5.0)
- MS5611 barometer functional
- ICM-20948 IMU operational

**Transitions:**
- → PAD_IDLE: When barometer calibration completes
- → ERROR: On sensor failure or calibration timeout

### 3. PAD_IDLE
**Ready state, waiting for arm command**
- Continuous sensor monitoring
- Ready to accept arm command
- Background data logging preparation
- LED: Green
- Duration: Indefinite (user controlled)

**Requirements:**
- All sensors operational
- Barometer calibrated
- SD card ready (if logging enabled)

**Transitions:**
- → ARMED: On `arm` command (if sensor health OK)
- → ERROR: On sensor failure

### 4. ARMED
**Armed and ready, monitoring for liftoff**
- Active launch detection monitoring
- Enhanced sensor sampling rates
- Pyro channel safety checks
- LED: Yellow
- Duration: Until launch or disarm

**Requirements:**
- All critical sensors healthy
- Launch detection threshold monitoring
- Pyro channel continuity (if equipped)

**Transitions:**
- → BOOST: When acceleration > `BOOST_ACCEL_THRESHOLD` (default: 2.0g)
- → PAD_IDLE: On timeout or manual disarm
- → ERROR: On critical sensor failure

### 5. BOOST
**Motor burn phase, detecting burnout**
- High-rate data logging
- KX134 accelerometer primary (high-G capable)
- Attitude capture for guidance reference
- Motor burnout detection
- LED: Bright white
- Duration: Typically 1-5 seconds

**Requirements:**
- Accelerometer functional (KX134 preferred)
- Data logging active

**Transitions:**
- → COAST: When acceleration < `COAST_ACCEL_THRESHOLD` (default: 0.5g)
- → ERROR: On critical sensor failure

### 6. COAST
**Coasting to apogee, monitoring for peak altitude**
- Primary apogee detection phase
- Attitude control active (if configured)
- Maximum altitude tracking
- Multiple detection methods active
- LED: Cyan
- Duration: Variable (altitude dependent)

**Requirements:**
- Barometer functional for primary apogee detection
- Backup detection methods available

**Transitions:**
- → APOGEE: On any apogee detection method trigger
- → ERROR: On critical sensor failure

### 7. APOGEE
**Peak altitude reached, preparing for deployment**
- Drogue parachute deployment trigger
- State persistence to EEPROM
- Brief holding state
- LED: Red
- Duration: Immediate transition

**Actions:**
- Activate drogue pyro channel
- Record apogee time and altitude
- Prepare for descent phase

**Transitions:**
- → DROGUE_DEPLOY: Immediate (after pyro activation)

### 8. DROGUE_DEPLOY
**Deploying drogue parachute**
- Pyro channel activation period
- Deployment confirmation monitoring
- LED: Red
- Duration: ~1 second (pyro burn time)

**Actions:**
- Maintain drogue pyro activation
- Monitor deployment success

**Transitions:**
- → DROGUE_DESCENT: After deployment delay

### 9. DROGUE_DESCENT
**Descending under drogue**
- Monitoring descent rate
- Main deployment altitude calculation
- Attitude control during descent (if configured)
- LED: Dark red
- Duration: Variable (altitude dependent)

**Requirements:**
- Altitude monitoring for main deployment trigger

**Transitions:**
- → MAIN_DEPLOY: When altitude ≤ main deployment altitude AGL
- → LANDED: If landing detected during drogue descent

### 10. MAIN_DEPLOY
**Deploying main parachute**
- Main parachute pyro activation
- Final deployment phase
- LED: Blue
- Duration: ~1 second (pyro burn time)

**Actions:**
- Activate main pyro channel
- Prepare for final descent

**Transitions:**
- → MAIN_DESCENT: After deployment delay

### 11. MAIN_DESCENT
**Descending under main parachute**
- Landing detection monitoring
- Reduced descent rate expected
- Pre-landing preparations
- LED: Dark blue
- Duration: Variable (altitude dependent)

**Requirements:**
- Landing detection algorithms active

**Transitions:**
- → LANDED: When landing conditions confirmed

### 12. LANDED
**Touchdown confirmed**
- Landing confirmation period
- Preparation for recovery mode
- Data logging continues
- LED: Purple
- Duration: `LANDED_TIMEOUT_MS` (configurable)

**Actions:**
- Confirm stable landing state
- Prepare recovery systems

**Transitions:**
- → RECOVERY: After timeout period

### 13. RECOVERY
**Post-flight recovery mode with location beacons**
- SOS audio beacon (···---···)
- LED strobe pattern for visual location
- GPS coordinate serial transmission
- Indefinite operation for recovery assistance
- LED: Green strobe pattern
- Duration: Until power off or reset

**Recovery Systems:**
- **Audio Beacon**: Repeating SOS pattern
- **Visual Beacon**: High-intensity LED strobe
- **GPS Beacon**: Serial coordinate transmission
- **Continuous Logging**: Ongoing data recording

### 14. ERROR
**Error state with diagnostic information**
- Sensor failure indication
- Diagnostic information output
- Recovery attempts
- LED: Red (fast flash)
- Duration: Until recovery or manual intervention

**Recovery Mechanisms:**
- Automatic sensor health monitoring
- Grace period protection against oscillation
- Manual recovery commands available

## Redundant Apogee Detection

To ensure reliable parachute deployment, the firmware employs four independent apogee detection methods:

### 1. Primary: Barometric Pressure
- **Method**: Tracks maximum altitude from MS5611 barometer
- **Trigger**: Current altitude consistently lower than maximum
- **Confirmation**: `APOGEE_CONFIRMATION_COUNT` consecutive readings (default: 5)
- **Reliability**: High (primary method)

### 2. Secondary: Accelerometer Freefall
- **Method**: Monitors ICM-20948 Z-axis acceleration
- **Trigger**: Negative g-force (freefall) detection
- **Confirmation**: `APOGEE_ACCEL_CONFIRMATION_COUNT` consecutive readings (default: 5)
- **Reliability**: Medium (backup to barometric)

### 3. Tertiary: GPS Altitude
- **Method**: Monitors u-blox GPS altitude readings
- **Trigger**: GPS altitude consistently lower than recorded maximum
- **Confirmation**: `APOGEE_GPS_CONFIRMATION_COUNT` consecutive readings (default: 3)
- **Hysteresis**: 5-meter threshold to prevent noise triggering
- **Reliability**: Medium (weather and signal dependent)

### 4. Failsafe: Backup Timer
- **Method**: Time-based failsafe from motor burnout
- **Trigger**: `BACKUP_APOGEE_TIME_MS` elapsed since boost end (default: 20 seconds)
- **Purpose**: Ensure deployment even if all sensor methods fail
- **Reliability**: Guaranteed (time-based)

**Any single method triggering will initiate apogee sequence and drogue deployment.**

## Dynamic Main Deployment Altitude

The main parachute deployment altitude is calculated dynamically during the arming process:

### Calculation Method
1. **Ground Level Reference**: Established during barometer calibration
2. **Target AGL**: `MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M` (default: 100m)
3. **Dynamic Calculation**: `g_main_deploy_altitude_m_agl = ground_level + target_agl`
4. **Persistence**: Stored in EEPROM for power-loss recovery

### Deployment Trigger
- **Condition**: Current altitude ≤ calculated main deployment altitude
- **Monitoring**: Active during `DROGUE_DESCENT` state
- **Backup**: Landing detection can trigger early deployment if altitude unreliable

## Safety Features

### Sensor Health Monitoring
- **Continuous Monitoring**: All flight phases except RECOVERY
- **Health Function**: `isSensorSuiteHealthy()` validates sensor status
- **State Requirements**: Different sensor requirements per flight state
- **Automatic Transitions**: To ERROR state on critical failures

### Error Recovery System
- **Automatic Recovery**: Periodic health checks in ERROR state
- **Grace Period**: 5-second protection against oscillation after recovery
- **Manual Recovery**: `clear_errors` and `clear_to_calibration` commands
- **State Persistence**: EEPROM state saving for power-loss scenarios

### Backup Systems
- **Timer Failsafes**: Backup timers for critical transitions
- **Multiple Detection Methods**: Redundancy in apogee detection
- **State Persistence**: Critical data saved to EEPROM
- **Manual Overrides**: Command-based recovery options

## Operational Procedures

### Pre-Flight Checklist
1. **Hardware Verification**: `status_sensors` command
2. **Calibration**: Ensure barometer calibrated (`calibrate` if needed)
3. **SD Card**: Verify logging ready (`sd_status`)
4. **Battery**: Check voltage levels
5. **Pyro Continuity**: Verify deployment circuit continuity
6. **GPS Fix**: Confirm GPS operational for calibration

### Launch Sequence
1. **System Check**: `status_sensors` final verification
2. **Arm System**: `arm` command when ready
3. **Launch Detection**: Automatic transition to BOOST
4. **Flight Management**: Automatic state progression
5. **Recovery**: Follow audio and visual beacons

### Post-Flight Procedures
1. **Data Recovery**: Download SD card log files
2. **System Status**: Check for any error conditions
3. **Battery Check**: Verify remaining power
4. **Hardware Inspection**: Check for damage or loose connections

### Emergency Procedures
- **Error State**: Use `clear_errors` after addressing issues
- **Sensor Failure**: Individual sensor diagnostics available
- **Manual Recovery**: Multiple command-based recovery options
- **Data Preservation**: EEPROM state persistence protects flight data

This comprehensive flight state management system ensures reliable operation throughout all phases of rocket flight with extensive safety features and recovery mechanisms.