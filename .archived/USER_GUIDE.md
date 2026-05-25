# TripleT Flight Firmware - User Guide

**Version:** v0.10.0
**Target Platform:** Teensy 4.1
**Last Updated:** February 2026

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Flight States & Operations](#flight-states--operations)
3. [Command Reference](#command-reference)
4. [Safety Features](#safety-features)
5. [Sensor Systems](#sensor-systems)
6. [Data Logging & Recovery](#data-logging--recovery)
7. [Web Interface](#web-interface)
8. [Recovery Beacon Operations](#recovery-beacon-operations)
9. [Troubleshooting](#troubleshooting)
10. [Advanced Features](#advanced-features)
11. [Specifications](#specifications)
12. [Appendices](#appendices)

---

## Quick Start

### System Overview

The TripleT Flight Firmware is a comprehensive flight control system designed for model rockets. It manages the complete flight sequence from pre-flight checks through recovery, utilizing advanced sensor fusion and multiple redundant safety systems.

**Key Capabilities:**
- Autonomous launch detection and thrust-phase management
- Real-time orientation tracking via Kalman filter sensor fusion
- Multi-method apogee detection (barometric, accelerometer, GPS, timer)
- Dual parachute deployment with pyrotechnic control
- Comprehensive data logging to SD card
- Real-time web-based telemetry and visualization
- Automatic recovery beacon activation with SOS signals
- Complete system diagnostics and self-health monitoring

### Pre-Flight Checklist

Before every flight, perform these checks:

1. **Hardware Verification**
   - [ ] Teensy 4.1 is securely mounted
   - [ ] All sensors are connected (ICM-20948, MS5611, GPS)
   - [ ] KX134 high-G accelerometer is present (if dual deploy)
   - [ ] SD card is inserted and formatted
   - [ ] NeoPixel status LEDs are functioning
   - [ ] Pyro channels 1 and 2 are connected to igniters
   - [ ] Battery voltage is at least 7.5V (for dual deploy)

2. **Software Verification**
   - [ ] Connect via USB serial at 115200 baud
   - [ ] System enters STARTUP state and initializes sensors
   - [ ] Issue `status` command to verify all sensors are ready
   - [ ] Barometer should show reasonable altitude and pressure
   - [ ] GPS should have a lock (watch for satellite count in status)

3. **Calibration**
   - [ ] If this is the first flight or after firmware update:
     - Issue `calibrate` command (system must be in PAD_IDLE or CALIBRATION state)
     - Hold GPS module level and still for calibration
     - Wait for GPS lock and altitude reference to be stored
   - [ ] If barometer was recently calibrated, no recalibration needed

4. **Sensor Health**
   - [ ] Issue `status_sensors` command
   - [ ] Verify all expected sensors show "Ready" status
   - [ ] Check for any "Error" indicators
   - [ ] If errors exist, issue `clear_errors` and retry

5. **Communication**
   - [ ] Enable serial CSV output with `0` command
   - [ ] Verify data is flowing in CSV format
   - [ ] Connect web interface (optional) to monitor telemetry
   - [ ] Disable CSV with `0` command again if using web interface only

### Basic Operation Flow

**Typical flight sequence:**

```
STARTUP (System initializes)
    ↓
CALIBRATION (User calibrates barometer with GPS)
    ↓
PAD_IDLE (Waiting for user command)
    ↓
[User issues: arm]
    ↓
ARMED (Waiting for launch detection)
    ↓
[Rocket launches - acceleration detected]
    ↓
BOOST (Motor burning - acceleration > 2.0g)
    ↓
COAST (Motor burnout - acceleration < 0.5g)
    ↓
APOGEE (Peak altitude detected)
    ↓
DROGUE_DEPLOY (Drogue parachute fires)
    ↓
DROGUE_DESCENT (Descending under drogue)
    ↓
MAIN_DEPLOY (Main parachute fires)
    ↓
MAIN_DESCENT (Descending under main)
    ↓
LANDED (Rocket touches ground)
    ↓
RECOVERY (Data collection and beacon active)
```

---

## Flight States & Operations

The TripleT system uses a 14-state flight state machine, with an additional ERROR state for fault conditions. Each state has specific behaviors and automatic transitions.

### State Descriptions

**STARTUP** (Automatic)
- Duration: ~2-5 seconds after power-on
- System performs initial hardware checks
- Sensors are initialized and tested
- Calibration data is loaded from EEPROM
- NeoPixel shows: **Red (dark)**
- Transition: Automatically moves to CALIBRATION once all hardware is ready

**CALIBRATION** (Manual - Optional First Time)
- User issues `calibrate` or `h` command
- System waits for GPS lock (satellite count displayed)
- Once GPS is locked, press Enter or issue command again
- Barometer altitude is set as reference (0 meters AGL)
- This calibration is saved to EEPROM and persists between flights
- NeoPixel shows: **Orange**
- Transition: Returns to PAD_IDLE automatically after calibration completes

**PAD_IDLE** (Standby)
- System is fully initialized and ready for launch
- Waiting for user to issue `arm` command
- All sensors are monitoring but launch detection is disabled
- Safe to perform final checks and sensor inspections
- NeoPixel shows: **Green**
- Transition: Manual via `arm` command moves to ARMED state

**ARMED** (Ready for Launch)
- Launch detection is now ACTIVE
- Any acceleration above 2.0g will trigger launch detection
- System is consuming sensor power continuously
- No manual input will stop the flight sequence once launch is detected
- NeoPixel shows: **Yellow**
- Transition: Automatic when acceleration > 2.0g moves to BOOST

**BOOST** (Motor Burning)
- Rocket is actively accelerating
- Acceleration threshold: > 0.5g sustained
- Guidance system is active (if enabled) - stabilizing rocket orientation
- Data logging is continuous at high rate
- NeoPixel shows: **Magenta (Purple)**
- Transition: Automatic when acceleration drops below 0.5g moves to COAST

**COAST** (Coasting to Apogee)
- Motor has burned out or separated
- Rocket is still rising but decelerating under gravity
- Apogee detection is ACTIVE (monitoring barometer, accelerometer, GPS)
- Guidance system continues to stabilize
- This state is typically 5-15 seconds depending on altitude target
- NeoPixel shows: **Cyan (Blue-Green)**
- Transition: Automatic when apogee is detected moves to APOGEE

**APOGEE** (Peak Altitude Reached)
- Rocket has reached maximum altitude
- Detected by one of four methods:
  - Barometric pressure rise (descent detected)
  - Accelerometer confirms downward acceleration
  - GPS altitude dropping
  - Backup timer (20 seconds after motor burnout)
- System prepares for parachute deployment
- NeoPixel shows: **White**
- Transition: Automatic after brief confirmation moves to DROGUE_DEPLOY

**DROGUE_DEPLOY** (Drogue Parachute Fires)
- Pyro channel 1 is activated for 1000ms
- Small drogue parachute is deployed to stabilize descent
- System waits for confirmation of deployment
- NeoPixel shows: **Red + Red (both pixels)**
- Transition: Automatic moves to DROGUE_DESCENT

**DROGUE_DESCENT** (Descending Under Drogue)
- Rocket is descending at relatively high velocity under drogue alone
- Typical descent rate: 15-30 m/s
- Main parachute deployment triggers at configured altitude (default: 100m AGL)
- If main not present, system transitions directly to LANDED detection
- NeoPixel shows: **Dark Red (Maroon)**
- Transition: Automatic when altitude drops to deployment threshold moves to MAIN_DEPLOY

**MAIN_DEPLOY** (Main Parachute Fires)
- Pyro channel 2 is activated for 1000ms
- Large main parachute is deployed for final descent
- Descent rate quickly reduces to ~5-8 m/s
- NeoPixel shows: **Red + Red (both pixels)**
- Transition: Automatic moves to MAIN_DESCENT

**MAIN_DESCENT** (Descending Under Main)
- Rocket descends under main parachute at slow, stable rate
- Altitude decreasing gradually toward ground
- Landing detection is active (monitoring acceleration and altitude stability)
- This phase typically lasts 1-3 minutes depending on altitude
- NeoPixel shows: **Dark Red (Maroon)**
- Transition: Automatic when landing is detected moves to LANDED

**LANDED** (On Ground)
- Rocket has touched the ground and settled
- System waits 10 seconds (LANDED_TIMEOUT_MS) to confirm stability
- This prevents false positives from hard landing bounces
- GPS is still updating position
- All actuators are inactive
- NeoPixel shows: **Orange**
- Transition: Automatic after timeout moves to RECOVERY

**RECOVERY** (Post-Flight Data Collection)
- Beacon is activated: LED strobe pattern and SOS audio
- GPS continues to transmit location
- Battery monitoring is active
- System remains in this state for 5 minutes (RECOVERY_TIMEOUT_MS)
- Data logging continues
- Recovery is successful when user physically recovers rocket
- NeoPixel shows: **Flashing pattern**
- Transition: Manual via power-off or automatic after 5 minutes

**ERROR** (Fault Condition)
- System has detected a critical fault
- Possible causes:
  - Sensor initialization failure
  - Barometer not calibrated
  - Multiple consecutive sensor read failures
  - Safety system violations
- Red NeoPixel indicates error state
- Flight is aborted, pyro channels are disabled
- User must issue `clear_errors` command to recover
- System will attempt automatic recovery during ARMED state

### State Transitions & Recovery Procedures

**Manual State Transitions:**
- PAD_IDLE → ARMED: Issue `arm` command
- ERROR → PAD_IDLE: Issue `clear_errors` command (if health check passes)
- ERROR → CALIBRATION: Issue `clear_to_calibration` command (if barometer ready)

**Automatic State Transitions:**
- Trigger: Specific conditions in current state
- Most transitions are irreversible (designed to prevent going backward through flight)
- Once BOOST is reached, rocket is committed to full flight sequence

**Emergency Procedures:**

If the system enters ERROR state during flight operations:

1. **Before Launch (In ARMED or Earlier States):**
   - Issue `status` command to diagnose
   - Issue `status_sensors` for detailed sensor info
   - Issue `clear_errors` to attempt automatic recovery
   - If recovery fails, check sensor connections

2. **During Flight (BOOST Through DESCENT):**
   - ERROR state acts as failsafe - pyro channels are disabled
   - Data logging continues for post-flight analysis
   - Recovery beacon activates
   - Manual search and recovery required

3. **After Flight (In RECOVERY):**
   - Issue `clear_errors` and perform full health check
   - Prepare for next flight or troubleshooting

---

## Command Reference

The TripleT system uses a serial command interface at 115200 baud. Commands are case-insensitive and can be sent via any serial terminal.

### Quick Command Summary

| Command | Shortcut | Function |
|---------|----------|----------|
| `help` | `a` | Display help menu |
| `status` | `b` | Show system status |
| `arm` | - | Arm system for flight |
| `calibrate` | `h` | Calibrate barometer with GPS |
| `clear_errors` | - | Clear error state |
| `status_sensors` | - | Detailed sensor diagnostics |
| Serial CSV | `0` | Toggle CSV output (use for web UI) |
| System Debug | `1` | Toggle general system debug output |
| IMU Debug | `2` | Toggle IMU data output |
| GPS Debug | `3` | Toggle GPS data output |
| Barometer Debug | `4` | Toggle barometer data output |
| Storage Debug | `5` | Toggle SD card debug output |
| ICM Raw Debug | `6` | Toggle raw ICM-20948 output |
| Start Logging | `7` | Attempt to start SD card logging |
| SD Status | `8` | Show SD card status |
| Shutdown | `9` | Prepare system for shutdown |

### Detailed Command Descriptions

**`arm`**
- **Purpose:** Arm the system for launch
- **Requirements:** System must be in PAD_IDLE state
- **Conditions:** All sensors must be healthy
- **Response:**
  ```
  > arm
  Attempting to arm system. Checking health for ARMED state...
  System ARMED. Ready for launch.
  ```
- **Notes:** Once armed, any acceleration > 2.0g triggers launch detection
- **Shortcut:** No single-character shortcut

**`calibrate` or `h`**
- **Purpose:** Calibrate barometer altitude reference using GPS altitude
- **Requirements:** GPS must have a lock (>= 4 satellites)
- **Procedure:**
  1. Hold GPS module level and stable
  2. Issue `calibrate` command
  3. System waits for GPS lock
  4. Once locked, press Enter or issue command again
  5. Barometer offset is calculated and saved
- **Response:**
  ```
  > calibrate
  Attempting to calibrate barometer...
  Waiting for GPS lock...
  [GPS Lock Acquired]
  Calibration complete. Barometer offset saved.
  ```
- **Notes:** Calibration persists across power cycles via EEPROM

**`status_sensors`**
- **Purpose:** Detailed sensor health and diagnostic information
- **Requirements:** None
- **Response:**
  ```
  === SENSOR HEALTH STATUS ===
  ICM-20948:        READY      Accel: 0.00g  Gyro: 0.0°/s
  KX134:            READY      Accel: 0.98g
  MS5611 Barometer: READY      Alt: 145.2m  Press: 1013.25hPa
  GPS Module:       READY      Sats: 12  Fix: 3D  Alt: 156m
  === END SENSOR STATUS ===
  ```
- **Shortcut:** None (too verbose for single character)

**`clear_errors`**
- **Purpose:** Clear ERROR state and return to safe configuration
- **Requirements:** System must be in ERROR state
- **Procedure:**
  1. Issue `clear_errors`
  2. System performs health check
  3. If all sensors healthy, returns to PAD_IDLE
  4. If some sensors failing, provides troubleshooting steps
- **Response:**
  ```
  > clear_errors
  Attempting to clear error state...
  Checking system health for PAD_IDLE state:
  Error state cleared. System reset to PAD_IDLE.
  ```

**`clear_to_calibration`**
- **Purpose:** Alternative error recovery when barometer needs calibration
- **Requirements:** System must be in ERROR state; barometer must be initialized
- **Response:**
  ```
  > clear_to_calibration
  Error state cleared. System reset to CALIBRATION state.
  Use 'calibrate' or 'h' command to calibrate barometer with GPS.
  ```

**Debug Flag Commands**

These commands control what data is printed to the serial console for diagnostics:

- `0` - Toggle **Serial CSV Output** (essential for web interface)
- `1` - Toggle **System Debug** (state transitions, timing)
- `2` - Toggle **IMU Debug** (acceleration, orientation data)
- `3` - Toggle **GPS Debug** (satellite info, position, altitude)
- `4` - Toggle **Barometer Debug** (altitude, pressure, temperature)
- `5` - Toggle **Storage Debug** (SD card operations, logging)
- `6` - Toggle **ICM Raw Debug** (raw sensor values before filtering)
- `7` - Attempt to **Start SD Logging**
- `8` - Show **SD Card Status** and available space
- `9` - **Shutdown** system (safe power-down sequence)

**Example Debug Flag Usage:**
```
> 0
Serial CSV output: ON
[CSV data will now stream every 100ms]

> 1
System debug: ON
[State transitions and timing info will print]

> 0
Serial CSV output: OFF
```

**Alternative Debug Commands:**

Instead of single digits, you can use named debug commands:

- `debug_system on` - Enable system debug
- `debug_system off` - Disable system debug
- `debug_imu` - Toggle IMU debug
- `debug_gps` - Toggle GPS debug
- `debug_baro` - Toggle barometer debug
- `debug_storage` - Toggle storage debug
- `debug_icm_raw` - Toggle ICM raw debug
- `debug_all_off` - Disable all debug flags

**Additional Commands:**

- `status` or `b` - Show complete system status
- `sd_status` - Display SD card status
- `start_log` - Attempt to start logging
- `scan_i2c` - Scan I2C bus for connected devices
- `sensor_requirements` - Show sensor requirements for flight
- `get_orientation_filter` - Display active orientation filter (Kalman)
- `calibrate_mag` - Advanced magnetometer calibration
- `calibrate_gyro` - Advanced gyroscope bias calibration

---

## Safety Features

The TripleT Flight Firmware includes multiple layers of safety systems to ensure reliable and safe flight operations.

### Apogee Detection Methods

The system uses four independent methods to detect apogee (peak altitude), ensuring reliability:

**1. Barometric Method (Primary)**
- Continuously monitors altitude from MS5611 barometer
- Detects when barometer reading increases (indicating descent)
- Requires 5 consecutive readings showing descent (configurable)
- Most reliable in stable conditions
- Threshold: Altitude increase of > 1 meter required to confirm descent
- Confirmation count: 5 consecutive readings

**2. Accelerometer Method (Backup)**
- Monitors Z-axis acceleration from ICM-20948
- Detects when vertical acceleration becomes negative (downward)
- Requires 5 consecutive samples showing negative acceleration
- Works well even with barometer errors
- Threshold: Z-axis acceleration < -0.1g
- Confirmation samples: 5 consecutive

**3. GPS Method (Tertiary)**
- Uses u-blox GPS module for altitude verification
- Detects when GPS altitude reading decreases
- Requires 3 consecutive GPS fixes showing descent
- GPS altitude updates are slower (typically 1Hz)
- Provides geographic reference for entire flight
- Confirmation count: 3 consecutive GPS readings

**4. Backup Timer (Failsafe)**
- Time-based failsafe mechanism
- Triggers automatically if no other method detects apogee
- Time: 20 seconds after motor burnout (when acceleration drops below 0.5g)
- Ensures parachute deployment even if all sensors fail
- Prevents infinite COAST state

**Apogee Confirmation Logic:**
- System requires ANY of the four methods to trigger
- Once triggered, system enters APOGEE state
- Multiple methods provide high confidence in detection
- System resists false positives by requiring confirmation

### Parachute Deployment System

**Dual Deploy Configuration** (Default)
- Drogue parachute at apogee
- Main parachute at 100m above ground level (AGL)
- Provides stable descent under drogue
- Main slows final descent to safe landing speed

**Single Deploy Configuration** (Alternative)
- Only main parachute
- Deployed at apogee
- Simpler configuration for low-altitude flights
- Still benefits from all redundancy systems

**Pyro Channel Activation:**
- Channel 1: Drogue parachute (GPIO Pin 2)
- Channel 2: Main parachute (GPIO Pin 3)
- Activation Duration: 1 second (1000ms) per channel
- Current draw: ~2-3A per channel during firing

**Safety Interlocks:**
- Pyro channels only active when in deployment states
- ERROR state immediately disables all pyro channels
- Pre-flight health check verifies GPIO continuity
- Power supply must provide sufficient current

### Failsafe Mechanisms

**Sensor Failure Recovery:**
- System monitors each sensor for read errors
- Threshold: 3 consecutive failures before error state
- Automatic failover to backup sensors when primary fails
- Kalman filter degrades gracefully with fewer inputs

**Watchdog Timer:**
- Timeout: 1000ms (1 second)
- Resets system if main loop stops executing
- Prevents system lockup during flight
- Automatic restart into RECOVERY state after watchdog reset

**Battery Voltage Monitoring:**
- Continuously monitors supply voltage
- Minimum safe voltage: 7.5V for dual deploy
- Minimum emergency voltage: 6.0V (main only)
- Low voltage warning activates in ERROR state

**Thermal Management:**
- ICM-20948 reports internal temperature
- System monitors for overheating
- Automatic shutdown if temperature > 85°C

**State Persistence:**
- Flight state saved to EEPROM after every state change
- If power is lost during flight, system recovers to last known state
- Prevents loss of deployment sequence

### Error Detection and Recovery

**Error Code System:**
- Each error generates an error code for troubleshooting
- Error code persists until `clear_errors` command is issued
- See Appendices for complete error code reference

**Automatic Recovery:**
- System attempts automatic recovery during ARMED state
- Sensor health checks happen continuously
- Some errors auto-clear if sensors become healthy again

**Manual Recovery:**
- `clear_errors` command initiates recovery process
- System performs full health check
- Returns to PAD_IDLE if all checks pass
- Provides troubleshooting guidance if checks fail

---

## Sensor Systems

The TripleT system uses multiple sensors, each with specific capabilities and roles in flight management.

### ICM-20948 Accelerometer/Gyroscope (Primary IMU)

**Specifications:**
- Range: ±16g for acceleration, ±2000°/s for gyroscope
- Resolution: 16-bit
- Update Rate: 200Hz (configurable)
- Interface: I2C
- Axes: 3 (X, Y, Z for both accel and gyro)

**Functionality:**
- Primary source for acceleration data
- Used for launch detection (threshold: 2.0g)
- Used for boost/coast phase detection
- Provides gyroscope data for orientation tracking
- Integrated into Kalman filter for orientation estimation

**Calibration:**
- Gyroscope bias automatically calibrated on startup
- Magnetometer calibration can be saved to EEPROM
- Manual calibration available via commands:
  - `calibrate_gyro` - Interactive gyroscope bias calibration
  - `calibrate_mag` - Interactive magnetometer calibration
  - `save_mag_cal` - Save calibration to EEPROM

**Data Output:**
- Acceleration (X, Y, Z) in g-forces
- Gyroscope (X, Y, Z) in radians/second
- Temperature in °C
- Quaternion orientation (q0, q1, q2, q3)

### KX134 High-G Accelerometer (Optional)

**Specifications:**
- Range: ±64g (or ±32g selectable)
- Resolution: 16-bit
- Update Rate: 1000Hz (configurable)
- Interface: I2C
- Axes: 3 (X, Y, Z only, no gyro)

**Functionality:**
- Activated during high-acceleration phases
- Detects extremely high-G events during motor burn
- Automatic switching based on acceleration magnitude
- Provides redundancy for launch and boost phase detection
- More robust during intense vibration/shock

**Automatic Switching Logic:**
- If acceleration exceeds 2.0g, KX134 becomes primary
- If KX134 reading < 2.0g, returns to ICM-20948
- Smooth transition between sensors
- User can monitor which sensor is active via debug output

**Enable/Disable:**
- Configured in `config.h`: `#define USE_KX134 1`
- Set to 0 to disable for weight/power savings
- System functions normally without KX134 (degrades gracefully)

### MS5611 Barometer

**Specifications:**
- Altitude Range: -500m to 35,000m
- Accuracy: ±60Pa (~0.5m altitude)
- Resolution: 0.01mb
- Update Rate: 10Hz (configurable)
- Interface: I2C
- Pressure and Temperature sensor

**Functionality:**
- Measures barometric pressure continuously
- Converts to altitude using atmospheric model
- Primary source for apogee detection
- Main parachute deployment triggering
- Landing altitude confirmation

**Calibration Process:**
1. System starts with factory calibration
2. User places module level on ground
3. Issues `calibrate` command
4. System waits for GPS lock (≥4 satellites)
5. GPS altitude becomes reference for barometer (0m AGL)
6. Offset is saved to EEPROM
7. Calibration persists across power cycles

**Altitude Calculation:**
- Barometer measures absolute altitude (above sea level)
- System subtracts calibrated ground reference
- Result is AGL (Above Ground Level) altitude
- Used for all altitude decisions (apogee, main deploy, landing)

### MS5611 Temperature Sensor

**Functionality:**
- Measures ambient temperature
- Improves altitude accuracy via temperature compensation
- Helps detect thermal events
- Logged with every data point

### u-blox GPS Module

**Specifications:**
- Receivers: Multi-constellation (GPS, GLONASS, Galileo, BeiDou)
- Accuracy: ±2.5m (horizontal), ±5m (vertical)
- Update Rate: 1Hz (configurable up to 10Hz)
- Cold Start Time: ~30 seconds
- Interface: SPI (default) or I2C (configurable in `config.h`)

**Functionality:**
- Provides geographic coordinates (latitude, longitude)
- Altitude above ellipsoid (secondary to barometer)
- Ground speed and heading information
- Satellite count and fix quality
- Backup method for apogee detection
- Recovery beacon location transmission

**GPS Data Fields:**
- Latitude/Longitude: ±1e-7 degrees precision (roughly ±1cm)
- Altitude: mm resolution
- Speed: mm/s resolution
- Heading: 1e-5 degrees resolution
- Fix Type: 0=None, 2=2D, 3=3D, 4=D-GNSS
- Satellites in View (SIV): Number of satellites being tracked
- PDOP: Position Dilution of Precision (lower is better)

**Cold Start Procedure:**
- At startup, GPS takes 20-45 seconds to acquire first lock
- System enters PAD_IDLE while awaiting GPS lock
- SOS beacon activates if GPS lock not acquired within timeout
- Calibration requires GPS lock (waits up to 60 seconds)

### Sensor Health Monitoring

**Continuous Health Checks:**
- Each sensor is read in main loop
- Read failures are counted per sensor
- After 3 consecutive read failures, sensor marked unhealthy
- System attempts to continue with healthy sensors

**Pre-Flight Health Verification:**
- `status_sensors` command performs full check
- System verifies at least one IMU (ICM-20948 or KX134) is healthy
- Barometer must be initialized (though not necessarily calibrated)
- GPS lock not required for flight (but recommended)

**Graceful Degradation:**
- Without KX134: System uses only ICM-20948 (still functions)
- Without GPS: System uses only barometer (still functions)
- Without one accelerometer: System uses other (still functions)
- Cannot fly without: Barometer or at least one IMU

### Sensor Calibration Procedures

**Barometer Calibration (Recommended Before Each Flight):**
```
> calibrate
Calibration starting...
Waiting for GPS lock (need 4+ satellites)...
[GPS Lock Acquired after ~45 seconds]
Calibration complete. Ground altitude set as 0m AGL.
New offset saved to EEPROM.
```

**Gyroscope Bias Calibration (Optional, if orientation seems off):**
```
> calibrate_gyro
Hold system level and still for calibration...
Gyroscope bias calibration in progress...
Calibration complete.
```

**Magnetometer Calibration (Optional, for better yaw accuracy):**
```
> calibrate_mag
Follow on-screen prompts...
Rotate system through all orientations...
Magnetometer calibration complete.
> save_mag_cal
Calibration saved to EEPROM.
```

---

## Data Logging & Recovery

The TripleT system logs extensive flight data to an SD card for post-flight analysis and recovery beacon tracking.

### SD Card Logging System

**Log File Format:**
- CSV (Comma-Separated Values) for compatibility
- Human-readable with standard spreadsheet applications
- One row of data per millisecond of flight (configurable)
- 62 data fields per log entry

**Log File Location:**
- SD card root directory: `/`
- File naming: `FLIGHT_XXX.csv` where XXX is sequential number
- New file created for each power cycle or manual start_log command
- System maintains up to 999 flight log files

**Log Data Fields:**
```
seqNum, timestamp, flightState, fixType, sats, latitude, longitude,
altitude, altitudeMSL, raw_altitude, calibrated_altitude, speed,
heading, pDOP, rtk, pressure, temperature, kx134_accel[3], icm_accel[3],
icm_gyro[3], icm_mag[3], icm_temp, q0, q1, q2, q3,
euler_roll, euler_pitch, euler_yaw, gyro_bias[3],
target_roll, target_pitch, target_yaw, pid_roll_integral,
pid_pitch_integral, pid_yaw_integral, actuator_output[3],
battery_voltage, last_error_code, stability_flags, max_pitch_rate,
max_roll_rate, max_yaw_rate, max_pitch_att_err, max_roll_att_err,
max_yaw_att_err, current_target_wp_idx, distance_to_target_wp,
bearing_to_target_wp, altitude_error_to_wp
```

**Log File Access:**

1. **After Flight:**
   - Power off system
   - Remove SD card carefully
   - Insert into card reader on computer

2. **Via USB:**
   - Connect Teensy to computer via USB
   - SD card may be readable as external drive (depends on OS)

3. **Via Web Interface:**
   - Open web interface (`index.html`)
   - Connect to Teensy via Web Serial API
   - Click "Download CSV" to get current flight data

### Log File Management

**Storage Capacity:**
- Minimum free space required: 5MB
- Typical flight log size: 2-5MB per flight hour
- System warns if SD card space is low

**File Retention:**
- All log files are retained indefinitely
- User is responsible for archival and backup
- Recommend regular backup to computer

**Logging Control:**

- **Start Logging:** Issue `7` or `start_log` command
- **Check Status:** Issue `8` or `sd_status` command
- **Statistics:** Issue `f` command
- **Stop Logging:** Automatic on power-off or state change

### Post-Flight Data Analysis

**Using Spreadsheet Software:**

1. Copy CSV file from SD card to computer
2. Open in Excel, Google Sheets, or LibreOffice
3. Data is already formatted as CSV with headers
4. Create charts for altitude vs time, acceleration vs time, etc.

**Using Web Interface:**

1. Open `web_interface/index.html` in Chrome/Edge
2. Connect to Teensy via "Connect" button
3. System loads most recent flight data
4. Web interface provides:
   - Real-time altitude plot
   - Acceleration visualization
   - 3D orientation visualization
   - System state timeline

**Typical Analysis Parameters:**

- Apogee altitude: Compare to rocket predictions
- Descent rates: Check parachute performance
- Acceleration profile: Verify motor thrust curve
- Orientation stability: Confirm attitude control
- Landing altitude: Verify terrain

### GPS Beacon Location Recovery

**GPS Data in Log File:**

Every log entry includes:
- Latitude (degrees × 1e7)
- Longitude (degrees × 1e7)
- Altitude above sea level (meters)

**Extracting Recovery Coordinates:**

1. Open CSV file after recovery
2. Look for RECOVERY state entries (last 300 seconds of flight)
3. Last GPS position provides recovery coordinates
4. Use with Google Maps or GPS device to navigate to rocket

**Example Coordinates:**
```
Latitude:  47.660455 (California coast)
Longitude: -122.312439
Altitude:  2847m (mean sea level)
```

---

## Web Interface

The TripleT system includes a real-time web-based telemetry interface for monitoring and visualization.

### Connecting to Web Interface

**Browser Requirements:**
- Google Chrome (v89+) or Microsoft Edge (v89+) recommended
- Firefox and Safari do not support Web Serial API
- Modern browser with JavaScript enabled

**Connection Steps:**

1. **Open Web Interface:**
   - Navigate to `web_interface/index.html`
   - Or use local server: `python3 -m http.server 8000`
   - Then open `http://localhost:8000`

2. **Connect to Device:**
   - Click "Connect" button in web interface
   - Browser shows list of available serial ports
   - Select Teensy serial port (usually `/dev/ttyACM0` on Linux/Mac, `COM3+` on Windows)
   - Click "Connect"

3. **Verify Connection:**
   - Status shows "Connected"
   - Data should begin streaming
   - Green indicator appears next to "Connection Status"

4. **Start Data Streaming:**
   - Teensy automatically sends CSV data over serial
   - If data not appearing, issue `0` command to enable CSV output
   - Disable CSV on Teensy if interfering with other operations

### Real-Time Telemetry Display

**Main Display Panels:**

1. **System Status Panel**
   - Current flight state
   - Time since state entry
   - Battery voltage
   - System errors (if any)

2. **Altitude Panel**
   - Current altitude (AGL)
   - Maximum altitude reached
   - Descent rate
   - Target altitude (for main deploy)

3. **Acceleration Panel**
   - Current acceleration (g-forces)
   - Maximum acceleration this flight
   - Acceleration in X, Y, Z axes

4. **GPS Status Panel**
   - Satellite count
   - GPS fix type (2D/3D)
   - Position Dilution of Precision (PDOP)
   - Current coordinates

5. **Orientation Panel**
   - Current attitude (roll, pitch, yaw)
   - Attitude rates (in degrees/second)
   - Gyroscope bias values

### Data Visualization

**Altitude Chart:**
- X-axis: Time (seconds since startup)
- Y-axis: Altitude (meters AGL)
- Updates in real-time during flight
- Color-coded by flight state
- Shows apogee marker

**Acceleration Chart:**
- X-axis: Time (seconds)
- Y-axis: Acceleration (g-forces)
- Three traces: X, Y, Z axes
- Shows launch detection threshold (2.0g)
- Shows coast threshold (0.5g)

**3D Orientation Visualization:**
- Real-time 3D model of rocket orientation
- Updates as orientation changes
- Shows roll, pitch, yaw angles
- Helps verify stability control

### Advanced Features

**CSV Data Export:**
- Click "Download CSV" button
- Downloads entire flight log
- File format: Comma-separated values
- Compatible with Excel, Google Sheets, etc.

**Debug Console:**
- Shows raw serial output
- Useful for troubleshooting
- Can be toggled on/off

**Flight State Timeline:**
- Graphical representation of state duration
- Color-coded to match state colors
- Shows all state transitions

---

## Recovery Beacon Operations

After landing, the TripleT system activates a recovery beacon to aid in locating the rocket.

### LED Strobe Patterns

**Status Indicators:**

| State | LED Pattern | Meaning |
|-------|-------------|---------|
| STARTUP | Solid Red (Dark) | System initializing |
| CALIBRATION | Orange | Awaiting calibration completion |
| PAD_IDLE | Solid Green | Ready for launch |
| ARMED | Solid Yellow | Waiting for launch detection |
| BOOST | Magenta/Purple | Motor burning |
| COAST | Cyan/Blue-Green | Coasting to apogee |
| APOGEE | White | Peak altitude reached |
| DROGUE_DEPLOY | Both Red (bright) | Deploying drogue |
| DROGUE_DESCENT | Dark Red | Descending under drogue |
| MAIN_DEPLOY | Both Red (bright) | Deploying main |
| MAIN_DESCENT | Dark Red | Descending under main |
| LANDED | Orange | On ground, stable |
| RECOVERY | Flashing Red | Recovery beacon active |
| ERROR | Red (flashing) | System error |

### Audio Beacon

**SOS Morse Code Pattern:**
- Pattern: Three short beeps, three long beeps, three short beeps
- Repeat interval: 3 seconds between SOS sequences
- Frequency: Configurable (default 2kHz)
- Duration: 1 hour minimum (until battery depleted)

**Activation:**
- Automatically activates in RECOVERY state
- Continues until system power-off
- Audible range: 50-100 meters in quiet conditions

### GPS Beacon Location

**Position Transmission:**
- GPS coordinates logged during RECOVERY state
- Last GPS position available in CSV log file
- Recovery coordinates accurate to ~2.5 meters

**Accessing GPS Position:**

1. **During Flight:**
   - Connect web interface for real-time position
   - Last known coordinates displayed on status panel

2. **After Recovery:**
   - Extract from CSV log file
   - Import into Google Maps or GPS device
   - Navigate to coordinates

---

## Troubleshooting

### Common Issues and Solutions

**Issue: System Enters ERROR State at Startup**

Symptoms:
- Red flashing LED
- Serial output shows "ERROR: Sensor suite unhealthy"
- Cannot arm system

Solutions:
1. Check barometer connection (I2C)
2. Issue `status_sensors` to identify which sensor is failing
3. Check I2C address conflicts with `scan_i2c` command
4. If barometer shows "Not Initialized":
   - Issue `calibrate` command to initialize barometer
   - May need to manually power-cycle after calibration
5. If ICM-20948 shows "Not Ready":
   - Check I2C connection
   - Verify power supply to sensor
   - Check for I2C bus conflicts

**Issue: Cannot Get GPS Lock**

Symptoms:
- GPS shows "0 satellites"
- Calibration cannot complete
- Recovery beacon cannot transmit GPS data

Solutions:
1. Wait longer - GPS cold start takes 30-60 seconds
2. Move GPS module to clear view of sky (avoid trees, buildings)
3. Check GPS antenna connection
4. Verify GPS module power supply (red LED should be on)
5. Reset GPS with power cycle
6. Try GPS indoors near window (satellite signals can penetrate)
7. If GPS still fails:
   - System can fly without GPS (though not ideal)
   - Barometer and accelerometer provide apogee detection
   - Recovery becomes manual search

**Issue: Barometer Calibration Fails**

Symptoms:
- Calibration command appears to work but altitude seems wrong
- Apogee detection doesn't work properly
- Calibration keeps resetting

Solutions:
1. Ensure GPS has lock before calibrating
2. Keep barometer level and still during calibration
3. Wait for altitude to stabilize (readings should not change)
4. Try calibrating in different location (avoid extreme temperatures)
5. If still failing:
   - Check barometer is detecting pressure changes
   - Issue `4` (barometer debug) to see raw readings
   - Raw pressure should be ~1013 hPa at sea level

**Issue: Web Interface Shows "Connection Error"**

Symptoms:
- Cannot connect to Teensy via Web Serial API
- Browser shows "Failed to open serial port"

Solutions:
1. Verify browser supports Web Serial API (Chrome/Edge required)
2. Check USB cable is properly connected
3. Verify Teensy is visible in system device manager
4. Try different USB port
5. Reset Teensy with reset button
6. Reload web interface page
7. Check Chrome version (must be 89+)
8. Try incognito/private browsing mode

**Issue: CSV Data Not Appearing in Web Interface**

Symptoms:
- Connection successful but no data on display
- Web interface status shows "No data received"

Solutions:
1. Issue `0` command to enable CSV output
2. Check baud rate is 115200 (must match web interface setting)
3. Verify data is flowing in serial terminal
4. Reload web interface page
5. Check browser console for JavaScript errors
6. If CSV output is on but not appearing:
   - May be issue with data format
   - Check serial terminal shows valid CSV data
   - Try refresh/reload web page

**Issue: Pyro Channels Not Firing**

Symptoms:
- Parachutes do not deploy at expected times
- No continuity beep when testing
- Pyro test shows "Channel dead"

Solutions:
1. Check physical connections to GPIO pins (2 and 3)
2. Verify igniters are properly installed in parachute deployment charges
3. Check battery voltage - must be > 7.5V for reliable firing
4. Issue `status` to check GPIO continuity
5. If continuity fails:
   - Check for loose connections on Teensy board
   - Verify GPIO pins are not damaged
   - Try different GPIO pins if available
6. Test with multimeter:
   - Remove igniter
   - Set multimeter to continuity
   - Should beep when pyro test fires

**Issue: Altitude Readings Seem Incorrect**

Symptoms:
- Apogee altitude different from expected
- Main deploy altitude inconsistent
- Descent rate calculations wrong

Solutions:
1. Recalibrate barometer (altitude reference may be off)
2. Check barometer is at flight location (not in different altitude)
3. Verify no extreme temperature changes since calibration
4. Compare with GPS altitude (should be similar)
5. Check for barometric pressure system (high/low)
6. If readings still wrong:
   - Issue `4` (barometer debug) to see raw pressure
   - Compare to local weather service
   - If pressure wrong, barometer may need service

**Issue: Acceleration Readings Seem Too High or Low**

Symptoms:
- Launch detection doesn't trigger
- Boost detection misses motor burnout
- Acceleration shown as 0 or constant

Solutions:
1. Verify IMU is mounted level and secure
2. Check IMU has not been physically damaged
3. Recalibrate gyroscope with `calibrate_gyro` command
4. Issue `2` (IMU debug) to view raw accelerometer data
5. Raw data should show ~1g in vertical direction (gravity)
6. If readings are still wrong:
   - Check I2C connection
   - Try `scan_i2c` to verify communication
   - Sensor may need replacement

**Issue: System Randomly Enters ERROR State During Flight**

Symptoms:
- ERROR state occurs unexpectedly
- GPS or barometer suddenly shows as unhealthy
- Pyro channels disable mid-flight

Solutions:
1. Check for vibration issues causing sensor disconnection
2. Verify I2C pull-up resistors are present (4.7k to 5V)
3. Reduce I2C bus speed if electrical noise is present
4. Check for loose wiring connections
5. If error occurs repeatedly:
   - Reduce flight vibration (different motor, frame design)
   - Add shielding to sensor wires
   - Consider sensor placement away from noise sources

### Sensor Health Diagnostics

**Command: `status_sensors`**

Output includes:
- Sensor name
- Initialization status (READY, Error, Not Initialized)
- Current readings
- Last error code (if applicable)

**Command: `scan_i2c`**

Output:
- Lists all I2C devices found on bus
- Shows address of each device
- Helps identify disconnected sensors or address conflicts

**Command: `1` (System Debug)**

Output:
- State transitions with timestamps
- Sensor read results
- Error conditions
- Helpful for understanding system behavior

### Battery Voltage Monitoring

**Command: `status`**

Shows current battery voltage:
- Green: > 8.5V (excellent)
- Yellow: 7.5-8.5V (acceptable)
- Red: < 7.5V (pyro may not fire reliably)

**Voltage Troubleshooting:**
- If voltage dropping during flight: Possible current draw issue
- Check for short circuits
- Verify all sensor power connections
- Replace battery if voltage doesn't hold

### Serial Communication Troubleshooting

**Cannot Connect to Serial Port:**

Solutions:
1. Verify USB cable is properly connected
2. Check device manager for Teensy device
3. On Linux, may need to add user to `dialout` group: `sudo usermod -a -G dialout $USER`
4. Try different terminal software
5. Reset Teensy with reset button
6. Try different USB port

**Garbled Serial Output:**

Solutions:
1. Verify baud rate is 115200
2. Check for loose USB connection
3. Try different serial cable
4. Check for electrical interference near USB cable
5. Terminate with correct line ending (LF or CR+LF depending on terminal)

---

## Advanced Features

### Guidance System and PID Control

**What is Guidance?**

The guidance system maintains stable rocket orientation during powered and unpowered flight. It works by:

1. Reading current orientation from IMU
2. Calculating desired orientation based on trajectory
3. Comparing actual vs desired orientation (error)
4. Using PID controller to move actuators
5. Correcting orientation to track desired path

**When Active:**
- BOOST phase: Stabilizes during motor burn
- COAST phase: Maintains orientation to apogee
- DROGUE_DESCENT phase: Keeps stable under parachute
- MAIN_DESCENT phase: Minimizes swing

**PID Parameters:**
- Can be tuned via `config.h`
- Proportional (P): Immediate response to error
- Integral (I): Accumulates error over time
- Derivative (D): Dampens corrections

**Monitoring Guidance:**
- Web interface shows target vs actual orientation
- CSV logs attitude error and PID values
- Debug output shows actuator commands

### Trajectory Following

**Waypoint Definition:**

Trajectories are defined as series of waypoints:
```
Waypoint 1: Lat 47.660, Long -122.312, Alt 500m
Waypoint 2: Lat 47.661, Long -122.311, Alt 800m
Waypoint 3: Lat 47.662, Long -122.310, Alt 300m
```

**Activation:**

- Requires configuration in firmware
- GPS provides navigation to each waypoint
- Guidance system adjusts trajectory as needed

**Monitoring:**
- CSV logs current target waypoint
- Distance and bearing to target
- Altitude error vs planned altitude

### Stability Monitoring

**Stability Metrics Logged:**
- Maximum pitch/roll/yaw rates (degrees/second)
- Maximum attitude errors (degrees)
- Saturation flags (when control reaches limits)
- Rate of stability violations

**Interpretation:**
- High rates: Rocket may be spinning/unstable
- Low rates: Good stability
- Frequent violations: Control system may need tuning

### High-G Event Handling

**KX134 High-G Accelerometer:**

When acceleration exceeds 2.0g:
- System switches to KX134 (if available)
- High-G sensor can measure up to ±64g
- Automatically switches back when below 2.0g

**Benefits:**
- Detects extreme acceleration events
- Survives high-vibration motor burns
- Provides backup if ICM-20948 saturates

**Monitoring:**
- Debug output shows when sensor switch occurs
- CSV logs data from both sensors

### Watchdog Recovery After System Reset

**What is Watchdog?**

A watchdog timer monitors main loop execution:
- If loop doesn't execute within 1 second, watchdog triggers
- System reboots automatically
- Recovery state is restored from EEPROM

**Automatic Recovery:**

After watchdog reset:
1. System reads last flight state from EEPROM
2. Re-initializes sensors
3. Resumes in last known state
4. If state was ARMED or later, enters RECOVERY (safe state)
5. Disables pyro channels to prevent accidental firing

**Monitoring:**
- System shows "Watchdog reset detected" on startup
- CSV logs indicate gap in data at reset point
- No data is lost - EEPROM preserves flight state

### Custom Configuration Parameters

**File: `config.h`**

Key parameters that can be modified:

```c
// Flight parameters
#define BOOST_ACCEL_THRESHOLD 2.0f      // Launch detection threshold
#define COAST_ACCEL_THRESHOLD 0.5f      // Motor burnout threshold
#define APOGEE_CONFIRMATION_COUNT 5     // Readings to confirm apogee
#define MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M 100.0f  // Main deploy height

// Timeouts
#define BACKUP_APOGEE_TIME_MS 20000     // Failsafe apogee time
#define LANDED_TIMEOUT_MS 10000         // Time to confirm landing
#define RECOVERY_TIMEOUT_MS 300000      // Recovery state duration

// Hardware
#define USE_KX134 1                     // Enable high-G sensor
#define ENABLE_GUIDANCE 1               // Enable guidance system

// Sensors
#define DROGUE_PRESENT true             // Dual deploy
#define MAIN_PRESENT true               // Main parachute required
```

**To Modify:**
1. Edit `src/config.h`
2. Change desired parameters
3. Recompile with `pio run`
4. Upload to Teensy with `pio run -t upload`

---

## Specifications

### Hardware Requirements

**Microcontroller:**
- Teensy 4.1 (ARM Cortex-M7, 600MHz)
- 1MB RAM, 2MB Flash
- USB-C for programming and communication

**Required Sensors:**
- ICM-20948: 6-axis IMU (accel + gyro + mag)
- MS5611: Barometric pressure and temperature
- u-blox GPS module
- Communication via I2C and SPI

**Optional Sensors:**
- KX134: High-G accelerometer (64g range)
- Recommended for high-power rockets

**Data Storage:**
- SD card via SDIO interface
- Supports up to 2TB capacity
- Minimum 5MB free space recommended

**Power Supply:**
- Input voltage: 8.0V - 12.0V
- Typical current: 150mA (sensors + GPS)
- Peak current: 3A+ (during pyro firing)
- Backup battery recommended

**Communication:**
- USB: Programming, serial console, web interface
- Serial (115200 baud): Command interface
- Web Serial API: Real-time telemetry

### Sensor Specifications and Ranges

**ICM-20948:**
- Accelerometer: ±16g, 16-bit
- Gyroscope: ±2000°/s, 16-bit
- Magnetometer: ±1200μT, 16-bit
- Update rate: 200Hz typical

**KX134:**
- Accelerometer: ±64g (or ±32g), 16-bit
- Update rate: 1000Hz typical

**MS5611:**
- Altitude: -500m to 35,000m
- Accuracy: ±60Pa (~0.5m)
- Temperature: -40°C to 85°C
- Update rate: 10Hz typical

**u-blox GPS:**
- Horizontal accuracy: ±2.5m
- Vertical accuracy: ±5m
- Constellations: GPS, GLONASS, Galileo, BeiDou
- Update rate: 1Hz (configurable)

### Communication Interfaces and Baud Rates

**Serial Interface:**
- Baud rate: 115200 (fixed)
- Data format: 8 data bits, 1 stop bit, no parity
- Flow control: None

**CSV Output Rate:**
- Frequency: 100Hz (one line every 10ms)
- Size: ~200-300 bytes per line
- Total bandwidth: ~20-30 KB/s

**Web Serial API:**
- Browser: Chrome or Edge (v89+) only
- Protocol: Standard serial COM port
- Secure context required (HTTPS or localhost)

### Power Requirements

**Minimum Power Budget:**
- Microcontroller: 50mA
- IMU sensors: 50mA
- Barometer: 10mA
- GPS: 50mA
- Data logging: 20mA
- **Total continuous: ~180mA**

**Peak Power (Pyro Firing):**
- Per pyro channel: 2-3A
- Duration: 1 second
- **Total peak: 5-6A**

**Battery Recommendations:**
- LiPo 3S (11.1V nominal): 2000mAh minimum
- NiMH: 10-cell pack (~12V)
- Lead-acid SLA: 12V 2Ah+

**Voltage Regulation:**
- Input: 8.0V - 12.0V
- Output: 5V for sensors, 3.3V for logic
- Must provide 3A+ during pyro firing

### Temperature Operating Range

**Operational:**
- Ambient: -20°C to 60°C
- Sensor electronics: -40°C to 85°C
- Battery: 0°C to 45°C (charged)

**Recommendations:**
- Cold starts (< 0°C): GPS lock may take longer
- Hot days (> 50°C): Ensure adequate airflow
- Extreme cold: Battery capacity reduced 50%

---

## Appendices

### A. Complete Configuration Parameter Reference

**From `config.h`:**

```c
// Firmware Version
#define FIRMWARE_VERSION "v0.10.0"

// Board Type
#define BOARD_TEENSY41

// Guidance System
#define ENABLE_GUIDANCE 1           // 1=Enable, 0=Disable
#define DROGUE_PRESENT true         // Drogue deployment
#define MAIN_PRESENT true           // Main deployment (required)
#define PYRO_CHANNEL_1 2            // Drogue GPIO pin
#define PYRO_CHANNEL_2 3            // Main GPIO pin

// Hardware Options
#define BUZZER_OUTPUT 1             // Buzzer enabled
#define USE_KX134 1                 // High-G sensor enabled
#define GPS_USE_SPI 1               // 1=SPI, 0=I2C
#define GPS_SPI_CS_PIN 10           // GPS CS pin

// Pin Definitions
#define FLASH_CS_PIN 6              // Serial flash
#define NEOPIXEL_PIN 2              // Status LED
#define BUZZER_PIN 9                // Audio beacon

// Flight Logic
#define BOOST_ACCEL_THRESHOLD 2.0f           // Launch threshold (g)
#define COAST_ACCEL_THRESHOLD 0.5f           // Burnout threshold (g)
#define APOGEE_CONFIRMATION_COUNT 5          // Reads to confirm
#define APOGEE_ACCEL_CONFIRMATION_COUNT 5    // Accel reads
#define APOGEE_GPS_CONFIRMATION_COUNT 3      // GPS reads
#define MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M 100.0f  // Deploy height (m)
#define LANDING_CONFIRMATION_COUNT 10        // Landing reads

// Timeouts
#define BACKUP_APOGEE_TIME_MS 20000         // Failsafe apogee (ms)
#define PYRO_FIRE_DURATION 1000             // Fire duration (ms)
#define LANDED_TIMEOUT_MS 10000             // Landing timeout (ms)
#define RECOVERY_TIMEOUT_MS 300000          // Recovery timeout (ms)
#define ERROR_RECOVERY_ATTEMPT_MS 10000     // Error retry (ms)

// Sensor Thresholds
#define MAX_SENSOR_FAILURES 3               // Failures before error
#define WATCHDOG_TIMEOUT_MS 1000            // Watchdog timeout (ms)
#define GPS_TIMEOUT_MS 5000                 // GPS timeout (ms)

// Storage
#define SD_CARD_MIN_FREE_SPACE 5*1024*1024  // Min free (bytes)
#define LOG_PREALLOC_SIZE 5000000           // Preallocate (bytes)
```

### B. LED Color and Strobe Codes

**NeoPixel LED States:**

| State | Color | Brightness | Pattern |
|-------|-------|-----------|---------|
| STARTUP | Red | Dark | Solid |
| CALIBRATION | Orange | Medium | Solid |
| PAD_IDLE | Green | Bright | Solid |
| ARMED | Yellow | Bright | Solid |
| BOOST | Magenta | Bright | Solid |
| COAST | Cyan | Bright | Solid |
| APOGEE | White | Bright | Solid |
| DROGUE_DEPLOY | Red | Bright | Both LEDs |
| DROGUE_DESCENT | Red | Dark | Solid |
| MAIN_DEPLOY | Red | Bright | Both LEDs |
| MAIN_DESCENT | Red | Dark | Solid |
| LANDED | Orange | Medium | Solid |
| RECOVERY | Red | Bright | Flashing 1Hz |
| ERROR | Red | Bright | Flashing 2Hz |

### C. Error Codes and Meanings

**Format:** `ERROR_[SENSOR]_[ISSUE]`

| Error Code | Meaning | Recovery |
|-----------|---------|----------|
| ERR_BARO_INIT | Barometer not initialized | Check I2C connection |
| ERR_BARO_CAL | Barometer not calibrated | Issue `calibrate` command |
| ERR_IMU_INIT | No IMU sensors ready | Check sensor connections |
| ERR_GPS_TIMEOUT | GPS lock not acquired | Move to clear sky, wait |
| ERR_SENSOR_READ | Repeated sensor read failures | Issue `clear_errors` |
| ERR_STATE_INVALID | Invalid state transition | Power cycle system |
| ERR_STORAGE | SD card initialization failed | Check card, reinsert |

### D. Flight State Machine Diagram

```
                    ┌─────────────┐
                    │   STARTUP   │
                    └──────┬──────┘
                           │ (Auto)
                           ▼
                    ┌─────────────┐
                    │ CALIBRATION │◄──┐
                    └──────┬──────┘   │
                           │ (Auto)  (Manual: calibrate)
                           ▼
                    ┌─────────────┐
                    │  PAD_IDLE   │◄──┐
                    └──────┬──────┘   │ (Manual: clear_errors)
                           │          │
                   (Manual: arm)      │
                           │          │
                           ▼          │
                    ┌─────────────┐   │
                    │   ARMED     │───┤
                    └──────┬──────┘   │
                           │          │
                   (Accel > 2.0g)    │
                           │          │
                           ▼          │
                    ┌─────────────┐   │
                    │    BOOST    │   │
                    └──────┬──────┘   │
                           │          │
                   (Accel < 0.5g)    │
                           │          │
                           ▼          │
                    ┌─────────────┐   │
                    │    COAST    │   │
                    └──────┬──────┘   │
                           │          │
                   (Apogee detected)  │
                           │          │
                           ▼          │
                    ┌─────────────┐   │
                    │   APOGEE    │   │
                    └──────┬──────┘   │
                           │ (Auto)   │
                           ▼          │
                    ┌─────────────────────┐
                    │  DROGUE_DEPLOY     │
                    └──────┬──────────────┘
                           │ (Auto)
                           ▼
                    ┌─────────────────────┐
                    │  DROGUE_DESCENT     │
                    └──────┬──────────────┘
                           │
                   (Alt < Deploy Height)
                           │
                           ▼
                    ┌─────────────────────┐
                    │  MAIN_DEPLOY       │
                    └──────┬──────────────┘
                           │ (Auto)
                           ▼
                    ┌─────────────────────┐
                    │  MAIN_DESCENT      │
                    └──────┬──────────────┘
                           │
                   (Landing detected)
                           │
                           ▼
                    ┌─────────────┐
                    │   LANDED    │
                    └──────┬──────┘
                           │ (Auto - 10s)
                           ▼
                    ┌─────────────┐
                    │  RECOVERY   │
                    └─────────────┘
                           │
                   (Any state/condition)
                           │
                           ▼
                        ┌────────┐
                        │ ERROR  │
                        └────────┘
```

### E. Pin Assignments and Connections

**Teensy 4.1 Pin Assignments:**

| Pin | Function | Connected To |
|-----|----------|--------------|
| 0 | Serial RX | GPS Module |
| 1 | Serial TX | GPS Module |
| 2 | NeoPixel | Status LED (DIN) |
| 2 (Alt) | Pyro 1 | Drogue Igniter |
| 3 | Pyro 2 | Main Igniter |
| 6 | Flash CS | Serial Flash |
| 9 | Buzzer | Audio Beacon |
| 10 | GPS CS (SPI) | GPS Module |
| 18 (SDA) | I2C Data | All I2C sensors |
| 19 (SCL) | I2C Clock | All I2C sensors |
| 43 (SD CS) | SD Card | SDIO |
| VUSB | USB Power | USB Input |
| GND | Ground | All devices |

**I2C Sensor Addresses:**
- ICM-20948: 0x68 (default) or 0x69
- MS5611: 0x76 (default) or 0x77
- KX134: 0x1E (default) or 0x1F

### F. Contact Information and Support

**Project Lead:**
- Name: Matthew Thom
- Role: Primary Developer
- Repository: github.com-madmonkey71/TripleT-Flight-Firmware

**Documentation:**
- Main README: `/README.md`
- Gap Analysis: `/UPDATED_GAP_ANALYSIS_2025.md`
- Implementation Plan: `/IMPLEMENTATION_PLAN_2026.md`
- Web Interface: `/web_interface/README.md`

**Issue Reporting:**
- GitHub Issues: Report bugs and feature requests
- Include: System state, error code, sensor status
- Attach: CSV log file if available

**Development Community:**
- Check existing documentation before opening issues
- Review recent commits for breaking changes
- Test with latest firmware version

---

## Quick Reference Card

### Essential Commands

**Pre-Flight:**
```
status              // Check overall system
status_sensors      // Verify all sensors
calibrate          // Calibrate with GPS
arm                // Arm for launch
```

**During Flight:**
```
0                  // Enable CSV for monitoring
[Monitor web interface]
```

**Post-Flight:**
```
0                  // Disable CSV if desired
status             // Check final state
clear_errors       // Clear any errors if present
```

### Emergency Procedures

**If ERROR State Before Launch:**
```
status_sensors     // Identify problem
calibrate          // If barometer issue
clear_errors       // Attempt recovery
```

**If Cannot Arm:**
```
1                  // Enable system debug
status_sensors     // Check sensors
```

**If GPS Not Locking:**
```
3                  // Enable GPS debug
[Move to clear sky location]
[Wait 60 seconds]
```

---

**Document Version:** v1.0
**Last Updated:** February 2026
**Firmware Version:** v0.10.0
**Status:** Beta (Ready for Test Flights)

For the latest information, see `/README.md` and `/CLAUDE.md` in the project repository.
