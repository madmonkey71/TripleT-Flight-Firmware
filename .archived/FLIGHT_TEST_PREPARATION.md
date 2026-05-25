# FLIGHT TEST PREPARATION GUIDE
## TripleT Flight Firmware v0.10.0+

**Purpose:** Comprehensive guide for preparing, executing, and analyzing successful flight tests of the TripleT Flight Firmware on model rockets using the Teensy 4.1 flight computer.

**Audience:** Flight test teams, range officers, safety personnel, and firmware developers.

**Last Updated:** February 2025
**Applies To:** Firmware versions v0.10.0 and later

---

## EXECUTIVE SUMMARY

### Flight Test Overview

The TripleT Flight Firmware manages model rocket flight through 14 distinct states, from startup through recovery. Each flight test validates one or more subsystems and provides telemetry data for analysis.

**Flight States:** STARTUP → CALIBRATION → PAD_IDLE → ARMED → BOOST → COAST → APOGEE → DROGUE_DEPLOY → DROGUE_DESCENT → MAIN_DEPLOY → MAIN_DESCENT → LANDED → RECOVERY (plus ERROR for fault conditions)

### Success Criteria

A flight test is successful when:
1. **All systems function** - Sensors report valid data throughout flight
2. **Apogee detected** - Two or more apogee detection methods vote correctly
3. **Deployments timed** - Drogue deploys at apogee, main at configured altitude (default 100m AGL)
4. **Data complete** - Full CSV log written to SD card without gaps or corruption
5. **Safe recovery** - Vehicle lands without damage, recovery beacon activates
6. **Predictions match** - Actual apogee, descent rates align with pre-flight calculations

### Safety Review Gates

Before each flight test, obtain sign-off from:
- **Safety Officer** - No hazardous conditions, compliance with RSO/range rules
- **Range Director** - Airspace cleared, weather acceptable, launch slot confirmed
- **Test Director** - All checklist items complete, firmware verified, data retrieval plan established
- **All Personnel** - Understand roles, emergency procedures, and abort criteria

---

## PRE-FLIGHT TEST PHASE
### Duration: 1-2 weeks before launch

### HARDWARE PREPARATION

#### 1. Board Assembly Verification

**Objective:** Ensure all components are properly installed and soldered.

**Procedure:**

1. **Visual Inspection**
   - Examine all solder joints under bright light or magnifying glass
   - Check for cold solder joints (dull, blobby appearance)
   - Verify no missing components
   - Confirm all header pins properly seated in sockets
   - Look for bridged traces or solder bridges between pins

2. **Power Distribution Continuity**
   - Measure resistance between +3.3V rail and GND (should be >100kΩ when powered off)
   - Verify voltage regulator output (should be 3.3V ±0.1V at idle)
   - Check power supply traces for continuity
   - Measure current draw at idle (should be <100mA at 3.3V)

3. **I2C Bus Verification**
   - Measure I2C bus voltage (should be 3.3V nominal)
   - Verify pull-up resistor values (typical: 2.2kΩ to 4.7kΩ on SDA/SCL)
   - Test I2C communication with each slave device:
     - ICM-20948 IMU (address 0x68 or 0x69)
     - MS5611 Barometer (address 0x76 or 0x77)
     - KX134 High-G sensor (address 0x1E or 0x1F) - if present

4. **Connector Inspection**
   - Verify USB connector fit (should have slight resistance, no wobble)
   - Check battery connector polarity (red=positive, black=ground)
   - Test servo connectors for secure fit
   - Verify SD card slot contacts are clean
   - Inspect all cable connections for corrosion or damage

**Expected Results:**
- All solder joints shiny and uniform
- No bridged connections
- I2C bus at 3.3V
- USB communication established
- All connectors secure

---

#### 2. Sensor Calibration

**Objective:** Establish accurate baseline measurements for flight data interpretation.

**ICM-20948 Accelerometer Calibration:**

1. Place flight computer on level surface
2. Connect via USB serial at 115200 baud
3. Issue command: `calibrate_accel`
4. Wait for completion message (approximately 30 seconds)
5. Device will measure +/- 1G on Z-axis to establish zero-g reference
6. Result stored in EEPROM, persists across power cycles

**Expected Output:**
```
Calibrating accelerometer...
Accel calibration complete.
Offset X: [value], Y: [value], Z: [value]
```

**Barometer Calibration (Ground Level Baseline):**

1. Place flight computer where launch will occur
2. Allow 5 minutes for thermal stabilization
3. Verify barometer is reading stable pressure
4. Command: `calibrate_baro` - sets current altitude as reference (0m AGL)
5. Record displayed barometric pressure in hPa
6. This altitude becomes the "ground level" for AGL calculations during flight

**Expected Output:**
```
Barometer calibrated to ground level.
Current pressure: [hPa]
Reference altitude: 0 m AGL
```

**Magnetometer Calibration Procedure:**

1. Power on flight computer in PAD_IDLE state
2. Perform figure-8 motion for 30 seconds to calibrate magnetometer
3. Rotate vehicle around all three axes at least once
4. Calibration data saved to EEPROM automatically
5. Verify no excessive magnetic interference from test area

**Verification Command:** `status_sensors` - should show all sensors HEALTHY

**GPS Receiver Testing:**

1. Power on flight computer and wait 30 seconds
2. Command: `status_sensors` - check GPS section
3. Verify "Satellites in view" increases (typical: 5-12 satellites)
4. Check "GPS Fix Type" shows "3D Fix" or better
5. Allow 5-10 minutes outdoors for cold start if first time
6. Record position and altitude - compare to known location

**Expected Output:**
```
GPS Status: HEALTHY
Satellites in view: 8+
Fix Type: 3D Fix (value: 3)
Latitude/Longitude: [coordinates]
Altitude MSL: [meters]
```

**KX134 High-G Sensor Functional Check:**

1. Verify KX134 present if `USE_KX134 1` in config.h
2. Command: `status_sensors`
3. Check for KX134 entry in output
4. Should show acceleration values near [0, 0, 1]G (1G downward from gravity)
5. Manually accelerate device and observe values change responsively
6. Note: KX134 used only when ICM-20948 would saturate (>16G accelerations)

---

#### 3. Actuator Testing

**Objective:** Verify all control surfaces and deployment systems respond correctly.

**Servo Range of Motion Testing:**

1. Connect servo to Teensy 4.1 pin (see config.h for pins):
   - Pitch: Pin 21
   - Roll: Pin 23
   - Yaw: Pin 20

2. Command: `test_servo [pin] [angle]`
   - Example: `test_servo 21 0` → Full deflection one direction
   - Example: `test_servo 21 90` → Neutral position
   - Example: `test_servo 21 180` → Full deflection other direction

3. Verify servo response:
   - Move observed at each command
   - Smooth motion without jitter
   - Reaches commanded angle within 1 second
   - No grinding or servo strain sounds

4. Document servo response time and range:
   - Min pulse width: 1000μs (0°) to 2000μs (180°)
   - Total range: 180 degrees
   - Dead band: <2° (acceptable)

**Servo Power Supply Capacity Check:**

1. Measure servo power supply voltage while servo is:
   - At rest (should be 5V nominal)
   - Moving (should drop <0.5V from nominal)
   - At maximum torque (stalled against stop)

2. If voltage drops >1V under load:
   - Check power supply capacity (servo can draw 1-2A peak)
   - Verify wiring gauge is adequate (18AWG minimum recommended)
   - Add capacitor (470μF) across servo power if unstable

**Servo Linearity Test:**

1. Command servo to 10° increments (0°, 10°, 20°, ... 180°)
2. Measure actual position with protractor or inclinometer
3. Plot: Commanded vs Actual
4. Linearity should be within 5% across range
5. If significant nonlinearity exists, recalibrate servo min/max pulse widths

**Pyro Channel Continuity Check:**

**CRITICAL SAFETY - Perform with NO CHARGES installed:**

1. Remove battery from flight computer
2. Disconnect pyro charges completely
3. Use multimeter in continuity mode (resistance measurement)
4. Test each pyro channel:
   - Pyro Channel 1 (Pin 2): Should show <1Ω to ground
   - Pyro Channel 2 (Pin 3): Should show <1Ω to ground

5. Verify no shorts between channels or to +3.3V
6. Reconnect battery only after verification complete

**Expected Results:**
- Both channels show continuity (<5Ω)
- No shorts detected
- No continuity to power rail

**Parachute Deployment Charge Circuit Test:**

1. With charges still disconnected, command: `test_pyro 1`
2. Observe:
   - Relay clicks or FET gate signals
   - No sparks or smoke (should be silent switching)
   - Current drain temporarily increases

3. Repeat for `test_pyro 2`

4. Verify firing duration is 1000ms (1 second) as configured

**Final Charge Installation** (only when ready to transport to range):
- Install charges in clips/holders
- Attach deployment charge leads to pyro output pads
- Apply kapton tape over connections
- Double-check polarity and continuity one final time

---

#### 4. Power Systems Testing

**Objective:** Ensure battery and power regulation are adequate for full flight profile.

**Battery Voltage Under Load:**

1. Install flight battery (typical: 3S LiPo = 11.1V nominal)
2. Connect USB (optional, for monitoring via serial)
3. Command: `status_battery` (if battery monitoring enabled)
4. Measure battery voltage with multimeter:
   - At rest: Record voltage (e.g., 12.0V)
   - Commanding servos: Voltage may drop 0.5-1.0V
   - Maximum stress (all servos + high accel): Record minimum

5. Verify voltage never drops below brownout threshold (~10.5V for 3S LiPo)

6. Estimate flight time from voltage decay:
   - Initial voltage: V_start
   - Voltage after 30s servo test: V_end
   - Power per 30s: I = (V_start - V_end) / R_total (where R is equivalent load resistance)
   - Estimated flight time: (V_start - V_min) / I (in similar units to consumption test)

**Voltage Regulator Stability:**

1. Measure 3.3V output from regulator:
   - At idle: Should be 3.3V ±0.05V
   - Under sensor load (all sensors active): Should be 3.3V ±0.1V
   - Under servo load: Should be 3.3V ±0.15V (regulator supplies logic only)

2. If regulator output drops >5% under load:
   - Add 10μF ceramic capacitor near regulator output
   - Add 100μF electrolytic capacitor for filtering
   - Re-measure stability

3. Use oscilloscope if available:
   - Check for ripple (should be <50mV peak-to-peak)
   - Look for oscillation (should be stable, no ringing)

**Brownout Voltage Threshold Test:**

1. Gradually reduce battery voltage (variable power supply) or use battery under load
2. Monitor when microcontroller resets or watchdog triggers
3. Should occur around 10.5V for 3S LiPo (3.5V × 3 cells)
4. Verify system recovers gracefully without data loss
5. Document actual brownout voltage for your setup

**Power Draw Profile:**

Record current consumption in each major operational mode:

| State | Typical Current | Duration | Notes |
|-------|-----------------|----------|-------|
| STARTUP/CALIBRATION | 150mA | 5-10s | Sensors initializing |
| PAD_IDLE (no servos) | 80mA | Variable | Waiting for arm |
| PAD_IDLE (with servo test) | 500-1000mA | 30s | Testing actuators |
| ARMED (no motion) | 100mA | Up to 5 min | Waiting for launch |
| BOOST (servos active) | 800-1200mA | 5-10s | Motor burning |
| COAST (servos active) | 600-1000mA | 5-20s | Unpowered flight |
| DESCENT (servos idle) | 150mA | 30-60s | Coasting down |
| RECOVERY (beacon active) | 200mA | Up to 5 min | SOS beacon + GPS |

**Total Flight Estimate:**
```
Total Charge = Sum of (Current × Duration) for each state
Example: (150mA × 0.008h) + (500mA × 0.17h) + ... = Total mAh
Battery capacity: (e.g., 2200mAh for typical 3S LiPo)
Margin = (Capacity - Total Charge) / Capacity × 100%
Safe if margin > 20%
```

**Expected Results:**
- Battery voltage stable throughout test
- No unexpected resets
- Recovery beacon operates full 5-minute duration
- Margin >20% remaining after simulated flight

---

#### 5. SD Card & Logging Verification

**Objective:** Verify data logging infrastructure works correctly.

**SD Card Preparation:**

1. Format SD card on computer:
   - File system: FAT32
   - Cluster size: 4KB (optimal for frequent writes)
   - Do NOT use exFAT or NTFS

2. Create directory structure:
   - `/logs/` - Flight test log files
   - `/cal/` - Calibration data (if used)

3. Copy to flight computer:
   - Insert SD card into Teensy 4.1 built-in SD slot
   - Power on flight computer
   - Should auto-mount and appear ready

**Logging Functional Test:**

1. Power on flight computer in PAD_IDLE
2. Allow 2-3 sensor cycles (should see status messages)
3. Command: `log_test` (if available) or simulate flight data recording
4. Wait 30 seconds for data writes
5. Command: `status_sd` to verify SD card status

**Expected Output:**
```
SD Card Status: HEALTHY
Free space: [XX] MB
Log file: [filename]
Entries written: [count]
```

6. Remove SD card (safely eject first if via USB)
7. On computer, verify:
   - Log file exists in `/logs/` directory
   - File size >50KB (indicates multiple entries written)
   - File is readable and formatted as CSV

**CSV Format Verification:**

1. Open log file in text editor or spreadsheet
2. First line should be header with 62 column names:
   ```
   seqNum,timestamp,flightState,fixType,sats,latitude,longitude,altitude,altitudeMSL,raw_altitude,calibrated_altitude,...
   ```

3. Verify subsequent rows have data:
   - All numeric fields populated
   - No excessive NaN or error values
   - Timestamps increasing monotonically

4. Check data types:
   - Sequence number: Integer (0, 1, 2, ...)
   - Timestamp: Integer (milliseconds)
   - Flight state: Integer (0-13, matching enum values)
   - Altitude: Float (meters)
   - Acceleration: Float (g-forces)

**Data Corruption Test:**

1. Re-insert SD card into flight computer
2. Perform emergency power-off (remove battery)
3. Re-power and check if SD card mounts
4. Verify last log entry is intact (no truncation)
5. If corruption occurs:
   - Try reformatting with different cluster size
   - Check SD card health (CRC errors)
   - Consider alternative SD card

**Free Space Requirements:**

- Minimum 5MB free space (configured in SD_CARD_MIN_FREE_SPACE)
- Typical flight generates 100-200KB of log data
- Budget 10MB for safety margin and multiple tests

**Expected Results:**
- SD card mounts cleanly
- Log file created with correct format
- 62 columns present with valid data
- File survives power cycle
- >5MB free space available

---

#### 6. Serial Interface Verification

**Objective:** Establish reliable ground monitoring and command interface.

**USB Communication Setup:**

1. Connect Teensy 4.1 to computer via USB cable
2. Identify COM port:
   - Windows: Device Manager → Ports (COM & LPT)
   - Linux: `ls /dev/ttyACM*` or `ls /dev/ttyUSB*`
   - macOS: `ls /dev/tty.usbmodem*`

3. Open serial terminal at 115200 baud:
   - Arduino IDE: Tools → Serial Monitor
   - PlatformIO: `pio device monitor --baud 115200`
   - PuTTY, Minicom, or equivalent

4. Should see startup messages:
   ```
   TripleT Flight Firmware v0.10.0
   Starting up...
   Initializing sensors...
   Flight state: STARTUP
   ```

**Command Processor Responsiveness:**

1. Type command: `help` and press Enter
2. Should receive list of available commands within 1 second
3. Type each command and verify response:
   - `status_sensors` - Full sensor health report
   - `status_sd` - SD card status
   - `arm` - Arm flight computer (only in PAD_IDLE state)
   - `disarm` - Disarm (only in ARMED state)

4. Verify no garbled output or transmission errors
5. Test at different baud rates if available:
   - 9600: May be too slow for real-time monitoring
   - 115200: Recommended for full data throughput
   - 230400: May cause buffer issues on some terminals

**Debug Output Monitoring:**

1. Command: `debug_flags` (if available) to list available debug outputs
2. Enable specific debug outputs for your area of interest:
   - Flight logic debug: Detailed state transitions
   - Sensor debug: Raw sensor values
   - Apogee debug: Voting and detection information
   - Servo debug: Actuator commands

3. Monitor serial output for expected debug messages
4. Verify no excessive CPU load from debug output (should not impact flight)

**Web Interface Connection (Optional):**

1. If using web interface (web_interface/index.html):
2. Ensure Web Serial API supported browser (Chrome, Edge, Opera)
3. Open file in browser
4. Click "Connect" button and select COM port
5. Should establish connection and show green indicator
6. Verify real-time data display updates
7. Check data parsing for accuracy (compare with serial terminal)

**Expected Results:**
- USB connection established
- Serial terminal shows startup messages
- Commands respond within 1 second
- All debug output visible and sensible
- Web interface connects and displays data (if used)
- No serial transmission errors or corruption

---

### FIRMWARE PREPARATION

#### 1. Build Verification

**Objective:** Ensure firmware compiles correctly and is ready for deployment.

**Compilation for Teensy 4.1:**

1. Install PlatformIO (if not already done):
   - Install VS Code extension: PlatformIO IDE
   - Or use command-line: `pip install platformio`

2. Navigate to project directory:
   ```bash
   cd /mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware
   ```

3. Build firmware:
   ```bash
   pio run -e teensy41
   ```

4. Watch for compilation output:
   - Should complete successfully
   - Binary size should be approximately 145-160KB
   - Verify no linker errors

5. Expected output:
   ```
   Checking size .pio/build/teensy41/firmware.elf
   Data:    [=====     ] 38.9% (used 31936 bytes from 82030 bytes)
   Program: [===>      ] 41.5% (used 271916 bytes from 655360 bytes)
   ```

**Compiler Warnings & Errors:**

1. Review any compiler warnings:
   - Unused variables: Generally harmless, but clean up if possible
   - Type mismatches: Must be resolved before flight
   - Implicit conversions: Verify accuracy (e.g., float to int)

2. Resolve any errors immediately:
   - Missing includes: Verify library installed
   - Undefined functions: Check function names and headers
   - Configuration conflicts: Review config.h for contradictions

3. Common issues and fixes:
   - **"Board not found"** → Install Teensy board definitions
   - **"SD card library error"** → Verify SdFat library installed
   - **"Sensor library missing"** → Run `pio lib install`

**Binary Size Verification:**

1. Optimal binary size: 140-160KB
2. If >160KB:
   - Check if debug code is still enabled (should be commented out for flight)
   - Verify no unnecessary libraries linked
   - Consider optimizing large data structures

3. If <140KB:
   - Likely missing functionality
   - Review config.h to ensure features are enabled

---

#### 2. Configuration Review

**Objective:** Verify firmware configuration matches flight hardware and mission requirements.

**Critical Configuration Parameters** (src/config.h):

Review and document these settings before each flight:

**Flight Parameters:**
```c
MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M 100.0f  // Main chute deployment altitude
BOOST_ACCEL_THRESHOLD 2.0f                 // Launch detection (2.0g)
COAST_ACCEL_THRESHOLD 0.5f                 // Motor burnout (0.5g)
BACKUP_APOGEE_TIME_MS 20000                // Apogee timeout (20s failsafe)
```

**Parachute Configuration:**
```c
DROGUE_PRESENT true              // Drogue chute installed?
MAIN_PRESENT true                // Main chute installed?
PYRO_CHANNEL_1 2                 // Drogue pyro pin
PYRO_CHANNEL_2 3                 // Main pyro pin
```

**Guidance System:**
```c
ENABLE_GUIDANCE 1                // Servo control: 1=on, 0=off
ACTUATOR_PITCH_PIN 21            // Pitch servo pin
ACTUATOR_ROLL_PIN 23             // Roll servo pin
ACTUATOR_YAW_PIN 20              // Yaw servo pin
```

**Sensor Options:**
```c
USE_KX134 1                       // High-G sensor: 1=enabled, 0=disabled
GPS_USE_SPI 1                     // GPS interface: 1=SPI, 0=I2C
KALMAN_FILTER_ACTIVE_BY_DEFAULT true  // Always use Kalman filter
```

**Data Logging:**
```c
SD_CARD_MIN_FREE_SPACE 5*1024*1024  // Minimum 5MB free
DISABLE_SDCARD_LOGGING false        // Enable logging
```

**Apogee Detection Thresholds:**
```c
APOGEE_BARO_DESCENT_THRESHOLD 1.0   // Barometer: 1m descent = apogee vote
APOGEE_ACCEL_THRESHOLD -0.1         // Accelerometer: -0.1g = vote
APOGEE_ACCEL_SAMPLES 5              // Need 5 consecutive samples
```

**Verification Checklist:**
- [ ] MAIN_PRESENT = true (required)
- [ ] DROGUE_PRESENT matches vehicle design
- [ ] Servo pins correct for your Teensy revision
- [ ] Pyro pins 2 and 3 (do not change)
- [ ] ENABLE_GUIDANCE matches mission type (1 for active control)
- [ ] GPS_USE_SPI matches your hardware
- [ ] Apogee thresholds conservative (prevent false triggers)
- [ ] SD logging enabled unless testing
- [ ] All compiler constants defined

**Servo Pulse Width Calibration:**

If servos are unresponsive during test:

1. Modify servo pulse widths (config.h):
   ```c
   #define SERVO_MIN_PULSE_WIDTH 1000  // Adjust +/-50μs if needed
   #define SERVO_MAX_PULSE_WIDTH 2000
   ```

2. Test with commands:
   ```
   test_servo 21 0      // Full deflection
   test_servo 21 180    // Other direction
   ```

3. If servo doesn't respond or responds incorrectly:
   - Increase pulse width range (e.g., 900-2100)
   - Or decrease range (e.g., 1050-1950)
   - Some servos have different response ranges

4. Re-test and document final values used

**Debug Flag Configuration:**

For first flights, enable some debug output to monitor system:

1. Identify debug flags (in debug_flags.h or config.h):
   - Flag 1: Flight logic debug
   - Flag 2: Sensor debug
   - Flag 3: Apogee detection debug
   - etc.

2. Enable via command:
   ```
   debug_flags 1 1  // Enable flight logic debug
   ```

3. Verify debug output doesn't slow system (monitor CPU usage)
4. For production flights, disable all debug after verification

---

#### 3. Upload & Verification

**Objective:** Deploy firmware and verify successful installation.

**Upload to Teensy 4.1:**

1. Connect Teensy to computer via USB
2. Upload firmware:
   ```bash
   pio run -e teensy41 -t upload
   ```

3. Watch for upload progress:
   - Should show transfer progress
   - Teensy LED will blink during programming
   - Upload completes within 10 seconds

4. Expected output:
   ```
   Uploading .pio/build/teensy41/firmware.bin
   ...
   [100%] Verifying firmware
   Upload complete!
   ```

**Post-Upload Verification:**

1. **Serial Connection Test:**
   - Open serial monitor immediately after upload
   - Should see startup messages within 2 seconds
   - Device automatically resets after programming

2. **Initialization Sequence:**
   Expected sequence in serial output:
   ```
   TripleT Flight Firmware v0.10.0
   Target: Teensy 4.1
   Starting up...

   Initializing sensors...
   ICM-20948: OK
   MS5611: OK
   KX134: OK
   GPS: Initializing...

   Sensors initialized.
   Flight state: STARTUP → CALIBRATION

   Calibration complete.
   Flight state: CALIBRATION → PAD_IDLE

   Ready for arm command.
   ```

3. **Sensor Health Check:**
   ```
   Command: status_sensors

   ICM-20948 Accelerometer: HEALTHY
   - X: 0.02g, Y: -0.01g, Z: 1.00g ✓
   MS5611 Barometer: HEALTHY
   - Pressure: 1013.25 hPa
   - Altitude: 0.0 m AGL ✓
   KX134: HEALTHY
   - Accel: [values] ✓
   GPS: ACQUIRING
   - Satellites in view: 7
   Status: Healthy (7 votes)
   ```

4. **Command Responsiveness:**
   - Type: `arm`
   - Verify: Flight state changes to ARMED
   - Type: `disarm`
   - Verify: Flight state returns to PAD_IDLE
   - All commands respond within 500ms

5. **LED Indicator Test:**
   - Observe NeoPixel(s) on board
   - Should show:
     - Blue during STARTUP/CALIBRATION
     - Green in PAD_IDLE state
     - Yellow when ARMED
     - Red if any errors

**Firmware Integrity Verification:**

1. Check firmware size matches build output
2. Verify no repeated uploads needed (successful first attempt)
3. Confirm no watchdog resets (system stable for >1 minute)
4. Test emergency commands:
   - `clear_errors` - Should clear any error states
   - `reboot` - System should reset gracefully

**Expected Results:**
- Upload completes in <15 seconds
- Startup messages appear immediately
- All sensors report HEALTHY
- State transitions work correctly
- LED indicators function properly
- No watchdog resets or errors

---

### RANGE PREPARATION

#### 1. Safety Review

**Objective:** Ensure flight complies with regulations and poses no hazard.

**Pre-Flight Safety Checklist:**

**Airspace & Legal:**
- [ ] Notam filed (if required by location)
- [ ] Airspace clearance confirmed with ATC/FAA
- [ ] Waiver approved (if altitude >400 feet AGL or>excluded airspace)
- [ ] Range waiver reviewed by RSO (Range Safety Officer)
- [ ] Launch location at least 500 feet from people not involved in flight

**Weather Conditions:**
- [ ] Wind speed <15 mph at launch
- [ ] No precipitation or heavy cloud cover
- [ ] Visibility >1 mile
- [ ] Temperature 0-40°C (acceptable operating range)
- [ ] Barometer trending stable (no rapid pressure changes)
- [ ] No lightning risk (clear skies expected for next 2 hours)

**Range Setup:**
- [ ] Launch pad level and stable
- [ ] No obstructions in flight path (buildings, power lines, etc.)
- [ ] Spectator area secured (min 500 feet downrange)
- [ ] Emergency response plan documented
- [ ] First aid kit available
- [ ] Fire extinguisher on site
- [ ] Communications plan established (radio/phone)

**Flight Computer Safety:**
- [ ] Firmware version recorded and approved
- [ ] Config.h reviewed and signed off
- [ ] Apogee detection methods verified (at least 2)
- [ ] Parachute deployment charges tested
- [ ] Backup timer functional
- [ ] All sensors report healthy

**Personnel & Roles:**
- [ ] Test Director appointed (flight controller)
- [ ] Safety Officer assigned (abort authority)
- [ ] Range Officer (range operations)
- [ ] Data Officer (log retrieval, analysis)
- [ ] Recovery Team (post-flight retrieval)
- [ ] All roles briefed on their responsibilities

---

#### 2. Rocket Preparation

**Objective:** Verify vehicle structure, parachutes, and aerodynamics.

**Structural Integrity Inspection:**

1. **Airframe:**
   - No cracks, dents, or deformations
   - All fins securely bonded
   - Nose cone fitted properly
   - Fin alignment ±2° of vertical

2. **Parachutes:**
   - Drogue chute: Packed and attached to flight computer
   - Main chute: Packed and attached to flight computer
   - Deployment bags intact (no holes)
   - No tangled lines

3. **Recovery System:**
   - Drogue bridle attached to booster (if two-stage)
   - Main chute bridle correct length
   - All attachment points secure
   - No loose lines trailing

**Parachute Packing Technique:**

1. **Drogue Parachute:**
   - Fold into z-fold pattern
   - Place in deployment bag with bridle
   - Attach bag to drogue charge via clip
   - Test clip by hand (should withstand manual pull)

2. **Main Parachute:**
   - Fold into three-inch flakes
   - Place in deployment bag with risers
   - Ensure no lines pinched
   - Attach bag to main charge via clip
   - Verify clips are identical and secure

3. **Verification:**
   - Both chutes packed identically to previous successful flights
   - Pack order: Parachute → bag → charge
   - No charge should deploy other than intentionally
   - Simulation: Manually simulate deployment (should pull out smoothly)

**Pyro Charge Installation:**

**CRITICAL SAFETY - Only trained personnel under supervision:**

1. **Location:** Designated safe area away from people
2. **Charges:**
   - Inspect each charge for corrosion or defects
   - Verify charge weight (should be consistent)
   - Check expiration date (if applicable)
3. **Installation:**
   - Insert charge into charge holder on flight computer
   - Secure with shielding cap or retaining ring
   - Verify charge cannot fall out
   - Test clip integrity (should not deploy under normal handling)
4. **Documentation:**
   - Record charge manufacturer and type
   - Document installation time and person
   - Note any anomalies

**Vehicle Weight & CG (Center of Gravity):**

1. **Total Weight:**
   - Measure complete vehicle weight
   - Document in flight plan
   - Compare to pre-flight prediction
   - Difference >5% may indicate problem

2. **CG Location:**
   - Measure distance from nose cone to CG
   - Record in terms of body tube diameters or length percentage
   - Typical: CG should be 1-2 body diameters behind nose
   - If CG too far back: Vehicle may become unstable
   - If CG too far forward: Vehicle may not rotate during boost

3. **CG Verification Method:**
   - Balance rocket horizontally on knife edge
   - Mark balance point
   - Measure distance from nose to balance point
   - Document and compare to prediction

**Stability Analysis (Static Stability):**

1. **Fin Design Review:**
   - Total fin area adequate for rocket diameter
   - Fin positioning symmetric around fuselage
   - No asymmetric damage or deflection

2. **Drop Test (if feasible):**
   - Hold rocket nose-up at 10 feet
   - Release and observe descent
   - Should rotate to fly base-first (stable)
   - If tumbles or spins unpredictably: Instability issue

3. **Simulation (if drop test not feasible):**
   - Use OpenRocket or RockSim simulation
   - Compare predicted stability margin to actual design
   - Should show >2X stability margin
   - Document simulation results

**Motor Selection & Installation:**

1. **Motor Specifications:**
   - Total impulse: [Ns] (e.g., J class motor ~240Ns)
   - Peak thrust: [N] (will affect boost acceleration)
   - Average thrust curve provided by manufacturer
   - Propellant type (Black powder, composite, etc.)

2. **Motor Installation:**
   - Verify motor fits snugly in motor mount
   - Install retainer ring or other locking mechanism
   - Secure motor cannot move during flight
   - Apply RTV or epoxy if necessary (per manufacturer)

3. **Safety Wires or Clips:**
   - If motor has lug, ensure clip in place
   - Verify motor cannot come out during burnout or recovery
   - No gaps between motor casing and mount

4. **Expected Motor Burnout Time:**
   - Document from thrust curve (typically 5-15 seconds)
   - Record expected apogee from simulation
   - Use this to set apogee timeout calibration

**Final Assembly Check:**

Checklist before vehicle moves to launch pad:
- [ ] All epoxy joints fully cured (24+ hours)
- [ ] No gaps or separation visible
- [ ] Parachutes packed and loaded (charges NOT yet installed)
- [ ] Pyro charges installed (only if in designated safe area)
- [ ] Motor installed and secured
- [ ] CG measured and documented
- [ ] Fins aligned and secure
- [ ] Nose cone sits flush
- [ ] All exterior clean (no loose debris)
- [ ] Flight computer mounted securely
- [ ] Antenna (GPS) unobstructed
- [ ] Vent holes (if applicable) clear
- [ ] Recovery beacon tested
- [ ] Overall weight and dimensions match prediction

---

#### 3. Launch Equipment Setup

**Objective:** Prepare ground support systems for safe and reliable launch.

**Launch Pad Continuity Checks:**

**CRITICAL SAFETY - No pyro charges on rocket during these checks:**

1. **Pad Wiring Verification:**
   - Continuity from launch controller to flight computer
   - Verify correct wiring:
     - Signal line to Teensy GPIO pin
     - Ground to Teensy GND
     - 12V power (if needed) to power supply

2. **Continuity Test on Launch Pad:**
   - Measure continuity from pad connector to flight computer
   - Should show <5Ω resistance
   - No shorts to power or other signals

3. **Load Test:**
   - With charges installed, verify pad can handle current draw
   - Test with igniter installed (simulated charge)
   - Measure voltage drop when "firing" (should be <1V)

**Ground Support Electronics:**

1. **Battery:**
   - Voltage: Adequate for servo power (typical 5V or battery voltage)
   - Capacity: 2000+ mAh (can deliver sustained current)
   - Health: No swelling or damage
   - Fully charged before flight

2. **Pyro Firing Circuit (if using separate controller):**
   - Safety switch or key interlock present
   - Red LED indicating power armed
   - Continuity indicator for each channel
   - Current limiting resistors in place
   - Fusing: 10-15A breaker per channel

3. **Launch Controller Safeguards:**
   - Two-step arming (safety switch + button)
   - Key-operated arming (cannot be accidental)
   - Clear labeling of channels
   - Documented firing sequence

**Battery Installation in Vehicle:**

1. **Battery Insertion:**
   - Match polarity carefully (red to +, black to -)
   - Secure battery with velcro or clips (cannot shift during flight)
   - Leave accessible for post-flight removal

2. **Voltage Check:**
   - Measure battery voltage immediately after installation
   - Should be within 10% of nominal
   - If voltage drops >5% quickly, battery may be defective

3. **Power Stabilization:**
   - Allow 30 seconds after battery insertion before arming
   - Flight computer completes initialization
   - All sensors establish baseline readings

**Final Sensor Health Checks:**

1. **15 Minutes Before Launch:**
   - Command: `status_sensors`
   - All sensors should report HEALTHY
   - No ERROR or WARNING flags

2. **5 Minutes Before Launch:**
   - Repeat `status_sensors`
   - Verify readings are fresh (timestamp recent)
   - Confirm no errors have developed

3. **At Launch Pad (1 minute before arm):**
   - Visual inspection of flight computer
   - No loose connectors
   - No corrosion or damage visible
   - LED indicators functioning (if visible)

**SD Card Final Verification:**

1. **SD Card Check:**
   - Verify SD card physically mounted
   - Should show green LED if inserted properly
   - Command: `status_sd` returns HEALTHY

2. **Free Space Confirmation:**
   - Verify >5MB free space (shown in status_sd output)
   - No previously unclosed log files

3. **Expected Log File:**
   - Log file created in `/logs/` directory
   - Filename: `flight_YYYYMMDD_HHMMSS.csv` (or similar)
   - File is writable (not read-only)

**Serial Connection Functional:**

1. **Last Communication Test:**
   - 30 seconds before arm
   - Send: `status_all` (summary of all systems)
   - Verify response within 1 second
   - No garbled output or errors

2. **Disconnect Option:**
   - Can disconnect USB if using battery only
   - Or leave connected to monitor flight via USB serial
   - If leaving connected, verify USB won't interfere with vehicle
   - Secure USB cable so it doesn't snag recovery equipment

**Expected Results:**
- All continuity checks pass (<5Ω)
- Battery voltage stable
- All sensors HEALTHY
- SD card ready with >5MB free
- Serial communication responsive
- Launch pad cleared and ready

---

## FLIGHT TEST PHASES

### PHASE 1: PAD IDLE TEST
**Timing:** 15 minutes before launch
**Objective:** Final verification that all systems ready, capture baseline calibration data

**Procedure:**

1. **Power-On Sequence (5 minutes before test):**
   - Install battery in flight computer
   - Observe LED sequence (should progress through startup colors)
   - Wait 5 seconds for initialization

2. **Serial Monitoring (if available):**
   - Open serial terminal (115200 baud)
   - Should see:
     ```
     TripleT Flight Firmware v0.10.0
     Flight state: STARTUP
     [sensor initialization messages]
     Flight state: CALIBRATION
     [calibration messages]
     Flight state: PAD_IDLE
     System ready for arm.
     ```

3. **Sensor Status Check:**
   ```
   Command: status_sensors
   Expected: All report HEALTHY
   ```

   Record baseline values:
   - Acceleration: Should read ~[0, 0, 1]G (1G down from gravity)
     - X: ±0.1g (drift acceptable)
     - Y: ±0.1g (drift acceptable)
     - Z: 0.95-1.05g (should be ~1G)
   - Barometric pressure: [___hPa]
   - Temperature: [___°C]
   - GPS status: [# satellites, fix type]

4. **Verify No Error States:**
   ```
   Command: status_all (or equivalent)
   Check for any ERROR, WARNING, or CRITICAL flags
   Should see: All systems HEALTHY
   ```

5. **LED Verification:**
   - If visible, NeoPixel should be green (PAD_IDLE)
   - Should be steady (not blinking)

6. **Arm Flight Computer:**
   ```
   Command: arm
   Expected response: "Flight computer ARMED"
   LED should change to yellow (ARMED state)
   ```

7. **Post-Arm Verification:**
   - System should remain in ARMED state
   - No automatic trigger (should wait for liftoff acceleration)
   - Monitor for 30 seconds to confirm stability

8. **Final Equipment Check:**
   - Visual inspection: No loose wires or connectors
   - Verify parachutes are secured
   - Pyro charges in place (if on pad)
   - SD card mounted
   - USB cable (if used) not obstructing recovery
   - Battery voltage still adequate (check if possible)

**Success Criteria:**
- [ ] All sensors report HEALTHY
- [ ] Acceleration baseline stable (±0.2g variation)
- [ ] Barometer pressure stable
- [ ] GPS acquiring satellites (if outdoor location)
- [ ] No error messages
- [ ] LED indicates ARMED state
- [ ] Flight computer remains in ARMED state without triggering
- [ ] No watchdog resets or anomalies in last 2 minutes

**If Any Test Fails:**
- Immediately DISARM via command: `disarm`
- Do NOT proceed to launch
- Troubleshoot issue and repeat PAD IDLE TEST
- Document problem for post-flight analysis

---

### PHASE 2: LAUNCH & ASCENT
**Timing:** 0-30 seconds nominal (typical motor burn 5-10s + coast 5-20s)
**Objective:** Detect liftoff correctly, monitor boost phase, verify coast state transition

**What to Monitor (from ground telemetry if available):**

1. **Liftoff Detection (0-1s after motor ignition):**
   - Acceleration magnitude jumps above 2.0g threshold
   - Serial output (if connected): "ARMED → BOOST"
   - LED may change to indicate BOOST state
   - Flight computer log should timestamp this event

2. **Boost Phase (0-10s typical):**
   - Continuous altitude increase (barometer reading goes up)
   - Acceleration remains >0.5g (motor still firing)
   - Velocity calculated from barometer becomes positive (going up)
   - If guidance enabled: Servo outputs should activate

3. **Motor Burnout Detection (5-15s depending on motor):**
   - Acceleration drops below 0.5g threshold
   - Velocity from barometer may plateau briefly
   - Flight state transitions: "BOOST → COAST"
   - System enters coast phase (unpowered flight)

**Expected Behavior During Phase 2:**

- Vehicle accelerates rapidly after ignition
- Altitude increases smoothly and continuously
- No sensor dropouts or errors
- Servo commands (if enabled) engage smoothly
- No excessive vibration or instability visible

**What Can Go Wrong:**

| Problem | Symptom | Action |
|---------|---------|--------|
| Liftoff not detected | Stays in ARMED state | Check acceleration threshold; may need recalibration |
| Motor burnout not detected | Stays in BOOST state | Check coast threshold; verify accelerometer working |
| Sensor dropout | Missing data or errors | Check I2C bus; may be hardware issue |
| Guidance instability | Servo chattering visibly | Check PID gains; may be too aggressive |

---

### PHASE 3: APOGEE DETECTION
**Timing:** Boost+5 to Boost+20 seconds (depends on motor and rocket)
**Objective:** Verify multi-method apogee detection, trigger drogue deployment, observe apogee vote counting

**What Triggers Apogee Detection:**

The system uses 4 independent methods, any 2 can vote for apogee:

1. **Barometric Method (Primary):**
   - Monitors altitude from pressure sensor
   - Looks for velocity crossing zero (altitude stops increasing)
   - Requires 3 consecutive negative velocity readings
   - Most reliable for typical altitudes (100-5000 feet)

2. **Accelerometer Method (Secondary):**
   - Monitors vertical acceleration from ICM-20948 or KX134
   - At apogee, acceleration becomes negative (starting to fall)
   - Requires 5 consecutive negative acceleration readings
   - Fast response time (~500ms)

3. **GPS Method (Tertiary):**
   - Uses GPS altitude (if available with 3D fix)
   - Slower due to GPS update rate (~1Hz typical)
   - More conservative threshold (-2.0 m/s vs -0.5 m/s for baro)
   - Validates barometric reading

4. **Timeout Failsafe:**
   - Hardcoded 20-second timeout after motor burnout
   - Ensures deployment even if sensors fail
   - Last resort: If 2 primary methods fail

**Expected Sequence:**

```
Time 0s: Motor burnout detected (accel <0.5g)
         Flight state: BOOST → COAST

Time 2-5s: Barometric velocity becomes negative
           Accel velocity becomes negative
           GPS (if available) shows altitude decrease

Time 6-10s: Voting accumulates:
            Vote 1: Barometer (altitude decreasing)
            Vote 2: Accelerometer (vertical accel negative)
            Total: 2 votes → APOGEE DETECTED

            Flight state: COAST → APOGEE
            Pyro channel 1 fires (drogue charge)
            Acceleration spike (parachute opening shock)

Time 11-15s: Drogue parachute deployed
             Acceleration stabilizes as parachute slows descent
             Flight state: APOGEE → DROGUE_DEPLOY → DROGUE_DESCENT
```

**Monitoring What Actually Happens:**

1. **From Serial Output (if connected):**
   - Watch for "COAST" message at motor burnout
   - Look for apogee detection vote printout (if debug enabled)
   - Watch for "APOGEE" message when apogee detected
   - May see which methods voted: "Baro + Accel" or similar

2. **From Data Log (post-flight analysis):**
   - Plot altitude vs time:
     - Should rise steadily during BOOST phase
     - Peak during COAST phase
     - Descend from that peak onwards
   - Plot acceleration vs time:
     - High positive during BOOST
     - Negative during COAST (free fall)
     - Large negative spike when drogue deploys
   - Plot velocity (derived):
     - Positive during ascent
     - Crosses zero at apogee
     - Becomes negative during descent

3. **From Flight Observation:**
   - Watch for brief bright flash (motor burnout)
   - Watch for continued ascent under momentum
   - Watch for separation/charge firing (may hear pop)
   - Watch for drogue parachute deployment
   - Watch for descent under drogue (should be controlled)

**Expected Results:**
- [ ] Motor burnout detected within 1s of actual burnout
- [ ] Apogee detected between 5-20s after motor burnout
- [ ] At least 2 detection methods voted
- [ ] Apogee height matches pre-flight prediction (±10% acceptable)
- [ ] Drogue deploys immediately after apogee detection
- [ ] Parachute opening visible/audible
- [ ] Descent rate under drogue appears stable (~40-60 ft/s typical)

**If Apogee Detection Fails:**

- Backup timeout (20s after burnout) will trigger deployment
- If even timeout fails to deploy drogue:
  - Check pyro continuity post-flight
  - Verify charge was actually installed
  - Check flight log for any error messages
  - May need to re-pack parachutes and re-test

---

### PHASE 4: DESCENT (Apogee to Main Deploy)
**Timing:** Apogee+0s to Main Deploy (duration 30-120s depending on apogee height)
**Objective:** Monitor controlled descent under drogue, verify main deployment trigger approaching

**Expected Behavior:**

1. **Under Drogue (0-30s post-apogee):**
   - Altitude decreases steadily
   - Descent rate stabilizes (typical 40-60 ft/s)
   - Temperature stays stable
   - No sensor errors
   - Flight state: DROGUE_DESCENT

2. **Descent Rate Calculation:**
   - Take two altitude measurements from barometer
   - Calculate: Rate = ΔAltitude / ΔTime
   - Should be constant (±5 ft/s variation acceptable)
   - Too fast (>80 ft/s): Drogue may not be deployed
   - Too slow (<20 ft/s): Drogue too large or entanglement

3. **Approaching Main Deploy Altitude:**
   - Monitor altitude approaching main deploy threshold (default 100m AGL)
   - System monitors continuously and prepares for deployment
   - If guidance enabled: Servos may make control adjustments

**What to Monitor:**

1. **From Serial (if connected):**
   - Flight state: DROGUE_DESCENT (should remain)
   - Altitude decreasing monotonically
   - No error or warning messages
   - Pressure readings sensible (decreasing with altitude)

2. **From Flight Observation:**
   - Rocket descending at steady rate
   - Drogue parachute open and stable
   - No tumbling or spinning
   - Appears to be maintaining level orientation

3. **Sensor Health:**
   - Barometer altitude decreasing proportionally with time
   - Calculated velocity negative (downward)
   - No GPS glitches (altitude shouldn't jump around)
   - Accelerometer reading ~1G downward (falling under parachute)

**Expected Results:**
- [ ] Drogue descent rate 40-80 ft/s
- [ ] Altitude decreases smoothly without jumps
- [ ] No sensor errors or dropouts
- [ ] Continues descending toward main deploy altitude
- [ ] System remains in DROGUE_DESCENT state
- [ ] No premature main deployment
- [ ] Acceleration remains ~1G (falling under drogue weight)

**If Descent Appears Unstable:**

- Too fast descent (>100 ft/s):
  - Drogue may have failed to deploy
  - Or drogue too small
  - Post-flight: Check parachute integrity and size

- Tumbling/spinning:
  - Vehicle may be unstable
  - Post-flight: Check CG and fin alignment
  - May need to add weight to nose or adjust fins

- Sensor errors:
  - Check barometer readings post-flight
  - May need recalibration

---

### PHASE 5: MAIN DEPLOYMENT
**Timing:** AGL <100m (configurable, default 100m)
**Objective:** Deploy main parachute, verify altitude-based trigger working, safe descent rate achieved

**Main Deploy Trigger Logic:**

System monitors current altitude continuously:
```
If (altitude_agl < MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M):
   Deploy main parachute
   Fire pyro channel 2
   Transition state: DROGUE_DESCENT → MAIN_DEPLOY → MAIN_DESCENT
```

**Expected Sequence:**

```
Time -2s (relative to deploy): Altitude approaching 100m
          Flight state still DROGUE_DESCENT
          Descent continuing under drogue

Time 0s: Altitude exactly at 100m AGL
        Deployment trigger activates
        Flight state: DROGUE_DESCENT → MAIN_DEPLOY
        Pyro channel 2 fires (main charge)
        Large acceleration spike (main opening shock)

Time +1s: Main parachute fully deployed
         Acceleration drops sharply (sudden deceleration)
         Descent rate drops significantly
         Flight state: MAIN_DEPLOY → MAIN_DESCENT

Time +2-5s: Main descent stabilizes
           Descent rate now 15-25 ft/s
           Steady controlled descent
           Altitude continues decreasing to ground
```

**Monitoring Main Deployment:**

1. **From Flight Observation:**
   - Watch for altitude trigger (should be around 100m)
   - Second charge deployment (bright flash or loud pop)
   - Main parachute deploys and fully opens
   - Dramatic decrease in descent rate
   - Should see canopy fully inflated before landing

2. **From Serial Output (if connected):**
   - Flight state: "MAIN_DEPLOY" message
   - May print: "Main deployment at [altitude] m AGL"
   - Followed by: "MAIN_DESCENT" state
   - No error messages

3. **From Data Log (post-flight):**
   - Acceleration plot: Large negative spike at deploy point
   - Velocity plot: Sudden shift toward zero (rapid deceleration)
   - Altitude plot: Clear marker where main deploys (often visible as sudden curve change)

**Descent Rate After Main Deploy:**

Calculate from altitude data:
```
Descent rate = ΔAltitude / ΔTime

Expected: 15-30 ft/s
Too fast (>50 ft/s): Main may not be fully deployed
Too slow (<10 ft/s): Main may be too large
```

**Expected Results:**
- [ ] Main deploys at correct altitude (within ±10m of target)
- [ ] Deployment timing triggers on descending altitude (not ascending)
- [ ] Descent rate drops to safe value after main opens
- [ ] No premature main deployment during ascent
- [ ] Parachute canopy appears intact
- [ ] Vehicle stable and under control during descent
- [ ] Continues descending steadily toward ground

**If Main Deployment Goes Wrong:**

| Symptom | Likely Cause | Recovery |
|---------|--------------|----------|
| Main deploys too high | Altitude calibration off | Check barometer post-flight |
| Main deploys too late | Barometer failure | Backup timeout may deploy |
| Main won't deploy | Pyro charge failed | May need manual deployment |
| Descent too fast after main | Main tangled or small | Verify parachute packed correctly |
| Descent too slow | Main too large | Design change for next flight |
| Vehicle unstable | CG or fin issue | Check vehicle design post-flight |

---

### PHASE 6: LANDING DETECTION
**Timing:** Descent to ground (final 30-60 seconds)
**Objective:** Detect landing event, transition to recovery mode, activate beacon

**Landing Detection Logic:**

System monitors three criteria:
1. **Acceleration near 1G** (0.9-1.1G) - No longer accelerating
2. **Altitude stable** (changes <1m between readings) - Height not changing
3. **Velocity near zero** (calculated from altitude) - Not moving significantly

When all three conditions stable for 2+ seconds:
```
Flight state: MAIN_DESCENT → LANDED
System waits 10 seconds in LANDED state
Then: LANDED → RECOVERY
Recovery beacon activates
```

**Expected Sequence:**

```
Time -10s (landing): Altitude 50m, descent rate steady
           Acceleration ~1G downward

Time -5s: Altitude 25m, still descending smoothly
         No significant acceleration change

Time 0s (touchdown): Rocket touches ground
        Acceleration briefly spikes then settles to ~1G
        Altitude stops decreasing
        All three landing conditions met

Time +2s: All conditions held stable for 2 seconds
         Flight state: MAIN_DESCENT → LANDED

Time +10s: Timeout elapsed
          Flight state: LANDED → RECOVERY
          Recovery beacon activates:
            - LED strobe pattern (white flash every 1 second)
            - Audio SOS beacon (·-·/---/···)
            - GPS beacon (coordinates every 10 seconds, if enabled)
```

**Monitoring Landing Detection:**

1. **From Flight Observation:**
   - Watch vehicle approach ground
   - Should touch down gently (controlled descent)
   - Momentary movement on impact (normal)
   - Then becomes stationary
   - Wait 10 seconds for beacon to activate

2. **From Serial Output (if connected):**
   - Watch altitude approach zero
   - Watch acceleration approach 1G
   - Watch for "LANDED" message
   - Wait 10 seconds for "RECOVERY" message
   - Listen for SOS beacon to start

3. **From Data Log (post-flight):**
   - Altitude plot: Should end near 0m AGL
   - Acceleration plot: Should settle to +1G at end
   - Velocity plot: Should approach zero at landing

**Recovery Beacon Activation:**

After LANDED state timeout (10s), system enters RECOVERY:

- **LED Strobe:**
  - NeoPixel flashes white
  - Pattern: 100ms on, 900ms off (1 second cycle)
  - Visible from 100+ feet away in daylight
  - More visible at night

- **Audio Beacon:**
  - Piezo buzzer emits SOS Morse code
  - ·-· (S) = dot-dash-dot (200ms dot, 600ms dash)
  - --- (O) = dash-dash-dash
  - ·-· (S) = dot-dash-dot
  - Repeats every ~2.5 seconds
  - Audible from 50+ feet away

- **GPS Beacon (if enabled):**
  - Prints current GPS coordinates to serial
  - Every 10 seconds (RECOVERY_GPS_BEACON_INTERVAL_MS)
  - Format: `GPS: Lat [degrees.decimals], Lon [degrees.decimals]`
  - Can be used for search if GPS available

**Expected Results:**
- [ ] Landing detected within 15 seconds of touchdown
- [ ] Vehicle stable at end of flight log
- [ ] Altitude ends near 0m AGL
- [ ] Acceleration near 1G at landing
- [ ] LED strobe activates after 10s
- [ ] Audio beacon heard from recovery team distance
- [ ] GPS coordinates transmitted (if applicable)
- [ ] System remains in RECOVERY state until power removed

**If Landing Detection Fails:**

- Recovery beacon may not activate automatically
- Post-flight: Check landing detection thresholds
  - May need to adjust LANDING_ACCEL_MIN_G / MAX_G
  - May need to adjust altitude stable threshold
- Manual recovery beacon may be available via command
- Can search for vehicle using GPS data from flight log

---

### PHASE 7: RECOVERY
**Timing:** Ground to shutdown (0-5 minutes in RECOVERY state)
**Objective:** Locate and recover vehicle safely, retrieve data

**Recovery Procedure:**

1. **Initial Approach (First minute):**
   - Wait at least 1 minute after touchdown before approaching
   - Allow pyro charges to cool if recently fired
   - Check for any fires or hazards
   - Listen for SOS beacon

2. **Locating the Vehicle:**
   - Follow LED strobe flash (visible as bright white flash every 1 second)
   - Listen for audio beacon (distinctive SOS Morse code pattern)
   - Use GPS coordinates if beacon transmitted them
   - Scan area visually (may be in tall grass, behind trees, etc.)

3. **Approach and Inspection:**
   - Approach vehicle carefully
   - Check around rocket for any hazards (spilled charges, hot spots)
   - Visually inspect for damage:
     - Parachute intact and no tears
     - Airframe not cracked or deformed
     - Nose cone properly attached
     - All fins intact
   - Check battery voltage if accessible (should be ~10V for 3S LiPo)

4. **Power-Off:**
   - Locate power switch or battery connector
   - Disconnect battery cleanly (prevent accidental re-firing)
   - System will power down
   - LED strobe and audio beacon stop

5. **Data Retrieval:**
   - Re-connect battery to power flight computer
   - Open serial terminal (115200 baud)
   - Command: `status_sd` to verify log file exists
   - Carefully remove SD card
   - Transfer to computer for analysis

6. **Environmental Documentation:**
   - Record time of recovery
   - Record GPS position if available (for range documentation)
   - Note any damage or anomalies
   - Record weather conditions at recovery time
   - Note distance and direction from launch point

**Expected Results:**
- [ ] Vehicle located using beacon
- [ ] No fires or hazards present
- [ ] Vehicle intact with no major damage
- [ ] Battery still has adequate charge
- [ ] SD card retrieves successfully
- [ ] Log file contains complete flight data
- [ ] GPS data useful for future searches
- [ ] All personnel safe

**Troubleshooting Recovery Issues:**

| Problem | Solution |
|---------|----------|
| Can't hear beacon | Check battery (may be low); walk grid search pattern |
| Can't see LED strobe | May be covered or inside rocket; listen for audio beacon |
| Vehicle damaged | Assess damage; may need to repair before next flight |
| SD card won't read | Verify card mount; try on different computer; may need recovery |
| Battery completely dead | System entered deep sleep; may recover with charging |
| Pyro charge still hot | Wait additional 5 minutes before handling |

---

## POST-FLIGHT TEST PHASE

### IMMEDIATE POST-FLIGHT (On the field, first 10 minutes)

#### 1. Safety Phase

1. **Wait for Cooling:**
   - Allow 1+ minute for pyro charges to cool
   - Designate exclusion zone around vehicle
   - Verify no fires or ongoing hazards

2. **Vehicle Security:**
   - Secure vehicle location (prevent accidental movement)
   - If unstable on ground, support with padding
   - Prevent unauthorized access until inspection complete

3. **Personnel Debrief:**
   - Gather all participants
   - Quick verbal debrief on flight observations:
     - "What did we see during boost?"
     - "When did motor burnout happen?"
     - "Did parachutes deploy at expected points?"
     - "Any anomalies observed?"
   - Document observations (time, altitude estimate, etc.)

#### 2. Vehicle Inspection

1. **Structural Integrity:**
   - Visually inspect all exterior surfaces
   - Check for cracks, splits, or dents
   - Verify nose cone attachment (pull test - should be secure)
   - Check fin integrity (visual + gentle flex test)
   - Verify all body tube joints are intact

2. **Parachute Condition:**
   - Inspect drogue parachute:
     - Look for tears or holes
     - Check bridle for tangles or damage
     - Verify fabric is not scorched
   - Inspect main parachute:
     - Look for tears or holes
     - Check bridle integrity
     - Verify suspension lines not broken
   - If damage noted: Document with photos

3. **Sensors and Flight Computer:**
   - Visual inspection of flight computer:
     - No water damage or corrosion
     - All connectors secure
     - No visible burn marks
   - Check that accelerometers and sensors are still mounted
   - Verify SD card still in place
   - Check antenna connection (if GPS external)

4. **Battery Status:**
   - Measure battery voltage with multimeter
   - Should be lower than pre-flight but >10.5V for 3S LiPo
   - If significantly lower than expected: Battery may have excessive drain
   - If over 11V: System may not have used expected power

#### 3. Data Retrieval

1. **SD Card Extraction:**
   - Power on flight computer if not already powered
   - Wait 5 seconds for SD card to mount
   - Verify LED status (should show RECOVERY state if post-flight)
   - Carefully remove SD card from slot:
     - Push inward until click (should partially eject)
     - Pull out rest of way
     - Handle by edges (avoid touching contacts)

2. **Initial Log File Verification:**
   - If USB still available on flight computer:
     - Command: `status_sd` - should show log file name
     - Can optionally stream log data via serial if needed
   - If no USB available:
     - Proceed to lab analysis

3. **Quick Serial Status Check:**
   - If USB available, connect serial terminal
   - Command: `status_all` or `status_sensors`
   - Verify all sensors still report HEALTHY
   - No corruption or errors present
   - Flight computer remains responsive

**Expected Results:**
- [ ] Vehicle structure intact
- [ ] All exterior undamaged
- [ ] Parachutes fully deployed and functional
- [ ] Battery voltage reasonable (not completely drained)
- [ ] SD card retrieves cleanly
- [ ] Flight computer still responsive
- [ ] No obvious errors or corruption

---

### LAB ANALYSIS (Within 24 hours of flight)

#### 1. Data Processing

1. **SD Card Recovery:**
   - Insert SD card into computer
   - Navigate to `/logs/` directory
   - Locate flight log file (e.g., `flight_20250215_143022.csv`)
   - Copy to analysis directory on computer

2. **CSV File Inspection:**
   - Open file in text editor (ensure UTF-8 encoding)
   - Verify header row has 62 columns:
     ```
     seqNum,timestamp,flightState,fixType,sats,latitude,longitude,...
     ```
   - Count data rows (should correspond to 10-30 minutes of flight)
   - Check for any blank rows or truncation

3. **Data Validation:**
   - Open in spreadsheet (Excel, LibreOffice, Python, etc.)
   - Sort by timestamp - should increase monotonically
   - Check for duplicate timestamps (indicates logging error)
   - Check for missing data points (should be continuous or nearly so)
   - Verify altitude data:
     - Should start near 0 (ground level)
     - Increases during boost/coast
     - Peaks during coast
     - Decreases during descent
     - Returns to near 0 at landing
   - Verify acceleration data:
     - Starts near 1G (gravity)
     - Spikes >2G during boost
     - Becomes negative during coast
     - Returns to ~1G during descent

4. **Error Detection:**
   - Look for NaN (Not a Number) values
   - Look for extreme outliers (e.g., pressure 5000 hPa)
   - Look for discontinuous jumps in altitude
   - Check GPS data for sanity (latitude/longitude within known range)
   - If errors found: Document and investigate cause

**Expected Results:**
- [ ] CSV file opens without corruption
- [ ] Header row complete with 62 columns
- [ ] Data rows match expected flight duration
- [ ] Timestamps monotonically increasing
- [ ] No significant gaps or missing data
- [ ] Altitude and acceleration data appear sensible
- [ ] No NaN or corrupted values
- [ ] File successfully parsed by analysis tools

#### 2. Performance Analysis

1. **Altitude Profile:**

   **Plot:**
   ```
   Generate plot: Altitude (m AGL) vs Time (seconds)
   ```

   **Analysis:**
   - Pre-flight baseline: Should be 0m ± calibration error
   - Boost phase: Rapid increase (>20 m/s typical)
   - Coast phase: Continued increase, rate decreasing
   - Peak altitude: Record this value
   - Descent phase: Smooth decrease under parachute
   - Landing: Should return to ~0m

   **Comparison:**
   - Compare actual peak altitude to prediction:
     - If using OpenRocket/RockSim: Expected [__] m
     - Actual: [__] m
     - Difference: [__] m or [__]%
     - Acceptable if <15% difference

   **Expected Values (example for typical mid-power rocket):**
   - Peak altitude: 800-1200 m
   - Boost phase duration: 5-8 seconds
   - Coast phase duration: 10-15 seconds
   - Drogue descent rate: 40-60 ft/s
   - Main descent rate: 15-25 ft/s

2. **Apogee Detection Analysis:**

   **Extract from Log:**
   - Find timestamp where altitude peaks
   - Look for state transitions:
     - BOOST → COAST (motor burnout)
     - COAST → APOGEE (apogee detection)
   - Calculate delay between peak and detection (should be <2 seconds)

   **Voting Analysis (if logged):**
   - Check which methods voted for apogee:
     - Barometer method: Did altitude reverse?
     - Accelerometer method: Did accel go negative?
     - GPS method: Did GPS altitude decrease?
     - Timeout: How many seconds elapsed?
   - Verify at least 2 methods detected apogee

   **Expected Results:**
   - Apogee detected within 2 seconds of peak altitude
   - At least 2 of 4 methods voted
   - No false apogee detections during boost/coast

3. **Deployment Timing Analysis:**

   **Drogue Deployment:**
   - Find timestamp and altitude at APOGEE state
   - Find timestamp at DROGUE_DEPLOY state
   - Calculate delay: Should be <0.5 seconds
   - Look for acceleration spike in data (parachute opening)
   - Verify descent rate drops after deployment

   **Main Deployment:**
   - Find altitude at MAIN_DEPLOY trigger (should be ~100m AGL)
   - Compare to configured threshold (MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M)
   - If significantly off: Check barometer calibration
   - Calculate delay between trigger and deployment (<1 second)

   **Expected Results:**
   - Drogue deploys immediately after apogee detection
   - Main deploys at configured altitude (±10m acceptable)
   - Clear acceleration spikes at both deployments
   - Descent rates change appropriately

4. **Descent Rate Analysis:**

   **Under Drogue:**
   ```
   From altitude data: Calculate velocity = ΔAltitude / ΔTime
   Expected: 40-60 ft/s (12-18 m/s)
   ```
   - Should be relatively constant throughout drogue phase
   - Plot descent rate over time (should be flat/stable)
   - If varies ±10%: Normal
   - If varies >20%: May indicate parachute instability

   **Under Main:**
   ```
   Calculate velocity after main deployment
   Expected: 15-25 ft/s (4.5-7.5 m/s)
   ```
   - Should be safe for landing
   - If >30 ft/s: Main not fully open or too small
   - If <10 ft/s: Main very large or drag excessive

5. **Sensor Data Quality:**

   **Barometer (MS5611):**
   - Check pressure readings (should be 950-1050 hPa)
   - Verify temperature readings (should be 0-40°C typical)
   - Look for pressure spikes (sensor noise)
   - Compare GPS altitude to barometer:
     - Should agree within ±30 meters
     - Larger difference may indicate calibration issue

   **Accelerometer (ICM-20948 + KX134):**
   - Check X,Y,Z acceleration components
   - At rest: Should be [0, 0, 1]G ± 0.1G
   - During flight: Peaks should match expected motor performance
   - Look for saturation or clipping (max value clamped)

   **GPS:**
   - Check position validity (latitude/longitude sensible)
   - Check satellite count (typically 6-10)
   - Check altitude validity (should match barometer)
   - Look for GPS glitches (sudden position jumps)

**Expected Results:**
- [ ] Altitude profile matches expected trajectory
- [ ] Peak altitude within 15% of prediction
- [ ] Motor burnout detected correctly
- [ ] Apogee detected by 2+ methods
- [ ] Drogue deploys immediately
- [ ] Main deploys at correct altitude
- [ ] Descent rates safe and predictable
- [ ] All sensor data quality acceptable
- [ ] No significant errors or corruption

---

#### 3. Comparison vs Prediction

1. **Pre-Flight Simulation:**

   Retrieve pre-flight prediction documents:
   - OpenRocket or RockSim simulation file
   - Recorded predicted apogee height
   - Predicted motor burnout time
   - Predicted descent rates

2. **Comparison Table:**

   ```
   Parameter            | Predicted | Actual | Error % | Status
   ==================== | ========= | ====== | ======= | ======
   Apogee Height (m)    |    1000   |  950   |   -5%   | PASS
   Motor Burnout (s)    |     7.5   |   7.3  |   -2%   | PASS
   Drogue Descent (f/s) |     50    |   48   |   -4%   | PASS
   Main Descent (f/s)   |     18    |   19   |   +6%   | PASS
   Total Flight Time(s) |     45    |   43   |   -4%   | PASS
   ```

3. **Anomaly Analysis:**

   - Any parameter >15% off prediction: Investigate
   - Possible causes:
     - Air density different than expected (weather/altitude)
     - Motor performance varied from spec
     - Vehicle weight different than predicted
     - Drag coefficient different (parachute condition)

**Expected Results:**
- [ ] Apogee within ±15% of prediction
- [ ] Motor burnout time within ±20% of prediction
- [ ] Descent rates within ±20% of prediction
- [ ] Flight duration reasonable
- [ ] All parameters pass go/no-go criteria

---

#### 4. Success Criteria - Post-Flight

Flight test declared SUCCESSFUL if:

**MANDATORY (must all pass):**
- [ ] All log entries recorded without gaps
- [ ] Apogee detected correctly (2+ methods voted)
- [ ] Drogue deployment at apogee
- [ ] Main deployment at correct altitude (±10m)
- [ ] Landing detected and recovery beacon activated
- [ ] No sensor failures or errors during flight
- [ ] Vehicle recovered safely with no major damage

**STRONGLY DESIRED (should pass):**
- [ ] Apogee height within 15% of prediction
- [ ] Descent rates match prediction ±20%
- [ ] All sensor data quality excellent
- [ ] CSV data complete and uncorrupted
- [ ] Flight time near prediction

**ACCEPTABLE VARIATIONS:**
- Motor burnout time varies ±20% (motor performance variation)
- GPS altitude ±30m different from barometer (normal GPS error)
- Landing detection delay up to 15 seconds (system tuning)
- Descent rate variation ±10% (parachute dynamics)

---

#### 5. Anomaly Investigation

If any parameter fails or shows unexpected behavior:

**Apogee Detection Missed:**

1. Check flight log for:
   - Did motor burnout occur? (Acceleration drop)
   - Did altitude stop increasing? (Barometer method?)
   - Did acceleration become negative? (Accel method?)
   - Did GPS show descent? (GPS method?)

2. Root cause analysis:
   - If no methods detected: All sensors may have failed
   - If 1 method detected: Not enough votes (need 2)
   - If timeout triggered: Primary methods all failed, but failsafe worked

3. Investigation steps:
   - Verify barometer was calibrated correctly
   - Check accelerometer scale/range in config
   - Verify GPS had 3D fix during coast
   - Review sensor health log

**Deployment Too Early or Late:**

1. Check actual deployment altitude vs configured:
   - If too early: Barometer calibration may be off (showing false altitude)
   - If too late: Same issue or altitude threshold too high

2. Barometer recalibration:
   - Repeat `calibrate_baro` at launch location
   - Ensure full 5-minute thermal stabilization
   - Compare to known baseline pressure

3. Future adjustments:
   - If consistently 20m too high: Reduce `MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M` by 20m
   - If consistently 20m too low: Increase threshold by 20m

**Servo Not Responding:**

1. From flight log:
   - Check servo command values in CSV
   - If all zeros: Guidance may be disabled
   - If all 90: Guidance active but no actual movement needed

2. Check physical servo:
   - Power servo directly from bench power supply
   - Apply PWM signal manually (using oscilloscope or Arduino)
   - If servo still doesn't move: Likely stuck or failed

3. Recalibration:
   - Adjust pulse width limits in config.h
   - Re-test on bench before next flight

**Data Corruption or Loss:**

1. If SD card file truncated:
   - May indicate power loss during flight
   - Check battery voltage in log before end
   - Verify battery not draining excessively

2. If individual fields corrupted (NaN values):
   - Specific sensor likely failed
   - Check sensor health status in log
   - Verify I2C bus voltage and termination

3. Prevention for next flight:
   - Use different SD card if current has bad sectors
   - Verify power supply adequate
   - Add decoupling capacitors if noise present

---

## TROUBLESHOOTING COMMON ISSUES

### Problem: Liftoff Not Detected

**Symptoms:** Flight computer stays in ARMED state during and after actual launch

**Root Causes:**
1. Acceleration threshold too high (default 2.0g is conservative)
2. Accelerometer not working (not reporting data)
3. Accelerometer calibration offset
4. Vehicle not actually reaching 2.0g acceleration

**Diagnostic Steps:**

1. **Check threshold setting:**
   ```c
   // In config.h
   #define BOOST_ACCEL_THRESHOLD 2.0f  // Try reducing to 1.5f
   ```

2. **Test accelerometer:**
   ```
   Command: status_sensors
   Check: ICM-20948 reports HEALTHY and shows acceleration
   ```

3. **Bench test:**
   - Hold flight computer and shake rapidly
   - Should see acceleration values increase
   - Command: `calibrate_accel` if offset present

4. **Flight simulation (no rocket):**
   - Hand-shake flight computer to simulate launch
   - Verify acceleration threshold triggers
   - Check serial output for state change

5. **Vehicle analysis:**
   - Re-run OpenRocket simulation for motor
   - Verify predicted peak acceleration >2.0g
   - If less: Use motor with higher acceleration

**Solution:**
- If threshold too high: Reduce BOOST_ACCEL_THRESHOLD
- If accelerometer not responding: Check I2C bus
- If motor underpowered: Select higher-impulse motor

**Prevention:**
- For first flight, use conservative (low) threshold
- Test liftoff detection on bench before flight
- Review motor specs: Ensure F_peak > 2.0g at vehicle mass

---

### Problem: Apogee Detection Misses

**Symptoms:** Drogue parachute never deploys, or deploys very late

**Root Causes:**
1. None of the 4 apogee detection methods working
2. Motor burn very long (>20s) - timeout is backup failsafe
3. Barometer not responding or calibration wrong
4. Accelerometer not detecting descent
5. Vehicle trajectory unusual (very slow coast phase)

**Diagnostic Steps:**

1. **Check flight log for detection votes:**
   - Look at altitude near peak
   - Calculate velocity: ΔAltitude / ΔTime
   - If velocity becomes negative: Barometer should detect apogee
   - If acceleration data shows negative: Accel should detect

2. **Individual method troubleshooting:**

   **Barometric Method:**
   ```
   Extract altitude data points near peak
   Calculate velocity = (Alt_t2 - Alt_t1) / (Time_t2 - Time_t1)
   If velocity consistently negative but not detected:
     - Sensor may be noisy (need smoothing)
     - Or threshold is too strict (change APOGEE_BARO_DESCENT_THRESHOLD)
   ```

   **Accelerometer Method:**
   ```
   Check Z-axis acceleration during coast
   Should become negative as rocket starts falling
   If stays positive: Sensor may be miscalibrated
   Try: calibrate_accel on bench
   ```

   **GPS Method:**
   ```
   Check if GPS had 3D fix during flight
   From log: Look for fixType and sats fields
   If no 3D fix: GPS method won't vote
   That's OK - still have 2 other methods
   ```

   **Timeout Method:**
   ```
   If 20-second timeout triggered: All 3 primary methods failed
   This is the failsafe working correctly (not ideal, but safe)
   ```

3. **Barometer calibration verification:**
   - Redo `calibrate_baro` at exact launch location
   - Check if pressure stable during flight
   - Look for pressure spikes (indicates sensor issue)

4. **Configuration check:**
   ```c
   // Verify these in config.h
   #define APOGEE_BARO_DESCENT_THRESHOLD 1.0   // Should be 1.0 or lower
   #define APOGEE_ACCEL_THRESHOLD -0.1        // Should be around -0.1
   #define BACKUP_APOGEE_TIME_MS 20000        // 20s timeout
   ```

**Solutions:**

1. **If barometer not working:**
   - Check MS5611 I2C communication
   - Re-calibrate: `calibrate_baro`
   - Verify pressure readings in `status_sensors`

2. **If accelerometer not working:**
   - Check ICM-20948 or KX134 I2C
   - Re-calibrate: `calibrate_accel`
   - Verify accel readings in `status_sensors`

3. **If thresholds too strict:**
   - Reduce `APOGEE_BARO_DESCENT_THRESHOLD` to 0.5
   - Reduce `APOGEE_ACCEL_SAMPLES` to 3 (from 5)
   - Reduce `APOGEE_ACCEL_CONFIRMATION_COUNT` to 3 (from 5)

4. **If motor burn is very long:**
   - 20-second timeout will eventually trigger
   - Or deploy at main altitude if coast extends that long
   - This is acceptable but not ideal
   - Try different motor for next flight

**Prevention:**
- Test apogee detection on bench (apply acceleration, verify state change)
- For first flights, use conservative thresholds
- Verify all three primary sensors working during pad test
- Simulate flight profile: Boost → Coast → Apogee in software

---

### Problem: Deployment Too Early or Late

**Symptoms:** Main parachute deploys at wrong altitude (not 100m AGL)

**Root Causes:**
1. Barometer calibration offset (most common)
2. Actual launch location altitude different from calibration
3. Altitude threshold configured incorrectly
4. GPS altitude being used instead of barometer (different reference)

**Diagnostic Steps:**

1. **Compare deployment altitude:**
   ```
   Configured: MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M = 100m
   Actual from log: [__ m AGL]
   Difference: [__ m] or [__ %]
   ```

2. **Check barometer calibration:**
   - At launch location, what is actual MSL pressure?
   - Compare to what flight computer shows during `status_sensors`
   - If very different: Calibration is off

3. **Verify calibration location:**
   - Was `calibrate_baro` done at launch location?
   - Or at different location with different altitude?
   - If at different altitude: Will affect AGL calculation

4. **Check GPS altitude:**
   - Does GPS show different altitude than barometer?
   - GPS can be ±30m off barometer
   - If large difference: May explain some deployment variance

**Solutions:**

1. **Fix barometer calibration:**
   - Move to exact launch location
   - Allow 5+ minutes for thermal stability
   - Command: `calibrate_baro`
   - Re-test altitude readings

2. **Adjust deployment threshold if needed:**
   ```c
   // If deployment consistently too early by 20m:
   #define MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M 120.0f  // Increase from 100m

   // If consistently too late by 20m:
   #define MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M 80.0f   // Decrease from 100m
   ```

3. **Account for wind:**
   - Barometer readings affected by wind pressure
   - High wind can show lower altitude (higher pressure)
   - Low wind shows higher altitude (lower pressure)
   - If very windy: Expect ±10% altitude variance

4. **GPS-based deployment (if available):**
   - If GPS available, can use GPS altitude as backup
   - GPS altitude may be more stable in windy conditions
   - Less accurate (±30m) but independent of weather

**Prevention:**
- Always calibrate barometer at launch location
- Allow 10+ minutes thermal stabilization
- Document deployment altitude for each flight
- If trend appears: Make small incremental adjustments

---

### Problem: Servo Not Responding

**Symptoms:** Servo doesn't move during flight, or moves erratically

**Root Causes:**
1. ENABLE_GUIDANCE set to 0 (disabled)
2. Servo pin misconfigured
3. Servo power supply inadequate
4. Servo stuck or mechanically jammed
5. Servo not receiving PWM signal

**Diagnostic Steps:**

1. **Check if guidance enabled:**
   ```c
   // In config.h
   #define ENABLE_GUIDANCE 1  // Should be 1 (0 = disabled)
   ```

2. **Bench test servo directly:**
   - Disconnect servo from Teensy
   - Connect directly to power supply (5V typical)
   - Use oscilloscope or PWM generator to apply test signal
   - Verify servo responds to 1000-2000μs pulse width range
   - If doesn't respond: Servo may be failed

3. **Test servo command via serial:**
   ```
   Command: test_servo 21 90    // Command pitch servo to 90°
   Expected: Servo moves to neutral
   If no movement: Check pins or power
   ```

4. **Check servo power supply:**
   - Measure voltage across servo power connector
   - During idle: Should be 5V nominal
   - During servo movement: Should stay >4.5V
   - If drops <4V: Power supply inadequate

5. **Verify pin configuration:**
   ```c
   #define ACTUATOR_PITCH_PIN 21  // Check these are correct for your board revision
   #define ACTUATOR_ROLL_PIN  23
   #define ACTUATOR_YAW_PIN   20
   ```

6. **Mechanical inspection:**
   - Remove servo and test on bench
   - Check for debris inside servo
   - Verify servo horn (arm) not bent
   - Listen for grinding noises (internal jam)

**Solutions:**

1. **If guidance disabled:**
   ```c
   #define ENABLE_GUIDANCE 1  // Change from 0 to 1
   Recompile and upload firmware
   ```

2. **If servo stuck:**
   - Check for mechanical jam
   - Try servo center position command: `test_servo 21 90`
   - If still won't move: Servo may need replacement

3. **If power inadequate:**
   - Verify power supply rated for servo current (1-2A peak)
   - Add larger capacitor (470μF) across power
   - Consider separate power supply for servos

4. **If pin wrong:**
   - Verify pin numbers match your Teensy revision
   - Some Teensy revisions have different pin layouts
   - Check board documentation

**Prevention:**
- Test servo operation on bench before first flight
- Use `test_servo` command at pad to verify response
- Monitor servo power supply voltage during test
- Document working servo specifications for future reference

---

### Problem: Recovery Beacon Not Activated

**Symptoms:** LED doesn't strobe and/or audio beacon doesn't sound after landing

**Root Causes:**
1. Landing not detected (see "Problem: Apogee Detection Misses")
2. LED power supply failed
3. Buzzer power supply failed
4. Landing detection threshold too strict
5. System still in LANDED state (hasn't reached RECOVERY yet)

**Diagnostic Steps:**

1. **Check if landing detected:**
   ```
   Look at flight log near end:
   Last altitude: Should be near 0m
   Last acceleration: Should be near 1G
   Final flight state: Should transition MAIN_DESCENT → LANDED
   ```

2. **Check timing:**
   - System transitions LANDED → RECOVERY after 10 seconds
   - If you're checking within 10 seconds of landing: Beacon won't be active yet
   - Wait 15 seconds after landing before looking for beacon

3. **LED power test:**
   - Command: `test_led` (if available)
   - Or observe if LED was active during startup (should have shown colors)
   - If never showed colors: LED power may be off

4. **Buzzer power test:**
   - Command: `test_buzzer` or `test_beacon`
   - Should hear SOS beep pattern
   - If silent: Buzzer may not have power

5. **Landing detection threshold check:**
   ```c
   // If landing never detected, check thresholds:
   #define LANDING_ACCEL_MIN_G 0.9f    // Min acceleration at landing
   #define LANDING_ACCEL_MAX_G 1.1f    // Max acceleration
   #define LANDING_CONFIRMATION_TIME_MS 2000 // Time to confirm (2 seconds)
   #define LANDING_ALTITUDE_STABLE_THRESHOLD 1.0 // Altitude change tolerance
   ```

**Solutions:**

1. **If landing not detected:**
   - See "Problem: Apogee Detection Misses" for barometer issues
   - Check if altitude/acceleration data sensible at end of flight
   - May need to widen landing detection thresholds

2. **If LED not working:**
   - Verify LED power supply connected
   - Test LED on bench (apply 3.3V)
   - If LED still doesn't glow: May be burned out
   - Replace NeoPixel and re-test

3. **If buzzer not working:**
   - Verify buzzer power supply connected
   - Test buzzer on bench (apply configured voltage)
   - If buzzer silent: May be broken
   - Replace buzzer and re-test

4. **If thresholds too strict:**
   ```c
   // Make landing easier to detect:
   #define LANDING_ACCEL_MIN_G 0.85f  // Widen range
   #define LANDING_ACCEL_MAX_G 1.15f
   #define LANDING_CONFIRMATION_TIME_MS 3000 // Wait longer
   ```

**Prevention:**
- Test LED and buzzer on bench before flight
- Verify beacon activation after pad idle test (if possible)
- Ensure adequate wait time (15+ seconds) before searching for beacon
- Document which pins are used for LED/buzzer (NEOPIXEL_PIN, BUZZER_PIN)

---

### Problem: Data Not Logged or Corrupted

**Symptoms:** SD card empty, log file truncated, or CSV data contains NaN/errors

**Root Causes:**
1. SD card not mounted or detected
2. SD card full or insufficient free space
3. Power loss during flight (incomplete writes)
4. File system corruption
5. Logging disabled in firmware

**Diagnostic Steps:**

1. **Check SD card detection:**
   ```
   Command: status_sd
   Should show: "SD Card Status: HEALTHY"
   If shows ERROR: SD card not detected
   ```

2. **Check file system:**
   - Remove SD card from rocket
   - Insert into computer SD card reader
   - Verify file system recognized (should be FAT32)
   - Check for `/logs/` directory

3. **Check log file existence:**
   - Navigate to `/logs/` directory on computer
   - Should have at least one file: `flight_*.csv`
   - If no files: Logging may not be enabled

4. **Check file integrity:**
   - Open log file in text editor
   - First line should be header (62 columns)
   - If file ends abruptly: Power loss or corruption

5. **Check power supply:**
   - Look at flight log battery voltage entry
   - Should remain >10.5V throughout flight
   - If drops below 10V: May cause data loss

**Solutions:**

1. **If SD card not detected:**
   - Try different SD card (may be failed card)
   - Verify SD card formatted as FAT32
   - Check SD card slot connections on Teensy
   - Try formatting card on computer, re-inserting

2. **If SD card full:**
   - Delete old log files to free space
   - Need minimum 5MB free (configured in SD_CARD_MIN_FREE_SPACE)
   - Format card completely if space reports incorrectly

3. **If power loss suspected:**
   - Check battery voltage in log
   - If drops near end: Battery depleted
   - Increase battery capacity for next flight
   - Add power supply noise filtering (capacitors)

4. **If logging disabled:**
   ```c
   // In config.h, verify:
   #define DISABLE_SDCARD_LOGGING false  // Should be false (disabled = false)

   // Should be true to enable logging
   // If true: Change to false, recompile, upload
   ```

5. **File recovery (if partially corrupted):**
   - Try opening in spreadsheet (may skip bad rows)
   - Use data recovery tools if header corrupted
   - Extract usable data from middle of file if end corrupted

**Prevention:**
- Format SD card with computer (full format, not quick format)
- Use Class 10 SD card (faster, more reliable)
- Delete old log files before flight
- Monitor battery voltage during pad idle test
- Test logging on bench before flight test

---

## ADVANCED TESTING SCENARIOS

### Test 1: Apogee Detection Voting System

**Objective:** Validate the 2-of-3 voting mechanism works correctly

**Procedure:**

1. **Test with all methods enabled (normal):**
   - Fly normal flight
   - Observe which methods detect apogee
   - Log should show voting information

2. **Test with barometer disabled (simulate failure):**
   - Modify code to skip barometer in voting
   - Fly and verify apogee detected by accel/GPS
   - Should still deploy drogue (2 other methods)

3. **Test with accelerometer disabled:**
   - Modify code to skip accel in voting
   - Fly and verify barometer/GPS detect apogee
   - Should still work correctly

4. **Test with GPS disabled:**
   - Fly indoors or in location with no GPS signal
   - Verify barometer/accel detect apogee
   - Should deploy normally

5. **Timeout failsafe test:**
   - Set apogee timeout to very short (5 seconds)
   - Fly with known coast phase duration
   - Verify timeout triggers if other methods fail
   - Should still deploy drogue safely

**Success Criteria:**
- [ ] At least 2 methods vote in all scenarios
- [ ] Voting tallies visible in log or debug output
- [ ] Apogee detected correctly even with 1 method disabled
- [ ] Timeout failsafe activates if needed
- [ ] Backup timer never prematurely triggers

---

### Test 2: Servo Control Stability

**Objective:** Verify guidance system doesn't cause oscillation or instability

**Procedure:**

1. **Pre-flight verification:**
   - Test servo on bench: Apply PWM signal, observe motion
   - Should be smooth, not chattering
   - Response within 1 second

2. **Pad idle test:**
   - Command: `test_servo 21 0` (full deflection)
   - Observe servo moves smoothly
   - Command: `test_servo 21 90` (neutral)
   - Observe returns to neutral without overshoot

3. **Flight test with low PID gains:**
   - Use conservative PID values (already configured as low)
   - Monitor servo commands in flight log
   - Should see smooth, gradual servo movements
   - Not rapid chattering

4. **Flight analysis:**
   - Plot servo command vs time
   - Should be smooth curve, not oscillating
   - Frequency <1Hz preferred
   - Amplitude increasing/decreasing gradually

5. **High-G test (if available):**
   - Fly high-acceleration motor
   - Verify servos don't saturate (commanded all the way)
   - Should see responsive control despite high acceleration

**Success Criteria:**
- [ ] Servo responds smoothly to commands
- [ ] No chattering or oscillation
- [ ] Servo commands appear reasonable in flight log
- [ ] Control doesn't go unstable under high acceleration
- [ ] Landing is stable (not tumbling or swinging)

---

### Test 3: Recovery Beacon Validation

**Objective:** Verify beacon can be heard/located at distance

**Procedure:**

1. **LED strobe range:**
   - Power on recovery beacon in dark area
   - Walk away from rocket until LED strobe no longer visible
   - Record distance: _____ feet
   - Repeat in daylight: _____ feet

2. **Audio beacon range:**
   - Enable audio beacon
   - Walk away until SOS pattern no longer heard
   - Record distance: _____ feet
   - Repeat in background noise environment: _____ feet

3. **GPS beacon (if applicable):**
   - Power on in open area with GPS lock
   - Move rocket 100 feet away
   - Check if GPS coordinates received match actual distance
   - Record accuracy: ±_____ feet

4. **Combined beacon test:**
   - Simulate recovery situation
   - Hide rocket in grass/weeds
   - Time how long to locate using beacon
   - Document beacon effectiveness

**Success Criteria:**
- [ ] LED visible at >100 feet in daylight
- [ ] LED visible at >500 feet in low light
- [ ] Audio beacon heard at >50 feet in quiet environment
- [ ] Audio beacon heard at >30 feet in background noise
- [ ] GPS coordinates within ±30 feet of actual location
- [ ] Recovery team can locate rocket within 5 minutes

---

### Test 4: Extreme Altitude Flight

**Objective:** Test barometer and GPS accuracy at high altitude (>5000 ft AGL)

**Procedure:**

1. **Pre-flight verification:**
   - Simulate high altitude in altitude chamber (if available)
   - Verify barometer still reads correctly
   - Verify GPS still acquires lock

2. **Flight execution:**
   - Use high-impulse motor or larger rocket
   - Target apogee >5000 feet
   - Record: Actual vs predicted altitude

3. **Sensor comparison:**
   - Compare barometer altitude to GPS altitude
   - Should agree within ±50 meters
   - Larger difference may indicate sensor saturation

4. **Deployment verification:**
   - Main deployment should still work at high altitude
   - Check if main altitude threshold needs adjustment
   - Descent rate may change with air density

5. **Data analysis:**
   - Verify altitude curve smooth (no jumps)
   - Check if any sensor saturation occurred
   - Verify barometer calibration valid at altitude

**Success Criteria:**
- [ ] Apogee detected correctly at >5000 feet
- [ ] Barometer remains responsive and accurate
- [ ] GPS altitude agrees with barometer
- [ ] Landing detection still works at low altitude
- [ ] All sensor data valid throughout flight
- [ ] No saturation or clipping in sensor readings

---

### Test 5: High-G Event Handling

**Objective:** Test KX134 high-G sensor activation and switching

**Procedure:**

1. **Pre-flight setup:**
   - Verify `USE_KX134 1` in config.h
   - Ensure KX134 sensor installed and initialized
   - Check I2C bus has both ICM-20948 and KX134

2. **Motor selection:**
   - Use motor with peak acceleration >15G (high-G motor)
   - This will exceed ICM-20948 ±16G range
   - Should trigger automatic switch to KX134

3. **Flight execution:**
   - Record flight with high-G motor
   - Watch for any sensor switching messages

4. **Data analysis:**
   - Check log for which sensor active during boost
   - Should show ICM data initially
   - May show KX134 data if threshold exceeded
   - Both sensors should show same general behavior

5. **Guidance verification:**
   - If guidance enabled, verify servos still respond
   - Should not glitch during sensor switch
   - Orientation should remain stable

**Success Criteria:**
- [ ] System detects high-G event automatically
- [ ] Switches to KX134 without glitching
- [ ] Flight continues normally
- [ ] Log shows which sensors were active when
- [ ] No data gaps during sensor switch
- [ ] Apogee detection still works with KX134

---

## RISK ASSESSMENT & MITIGATION

| Risk | Probability | Impact | Severity | Mitigation |
|------|-------------|--------|----------|-----------|
| Apogee missed | Low | High | CRITICAL | 2-of-3 voting, backup 20s timeout |
| Drogue fails | Low | High | CRITICAL | Main parachute backup, reserve procedure |
| Main fails | Low | Severe | CRITICAL | Manual recovery procedures, ejection seat? |
| Firmware crash | Very Low | High | HIGH | Watchdog timer auto-resets, EEPROM recovery |
| Sensor dropout | Low | Medium | MEDIUM | Redundant sensors, health checks, fallback |
| GPS loss | Medium | Low | LOW | Barometer primary, GPS validation only |
| Battery depletion | Very Low | High | HIGH | Pre-flight battery test, capacity analysis |
| Parachute damage | Low | High | HIGH | Visual inspection before flight, reserve |
| Loss of vehicle | Medium | Medium | MEDIUM | Recovery beacon, GPS tracking, search grid |
| Data corruption | Very Low | High | HIGH | SD card testing, multiple log attempts |
| Servo jam | Low | Medium | MEDIUM | Pre-flight servo test, current monitoring |
| I2C bus failure | Very Low | Medium | MEDIUM | Pull-up verification, sensor diagnostics |
| Power regulator failure | Very Low | High | HIGH | Voltage monitoring, backup power scheme |
| Structural failure | Very Low | High | CRITICAL | Airframe inspection, CG/stability check |

**Mitigation Strategies:**

1. **Pre-Flight Verification:** Thorough testing catches 90% of issues before flight
2. **Redundant Systems:** Multiple apogee methods, parachutes, sensors
3. **Failsafe Mechanisms:** Backup timers, watchdog, emergency procedures
4. **Monitoring:** Real-time telemetry (if available), LED/beacon indicators
5. **Recovery Planning:** GPS tracking, beacon, recovery grid, communication plan
6. **Post-Flight Analysis:** Detailed log review identifies trends for next flight

---

## SIGN-OFF REQUIREMENTS

**Before Each Flight Test:**

Safety Officer Sign-Off:
- [ ] All safety hazards identified and mitigated
- [ ] Personnel briefed on emergency procedures
- [ ] Recovery plan documented and available
- [ ] No waivers outstanding

Signature: _________________ Date: _______

Range Director Clearance:
- [ ] Airspace officially cleared
- [ ] Waiver approved (if applicable)
- [ ] Weather acceptable for launch
- [ ] Range procedures reviewed

Signature: _________________ Date: _______

Test Director Authorization:
- [ ] All checklist items complete
- [ ] Firmware version recorded: _____
- [ ] Configuration reviewed: _____
- [ ] Backup recovery plan in place
- [ ] Data retrieval plan established

Signature: _________________ Date: _______

Team Lead Acknowledgment:
- [ ] All personnel briefed and understand roles
- [ ] No exceptions or waivers outstanding
- [ ] Abort criteria understood

Signature: _________________ Date: _______

---

**After Successful Test:**

Data Officer Verification:
- [ ] Flight log retrieved and backed up
- [ ] Data analysis complete
- [ ] Flight report written
- [ ] Issues documented

Signature: _________________ Date: _______

Safety Officer Debrief:
- [ ] No hazardous incidents occurred
- [ ] Vehicle recovered safely
- [ ] Any anomalies noted for next flight
- [ ] Recommendations documented

Signature: _________________ Date: _______

---

## FLIGHT TEST REPORT TEMPLATE

Use this template to document each flight test:

```
FLIGHT TEST REPORT
TripleT Flight Firmware Test Flight
====================================

GENERAL INFORMATION
-------------------
Date: [YYYY-MM-DD]
Time (Launch): [HH:MM UTC]
Test Director: [Name]
Safety Officer: [Name]
Location: [Launch Site]
Test Number: [#]
Firmware Version: [v0.10.0]

ENVIRONMENTAL CONDITIONS
------------------------
Temperature: [°C]
Wind Speed: [mph]
Wind Direction: [direction]
Barometric Pressure: [hPa]
Visibility: [feet]
Cloud Cover: [%]
Solar Condition: [clear/hazy/overcast]
Comments: [Any weather notes]

VEHICLE CONFIGURATION
---------------------
Rocket Type: [Design name]
Motor: [Type and impulse class, e.g., L1354 J]
Predicted Apogee: [feet] (from [simulation tool])
Vehicle Mass: [pounds]
CG Location: [% of length] / [distance from nose]
Payload Mass: [pounds]
Drogue Parachute: [Size, type]
Main Parachute: [Size, type]
Comments: [Any modifications from standard]

FLIGHT COMPUTER CONFIGURATION
------------------------------
Flight Computer: Teensy 4.1
Firmware Build: [git commit hash or version]
SD Card Status: OK
Battery Voltage: [V] (pre-flight), [V] (post-flight)
Main Deploy Altitude: 100 m AGL
Apogee Timeout: 20 seconds
Sensors Active:
  - ICM-20948: YES
  - MS5611: YES
  - KX134: [YES/NO]
  - GPS: YES (SPI)
  - NeoPixel LED: YES
  - Audio Beacon: YES
Guidance System: [ENABLED/DISABLED]
Debug Output: [Flags enabled for monitoring]

PREFLIGHT VERIFICATION
----------------------
Hardware Checks:
  - [ ] All solder joints clean
  - [ ] I2C bus voltage correct (3.3V)
  - [ ] Battery voltage adequate (>11V for 3S)
  - [ ] Servo range verified (0-180°)
  - [ ] Pyro continuity confirmed
  - [ ] SD card mounted and >5MB free

Sensor Verification:
  - [ ] Accelerometer: ±0.1g baseline noise
  - [ ] Barometer: Stable and calibrated
  - [ ] GPS: 3D fix with 8+ satellites
  - [ ] Magnetometer: Calibrated

Firmware Verification:
  - [ ] Serial communication at 115200 baud
  - [ ] All commands responsive
  - [ ] Sensors report HEALTHY in status_sensors
  - [ ] LED indicators working
  - [ ] No error messages or warnings

Range Verification:
  - [ ] Airspace cleared
  - [ ] Weather acceptable
  - [ ] Safety perimeter established
  - [ ] Personnel briefed
  - [ ] Backup recovery plan ready

PAD IDLE TEST (15 min before launch)
-----------------------------------
Time Started: [HH:MM UTC]
Time Completed: [HH:MM UTC]
Results:
  - [ ] Liftoff detection: READY (threshold 2.0g)
  - [ ] All sensors: HEALTHY
  - [ ] Flight computer: ARMED
  - [ ] LED indicator: YELLOW (ARMED)
  - [ ] No error states
  - [ ] Acceleration baseline: [X]g, [Y]g, [Z]g
  - [ ] Barometer baseline: [pressure] hPa, [altitude] m
  - [ ] GPS satellites: [count] in view
Final Status: [READY TO LAUNCH / NOT READY - REASON]

FLIGHT EXECUTION
-----------------
Motor Ignition Time: [HH:MM:SS UTC]
Launch Detection Time: [+Xs after ignition]
Flight Duration: [total seconds]

FLIGHT EVENTS OBSERVED:
  Time  Event                        Notes
  ----  -----                        -----
  0s    Motor ignition               Thrust visible
  +1s   Liftoff detected (BOOST)     Acceleration >2.0g
  +7s   Motor burnout (COAST)        Acceleration dropped <0.5g
  +12s  Apogee detected              Altitude peaked
  +13s  Drogue deployed              Parachute deployed
  +14s  Descent under drogue         Stable descent rate
  +45s  Main altitude reached        Approaching 100m
  +47s  Main deployed                Descent rate decreased
  +65s  Landing detected             Acceleration ~1G
  +75s  Recovery beacon activated    LED strobe + audio beacon

Visual Observations:
  - Liftoff: [Description]
  - Boost phase: [Description]
  - Apogee: [Description - altitude, aspect]
  - Drogue deployment: [Description]
  - Main deployment: [Description]
  - Landing: [Description]
  - Recovery: [Description]

FLIGHT COMPUTER DATA
--------------------
SD Card Log File: [flight_YYYYMMDD_HHMMSS.csv]
File Size: [KB]
Data Points: [count]
Duration: [seconds]
Status: [COMPLETE / PARTIAL / CORRUPT]

Log Data Summary:
  Apogee Height (Barometer): [feet] AGL
  Apogee Height (GPS): [feet] AGL
  Apogee Detection Method(s): [Baro/Accel/GPS/Timeout]
  Drogue Deploy Altitude: [feet] AGL
  Main Deploy Altitude: [feet] AGL
  Main Descent Rate: [ft/s]
  Total Flight Time: [seconds]
  Landing Altitude: [feet] (should be near 0)
  Max Temperature: [°C]
  Max Acceleration: [g]
  Min Pressure: [hPa] (at apogee)

Verification:
  - [ ] All 62 CSV columns present
  - [ ] No truncation or corruption
  - [ ] Timestamps monotonically increasing
  - [ ] Altitude profile sensible
  - [ ] Acceleration profile sensible
  - [ ] No NaN or error values
  - [ ] Data parsable by analysis tools

PERFORMANCE ANALYSIS
-------------------
Apogee Comparison:
  Predicted: [feet] (from [tool])
  Actual: [feet]
  Difference: [feet] or [%]
  Status: [PASS / MARGINAL / FAIL]

Motor Burnout:
  Predicted: [seconds]
  Actual: [seconds]
  Status: [PASS / MARGINAL / FAIL]

Descent Rates:
  Drogue Predicted: [ft/s]
  Drogue Actual: [ft/s]
  Main Predicted: [ft/s]
  Main Actual: [ft/s]
  Status: [PASS / MARGINAL / FAIL]

Recovery:
  Predicted Time to Land: [seconds]
  Actual Time to Land: [seconds]
  Landing Detection: [PASS / FAIL]
  Beacon Activation: [PASS / FAIL]

VEHICLE CONDITION POST-FLIGHT
-----------------------------
Vehicle Recovered: [Time - HH:MM UTC]
Damage Assessment:
  - Airframe: [INTACT / DAMAGED]
  - Fins: [INTACT / DAMAGED]
  - Nose Cone: [INTACT / DAMAGED]
  - Parachutes: [INTACT / DEPLOYED / TORN]
  - Overall Condition: [FLYABLE / NEEDS REPAIR / DESTROY]

Notes: [Description of any damage]

Post-Flight Battery Voltage: [V]
Flight Computer Status: [RESPONSIVE / UNRESPONSIVE]
SD Card Retrieved: [YES / NO]
Logs Retrieved: [YES / NO]

ISSUES & ANOMALIES
-----------
[List any anomalies, including:
 - Unexpected state transitions
 - Sensor errors or dropouts
 - Deployment timing issues
 - Data gaps or corruption
 - Servo glitches
 - Audio/LED beacon issues
 - Other observations]

Issue 1: [Description]
Severity: [CRITICAL / HIGH / MEDIUM / LOW]
Impact: [Mission success / Data quality / Safety]
Likely Cause: [Analysis]
Resolution for Next Flight: [Recommendation]

[Repeat for each issue]

LESSONS LEARNED
---------------
What Went Well:
1. [Aspect that worked well]
2. [Positive observation]
3. [Feature that performed well]

Areas for Improvement:
1. [Suggested change]
2. [Configuration adjustment]
3. [Testing recommendation]

Configuration Changes Recommended:
  [List any config.h changes for next flight]

Hardware Changes Recommended:
  [List any hardware modifications needed]

NEXT STEPS
----------
- [ ] SD card data archived and backed up
- [ ] Flight log analyzed and documented
- [ ] Issues investigated and understood
- [ ] Vehicle repaired (if needed)
- [ ] Configuration updated (if needed)
- [ ] Lessons learned incorporated
- [ ] Date scheduled for next test flight: [YYYY-MM-DD]
- [ ] Objectives for next test: [Description]

APPROVAL & SIGN-OFF
-------------------
Test Successful: [YES / NO]

Go/No-Go for Next Flight: [GO / NO-GO - REASON]

Test Director Approval:
Signature: _________________ Date: _______
Name: _________________ Title: _______

Safety Officer Approval:
Signature: _________________ Date: _______
Name: _________________ Title: _______

Data Officer Verification:
Signature: _________________ Date: _______
Name: _________________ Title: _______

Team Lead:
Signature: _________________ Date: _______
Name: _________________ Title: _______

ATTACHMENTS
-----------
- [ ] Flight log CSV file
- [ ] Altitude vs Time plot
- [ ] Acceleration vs Time plot
- [ ] Velocity vs Time plot
- [ ] Post-flight vehicle photos
- [ ] Any flight video footage
- [ ] Simulation comparison document
```

---

## SUMMARY

This comprehensive flight test preparation guide covers:

**1. Pre-Flight Phase (1-2 weeks):**
- Hardware assembly verification and sensor calibration
- Firmware build and configuration review
- Range safety and rocket preparation
- Launch equipment setup

**2. Flight Phases (0-5 minutes):**
- Pad idle verification (final checks before launch)
- Launch and ascent monitoring
- Apogee detection voting
- Descent phases (drogue and main)
- Landing detection and recovery beacon activation
- Post-flight recovery procedures

**3. Post-Flight Phase (24 hours):**
- Immediate safety assessment and data retrieval
- Lab analysis of flight logs
- Performance comparison to predictions
- Anomaly investigation and lessons learned
- Sign-off and approval workflow

**4. Troubleshooting & Advanced Topics:**
- Common issues and solutions
- Advanced testing scenarios
- Risk assessment and mitigation
- Flight test reporting template

**Success depends on:**
- Thorough pre-flight preparation
- Clear communication and defined roles
- Real-time monitoring and decision-making
- Complete data logging and post-flight analysis
- Continuous improvement based on results

Use this document as a reference guide for every flight test to ensure consistency, safety, and data quality across the TripleT Flight Firmware validation campaign.
