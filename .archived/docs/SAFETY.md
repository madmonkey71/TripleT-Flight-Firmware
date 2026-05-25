# Safety Documentation - TripleT Flight Firmware v0.9.0

**Last Updated:** February 15, 2026
**Target Audience:** Flight crews, safety officers, certification reviewers

---

## Table of Contents

1. [Safety Philosophy](#safety-philosophy)
2. [Hardware Fault Detection](#hardware-fault-detection)
3. [Single Point of Failure Analysis](#single-point-of-failure-analysis)
4. [Error Recovery Procedures](#error-recovery-procedures)
5. [Critical System Tests](#critical-system-tests)
6. [Pre-Flight Checklist](#pre-flight-checklist)
7. [Watchdog Reset Recovery](#watchdog-reset-recovery)
8. [Failure Scenarios](#failure-scenarios)
9. [Design Redundancy](#design-redundancy)

---

## Safety Philosophy

The TripleT Flight Firmware implements a **defense-in-depth** safety strategy:

```
Layer 1: Prevention
  ├─ Sensor validation before use
  ├─ Configuration checks at startup
  └─ Pre-flight system tests

Layer 2: Detection
  ├─ Continuous sensor health monitoring
  ├─ Range checking on all values
  └─ Timeout watchdogs

Layer 3: Mitigation
  ├─ Automatic failover to backup systems
  ├─ Graceful degradation (continue with reduced capability)
  └─ Error state with human override

Layer 4: Recovery
  ├─ Watchdog timer hardware reset
  ├─ EEPROM state persistence
  └─ Failsafe parachute deployment
```

### Core Principles

1. **Fail Safe by Default**
   - If system cannot make a decision, default to recovery actions
   - Better to deploy parachute unnecessarily than miss deployment

2. **Redundancy at Critical Points**
   - Apogee: 4 independent detection methods
   - Altitude: Multiple sensors (baro + GPS)
   - Acceleration: Two IMU sensors

3. **Strict Validation**
   - All sensor inputs checked before use
   - Range checking on all calculations
   - Bounds verification on state transitions

4. **Human Override**
   - Serial commands allow manual intervention
   - Clear error messages to guide recovery
   - Recovery state for safe data download

---

## Hardware Fault Detection

### Sensor Health Checks

#### Accelerometer (ICM-20948) - Primary

```
Health Check    │ Method              │ Frequency │ Threshold
────────────────┼─────────────────────┼───────────┼─────────────
I2C Response    │ Ping sensor         │ Every 10ms│ Must respond
Data Freshness  │ Timestamp tracking  │ Every 10ms│ < 100ms old
Range Check     │ ±100 m/s² limits    │ Every 10ms│ Within range
Temperature     │ -50°C to +150°C     │ Every 10ms│ In range
Self-Test       │ On startup only     │ Once      │ Must pass
```

**Failure Mode:** If consecutive failures > 3, mark sensor unhealthy

**Recovery:** Switch to backup sensor (KX134/BNO085)

#### Accelerometer (KX134) - Backup

```
Health Check    │ Method              │ Frequency │ Threshold
────────────────┼─────────────────────┼───────────┼─────────────
I2C Response    │ Ping sensor         │ Every 10ms│ Must respond
Data Freshness  │ Timestamp tracking  │ Every 10ms│ < 100ms old
Range Check     │ ±150 m/s² limits    │ Every 10ms│ Within range
Temperature     │ -40°C to +125°C     │ Every 10ms│ In range
```

**Failure Mode:** If both sensors fail, enter ERROR state

**Recovery:** Wait for manual `clear_errors` command

#### Barometer (MS5611)

```
Health Check    │ Method              │ Frequency │ Threshold
────────────────┼─────────────────────┼───────────┼─────────────
I2C Response    │ Ping sensor         │ Every 100ms│ Must respond
Data Freshness  │ Timestamp tracking  │ Every 100ms│ < 1s old
Range Check     │ 0m to 50,000m       │ Every 100ms│ Within range
Rate of Change  │ Max Δ altitude      │ Every 100ms│ < 1000 m/s
```

**Rate of Change Validation:**
```
Max physically possible rates:
  ├─ Launch phase (BOOST): < 500 m/s (Mach 1.5+)
  ├─ Coasting phase: < 300 m/s
  ├─ Descent phase: < 100 m/s (terminal velocity)
  └─ Recovery phase: < 30 m/s
```

**Failure Mode:** Pressure reading > 1000m change in 100ms = sensor error

#### GPS Receiver

```
Health Check    │ Method              │ Frequency │ Threshold
────────────────┼─────────────────────┼───────────┼─────────────
Serial Response │ Parse NMEA data     │ Every 1s  │ Must respond
Lock Status     │ GPS fix status      │ Every 1s  │ 2D or 3D lock
Data Freshness  │ Age of last fix     │ Every 1s  │ < 5s old
Altitude Change │ Rate validation     │ Every 1s  │ < 500 m/s
```

**Failure Mode:** No GPS fix after 30 seconds = degraded (continue with baro)

**Recovery:** Use barometer-only apogee detection (still have 2 methods)

### Sensor Validation Algorithm

```cpp
bool validateSensorData(const IMUData& data) {
  // 1. Range checks
  if (abs(data.accel_x) > 100.0f) return false;  // m/s²
  if (abs(data.accel_y) > 100.0f) return false;
  if (abs(data.accel_z) > 100.0f) return false;

  // 2. Consistency checks
  float mag = sqrt(data.accel_x² + data.accel_y² + data.accel_z²);
  if (mag > 100.0f) return false;  // sqrt(3*100²) worst case

  // 3. Quaternion normalization
  float qmag = sqrt(data.qw² + data.qx² + data.qy² + data.qz²);
  if (abs(qmag - 1.0f) > 0.1f) return false;  // Should be ~1.0

  // 4. Rate-of-change detection
  if (abs(data.altitude - last_altitude) > 1000.0f) return false;

  // 5. Temperature sanity
  if (data.temp < -50.0f || data.temp > 150.0f) return false;

  return true;  // All checks passed
}
```

### Watchdog Timer (IWatchdog)

```
Function              │ Timeout    │ Action
──────────────────────┼────────────┼────────────────
Main Loop            │ 1000ms     │ Automatic reset
Sensor Read          │ 500ms      │ Skip update
I2C Operation        │ 100ms      │ Abort, return error
SD Card Write        │ 5000ms     │ Timeout, continue
GPS Serial Read      │ 2000ms     │ No new data
```

**Watchdog Feed Strategy:**

```cpp
void loop() {
  // Pet watchdog regularly
  if (all_systems_ok) {
    hal->watchdog()->feed();  // Reset timeout counter
  }
  // If watchdog times out, system resets automatically
  // Upon recovery, check EEPROM for last flight state
}

// Detect watchdog reset
void setup() {
  if (hal->watchdog()->isWatchdogReset()) {
    // Power-loss recovery: restore flight state from EEPROM
    FlightStateData state_data;
    hal->eeprom()->read(EEPROM_STATE_ADDR, &state_data, sizeof(state_data));
    setFlightState(state_data.state);
    log_event("RECOVERED_FROM_WATCHDOG");
  }
}
```

---

## Single Point of Failure Analysis

### SPF 1: Microcontroller (Teensy 4.1)

**Failure Impact:** Complete system shutdown

**Symptoms:**
- No serial output
- No LED indicators
- No recovery system activation

**Mitigation:**
- Watchdog timer (hardware reset on hang)
- Power-loss recovery via EEPROM
- Failsafe timer: Pyro charges activate 25s after launch if no signal

**Pre-Flight Check:**
```
1. Serial communication working (at 115200 baud)
2. LED blink test (blue LED should blink)
3. Watchdog test (manual command triggers reset)
```

### SPF 2: Primary IMU Sensor (ICM-20948)

**Failure Impact:** No acceleration/rotation data, apogee detection impaired

**Symptoms:**
- Serial `status_sensors` shows "ICM: FAIL"
- Apogee only uses barometer + GPS + timer (still 3 methods)
- Guidance system disabled (no attitude reference)

**Mitigation:**
- Automatic failover to KX134 high-G accelerometer
- Backup apogee detection via barometer + GPS
- Timeout failsafe (20s after motor burnout)

**Recovery:**
- System continues in degraded mode
- Manual guidance disabled; unguided descent
- Parachutes still deploy at correct altitudes

**Pre-Flight Check:**
```
1. Serial `status_sensors` command
2. Check "ICM: OK" status
3. Verify accel/gyro values changing with movement
4. Confirm temperature reading in reasonable range
```

### SPF 3: Barometer (MS5611)

**Failure Impact:** Altitude-based decisions unreliable

**Symptoms:**
- `status_sensors` shows "BARO: FAIL" or "BARO: UNRELIABLE"
- Apogee uses accel + GPS + timer (still 3 methods)
- Main deployment uses GPS altitude only

**Mitigation:**
- GPS altitude as backup (typically ±10m accurate)
- Accelerometer-based apogee detection
- Timeout failsafe (20s after motor burnout)

**Recovery:**
- System continues with GPS + accel methods
- Loss of precision but maintains safety

**Pre-Flight Check:**
```
1. Barometer reading within 100m of actual altitude
2. Sensitivity: Read altitude, climb 10m stairs, verify Δ~10m
3. Temperature compensation working
```

### SPF 4: GPS Receiver

**Failure Impact:** Position and high-altitude data unavailable

**Symptoms:**
- `status_sensors` shows "GPS: NO FIX"
- Still have barometer + accel for apogee
- Recovery beacon cannot transmit coordinates

**Mitigation:**
- Multi-method apogee detection (works without GPS)
- Barometric altitude-based main deployment
- Recovery state: Buzzer still active; LED still strobes

**Recovery:**
- System functions normally
- Search must use last known GPS position from pre-flight

**Pre-Flight Check:**
```
1. GPS lock obtained (typically 30-60 seconds)
2. Coordinates reasonable (within 100m of launch site)
3. Altitude within 50m of known elevation
4. SNR/Quality indicators show good signal
```

### SPF 5: SD Card

**Failure Impact:** Data not logged (no flight telemetry recovery)

**Symptoms:**
- `status_sensors` shows "SD: FAIL"
- Flight continues normally
- No CSV data file created

**Mitigation:**
- Flight proceeds without logging
- Recovery system still works
- Manual data retrieval from flight computer (serial dump)

**Recovery:**
- Insert good SD card for next flight
- Previous flight data lost (sad, but not unsafe)

**Pre-Flight Check:**
```
1. SD card inserted
2. Card recognized (`status_sensors` shows "SD: OK")
3. Write test: `log_test` command creates test entry
4. Free space > 100MB
```

### SPF 6: Parachute Deployment Pyro

**Failure Impact:** Parachute not deployed

**Symptoms:**
- Physical: No ignition of pyro charge
- Electrical: GPIO pin stuck LOW (never goes HIGH)
- Detection: Deployment occurs but no separation detected

**Mitigation:**
- Redundant charges (drogue + main = 2 separate circuits)
- If both fail, manual back-up deployment
- Pre-flight continuity check (buzzer tone)

**Recovery:**
- Automatic descent under terminal velocity (~50 m/s)
- Still safe if parachute pack manually triggered

**Pre-Flight Check:**
```
1. Pyro continuity test: Command `pyro_test` for 100ms
2. Listen for clicking relay
3. Multimeter check: 0Ω when active, >1MΩ when inactive
4. Visual: Check pyro charges installed, crimped properly
```

---

## Error Recovery Procedures

### ERROR State Behavior

When a critical error is detected:

```
Detection → Log Error → Enter ERROR State
                ↓
            Attempt Recovery:
              ├─ Sensor health checks
              ├─ Reinitialize bad sensors
              └─ Retry operations

            If Recovery Failed → Stay in ERROR
              ├─ Disable critical systems
              ├─ Activate beacon/light
              ├─ Wait for manual intervention
              └─ Accept serial commands only
```

### Serial Error Commands

```
Command              │ Action
─────────────────────┼─────────────────────────────────────
status_sensors       │ Report all sensor health
clear_errors         │ Attempt to recover and return to PAD_IDLE
reset                │ Force watchdog reset
calibrate            │ Re-run sensor calibration
help                 │ List available commands
```

### Error Recovery Flow

```cpp
void handleErrorState() {
  unsigned long error_start = hal->timer()->millis();

  while (current_state == ERROR) {
    // 1. Attempt recovery every 10 seconds
    if (hal->timer()->millis() - error_start > 10000) {
      log_event("Attempting error recovery...");

      // 2. Re-check all sensors
      bool all_ok = true;
      if (!imu_manager->isAnyHealthy()) all_ok = false;
      if (!isBarometerHealthy()) all_ok = false;

      // 3. If sensors now OK, try to recover
      if (all_ok) {
        current_state = PAD_IDLE;
        log_event("Recovered from ERROR state");
        return;
      }

      error_start = hal->timer()->millis();  // Reset timer
    }

    // 4. Activate safe indicators
    hal->gpio()->digitalWrite(BUZZER_PIN, HIGH);
    hal->timer()->delay(500);
    hal->gpio()->digitalWrite(BUZZER_PIN, LOW);
    hal->timer()->delay(500);

    // 5. Check for manual clear command
    if (hal->serial()->available()) {
      char cmd = hal->serial()->read();
      if (cmd == 'c') {  // Manual clear
        hal->serial()->println("Manual clear requested");
        current_state = PAD_IDLE;
        return;
      }
    }

    hal->watchdog()->feed();  // Keep watchdog happy
  }
}
```

### Recovery Decision Tree

```
System Error Detected?
  │
  ├─ NO → Continue normal operation
  │
  └─ YES → Identify Error Type
       │
       ├─ PRIMARY IMU FAIL
       │   ├─ Can use KX134?
       │   │   ├─ YES → Switch to backup, continue (log event)
       │   │   └─ NO → Enter ERROR
       │   │
       ├─ BOTH IMU FAIL
       │   └─ Enter ERROR
       │
       ├─ BAROMETER FAIL
       │   ├─ In COAST state?
       │   │   ├─ YES → Use accel + GPS + timer, continue
       │   │   └─ NO → Enter ERROR (can't detect apogee later)
       │   │
       ├─ GPS FAIL
       │   ├─ Have baro + accel?
       │   │   ├─ YES → Continue (lose position only)
       │   │   └─ NO → Enter ERROR
       │   │
       ├─ SD CARD FAIL
       │   └─ Continue (no logging, but flight OK)
       │
       └─ WATCHDOG TIMEOUT
           └─ Auto-reset → Recovery from EEPROM state
```

---

## Critical System Tests

### Test 1: Sensor Initialization

**Objective:** Verify all sensors respond and provide valid data

**Procedure:**
```bash
# Connect Teensy via USB
pio device monitor --baud 115200

# In monitor window, type:
> status_sensors
```

**Expected Output:**
```
=== SENSOR STATUS ===
IMU Primary (ICM-20948):  HEALTHY
  Accel: +0.0, -0.0, +9.81 m/s²
  Gyro:  +0.1, -0.2, +0.0 deg/s
  Temp:  +25.3°C

IMU Backup (KX134):       HEALTHY
  Accel: +0.0, -0.1, +9.81 m/s²

Barometer (MS5611):       HEALTHY
  Altitude: 125m
  Pressure: 101325 Pa

GPS (u-blox):             HEALTHY (3D fix)
  Latitude: 40.1234°N
  Longitude: -105.5678°W
  Altitude: 1234m MSL
```

**Pass Criteria:**
- All sensors report HEALTHY
- Acceleration ~9.81 m/s² at rest (±0.5)
- Temperature in reasonable range (-50°C to +80°C)
- Barometer altitude within ±50m of known value
- GPS lock with valid position

**Fail Criteria:**
- Any sensor reports FAIL or ERROR
- Acceleration > 10 m/s² at rest (indicates misalignment)
- Temperature out of range (may indicate hardware failure)
- Do NOT proceed with flight if any fail criteria met

### Test 2: IMU Movement Response

**Objective:** Verify sensors respond to physical motion

**Procedure:**
```bash
> status_sensors
> [Tilt Teensy 90° to one side]
> status_sensors
> [Rotate Teensy 180°]
> status_sensors
```

**Expected Changes:**
- Acceleration values change significantly
- Gyroscope shows rotation rate during motion
- Quaternion updates to reflect new orientation

**Pass Criteria:**
- Accel values change by > 5 m/s² when tilted
- Gyro shows > 100 deg/s during rotation
- Quaternion noticeably different after rotation

### Test 3: Pyro Continuity

**Objective:** Verify pyro ignition circuits functional

**Procedure:**
```bash
# 1. Connect multimeter to pyro pin
# 2. In monitor, type:
> pyro_test
```

**Expected Behavior:**
- Buzzer sounds (beep)
- Relay clicks (audible)
- Multimeter shows 5V for 500ms, then 0V
- No actual ignition (circuit not armed)

**Pass Criteria:**
- Clear beep and click heard
- Multimeter shows momentary voltage
- Continuity check passes (< 10Ω resistance when active)

### Test 4: Watchdog Timer

**Objective:** Verify watchdog can reset system

**Procedure:**
```bash
# In monitor, type:
> watchdog_test
> [System should reset within 2 seconds]
> [Teensy LED should flash, system restarts]
```

**Expected Output After Reset:**
```
TripleT Flight Firmware v0.9.0
Recovering from watchdog reset...
State recovered: PAD_IDLE
Altitude: 125m
Event: WATCHDOG_RESET
```

**Pass Criteria:**
- System resets automatically
- Recovers to PAD_IDLE state
- Watchdog reset event logged

### Test 5: Data Logging

**Objective:** Verify SD card logging functional

**Procedure:**
```bash
> log_test
> [Wait 10 seconds]
> status_sensors
```

**Expected Output:**
```
SD Card: HEALTHY
Last logged: 0.5 seconds ago
Log file: FLIGHT001.CSV
Records written: 50
```

**Pass Criteria:**
- SD card recognized and writable
- CSV file created with correct headers
- Data being written every ~20ms
- File size growing

### Test 6: Full Flight Simulation

**Objective:** Simulate complete flight sequence

**Procedure:**
1. Use mock sensor or replay recorded flight data
2. Verify state transitions occur correctly
3. Check apogee detection triggers at right time
4. Verify parachute deployment signals generated

**Expected Sequence:**
```
STARTUP (2s)
  └─ CALIBRATION (5s)
      └─ PAD_IDLE (wait for arm)
          └─ ARMED (accept acceleration input)
              └─ BOOST (50 m/s accel)
                  └─ COAST (deceleration)
                      └─ APOGEE (2-of-3 vote)
                          └─ DROGUE_DEPLOY (fire!)
                              └─ DROGUE_DESCENT (50 seconds)
                                  └─ MAIN_DEPLOY (alt < 300m)
                                      └─ MAIN_DESCENT (smooth)
                                          └─ LANDED (0 velocity)
                                              └─ RECOVERY (beacon on)
```

---

## Pre-Flight Checklist

### 30 Minutes Before Flight

```
□ Battery Level
  ├─ [ ] Main battery > 10V
  ├─ [ ] Backup battery (if present) > 3V
  └─ [ ] No battery warning in logs

□ Sensor Health
  ├─ [ ] Connect Teensy via USB
  ├─ [ ] Open serial monitor (115200 baud)
  ├─ [ ] Type: status_sensors
  ├─ [ ] All sensors report HEALTHY
  ├─ [ ] GPS shows 3D fix (or 2D minimum)
  └─ [ ] Disconnect USB

□ Physical Inspection
  ├─ [ ] Pyro charges installed (both drogue + main)
  ├─ [ ] Pyro charges crimped and continuity checked
  ├─ [ ] Parachutes packed correctly
  ├─ [ ] Parachute backup deployment manual ripcord functional
  ├─ [ ] Nosecone / altimeter bay sealed
  └─ [ ] Airframe structurally sound

□ SD Card
  ├─ [ ] SD card inserted
  ├─ [ ] Card formatted as FAT32
  ├─ [ ] Free space > 100MB
  └─ [ ] Previous flight logs backed up

□ Firmware
  ├─ [ ] Latest firmware uploaded
  ├─ [ ] Firmware version matches release notes
  └─ [ ] No compilation warnings
```

### 5 Minutes Before Flight

```
□ Final System Check
  ├─ [ ] Connect Teensy via USB
  ├─ [ ] Serial monitor: status_sensors (all HEALTHY)
  ├─ [ ] Payloads secured
  ├─ [ ] Recovery system armed (flags in place)
  └─ [ ] Disconnect USB

□ Launch Site Safety
  ├─ [ ] Clear area 500m downrange
  ├─ [ ] All personnel behind launch pad
  ├─ [ ] Recovery team ready
  └─ [ ] Spotter ready
```

### 1 Minute Before Flight

```
□ Final Checks
  ├─ [ ] Rocket on pad, no vibration
  ├─ [ ] Wind acceptable (< 15 mph)
  ├─ [ ] Sky clear (no aircraft)
  └─ [ ] Launch officer ready

□ Arm Sequence
  ├─ [ ] Clear everyone back
  ├─ [ ] Plug in Teensy via USB
  ├─ [ ] Serial: arm
  ├─ [ ] Verify: ARMED state acknowledged
  ├─ [ ] Disconnect USB
  └─ [ ] READY TO LAUNCH
```

### Post-Flight Procedures

```
□ Immediate (After Recovery)
  ├─ [ ] Rocket secured and disarmed
  ├─ [ ] All pyro charges removed (if intact)
  ├─ [ ] Rocket inspected for damage
  └─ [ ] Recovery system re-packed

□ Data Recovery (Within 1 hour)
  ├─ [ ] Connect Teensy via USB
  ├─ [ ] Download flight data CSV
  ├─ [ ] Verify complete flight recorded
  ├─ [ ] Extract interesting events
  └─ [ ] Create backup copy

□ Analysis
  ├─ [ ] Plot altitude vs time
  ├─ [ ] Verify apogee detection accuracy
  ├─ [ ] Check parachute deployment timing
  ├─ [ ] Identify any anomalies
  └─ [ ] Log issues for future improvement
```

---

## Watchdog Reset Recovery

### Watchdog Operation

```
Normal Operation                Watchdog Timeout
───────────────────             ────────────────
loop() executing
  │
  ├─ Read sensors ✓
  ├─ Update state ✓
  ├─ Log data ✓
  └─ Feed watchdog ✓ ◄─ Reset timeout counter
      │
      └─ Repeat in 10ms


If loop() hangs (infinite loop, deadlock):
  │
  ├─ watchdog.feed() never called
  ├─ Timeout expires (1000ms)
  └─ Hardware automatically resets Teensy ✓
      │
      └─ setup() runs again
          └─ Detect: watchdog()->isWatchdogReset() = true
              └─ Restore state from EEPROM
                  └─ Continue flight from saved state
```

### Recovery from Watchdog Reset

```cpp
// In setup()
void setup() {
  // ... initialize HAL ...

  // 1. Detect watchdog reset
  if (hal->watchdog()->isWatchdogReset()) {
    hal->serial()->println("Watchdog reset detected!");

    // 2. Restore flight state from EEPROM
    FlightStateData saved_state;
    hal->eeprom()->read(EEPROM_STATE_ADDR,
                        (uint8_t*)&saved_state,
                        sizeof(saved_state));

    // 3. Validate saved state
    if (saved_state.signature == EEPROM_SIGNATURE_VALUE) {
      // 4. Restore to saved state
      current_flight_state = saved_state.state;
      current_altitude = saved_state.altitude;
      current_timestamp = saved_state.timestamp;

      // 5. Log recovery event
      log_event("RECOVERED_FROM_WATCHDOG");
      hal->serial()->println("Flight state restored");
    } else {
      // Signature invalid, start from scratch
      hal->serial()->println("Signature mismatch, full restart");
      goto normal_startup;
    }
  } else {
normal_startup:
    // Normal cold start
    // ... calibration ...
  }

  // 6. Reinitialize watchdog
  hal->watchdog()->begin(WATCHDOG_TIMEOUT_MS);
}
```

### EEPROM State Persistence

```
Offset  │ Size  │ Field              │ Purpose
────────┼───────┼────────────────────┼──────────────────────────
0       │ 1     │ flight_state       │ Current state enum
1       │ 4     │ timestamp          │ Time when saved
5       │ 4     │ altitude_m         │ Current altitude
9       │ 2     │ signature          │ Validity check (0xBEEF)
```

**Saved After Each State Change:**
```cpp
void setFlightState(FlightState new_state) {
  current_state = new_state;

  // Save to EEPROM immediately
  FlightStateData data;
  data.state = current_state;
  data.timestamp = hal->timer()->millis();
  data.altitude = current_altitude_m;
  data.signature = EEPROM_SIGNATURE_VALUE;

  hal->eeprom()->write(EEPROM_STATE_ADDR,
                       (uint8_t*)&data,
                       sizeof(data));
  hal->eeprom()->commit();  // Ensure write completes
}
```

---

## Failure Scenarios

### Scenario 1: Primary IMU Fails During COAST

**Initial State:** COAST phase, actively fusing orientation

**Failure:** ICM-20948 stops responding (I2C timeout)

**System Response:**
1. IMUManager::read() returns false
2. primary_healthy = false
3. Next read uses KX134 (backup)
4. Guidance control automatically disabled (KX134 has no gyro)
5. Rocket continues unpowered flight

**Outcome:** Safe
- Apogee detection still works (barometer + GPS + timer)
- Parachute deployment unaffected
- Loss of guidance, but unguided rockets still safe

**Recovery:** Rocket descends under parachute, manual recovery

---

### Scenario 2: Barometer Fails, GPS Fails

**Initial State:** Just entered COAST phase

**Failures:** Both MS5611 and GPS stop responding

**System Response:**
1. Apogee detection switches to accel-only + timer
2. Accelerometer downward reading triggers APOGEE
3. Drogue deploys (might be early, but safe)
4. Main deploys based on timer (30s after drogue)

**Outcome:** Safe (but suboptimal)
- Drogue might deploy mid-coast instead of at apogee
- Main deploys at fixed time, might be too high or too low
- No position data for recovery

**Mitigation:** Pre-flight GPS lock + barometer check prevents this

---

### Scenario 3: Watchdog Timeout During Launch

**Initial State:** BOOST phase, peak acceleration

**Failure:** Main loop hangs (example: I2C deadlock reading sensor)

**System Response:**
1. Watchdog times out (1000ms)
2. Hardware resets Teensy
3. setup() detects watchdog reset
4. EEPROM restores: current_state = BOOST, altitude = 500m
5. loop() resumes as if normal
6. Flight continues

**Outcome:** Safe
- Brief (< 1s) loss of data logging
- Flight state preserved
- Recovery system unaffected

---

### Scenario 4: Power Supply Failure

**Initial State:** Mid-flight (any state)

**Failure:** Battery connection lost (example: bad contact)

**System Response:**
1. Teensy powers down abruptly
2. Watchdog cannot help (no power)
3. Rocket continues coasting

**Recovery Action:**
1. On impact, power contacts may reseat
2. On power-up, watchdog reset flag set
3. EEPROM restores previous state
4. Recovery beacon activates
5. GPS coordinates transmitted (if GPS working)

**Outcome:** Degraded but recoverable
- May miss apogee detection (timer failsafe deploys drogue)
- Data loss since last save
- Recovery still possible with beacon + GPS

**Mitigation:** Redundant power (two battery packs if possible)

---

### Scenario 5: Both Parachutes Fail to Deploy

**Initial State:** APOGEE detected, drogue deployment commanded

**Failure:** Both pyro charges misfire (bad crimping, weak ignition)

**System Response:**
1. GPIO command sent, relay clicks, but no ignition
2. No feedback (no accelerometer-based detection of deployment)
3. After 5 seconds, manual backup system should trigger
4. If all electronic systems fail, manual ripcord must be used

**Outcome:** Degraded
- Rocket falls uncontrolled at terminal velocity (~50 m/s)
- Manual backup system must function
- Recovery team must locate and retrieve

**Mitigation:**
- Pre-flight continuity testing
- Pyro charge backup (cold fire)
- Manual ripcord backup
- Beacon for location finding

---

## Design Redundancy

### Apogee Detection - 4-Method Voting

```
Method 1: Barometer (MS5611)
  ├─ Detects altitude decline
  └─ Most reliable in clear air

Method 2: Accelerometer (ICM-20948 or KX134)
  ├─ Detects downward acceleration
  └─ Measures magnitude directly

Method 3: GPS (u-blox)
  ├─ Detects GPS altitude decline
  └─ Independent of onboard sensors

Method 4: Timeout Failsafe
  ├─ Triggers 20s after motor burnout
  └─ Catches anything unexpected

Deployment Logic: (baro_descent && accel_descent) || gps_descent || timeout
```

### Altitude-Based Decisions - 2-Method Voting

```
Main Deployment Altitude Decision:

Method 1: Barometer (MS5611)
  └─ Primary source

Method 2: GPS Altitude (u-blox)
  └─ Backup (typically ±10m accurate)

Decision: Use barometer if healthy, else GPS
```

### Sensor Redundancy

```
Acceleration Measurement:
  ├─ Primary: ICM-20948 (±16G, built-in 9-DOF)
  └─ Backup: KX134 (±64G for high-G events) OR BNO085

Altitude Measurement:
  ├─ Primary: MS5611 Barometer (±1m typical)
  └─ Backup: GPS (±10m typical)

Orientation Measurement:
  ├─ Primary: Kalman filter (gyro + accel fusion)
  └─ Backup: Gravity vector (if gyro fails)

Position Measurement:
  ├─ Primary: GPS (pre-flight reference)
  └─ Backup: Beacon + manual search

Recovery Signaling:
  ├─ Primary: Audio beacon (buzzer at 4kHz)
  ├─ Secondary: LED strobe (visible in daylight)
  └─ Tertiary: GPS position (if receiver working)
```

---

## Testing Safety-Critical Code

### Unit Test Example: Apogee Detection

```cpp
void test_apogee_barometric_descent() {
  // Setup: Simulate coasting at 500m, starting descent
  current_state = COAST;
  setAltitude(500.0f);

  // Act: Simulate 5 consecutive altitude declines
  for (int i = 0; i < 5; i++) {
    setAltitude(500.0f - (i * 10.0f));  // 490, 480, 470, ...
    update_flight_logic();
  }

  // Assert: Should detect apogee
  TEST_ASSERT_EQUAL(APOGEE, current_state);
}

void test_apogee_false_positive_prevention() {
  // Setup: Rapid pressure fluctuation (sensor noise)
  current_state = COAST;
  setAltitude(500.0f);

  // Act: Single altitude drop (not consecutive)
  setAltitude(499.0f);
  update_flight_logic();

  // Assert: Should NOT trigger apogee
  TEST_ASSERT_EQUAL(COAST, current_state);
}

void test_apogee_timer_failsafe() {
  // Setup: Motor burnout detected 20 seconds ago
  current_state = COAST;
  motor_burnout_time = current_time - 20000;  // 20 seconds ago

  // Act: No other apogee indicators present
  // (barometer stuck, GPS jammed, accel filtered)

  // Assert: Timer failsafe should trigger apogee
  update_flight_logic();
  TEST_ASSERT_EQUAL(APOGEE, current_state);
}
```

---

## References & Further Reading

- **DO-178C** - Safety-critical avionics standard
- **MISRA C++** - Embedded systems coding guidelines
- **ARP4761** - Failure analysis methodology
- **IEC 61508** - Functional safety standard

---

**For implementation questions, see:**
- `DEVELOPER_GUIDE.md` - Code patterns
- `src/flight_logic.cpp` - State machine implementation
- `src/command_processor.cpp` - Command handling
- `docs/ERROR_CODES.md` - Error reference
