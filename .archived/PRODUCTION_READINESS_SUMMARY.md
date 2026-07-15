# Phase 6.3 Production Readiness - Quick Reference

**Document:** PRODUCTION_READINESS_PLAN.md (complete implementation details)
**Status:** Planning → Implementation (Ready to start)
**Duration:** 2.5 weeks
**Target Release:** v1.0.0-RC1

---

## At-a-Glance: 8 Major Components

### 1. Power Management (PowerManager class)
- **4 modes:** ACTIVE (180mA) → COAST_OPTIMIZED (120mA) → RECOVERY (30mA) → SLEEP (2mA)
- **Methods:** Battery monitoring, subsystem control, flight time estimation
- **Location:** `src/power_management.h/cpp`
- **Effort:** 400 lines, 1.5 days

### 2. Edge Case Handling
```
GPS Loss        → Barometer-only apogee detection
High Wind       → Guidance gain reduction + stability margin increase
Sensor Saturation → Already handled (Phase 4 IMUManager failover)
EEPROM Corruption → Checksum verification + safe state fallback (PAD_IDLE)
```
- **Location:** `src/edge_cases/` (4 handler classes)
- **Effort:** 300 lines, 1.5 days

### 3. Pre-Flight Verification (PreflightChecker class)
- **7 checks:** Sensors, Power, Storage, Firmware, EEPROM, Servos, Pyro
- **Time limit:** 30 seconds (enforced timeout)
- **Output:** User-friendly PASS/WARN/FAIL status
- **Location:** `src/preflight_checks.h/cpp`
- **Effort:** 350 lines, 1 day

### 4. New Serial Commands (8 total)
```
preflight         - Run full pre-flight check
telemetry_on/off  - Control power saving
servo_test        - Cycle servos
pyro_test         - Check continuity
load_trajectory   - Load from SD card
start_trajectory  - Begin following
power_mode        - Switch modes
battery           - Voltage + flight time
```
- **Location:** `src/command_processor.cpp`
- **Effort:** 250 lines, 0.5 days

### 5. Thermal Management (ThermalManager class)
- **Thresholds:** Warning @70°C → Critical @85°C → Shutdown @95°C
- **Actions:** Reduce update rates, disable subsystems, enter passive mode
- **Location:** `src/thermal_management.h/cpp`
- **Effort:** 200 lines, 0.5 days

### 6. Signal Integrity & Filtering
- **Kalman filter:** Tuned Q/R matrices from Phase 4 test data
- **Servo filter:** 20Hz low-pass (α=0.3)
- **Gyro filter:** 0.1Hz high-pass (remove bias drift)
- **Notch filter:** Optional, for servo resonance
- **Location:** `src/kalman_filter.h`, `src/servo_control.h`, etc.
- **Effort:** 200 lines, 0.5 days

### 7. Integration & Testing
- **Unit tests:** 10+ tests for all new features
- **Flight tests:** 5 scenarios (long duration, high wind, GPS loss, high-G, extended recovery)
- **Success:** All tests pass, 99% uptime
- **Effort:** 2.5 days

### 8. Main Loop Integration
```cpp
void loop() {
  // 1. Sensors
  imu_manager->update();

  // 2. Power + Thermal (EVERY LOOP)
  power_manager->update();
  thermal_manager->update();

  // 3. Flight logic
  updateFlightLogic();

  // 4. Apply optimizations
  applyPowerOptimizations();
  applyThermalThrottling();

  // 5. Data logging (respects power mode)
  logFlightData();
}
```

---

## System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    MAIN FLIGHT LOOP (10Hz)                   │
└─────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
   ┌─────────┐          ┌──────────────┐      ┌──────────┐
   │ Sensors │          │ Power Mgmt   │      │ Thermal  │
   │         │          │ (Every loop) │      │ Mgmt     │
   │ - IMU   │◄─────┐   │              │      │ (Every   │
   │ - GPS   │      │   │ getMetrics() │      │ loop)    │
   │ - Baro  │      │   │              │      │          │
   └─────────┘      │   └──────────────┘      └──────────┘
        │            │         │                    │
        │            └─────────┼────────────────────┤
        │                      │                    │
        ▼                      ▼                    ▼
   ┌──────────────────────────────────────────────────────┐
   │         Flight Logic + State Machine                  │
   │                                                        │
   │  BOOST → COAST → APOGEE → DROGUE → MAIN → LANDED    │
   │                                                        │
   │  Edge Case Handlers:                                  │
   │  - GPS Loss → Barometer fallback                      │
   │  - High Wind → Reduce guidance gains                  │
   │  - Saturation → Failover to backup sensor            │
   │  - EEPROM corruption → Safe state (PAD_IDLE)        │
   └──────────────────────────────────────────────────────┘
        │
        ▼
   ┌──────────────────────────────────────────────────────┐
   │  Guidance Control                                     │
   │  (Respects power/thermal throttling)                  │
   │                                                        │
   │  - Servo commands (filtered 20Hz low-pass)           │
   │  - PID control (optionally reduced rate)             │
   │  - Stability monitoring                               │
   └──────────────────────────────────────────────────────┘
        │
        ▼
   ┌──────────────────────────────────────────────────────┐
   │  Data Logging (Respects power mode)                   │
   │                                                        │
   │  ACTIVE: Every 500ms                                 │
   │  COAST: Every 1000ms (buffered)                      │
   │  RECOVERY: Disabled                                   │
   └──────────────────────────────────────────────────────┘
        │
        ▼
   ┌──────────────────────────────────────────────────────┐
   │  Telemetry (Respects power mode)                      │
   │                                                        │
   │  ACTIVE: Every 100ms (all fields)                    │
   │  COAST: Every 200ms (50% decimation)                 │
   │  RECOVERY: Disabled                                   │
   └──────────────────────────────────────────────────────┘
```

---

## Power Mode Transitions

```
START (ACTIVE mode)
  │
  ├─→ GPS lost? → GPS_LOSS_HANDLER (barometer fallback)
  ├─→ Battery low? → COAST_OPTIMIZED (save power)
  ├─→ Temp > 70°C? → COAST_OPTIMIZED (thermal throttle)
  │
  └─→ During COAST phase:
      ├─→ Reduce sensor frequency 10Hz→5Hz
      ├─→ Reduce servo update rate
      ├─→ Decimate telemetry 50%
      ├─→ Buffer SD card writes
      └─→ Current: 180mA → 120mA

  └─→ Post-landing RECOVERY mode:
      ├─→ GPS beacon every 30s
      ├─→ All other systems disabled
      └─→ Current: 120mA → 30mA

  └─→ After 24 hours (optional):
      ├─→ SLEEP mode
      ├─→ Watchdog only
      └─→ Current: 30mA → 2mA
```

---

## File Organization

```
src/
├── power_management.h          (PowerManager class)
├── power_management.cpp
│
├── thermal_management.h        (ThermalManager class)
├── thermal_management.cpp
│
├── preflight_checks.h          (PreflightChecker class)
├── preflight_checks.cpp
│
├── command_processor.cpp       (Updated - 8 new commands)
│
├── edge_cases/                 (New directory)
│   ├── gps_loss_handler.h
│   ├── wind_handler.h
│   ├── eeprom_recovery.h
│   └── edge_case_handler.h
│
├── servo_control.h             (Updated - add ServoCommandFilter)
├── servo_smoother.h            (Updated - add filters)
│
├── kalman_filter.h             (Updated - tuned Q/R matrices)
└── gyro_filter.h               (New - high-pass filter)

test/unit/
├── test_power_management.cpp   (10+ tests)
├── test_preflight_checks.cpp
├── test_edge_cases.cpp
├── test_thermal_management.cpp
├── test_filters.cpp
└── test_commands.cpp

test/flight/
├── test_flight_1_long_duration.md
├── test_flight_2_high_wind.md
├── test_flight_3_gps_loss.md
├── test_flight_4_high_g.md
└── test_flight_5_extended_recovery.md
```

---

## Key Integration Points

### 1. Main Loop (100ms tick)
```cpp
power_manager->update();       // Every loop
thermal_manager->update();     // Every loop
```

### 2. Flight State Transitions
```cpp
if (power_manager->isBatteryCritical()) {
  setFlightState(LANDED);
}
```

### 3. Sensor Reads (respecting power mode)
```cpp
if (power_manager->isReduced()) {
  // 5Hz instead of 10Hz
  sample_rate = 5;
}
```

### 4. Data Logging (respecting power mode)
```cpp
uint32_t write_interval = power_manager->getSDCardWriteInterval();
if (now - last_write > write_interval) {
  writeToSD(log_data);
}
```

### 5. Telemetry Output (respecting power mode)
```cpp
bool should_send = power_manager->shouldSendTelemetry();
if (should_send) {
  serial->print(telemetry_packet);
}
```

---

## Pre-Flight Check Output Example

```
╔════════════════════════════════════════╗
║      PRE-FLIGHT VERIFICATION (v1.0)     ║
╚════════════════════════════════════════╝

[1/7] Sensor Health Check...
  ✓ ICM-20948 primary sensor: OK
  ✓ KX134 backup accelerometer: OK
  ✓ MS5611 barometer: OK
  ⚠ GPS receiver: No fix (will use barometer)
  Status: WARN

[2/7] Power System Check...
  ✓ Battery voltage: 7.45V (85% capacity)
  ✓ Estimated flight time: 8.2 minutes
  Status: PASS

[3/7] Storage System Check...
  ✓ SD card: FAT32, 2.1 GB free
  Status: PASS

[4/7] Firmware Configuration Check...
  ✓ Apogee timeout: 60,000ms
  ✓ Stability margin: 20°
  Status: PASS

[5/7] Flight State Check...
  ✓ EEPROM checksum: Valid
  Status: PASS

[6/7] Servo Actuation Test...
  ✓ Roll servo: Full range OK
  ✓ Pitch servo: Full range OK
  Status: PASS

[7/7] Pyro Channel Continuity...
  ✓ Drogue channel: Continuity confirmed
  ✓ Main channel: Continuity confirmed
  Status: PASS

════════════════════════════════════════

SUMMARY:
  Passed: 20/20
  Warnings: 1 (GPS acceptable)
  Failures: 0
  Total time: 28 seconds

RESULT: ✓ SYSTEM READY TO FLY
```

---

## New Serial Commands Quick Reference

| Command | Arguments | Purpose | Response Time |
|---------|-----------|---------|---|
| `preflight` | (none) | Full system check | 30s |
| `telemetry_on` | (none) | Enable data stream | Immediate |
| `telemetry_off` | (none) | Disable (power save) | Immediate |
| `servo_test` | (none) | Cycle servo range | 4s |
| `pyro_test` | (none) | Check continuity | 2s |
| `load_trajectory` | filename | Load from SD | 1s |
| `start_trajectory` | (none) | Begin guidance | Immediate |
| `power_mode` | mode_num | Show/set power mode | Immediate |
| `battery` | (none) | Voltage + flight time | Immediate |

---

## Battery Voltage to Flight Time Lookup

```
Battery     SOC%    Available   Est. Flight Time
Voltage             Energy      (ACTIVE mode)
─────────────────────────────────────────────────
8.40V       100%    2000mAh     11.1 min
8.00V       85%     1700mAh     9.4 min
7.40V       60%     1200mAh     6.7 min
7.00V       40%     800mAh      4.4 min
6.60V       20%     400mAh      2.2 min
6.00V       0%      0mAh        0.0 min

Assumptions:
- 2S LiPo battery, 2000mAh capacity
- ACTIVE mode: 180mA avg consumption
- Battery model from Phase 6.3 spec
```

---

## Critical Thresholds

```
BATTERY VOLTAGE:
  <2.7V  → Critical (switch to ERROR state)
  <3.0V  → Low (switch to RECOVERY mode)
  <3.3V  → Warning (switch to COAST_OPTIMIZED)
  ≥3.3V  → OK (stay in ACTIVE/COAST)

TEMPERATURE (IMU sensor):
  <70°C   → Normal operation
  70-85°C → Throttle (5Hz sampling, reduced PID)
  85-95°C → Critical (2Hz sampling, passive guidance)
  >95°C   → Shutdown (sleep mode)

WIND SPEED (estimated from servo activity):
  <5 mph      → CALM (normal operation)
  5-10 mph    → LIGHT (normal operation)
  10-20 mph   → MODERATE (reduce Ki gain 10%)
  20-30 mph   → STRONG (reduce Kp 20%, increase Kd 20%)
  >30 mph     → EXTREME (disable guidance, passive mode)

GPS STATUS:
  No fix      → Use barometer + accelerometer for apogee
  Lost after boot → Fallback to barometric only
  Duration >5s → Log event, continue with backup

SENSOR SATURATION:
  Primary saturated → Switch to backup (KX134)
  Both saturated → Log high-G event, use last valid quaternion
  Both saturated >1s → Switch to passive guidance mode
```

---

## Testing Checklist

### Unit Tests (10+ tests)
- [ ] Power consumption per mode
- [ ] Battery voltage reading and capacity model
- [ ] GPS loss fallback (barometer+accel voting)
- [ ] Sensor saturation handling
- [ ] EEPROM checksum verification
- [ ] Pre-flight checker (all 7 checks)
- [ ] Thermal throttling thresholds
- [ ] Servo command filtering
- [ ] All 8 new serial commands
- [ ] Edge case handler activation

### Bench Tests (5 days)
- [ ] Day 1: Unit test execution (all passing)
- [ ] Day 2: Subsystem integration
- [ ] Day 3-5: Bench verification, hardware validation

### Flight Tests (5 scenarios)
- [ ] **Flight 1:** Long duration (battery drain)
  - Target: Flight time matches estimate within ±20%
  - Wind: <5 mph

- [ ] **Flight 2:** High wind (stability)
  - Target: Stable flight, gains auto-reduced
  - Wind: 15-20 mph

- [ ] **Flight 3:** GPS loss (sensor redundancy)
  - Target: Apogee detected by barometer+accel, no guidance
  - GPS: Disabled before flight

- [ ] **Flight 4:** High-G boost (saturation)
  - Target: Complete high-G profile logged, KX134 activated
  - Motor: I218 (high power)

- [ ] **Flight 5:** Extended recovery beacon (power modes)
  - Target: Recovery mode <5 seconds after landing, beacon 24+ hours
  - Setup: Manual RECOVERY mode entry

---

## Timeline (2.5 weeks)

```
WEEK 1:
  Mon-Tue  (2d): PowerManager + Edge cases
  Wed-Thu  (1.5d): PreflightChecker + Commands
  Fri      (1d): ThermalManager + Filters

WEEK 2:
  Mon      (0.5d): Integration into main loop
  Tue-Wed  (1.5d): Unit test dev + execution
  Thu-Fri  (1d): Bench testing + flight prep

WEEK 3:
  Mon-Wed  (3d): 5 Flight tests (1 flight per day)
  Thu-Fri  (2d): Analysis, fixes, documentation

TOTAL: 2.5 weeks
```

---

## Success Criteria Summary

| Metric | Target | Status |
|--------|--------|--------|
| Power consumption | 180→120→30→2 mA | ▢ To implement |
| Edge cases handled | 4/4 scenarios | ▢ To test |
| Pre-flight time | <5 minutes | ▢ To verify |
| New commands | 8/8 functional | ▢ To test |
| Thermal control | Works at thresholds | ▢ To test |
| Flight reliability | 99% uptime (5/5 flights) | ▢ To verify |
| Test coverage | >95% code | ▢ To measure |

---

## Next Steps

1. **This Week:** Create file structure + PowerManager class stub
2. **Week 2:** Implement all 8 components in parallel
3. **Week 3:** Integrate + test + flight validation
4. **Release:** Tag v1.0.0-RC1 after successful flight tests

See **PRODUCTION_READINESS_PLAN.md** for complete implementation details.

---

**Document Version:** 1.0
**Created:** February 16, 2026
**Status:** Ready for Implementation
**Next Review:** After PowerManager implementation (Day 2)
