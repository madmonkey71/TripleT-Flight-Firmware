# TripleT Flight Firmware - Code Review Findings 2026

**Review Date:** February 14, 2026
**Firmware Version:** v0.51 (Beta)
**Codebase Size:** ~6,700 lines of code
**Reviewers:** Automated analysis + Claude Code comprehensive review

---

## Executive Summary

The TripleT Flight Firmware (v0.51) has successfully addressed **all critical safety bugs** identified in previous reviews. The code is **functionally complete and flight-ready for hobby/personal use**. However, architectural debt exists that limits testability and maintainability. This review identifies remaining issues and provides actionable recommendations for production quality.

**Overall Code Quality Score: 5.4/10**

### Critical Bugs Fixed Since Last Review ✅

1. ✅ **Non-blocking Serial I/O** - Previously blocked main loop with `Serial.readStringUntil()`
2. ✅ **Landing Detection Timer Reset** - Fixed commented-out timer reset bug
3. ✅ **Non-blocking Pyro Firing** - Replaced `delay()` with time-based state management
4. ✅ **Calibrate Command DoS** - Added state guards to prevent freeze during flight
5. ✅ **Watchdog Timer** - Implemented WDT_T4 with 5-second timeout

### New Critical Issue Identified 🔴

1. **Apogee Detection Counter Reset Bug** - Fixed in this review (February 14, 2026)
   - **File**: `src/flight_logic.cpp:904-966`
   - **Issue**: Static counters not reset between flights
   - **Risk**: False apogee on flight reuse without restart
   - **Status**: **FIXED** (refactored to file-scope static with reset function)

---

## Code Quality Scorecard

| Dimension | Score | Status | Notes |
|-----------|-------|--------|-------|
| **Real-Time Safety** | 7/10 | ✅ Good | Critical fixes implemented, needs testing |
| **Code Organization** | 4/10 | ⚠️ Poor | Monolithic functions, needs refactoring |
| **Modularity** | 3/10 | ⚠️ Poor | 21 global externs create tight coupling |
| **Testing Coverage** | 0/10 | ❌ Critical | <5% automated test coverage |
| **Documentation** | 5/10 | ⚠️ Acceptable | Sparse inline comments, good high-level docs |
| **Error Handling** | 6/10 | ✅ Acceptable | State guards good, sensor validation weak |
| **Memory Safety** | 6/10 | ✅ Acceptable | Buffer overflow protected, String class risk |
| **Performance** | 8/10 | ✅ Good | Adequate for application, some inefficiencies |
| **Maintainability** | 4/10 | ⚠️ Poor | God objects and globals reduce maintainability |

---

## Detailed Findings

### 1. CRITICAL ISSUES (Flight Safety Impact)

#### 1.1 ✅ RESOLVED: Apogee Detection Counter Reset Bug

**Severity:** CRITICAL (Flight Safety)
**Status:** FIXED (February 14, 2026)
**File:** `src/flight_logic.cpp:904-966`

**Issue:**
Static variables in `detectApogee()` function were never reset when entering COAST state. This could cause false apogee detection on flight reuse without restart.

**Original Code (Problematic):**
```cpp
bool detectApogee() {
    static int baro_descending_count = 0;     // Never reset!
    static int accel_negative_count = 0;      // Never reset!
    static int gps_descending_count = 0;      // Never reset!
    static float maxGpsAltitude = 0.0f;       // Never reset!
    // ...
}
```

**Fix Applied:**
```cpp
// File-scope static variables (moved from function scope)
static int s_baro_descending_count = 0;
static int s_accel_negative_count = 0;
static int s_gps_descending_count = 0;
static float s_maxGpsAltitude = 0.0f;

void resetApogeeDetectionCounters() {
    s_baro_descending_count = 0;
    s_accel_negative_count = 0;
    s_gps_descending_count = 0;
    s_maxGpsAltitude = 0.0f;
}

// Called in COAST state entry (line ~547)
case COAST:
    resetApogeeDetectionCounters(); // Reset on state entry
    // ...
```

**Verification:**
- [ ] Test flight reuse without restart
- [ ] Verify counters reset on COAST entry
- [ ] Test multiple consecutive flights

---

#### 1.2 Missing Sensor Data Sanity Checks

**Severity:** HIGH (Flight Safety)
**Status:** NOT IMPLEMENTED
**Files:** `src/flight_logic.cpp`, all sensor drivers

**Issue:**
No validation of sensor outputs against physical limits or cross-sensor consistency checks.

**Examples of Missing Checks:**
```cpp
// GPS vs Barometer altitude cross-check
if (abs(GPS_altitude - baro_altitude) > 50.0) {
    // Sensor conflict - should trigger warning or ERROR state
}

// Accelerometer magnitude sanity check
if (accel_magnitude > 100.0) {
    // Physically impossible - should reject reading
}

// Gyro rate sanity check
if (abs(gyro_rate) > 2000.0) {
    // Beyond sensor limits - invalid data
}
```

**Recommendation:**
Create `validateSensorReadings()` function:
- Cross-check GPS vs barometer altitude (tolerance: ±50m)
- Validate accelerometer magnitude (range: 0-100g)
- Validate gyro rates (range: ±2000 dps)
- Check for stuck sensor values (no change over time)

**Priority:** HIGH
**Effort:** 2-4 hours
**Impact:** Prevents false triggers from sensor errors

---

#### 1.3 No Sensor Degradation Paths

**Severity:** HIGH (Reliability)
**Status:** NOT IMPLEMENTED
**File:** `src/flight_logic.cpp`

**Issue:**
Single sensor failure causes transition to ERROR state. No graceful degradation for partial failures.

**Current Behavior:**
```cpp
// In ProcessFlightState()
if (!isSensorSuiteHealthy(...)) {
    g_currentFlightState = ERROR;  // Immediate failure
}
```

**Recommended Behavior:**
```cpp
enum DegradedMode {
    NORMAL,
    NO_GPS,           // Continue with baro + IMU only
    NO_HIGH_G_ACCEL,  // Continue with ICM-20948 only
    BARO_ONLY         // Apogee detection via barometer + timer
};

if (!isSensorSuiteHealthy(...)) {
    if (canOperateInDegradedMode()) {
        enterDegradedMode();  // Continue with reduced functionality
    } else {
        g_currentFlightState = ERROR;  // Only if critical sensors fail
    }
}
```

**Priority:** HIGH
**Effort:** 1-2 weeks
**Impact:** Improves mission success rate, prevents unnecessary aborts

---

### 2. HIGH PRIORITY ISSUES (Code Quality & Architecture)

#### 2.1 Monolithic ProcessFlightState() Function

**Severity:** HIGH (Maintainability)
**File:** `src/flight_logic.cpp`
**Lines:** 1-782 (entire function)

**Issue:**
`ProcessFlightState()` is a **782-line God Function** with massive switch statement. Violates single responsibility principle.

**Impact:**
- Extremely difficult to test individual state logic
- High risk of logic bugs due to complexity
- Poor maintainability and readability
- Multiple state entry/exit handlers intertwined

**Cyclomatic Complexity:** Very High (estimated 50+)

**Recommended Refactoring:**

**State Pattern Implementation:**
```cpp
// state_handlers.h
class StateHandler {
public:
    virtual void onEntry(FlightContext& ctx) = 0;
    virtual void process(FlightContext& ctx) = 0;
    virtual void onExit(FlightContext& ctx) = 0;
};

// Individual handlers
class StartupHandler : public StateHandler { /* ~30-50 lines */ };
class CalibrationHandler : public StateHandler { /* ~30-50 lines */ };
class PadIdleHandler : public StateHandler { /* ~30-50 lines */ };
// ... 13 more handlers

// Refactored ProcessFlightState()
void ProcessFlightState() {
    StateHandler* handler = stateHandlerTable[g_currentFlightState];
    handler->process(g_flightContext);
}  // ~50 lines total
```

**Benefits:**
- Each state independently testable
- Reduced complexity (13 files of ~50 lines vs 1 file of 782 lines)
- Easier to maintain and extend
- Clear separation of concerns

**Priority:** HIGH
**Effort:** 8 hours
**ROI:** Dramatic improvement in testability and maintainability

---

#### 2.2 Excessive Global Variable Coupling

**Severity:** HIGH (Testability)
**File:** `src/flight_logic.cpp:20-50`
**Count:** 21 extern declarations

**Issue:**
Makes unit testing impossible. Hidden dependencies make code flow difficult to trace. State mutations from multiple functions hard to verify.

**Current Structure:**
```cpp
// In flight_logic.cpp
extern ErrorCode_t g_last_error_code;
extern FlightState g_currentFlightState;
extern float g_launchAltitude;
extern float g_maxAltitudeReached;
extern bool g_baroCalibrated;
extern MS5611 g_ms5611Sensor;
extern SFE_UBLOX_GNSS myGNSS;
// ... 14 more externs
```

**Recommended Refactoring:**

**FlightContext Struct:**
```cpp
// flight_context.h
struct FlightContext {
    // State
    FlightState currentState;
    ErrorCode_t lastError;

    // Altitude tracking
    float launchAltitude;
    float maxAltitude;
    float currentAltitude;
    float mainDeployAltitude;

    // Sensor references (via dependency injection)
    BarometerInterface* barometer;
    IMUInterface* imu;
    GPSInterface* gps;

    // Flags
    bool baroCalibrated;
    bool systemHealthy;

    // Timing
    unsigned long stateEntryTime;
    unsigned long lastSensorReadTime;
};

// Refactored function signatures
void ProcessFlightState(FlightContext& ctx);
bool detectApogee(const FlightContext& ctx);
bool detectLanding(const FlightContext& ctx);
```

**Benefits:**
- Reduces global coupling from 21 externs to 1 context reference
- Enables dependency injection for testing
- Makes data flow explicit
- Supports multiple flight controller instances (future)

**Priority:** HIGH
**Effort:** 6 hours
**Impact:** Enables unit testing, improves code clarity

---

#### 2.3 Arduino String Class Memory Risk

**Severity:** MEDIUM (Memory Safety)
**File:** `src/TripleT_Flight_Firmware.cpp:72-73`

**Issue:**
Arduino `String` class still used for logging despite high-frequency operations. Risk of heap fragmentation on long-duration flights.

**Current Code:**
```cpp
String g_FileDateString = "";
String g_LogDataString = "";
```

**Impact:**
- Heap fragmentation on long flights
- Allocation failures possible during critical flight phases
- Unpredictable performance during time-critical operations
- SRAM on Teensy 4.1: 512KB (sufficient but String class wastes it)

**Recommended Fix:**
```cpp
// Replace with fixed-size buffers
char g_FileDateString[32];  // e.g., "DATA_20260214_143052.csv"
char g_LogDataString[512];  // CSV line buffer

// Use snprintf for formatting
snprintf(g_FileDateString, sizeof(g_FileDateString),
         "DATA_%04d%02d%02d_%02d%02d%02d.csv",
         year, month, day, hour, minute, second);

snprintf(g_LogDataString, sizeof(g_LogDataString),
         "%lu,%.2f,%.2f,%.2f...",
         timestamp, altitude, velocity, ...);
```

**Priority:** MEDIUM
**Effort:** 3 hours
**Impact:** Eliminates heap fragmentation risk, predictable memory usage

---

#### 2.4 Blocking Delays in Command Processing

**Severity:** MEDIUM (Responsiveness)
**File:** `src/command_processor.cpp`
**Lines:** 184, 189, 278, 281

**Issue:**
Functions called from main loop contain blocking delays.

**Problematic Code:**
```cpp
// In performCalibration()
digitalWrite(NEOPIXEL_PIN, HIGH);
delay(1000);  // BLOCKING! (line 184)
digitalWrite(NEOPIXEL_PIN, LOW);
delay(1000);  // BLOCKING! (line 189)

// In system shutdown
delay(5000);  // BLOCKING! (line 281)
```

**Mitigation:**
- 1000ms delays in CALIBRATION state (acceptable - not time-critical)
- Shutdown delay problematic if entered during flight

**Recommended Fix:**
```cpp
// Non-blocking LED feedback
static unsigned long ledToggleTime = 0;
static bool ledState = false;

if (millis() - ledToggleTime > 1000) {
    ledState = !ledState;
    digitalWrite(NEOPIXEL_PIN, ledState);
    ledToggleTime = millis();
}
```

**Priority:** MEDIUM
**Effort:** 1 hour
**Impact:** Improved responsiveness, prevents loop blocking

---

### 3. MEDIUM PRIORITY ISSUES (Polish & Efficiency)

#### 3.1 No Safety Monitoring Enforcement

**Severity:** MEDIUM
**File:** `src/guidance_control.cpp`
**Lines:** 400-500 (approx)

**Issue:**
`guidance_check_stability()` only warns; doesn't trigger recovery action.

**Current Behavior:**
```cpp
if (guidance_is_stability_compromised()) {
    Serial.println(F("WARNING: Stability compromised"));
    // No action taken - just a warning!
}
```

**Recommended Behavior:**
```cpp
if (guidance_is_stability_compromised()) {
    Serial.println(F("CRITICAL: Guidance stability compromised!"));
    g_last_error_code = GUIDANCE_STABILITY_FAIL;
    g_currentFlightState = ERROR;  // Trigger recovery action
}
```

**Note:** Current v0.51 code DOES trigger ERROR in BOOST and COAST states (lines 531-536, 558-563). This issue was partially addressed. Verify all states have this protection.

**Priority:** MEDIUM
**Effort:** 2 hours
**Impact:** Prevents unstable flight from runaway guidance

---

#### 3.2 Incomplete Error Code Coverage

**Severity:** MEDIUM
**File:** `src/error_codes.h`

**Issue:**
Missing error codes for several failure scenarios.

**Current Codes:** 20+ defined error codes

**Missing Codes:**
```cpp
// Recommended additions
SERVO_ACTUATION_FAILURE = 95,     // Servo command failed
GUIDANCE_INIT_FAILURE = 96,       // Guidance system init failed
LOW_BATTERY_WARNING = 54,         // Battery voltage low (defined but not used)
KALMAN_CONVERGENCE_TIMEOUT = 97,  // Orientation filter not converging
TELEMETRY_TX_FAILURE = 98,        // Telemetry transmission failed
SD_WRITE_TIMEOUT = 55,            // SD card write timeout (vs full failure)
EEPROM_WRITE_FAILURE = 85,        // EEPROM write verification failed
```

**Priority:** MEDIUM
**Effort:** 1 hour
**Impact:** Better post-flight diagnostics

---

#### 3.3 Naming Convention Inconsistency

**Severity:** LOW (Readability)
**Files:** Throughout codebase

**Issue:**
Mixed naming conventions reduce readability.

**Examples:**
```cpp
// Good: Consistent g_ prefix for globals
float g_launchAltitude;
float g_maxAltitudeReached;

// Good: Consistent _ms suffix for milliseconds
#define BACKUP_APOGEE_TIME_MS 20000

// Inconsistent: Mixed camelCase and snake_case
unsigned long firstLandedTime;          // camelCase
float max_pitch_rate_current_state_rps; // snake_case
```

**Recommendation:**
- **Global variables**: `g_camelCase`
- **Constants**: `UPPER_SNAKE_CASE`
- **Functions**: `camelCase`
- **Local variables**: `camelCase`
- **File-scope statics**: `s_camelCase`

**Priority:** LOW
**Effort:** 3 hours (audit and fix)
**Impact:** Improved code readability

---

### 4. ARCHITECTURE & DESIGN ISSUES

#### 4.1 Violation of Separation of Concerns

**Current Structure:**
```
TripleT_Flight_Firmware.cpp (main) - 1060 lines
  ├─ Sensor reading
  ├─ State machine logic (ProcessFlightState)
  ├─ Data logging
  ├─ Serial command processing
  ├─ Kalman filtering
  └─ Guidance control
```

**Issues:**
- Main file tightly coupled to all subsystems
- No clear interface boundaries
- Hard to test individual components

**Recommended Architecture:**
```
FlightController (orchestrator, ~200 lines)
  ├─ HAL (Hardware Abstraction Layer) ← NEW
  │   ├─ Interfaces (I2C, GPIO, Timer, Serial, Storage)
  │   ├─ Teensy implementations (real hardware)
  │   └─ Mock implementations (testing)
  ├─ SensorManager (ICM, KX134, MS5611, GPS)
  ├─ StateManager (State Pattern handlers)
  ├─ DataLogger (logging logic)
  ├─ CommandProcessor (serial interface)
  ├─ OrientationFilter (Kalman)
  └─ GuidanceController (PID + servo control)
```

**Benefits:**
- Clear module boundaries
- Testable without hardware (via HAL mocks)
- Easier to maintain and extend
- Supports parallel development

**Priority:** HIGH (long-term)
**Effort:** 10-12 weeks
**Impact:** Transforms codebase quality, enables testing

---

#### 4.2 Hardware Coupling Prevents Testing

**Issue:**
891+ direct hardware calls create tight coupling.

**Examples:**
```cpp
Wire.begin();                    // I2C hardware
digitalWrite(PYRO_CHANNEL_1, HIGH);  // GPIO hardware
millis();                        // Timing hardware
Serial.println();                // UART hardware
g_SD.write(...);                 // SD card hardware
```

**Impact:**
- Cannot run unit tests without Teensy 4.1 hardware
- Cannot simulate flight scenarios
- Cannot test state machine logic in isolation
- High regression risk when modifying code

**Solution:** Hardware Abstraction Layer (HAL)
See Section 4.1 recommended architecture.

**Priority:** HIGH (long-term)
**Effort:** 10-12 weeks (part of full refactor)
**Impact:** Enables comprehensive testing

---

### 5. PERFORMANCE & RESOURCE USAGE

#### 5.1 Loop Timing Not Deterministic

**Issue:**
Kalman filter and control loops run at unpredictable `dt`.

**Current Code:**
```cpp
// In TripleT_Flight_Firmware.cpp:933-934
if (dt_kalman > 0.0f && dt_kalman < 1.0f) { // "Basic sanity check"
    kalman_predict(dt_kalman);
}
```

**Problem:**
- `dt_kalman` varies based on loop execution time
- SD card writes cause jitter
- Sensor reads are non-deterministic
- No guarantee of consistent control loop frequency

**Recommendation:**
```cpp
// Fixed timestep control loop
const unsigned long CONTROL_LOOP_INTERVAL_MS = 10;  // 100Hz
static unsigned long lastControlLoopTime = 0;

if (millis() - lastControlLoopTime >= CONTROL_LOOP_INTERVAL_MS) {
    const float dt = CONTROL_LOOP_INTERVAL_MS / 1000.0f;  // Fixed dt
    kalman_predict(dt);
    guidance_update(dt);
    lastControlLoopTime = millis();
}
```

**Priority:** MEDIUM
**Effort:** 2 hours
**Impact:** Predictable control behavior, better stability

---

#### 5.2 Excessive Data Logging Overhead

**Issue:**
Logging 62+ fields at 10Hz with String-based formatting.

**Current Code:**
```cpp
String g_LogDataString = "";
// CSV string constructed character-by-character using dtostrf()
```

**Performance Impact:**
- Repeated memory allocations (String class)
- Character-by-character concatenation
- ~350+ bytes per LogData struct
- 10Hz = 3.5KB/sec logging rate

**Recommendation:**
1. Replace String with fixed buffer (already mentioned in 2.3)
2. Consider binary logging for high-rate data:
   ```cpp
   // Binary format: much faster, smaller files
   fwrite(&logData, sizeof(LogData), 1, logFile);
   ```
3. Implement selective logging:
   ```cpp
   // Log only changed fields or at variable rates
   if (g_currentFlightState == BOOST || g_currentFlightState == COAST) {
       logRate = 100Hz;  // High rate during critical phases
   } else {
       logRate = 10Hz;   // Lower rate during non-critical phases
   }
   ```

**Priority:** LOW
**Effort:** 4 hours
**Impact:** Reduced SD card wear, faster logging, smaller files

---

### 6. TESTING INFRASTRUCTURE

#### 6.1 Virtually No Automated Tests

**Severity:** CRITICAL
**Current Coverage:** <5%

**Existing Tests:**
- `test/test_flight_logic.cpp`: Partial apogee detection test (incomplete)
- `test/test_core_logic/test_main.cpp`: Kalman filter init test (incomplete)

**Missing Tests:**
- State machine transitions (0 tests)
- Landing detection (0 tests)
- Pyro firing logic (0 tests)
- Command processor (0 tests)
- Sensor fusion (0 tests)
- Guidance control (0 tests)
- Data logging (0 tests)

**Recommendation:**
Implement comprehensive test suite (see `PROJECT_PLAN_2026.md` Phase 3):
- **Target**: 137+ unit tests
- **Coverage**: 60%+ overall, 85%+ safety-critical
- **Framework**: Unity + HAL mocks
- **CI/CD**: GitHub Actions automation

**Priority:** CRITICAL
**Effort:** 14 weeks (see project plan)
**Impact:** Dramatically reduced regression risk, production quality

---

#### 6.2 No CI/CD Pipeline

**Severity:** HIGH
**Status:** NOT IMPLEMENTED

**Current Process:**
- Manual compilation
- No automated build verification
- No pre-merge testing
- Syntax errors only caught at upload time

**Recommendation:**
Create GitHub Actions workflow:

```yaml
# .github/workflows/build.yml
name: Build & Test
on: [push, pull_request]
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Setup PlatformIO
        run: pip install platformio
      - name: Build Firmware
        run: pio run -e teensy41
      - name: Run Unit Tests
        run: pio test -e native
      - name: Generate Coverage
        run: pio test -e native --with-coverage
      - name: Upload Coverage
        uses: codecov/codecov-action@v3
```

**Priority:** HIGH
**Effort:** 8 hours
**Impact:** Automated quality checks, prevents regressions

---

## Prioritized Action Plan

### Week 1: Critical Fixes

1. ✅ **Fix Apogee Counter Reset Bug** (15 min) - COMPLETED
2. **Add Sensor Sanity Checks** (2 hours)
3. **Create First Unit Test** (2 hours)

### Weeks 2-5: Testing Infrastructure

4. **Implement HAL Interfaces** (40 hours)
5. **Create Mock Implementations** (20 hours)
6. **Set up CI/CD Pipeline** (8 hours)

### Weeks 6-9: Refactoring

7. **Refactor ProcessFlightState to State Pattern** (8 hours)
8. **Create FlightContext Struct** (6 hours)
9. **Eliminate String Class Usage** (3 hours)

### Weeks 10-14: Testing

10. **Implement Safety-Critical Tests** (25 hours)
11. **Implement State Machine Tests** (20 hours)
12. **Implement Integration Tests** (15 hours)

---

## Conclusions & Recommendations

### Current State Assessment

The TripleT Flight Firmware v0.51 is **functionally complete and flight-worthy for hobby/personal use**. All critical safety bugs from previous reviews have been resolved. The newly identified apogee counter reset bug has been fixed.

**Strengths:**
- ✅ Critical safety features operational
- ✅ Multi-method redundancy in apogee detection
- ✅ Non-blocking real-time operations
- ✅ Comprehensive error handling framework
- ✅ State persistence for power-loss recovery

**Weaknesses:**
- ❌ Virtually no automated testing (<5% coverage)
- ❌ Monolithic functions (ProcessFlightState: 782 lines)
- ❌ Excessive global coupling (21 externs)
- ❌ No hardware abstraction (891+ direct hardware calls)
- ❌ String class memory risk

### Path to Production Quality

To achieve **competition or commercial quality**, implement:

1. **Testing Infrastructure** (14 weeks)
   - HAL for hardware abstraction
   - 137+ unit tests
   - 60%+ code coverage
   - CI/CD automation

2. **Architecture Refactoring** (4 weeks)
   - State Pattern for ProcessFlightState()
   - FlightContext struct to eliminate globals
   - String class elimination

3. **Robustness Improvements** (2 weeks)
   - Sensor sanity checks
   - Degraded mode operation
   - Comprehensive error codes

**Total Effort:** ~24 weeks (6 months) solo developer
**Alternative:** 12 weeks with 2-3 person team

**Recommended Approach:** Implement critical fixes immediately (Week 1), then proceed with testing infrastructure (Weeks 2-14) before major refactoring.

---

## Appendices

### A. File Size Metrics

| File | Lines | Complexity | Status |
|------|-------|-----------|--------|
| `flight_logic.cpp` | 1020 | Very High | Needs refactoring |
| `TripleT_Flight_Firmware.cpp` | 1060 | High | Needs refactoring |
| `guidance_control.cpp` | 686 | Medium | Acceptable |
| `command_processor.cpp` | 592 | Medium | Acceptable |
| `icm_20948_functions.cpp` | 517 | Low | Good |
| `ms5611_functions.cpp` | 325 | Low | Good |
| `utility_functions.cpp` | 562 | Low | Good |

### B. Global Variable Inventory

**Count:** 21 externs in flight_logic.cpp

**Categories:**
- Flight state: 4 variables
- Altitude tracking: 5 variables
- Sensor objects: 6 variables
- Flags: 4 variables
- Error tracking: 2 variables

### C. Hardware Dependencies

**Total Hardware Calls:** 891+ throughout codebase

**Categories:**
- Serial I/O: 200+ calls
- I2C (Wire): 150+ calls
- GPIO (digitalWrite, pinMode): 100+ calls
- Timing (millis, delay): 108+ calls
- SD Card: 50+ calls
- EEPROM: 10+ calls
- PWM (Servo): 20+ calls

### D. References

- **Previous Review**: `FIRMWARE_REVIEW_REPORT.md` (pre-v0.51)
- **Previous Review**: `COMPREHENSIVE_REVIEW.md` (v0.51, December 2025)
- **Gap Analysis**: `UPDATED_GAP_ANALYSIS_2025.md`
- **Project Plan**: `PROJECT_PLAN_2026.md`

---

**End of Code Review Findings**
**Next Review Recommended:** After Phase 3 (Testing Infrastructure) completion
