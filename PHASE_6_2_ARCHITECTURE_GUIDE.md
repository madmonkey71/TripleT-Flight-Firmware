# Phase 6.2: Advanced Guidance Control - Architecture Guide

**Quick Reference for Developers**
**Last Updated:** February 16, 2026

---

## Quick Navigation

- **Implementation Plan**: `PHASE_6_2_IMPLEMENTATION_PLAN.md` - Full details, step-by-step
- **This Document**: Architecture overview, diagrams, code examples
- **Config Reference**: `src/config.h` - Phase 6.2 parameters (see lines ~250+)
- **Source Files**: `src/stability_monitor.*`, `src/servo_smoother.*`, `src/guidance_failsafe.cpp`

---

## Architecture Overview

### Three-Layer Stability & Control System

```
┌─────────────────────────────────────────────────────────┐
│         MAIN FLIGHT LOOP (100 Hz nominal)              │
│         guidance_update() + failsafe_check()           │
└────────────────────┬────────────────────────────────────┘
                     │
        ┌────────────┴────────────┐
        │                         │
        ▼                         ▼
   ┌─────────────┐          ┌──────────────┐
   │   Kalman    │          │ PID Control  │
   │   Filter    │          │ (Phase 4)    │
   │             │          │              │
   │ Quaternion  │          │ Actuator     │
   │ + Gyro data │          │ commands     │
   └──────┬──────┘          └───┬──────────┘
          │                     │
          │         ┌───────────┘
          │         │
          ▼         ▼
    ┌──────────────────────────────┐
    │  STABILITY MONITOR (NEW)     │  ◄── LAYER 1: Detection
    │  ────────────────────────    │
    │  • Angular rates (gyro)      │
    │  • Attitude errors (euler)   │
    │  • Saturation detection      │
    │  • Violation timing          │
    │                              │
    │  Output: StabilityMetrics    │
    │  {is_stable, violation_type} │
    └──────────┬───────────────────┘
               │
               ▼
    ┌──────────────────────────────┐
    │  FAILSAFE LOGIC (NEW)        │  ◄── LAYER 2: Response
    │  ────────────────────────    │
    │  Level 1: Reduce PID gains   │
    │  Level 2: Passive mode       │
    │  Level 3: ERROR state        │
    │                              │
    │  Output: gain_factor,        │
    │           passive_mode_flag  │
    └──────────┬───────────────────┘
               │
               ▼
    ┌──────────────────────────────┐
    │  SERVO SMOOTHER (NEW)        │  ◄── LAYER 3: Execution
    │  ────────────────────────    │
    │  • Rate limiting             │
    │  • Deadband filtering        │
    │  • Low-pass smoothing        │
    │                              │
    │  Input: PID commands         │
    │  Output: Servo angles        │
    └──────────┬───────────────────┘
               │
               ▼
          ┌─────────┐
          │ Servos  │
          │ (PWM)   │
          └─────────┘
```

---

## Data Flow Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                      SENSOR INPUTS                               │
│  IMU (Quaternion), Gyroscope, Actuator Commands                  │
└─────────────────────┬────────────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────┐
        │   StabilityMonitor::update()    │
        │   (Convert to Euler, check      │
        │    thresholds, track timing)    │
        └────────────┬────────────────────┘
                     │
                     ▼
        StabilityMetrics {
          roll_rate_dps: 185.5,           ◄─── Exceeds 180 limit
          pitch_rate_dps: 45.2,
          yaw_rate_dps: 120.0,
          roll_error_deg: 15.2,
          pitch_error_deg: 8.5,
          yaw_error_deg: 3.2,
          actuator_saturation_percent: 42.3,
          is_stable: false,               ◄─── VIOLATION!
          violation_type: "RATE",
          violation_bitmask: 0x01
        }
                     │
        ┌────────────┴────────────────────┐
        │                                 │
        ▼                                 ▼
   ┌──────────────────┐          ┌─────────────────────┐
   │   Stability OK   │          │ Stability Problem   │
   │   (is_stable =   │          │ (is_stable = false) │
   │     true)        │          │                     │
   │                  │          │ Start violation     │
   │ Do nothing       │          │ timer               │
   │ Continue normal  │          │                     │
   │ operation        │          └────────┬────────────┘
   │                  │                   │
   └────────┬─────────┘          ┌────────┴──────────┐
            │                    │                   │
            │          Check timer duration         │
            │                    │                   │
            │          ┌─────────┴──────────┐        │
            │          │                    │        │
            │    <500ms │              ≥500ms        │
            │    no action           (PERSISTENT)    │
            │          │                    │        │
            │          │                    ▼
            │          │         ┌─────────────────────────────┐
            │          │         │ guidance_failsafe_check()   │
            │          │         │                             │
            │          │         │ Time  │ Action              │
            │          │         │ ────────────────────        │
            │          │         │ 0-1s  │ Reduce gains 10%    │
            │          │         │ 1-2s  │ Enter passive mode  │
            │          │         │ >2s   │ ERROR state         │
            │          │         │       │                     │
            │          │         │ gain_factor ← 0.9           │
            │          │         │ passive_mode_flag ← 1       │
            │          │         └──────────┬──────────────────┘
            │          │                    │
            │          └────────┬───────────┘
            │                   │
            ▼                   ▼
        ┌──────────────────────────────────┐
        │  Apply Failsafe Gain Factor      │
        │  PID_output *= gain_factor       │
        │  (default 1.0, reduced to 0.3)   │
        └────────────┬─────────────────────┘
                     │
        ┌────────────┴────────────────────┐
        │                                 │
        ▼                                 ▼
    ┌─────────────────────┐      ┌──────────────────────────┐
    │ Normal Mode         │      │ Passive Mode             │
    │ (passive_mode=0)    │      │ (passive_mode=1)         │
    │                     │      │                          │
    │ PID outputs valid   │      │ Center servos            │
    │ Pass to smoother    │      │ No guidance control      │
    └────────────┬────────┘      └──────────┬───────────────┘
                 │                          │
                 └──────────────┬───────────┘
                                │
                ┌───────────────┴───────────────┐
                │                               │
                ▼                               ▼
        ┌─────────────────────────┐     ┌──────────────────┐
        │ ServoSmoother::smooth() │     │ Centered Outputs │
        │ for each axis:          │     │ (90° each)       │
        │                         │     │                  │
        │ 1. Rate limiting        │     └────────┬─────────┘
        │    max 10°/100ms        │              │
        │                         │              │
        │ 2. Deadband             │              │
        │    ignore <0.5°         │              │
        │                         │              │
        │ 3. Low-pass filter      │              │
        │    2 Hz cutoff          │              │
        │                         │              │
        │ Output: smooth angles   │              │
        └────────────┬────────────┘              │
                     │                           │
                     └──────────┬────────────────┘
                                │
                ┌───────────────┴───────────────┐
                │                               │
                ▼                               ▼
        ┌──────────────────────┐      ┌─────────────────────┐
        │ PWMServo::write()    │      │ PWMServo::write(90) │
        │ (Smooth commands)    │      │ (Centered)          │
        └─────────────┬────────┘      └────────────┬────────┘
                      │                            │
                      └─────────────┬──────────────┘
                                    │
                                    ▼
                            ┌─────────────────┐
                            │   Servos Move   │
                            │   (Physical)    │
                            └─────────────────┘
```

---

## Class Hierarchy

```
┌─────────────────────────────────────────────────────┐
│         StabilityMonitor                            │
├─────────────────────────────────────────────────────┤
│ Public:                                             │
│  + StabilityMonitor()                               │
│  + init()                                           │
│  + update(q_curr, q_desired, gyro, cmd...)          │
│  + getMetrics() -> StabilityMetrics                 │
│  + isStabilityViolation() -> bool                   │
│  + isViolationPersistent() -> bool                  │
│  + getViolationDuration() -> uint32_t               │
│  + resetViolation()                                 │
│  + printDiagnostics()                               │
├─────────────────────────────────────────────────────┤
│ Private:                                            │
│  - current_metrics: StabilityMetrics                │
│  - violation_start_time_ms: uint32_t                │
│  - in_violation: bool                               │
│  - thresholds: {rates, errors, saturation, ...}     │
│  - quaternion_to_euler(...)                         │
│  - check_angular_rates(...)                         │
│  - check_attitude_error(...)                        │
│  - check_actuator_saturation(...)                   │
└─────────────────────────────────────────────────────┘
         ▲
         │ Contains
         │
    ┌────┴──────────────────────────────────────────┐
    │         StabilityMetrics (struct)             │
    ├───────────────────────────────────────────────┤
    │  roll_rate_dps: float                         │
    │  pitch_rate_dps: float                        │
    │  yaw_rate_dps: float                          │
    │  roll_error_deg: float                        │
    │  pitch_error_deg: float                       │
    │  yaw_error_deg: float                         │
    │  actuator_saturation_percent: float           │
    │  pitch_saturation_percent: float              │
    │  roll_saturation_percent: float               │
    │  yaw_saturation_percent: float                │
    │  is_stable: bool                              │
    │  violation_type: const char*                  │
    │  violation_bitmask: uint8_t                   │
    └───────────────────────────────────────────────┘


┌─────────────────────────────────────────────────────┐
│         ServoSmoother                               │
├─────────────────────────────────────────────────────┤
│ Public:                                             │
│  + ServoSmoother()                                  │
│  + init(FilterType mode)                            │
│  + smooth(desired, current, dt_ms, axis) -> float   │
│  + smoothBatch(desired[], current[], dt, output[])  │
│  + reset()                                          │
│  + getLastOutput(axis_id) -> float                  │
├─────────────────────────────────────────────────────┤
│ Private:                                            │
│  - active_filter: FilterType                        │
│  - last_output[3]: float                            │
│  - lowpass.prev_output[3]: float                    │
│  - lowpass.alpha: float                             │
│  - rate_limit.max_rate_per_ms[3]: float             │
│  - deadband.deadband_deg: float                     │
│  - apply_rate_limit(...)                            │
│  - apply_deadband(...)                              │
│  - apply_lowpass(...)                               │
│  - clamp_angle(...)                                 │
└─────────────────────────────────────────────────────┘
         ▲
         │ Uses
         │
    ┌────┴──────────────────────────────────────────┐
    │    FilterType Enumeration                     │
    ├───────────────────────────────────────────────┤
    │  0: RATE_LIMIT_ONLY                           │
    │  1: DEADBAND_ONLY                             │
    │  2: LOWPASS_ONLY                              │
    │  3: RATE_LIMIT_THEN_LOWPASS                   │
    │  4: FULL_FILTERING (all three)                │
    └───────────────────────────────────────────────┘


Failsafe Functions (in guidance_failsafe.cpp):
├─ guidance_failsafe_check(current_time_ms) -> bool
├─ guidance_failsafe_reset()
├─ guidance_get_failsafe_gain_factor() -> float
├─ guidance_is_passive_mode_active() -> bool
└─ log_guidance_event(message)

Global State:
├─ g_stability_monitor: StabilityMonitor instance
├─ g_servo_smoother: ServoSmoother instance
└─ failsafe_state: struct with gain/mode tracking
```

---

## Configuration & Tuning

### Default Configuration (from config.h)

```cpp
// STABILITY THRESHOLDS
GUIDANCE_STABILITY_ROLL_RATE_LIMIT_DPS = 180.0f     // deg/s
GUIDANCE_STABILITY_PITCH_RATE_LIMIT_DPS = 180.0f
GUIDANCE_STABILITY_YAW_RATE_LIMIT_DPS = 360.0f     // Higher for yaw

GUIDANCE_STABILITY_ROLL_ERROR_LIMIT_DEG = 30.0f    // degrees
GUIDANCE_STABILITY_PITCH_ERROR_LIMIT_DEG = 20.0f   // More restrictive
GUIDANCE_STABILITY_YAW_ERROR_LIMIT_DEG = 20.0f

GUIDANCE_STABILITY_SATURATION_LIMIT_PERCENT = 95.0f
GUIDANCE_STABILITY_VIOLATION_DURATION_MS = 500      // 500ms persistence check

// FAILSAFE TIMINGS
GUIDANCE_FAILSAFE_LEVEL1_MS = 1000                  // Start gain reduction
GUIDANCE_FAILSAFE_LEVEL2_MS = 2000                  // Enter passive mode
GUIDANCE_FAILSAFE_LEVEL3_MS = 5000                  // Enter ERROR state
GUIDANCE_FAILSAFE_MIN_GAIN = 0.3f                   // Minimum 30% gain

// SERVO SMOOTHER
SERVO_SMOOTHER_ENABLED = 1
SERVO_SMOOTHER_TYPE = 4                             // FULL_FILTERING
SERVO_RATE_LIMIT_DPS = 10.0f                        // 10°/100ms
SERVO_DEADBAND_DEG = 0.5f                           // Ignore <0.5°
SERVO_LOWPASS_CUTOFF_HZ = 2.0f                      // 2 Hz filter
```

### Tuning Guidance

**If overshooting during guidance:**
1. Reduce `SERVO_RATE_LIMIT_DPS` (slower response)
2. Reduce `SERVO_LOWPASS_CUTOFF_HZ` (more smoothing)
3. Reduce PID gains `PID_*_KP`, `PID_*_KI`

**If not responding fast enough:**
1. Increase `SERVO_RATE_LIMIT_DPS` (up to 20°/100ms)
2. Increase `SERVO_LOWPASS_CUTOFF_HZ` (up to 5 Hz)
3. Increase PID gains gradually

**If stability violations triggered falsely:**
1. Increase rate limits (more margin)
2. Increase error limits
3. Increase `GUIDANCE_STABILITY_VIOLATION_DURATION_MS` to 1000ms

**If failsafe too aggressive:**
1. Increase `GUIDANCE_FAILSAFE_LEVEL*_MS` timings (later trigger)
2. Increase `GUIDANCE_FAILSAFE_MIN_GAIN` (don't reduce as much)
3. Reduce `GUIDANCE_STABILITY_SATURATION_LIMIT_PERCENT`

---

## Integration Checklist

### Before Merging Phase 6.2

**Code Quality:**
- [ ] All `.cpp` files compile without warnings
- [ ] All `.h` files have include guards
- [ ] Comments in all public methods
- [ ] No global state except through HAL and controller instances

**Testing:**
- [ ] Unit tests pass locally: `pio test -e native_test`
- [ ] Teensy build successful: `pio run -e teensy41`
- [ ] Upload to hardware and test manually

**Integration:**
- [ ] `guidance_init()` calls `g_stability_monitor.init()`
- [ ] `guidance_update()` calls `g_servo_smoother.smooth()` before servo writes
- [ ] `guidance_check_stability()` calls `g_stability_monitor.update()`
- [ ] Main loop calls `guidance_failsafe_check()` at 10 Hz
- [ ] LogData struct updated with stability fields
- [ ] CSV headers include new fields

**Configuration:**
- [ ] New parameters added to `config.h` with defaults
- [ ] Parameters match expected ranges (rates in DPS, angles in DEG)
- [ ] Thresholds conservative (won't trigger on normal flight)

**Documentation:**
- [ ] README comment in all new functions
- [ ] Inline comments for complex algorithms
- [ ] Config parameter comments explaining tuning

---

## Debugging & Diagnostics

### Serial Commands (Phase 6.2)

```
# Check current stability status
> stability_status

Sample output:
Roll Rate: 45.2 DPS (limit: 180)
Pitch Rate: 12.3 DPS (limit: 180)
Yaw Rate: 8.5 DPS (limit: 360)
Roll Error: 8.2 deg (limit: 30)
Pitch Error: 5.1 deg (limit: 20)
Yaw Error: 2.3 deg (limit: 20)
Saturation: 42.3% (limit: 95%)
Status: STABLE
Failsafe: OFF
```

### CSV Analysis Post-Flight

```
# Extract stability metrics from flight log
stability_roll_rate_dps, stability_pitch_rate_dps, ...

# Find violation events
grep -E "stability_roll_rate_dps.*18[0-9]|19[0-9]|2[0-9][0-9]" flight.csv

# Check failsafe triggering
grep -E "failsafe_status.*[123]" flight.csv

# Plot stability metrics
# (Use Python/MATLAB with pandas)
import pandas as pd
df = pd.read_csv('flight.csv')
df.plot(x='timestamp', y=['stability_roll_rate_dps', 'stability_pitch_rate_dps'])
```

### Common Issues & Solutions

**Problem: Stability violations on every flight**
- [ ] Check quaternion data is valid (not NaN)
- [ ] Verify gyro calibration (bias not drifting)
- [ ] Reduce initial PID gains (start conservative)
- [ ] Increase violation duration threshold

**Problem: Servo chattering (oscillating)**
- [ ] Reduce `SERVO_LOWPASS_CUTOFF_HZ` (increase smoothing)
- [ ] Increase `SERVO_DEADBAND_DEG` (suppress small commands)
- [ ] Reduce `SERVO_RATE_LIMIT_DPS` (slower response)
- [ ] Reduce `PID_*_KD` (less derivative gain)

**Problem: Slow servo response**
- [ ] Increase `SERVO_RATE_LIMIT_DPS`
- [ ] Increase `SERVO_LOWPASS_CUTOFF_HZ`
- [ ] Increase PID gains `Kp`, `Ki`

**Problem: Failsafe triggering inappropriately**
- [ ] Increase rate/error limits
- [ ] Increase violation duration
- [ ] Check sensor calibration (IMU stable?)
- [ ] Reduce PID gains to reduce overshoot

---

## Performance Monitoring

### RAM Usage

```
StabilityMonitor:  ~220 bytes
ServoSmoother:     ~124 bytes
Failsafe state:    ~24 bytes
──────────────────────────────
Total:             ~368 bytes

Teensy 4.1: 960 KB available
Usage:      0.04% for Phase 6.2
```

### CPU Timing

```
Per-Update Cycle (10 Hz):
  StabilityMonitor::update()  ~0.35 ms
  ServoSmoother::smooth() x3  ~0.25 ms
  Failsafe check()            ~0.1 ms
  ─────────────────────────────────
  Total:                       ~0.7 ms

Main loop budget: 10 ms (100 Hz)
Phase 6.2 usage:  0.7 ms (7%)
Headroom:         9.3 ms (93%)
```

### Flash Usage

```
New code:
  StabilityMonitor:  ~8 KB
  ServoSmoother:     ~6 KB
  Failsafe logic:    ~3 KB
  ─────────────────────────────
  Total:             ~17 KB

Teensy 4.1: 384 KB available
Currently: ~145 KB (38%)
After:     ~162 KB (42%)
Remaining: ~220 KB (58%)
```

---

## Testing Quick Start

### Run Desktop Tests

```bash
# Build and run all unit tests
pio test -e native_test

# Run specific test file
pio test -e native_test -f test_stability_monitor

# Run with verbose output
pio test -e native_test -v
```

### Upload to Teensy

```bash
# Build for Teensy 4.1
pio run -e teensy41

# Upload
pio run -e teensy41 -t upload

# Monitor serial output
pio device monitor --baud 115200
```

### Hardware Testing Script

```bash
#!/bin/bash
# test_phase_6_2.sh

echo "=== Phase 6.2 Testing ==="

# 1. Desktop unit tests
echo "Running desktop tests..."
pio test -e native_test || exit 1

# 2. Teensy build
echo "Building for Teensy..."
pio run -e teensy41 || exit 1

# 3. Upload (if device detected)
if lsusb | grep -q "Teensy"; then
  echo "Uploading firmware..."
  pio run -e teensy41 -t upload || exit 1
  sleep 2
fi

# 4. Serial test (30 seconds)
echo "Testing serial commands..."
timeout 30 pio device monitor --baud 115200 << EOF || true
status_sensors
stability_status
EOF

echo "=== All tests completed ==="
```

---

## Code Review Checklist

**For Pull Requests including Phase 6.2:**

- [ ] All files follow existing code style (spacing, naming)
- [ ] No `std::` containers used (Arduino compatibility)
- [ ] No dynamic allocation (`new`/`delete`) outside constructors
- [ ] All math operations use `float` consistently
- [ ] Trigonometric functions use radians internally
- [ ] Comments explain "why", not "what"
- [ ] No compiler warnings on either platform
- [ ] Test coverage >80% for new code
- [ ] Integration with existing systems verified
- [ ] Performance impact measured and acceptable
- [ ] Configuration parameters documented

---

## Troubleshooting Compilation

**If `StabilityMonitor` won't compile:**

```cpp
// Check includes in guidance_control.cpp:
#include "stability_monitor.h"       // ✓ Include header
#include "data_structures.h"         // ✓ For LogData

// Check quaternion globals available:
extern float g_q0, g_q1, g_q2, g_q3;  // ✓ From Kalman filter
```

**If `ServoSmoother` fails linking:**

```cpp
// Ensure implementation in .cpp has external linkage
// Not in header as `static` function

// In servo_smoother.cpp:
float ServoSmoother::apply_rate_limit(...) {  // ✓ Member function
  // ...
}
```

**If `guidance_failsafe_check` missing:**

```cpp
// In guidance_control.cpp, add:
#include "guidance_failsafe.cpp"     // ✓ Include implementation

// Or in platformio.ini, ensure both .cpp files compiled:
src_filter = ["+<*>", "-<.git/>" ... ]
```

---

## Next Steps

After Phase 6.2 implementation:

1. **Phase 6.3:** Production Readiness (power optimization, edge cases)
2. **Phase 6.4:** Validation Suite (regression tests, system verification)
3. **Phase 6.5:** Cloud Integration (optional telemetry)
4. **Phase 6.6:** Release Preparation (docs, videos, final testing)

---

**Document Version:** 1.0
**Created:** February 16, 2026
**For:** TripleT Flight Firmware v1.0.0 (Phase 6.2)
