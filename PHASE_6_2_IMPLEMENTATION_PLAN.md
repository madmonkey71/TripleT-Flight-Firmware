# Phase 6.2: Advanced Guidance Control - Implementation Plan

**Status:** Ready for Implementation
**Target Release:** v1.0.0-RC1
**Estimated Duration:** 1 week (5-7 days)
**Code Scope:** 1,500 lines
**Last Updated:** February 16, 2026

---

## Executive Summary

Phase 6.2 implements advanced guidance control features that enhance stability, reliability, and safety of the servo control system. The phase is structured in 5 sequential steps, each building on prior work:

1. **StabilityMonitor** - Detect angular rate, attitude error, and actuator saturation violations
2. **Integration** - Connect stability monitoring to existing `guidance_check_stability()` in Phase 4
3. **ServoSmoother** - Add rate limiting, deadband, and filtering to servo commands
4. **Failsafe** - Implement automatic gain reduction and passive mode on violations
5. **Logging** - Stream stability metrics to CSV for analysis

**Key Dependency:** Phase 6.2 depends on Phase 4 (Safety Features) being complete. The existing `guidance_check_stability()` function must be functional with timing infrastructure already in place.

---

## 1. FILE STRUCTURE & INTEGRATION

### 1.1 New Files Required

```
src/
├── stability_monitor.h           [NEW] Core monitor class & metrics struct
├── stability_monitor.cpp          [NEW] Implementation
├── servo_smoother.h               [NEW] Rate limiter, deadband, filter
├── servo_smoother.cpp             [NEW] Implementation
└── guidance_failsafe.cpp          [NEW] Integration logic (no header; functions only)

test/unit/
├── test_stability_monitor.cpp     [NEW] Unit tests for StabilityMonitor
├── test_servo_smoother.cpp        [NEW] Unit tests for ServoSmoother
└── test_guidance_failsafe.cpp     [NEW] Integration tests
```

### 1.2 Modified Files

```
src/
├── guidance_control.h             [MOD] Add failsafe integration declarations
├── guidance_control.cpp           [MOD] Call failsafe checks in guidance_update()
├── config.h                       [MOD] Add stability thresholds + failsafe params
├── data_structures.h              [MOD] Add fields for stability reporting
├── log_format_definition.cpp      [MOD] Add CSV headers for stability metrics

src/hal/
└── hal_interfaces.h               [NO CHANGE] Already has ITimer access needed

test/
└── unit/test_stability_debug.cpp  [NEW] Debug helpers (optional)
```

### 1.3 Integration Diagram

```
                    Main Flight Loop (10 Hz)
                            |
                    +-------v-------+
                    | guidance_update()    <-- Existing Phase 4
                    +-------+-------+
                            |
            +-------+-------+-------+-------+
            |       |       |       |       |
          Kalman   PID    Servo  <-+-> guidance_failsafe_check()
          Filter  Control Smoother |    [NEW 10 Hz]
                              |    |
                              |    v
                        guidance_check_stability()
                        [Phase 4 - EXISTING]
                              |
                              v
                        StabilityMonitor
                        [NEW - this phase]
                              |
                              v
                        Metrics {
                          roll_rate_dps,
                          pitch_rate_dps,
                          yaw_rate_dps,
                          roll_error_deg,
                          pitch_error_deg,
                          yaw_error_deg,
                          actuator_saturation_percent,
                          is_stable,
                          warning_msg
                        }
```

---

## 2. DETAILED CLASS/FUNCTION SIGNATURES

### 2.1 StabilityMonitor Class (stability_monitor.h/cpp)

**Header File: `src/stability_monitor.h`**

```cpp
#ifndef STABILITY_MONITOR_H
#define STABILITY_MONITOR_H

#include <Arduino.h>
#include <stdint.h>
#include "data_structures.h"  // For access to quaternion data

class StabilityMonitor {
public:
  // Metrics structure returned by getMetrics()
  struct StabilityMetrics {
    // Angular Rates (degrees per second)
    float roll_rate_dps;
    float pitch_rate_dps;
    float yaw_rate_dps;

    // Attitude Errors vs Desired (degrees)
    float roll_error_deg;
    float pitch_error_deg;
    float yaw_error_deg;

    // Actuator State
    float actuator_saturation_percent;  // 0-100%, max of all axes
    float pitch_saturation_percent;
    float roll_saturation_percent;
    float yaw_saturation_percent;

    // Overall Status
    bool is_stable;
    const char* violation_type;  // NULL if stable, else "RATE", "ATTITUDE", "SATURATION"
    uint8_t violation_bitmask;   // Bit 0=rate, 1=attitude, 2=saturation
  };

  // Constructor
  StabilityMonitor();

  // Initialize with threshold values from config
  void init();

  /**
   * @brief Update stability assessment
   * @param current_quat Quaternion from Kalman filter [q0, q1, q2, q3]
   * @param desired_quat Desired orientation quaternion
   * @param gyro_rate_radps [roll_rate, pitch_rate, yaw_rate] in rad/s
   * @param cmd_pitch Normalized actuator command [-1.0, 1.0]
   * @param cmd_roll  Normalized actuator command [-1.0, 1.0]
   * @param cmd_yaw   Normalized actuator command [-1.0, 1.0]
   * @param current_time_ms System time in milliseconds
   */
  void update(const float current_quat[4],
              const float desired_quat[4],
              const float gyro_rate_radps[3],
              float cmd_pitch,
              float cmd_roll,
              float cmd_yaw,
              uint32_t current_time_ms);

  // Get current metrics
  StabilityMetrics getMetrics() const;

  // Check if currently in violation
  bool isStabilityViolation() const;

  // Check if violation duration exceeded threshold
  bool isViolationPersistent() const;

  // Get time in current violation (ms)
  uint32_t getViolationDuration() const;

  // Reset violation timer and status
  void resetViolation();

  // Diagnostics: Print current state to serial
  void printDiagnostics() const;

private:
  // Internal state
  StabilityMetrics current_metrics;
  uint32_t violation_start_time_ms;
  bool in_violation;

  // Thresholds (loaded from config.h in init())
  struct {
    float roll_rate_limit_dps;         // Default: 180 DPS
    float pitch_rate_limit_dps;        // Default: 180 DPS
    float yaw_rate_limit_dps;          // Default: 360 DPS (higher for yaw)

    float roll_error_limit_deg;        // Default: 30 degrees
    float pitch_error_limit_deg;       // Default: 20 degrees
    float yaw_error_limit_deg;         // Default: 20 degrees

    float saturation_limit_percent;    // Default: 95%
    uint32_t violation_duration_ms;    // Default: 500 ms

  } thresholds;

  // Helper methods
  void quaternion_to_euler(const float q[4], float& roll, float& pitch, float& yaw);
  float rad_to_deg(float rad) const;
  float normalize_angle_deg(float deg) const;
  bool check_angular_rates(float roll, float pitch, float yaw);
  bool check_attitude_error(float roll_err, float pitch_err, float yaw_err);
  bool check_actuator_saturation(float cmd_p, float cmd_r, float cmd_y);
};

#endif // STABILITY_MONITOR_H
```

**Key Design Decisions:**

1. **Metrics Structure**: Includes individual axis saturation for diagnostic purposes
2. **Violation Bitmask**: Bitwise flags allow identifying multiple simultaneous violations
3. **Persistent Check**: Separate method for time-based threshold (500ms default) to prevent oscillation
4. **Helper Methods**: Quaternion-to-Euler conversion internal to class (cleaner encapsulation)

---

### 2.2 ServoSmoother Class (servo_smoother.h/cpp)

**Header File: `src/servo_smoother.h`**

```cpp
#ifndef SERVO_SMOOTHER_H
#define SERVO_SMOOTHER_H

#include <Arduino.h>
#include <stdint.h>

class ServoSmoother {
public:
  // Filtering type enumeration
  enum FilterType {
    RATE_LIMIT_ONLY,         // 0: Only rate limiting
    DEADBAND_ONLY,           // 1: Only deadband
    LOWPASS_ONLY,            // 2: Only low-pass filter
    RATE_LIMIT_THEN_LOWPASS, // 3: Rate limit, then filter
    FULL_FILTERING           // 4: All three applied in sequence
  };

  // Constructor
  ServoSmoother();

  // Initialize filter parameters from config
  void init(FilterType filter_type = FULL_FILTERING);

  /**
   * @brief Apply smoothing to servo command
   * @param desired_angle Desired servo angle (degrees, typically 0-180)
   * @param current_angle Current servo angle from last iteration (degrees)
   * @param time_delta_ms Time since last call (milliseconds)
   * @param axis_id 0=pitch, 1=roll, 2=yaw (for individual parameters if needed)
   * @return Smoothed command angle
   */
  float smooth(float desired_angle, float current_angle,
               uint32_t time_delta_ms, uint8_t axis_id = 0);

  /**
   * @brief Batch smooth all three axes
   * @param desired [pitch, roll, yaw] angles
   * @param current [pitch, roll, yaw] current angles
   * @param time_delta_ms Time delta
   * @param output [pitch, roll, yaw] smoothed outputs
   */
  void smoothBatch(const float desired[3], const float current[3],
                   uint32_t time_delta_ms, float output[3]);

  // Reset all filters (call when entering new state)
  void reset();

  // Get last output (useful for main loop)
  float getLastOutput(uint8_t axis_id) const;

private:
  FilterType active_filter;

  // Current outputs (used in rate limiting and filtering)
  float last_output[3];

  // Low-pass filter state (first-order)
  struct {
    float prev_output[3];  // Previous filter output
    float cutoff_hz;       // Filter cutoff frequency
    float alpha;           // Smoothing factor (recalculated per update)
  } lowpass;

  // Rate limiting parameters
  struct {
    float max_rate_per_ms[3];  // Max degrees per millisecond per axis
                               // Pitch: 10.0 deg/100ms = 0.1 deg/ms
                               // Roll:  10.0 deg/100ms = 0.1 deg/ms
                               // Yaw:   15.0 deg/100ms = 0.15 deg/ms
  } rate_limit;

  // Deadband parameters
  struct {
    float deadband_deg;        // Default: 0.5 degrees
  } deadband;

  // Helper functions
  float apply_rate_limit(float desired, float current,
                         float time_delta_ms, uint8_t axis_id);
  float apply_deadband(float command);
  float apply_lowpass(float command, uint32_t time_delta_ms, uint8_t axis_id);
  float clamp_angle(float angle, float min_ang, float max_ang) const;
};

#endif // SERVO_SMOOTHER_H
```

**Smoothing Algorithm Details:**

```
Step 1: Apply Rate Limiting
  max_change_this_frame = max_rate_per_ms[axis] * time_delta_ms
  limited_command = clamp(desired - current, -max_change, +max_change) + current

Step 2: Apply Deadband
  if |limited_command - current| < deadband:
    deadband_command = current
  else:
    deadband_command = limited_command

Step 3: Apply Low-Pass Filter (1st order)
  alpha = 2 * pi * cutoff_hz * time_delta_ms / 1000
  alpha = clamp(alpha, 0.0, 1.0)
  filtered_output = alpha * deadband_command + (1 - alpha) * prev_output
  prev_output = filtered_output
```

---

### 2.3 Failsafe Integration (guidance_failsafe.cpp - New Functions)

**New functions in `src/guidance_failsafe.cpp`:**

```cpp
// File: src/guidance_failsafe.cpp
// No header file; functions called directly from guidance_control.cpp

#include "guidance_control.h"
#include "stability_monitor.h"
#include "config.h"
#include <cstring>

// Global references to stability monitor and servo smoother
// Instantiated in guidance_init()
extern StabilityMonitor g_stability_monitor;
extern ServoSmoother g_servo_smoother;

// Failsafe state tracking
static struct {
  bool failsafe_active;
  uint32_t failsafe_start_ms;
  float gain_reduction_factor;  // 1.0 = full gain, 0.5 = 50% gain
  bool passive_mode_active;      // Servos centered, no control
} failsafe_state = {
  false, 0, 1.0f, false
};

/**
 * @brief Check stability and apply failsafe actions if needed
 * Called from main loop at 10 Hz (or every guidance_update)
 *
 * @param current_time_ms Current system time
 * @return true if failsafe was triggered, false otherwise
 */
bool guidance_failsafe_check(uint32_t current_time_ms) {
  if (!ENABLE_GUIDANCE) return false;

  const StabilityMonitor::StabilityMetrics metrics = g_stability_monitor.getMetrics();

  // Check for persistent violation (exceeds duration threshold)
  if (metrics.is_stable == false &&
      g_stability_monitor.isViolationPersistent()) {

    if (!failsafe_state.failsafe_active) {
      // First time entering failsafe
      failsafe_state.failsafe_active = true;
      failsafe_state.failsafe_start_ms = current_time_ms;
      failsafe_state.gain_reduction_factor = 1.0f;
    }

    uint32_t failsafe_duration = current_time_ms - failsafe_state.failsafe_start_ms;

    // LEVEL 1: Reduce PID gains (if enabled)
    if (failsafe_duration > GUIDANCE_FAILSAFE_LEVEL1_MS) {
      if (failsafe_state.gain_reduction_factor > GUIDANCE_FAILSAFE_MIN_GAIN) {
        failsafe_state.gain_reduction_factor -= 0.1f;  // Reduce by 10%
        if (failsafe_state.gain_reduction_factor < GUIDANCE_FAILSAFE_MIN_GAIN) {
          failsafe_state.gain_reduction_factor = GUIDANCE_FAILSAFE_MIN_GAIN;
        }
        log_guidance_event("Failsafe: Reducing PID gains");
      }
    }

    // LEVEL 2: Enter passive mode (if enabled and high saturation)
    if (failsafe_duration > GUIDANCE_FAILSAFE_LEVEL2_MS ||
        metrics.actuator_saturation_percent > 95.0f) {
      failsafe_state.passive_mode_active = true;
      guidance_center_servos();  // Existing function
      log_guidance_event("Failsafe: Entering passive mode (servos centered)");
    }

    // LEVEL 3: Enter ERROR state (if enabled and extended violation)
    if (failsafe_duration > GUIDANCE_FAILSAFE_LEVEL3_MS) {
      log_guidance_event("CRITICAL: Stability failsafe triggered ERROR state");
      setFlightState(ERROR);  // Existing flight_logic function
      return true;
    }

    return true;  // Failsafe active but not critical yet
  }

  // If stability restored and failsafe was active, recover
  if (failsafe_state.failsafe_active && metrics.is_stable == true) {
    if (failsafe_state.gain_reduction_factor < 1.0f) {
      failsafe_state.gain_reduction_factor += 0.05f;  // Recover 5% per check
      if (failsafe_state.gain_reduction_factor > 1.0f) {
        failsafe_state.gain_reduction_factor = 1.0f;
      }
    }

    if (!failsafe_state.passive_mode_active) {
      failsafe_state.failsafe_active = false;
      log_guidance_event("Failsafe: Stability recovered, resuming normal operation");
    }
  }

  return false;
}

/**
 * @brief Reset failsafe state
 * Called when entering new flight state or manually resetting system
 */
void guidance_failsafe_reset() {
  failsafe_state.failsafe_active = false;
  failsafe_state.failsafe_start_ms = 0;
  failsafe_state.gain_reduction_factor = 1.0f;
  failsafe_state.passive_mode_active = false;
}

/**
 * @brief Get current failsafe gain reduction factor
 * @return Factor from 0.0 to 1.0 to multiply PID gains by
 */
float guidance_get_failsafe_gain_factor() {
  return failsafe_state.gain_reduction_factor;
}

/**
 * @brief Check if passive mode is active
 * @return true if servos are centered and not responding to guidance
 */
bool guidance_is_passive_mode_active() {
  return failsafe_state.passive_mode_active;
}

/**
 * @brief Log guidance event for debugging
 * @param message Event message (max 100 chars)
 */
static void log_guidance_event(const char* message) {
  // Stub for now; in real impl, write to log buffer or EEPROM
  if (DEBUG_GUIDANCE) {
    Serial.print("[GUIDANCE] ");
    Serial.println(message);
  }
}
```

---

### 2.4 Modified guidance_control.cpp Integration

**Changes to existing `guidance_update()` function:**

```cpp
// In src/guidance_control.cpp - EXISTING FUNCTION (modified)

// At top of file, add includes:
#include "stability_monitor.h"
#include "servo_smoother.h"

// Add to global state:
extern StabilityMonitor g_stability_monitor;  // Defined in guidance_init()
extern ServoSmoother g_servo_smoother;        // Defined in guidance_init()

// In guidance_update() - ADD AFTER EXISTING PID CALCULATIONS:
//
// After line ~270 (after calculating PID outputs):
//
//   // Apply failsafe gain reduction
//   float gain_factor = guidance_get_failsafe_gain_factor();
//   actuator_output_x_g *= gain_factor;
//   actuator_output_y_g *= gain_factor;
//   actuator_output_z_g *= gain_factor;
//
//   // Apply servo smoothing before writing
//   // Convert to servo angles (0-180 from -1.0 to 1.0)
//   float servo_angle_pitch = 90.0f + (actuator_output_x_g * 45.0f);  // -1 to 1 -> 45 to 135 deg
//   float servo_angle_roll = 90.0f + (actuator_output_y_g * 45.0f);
//   float servo_angle_yaw = 90.0f + (actuator_output_z_g * 45.0f);
//
//   float servo_angle_pitch_smoothed = g_servo_smoother.smooth(
//     servo_angle_pitch, last_servo_angle_pitch, deltat * 1000.0f, 0);
//   float servo_angle_roll_smoothed = g_servo_smoother.smooth(
//     servo_angle_roll, last_servo_angle_roll, deltat * 1000.0f, 1);
//   float servo_angle_yaw_smoothed = g_servo_smoother.smooth(
//     servo_angle_yaw, last_servo_angle_yaw, deltat * 1000.0f, 2);
//
//   // Write smoothed angles to servos (if ENABLE_GUIDANCE)
//   if (ENABLE_GUIDANCE && !guidance_is_passive_mode_active()) {
//     servo_pitch.write(servo_angle_pitch_smoothed);
//     servo_roll.write(servo_angle_roll_smoothed);
//     servo_yaw.write(servo_angle_yaw_smoothed);
//   }
//
//   // Update servo smoother state
//   last_servo_angle_pitch = servo_angle_pitch_smoothed;
//   last_servo_angle_roll = servo_angle_roll_smoothed;
//   last_servo_angle_yaw = servo_angle_yaw_smoothed;
```

---

### 2.5 Modified guidance_check_stability()

**Changes to existing Phase 4 function (in src/guidance_control.cpp):**

```cpp
// EXISTING function - MODIFY to call new StabilityMonitor

void guidance_check_stability(float current_roll_rad, float current_pitch_rad, float current_yaw_rad,
                              float current_roll_rate_radps, float current_pitch_rate_radps, float current_yaw_rate_radps,
                              float actuator_cmd_pitch, float actuator_cmd_yaw, float actuator_cmd_roll,
                              unsigned long current_time_ms) {

  // NEW CODE: Update StabilityMonitor with current data
  float gyro_rad_s[3] = {
    current_roll_rate_radps,
    current_pitch_rate_radps,
    current_yaw_rate_radps
  };

  // Get current quaternion from Kalman filter
  // Assuming these are available globally:
  // extern float g_q0, g_q1, g_q2, g_q3;
  // (These should already exist from Phase 4 Kalman filter)
  float current_quat[4] = {g_q0, g_q1, g_q2, g_q3};

  // Desired quaternion (identity for now, or calculated from targets)
  float desired_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};

  g_stability_monitor.update(current_quat, desired_quat, gyro_rad_s,
                             actuator_cmd_pitch, actuator_cmd_roll, actuator_cmd_yaw,
                             current_time_ms);

  // EXISTING CODE BELOW (unchanged):
  // ... rest of existing stability checks ...
}
```

---

## 3. IMPLEMENTATION SEQUENCE

### Step 1: Create StabilityMonitor Core (Day 1)

**Files:** `src/stability_monitor.h`, `src/stability_monitor.cpp`

**Tasks:**
- [ ] Define `StabilityMetrics` structure with all fields
- [ ] Implement constructor and `init()` method
- [ ] Implement quaternion-to-Euler conversion helper
- [ ] Implement `update()` method with angular rate detection
- [ ] Implement `update()` method with attitude error detection
- [ ] Implement `update()` method with saturation detection
- [ ] Implement `getMetrics()` and status checking methods
- [ ] Add serial diagnostic output method
- [ ] Verify compilation on Teensy 4.1

**Testing:**
```cpp
// Unit test: test_stability_monitor.cpp
void test_angular_rate_detection_pitch_violation();
void test_angular_rate_detection_all_axes();
void test_attitude_error_detection_pitch();
void test_attitude_error_detection_roll_yaw();
void test_actuator_saturation_detection_individual();
void test_saturation_combined_axes();
void test_violation_timer_accumulation();
void test_metrics_after_violation_clear();
```

**Acceptance Criteria:**
- [ ] All unit tests pass (6+ tests)
- [ ] Quaternion conversion mathematically correct
- [ ] Metrics update with fresh data each call
- [ ] RAM usage < 500 bytes

---

### Step 2: Integrate with guidance_check_stability() (Day 1-2)

**Files:** `src/guidance_control.h`, `src/guidance_control.cpp`

**Tasks:**
- [ ] Add StabilityMonitor global instance to guidance_init()
- [ ] Modify `guidance_check_stability()` to call monitor.update()
- [ ] Pass quaternion and gyro data to monitor
- [ ] Verify existing stability checks still work
- [ ] Add new functions: `guidance_get_latest_stability_metrics()`
- [ ] Test with hardware or mock data

**Integration Points:**
- StabilityMonitor instance created in `guidance_init()`
- Called every `guidance_check_stability()` invocation
- Metrics available via new getter function for logging

**Acceptance Criteria:**
- [ ] Teensy 4.1 compiles without errors
- [ ] `guidance_check_stability()` calls monitor correctly
- [ ] Metrics retrievable and reasonable values observed
- [ ] No performance degradation (<1ms added per call)

---

### Step 3: Implement ServoSmoother (Day 2-3)

**Files:** `src/servo_smoother.h`, `src/servo_smoother.cpp`

**Tasks:**
- [ ] Implement rate limiting algorithm (per-axis max change)
- [ ] Implement deadband (ignore commands < threshold)
- [ ] Implement 1st-order low-pass filter (alpha calculation)
- [ ] Implement batch smoothing for all 3 axes
- [ ] Implement filter reset functionality
- [ ] Add configuration from config.h for thresholds
- [ ] Test individual filter operations

**Rate Limiting Algorithm:**
```
max_change = (max_rate_deg_per_sec / 1000.0) * time_delta_ms
limited = clamp(desired - current, -max_change, +max_change) + current
```

**Low-Pass Filter Algorithm (1st-order IIR):**
```
alpha = min(2*pi*f_c*dt, 1.0)  where f_c=cutoff Hz, dt=time in seconds
output = alpha*input + (1-alpha)*prev_output
```

**Acceptance Criteria:**
- [ ] Rate limiting prevents servo velocity from exceeding limits
- [ ] Deadband successfully rejects small commands
- [ ] Low-pass filter smooths without excessive lag
- [ ] All modes can be enabled/disabled via config
- [ ] Unit tests all pass (8+ tests)

---

### Step 4: Implement Failsafe Logic (Day 3-4)

**Files:** `src/guidance_failsafe.cpp`

**Tasks:**
- [ ] Create failsafe state tracking structure
- [ ] Implement `guidance_failsafe_check()` function
- [ ] Add Level 1: Gain reduction (10% decrements)
- [ ] Add Level 2: Passive mode (center servos)
- [ ] Add Level 3: ERROR state transition
- [ ] Implement recovery logic (gradual gain increase)
- [ ] Add logging/debugging output
- [ ] Integrate with guidance_update() main loop

**Failsafe Levels (default timings):**

| Level | Condition | Action | Duration |
|-------|-----------|--------|----------|
| 1 | Stability violation | Reduce PID gains 10% | 1000 ms |
| 2 | Continued violation | Center servos (passive) | 2000 ms |
| 3 | Persistent violation | Enter ERROR state | 5000 ms |

**Recovery Strategy:**
- When stability restored, increase gains by 5% per cycle until full
- Exit passive mode only when gains fully recovered
- Log all transitions for post-flight analysis

**Acceptance Criteria:**
- [ ] Failsafe correctly detects violation persistence
- [ ] Gain reduction applied to PID outputs
- [ ] Passive mode centers servos and disables guidance
- [ ] ERROR state entered after extended violation
- [ ] Recovery works with smooth gain increase
- [ ] Integration tests pass (6+ tests)

---

### Step 5: Add Logging & Configuration (Day 4-5)

**Files:** `src/config.h`, `src/data_structures.h`, `src/log_format_definition.cpp`

**Tasks:**
- [ ] Add stability thresholds to config.h
- [ ] Add failsafe parameters to config.h
- [ ] Add servo smoother parameters to config.h
- [ ] Extend LogData struct with stability fields
- [ ] Add CSV headers for stability metrics
- [ ] Populate log data in main flight loop
- [ ] Test CSV output with post-flight parser

**New config.h Parameters:**

```cpp
// === STABILITY MONITORING (Phase 6.2) ===
#define GUIDANCE_STABILITY_ROLL_RATE_LIMIT_DPS 180.0f
#define GUIDANCE_STABILITY_PITCH_RATE_LIMIT_DPS 180.0f
#define GUIDANCE_STABILITY_YAW_RATE_LIMIT_DPS 360.0f

#define GUIDANCE_STABILITY_ROLL_ERROR_LIMIT_DEG 30.0f
#define GUIDANCE_STABILITY_PITCH_ERROR_LIMIT_DEG 20.0f
#define GUIDANCE_STABILITY_YAW_ERROR_LIMIT_DEG 20.0f

#define GUIDANCE_STABILITY_SATURATION_LIMIT_PERCENT 95.0f
#define GUIDANCE_STABILITY_VIOLATION_DURATION_MS 500

// === FAILSAFE (Phase 6.2) ===
#define GUIDANCE_FAILSAFE_LEVEL1_MS 1000  // Start gain reduction
#define GUIDANCE_FAILSAFE_LEVEL2_MS 2000  // Enter passive mode
#define GUIDANCE_FAILSAFE_LEVEL3_MS 5000  // Enter ERROR state
#define GUIDANCE_FAILSAFE_MIN_GAIN 0.3f   // Minimum gain factor

// === SERVO SMOOTHER (Phase 6.2) ===
#define SERVO_SMOOTHER_ACTIVE 1
#define SERVO_SMOOTHER_TYPE 4  // 4 = FULL_FILTERING
#define SERVO_RATE_LIMIT_DPS 10.0f  // 10 deg per 100ms
#define SERVO_DEADBAND_DEG 0.5f
#define SERVO_LOWPASS_CUTOFF_HZ 2.0f  // 2 Hz cutoff
```

**New LogData Fields:**

```cpp
struct LogData {
  // ... existing fields ...

  // Phase 6.2: Stability Metrics
  float stability_roll_rate_dps;
  float stability_pitch_rate_dps;
  float stability_yaw_rate_dps;
  float stability_roll_error_deg;
  float stability_pitch_error_deg;
  float stability_yaw_error_deg;
  float stability_saturation_percent;
  uint8_t stability_violation_bitmask;
  uint8_t failsafe_status;  // 0=normal, 1=gain_reduction, 2=passive, 3=error
};
```

**CSV Headers:**

```
stability_roll_rate_dps,
stability_pitch_rate_dps,
stability_yaw_rate_dps,
stability_roll_error_deg,
stability_pitch_error_deg,
stability_yaw_error_deg,
stability_saturation_percent,
stability_violation_bitmask,
failsafe_status
```

**Acceptance Criteria:**
- [ ] All new parameters defined in config.h
- [ ] LogData struct extended without breaking existing code
- [ ] CSV headers correct and headers written first
- [ ] All fields populated in flight loop
- [ ] Post-flight CSV parsing handles new fields

---

## 4. TESTING STRATEGY

### 4.1 Unit Tests (Desktop - No Hardware)

**File: `test/unit/test_stability_monitor.cpp`**

```cpp
#include <unity.h>
#include "stability_monitor.h"
#include <cmath>

// Setup/Teardown
void setUp() {
  // Initialize fresh monitor before each test
  monitor = new StabilityMonitor();
  monitor->init();
}

void tearDown() {
  delete monitor;
}

// Test Cases

void test_quaternion_to_euler_identity() {
  float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // Identity quaternion
  float roll, pitch, yaw;
  // Should give 0, 0, 0 for identity
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, roll);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, pitch);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, yaw);
}

void test_angular_rate_detection_exceeds_pitch() {
  float current_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float desired_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float gyro[3] = {0.0f, 5.0f, 0.0f};  // 5 rad/s = ~287 DPS (exceeds 180 limit)
  float cmd[3] = {0.0f, 0.0f, 0.0f};

  monitor->update(current_q, desired_q, gyro, cmd[0], cmd[1], cmd[2], 0);

  auto metrics = monitor->getMetrics();
  TEST_ASSERT_FALSE(metrics.is_stable);
  TEST_ASSERT_EQUAL_INT(1, metrics.violation_bitmask & 0x01);  // Rate bit set
}

void test_attitude_error_detection_exceeds_roll() {
  float current_q[4] = {0.966f, 0.259f, 0.0f, 0.0f};  // ~30 deg roll
  float desired_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float gyro[3] = {0.0f, 0.0f, 0.0f};
  float cmd[3] = {0.0f, 0.0f, 0.0f};

  monitor->update(current_q, desired_q, gyro, cmd[0], cmd[1], cmd[2], 0);

  auto metrics = monitor->getMetrics();
  // Should detect >30 deg roll error (if limit is 30)
}

void test_saturation_detection_all_maxed() {
  float current_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float desired_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float gyro[3] = {0.0f, 0.0f, 0.0f};
  float cmd[3] = {1.0f, 1.0f, 1.0f};  // All maxed

  monitor->update(current_q, desired_q, gyro, cmd[0], cmd[1], cmd[2], 0);

  auto metrics = monitor->getMetrics();
  TEST_ASSERT_GREATER_THAN_FLOAT(90.0f, metrics.actuator_saturation_percent);
  TEST_ASSERT_EQUAL_INT(1, metrics.violation_bitmask & 0x04);  // Saturation bit
}

void test_violation_timer_accumulation() {
  // Continuously violate for 1 second
  float current_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float desired_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float gyro[3] = {0.0f, 5.0f, 0.0f};  // Always in violation
  float cmd[3] = {0.0f, 0.0f, 0.0f};

  for (int i = 0; i < 10; i++) {
    monitor->update(current_q, desired_q, gyro, cmd[0], cmd[1], cmd[2], i * 100);
  }

  uint32_t duration = monitor->getViolationDuration();
  TEST_ASSERT_GREATER_THAN_UINT32(400, duration);  // Should be ~900 ms
}

void test_violation_reset() {
  // Create violation, then clear metrics
  // ... setup violation ...
  monitor->resetViolation();

  TEST_ASSERT_FALSE(monitor->isStabilityViolation());
  TEST_ASSERT_EQUAL_UINT32(0, monitor->getViolationDuration());
}
```

**File: `test/unit/test_servo_smoother.cpp`**

```cpp
#include <unity.h>
#include "servo_smoother.h"

void setUp() {
  smoother = new ServoSmoother();
  smoother->init(ServoSmoother::FULL_FILTERING);
}

void tearDown() {
  delete smoother;
}

void test_rate_limiting_prevents_fast_change() {
  float current = 90.0f;
  float desired = 135.0f;  // 45 degree jump
  uint32_t dt = 50;  // 50 ms

  float output = smoother->smooth(desired, current, dt, 0);

  // With 10 deg/100ms limit, should only move 5 deg in 50ms
  float max_expected = current + 5.0f;
  TEST_ASSERT_LESS_THAN_FLOAT(max_expected, output);
}

void test_deadband_suppresses_small_commands() {
  float current = 90.0f;
  float desired = 90.3f;  // 0.3 degree change (< 0.5 deadband)
  uint32_t dt = 100;

  float output = smoother->smooth(desired, current, dt, 0);

  // Should stay at current due to deadband
  TEST_ASSERT_FLOAT_WITHIN(0.1f, current, output);
}

void test_lowpass_filter_smooths_noise() {
  float current = 90.0f;
  float* outputs = new float[10];

  // Apply oscillating commands
  for (int i = 0; i < 10; i++) {
    float desired = (i % 2 == 0) ? 91.0f : 89.0f;
    outputs[i] = smoother->smooth(desired, current, 100, 0);
    current = outputs[i];
  }

  // Should not oscillate severely - outputs should be damped
  for (int i = 2; i < 10; i++) {
    TEST_ASSERT_LESS_THAN_FLOAT(0.5f, fabs(outputs[i] - outputs[i-1]));
  }
}

void test_batch_smooth_all_axes() {
  float desired[3] = {100.0f, 85.0f, 95.0f};
  float current[3] = {90.0f, 90.0f, 90.0f};
  float output[3];

  smoother->smoothBatch(desired, current, 100, output);

  // All should be smoothed
  for (int i = 0; i < 3; i++) {
    TEST_ASSERT_GREATER_THAN_FLOAT(current[i], output[i]);
    TEST_ASSERT_LESS_THAN_FLOAT(desired[i], output[i]);
  }
}

void test_reset_clears_filter_state() {
  float desired = 100.0f;
  float current = 90.0f;
  smoother->smooth(desired, current, 100, 0);

  smoother->reset();

  float output_after_reset = smoother->smooth(desired, current, 100, 0);
  // Should be same as first call
}
```

**File: `test/unit/test_guidance_failsafe.cpp`**

```cpp
#include <unity.h>
#include "guidance_failsafe.h"
#include "stability_monitor.h"

void test_failsafe_gain_reduction_triggered() {
  // Simulate persistent stability violation
  // Check that gain factor decreases
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, guidance_get_failsafe_gain_factor());

  // After violation...
  guidance_failsafe_check(2000);  // 2 second violation

  TEST_ASSERT_LESS_THAN_FLOAT(1.0f, guidance_get_failsafe_gain_factor());
}

void test_failsafe_passive_mode_active() {
  // Trigger Level 2 (passive mode)
  guidance_failsafe_check(3000);

  TEST_ASSERT_TRUE(guidance_is_passive_mode_active());
}

void test_failsafe_recovery_after_stability_restored() {
  // ... trigger failsafe ...
  // ... then restore stability ...
  // Check that gain increases back to 1.0
}
```

### 4.2 Hardware-in-Loop (HIL) Tests

**Test 1: Disturbance Response**

```
Procedure:
1. Arm and enter COAST state
2. Apply simulated pitch disturbance (+5 deg)
3. Observe servo response and measure settling time
4. Verify stability metrics report violation and recovery

Expected Results:
- Servo responds within 100ms
- Disturbance corrected within 500ms
- Stability metrics show violation then recovery
- Failsafe not triggered for single disturbance
```

**Test 2: Actuator Saturation Handling**

```
Procedure:
1. Arm and enter COAST state
2. Command extreme attitude (e.g., 45 deg roll)
3. Observe actuator saturation percentage
4. Verify failsafe triggers if sustained

Expected Results:
- Saturation >90% when extreme attitude demanded
- Failsafe triggers after 500ms violation
- Gains reduce or passive mode activates
- Servos center after Level 2 failsafe
```

**Test 3: Servo Smoother Effectiveness**

```
Procedure:
1. Log servo outputs with and without smoother
2. Command rapid angle changes (e.g., 90->135 in 100ms)
3. Measure max rate of change and smoothness

Expected Results:
- Rate of change limited to configured max
- No oscillations from servo resonance
- Settling time <1 second for typical disturbance
```

### 4.3 Flight Validation

**Test Flight 1: Nominal Trajectory**
- Verify stability metrics logged correctly
- Confirm no false failsafe triggers during normal flight
- Measure actual servo response vs commanded

**Test Flight 2: High-Wind Conditions**
- Test with wind causing attitude disturbances
- Verify failsafe doesn't trigger excessively
- Check gain reduction effectiveness

**Test Flight 3: Extreme Boost Phase**
- High acceleration and rotation rates expected
- Verify stability monitoring handles high angular rates
- Confirm servo smoother prevents overshoot

---

## 5. CONFIGURATION INTEGRATION

### 5.1 Current Configuration Sources

**Primary:** `src/config.h`

```cpp
// Existing PID gains (Phase 4)
#define PID_ROLL_KP 0.3f
#define PID_ROLL_KI 0.1f
#define PID_ROLL_KD 0.01f

// Existing servo config (Phase 4)
#define SERVO_MIN_PULSE_WIDTH 1000
#define SERVO_MAX_PULSE_WIDTH 2000
#define SERVO_DEFAULT_ANGLE 90
```

### 5.2 New Phase 6.2 Configuration

**Add to `src/config.h` after line ~250:**

```cpp
// ===== PHASE 6.2: ADVANCED GUIDANCE CONTROL =====

// --- Stability Monitoring Thresholds ---
#define GUIDANCE_STABILITY_ROLL_RATE_LIMIT_DPS 180.0f
#define GUIDANCE_STABILITY_PITCH_RATE_LIMIT_DPS 180.0f
#define GUIDANCE_STABILITY_YAW_RATE_LIMIT_DPS 360.0f

#define GUIDANCE_STABILITY_ROLL_ERROR_LIMIT_DEG 30.0f
#define GUIDANCE_STABILITY_PITCH_ERROR_LIMIT_DEG 20.0f
#define GUIDANCE_STABILITY_YAW_ERROR_LIMIT_DEG 20.0f

#define GUIDANCE_STABILITY_SATURATION_LIMIT_PERCENT 95.0f
#define GUIDANCE_STABILITY_VIOLATION_DURATION_MS 500

// --- Failsafe Configuration ---
#define GUIDANCE_FAILSAFE_ENABLED 1
#define GUIDANCE_FAILSAFE_LEVEL1_MS 1000      // Start gain reduction at 1s
#define GUIDANCE_FAILSAFE_LEVEL2_MS 2000      // Enter passive mode at 2s
#define GUIDANCE_FAILSAFE_LEVEL3_MS 5000      // Enter ERROR state at 5s
#define GUIDANCE_FAILSAFE_MIN_GAIN 0.3f       // Minimum 30% gain

// --- Servo Smoother Configuration ---
#define SERVO_SMOOTHER_ENABLED 1
#define SERVO_SMOOTHER_TYPE 4                 // 0-4: filter mode
#define SERVO_RATE_LIMIT_DPS 10.0f            // Max 10 deg per 100ms
#define SERVO_DEADBAND_DEG 0.5f               // Ignore changes < 0.5 deg
#define SERVO_LOWPASS_CUTOFF_HZ 2.0f          // 2 Hz low-pass filter

// --- Runtime Gain Adjustment (Optional) ---
#define GUIDANCE_GAIN_ADJUSTMENT_ENABLED 1
```

### 5.3 Runtime Configuration Points

**Phase 6.2 allows limited runtime adjustment via serial commands:**

```cpp
// Serial command interface (in command_processor.cpp):

"guidance_gains [Kp] [Ki] [Kd]"
  Set PID gains at runtime (e.g., "guidance_gains 0.4 0.12 0.015")

"guidance_thresholds [roll_rate] [pitch_rate] [yaw_rate]"
  Adjust rate limits (e.g., "guidance_thresholds 200 200 400")

"servo_smooth [rate_limit] [deadband] [cutoff_hz]"
  Adjust smoother (e.g., "servo_smooth 12 0.3 2.5")

"failsafe_levels [l1] [l2] [l3]"
  Adjust failsafe timing (e.g., "failsafe_levels 800 1800 4500")

"stability_status"
  Print current stability metrics and failsafe state
```

### 5.4 EEPROM Configuration Storage (Optional Future)

For Phase 7, could add EEPROM persistence:

```cpp
#define EEPROM_GUIDANCE_PARAMS_ADDR 500
#define EEPROM_GUIDANCE_PARAMS_SIZE 128

// Store struct:
struct GuidanceParamsEEPROM {
  float pid_kp, pid_ki, pid_kd;
  float rate_limits[3];
  float thresholds[3];
  uint8_t checksum;
};
```

---

## 6. DEPENDENCIES & LIBRARIES

### 6.1 New External Dependencies

**None!** Phase 6.2 uses only:
- Standard C++ math library (`<cmath>`)
- Arduino framework (`<Arduino.h>`)
- Existing Kalman filter quaternion data
- Existing servo control library (PWMServo already in use)

### 6.2 Internal Dependencies

**Required Phase 4+ Components:**
- `src/guidance_control.cpp` - PID control loop
- `src/kalman_filter.h` - Quaternion orientation data
- `src/flight_logic.cpp` - Flight state management
- `src/config.h` - Configuration parameters
- Existing servo control setup in main firmware

**Can Use Data From:**
- `IMUManager` - Angular rates (gyroscope)
- `Kalman filter` - Quaternion orientation
- `PID controller` - Current actuator commands
- `FlightState` - Current flight phase

### 6.3 No Library Additions Needed

The architecture uses only existing Teensy 4.1 capabilities:
- PWMServo for servo control (already included)
- Floating point math (hardware accelerated)
- Time tracking via `millis()` (already available)

---

## 7. MEMORY & PERFORMANCE IMPACT

### 7.1 RAM Impact (Teensy 4.1)

```
StabilityMonitor:
  - StabilityMetrics struct:          ~100 bytes
  - Thresholds struct:                ~100 bytes
  - State tracking:                   ~20 bytes
  - Total per instance:               ~220 bytes

ServoSmoother:
  - Filter state (3 axes):            ~60 bytes
  - Last output tracking:             ~24 bytes
  - Configuration:                    ~40 bytes
  - Total per instance:               ~124 bytes

Failsafe state:
  - State tracking struct:            ~24 bytes

TOTAL PHASE 6.2 RAM:  ~368 bytes (~4% of Teensy 4.1's 960 KB)
```

**Impact on Available RAM:**
- Currently available: ~700 KB
- After Phase 6.2: ~699.6 KB
- **Headroom:** Still plenty for Phase 6.3+ features

### 7.2 Flash Memory Impact

```
StabilityMonitor code:
  - Header + impl:                    ~8 KB

ServoSmoother code:
  - Header + impl:                    ~6 KB

Failsafe logic:
  - Functions + integration:          ~3 KB

TOTAL NEW CODE:  ~17 KB

Current Flash: ~145 KB used (40% of 384 KB)
After Phase 6.2: ~162 KB (42% of 384 KB)

Remaining: ~220 KB for Phase 6.3-6.6
```

### 7.3 CPU Performance Impact

**Per-Update Cycle (called at 10 Hz from guidance_update or main loop):**

```
StabilityMonitor::update():
  - Quaternion to Euler:              ~0.2 ms (3 atan2 calls)
  - Rate checking (6 comparisons):    ~0.05 ms
  - Attitude checking (6 comparisons):~0.05 ms
  - Saturation checking (3 comparisons): ~0.05 ms
  - Total per call:                   ~0.35 ms

ServoSmoother::smooth() (for 3 axes):
  - Rate limiting (3 axes):           ~0.1 ms
  - Deadband check (3 axes):          ~0.05 ms
  - Low-pass filter (3 axes):         ~0.1 ms
  - Total:                            ~0.25 ms

Failsafe check:
  - Metric query + comparison:        ~0.1 ms

TOTAL PER LOOP:  ~0.7 ms
```

**Main Loop Timing (100 Hz nominal):**
- Current budget: 10 ms per cycle
- Phase 6.2 adds: 0.7 ms (7% utilization)
- **Remaining:** 9.3 ms (~93% available)

**Conclusion:** Phase 6.2 has negligible performance impact.

---

## 8. INTEGRATION WITH EXISTING CODE

### 8.1 Flight Logic Integration

**In `src/flight_logic.cpp` - ProcessFlightState():**

```cpp
// Add failsafe check when in active guidance states
if (current_flight_state == COAST ||
    current_flight_state == DROGUE_DESCENT ||
    current_flight_state == MAIN_DESCENT) {

  // 10 Hz failsafe check
  static uint32_t last_failsafe_check = 0;
  if (millis() - last_failsafe_check > 100) {
    if (guidance_failsafe_check(millis())) {
      log_event("Failsafe triggered during guidance");
    }
    last_failsafe_check = millis();
  }
}
```

### 8.2 Data Logging Integration

**In `src/TripleT_Flight_Firmware.cpp` - main flight loop:**

```cpp
// After guidance_update() call, populate stability metrics:
if (ENABLE_GUIDANCE && guidance_is_trajectory_active()) {
  auto metrics = guidance_get_latest_stability_metrics();

  log_data.stability_roll_rate_dps = metrics.roll_rate_dps;
  log_data.stability_pitch_rate_dps = metrics.pitch_rate_dps;
  log_data.stability_yaw_rate_dps = metrics.yaw_rate_dps;
  log_data.stability_roll_error_deg = metrics.roll_error_deg;
  log_data.stability_pitch_error_deg = metrics.pitch_error_deg;
  log_data.stability_yaw_error_deg = metrics.yaw_error_deg;
  log_data.stability_saturation_percent = metrics.actuator_saturation_percent;
  log_data.stability_violation_bitmask = metrics.violation_bitmask;
  log_data.failsafe_status = guidance_is_passive_mode_active() ? 2 : 0;
}
```

### 8.3 Command Processor Integration

**In `src/command_processor.cpp` - Add new commands:**

```cpp
else if (strcmp(cmd, "stability_status") == 0) {
  auto metrics = guidance_get_latest_stability_metrics();
  serial->print("Roll Rate: ");
  serial->print(metrics.roll_rate_dps);
  serial->println(" DPS");
  // ... print all metrics ...
}

else if (strcmp(cmd, "guidance_gains") == 0) {
  // Parse and apply new gains
}
```

---

## 9. IMPLEMENTATION CHECKPOINTS

### Checkpoint 1: StabilityMonitor Functional (Day 1-2)

**Deliverables:**
- [ ] `stability_monitor.h/cpp` compiles on Teensy
- [ ] 6+ unit tests pass
- [ ] Metrics computed correctly for known inputs
- [ ] Integration with `guidance_check_stability()` working

**Verification:**
```bash
pio test -e native_test -f test_stability_monitor
pio run -e teensy41  # Verify compiles
```

### Checkpoint 2: ServoSmoother Functional (Day 2-3)

**Deliverables:**
- [ ] `servo_smoother.h/cpp` compiles
- [ ] 8+ unit tests pass
- [ ] All filter modes work (rate limit, deadband, lowpass)
- [ ] Batch processing functional

**Verification:**
```bash
pio test -e native_test -f test_servo_smoother
```

### Checkpoint 3: Failsafe Logic Complete (Day 3-4)

**Deliverables:**
- [ ] `guidance_failsafe.cpp` compiles
- [ ] Gain reduction working
- [ ] Passive mode centers servos
- [ ] ERROR state transition on Level 3
- [ ] Recovery logic functional
- [ ] 6+ integration tests pass

**Verification:**
```bash
pio test -e native_test -f test_guidance_failsafe
```

### Checkpoint 4: Full Integration (Day 4-5)

**Deliverables:**
- [ ] All three components integrated
- [ ] Teensy 4.1 firmware compiles without warnings
- [ ] Flight loop calls all new components
- [ ] CSV logging includes stability fields
- [ ] Serial commands functional
- [ ] No performance degradation

**Verification:**
```bash
pio run -e teensy41
# Upload and test on hardware:
# 1. arm command
# 2. status_sensors command
# 3. stability_status command
```

### Checkpoint 5: Flight Validation (Day 5-7)

**Deliverables:**
- [ ] 3+ test flights completed
- [ ] Stability metrics logged correctly
- [ ] CSV files parse without errors
- [ ] No unexpected failsafe triggers
- [ ] Post-flight analysis completed

---

## 10. RISK ASSESSMENT & MITIGATION

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| Quaternion-to-Euler conversion errors | Medium | High | Validate with known quaternions, unit tests |
| Servo smoother introduces lag | Low | Medium | HIL testing with response time measurement |
| Failsafe triggers on false positives | Medium | High | Conservative thresholds, tuning during flights |
| Performance degradation | Low | Medium | Profiling, optimization of hot loops |
| Integration issues with Phase 4 | Low | High | Early integration testing, careful code review |
| PID gain reduction insufficient | Low | Medium | Implement Level 2 (passive mode) fallback |

---

## 11. DELIVERABLES CHECKLIST

### Code Files
- [ ] `src/stability_monitor.h` - Complete, documented
- [ ] `src/stability_monitor.cpp` - Implementation, no warnings
- [ ] `src/servo_smoother.h` - Complete, documented
- [ ] `src/servo_smoother.cpp` - Implementation, no warnings
- [ ] `src/guidance_failsafe.cpp` - Implementation, integrated

### Modified Files
- [ ] `src/guidance_control.h` - Declarations for failsafe functions
- [ ] `src/guidance_control.cpp` - Integration points, failsafe calls
- [ ] `src/config.h` - New Phase 6.2 parameters
- [ ] `src/data_structures.h` - Stability metric fields in LogData
- [ ] `src/log_format_definition.cpp` - CSV headers

### Test Files
- [ ] `test/unit/test_stability_monitor.cpp` - 6+ tests passing
- [ ] `test/unit/test_servo_smoother.cpp` - 8+ tests passing
- [ ] `test/unit/test_guidance_failsafe.cpp` - 6+ tests passing

### Documentation
- [ ] Code comments in all new functions
- [ ] README explaining stability monitoring
- [ ] Configuration guide for tuning thresholds
- [ ] Post-flight analysis guide for stability CSV fields

### Hardware Validation
- [ ] 3+ successful test flights
- [ ] Stability metrics logged and analyzed
- [ ] No unexpected failures or false positives
- [ ] Performance meets specifications

---

## 12. NEXT STEPS AFTER PHASE 6.2

**Upon completion, Phase 6.3 (Production Readiness) can begin:**

1. **Power Management** - Optimize power consumption per mode
2. **Edge Cases** - Handle GPS loss, sensor saturation, thermal throttling
3. **Pre-Flight Checks** - Automated system verification
4. **Enhanced Commands** - New serial interface commands

**Estimated Timeline:**
- Phase 6.2: 1 week (5 days coding + 2 days validation)
- Phase 6.3: 1-1.5 weeks
- Phase 6.4-6.6: 2-3 weeks
- **Total Phase 6:** 4-5 weeks
- **v1.0.0 Release:** Early March 2026

---

## 13. DOCUMENT APPROVAL & SIGN-OFF

**Prepared By:** Claude Code (AI Assistant)
**Date:** February 16, 2026
**Status:** Ready for Implementation

**Required Approvals:**
- [ ] Project Lead - Architecture approved
- [ ] Safety Lead - Failsafe logic reviewed
- [ ] Test Lead - Test plan reviewed

**Document Version:** 1.0
**Last Updated:** 2026-02-16

---

## Appendix A: Detailed Quaternion-to-Euler Conversion

For reference, the quaternion-to-Euler conversion used internally:

```cpp
void StabilityMonitor::quaternion_to_euler(const float q[4],
                                            float& roll,
                                            float& pitch,
                                            float& yaw) {
  // q[0] = w (scalar), q[1] = x, q[2] = y, q[3] = z

  // Roll (phi)
  float sin_roll = 2.0f * (q[0] * q[1] + q[2] * q[3]);
  float cos_roll = 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]);
  roll = atan2(sin_roll, cos_roll);

  // Pitch (theta)
  float sin_pitch = 2.0f * (q[0] * q[2] - q[3] * q[1]);
  sin_pitch = constrain(sin_pitch, -1.0f, 1.0f);  // Clamp for numerical stability
  pitch = asin(sin_pitch);

  // Yaw (psi)
  float sin_yaw = 2.0f * (q[0] * q[3] + q[1] * q[2]);
  float cos_yaw = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
  yaw = atan2(sin_yaw, cos_yaw);
}
```

---

## Appendix B: Low-Pass Filter Design Notes

For the servo smoothing low-pass filter:

**Transfer Function (1st order):**
```
H(s) = ω_c / (s + ω_c)

where ω_c = 2π × f_c (cutoff frequency in rad/s)
```

**Discretization (Tustin method):**
```
α = 2π × f_c × Δt / (1 + 2π × f_c × Δt)
y[n] = α × u[n] + (1 - α) × y[n-1]
```

**Typical Parameters:**
- **Cutoff Frequency:** 2 Hz (smooth but responsive)
- **Time Step:** ~10 ms (100 Hz control)
- **α Value:** ~0.11

**Response:**
- Attenuation at 1 Hz: -3 dB (half power)
- Roll-off: -20 dB/decade above cutoff
- Suitable for servo applications

---

**End of Document**
