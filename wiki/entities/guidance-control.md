---
title: guidance_control — PID Guidance & Trajectory Following
type: entity
tags: [guidance, pid, trajectory, servos, stability]
created: 2026-04-15
updated: 2026-07-02
related_files: [src/guidance_control.cpp, src/guidance_control.h, src/guidance_failsafe.cpp, src/stability_monitor.h, src/servo_smoother.cpp]
---

PID-based active guidance system for fin-controlled rockets. Servo-writing PID control runs during **COAST, DROGUE_DESCENT, and MAIN_DESCENT** (and only while the vehicle is not detected as stationary) — during **BOOST** only stability monitoring runs; there is no active fin control under thrust. Includes trajectory following and a graceful degradation failsafe. See [[concepts/guidance-degradation]].

## Enable/Disable

```cpp
// src/config.h
#define ENABLE_GUIDANCE 1  // 0 = passive rocket (no servos needed)

extern bool g_guidance_active;  // Runtime flag; false = servos centered, no PID
```

## Core Workflow

```
guidance_init() → once on startup
  ↓ (each guidance loop iteration, fixed 20 ms = 50 Hz)
guidance_set_target_orientation_euler(roll, pitch, yaw)  ← attitude captured at COAST entry, or trajectory
guidance_update(current_roll, current_pitch, current_yaw,
                roll_rate, pitch_rate, yaw_rate,
                lat, lon, alt_msl, dt)
  → runs the StabilityMonitor + guidance_failsafe_check(ms) at 50 Hz
    (may reduce gains or set g_guidance_active = false)
  → outputs to PIDControllerState structs
guidance_get_actuator_outputs(x, y, z)  → [-1.0, 1.0] normalized
  → smoothed by ServoSmoother, then mapped norm*90 + 90 → 0-180° servo angles
```

The separate legacy `guidance_check_stability(...)` path is called from the state machine during BOOST/COAST; both paths now enforce the same threshold set (the legacy `STABILITY_*` constants alias the Phase-6.2 `GUIDANCE_STABILITY_*` values in `config.h`).

## PID State

```cpp
typedef struct {
  float integral;         // I-term accumulator
  float previous_error;   // D-term (error-based derivative)
  float previous_value;   // D-term (measurement-based derivative)
} PIDControllerState;
```

Three instances: roll, pitch, yaw.

## Trajectory Following

Hardcoded test trajectory via `guidance_load_test_trajectory()`. Future: SD-card load.

```cpp
typedef struct {
  int32_t latitude;   // deg * 1e7
  int32_t longitude;  // deg * 1e7
  float altitude_msl; // meters MSL
} TrajectoryWaypoint_t;

typedef struct {
  TrajectoryWaypoint_t waypoints[MAX_TRAJECTORY_WAYPOINTS];
  uint8_t num_waypoints;
  uint8_t current_target_wp_index;
  bool is_active;
  bool is_loaded;
} Trajectory_t;
```

Key trajectory functions:
- `guidance_load_test_trajectory()` — load hardcoded waypoints
- `guidance_activate_trajectory(bool)` — enable/disable following
- `guidance_reset_trajectory_state()` — reset on new flight

## Failsafe & Stability

Handled by `GuidanceStabilityStatus` struct and `StabilityMonitor` class:
- Monitor: `src/stability_monitor.h` — attitude error + rate + saturation checks; publishes `is_stable` each update, which drives automatic gain recovery
- Failsafe: `src/guidance_failsafe.cpp` — 4-level escalation (normal → gain reduction at ≥1 s → passive/servos centered at ≥2 s → permanent guidance disable at ≥5 s). No ERROR state is triggered; parachute logic is unaffected.

See [[concepts/guidance-degradation]] for full behavior.

## Servo Smoothing

`ServoSmoother` (`src/servo_smoother.cpp`) operates on the normalized [-1.0, 1.0] commands: rate limiting (feeding back the previous smoothed output), deadband, and a 2 Hz low-pass IIR. The config constants are specified in the servo-degree domain (`SERVO_RATE_LIMIT_DPS` 10 deg/100 ms, `SERVO_DEADBAND_DEG` 0.5°, `SERVO_LOWPASS_CUTOFF_HZ` 2 Hz) and converted to the normalized domain (÷90) at init. Smoothed outputs are then mapped `norm*90 + 90` to a 0–180° servo angle (neutral 90°) before writing.
