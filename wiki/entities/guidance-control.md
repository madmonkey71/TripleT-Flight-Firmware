---
title: guidance_control — PID Guidance & Trajectory Following
type: entity
tags: [guidance, pid, trajectory, servos, stability]
created: 2026-04-15
updated: 2026-04-15
related_files: [src/guidance_control.cpp, src/guidance_control.h, src/guidance_failsafe.cpp, src/stability_monitor.h, src/servo_smoother.cpp]
---

PID-based active guidance system for fin-controlled rockets. Controls 3-axis orientation during BOOST and COAST. Includes trajectory following and a graceful degradation failsafe. See [[concepts/guidance-degradation]].

## Enable/Disable

```cpp
// src/config.h
#define ENABLE_GUIDANCE 1  // 0 = passive rocket (no servos needed)

extern bool g_guidance_active;  // Runtime flag; false = servos centered, no PID
```

## Core Workflow

```
guidance_init() → once on startup
  ↓ (each guidance loop iteration, ~20-50ms)
guidance_set_target_orientation_euler(roll, pitch, yaw)  ← from trajectory or 0,0,0
guidance_update(current_roll, current_pitch, current_yaw,
                roll_rate, pitch_rate, yaw_rate,
                lat, lon, alt_msl, dt)
  → internally calls guidance_check_stability(...)
  → outputs to PIDControllerState structs
guidance_get_actuator_outputs(x, y, z)  → [-1.0, 1.0] normalized
  → mapped to servo angles via ServoSmoother
guidance_failsafe_check(ms)  → may set g_guidance_active = false
```

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
- Monitor: `src/stability_monitor.h` — quaternion-based attitude error + rate checks
- Failsafe: `src/guidance_failsafe.cpp` — 4-level escalation (normal → gain reduction → passive → error)

See [[concepts/guidance-degradation]] for full behavior.

## Servo Smoothing

`ServoSmoother` (`src/servo_smoother.cpp`) applies rate limiting to prevent abrupt fin movements. Normalized outputs [-1.0, 1.0] are mapped to PWM microsecond values before writing to servos.
