---
title: Graceful Guidance Degradation
type: concept
tags: [guidance, safety, failsafe, stability]
created: 2026-04-15
updated: 2026-04-15
related_files: [src/guidance_control.cpp, src/guidance_failsafe.cpp, src/stability_monitor.h, src/guidance_control.h]
---

When the guidance system detects a stability violation during BOOST or COAST, it gracefully disables itself rather than triggering an ERROR state. This ensures parachute deployment is never skipped.

## Problem Solved

**Before**: Stability failure → ERROR state → auto-recovery → PAD_IDLE. Apogee detection and parachute deploy were skipped entirely.

**After**: Stability failure → guidance disabled (orange LED) → flight continues normally → COAST → APOGEE → parachute deploy.

## Runtime Flag

```cpp
extern bool g_guidance_active;  // global, default = true
```

When set to `false`:
- `guidance_update()` is not called (servos receive no PID commands)
- `guidance_center_servos()` is called once — fins return to 90° neutral
- Status LED turns orange
- Log field `GuidanceActive` = 0

## Stability Monitor

`StabilityMonitor` class (`src/stability_monitor.h`) tracks violations against thresholds:

| Metric | Default Limit |
|--------|--------------|
| Roll/Pitch rate | 180 DPS |
| Yaw rate | 360 DPS |
| Roll/Pitch attitude error | 30° / 20° |
| Yaw attitude error | 20° |
| Actuator saturation | 95% |
| Violation persistence | 500 ms |

A violation must persist for 500 ms before action is taken (prevents noise-triggered disables).

## Failsafe Escalation

`guidance_failsafe_check()` (`src/guidance_failsafe.cpp`) implements a 4-level escalation:

| Level | State | Action |
|-------|-------|--------|
| 0 | Normal | PID control at full gains |
| 1 | Gain Reduction | PID gains multiplied by < 1.0 factor |
| 2 | Passive | Servos centered, control disabled (`g_guidance_active = false`) |
| 3 | Error | (Reserved; currently maps to level 2 to avoid ERROR state) |

## Key Functions

| Function | File | Purpose |
|----------|------|---------|
| `guidance_center_servos()` | `guidance_control.cpp` | Return all fins to 90° neutral |
| `guidance_failsafe_check(ms)` | `guidance_failsafe.cpp` | Check and escalate if needed |
| `guidance_failsafe_reset()` | `guidance_failsafe.cpp` | Reset on flight state entry |
| `guidance_failsafe_get_gain_factor()` | `guidance_failsafe.cpp` | Multiplier for PID gains (0.0–1.0) |
| `guidance_is_stability_compromised()` | `guidance_control.cpp` | Query current stability state |

## Data Logging

`LogData.guidance_active` (`bool`, CSV column: `GuidanceActive`) logged every 200ms. Use for post-flight analysis to determine when and why guidance was disabled.
