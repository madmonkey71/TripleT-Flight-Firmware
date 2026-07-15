---
title: Graceful Guidance Degradation
type: concept
tags: [guidance, safety, failsafe, stability]
created: 2026-04-15
updated: 2026-07-02
related_files: [src/guidance_control.cpp, src/guidance_failsafe.cpp, src/stability_monitor.h, src/guidance_control.h]
---

When the guidance system detects a sustained stability violation, it gracefully degrades and ultimately disables itself rather than triggering an ERROR state. This ensures parachute deployment is never skipped. Two paths enforce this: the Phase-6.2 `StabilityMonitor` + failsafe escalation running at 50 Hz inside `guidance_update()` (active guidance states: COAST / DROGUE_DESCENT / MAIN_DESCENT), and a legacy `guidance_check_stability()` check called by the state machine during BOOST and COAST. Both now use the **same threshold set** — the legacy `STABILITY_*` constants alias the `GUIDANCE_STABILITY_*` values in `config.h`.

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

`StabilityMonitor` class (`src/stability_monitor.h`) tracks violations against thresholds (the single unified set, `GUIDANCE_STABILITY_*` in `config.h`, also aliased by the legacy `STABILITY_*` names):

| Metric | Default Limit |
|--------|--------------|
| Roll/Pitch rate | 180 DPS |
| Yaw rate | 360 DPS |
| Roll/Pitch attitude error | 30° / 20° |
| Yaw attitude error | 20° |
| Actuator saturation | 95% |
| Violation persistence | 500 ms |

A violation must persist for 500 ms before action is taken (prevents noise-triggered disables). The monitor sets `is_stable` on every update, so once conditions return to normal the failsafe automatically recovers gains.

## Failsafe Escalation

`guidance_failsafe_check()` (`src/guidance_failsafe.cpp`) implements a 4-level escalation, checked at 50 Hz inside `guidance_update()`:

| Level | Trigger | Action |
|-------|---------|--------|
| 0 — Normal | — | PID control at full gains |
| 1 — Gain Reduction | violation ≥ 1 s | PID gain factor reduced −0.02 per call, floor `GUIDANCE_FAILSAFE_MIN_GAIN` = 0.3 |
| 2 — Passive | violation ≥ 2 s, or saturation > 95% | Servos centered, control disabled |
| 3 — Disabled | violation ≥ 5 s | `g_guidance_active = false` **permanently** — guidance off for the rest of the flight; no ERROR state; parachute logic unaffected |

Level 3 is implemented and distinct from level 2 (it is not "reserved"). **Recovery:** when the monitor reports stable again before level 3, gains climb back at +0.05 per call and the failsafe deactivates once gains reach 1.0 (levels below 2 only). A level-3 disable is not undone by recovery — only `guidance_failsafe_reset()` on a flight-state transition clears the failsafe state.

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
