---
title: Code Review Findings (2026-02, v0.51 baseline)
type: query
tags: [audit, code-quality, technical-debt, history]
created: 2026-04-22
updated: 2026-04-22
related_files: [.archived/CODE_REVIEW_FINDINGS_2026.md, .archived/EXECUTIVE_SUMMARY_2026.md, .archived/FIX_WALKTHROUGH.md]
---

Archival snapshot of the Feb 2026 code review that catalysed the refactoring captured in [[queries/roadmap-2026]]. Scored the firmware **5.4 / 10** at v0.51 and proposed a target of 8+ post-Phases 1-4. Most critical findings have since been addressed; this page records what was found and where it landed.

## Scorecard at v0.51

| Axis | Score | Target | Status (v0.10.0) |
|------|-------|--------|------------------|
| Real-time safety | 7 | 8+ | ✅ watchdog + EEPROM + non-blocking pyro merged |
| Code organisation | 4 | 7+ | 🟡 HAL + sensor layers done; state-handler refactor still pending |
| Modularity | 3 | 8+ | ✅ HAL + `IMUInterface` delivered |
| Testing | 0 | 8+ | 🟡 Unity + CI in place; coverage growing, not at 70 % yet |
| Documentation | 5 | 8+ | 🟡 `docs/` + wiki; consistency check ongoing |
| Error handling | 6 | 8+ | ✅ `ErrorCode_t` taxonomy + graceful degradation merged |
| Memory safety | 6 | 8+ | 🟡 Arduino `String` still used in spots |
| Performance | 8 | 8+ | ✅ acceptable; fixed timestep not yet enforced |
| Maintainability | 4 | 7+ | 🟡 improving as layers settle |

## Critical Fixes (all merged)

| Finding | Resolution |
|---------|------------|
| Blocking serial I/O could stall main loop | Non-blocking UART handling in `command_processor.cpp` |
| Landing detection timer didn't reset on false trigger | Reset on accel excursion |
| Pyro firing blocked main loop for ~250 ms | Non-blocking state-driven pyro timing |
| `calibrate` could be spammed causing DoS | State-guard: only accepted in `PAD_IDLE` / `CALIBRATION` |
| No watchdog | `WDT_T4` 1000 ms + EEPROM state recovery ([[concepts/system-robustness]]) |
| Apogee counter not reset on COAST re-entry | `resetApogeeDetectionCounters()` added (per [[concepts/apogee-detection]]) |
| Burnout detection jittery | Required 3 consecutive samples below threshold (commit `9a6700c`) |

## Architectural Debt Flagged

| Debt | Recommendation | Current status |
|------|---------------|----------------|
| `ProcessFlightState()` was a 782-line "god function" | State Pattern refactor | 🟡 still monolithic; tracked for Phase 6+ |
| 21 `extern` declarations coupling flight logic to globals | Introduce `FlightContext` struct | 🟡 not yet addressed |
| 891+ direct Arduino/Teensy hardware calls | HAL abstraction | ✅ HAL merged; flight-logic-level callers migrated |
| Arduino `String` class — heap fragmentation risk | Replace with fixed buffers | 🟡 partial |
| Kalman `dt` varies (not fixed-timestep) | Implement fixed-timestep loop | 🟡 not enforced |
| <5 % test coverage | Grow toward 70 %+ | 🟡 infrastructure delivered (Phase 3); backfill in progress |

## Missing Features Flagged

- Quaternion orientation (gimbal-lock risk at ±90° pitch) — plan lives in `docs/QUATERNION_MIGRATION_PLAN.md`; see [[concepts/kalman-filter]].
- Trajectory SD-card loader — scoped for Phase 6.1.
- Cross-track error control — scoped for Phase 6.1.
- Live telemetry — scoped for Phase 6.x; see [[entities/esp32-telemetry]].

## Prioritised Action Plan (from the original review)

- **Week 1**: apogee-reset fix + first unit test + sanity checks → shipped.
- **Weeks 2–5**: HAL + mocks → shipped (Phases 1 + 2).
- **Weeks 6–9**: refactor god function + global extraction → partial.
- **Weeks 10–14**: test coverage to ≥ 70 % overall, ≥ 85 % safety-critical → ongoing.

## How to Use This Document

This page is **historical**. Do not treat it as current guidance. For the current state, use:

- [[queries/development-status-2026-04]] — what's actually shipped right now.
- [[queries/roadmap-2026]] — what's planned next.
- [[concepts/system-robustness]] — the current defensive-design picture.

Preserved because the findings provide *why* context for subsequent architectural decisions — particularly the HAL and the graceful-degradation model.
