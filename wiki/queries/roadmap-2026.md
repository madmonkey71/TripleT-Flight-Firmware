---
title: Development Roadmap (v0.6.0 → v1.0.0)
type: query
tags: [roadmap, phases, planning, versions]
created: 2026-04-22
updated: 2026-07-02
related_files: [.archived/IMPLEMENTATION_PLAN_2026.md, .archived/EXECUTIVE_SUMMARY_2026.md, src/config.h]
---

*Correction (2026-07-02): any telemetry status in this snapshot is stale — the Teensy telemetry module (`src/telemetry.h/.cpp`) and both ESP32 firmwares DO exist in the tree and are complete implementations, wired behind `ENABLE_TELEMETRY` (default 0). The remaining telemetry gaps are only the web-console `TELEM` parser and the end-to-end bench test. Additionally, "2-of-3 apogee voting" in this snapshot describes the dormant `ApogeeDetector` class; the live `detectApogee()` is OR/first-match across four methods. See [[queries/system-workflow-audit-2026-07]].*

Snapshot of the 2026 roadmap taking TripleT from a refactoring baseline at v0.6.0-dev to a v1.0.0 production release. Phase structure is per `.archived/IMPLEMENTATION_PLAN_2026.md`; status fields reflect what is merged as of **2026-05-25** (verified against `git log`).

> **2026-05-25 scope decision:** v1.0.0 is gated on **trajectory (6.1) + telemetry + flight validation**, NOT on full Phase 6.3. `PowerManager`, edge-case handlers, `PreflightChecker`, and thermal management are deferred to v1.1. See [[queries/v1-release-gate-2026-05]] for the concrete gate and acceptance criteria.

## Phase Summary

| Phase | Target version | Scope | Status |
|-------|----------------|-------|--------|
| 1 — HAL Foundation | v0.6.0 → v0.7.0 | Pure-virtual interfaces (ITimer, ISerial, IGPIO, II2C, IEEPROM, ISDCard, IServo, IWatchdog); Teensy + Mock impls | ✅ merged |
| 2 — Sensor Modularity | v0.7.0 → v0.8.0 | `IMUInterface`; ICM/KX134/BNO085 adapters; `IMUManager`; sensor factory | ✅ merged |
| 3 — Testing Infrastructure | v0.8.0 → v0.9.0 | Unity native tests, GitHub Actions, 3-tier mock strategy | ✅ merged (d797801) |
| 4 — Safety & Redundancy | v0.9.0 → v0.10.0 | Multi-path apogee, cross-validation, watchdog recovery, graceful guidance degrade | ✅ merged (dd7360c, f121c3f, 8aeb589) |
| 5 — Documentation & Polish | v0.10.0 → v1.0.0-rc1 | Developer / user / safety guides; wiki consolidation; release notes | ✅ merged (3198d91 — wiki is single source of truth) |
| 6 — Advanced features + production readiness | toward v1.0.0 / v1.1 | Sub-phases 6.1 (trajectory), 6.2 (guidance stability), 6.3 (power/edge-cases/pre-flight), 6.4 (validation release) | 🟡 6.2 merged (a560c9a); 6.1 in progress; **6.3 deferred to v1.1**; 6.4 follows 6.1 |

## Current version

`#define FIRMWARE_VERSION "v0.10.0"` — source of truth in `src/config.h`.

## Phase-by-Phase Detail

### Phase 1 — HAL Foundation ✅
Deliverables: 8 pure-virtual interfaces, Teensy and Mock implementations, compile-flag switch (`NATIVE_TEST_BUILD`), no behaviour change to flight logic. See [[concepts/hal-abstraction]].

### Phase 2 — Sensor Modularity ✅
Deliverables: `IMUInterface` (20 methods); adapter classes for ICM-20948, KX134, BNO085 (stub); `IMUManager` with automatic failover; `sensor_factory` compile-time selection. See [[concepts/sensor-redundancy]].

### Phase 3 — Testing Infrastructure ✅
Deliverables: Unity test framework under `test/unit/`; ArduinoFake + mock HAL; synthetic / recorded / failure-injection fixture model; GitHub Actions CI/CD (`.github/workflows/test.yml`). See [[concepts/testing-strategy]].

### Phase 4 — Safety & Redundancy ✅
Deliverables: 2-of-3 apogee voting + 20 s backup timer ([[concepts/apogee-detection]]); cross-sensor validation (`src/sensor_validator.h`); watchdog + EEPROM recovery ([[concepts/system-robustness]]); graceful guidance degradation on stability violation ([[concepts/guidance-degradation]]).

### Phase 5 — Documentation & Polish ✅
Merged: wiki is now the single source of truth (PR #13, commit `3198d91`). Architecture, ADRs, dev workflow, testing strategy, hardware platform, command catalogue all consolidated under `wiki/`; historical/superseded docs preserved under `.archived/`. Phase closed.

### Phase 6 — Advanced Features 🟡

- **6.1 Trajectory following** — Waypoint navigation, cross-track error / altitude PIDs, SD-card trajectory loader. Status: partial (controller present, SD loader missing, hard-coded test waypoints in `guidance_control.cpp:744-768`, cross-track error commented out at `:872`). **Required for v1.0.0.**
- **6.2 Advanced guidance control** — Stability monitor with grace period, failsafe escalation. Status: ✅ merged `a560c9a`; follow-up fix `8aeb589` aligned failsafe with graceful-degrade philosophy.
- **6.3 Production readiness** — `PowerManager`, edge-case handlers, `PreflightChecker`, thermal management. Status: **deferred to v1.1**. The architectural foundations (HAL, sensor redundancy, graceful degradation) already cover the safety basics; the remaining 6.3 work is operational polish, not safety, so it does not block a tagged v1.0.0.
- **6.4 Validation & release** — Integration-test suite, doc finalisation, v1.0.0 tag. Status: gated on 6.1 + telemetry + flight evidence — see [[queries/v1-release-gate-2026-05]].

## Outstanding Gaps (release-gate priority)

**Blocks v1.0.0:**

1. **Trajectory SD loading + cross-track error** — Replace hard-coded waypoints in `guidance_control.cpp:744-768`; implement waypoint-file parser; finish cross-track error calc at `:872` and wire into the PID.
2. **Live telemetry** — ESP32 transmitter/receiver firmware exists as stubs; Teensy-side `ENABLE_TELEMETRY` + Serial5 output not wired in. See [[entities/esp32-telemetry]].
3. **Flight validation** — Need ≥ 5 successful real flights on the v0.10.0+ firmware before tagging v1.0.0.

**Deferred to v1.1 (documented limitation, not blocker):**

- **Quaternion filter** — Kalman currently propagates via Euler angles; plan in `.archived/docs/QUATERNION_MIGRATION_PLAN.md`. Gimbal-lock zone above ±80° pitch is documented and unlikely to be reached pre-deployment for a near-vertical rocket. See [[concepts/kalman-filter]].
- **Phase 6.3 production readiness** — `PowerManager`, `PreflightChecker`, thermal management, edge-case handlers. Specified in `.archived/PRODUCTION_READINESS_PLAN.md`.

## Success Criteria for v1.0.0

- Phase 6.1 (trajectory) complete and merged.
- Telemetry path wired Teensy → ESP32 → ground; CSV-equivalent stream visible in [[entities/web-interface|web console]].
- ≥ 5 real flights on v0.10.0+ firmware with no critical regressions; flight logs reviewed and archived.
- ≥ 70 % overall code coverage; ≥ 95 % on flight-critical paths.
- Documentation (wiki) consistent with shipped code — see [[schema]] for the update protocol.
- Known limitations (Euler Kalman, no power management) explicitly documented in the v1.0.0 release notes.

Full gate detail: [[queries/v1-release-gate-2026-05]].

## Notes on Timeline

Document dates below are when the planning material was *written*, not when the feature shipped. Use `git log` to correlate with actual merge dates. This page itself should be refreshed whenever a phase completes — append to the Status column rather than rewriting history.

## Related

- [[overview]] — current architecture snapshot
- [[queries/code-review-findings-2026]] — quality audit that seeded this plan
- [[queries/development-status-2026-04]] — current gap ledger
- [[concepts/layered-architecture]] — the architectural target this roadmap serves
