---
title: Development Roadmap (v0.6.0 → v1.0.0)
type: query
tags: [roadmap, phases, planning, versions]
created: 2026-04-22
updated: 2026-04-22
related_files: [IMPLEMENTATION_PLAN_2026.md, EXECUTIVE_SUMMARY_2026.md, src/config.h]
---

Snapshot of the 2026 roadmap taking TripleT from a refactoring baseline at v0.6.0-dev to a v1.0.0 production release. Phase structure is per `IMPLEMENTATION_PLAN_2026.md`; status fields reflect what is merged as of **2026-04-22** (verified against `git log`).

## Phase Summary

| Phase | Target version | Scope | Status |
|-------|----------------|-------|--------|
| 1 — HAL Foundation | v0.6.0 → v0.7.0 | Pure-virtual interfaces (ITimer, ISerial, IGPIO, II2C, IEEPROM, ISDCard, IServo, IWatchdog); Teensy + Mock impls | ✅ merged |
| 2 — Sensor Modularity | v0.7.0 → v0.8.0 | `IMUInterface`; ICM/KX134/BNO085 adapters; `IMUManager`; sensor factory | ✅ merged |
| 3 — Testing Infrastructure | v0.8.0 → v0.9.0 | Unity native tests, GitHub Actions, 3-tier mock strategy | ✅ merged (d797801) |
| 4 — Safety & Redundancy | v0.9.0 → v0.10.0 | Multi-path apogee, cross-validation, watchdog recovery, graceful guidance degrade | ✅ merged (dd7360c, f121c3f, 8aeb589) |
| 5 — Documentation & Polish | v0.10.0 → v1.0.0-rc1 | Developer / user / safety guides; hardware eval; release notes | ⏳ partial (557f8c8); this wiki integration continues that work |
| 6 — Advanced features + production readiness | toward v1.0.0 | Sub-phases 6.1 (trajectory), 6.2 (guidance stability), 6.3 (power/edge-cases/pre-flight), 6.4 (validation release) | 🟡 6.2 merged (a560c9a); 6.1, 6.3, 6.4 in spec/progress |

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

### Phase 5 — Documentation & Polish ⏳ partial
Merged: `docs/` suite (Architecture, Developer Guide, Safety, Configuration, Commands, etc.) plus initial wiki bootstrap. Outstanding: integrate remaining docs into wiki (in progress via this update); hardware-evaluation narrative; release notes authoring.

### Phase 6 — Advanced Features + Production Readiness 🟡

- **6.1 Trajectory following** — Waypoint navigation, cross-track error / altitude PIDs, SD-card trajectory loader. Status: partial (controller present, SD loader incomplete).
- **6.2 Advanced guidance control** — Stability monitor with grace period, failsafe escalation. Status: ✅ merged `a560c9a`; follow-up fix `8aeb589` aligned failsafe with graceful-degrade philosophy.
- **6.3 Production readiness** — `PowerManager` (ACTIVE / COAST_OPT / RECOVERY / SLEEP modes), edge-case handlers (GPS loss, high wind, sensor saturation), `PreflightChecker`, thermal management. Status: specified in `PRODUCTION_READINESS_PLAN.md`; implementation pending.
- **6.4 Validation & release** — Integration-test suite, doc finalisation, v1.0.0 tag. Status: specified; gated on 6.3.

## Outstanding Gaps (high-priority)

1. **Live telemetry** — ESP32 transmitter/receiver firmware exists as stubs; Teensy-side `ENABLE_TELEMETRY` + Serial5 output not wired in. See [[entities/esp32-telemetry]].
2. **Quaternion filter** — Kalman currently propagates via Euler angles; plan in `docs/QUATERNION_MIGRATION_PLAN.md` would eliminate gimbal lock at ±90° pitch. See [[concepts/kalman-filter]].
3. **Trajectory SD loading** — Waypoint file parser incomplete; only hard-coded test trajectory works today.
4. **Flight validation** — Need ≥ 5 successful real flights on the v0.10.0+ firmware before tagging v1.0.0.

## Success Criteria for v1.0.0

- All sub-phases of 6 complete and merged.
- ≥ 70 % overall code coverage; ≥ 95 % on flight-critical paths.
- 5+ real flights with no critical regressions.
- Documentation (wiki + `docs/`) consistent with shipped code — see `DOCS_MAINTENANCE.md`.

## Notes on Timeline

Document dates below are when the planning material was *written*, not when the feature shipped. Use `git log` to correlate with actual merge dates. This page itself should be refreshed whenever a phase completes — append to the Status column rather than rewriting history.

## Related

- [[overview]] — current architecture snapshot
- [[queries/code-review-findings-2026]] — quality audit that seeded this plan
- [[queries/development-status-2026-04]] — current gap ledger
- [[concepts/layered-architecture]] — the architectural target this roadmap serves
