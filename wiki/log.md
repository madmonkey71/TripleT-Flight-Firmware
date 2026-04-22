# Wiki Operation Log

Append-only chronological record of all wiki operations.

---

## 2026-04-15 — INIT

**Operation**: Initial wiki creation  
**Branch**: fix/gps-bugs-new  
**Firmware version**: v0.10.0 (Safety Features phase complete)

**Files created**:
- `wiki/overview.md` — Full architecture overview, state machine diagram, tech stack, directory layout
- `wiki/schema.md` — Conventions and update protocol
- `wiki/index.md` — Content catalog
- `wiki/log.md` — This file
- `wiki/concepts/hal-abstraction.md` — HAL interfaces and singleton pattern
- `wiki/concepts/sensor-redundancy.md` — IMUInterface, adapters, IMUManager failover
- `wiki/concepts/apogee-detection.md` — 2-of-3 voting + backup timer
- `wiki/concepts/guidance-degradation.md` — Graceful guidance disable, failsafe escalation
- `wiki/concepts/kalman-filter.md` — Kalman AHRS, state vector, API
- `wiki/concepts/data-logging.md` — LogData, CSV, web interface, ESP32 telemetry
- `wiki/concepts/testing-strategy.md` — Unity native, ArduinoFake, 3-tier strategy
- `wiki/entities/flight-logic.md` — State machine, transition thresholds
- `wiki/entities/guidance-control.md` — PID, trajectory, stability, servos
- `wiki/entities/state-management.md` — EEPROM persistence
- `wiki/entities/command-processor.md` — Serial commands

**Sources consulted**: CLAUDE.md, MEMORY.md, src/config.h, src/data_structures.h, src/flight_logic.h, src/guidance_control.h, src/stability_monitor.h, src/kalman_filter.h, src/hal/hal_interfaces.h, platformio.ini, git log

---

## 2026-04-22 — INTEGRATE (doc sweep)

**Operation**: Integrate project documentation (root-level `.md`/`.txt` and `docs/`) into the wiki
**Branch**: develop
**Firmware version**: v0.10.0
**Method**: 4 parallel subagents reviewed document clusters (user/ops, architecture, testing, project plans) and returned structured proposals; this session synthesised them into new pages plus updates.

**Files created**:
- `wiki/entities/hardware-platform.md` — Teensy 4.1, sensors, I2C, SD, pyro
- `wiki/entities/configuration-system.md` — `config.h` categories, compile flags, debug flags
- `wiki/entities/error-handling.md` — `ErrorCode_t` table, hard vs soft, recovery
- `wiki/entities/web-interface.md` — Web Serial console, parser, dashboard
- `wiki/entities/esp32-telemetry.md` — planned ESP-NOW bridge
- `wiki/concepts/layered-architecture.md` — HAL → sensors → flight → guidance
- `wiki/concepts/flight-state-transitions.md` — transition reference
- `wiki/concepts/sensor-evaluation.md` — selection rationale & alternatives
- `wiki/concepts/calibration.md` — baro/gyro/mag procedures
- `wiki/concepts/system-robustness.md` — 4-layer defence, SPF ledger
- `wiki/concepts/developer-workflow.md` — build/test/flash cycle
- `wiki/queries/roadmap-2026.md` — phase plan + status
- `wiki/queries/code-review-findings-2026.md` — archival audit
- `wiki/queries/development-status-2026-04.md` — current gap ledger

**Files updated**:
- `wiki/overview.md` — phase progression, known limitations, related-page cross-refs
- `wiki/concepts/data-logging.md` — expanded CSV details, web-interface / ESP32 cross-refs, post-flight analysis
- `wiki/concepts/kalman-filter.md` — note on Euler state + planned quaternion migration
- `wiki/concepts/testing-strategy.md` — CI/CD, flight-critical coverage targets
- `wiki/entities/command-processor.md` — expanded catalogue, named debug commands, planned Phase 6.3 commands
- `wiki/schema.md` — clarified `queries/` convention (time-stamped snapshots)
- `wiki/index.md` — rebuilt catalogue with all new pages

**Docs reviewed (representative)**: `README.md`, `USER_GUIDE.md`, `docs/ARCHITECTURE.md`, `docs/HARDWARE.md`, `docs/COMMANDS.md`, `docs/CONFIGURATION.md`, `docs/SAFETY.md`, `docs/ERROR_CODES.md`, `docs/FLIGHT_STATE_MACHINE.md`, `docs/SENSOR_EVALUATION.md`, `docs/TELEMETRY_IMPLEMENTATION_PLAN.md`, `docs/QUATERNION_MIGRATION_PLAN.md`, `docs/STM32_MIGRATION_ANALYSIS.md`, `docs/DEVELOPER_GUIDE.md`, `docs/TESTING.md`, `docs/DEVELOPMENT_STATUS.md`, `docs/Feature_Usage_And_Configuration.md`, `IMPLEMENTATION_PLAN_2026.md`, `CODE_REVIEW_FINDINGS_2026.md`, `PRODUCTION_READINESS_PLAN.md`, `UPDATED_GAP_ANALYSIS_2025.md`, `PHASE_6*`, `BENCH_TEST_PROCEDURE.md`, `FLIGHT_TEST_PREPARATION.md`, `TESTING_*`, `web_interface/README.md`, `esp32_*/README.md`, `src/error_codes.h`, `src/config.h`, `git log`.

**Docs deemed stale / skip / out-of-scope for the wiki**:
- `TripleT UKF Documentation.md` — UKF experimental (`src/ukf.cpp`); production filter is Kalman (see `concepts/kalman-filter.md`)
- `COMPETITOR_ANALYSIS.md` — market intel, not firmware architecture
- `FLIGHT_PROFILE_H125W_SUMMARY.md` — single-flight test case; better archived with flight data
- `ACTUATOR_VISUALIZATION_FIX.md` — narrow bugfix note
- `Firmware Function Documentation.md` / `TripleT Flight Firmware Function Documentation.md` — pre-dates Phases 1-5; superseded by entity pages
- `TESTING_DOCUMENTATION_INDEX.md`, `TESTING_QUICK_REFERENCE.md`, `TESTING_STRATEGY_SUMMARY.txt` — meta/duplicates of `COMPREHENSIVE_TESTING_STRATEGY.md` and `docs/TESTING.md`
- `PHASE_6_4_INDEX.md` and other redundant PHASE_6_4 variants — planning churn; essentials captured in `queries/roadmap-2026.md`
- `FIX_WALKTHROUGH.md` — minimal / dated
- `AI.md` — workflow meta, not functional spec
