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

---

## 2026-05-25 — CONSOLIDATE (move sources to .archived/)

**Operation**: Consolidate all project `.md` files under `wiki/`. Move already-integrated and superseded sources to `.archived/` to leave a single coherent documentation tree.
**Branch**: develop
**Firmware version**: v0.10.0

**Decisions confirmed with user**:
- Keep `README.md` and `AI.md` in project root.
- Move `CLAUDE.md` into wiki (ADRs extracted to new page; the rest archived). NOTE: Claude Code will no longer auto-load `CLAUDE.md` from root; project-level LLM guidance now lives in `AI.md` + `wiki/`.
- Keep component-specific READMEs in place (`esp32_*/README.md`, `web_interface/README.md`, `web_interface/SYSTEM_STATUS.md`, `test/README.md`).
- Archive originals of already-integrated docs (`.archived/` rather than delete).
- Archive stale/superseded files (`.archived/`).

**New wiki content**:
- `wiki/concepts/architecture-decisions.md` — ADR-001..ADR-005 extracted from `CLAUDE.md`.
- `wiki/concepts/developer-workflow.md` — added Common Development Tasks, Quick Decision Matrix ("what goes where"), Branch & Release Workflow sections.

**Drift fixed during this pass**:
- Test layout: `test/unit/*.cpp` (claimed) → `test/test_<name>/test_<name>.cpp` (actual PlatformIO Unity discovery layout).
- Test env name: `native_test` (claimed) → `native` (actual env in `platformio.ini`; CI runs `pio test -e native -vv`).
- Test compile flag: noted both `-DUNIT_TEST_NATIVE` (platformio.ini) and `-DNATIVE_TEST_BUILD` (referenced in `src/hal/hal_factory.h`).
- Test suite list updated to actual 11 suites (added: `flight_logic`, `landing_detection`, `sensor_health`, `gps_validation`, `altitude_calculations`).
- Updated `overview.md`, `concepts/testing-strategy.md`, `concepts/developer-workflow.md`.
- Updated wiki internal references that pointed at root/`docs/` paths to use `.archived/...` so they still resolve.

**Files moved to `.archived/`** (root-level, 29 files):
`ACTUATOR_VISUALIZATION_FIX.md`, `BENCH_TEST_PROCEDURE.md`, `CLAUDE.md`, `CODE_REVIEW_FINDINGS_2026.md`, `COMPETITOR_ANALYSIS.md`, `COMPREHENSIVE_TESTING_STRATEGY.md`, `EXECUTIVE_SUMMARY_2026.md`, `Firmware Function Documentation.md`, `FIX_WALKTHROUGH.md`, `FLIGHT_PROFILE_H125W_SUMMARY.md`, `FLIGHT_TEST_PREPARATION.md`, `IMPLEMENTATION_PLAN_2026.md`, `PHASE_6.3_INTEGRATION_GUIDE.md`, `PHASE_6.3_README.md`, `PHASE_6_4_INDEX.md`, `PHASE_6_4_QUICK_REFERENCE.md`, `PHASE_6_4_SUMMARY.md`, `PHASE_6_4_TEST_IMPLEMENTATION_GUIDE.md`, `PHASE_6_4_TESTING_FRAMEWORK.md`, `PHASE_6_PLAN.md`, `PRODUCTION_READINESS_PLAN.md`, `PRODUCTION_READINESS_SUMMARY.md`, `TESTING_DOCUMENTATION_INDEX.md`, `TESTING_IMPLEMENTATION_GUIDE.md`, `TESTING_QUICK_REFERENCE.md`, `TripleT Flight Firmware Function Documentation.md`, `TripleT UKF Documentation.md`, `UPDATED_GAP_ANALYSIS_2025.md`, `USER_GUIDE.md`.

**Files moved to `.archived/docs/`** (19 files, then empty `docs/` removed):
`ARCHITECTURE.md`, `COMMANDS.md`, `CONFIGURATION.md`, `DEVELOPER_GUIDE.md`, `DEVELOPMENT_STATUS.md`, `DOCS_MAINTENANCE.md`, `ERROR_CODES.md`, `Feature_Usage_And_Configuration.md`, `FLIGHT_STATE_MACHINE.md`, `GETTING_STARTED.md`, `HARDWARE.md`, `PHASE_6_6_DOCUMENTATION_PLAN.md`, `QUATERNION_MIGRATION_PLAN.md`, `SAFETY.md`, `SENSOR_EVALUATION.md`, `STM32_MIGRATION_ANALYSIS.md`, `TELEMETRY_IMPLEMENTATION_PLAN.md`, `TESTING.md`, `TripleT_Flight_Firmware_Documentation.md`.

**README.md & AI.md edits (root, intentionally not moved)**:
- README "Project Status & Roadmap" and "Documentation" sections now point at `wiki/` pages instead of removed `IMPLEMENTATION_PLAN_2026.md` / `PHASE_6_PLAN.md` / `docs/*.md`.
- AI.md "Key Files to Know", directory layout, "Starting Work", "Before Committing", "Asking for Help", and footer all updated to point at `wiki/`; test command corrected to `pio test -e native -vv`.

**Final state**:
- Root: `README.md`, `AI.md`, `MEMORY.md` (+ project files). No other root-level `.md`.
- `docs/` directory removed.
- All architectural / conceptual / planning docs live under `wiki/`.
- Component-specific READMEs (`esp32_*`, `web_interface/`, `test/`) untouched.
- Historical/superseded docs preserved under `.archived/` and `.archived/docs/`.

---

## 2026-05-25 — SCOPE DECISION (v1.0.0 release gate)

**Operation**: Capture the v1.0.0 release-gate decision in the wiki, fix stale references uncovered during the post-PR-#13 review.
**Branch**: docs/v1-release-gate
**Firmware version**: v0.10.0

**Decision summary** (from the code review report on PR #13):

- v1.0.0 ships when **trajectory (6.1) + live telemetry + 5 flight validations** are done.
- Phase 6.3 (`PowerManager`, `PreflightChecker`, thermal management, edge-case handlers) **deferred to v1.1**. Existing redundancy/safety layers already meet the bar; 6.3 is operational polish, not a safety gate.
- Quaternion Kalman migration stays deferred; Euler-state gimbal-lock zone (±80° pitch) documented as a known v1.0.0 limitation.

**Wiki changes**:
- New: `wiki/queries/v1-release-gate-2026-05.md` — explicit acceptance criteria for trajectory SD loader + XTE, telemetry round-trip, flight validation; deferred-list with rationale; refactoring-debt parallel track.
- Updated `wiki/queries/roadmap-2026.md` — Phase 5 marked ✅; Phase 6 split into v1.0.0 vs v1.1 lines; success criteria rewritten around the gate.
- Updated `wiki/queries/development-status-2026-04.md` — outstanding-gaps table split into "blocks v1.0.0" vs "deferred to v1.1"; Next Actions reordered to gate priority.
- Updated `wiki/index.md` — added the new gate page at the top of Queries.

**MEMORY.md reference cleanup**:
- `AI.md` referenced `MEMORY.md` as if it lived at the project root. Actual location is `~/.claude/projects/-mnt-GAMES-SSD-matt-Code-TripleT-Flight-Firmware/memory/MEMORY.md` (Claude Code auto-memory). Updated all 6 references in `AI.md` and the closing line of `CLAUDE.md` to point at the real path.

**Not changed in this pass**:
- No source code edits. The trajectory + telemetry implementation work is the next step (see [[queries/v1-release-gate-2026-05]] for the action list).


