# TripleT Flight Firmware — Wiki Index

Content catalog for the project wiki. See [overview.md](overview.md) for the architecture overview and [schema.md](schema.md) for page conventions.

## Overview

- [overview.md](overview.md) — Architecture, tech stack, directory layout, state machine, phase progression, known limitations
- [schema.md](schema.md) — Page types, frontmatter, cross-reference conventions

## Concepts — Architectural Patterns

- [concepts/layered-architecture.md](concepts/layered-architecture.md) — HAL → sensors → flight logic → guidance; dependency rules
- [concepts/architecture-decisions.md](concepts/architecture-decisions.md) — ADRs: HAL, IMUInterface, apogee voting, EEPROM persistence, Kalman
- [concepts/hal-abstraction.md](concepts/hal-abstraction.md) — 8 pure-virtual HAL interfaces enabling desktop testing
- [concepts/sensor-redundancy.md](concepts/sensor-redundancy.md) — IMUInterface adapters, IMUManager primary/backup failover
- [concepts/sensor-evaluation.md](concepts/sensor-evaluation.md) — Sensor selection rationale and alternatives (BNO085, ICM-20649, BMP388)
- [concepts/apogee-detection.md](concepts/apogee-detection.md) — 2-of-3 voting (baro + accel + GPS) + 20 s backup timer
- [concepts/flight-state-transitions.md](concepts/flight-state-transitions.md) — Transition table, guards, timing, edge cases
- [concepts/guidance-degradation.md](concepts/guidance-degradation.md) — Soft error 90; guidance disabled without aborting flight
- [concepts/kalman-filter.md](concepts/kalman-filter.md) — Kalman AHRS; current Euler state; quaternion migration plan
- [concepts/calibration.md](concepts/calibration.md) — Barometer, gyro, magnetometer calibration procedures
- [concepts/system-robustness.md](concepts/system-robustness.md) — 4-layer defence, SPF ledger, failure-scenario walkthroughs
- [concepts/data-logging.md](concepts/data-logging.md) — LogData, 62-field CSV, SD card, post-flight analysis
- [concepts/testing-strategy.md](concepts/testing-strategy.md) — Unity native, ArduinoFake, 3-tier mocks, CI/CD, coverage targets
- [concepts/developer-workflow.md](concepts/developer-workflow.md) — Build, test, flash, debug; common dev tasks; branch & release flow

## Entities — Module & Component Reference

- [entities/hardware-platform.md](entities/hardware-platform.md) — Teensy 4.1, sensors, I2C, SD, pyro, actuators
- [entities/configuration-system.md](entities/configuration-system.md) — `config.h`, compile flags, `debug_flags.h`
- [entities/flight-logic.md](entities/flight-logic.md) — State-machine dispatcher, transition thresholds
- [entities/state-management.md](entities/state-management.md) — EEPROM persistence, boot recovery
- [entities/guidance-control.md](entities/guidance-control.md) — PID, trajectory, stability monitor, servo smoother
- [entities/command-processor.md](entities/command-processor.md) — Serial command catalogue, debug toggles
- [entities/error-handling.md](entities/error-handling.md) — `ErrorCode_t` taxonomy, hard vs soft errors, recovery
- [entities/web-interface.md](entities/web-interface.md) — Browser console, Web Serial API, CSV parser
- [entities/esp32-telemetry.md](entities/esp32-telemetry.md) — Planned ESP-NOW wireless bridge

## Queries — Time-Stamped Snapshots

- [queries/roadmap-2026.md](queries/roadmap-2026.md) — Phase plan v0.6.0 → v1.0.0 (status as of 2026-04-22)
- [queries/code-review-findings-2026.md](queries/code-review-findings-2026.md) — Feb-2026 audit on v0.51 baseline (archival)
- [queries/development-status-2026-04.md](queries/development-status-2026-04.md) — Shipped / outstanding / next actions

## Operation Log

- [log.md](log.md) — Chronological record of all wiki operations
