# TripleT Flight Firmware — Wiki Index

Content catalog for the project wiki. See [overview.md](overview.md) for the full architecture overview.

## Overview
- [overview.md](overview.md) — Full architecture, tech stack, directory layout, state machine diagram, key decisions
- [schema.md](schema.md) — Wiki conventions, page types, update protocol

## Concepts — Architectural Patterns

- [concepts/hal-abstraction.md](concepts/hal-abstraction.md) — 8 pure-virtual HAL interfaces enabling desktop testing without Teensy hardware
- [concepts/sensor-redundancy.md](concepts/sensor-redundancy.md) — IMUInterface adapters, IMUManager automatic failover (ICM-20948 → KX134)
- [concepts/apogee-detection.md](concepts/apogee-detection.md) — 2-of-3 voting (baro + accel + GPS) plus 20s backup timer
- [concepts/guidance-degradation.md](concepts/guidance-degradation.md) — Graceful guidance disable on stability violation; preserves parachute deploy
- [concepts/kalman-filter.md](concepts/kalman-filter.md) — Kalman AHRS: gyro predict + accel/mag update, bias estimation
- [concepts/data-logging.md](concepts/data-logging.md) — LogData struct, CSV on SD card, Web Serial dashboard, ESP32 telemetry
- [concepts/testing-strategy.md](concepts/testing-strategy.md) — Unity native tests, ArduinoFake mocks, 3-tier strategy, CI/CD

## Entities — Module Reference

- [entities/flight-logic.md](entities/flight-logic.md) — State machine dispatcher, transition thresholds, detectApogee/Landing/BoostEnd
- [entities/guidance-control.md](entities/guidance-control.md) — PID loops, trajectory following, stability monitor, servo smoothing
- [entities/state-management.md](entities/state-management.md) — EEPROM persistence of flight state, boot recovery
- [entities/command-processor.md](entities/command-processor.md) — Serial command interface, arm/calibrate/status commands, debug flags

## Operation Log
- [log.md](log.md) — Chronological record of all wiki operations
