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
