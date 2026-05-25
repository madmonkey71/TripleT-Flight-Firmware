---
title: Architecture Decision Records
type: concept
tags: [adr, architecture, decisions, history]
created: 2026-05-25
updated: 2026-05-25
related_files: [src/hal/hal_interfaces.h, src/sensors/imu_interface.h, src/flight_logic.cpp, src/state_management.cpp, src/kalman_filter.cpp]
---

Major architectural decisions and their rationale. Each ADR records *what was chosen*, *why*, *what alternatives were rejected*, and *the resulting trade-offs*. Source code is the final truth; this page exists so future contributors don't relitigate decisions whose context has decayed.

## ADR-001: Hardware Abstraction Layer (HAL)

**Date:** 2026-02-15
**Status:** ACCEPTED (shipped in v0.7.0)

**Decision:** Introduce a HAL abstraction layer — `ITimer`, `ISerial`, `IGPIO`, `II2C`, `IEEPROM`, `ISDCard`, `IServo`, `IWatchdog`.

**Rationale:**
- Enable desktop testing without hardware.
- Support platform migration (Teensy → STM32) without rewriting flight logic.
- Keep hardware concerns localised; upper layers never touch Arduino APIs directly.

**Implementation:** `src/hal/hal_interfaces.h`. See [[concepts/hal-abstraction]].

**Alternatives considered:**
- Direct Arduino API calls — *rejected*: not testable, couples flight logic to hardware.
- HAL generator tool — *rejected*: over-engineering for 8 interfaces.

**Consequences:**
- One extra layer per hardware call (virtual dispatch overhead is negligible — see [[concepts/hal-abstraction]]).
- All hardware interaction must route through HAL (discipline required).
- Adding a mock for a new test is trivial.

## ADR-002: Sensor Interface Pattern (IMUInterface)

**Date:** 2026-02-15
**Status:** ACCEPTED (shipped in v0.8.0)

**Decision:** All motion sensors implement a common `IMUInterface`; an `IMUManager` runs primary-with-fallback redundancy at runtime.

**Rationale:**
- Compile-time sensor swap (ICM-20948 ↔ BNO085) without flight-logic edits.
- Automatic failover when a sensor degrades, without conditional code at call sites.

**Implementation:** `src/sensors/imu_interface.h`, `src/sensors/imu_manager.h`. See [[concepts/sensor-redundancy]].

**Alternatives considered:**
- Compile-time template specialisation — *rejected*: too complex for marginal benefit.
- Runtime polymorphism only (no manager) — partially accepted; the manager wraps polymorphic adapters.

**Consequences:**
- Virtual-call overhead (~2-3 %, well under sensor-read budget).
- Adding a new sensor is mechanical: implement 20 methods, register in `sensor_factory`.

## ADR-003: Multi-Method Apogee Detection (2-of-3 voting)

**Date:** 2026-02-15
**Status:** ACCEPTED (shipped in v0.10.0)

**Decision:** Apogee is declared when at least 2 of 3 independent methods (barometric, accelerometer, GPS) agree, with a 20 s backup timer as final failsafe.

**Rationale:**
- Single-sensor failure does not block apogee detection.
- Noisy data is filtered by the consensus requirement.
- Backup timer guarantees deployment even if every sensor degrades.

**Implementation:** `src/flight_logic.cpp` (apogee detection block). See [[concepts/apogee-detection]].

**Alternatives considered:**
- Single-sensor — *rejected*: insufficient redundancy for a safety-critical event.
- 3-of-3 unanimous — *rejected*: too strict; one bad sensor blocks deployment.
- Any-1-of-3 (OR) — *rejected*: too lenient; a single false positive deploys early.

**Consequences:**
- Slightly delayed apogee declaration (wait for consensus, ~50-100 ms).
- Very robust to single-sensor noise or failure.
- Backup timer is the last-line guarantee even with total sensor loss.

## ADR-004: State Persistence in EEPROM

**Date:** 2026-02-15
**Status:** ACCEPTED (shipped in v0.10.0)

**Decision:** Save flight state to EEPROM on every state transition.

**Rationale:**
- Power-loss recovery: resume flight from the most recent state, not from STARTUP.
- Watchdog-reset recovery: don't lose track of the flight phase across a reset.

**Implementation:** `src/state_management.cpp`. See [[entities/state-management]].

**Alternatives considered:**
- No persistence — *rejected*: power glitch ⇒ flight lost.
- Periodic save — *rejected*: could miss a state change.
- Save on every sensor read — *rejected*: EEPROM wear.

**Consequences:**
- EEPROM wear is bounded (~1k transitions per flight, EEPROM rated > 100k writes).
- Small write overhead per state change (acceptable).
- Survives power interruptions and watchdog resets.

## ADR-005: Kalman Filter for Orientation

**Date:** 2026-02-15
**Status:** ACCEPTED (replaces deprecated Madgwick)

**Decision:** Use a Kalman-based AHRS for orientation estimation. Madgwick complementary filter retired.

**Rationale:**
- Better gyro + accel fusion than the complementary filter.
- Tunable process / measurement noise (`Q`, `R`) — important for safety-critical use.
- Compatible with planned quaternion migration (see [[concepts/kalman-filter]]).

**Implementation:** `src/kalman_filter.cpp`. See [[concepts/kalman-filter]].

**Alternatives considered:**
- Madgwick — *rejected*: less accurate, harder to certify.
- Extended Kalman (EKF) — *rejected*: complexity not justified yet.
- No fusion — *rejected*: raw gyro drifts.

**Consequences:**
- Slightly higher per-loop CPU cost (acceptable on Cortex-M7).
- Requires `Q`/`R` tuning from real flight data.
- Current implementation uses an Euler state vector — quaternion migration is planned to avoid gimbal lock above ±80° pitch.

## Related

- [[overview]] — high-level architecture this set of decisions produces
- [[concepts/layered-architecture]] — the dependency model implied by ADR-001 and ADR-002
- [[concepts/system-robustness]] — how ADR-003 and ADR-004 combine into the 4-layer defence
- [[queries/roadmap-2026]] — which ADRs landed in which version
