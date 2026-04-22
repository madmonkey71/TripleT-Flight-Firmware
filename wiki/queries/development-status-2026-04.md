---
title: Development Status — 2026-04
type: query
tags: [status, gaps, current-state]
created: 2026-04-22
updated: 2026-04-22
related_files: [src/config.h, docs/DEVELOPMENT_STATUS.md, UPDATED_GAP_ANALYSIS_2025.md]
---

Point-in-time snapshot of what's merged, what's live, and what remains before v1.0.0. Grounded in `git log` as of **2026-04-22** and the shipping `FIRMWARE_VERSION = "v0.10.0"`.

## Shipped (production-ready for basic flight)

| Area | What works |
|------|-----------|
| MCU + sensors | Teensy 4.1 + ICM-20948 + KX134 + MS5611 + u-blox GNSS ([[entities/hardware-platform]]) |
| Architecture | Layered HAL → sensors → flight logic → guidance ([[concepts/layered-architecture]]) |
| State machine | 14 states with EEPROM persistence ([[concepts/flight-state-transitions]]) |
| Apogee detection | 2-of-3 voting (baro + accel + GPS) + 20 s backup timer ([[concepts/apogee-detection]]) |
| Sensor redundancy | ICM primary / KX134 backup with auto-failover ([[concepts/sensor-redundancy]]) |
| Orientation | Kalman AHRS (gyro + accel + mag), bias estimation ([[concepts/kalman-filter]]) |
| Pyro firing | Non-blocking drogue + main deploy |
| Error handling | Hard / soft error split; auto-recovery; `ErrorCode_t` ([[entities/error-handling]]) |
| Graceful degradation | Soft error 90 disables guidance without aborting flight ([[concepts/guidance-degradation]]) |
| Recovery | Audio beacon + LED strobe + GPS position in `RECOVERY` state |
| Data logging | 62-field CSV to SD card ([[concepts/data-logging]]) |
| Local visualisation | Web Serial console in `web_interface/` ([[entities/web-interface]]) |
| Testing | Unity native tests + GitHub Actions CI ([[concepts/testing-strategy]]) |

## Basic guidance (ready, needs flight data)

- PID loops (roll / pitch / yaw) wired and tunable.
- Attitude-hold mode present; captures orientation at burnout.
- Stability monitor with grace period and failsafe escalation (Phase 6.2 merged).

Flight validation is the gate: needs recorded flights for PID tuning confidence before turning guidance on by default.

## Outstanding (blocks v1.0.0)

| Gap | Owner area | Notes |
|-----|-----------|-------|
| Live telemetry bridge | [[entities/esp32-telemetry]] | ESP32 stubs exist; Teensy `ENABLE_TELEMETRY` / Serial5 not wired yet |
| Quaternion Kalman | [[concepts/kalman-filter]] | Plan in `docs/QUATERNION_MIGRATION_PLAN.md`; gimbal lock above ±80° pitch is a real risk |
| Trajectory SD loading | Phase 6.1 | Only hard-coded test trajectory; waypoint-file parser incomplete |
| `PowerManager` | Phase 6.3 | 4-mode power scheme specified; not yet implemented |
| Edge-case handlers | Phase 6.3 | GPS loss, wind-driven gain reduction, sensor saturation, EEPROM corruption |
| `PreflightChecker` | Phase 6.3 | 7-check runner + `preflight` command; spec done |
| Thermal management | Phase 6.3 | 70/85/95 °C thresholds, reactive throttling |
| Flight validation | Phase 6.4 | ≥ 5 good flights on v0.10.0+ |
| Test coverage | Phase 3+ | Infrastructure delivered; keep climbing toward 70 % / 95 % |

## Known Limitations (current code)

- Kalman propagates via Euler angles → avoid sustained pitch beyond ±80°.
- No SD-card-driven trajectory; only hard-coded.
- No wireless telemetry; USB-cable tether or SD post-flight only.
- Fixed-timestep not enforced; Kalman `dt` varies slightly with loop load.
- `ProcessFlightState()` is still monolithic; unit tests compensate.

## Recent Milestones

| Date (commit) | Change |
|---------------|--------|
| `ee7f7b6` | GPS-bug fixes merged (PR #12) |
| `770fa09` | Wiki initialised |
| `9a6700c` | Burnout detection: require 3 consecutive reads |
| `eecbf86` | Auto-calibrate on GPS fix + timeout fallback |
| `b50d754` | Teensy 4.1 SPI pin mapping corrected for GPS |
| `8aeb589` | Failsafe escalation aligned with graceful degradation |
| `a560c9a` | Phase 6.2 advanced guidance + stability monitor merged |
| `dd7360c` | Multi-path apogee + cross-validation (Phase 4) |
| `d797801` | Phase 3 testing infrastructure |
| `557f8c8` | Phase 5 doc suite |

## Next Actions (short list)

1. Wire `ENABLE_TELEMETRY` on Teensy and field-test the ESP-NOW link.
2. Either deliver the quaternion Kalman upgrade or add a compile-time guard that caps attitude at ±80° pitch in guidance.
3. Implement `PreflightChecker` — biggest single operational win per effort.
4. Backfill unit tests toward the 70 %/95 % targets; apogee-detection path should be ≥ 95 %.
5. Schedule 5 real flights to validate v0.10.0 end-to-end.

## Related

- [[queries/roadmap-2026]] — phase plan this snapshot tracks against
- [[queries/code-review-findings-2026]] — origin of the modernisation arc
- [[overview]] — architecture as shipped today
