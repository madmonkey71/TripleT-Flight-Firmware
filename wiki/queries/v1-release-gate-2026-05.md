---
title: v1.0.0 Release Gate (decided 2026-05-25)
type: query
tags: [release, v1.0.0, gate, planning, trajectory, telemetry, flight-validation]
created: 2026-05-25
updated: 2026-07-02
related_files: [src/guidance_control.cpp, src/TripleT_Flight_Firmware.cpp, src/config.h, esp32_telemetry_transmitter/, esp32_ground_station_receiver/]
---

*Correction (2026-07-02): this snapshot's telemetry "state today" is stale — the Teensy telemetry module (`src/telemetry.h/.cpp`) and both ESP32 firmwares DO exist in the tree and are complete implementations, wired behind `ENABLE_TELEMETRY` (default 0). The remaining telemetry gaps are only the web-console `TELEM` parser and the end-to-end bench test. Additionally, "2-of-3 apogee voting" in this snapshot describes the dormant `ApogeeDetector` class; the live `detectApogee()` is OR/first-match across four methods. See [[queries/system-workflow-audit-2026-07]].*

The scope decision for v1.0.0 made on **2026-05-25** after the docs-consolidation review (PR #13). Phase 6.3 (power / thermal / pre-flight / edge-case) is deferred to **v1.1**; v1.0.0 ships when **trajectory + telemetry + flight validation** are done.

## Why this scope, not the full Phase 6

Phases 1–4 already deliver the safety floor that "production-ready" means for a flight computer: HAL isolation, sensor redundancy with auto-failover, 2-of-3 apogee voting + backup timer, EEPROM state persistence, graceful guidance degradation. The remaining Phase 6.3 work (`PowerManager`, `PreflightChecker`, thermal throttling, edge-case handlers) is operational polish — it makes long missions, low-battery scenarios, and ground-prep easier, but it does not affect whether the rocket flies safely.

Shipping v1.0.0 on **what's actually demonstrated to work** is more honest than holding the tag for a year of feature work that's never been flown.

## In scope for v1.0.0 (must ship)

### 1. Trajectory following (Phase 6.1)

**State today:**
- Controller present in `src/guidance_control.cpp` (PID loops, waypoint indexing, distance/bearing calc).
- Hard-coded test trajectory at `guidance_control.cpp:744-768` with explicit "IMPORTANT: These lat/lon are placeholders" comment.
- Cross-track error function commented out at `guidance_control.cpp:872-877`.
- `data_structures.h:89` has `// float cross_track_error_m; // for future` — type is reserved but unused.

**Acceptance for v1.0.0:**
- [ ] SD-card waypoint file format defined (CSV or JSON; CSV preferred for tooling compatibility with the existing log format).
- [ ] `guidance_load_trajectory_from_file(const char* filename)` implemented and replaces the hard-coded loader in normal use.
- [ ] `calculate_crosstrack_error_m()` implemented (great-circle cross-track formula) and wired into the XTE PID.
- [ ] Unit tests in `test/test_flight_logic/` or a new `test/test_trajectory/` covering: waypoint advance, distance/bearing, XTE sign convention, file-parse error paths.
- [ ] `load_trajectory` serial command (already specced in archived `PRODUCTION_READINESS_PLAN.md`).

### 2. Live telemetry (Teensy → ESP32 → ground)

**State today:**
- ESP32 transmitter/receiver firmware skeletons exist in `esp32_telemetry_transmitter/` and `esp32_ground_station_receiver/`.
- Packet layout specified in [[entities/esp32-telemetry]].
- Teensy side: no `ENABLE_TELEMETRY` symbol, no `Serial5.begin()`, no `sendTelemetryPacket()`.

**Acceptance for v1.0.0:**
- [ ] `ENABLE_TELEMETRY` compile flag in `src/config.h`.
- [ ] `Serial5.begin(115200)` in `setup()` behind that flag.
- [ ] `sendTelemetryPacket()` (or equivalent inline in `WriteLogData`) packs the subset defined in [[entities/esp32-telemetry]] and writes to `Serial5`. USB CSV path stays unchanged.
- [ ] ESP32 TX firmware: receives the packet on UART, broadcasts via ESP-NOW.
- [ ] ESP32 RX firmware: receives ESP-NOW, emits CSV-compatible line over USB.
- [ ] Ground-side: web console parses the radio CSV identically to the USB CSV (same `flight_console_data_mapping.json`).
- [ ] Bench test: bidirectional packet flight with telemetry visible in `web_interface/` over the radio link.

### 3. Flight validation

**State today:** unknown how many flights have been done on v0.10.0; need explicit log.

**Acceptance for v1.0.0:**
- [ ] **5+ successful flights** on v0.10.0+ firmware, with SD logs reviewed and stored under a `flight_logs/` directory (or external archive linked from here).
- [ ] Each flight log post-processed: apogee detected within ±2 m of GPS-confirmed peak; main parachute deployed at expected altitude; landed state captured; no spurious ERROR state transitions.
- [ ] At least one flight with intentional sensor degradation (e.g., GPS antenna disconnected at altitude) to validate failover.
- [ ] At least one flight with the new trajectory feature active (waypoint navigation engaged in COAST), even if just a 2-3 waypoint demo.

## Deferred to v1.1 (explicitly NOT blocking v1.0.0)

| Item | Reason for deferral |
|------|--------------------|
| Quaternion Kalman migration | Documented limitation; gimbal-lock zone (±80° pitch) unlikely pre-deployment for a near-vertical rocket. See [[concepts/kalman-filter]]. |
| `PowerManager` (4 modes) | Operational optimisation, not safety; current power draw acceptable for typical flight durations. |
| `PreflightChecker` | Manual checklist suffices for v1.0.0; automation is a quality-of-life win. |
| Thermal management | Teensy 4.1 + sensors operate within spec at typical ambient; throttling protects edge cases, not nominal flight. |
| Edge-case handlers (GPS loss / high wind / sensor saturation / EEPROM corruption) | Existing redundancy (sensor failover, multi-path apogee, EEPROM-on-state-change) covers most cases. Explicit handlers are belt-and-braces for v1.1. |

All deferred items remain documented; the v1.0.0 release notes will list them explicitly as known limitations.

## Refactoring debt (parallel track, not blocking)

These do not block v1.0.0 but should be addressed during the v1.0.0 development window to reduce conflict risk while editing flight code:

- **Split `ProcessFlightState()`** in `src/flight_logic.cpp:111-478` into per-state handlers. Mechanical refactor; each `case` block becomes `handle_<state>()`.
- **Split `TripleT_Flight_Firmware.cpp`** (1063 lines) at the seams between main loop, logging, and sensor scheduling.
- **Fill log placeholders** at `TripleT_Flight_Firmware.cpp:394-395` ("logging placeholders (0.0f)") — either implement the getters or remove the unused fields.

## Suggested sequencing

1. **Telemetry first** (smaller scope, well-defined spec, unblocks ground-station development). ~1-2 days of Teensy-side work.
2. **Trajectory SD loader + XTE** in parallel with telemetry, or immediately after. ~2-3 days including tests.
3. **Bench-test both together** with the web console and recorded flight data replay before any real flight.
4. **Start flight validation.** Five flights is a multi-weekend programme even with perfect weather. Begin scheduling as soon as bench tests are green.

## Acceptance gate (binary)

v1.0.0 ships when all of:

- [ ] Trajectory SD loader + XTE implemented, tested, demonstrated in bench replay.
- [ ] Telemetry round-trip works end-to-end (Teensy → ESP32 TX → ESP-NOW → ESP32 RX → web console).
- [ ] ≥ 5 successful flights logged and post-processed.
- [ ] Release notes drafted, including the deferred-to-v1.1 list.
- [ ] `FIRMWARE_VERSION` bumped in `src/config.h`; git tag created on `master`.

## Related

- [[queries/roadmap-2026]] — full phase plan
- [[queries/development-status-2026-04]] — current gap ledger (reflects this scope decision)
- [[entities/esp32-telemetry]] — telemetry design spec
- [[concepts/kalman-filter]] — documented quaternion deferral
- [[concepts/developer-workflow]] — branch / release flow for the v1.0.0 tag
