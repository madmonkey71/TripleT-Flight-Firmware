---
title: state_management — EEPROM Persistence & Boot Recovery
type: entity
tags: [state, eeprom, persistence, recovery, pyro-safety]
created: 2026-04-15
updated: 2026-09-30
related_files: [src/state_management.cpp, src/state_management.h, src/startup_state.cpp, src/pyro_control.cpp, src/data_structures.h, src/flight_commands.cpp]
---

Persists the flight state, key altitudes, the barometer calibration reference and — since beta-0.58 — the flight-safety flags to EEPROM, and decides at boot whether a saved in-flight state may be **resumed**. Reworked in beta-0.58 ([[queries/flight-logic-audit-2026-09]] #1, #3, #6, #7): stale EEPROM can no longer fire a pyro at power-up.

## EEPROM Structure

Stored at `EEPROM_STATE_ADDR` (0); a `static_assert` keeps it clear of the magnetometer calibration block at `MAG_CAL_EEPROM_ADDR` (100).

```cpp
struct FlightStateData {
  uint8_t state;                 // FlightState enum cast to uint8_t
  float launchAltitude, maxAltitude, currentAltitude, mainDeployAltitudeAgl;
  float baroAltitudeOffset;      // GPS-referenced barometer calibration offset (m)
  uint8_t baroCalibrated;
  uint8_t flightInProgress;      // set at BOOST entry; cleared ONLY by reset_flight
  uint8_t pyroFiredMask;         // bit0 drogue window completed, bit1 main
  uint8_t resumeCount;           // in-flight resumes since launch
  uint32_t burnoutAgeMs;         // ms since burnout at save time (COAST), else 0
  unsigned long timestamp;       // uptime at save: NOT comparable across reboots, debug only
  uint16_t signature;            // EEPROM_SIGNATURE_VALUE
};
```

Layout changes shift the signature offset, which **intentionally invalidates records from older firmware** (they boot fresh from `STARTUP`).

## When State is Saved

- `saveStateToEEPROM()` runs on **every** state change, every safety-flag change (BOOST entry, pyro window completion, recovery decisions) and after `reset_flight`. It is **not throttled** (the old 60 s `EEPROM_UPDATE_INTERVAL` is gone) and is **put-if-changed**: if nothing but the uptime timestamp differs from what is stored, no flash write happens.
- `saveFlightProgressPeriodic()` (every loop pass) refreshes the record every `EEPROM_PROGRESS_SAVE_INTERVAL_MS` (1 s) while in `BOOST`/`COAST` so a reset resumes with a current max altitude and burnout age.
- Wear: ~13 transitions + ~1 write/s for the ~15–60 s of BOOST/COAST, per-byte update semantics — well under 100 k cycles for thousands of flights. **Bench-check** that the occasional flash re-organisation stall does not disturb the loop.

## Boot Recovery (two phases)

**Phase 1 — `recoverFromPowerLoss()`** (early in `setup()`, no sensors yet): validates the signature; restores `g_pyroFiredMask` and `g_flightInProgress` *unconditionally*; resolves every saved state that needs no sensor evidence. An in-flight saved state (`BOOST`…`MAIN_DESCENT`) stays **pending** and the vehicle stays in the pyro-inert `STARTUP` (`ProcessFlightState()` returns immediately while `recoveryPending()`).

**Phase 2 — `recoveryEvidenceStep()`** (every loop pass from `handleInitialStateManagement()` in `startup_state.cpp` until it returns true): watches the live barometer for `RECOVERY_EVIDENCE_WINDOW_MS` (800 ms, ≥ `RECOVERY_EVIDENCE_MIN_SAMPLES` *fresh* samples; gives up after `RECOVERY_EVIDENCE_TIMEOUT_MS`), then applies the pure `decideRecovery()`.

A saved in-flight state is **resumed** only if all hold: barometer working and the record's calibration valid; AGL ≥ `RECOVERY_MIN_AGL_M` (30 m) and ≤ saved max + `RECOVERY_ALT_MARGIN_M`; **|vertical rate| ≥ `RECOVERY_MIN_VERTICAL_RATE_MPS`** (a vehicle on the pad — even one at a different site so "AGL" looks plausible — is stationary); fewer than `RECOVERY_MAX_RESUMES` resets. Otherwise it restarts in `RECOVERY` (pyro-inert, beacon; `reset_flight` needed).

### Recovery table

| Saved state | In-flight evidence | Restarts as |
|-------------|-------------------|-------------|
| `STARTUP` / `CALIBRATION` / `PAD_IDLE` | n/a | `STARTUP` (normal boot sequence) |
| `ARMED` | n/a | `PAD_IDLE` (disarmed; baro reference kept) |
| `BOOST`, `COAST` | plausible | `COAST` — apogee detection re-armed, backup timer restored from `burnoutAgeMs` + `RECOVERY_BACKUP_TIMER_ALLOWANCE_MS` (never later than nominal) |
| `APOGEE`, `DROGUE_DEPLOY` | plausible | drogue already fired ? `DROGUE_DESCENT` : `APOGEE` (fires the drogue) |
| `DROGUE_DESCENT` | plausible | `DROGUE_DESCENT` |
| `MAIN_DEPLOY` | plausible | main already fired ? `MAIN_DESCENT` : `DROGUE_DESCENT` (main altitude gate re-evaluated) |
| `MAIN_DESCENT` | plausible | `MAIN_DESCENT` |
| any of `BOOST`…`MAIN_DESCENT` | **not** plausible | `RECOVERY` |
| `LANDED`, `RECOVERY` | n/a | `RECOVERY` |
| `ERROR` | n/a | flight in progress ? `RECOVERY` : `ERROR` |
| invalid value | n/a | `STARTUP` |

No row restores `DROGUE_DEPLOY` or `MAIN_DEPLOY` directly. The fired flags are set when a fire window **completes** (`pyro_service()`), so a reset mid-window re-fires that channel (a second pulse into an already-fired e-match is harmless; a missed drogue is not) and a reset after completion never does.

## `flightInProgress` and `reset_flight`

Set at BOOST entry, persisted, cleared only by the token-confirmed `reset_flight` command ([[entities/command-processor]]). While set, ERROR auto-recovery, `clear_errors`, `clear_to_calibration` and `skip_calibration` (from ERROR) are refused, and `ERROR` cannot be entered. `reset_flight` also clears the fired mask, resume count and max altitude, and returns to `PAD_IDLE`/`CALIBRATION`.

## Related

[[concepts/flight-state-transitions]] · [[concepts/architecture-decisions]] (ADR-004) · [[queries/flight-logic-audit-2026-09]]
