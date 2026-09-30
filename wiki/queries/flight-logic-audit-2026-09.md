---
title: Flight-Logic Audit (2026-09) — beta-0.58 fixes
type: query
tags: [audit, safety, flight-logic, recovery, apogee, eeprom, pyro]
created: 2026-09-30
updated: 2026-09-30
related_files: [src/flight_logic.cpp, src/state_management.cpp, src/startup_state.cpp, src/pyro_control.cpp, src/kalman_filter.cpp, src/sensor_samples.h, src/config.h, test/support/flight_harness.h]
---

Snapshot of the flight-safety / logic / numerical audit of `develop @ c1f0073` and how branch `beta-0.58` addresses it. One section per finding: **finding → fix → test**. Source code wins over this page; line numbers refer to `develop @ c1f0073`.

## How the fixes are tested

The pre-existing `test/test_flight_logic` and `test/test_apogee_detection` suites test *copies* of the logic (`testable_detectApogee`, mocks) and could not catch any of these defects. New `test_real_*` suites compile the **shipped** `src/*.cpp` files natively:

- `test/stubs/` — host stand-in for the Arduino core (injectable `millis()`, recorded pin writes, captured `Serial`, RAM `EEPROM`) and header stubs for the hardware libraries. Wired in through `-I test/stubs` in `[env:native]`; ArduinoFake was dropped from that env (see [[concepts/testing-strategy]]).
- `test/support/flight_harness.h` — firmware globals + recording stubs for the hardware/IO boundary only (sensor values, sensor health, guidance, logging). Flight decisions, persistence and recovery are the real code.
- `test/support/real_sources.h` — includes the real `.cpp` files into one translation unit.

Verification caveat for this branch: the authoring sandbox blocked the PlatformIO registry, so `pio test -e native` / `pio run -e teensy41` were **not** run there; equivalent host `g++` + Unity runs and an `arm-none-eabi-g++` compile of every source were used. Re-run both `pio` commands before flight.

## Findings

### #1 Stale EEPROM fires pyros at boot (critical)

- **Finding.** `recoverFromPowerLoss()` restored `DROGUE_DEPLOY` / `MAIN_DEPLOY` unconditionally, at a point in `setup()` where no sensor was up, and `handleInitialStateManagement()` only handles `STARTUP` / `ERROR`. A stale record left on the pad would fire the drogue/main on the next power-up. Nothing recorded that a channel had already fired, and the pyro pins were only set LOW after the 3 s Serial wait and watchdog start.
- **Fix.**
  - Two-phase recovery (`state_management.cpp`). Phase 1 (`recoverFromPowerLoss()`, in `setup()`) restores the pyro-fired mask and flight-in-progress flag and resolves every saved state that needs no evidence; an in-flight saved state stays *pending* and the vehicle stays in pyro-inert `STARTUP`. Phase 2 (`recoveryEvidenceStep()`, driven from `handleInitialStateManagement()` in `startup_state.cpp`) watches the live barometer for `RECOVERY_EVIDENCE_WINDOW_MS` (fresh samples only) and calls the pure `decideRecovery()`.
  - Resume requires: baro valid and calibrated record, AGL ≥ `RECOVERY_MIN_AGL_M` and ≤ saved max + `RECOVERY_ALT_MARGIN_M`, **|vertical rate| ≥ `RECOVERY_MIN_VERTICAL_RATE_MPS`** (a vehicle sitting on the pad at a different site is stationary), and fewer than `RECOVERY_MAX_RESUMES` resets. Otherwise → `RECOVERY` (pyro-inert, beacon).
  - No table row restores `DROGUE_DEPLOY` / `MAIN_DEPLOY`. See the recovery table in [[entities/state-management]].
  - `pyroFiredMask` (bit0 drogue, bit1 main) is persisted when a fire window **completes**; a set bit blocks re-firing. A reset *during* the window leaves the bit clear, so the channel fires again (documented decision).
  - `pyro_init_safe()` (`pyro_control.cpp`) is the first statement of `setup()`.
  - `FlightStateData` gained `flightInProgress`, `pyroFiredMask`, `resumeCount` (layout change ⇒ old EEPROM records fail the signature check and boot fresh).
- **Tests.** `test/test_real_recovery`: every saved state × flag × mask on a stationary pad never drives a pyro pin HIGH; stale record at a different-altitude site does not resume; the full recovery table (`decideRecovery`), implausible-evidence cases, resume/refire behaviour, dead barometer at boot, pending-state inertness, persistence of the flags, `pyro_init_safe()`, and a source-order check that `setup()` starts with `pyro_init_safe();`.
