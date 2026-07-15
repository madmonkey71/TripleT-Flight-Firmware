---
title: Developer Workflow — Build, Test, Upload, Debug
type: concept
tags: [build, testing, workflow, platformio]
created: 2026-04-22
updated: 2026-07-02
related_files: [platformio.ini, .github/workflows/test.yml, test/]
---

Standard day-in-the-life for firmware work: edit → build → unit test → flash → serial monitor → commit. The same unit tests run locally and in CI.

## Common Commands

### Build

```bash
pio run                          # default (teensy41)
pio run -e teensy41              # production
pio run -e native                # desktop build (links against ArduinoFake)
pio run -t clean                 # nuke build artifacts
```

### Unit tests (no hardware)

```bash
pio test -e native -vv                                # run all
pio test -e native -f test_apogee_detection           # one suite
```

PlatformIO auto-discovers Unity suites in `test/test_<name>/test_<name>.cpp`. Expected output: per-test `[PASSED]` lines ending in a `tests passed / failed / ignored` summary.

### Flash & monitor

```bash
pio run -e teensy41 -t upload                 # compile + flash
pio device monitor --baud 115200              # serial console
```

See [[entities/command-processor]] for commands to send once connected.

## Editing → Committing Loop

1. Change source; keep edits scoped (one concept per commit).
2. `pio run -e teensy41` — production build must pass.
3. `pio test -e native -vv` — all tests must pass.
4. Add/update unit tests for new or changed flight-logic paths ([[concepts/testing-strategy]]).
5. Commit with conventional-commit style: `<type>(<scope>): <subject>`. Types: `feat`, `fix`, `test`, `docs`, `refactor`, `chore`. Examples: `feat(hal): add ITimer interface`; `fix(gps): validate data before use`; `test(apogee): add high-G scenario`.
6. Push; GitHub Actions re-runs the same tests.

**Rule**: never push firmware that doesn't compile for both `teensy41` and `native_test`.

## Coverage Targets

- Overall repo: > 70 %.
- Flight-critical code (apogee, landing, pyro-firing, state transitions, error recovery): > 95 %.
- New feature PRs: should not decrease coverage.

See [[concepts/testing-strategy]] for the 3-tier mock strategy (synthetic / recorded / injected).

## Debugging Playbook

| Problem | First step |
|---------|-----------|
| Test fails locally | `pio test -e native -f <name> -vv`; read assertion message |
| Builds locally, fails on Teensy | Check `-DUNIT_TEST_NATIVE` / `-DNATIVE_TEST_BUILD` guards; did production path really get exercised? |
| Runtime hang on hardware | Watchdog should reset; add `enableSystemDebug` and look at serial around the event |
| Sensor returns NaN/inf | Run `status`; confirm I2C bus with `scan_i2c` |
| Flight state machine misbehaves | Enable `debug_system` (`enableSystemDebug`), reproduce; add a unit test before fixing |

## Adding a Feature

A rough contract for new flight-logic work:

1. **Write the test first.** Create `test/test_<feature>/test_<feature>.cpp`. Use Unity `TEST_ASSERT_*`.
2. **Implement.** Prefer editing existing modules to creating new ones.
3. **Check both builds pass** (`teensy41` + `native`).
4. **Update the wiki** if the change is architectural — see [[overview]] and the relevant concept/entity page.
5. **Commit** with a `feat(...)` or `fix(...)` message; include test output in the PR description.

## CI/CD

`.github/workflows/test.yml` triggers on every push and PR:

- **Unit test job**: `pio test -e native -vv`.
- **Firmware build job**: `pio run -e teensy41` — verifies the Teensy build still links.
- Artifacts: `.pio/test/**` test logs; firmware `.hex` for smoke-test downloads.
- Branch protection on `master` / `develop` requires both jobs green before merge.

Performance budget: full CI < ~5 min; tests < 10 s; firmware build < 30 s.

## Common Development Tasks

### Add a configuration parameter

1. Define in `src/config.h`: `#define NEW_PARAMETER 42`.
2. Reference it from the code path that needs it.
3. Surface it via `status` (or a new debug command) so you can sanity-check it on hardware.
4. Document it on [[entities/configuration-system]] if it is user-facing.

### Add a logged data field

1. Extend the `LogData` struct in `src/data_structures.h`.
2. Add the column to the CSV header in `src/log_format_definition.cpp`.
3. Populate the field from the main loop (see `src/TripleT_Flight_Firmware.cpp`).
4. Update the web-interface parser (`web_interface/flight_console_data_mapping.json`) — column counts must match. See [[entities/web-interface]].
5. Verify via `log_test` serial command.

### Add a serial command

1. Implement the handler in `src/command_processor.cpp` (takes `SystemStatusContext`).
2. Register the dispatch in the command table.
3. Add help text to the printed help banner.
4. Document on [[entities/command-processor]].

### Modify a flight-state transition

1. Edit `src/flight_logic.cpp` — locate the `case` for the current state and update the guard.
2. Write a Unity test in `test/test_state_machine/test_state_machine.cpp` before changing behaviour.
3. Run `pio test -e native -vv` until green.
4. Confirm both builds (`teensy41` + `native`) still pass.
5. Update [[concepts/flight-state-transitions]] and [[entities/flight-logic]] if the transition rules changed.

### Compare a change to baseline (binary footprint)

```bash
git stash                         # park the change
pio run -e teensy41 -v | grep -E 'RAM|Flash' > /tmp/baseline.txt
git stash pop
pio run -e teensy41 -v | grep -E 'RAM|Flash' > /tmp/current.txt
diff /tmp/baseline.txt /tmp/current.txt
```

Use this whenever a refactor lands; the production Teensy build is the budget that matters.

## Quick Decision Matrix — "What goes where?"

| What | Where | Why |
|------|-------|-----|
| Configuration parameter | `src/config.h` | Compile-time selection, single source of truth |
| Flight logic | `src/flight_logic.cpp` | Core state machine |
| Sensor driver | `src/sensors/*.h` | Modular, behind `IMUInterface` |
| HAL interface | `src/hal/hal_interfaces.h` | Hardware abstraction |
| Serial command | `src/command_processor.cpp` | Command handling |
| Shared type / LogData field | `src/data_structures.h` | Cross-module types |
| Unit test | `test/test_<feature>/test_<feature>.cpp` | Test code (PlatformIO Unity layout) |
| Mock object | `test/mocks/mock_<system>.h` | Test harness |
| Conceptual / architectural doc | `wiki/concepts/<page>.md` | Patterns, flows, algorithms |
| Module / component doc | `wiki/entities/<page>.md` | Specific files, classes |
| Time-stamped status / audit | `wiki/queries/<page>.md` | Roadmap, gap analysis, snapshots |

## Branch & Release Workflow

- **`master`** — tagged releases only.
- **`develop`** — integration branch for feature work.
- **`feature/<name>`** — branch from `develop`, PR back to `develop`.
- **`release/v<x.y.z>`** — release preparation branches.
- **`hotfix/<issue>`** — fast-track patches to a released version.

Squash-merge feature branches into `develop`; merge-commit release branches into `master`. Tag the release on `master` (`git tag -a v0.10.0 -m "..."`) and bump `FIRMWARE_VERSION` in `src/config.h` on `develop` the next day.

## Related

- [[concepts/testing-strategy]] — test design philosophy
- [[concepts/architecture-decisions]] — major ADRs (HAL, IMUInterface, apogee voting, EEPROM persistence, Kalman)
- [[concepts/hal-abstraction]] — why native tests work at all
- [[entities/command-processor]] — serial console commands
- [[concepts/flight-state-transitions]] — what you're usually debugging
- [[queries/roadmap-2026]] — phase plan and version cadence
