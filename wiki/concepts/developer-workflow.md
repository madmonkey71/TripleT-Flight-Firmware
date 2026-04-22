---
title: Developer Workflow — Build, Test, Upload, Debug
type: concept
tags: [build, testing, workflow, platformio]
created: 2026-04-22
updated: 2026-04-22
related_files: [platformio.ini, .github/workflows/test.yml, test/unit/]
---

Standard day-in-the-life for firmware work: edit → build → unit test → flash → serial monitor → commit. The same unit tests run locally and in CI.

## Common Commands

### Build

```bash
pio run                          # default (teensy41)
pio run -e teensy41              # production
pio run -e native_test           # desktop build (links against Unity + mocks)
pio run -t clean                 # nuke build artifacts
```

### Unit tests (no hardware)

```bash
pio test -e native_test                       # run all
pio test -e native_test -f test_apogee_detection   # one file
pio test -e native_test -v                    # verbose
```

Expected output: `test/unit/test_*.cpp :: <case> [PASSED]` lines ending in `===== N passed =====`.

### Flash & monitor

```bash
pio run -e teensy41 -t upload                 # compile + flash
pio device monitor --baud 115200              # serial console
```

See [[entities/command-processor]] for commands to send once connected.

## Editing → Committing Loop

1. Change source; keep edits scoped (one concept per commit).
2. `pio run -e teensy41` — production build must pass.
3. `pio test -e native_test` — all tests must pass.
4. Add/update unit tests for new or changed flight-logic paths ([[concepts/testing-strategy]]).
5. Commit with conventional-commit style (see `CLAUDE.md`): `<type>(<scope>): <subject>`.
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
| Test fails locally | `pio test -e native_test -f <name> -v`; read assertion message |
| Builds locally, fails on Teensy | Check `-DNATIVE_TEST_BUILD` guards; did production path really get exercised? |
| Runtime hang on hardware | Watchdog should reset; add `enableSystemDebug` and look at serial around the event |
| Sensor returns NaN/inf | Run `status_sensors`; confirm I2C bus with `scan_i2c` |
| Flight state machine misbehaves | Enable `enableStateDebug`, reproduce; add a unit test before fixing |

## Adding a Feature

A rough contract for new flight-logic work:

1. **Write the test first.** Name `test/unit/test_<feature>.cpp`. Use Unity `TEST_ASSERT_*`.
2. **Implement.** Prefer editing existing modules to creating new ones.
3. **Check both builds pass** (`teensy41` + `native_test`).
4. **Update the wiki** if the change is architectural — see [[overview]] and the relevant concept/entity page.
5. **Commit** with a `feat(...)` or `fix(...)` message; include test output in the PR description.

## CI/CD

`.github/workflows/test.yml` triggers on every push and PR:

- **Unit test job**: `pio test -e native_test`.
- **Firmware build job**: `pio run -e teensy41` — verifies the Teensy build still links.
- Artifacts: `.pio/test/**` test logs; firmware `.hex` for smoke-test downloads.
- Branch protection on `master` / `develop` requires both jobs green before merge.

Performance budget: full CI < ~5 min; tests < 10 s; firmware build < 30 s.

## Related

- [[concepts/testing-strategy]] — test design philosophy
- [[concepts/hal-abstraction]] — why native tests work at all
- [[entities/command-processor]] — serial console commands
- [[concepts/flight-state-transitions]] — what you're usually debugging
