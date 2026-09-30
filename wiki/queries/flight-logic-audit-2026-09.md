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

### #2 ERROR mid-flight stops all deployment (critical)

- **Finding.** Three routes sent a flying vehicle to `ERROR`, where the state machine stops running apogee, the backup timer and main deploy: BOOST entry with the ICM not ready (`flight_logic.cpp` ~374), the 1 Hz periodic health check that applied to every non-terminal state (~138-178), and an `ERROR` record restored at boot (`state_management.cpp` ~172). Also `handleInitialStateManagement()` could set `ERROR` on a resumed flight.
- **Fix.** `flight_error_allowed(state)` — `ERROR` is only reachable from `STARTUP/CALIBRATION/PAD_IDLE/ARMED` and never once `g_flightInProgress`. In the air a failing health check, a missing ICM at liftoff, or an unhealthy suite after a resume calls `flightDegrade()`: log (rate limited), set `last_error_code`, orange LED, `g_guidance_active = false` + fins centred (never re-enabled mid-flight) — and every deployment path keeps running. An out-of-range state value goes to `ERROR` on the pad but to the pyro-inert `RECOVERY` once a flight has begun. Boot-time `ERROR` restore is handled by the recovery table (`RECOVERY` if a flight was in progress).
- **Tests.** `test_real_flight_logic`: unhealthy sensors in each of BOOST…MAIN_DESCENT for 10 s never enter `ERROR` (and guidance is off, fins centred, LED orange, error code logged); the backup timer still fires the drogue while degraded; ICM missing at liftoff degrades and still reaches COAST; pre-flight failures still enter `ERROR`; the flight flag blocks `ERROR` from every state; unknown state value. `test_real_recovery`: a resumed flight with an unhealthy suite still deploys main.

### #3 Auto-recovery from ERROR sends an airborne vehicle to PAD_IDLE (critical)

- **Finding.** The ERROR branch of `ProcessFlightState()` (~200-237) moved to `PAD_IDLE`/`CALIBRATION` as soon as `isSensorSuiteHealthy()` passed — which re-captured the launch altitude, zeroed the max altitude and re-enabled arming on a vehicle that could be in the air. `clear_errors`, `clear_to_calibration`, `skip_calibration` (ERROR → PAD_IDLE) and the boot-time ERROR recovery had no guard either.
- **Fix.** `flight_is_provably_on_ground()`: the persisted `flightInProgress` flag is clear (set at BOOST, survives resets, cleared only by `reset_flight` — item 7), the state is not airborne, and — if the barometer is calibrated, has a ground reference and fresh samples — AGL is within `GROUND_AGL_TOLERANCE_M`. All four routes require it. The three commands moved to `src/flight_commands.cpp` (testable natively); refusals print `REFUSED: …` and leave the state alone.
- **Tests.** `test_real_commands`: auto-recovery refused with the flight flag set (launch/max altitude preserved) and when the baro shows the vehicle 250 m up with no flag; still works on the ground; the predicate at the tolerance boundary, with a stale baro and in airborne states; each command refused in flight / airborne and allowed on the ground; boot with a saved `ERROR` + flight flag ends in `RECOVERY`.

### #4 Apogee "confirmation counts" don't confirm anything; accel method unsound (critical)

- **Finding.** `detectApogee()` / `detectBoostEnd()` ran on every loop pass (kHz) against sensor values cached at 10 Hz, so `APOGEE_CONFIRMATION_COUNT` (5) etc. were satisfied by **one** sample re-read within about a millisecond. The GPS counter also never reset when the altitude recovered. The accel method tested `icm_accel[2] < 0.0f` — true at rest if the IMU is mounted upside down, and pure noise around zero in free fall; `APOGEE_ACCEL_THRESHOLD` / `APOGEE_ACCEL_SAMPLES` were unused. No minimum time after burnout, and the barometer method could fire inside the transonic pressure disturbance.
- **Fix.**
  - `sensor_samples.h`: every driver stamps a `SampleClock` (`seq`, `lastMs`) when it stores a genuinely new sample (`ms5611_read`, `ICM_20948_read`, `kx134_read`, `gps_read`). `FreshCounter` only advances when `seq` moved; `BaroTrack` keeps the last 20 fresh baro samples (vertical speed, range, mean).
  - `readAccel()` picks KX134 (launch/burnout/high-force) or ICM (free fall); a source must be initialised, non-zero and fresh within `ACCEL_STALE_TIMEOUT_MS`.
  - **Accel method** is now the *magnitude* of specific force below `APOGEE_ACCEL_FREEFALL_G` (0.3 g) for `APOGEE_ACCEL_SAMPLES` fresh samples over `APOGEE_ACCEL_FREEFALL_WINDOW_MS` (longer without a barometer) — independent of IMU mounting and axis sign.
  - **Gates on every non-timer method:** `APOGEE_MIN_TIME_AFTER_BURNOUT_MS`, `APOGEE_MIN_ALTITUDE_GAIN_M` (when the baro works); baro locked out for `APOGEE_BARO_TRANSONIC_LOCKOUT_MS` after burnout with its descent reference restarting when it ends.
  - **Independent plausibility (decision D-4):** OR/first-match kept, but baro is vetoed by hard deceleration (`APOGEE_HIGH_FORCE_VETO_G`), accel is vetoed by a climbing baro (`APOGEE_CLIMB_VETO_MPS`), GPS by either; a missing/stale cross-check sensor never vetoes. The backup timer is ungated. GPS also now needs a 3D fix and fresh PVT.
  - The barometer is "usable" iff calibrated **and** fresh (`BARO_STALE_TIMEOUT_MS`); the per-pass `g_ms5611Sensor.isConnected()` I2C ping in the flight loop is gone.
  - Removed `APOGEE_ACCEL_CONFIRMATION_COUNT` and `APOGEE_ACCEL_THRESHOLD`; `APOGEE_ACCEL_SAMPLES` is now used.
- **Tests.** `test_real_apogee` (15): thousands of loop passes on one cached sample never confirm baro or burnout; exactly N fresh samples do; non-consecutive GPS drops don't accumulate; an upside-down IMU at 1.2 g never fires; free fall on three different axes fires only after the min time + window; climbing baro vetoes free-fall and GPS; high force vetoes baro; transonic spike can't poison the reference; min-altitude gate; ungated backup timer with no sensors; nominal latency bounds. Mutation-checked: removing the fresh-sample check, the lockout, the min-time gate, or restoring `icm_accel[2] < 0` each fails tests.

### #5 Main deploy depends on the baro alone with no debounce (critical)

- **Finding.** `DROGUE_DESCENT` fired the main on `baro AGL < g_main_deploy_altitude_m_agl` read from one cached value — a single glitchy sample deployed it, and with the barometer stale/disconnected (`isConnected()` gate) there was no path to `MAIN_DEPLOY` at all, so the vehicle sat in `DROGUE_DESCENT` forever. `MAIN_PRESENT` builds had no landing check or timeout in that state.
- **Fix.**
  - Debounce: `MAIN_DEPLOY_CONFIRMATION_COUNT` consecutive **fresh** samples below the deploy altitude (`FreshCounter`).
  - Baro unavailable (stale/uncalibrated): deploy after an *estimated descent time* `(max AGL − main altitude) / MAIN_DEPLOY_ASSUMED_DROGUE_RATE_MPS × MAIN_DEPLOY_FALLBACK_MARGIN`, floored at `MAIN_DEPLOY_FALLBACK_MIN_MS` and capped by `MAIN_DEPLOY_FALLBACK_TIME_MS` (fixed cap when the apogee is unknown). The margin is < 1 on purpose: an early main is survivable, a late one is not (decision D-5).
  - Baro alive but wrong (stuck high): hard limit `MAIN_DEPLOY_MAX_DROGUE_TIME_MS` since drogue descent began.
  - `detectLanding()` now also runs in `DROGUE_DESCENT`; both descent states are forced to `LANDED` after `DESCENT_STATE_TIMEOUT_MS`.
- **Tests.** `test_real_main_deploy` (9): thousands of passes on one low cached sample never deploy; a short glitch resets the count; N fresh low samples deploy and reach MAIN_DESCENT; dead barometer → estimated-time fallback (not before, fires after); unknown apogee → fixed fallback; stuck-high baro → hard limit; touchdown in DROGUE_DESCENT → LANDED; both descent states time out. Mutation-checked (debounce, fallback and hard-limit each removed ⇒ failures).

### #6 EEPROM save throttle; BOOST/COAST reset skips the drogue (high)

- **Finding.** `saveStateToEEPROM()` skipped any save within `EEPROM_UPDATE_INTERVAL` (60 s) of the previous one unless the state was APOGEE / DROGUE_DEPLOY / MAIN_DEPLOY / LANDED, so a fast PAD_IDLE → ARMED → BOOST → COAST left the record stale and max altitude was never refreshed. A reset in BOOST/COAST jumped straight to `DROGUE_DESCENT`, skipping apogee detection and the drogue.
- **Fix.** `saveStateToEEPROM()` is unthrottled and **put-if-changed** (all fields except the uptime timestamp are compared; identical ⇒ no flash write). `saveFlightProgressPeriodic()` (called every loop) refreshes the record every `EEPROM_PROGRESS_SAVE_INTERVAL_MS` in BOOST/COAST (max altitude, burnout age). `burnoutAgeMs` is persisted; a BOOST/COAST record resumes in `COAST` with `boostEndTime = now − (age + RECOVERY_BACKUP_TIMER_ALLOWANCE_MS)`, so the backup timer never fires *later* than nominal (decision D-6). `EEPROM_UPDATE_INTERVAL` removed. Behaviour for every saved state is the table in [[entities/state-management]].
- **Tests.** `test_real_persistence` (8): each transition saved immediately; put-if-changed (50 unchanged saves ⇒ 0 writes); ≈1 write/s and ≤1 s stale max altitude in COAST, none on the pad; COAST resume fires the backup timer at 20 s − persisted age − allowance and does not skip the drogue; BOOST resume; sensor apogee after resume; **the full table for every saved state** through the real boot path (pyro fire/no-fire expectations per row). Mutation-checked (throttle restored, age ignored, periodic save removed).
- **Bench note.** EEPROM writes on Teensy 4.1 can stall the loop for a few ms (occasionally longer when flash is re-organised). Confirm on the bench that 1 Hz saves in BOOST/COAST do not disturb the 5 s watchdog or the 50 Hz guidance loop.

### #7 No path from RECOVERY/LANDED back to PAD_IDLE (high)

- **Finding.** `LANDED → RECOVERY` is terminal in the state machine, and with the flight flag persisted (items 1/3) every ERROR-clear route is (rightly) refused after a flight, so once a flight had been recorded the only way to fly again was to wipe the EEPROM by other means.
- **Fix.** `reset_flight` (`flight_commands.cpp`), listed in `help`:
  - allowed only in `LANDED`, `RECOVERY`, `ERROR`, `PAD_IDLE` (refused — and no token issued — anywhere else);
  - requires the vehicle at rest: `flight_is_stationary_on_ground()` = barometric vertical speed ≈ 0 (`RESET_FLIGHT_MAX_VERTICAL_SPEED_MPS`) and ~1 g. It deliberately does **not** compare with the launch altitude (a rocket can land tens of metres above or below the pad); with neither sensor able to speak only `LANDED`/`RECOVERY` are trusted;
  - two steps: `reset_flight` prints a 4-digit token valid for `RESET_FLIGHT_TOKEN_TIMEOUT_MS`; `reset_flight <token>` executes it (single use);
  - effect: clears flight-in-progress, pyro-fired mask, resume count, max altitude, detectors; state → `PAD_IDLE` (baro calibrated and healthy) / `CALIBRATION` / `ERROR`; writes the record.
- **Tests.** `test_real_commands` (+6): token required / wrong / expired / single use; persists and the next boot starts normally; works from every ground state; refused (no token) from the 10 other states; refused while the baro descends or the accelerometer reads 3 g; help text lists it. Mutation-checked.

### #8 False BOOST and missed burnout (high)

- **Finding.** `ARMED → BOOST` fired on a single loop pass over one cached sample above 2 g (any bump on the pad). BOOST had no timeout and no apogee handling, so a missed burnout left the apogee / backup-timer logic (COAST only) unreachable. Burnout required |a| < 0.5 g, which a high-drag vehicle never reaches after motor burnout.
- **Fix.**
  - Launch: `LAUNCH_CONFIRMATION_COUNT` consecutive **fresh** samples above `BOOST_ACCEL_THRESHOLD` (KX134 preferred, ICM fallback; stale/dead sensors cannot launch).
  - `BOOST_TIMEOUT_MS` → `COAST` with `boostEndTime` set, so the sensor gates and the backup timer are always reachable.
  - Burnout: absolute (`COAST_ACCEL_THRESHOLD`, low-drag vehicles) **or** relative — |a| below `BOOST_BURNOUT_PEAK_FRACTION` × the smoothed peak boost level (`BOOST_ACCEL_EMA_ALPHA`), for high-drag vehicles whose post-burnout drag stays above 0.5 g. Both need `COAST_CONFIRMATION_COUNT` fresh samples. **Choice (decision D-8):** the relative test uses the magnitude of specific force, not a "thrust-axis" component, because the IMU mounting/axis sign is not known to the firmware. Known limitation: a motor whose sustain thrust is below 35 % of its (multi-sample) peak would be declared burned out early; the consequence is benign because COAST's sensor methods are gated by minimum time and independent cross-checks, but tune `BOOST_BURNOUT_PEAK_FRACTION` for such motors.
- **Tests.** `test_real_boost` (10): a single bump and thousands of cached re-reads don't launch; shorter-than-N bumps reset; exactly N fresh samples launch; dead accelerometers can't launch; BOOST timeout → COAST and the backup timer then deploys even with the accelerometer stuck high; dead accelerometers still leave BOOST; high-drag (1.8 g) burnout detected; low-drag absolute burnout; burnout needs N fresh samples; a realistic tail-off curve doesn't trigger it. Mutation-checked.

### #9 Kalman filter uses the accelerometer as a gravity reference at all times (high)

- **Finding.** `kalman_update_accel()` (`kalman_filter.cpp` ~83-109) ran on every sample: under 4-5 g of thrust, drag, or in free fall the specific-force vector was treated as "down", dragging roll/pitch towards it. `atan2(ay, az)` at `ay = az = 0` (nose vertical) silently produced roll = 0 and no input validation existed.
- **Fix.** The update is gated on `KALMAN_ACCEL_GATE_LOW_G ≤ |a| ≤ KALMAN_ACCEL_GATE_HIGH_G` (0.9–1.1 g) **and** the gyro rate from the last `kalman_predict()` ≤ `KALMAN_ACCEL_GATE_MAX_GYRO_RPS`; outside it the update is skipped (returns `false`), P keeps growing by `Q·dt` so it reconverges quickly when the gate reopens. Non-finite input is rejected; when `ay²+az² ≈ 0` only the pitch measurement is applied (roll unobservable). New `kalman_get_variance()` / `kalman_accel_updates_skipped()` diagnostics. The filter is **not** rewritten to quaternions (out of scope; gimbal-lock limitation unchanged).
- **Tests.** `test_real_kalman` (+9): 4 g thrust for 3 s leaves a tilted estimate untouched; free fall (incl. exact zero vector) skipped with finite output; rest still converges; band edges 0.85/0.89/0.91/1.0/1.09/1.11/1.5/16 g; 6 rad/s rotation gates even at 1 g; variance never shrinks while gated and the filter reconverges; the `atan2(0,0)` case; NaN/Inf rejected; full pad→boost→coast→free-fall→rest profile. Removing the gate fails 6 tests.

### #10 Pyro pin left HIGH if the state changes during the fire window (medium)

- **Finding.** The pin was raised and lowered inside the `DROGUE_DEPLOY` / `MAIN_DEPLOY` cases. Any state change while the window was open (a recovery, an external state write, an ERROR route) meant nothing ever lowered the pin — it stayed HIGH indefinitely. The function-local `static bool drogueHasFired/mainHasFired` also stayed `true` in that case and silently suppressed the next flight's fire.
- **Fix.** `pyro_control.cpp` owns every pyro pin: `pyro_request_fire(channel)` starts a window (refused if that channel's persisted fired bit is set); `pyro_service()`, called first on **every** `loop()` pass regardless of state, ends the window after `PYRO_FIRE_DURATION`, records completion in `g_pyroFiredMask` and saves it, and actively drives every idle channel LOW. The deploy states only *request* the fire and wait for `pyro_fire_complete()`; the static flags are gone. `pyro_init_safe()` aborts any window (boot only). The PAD_IDLE entry no longer touches the pins.
- **Tests.** `test_real_pyro` (7): for seven different states entered while the window is open, the pin is still on inside the window, LOW after `PYRO_FIRE_DURATION`, exactly one pulse, and completion recorded; window length within 30 ms of the configured duration; a second flight (after an aborted window and `reset_flight`) fires again; an idle channel forced HIGH is driven LOW; a completed channel can't be re-requested; repeated requests neither extend nor retrigger; no non-deploy state ever raises either pin. Mutation-checked (no end-of-window, no idle-LOW, no fired-bit guard).

### #11 Launch altitude from one sample; landing = "back at launch altitude ±1 m" (medium)

- **Finding.** `g_launchAltitude` was a single barometer sample taken at PAD_IDLE entry (its noise, or weather drift since, went into every AGL comparison of the flight). `detectLanding()` averaged altitude over a **zero-initialised** buffer, compared it with the launch altitude ±1 m — so a vehicle that lands on a hill/valley never "landed" — its confirmation timer was reset only when the *altitude* test failed (not when the accelerometer test did, so it was not consecutive) and the static buffer/timer leaked from one flight to the next.
- **Fix.**
  - Ground reference: rolling mean of the last `LAUNCH_ALT_AVG_SAMPLES` fresh samples (`BaroTrack`, reset on PAD_IDLE entry because the calibration offset can change), **re-zeroed from the average on entering ARMED** (needs `LAUNCH_ALT_MIN_SAMPLES`) and frozen while armed; the main-deploy altitude is derived from the re-zeroed AGL.
  - Landing = stationarity, evaluated once per fresh baro sample: barometric altitude span over `LANDING_WINDOW_SAMPLES` < `LANDING_ALTITUDE_STABLE_THRESHOLD`, specific force ≈ 1 g (`LANDING_ACCEL_MIN_G/MAX_G`), and (ICM alive) the IMU's own `isStationary` motion detector — the third witness stops a *frozen* barometer at ~1 g under a canopy from declaring landing mid-air (decision D-11). Needs `LANDING_CONFIRMATION_COUNT` **consecutive** samples and `LANDING_CONFIRMATION_TIME_MS`. No barometer ⇒ no landing detection (the descent timeout covers it).
  - `resetFlightDetectors()` on PAD_IDLE entry clears every per-flight counter/timer/averager (apogee, launch, burnout tracker, main gate, landing, degraded flag).
- **Tests.** `test_real_landing` (9): the pad average beats ±1 m noise even when the entry sample is 0.9 m off; arming re-zeroes from the average, follows drift, then freezes; landing detected 60 m above / at / 20 m below the pad elevation; not instantaneous; a steady 5 m/s canopy descent at 1 g is not landing; a frozen baro under a swinging canopy is not landing; consecutive-only; no fresh baro ⇒ no landing; **a complete second flight after `reset_flight` behaves exactly like the first** (same apogee time ±300 ms, both channels fire again). Mutation-checked.

### #12 Guidance commands servos under a parachute; first guidance dt is `millis() − 0` (medium)

- **Finding.** The main loop ran guidance and wrote the servos in `COAST`, `DROGUE_DESCENT` **and** `MAIN_DESCENT`, so the fins kept being steered under canopy. The first `dt` after entering that condition was `(millis() − g_lastGuidanceUpdateTime) / 1000` with the timer at 0 — i.e. minutes of uptime — injected into the PID integrators/derivatives.
- **Fix.** `flightGuidanceStep()` (`flight_logic.cpp`, called from `loop()`) owns the policy: guidance runs only in `COAST` while enabled and not stationary; in every state from `APOGEE` on (and in `COAST` with guidance disabled) `guidance_center_servos()` is called exactly once and the servos are never commanded; the timer is (re)initialised on every entry into the running condition with the nominal interval as the first `dt`, and `dt` is clamped to `GUIDANCE_MAX_DT_S` after a stall. `GUIDANCE_UPDATE_INTERVAL_MS` moved to `config.h`.
- **Tests.** `test_real_guidance` (7): across all 14 states only `COAST` runs; disabled / stationary don't run; each post-COAST state centres exactly once and never runs; disabled guidance in COAST centres once; the first `dt` at 20 min uptime is 0.02 s (old code: ~1234 s), 35 ms steps give 0.035, a 3 s stall is clamped, re-entry re-primes; end-to-end through the real state machine (steering in COAST, none after apogee, one centring). Mutation-checked.

### #13 `TEST_FREEZE` reachable in flight builds (medium)

- **Finding.** `processCommand()` always accepted `TEST_FREEZE`, which `delay(6000)`s inside the command handler — a serial command that hangs a flight computer past its 5 s watchdog in **any** state (including mid-flight, over the telemetry/USB link).
- **Fix.** The command moved to `flight_commands.cpp` behind `ENABLE_TEST_COMMANDS` (new in `config.h`, default **0**, overridable with `-D ENABLE_TEST_COMMANDS=1` for a bench environment). When compiled out the command does not exist (`Unknown command`). When compiled in it is refused outside `PAD_IDLE`.
- **Tests.** `test_real_test_commands_off` (default build): flag is 0; `TEST_FREEZE` is not recognised in any of the 14 states and never advances the clock; `command_processor.cpp` no longer contains `delay(6000)` / `"TEST_FREEZE"`. `test_real_test_commands_on` (`#define ENABLE_TEST_COMMANDS 1` before the includes): runs (and blocks ≥ 6 s) in `PAD_IDLE`; recognised-but-refused with no delay in the other 13 states. The `=1` path also compiles for Teensy. Flipping the default to 1 fails the "off" suite.
