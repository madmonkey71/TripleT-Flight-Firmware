# TripleT Flight Firmware - Comprehensive Project Plan 2026

**Version**: 1.0
**Date**: February 14, 2026
**Current Firmware Version**: v0.51 (Beta)
**Project Status**: 70% Feature Complete, 5% Test Coverage

---

## Executive Summary

This comprehensive review of the TripleT Flight Firmware has identified:

- **Documentation**: 31 markdown files with significant redundancy and some outdated content
- **Code Quality**: Functionally complete with critical safety fixes in place, but architectural debt exists
- **Feature Completeness**: ~70% complete - core flight systems work, but quaternion orientation, trajectory loading, and telemetry are missing
- **Testing**: <5% automated test coverage - critical gap for safety-critical flight software
- **Hardware Dependencies**: 891+ direct hardware calls creating tight coupling

**Overall Assessment**: The firmware is **flight-ready for hobby/personal use** but requires additional work for competition or commercial deployment. Critical safety features (apogee detection, pyro deployment, state machine) are implemented and previous critical bugs have been fixed.

---

## Phase 1: Immediate Actions (Weeks 1-2)

### Priority: CRITICAL - Flight Safety

#### 1.1 Fix Critical Code Issues
**Timeline**: Week 1
**Effort**: 8 hours
**Responsibility**: Lead Developer

**Tasks**:
1. **Fix Apogee Detection Static Counter Reset** (15 minutes)
   - File: `src/flight_logic.cpp:904-966`
   - Issue: Static counters not reset when entering COAST state
   - Risk: False apogee detection on flight reuse without restart
   - **Action**: Add counter reset in COAST state entry at line ~550

2. **Add Sensor Data Sanity Checks** (2 hours)
   - File: `src/flight_logic.cpp` (new function `validateSensorReadings()`)
   - Check GPS vs Barometer altitude difference (< 50m threshold)
   - Check accelerometer magnitude against physical limits
   - Validate gyro rates against maximum rotation speeds
   - **Action**: Create validation function, call before state processing

3. **Implement Sensor Degradation Paths** (4 hours)
   - File: `src/flight_logic.cpp` (modify state machine)
   - **Current**: Single sensor failure → ERROR state
   - **Target**: Graceful degradation (e.g., continue without GPS if barometer works)
   - **Action**: Add DEGRADED_MODE state with reduced functionality

4. **Add Missing Error Codes** (1 hour)
   - File: `src/error_codes.h`
   - Add codes for: servo failures, guidance init failures, battery warnings
   - **Action**: Extend enum with 5-7 new codes, document in ERROR_CODES.md

**Success Criteria**:
- [ ] All static counters properly reset
- [ ] Sensor validation catches out-of-range values
- [ ] System continues operation with partial sensor failures
- [ ] New error codes documented and integrated

---

## Phase 2: Documentation Consolidation (Weeks 1-3)

### Priority: HIGH - Maintainability

#### 2.1 Consolidate Redundant Documentation
**Timeline**: Week 1-2
**Effort**: 16 hours
**Responsibility**: Technical Writer / Developer

**Consolidation Plan**:

1. **Merge State Machine Documentation** (3 hours)
   - Combine: `State Machine.md` + `FLIGHT_OPERATIONS.md`
   - **New File**: `docs/FLIGHT_STATE_MACHINE.md`
   - Include: State diagrams, transitions, timing, operational procedures
   - **Delete**: Original files after merge

2. **Merge Code Review Documents** (2 hours)
   - Combine: `FIRMWARE_REVIEW_REPORT.md` + `COMPREHENSIVE_REVIEW.md`
   - **New File**: `CODE_REVIEW_FINDINGS_2026.md`
   - Include: All identified issues, remediation status, new findings
   - **Delete**: Redundant old reviews

3. **Merge Architecture Documentation** (4 hours)
   - Combine: `TripleT_Flight_Firmware_Documentation.md` + `Feature_Usage_And_Configuration.md`
   - **New File**: `docs/SYSTEM_ARCHITECTURE.md`
   - **CRITICAL**: Fix all Madgwick→Kalman filter references
   - Include: System overview, sensor fusion, guidance control
   - **Delete**: Old files after verification

4. **Update Gap Analysis Integration** (2 hours)
   - Merge: `IMPLEMENTATION_PLAN.md` into `UPDATED_GAP_ANALYSIS_2025.md`
   - **Rename**: `UPDATED_GAP_ANALYSIS_2025.md` → `FEATURE_STATUS.md`
   - Keep as single source of truth for feature completion status
   - **Delete**: `IMPLEMENTATION_PLAN.md`

5. **Archive Historical Documents** (1 hour)
   - **Delete**: `backup/*.md` (April 2025 backups)
   - **Delete**: `VERIFICATION_REPORT.md` (branch-specific snapshot)
   - **Archive**: `web_interface/SYSTEM_STATUS.md` → `backup/SYSTEM_STATUS_SNAPSHOT_2025.md`

6. **Update All References** (4 hours)
   - Update `CLAUDE.md` with new file locations
   - Update `README.md` documentation index
   - Verify all cross-references in remaining docs
   - Update `DOCS_MAINTENANCE.md` with new structure

**Before/After Structure**:

**Before**: 31 markdown files
**After**: 18 markdown files (42% reduction)

**Consolidated Documents**:
- `docs/FLIGHT_STATE_MACHINE.md` (merged from 2)
- `CODE_REVIEW_FINDINGS_2026.md` (merged from 2)
- `docs/SYSTEM_ARCHITECTURE.md` (merged from 2)
- `FEATURE_STATUS.md` (merged from 2)

**Success Criteria**:
- [ ] No information loss during consolidation
- [ ] All Madgwick references replaced with Kalman
- [ ] Cross-references updated and verified
- [ ] `README.md` index accurate
- [ ] Reduced file count by 40%+

---

## Phase 3: Testing Infrastructure (Weeks 2-14)

### Priority: CRITICAL - Quality Assurance

#### 3.1 Create Hardware Abstraction Layer (HAL)
**Timeline**: Weeks 2-5 (4 weeks)
**Effort**: 80 hours
**Responsibility**: Senior Developer

**Architecture**: 3-Layer Abstraction

```
Business Logic (flight_logic.cpp, guidance_control.cpp)
         ↓
  HAL Interfaces (hal/*.h)
         ↓
Hardware Implementations (hal/teensy/*.cpp) | Mock Implementations (hal/mock/*.cpp)
```

**Week 2-3: Core HAL Interfaces** (40 hours)
- `hal/hal_i2c.h` - I2C communication interface
- `hal/hal_sensor.h` - Generic sensor interface
- `hal/hal_gpio.h` - Digital I/O interface
- `hal/hal_timer.h` - Timing/delay interface
- `hal/hal_serial.h` - UART communication interface
- `hal/hal_storage.h` - EEPROM/SD card interface

**Week 4: Hardware Implementations** (20 hours)
- `hal/teensy/teensy_i2c.cpp` - Real Teensy Wire wrapper
- `hal/teensy/teensy_gpio.cpp` - Real GPIO wrapper
- `hal/teensy/teensy_timer.cpp` - Real millis/delay wrapper
- `hal/teensy/teensy_serial.cpp` - Real Serial wrapper
- `hal/teensy/teensy_storage.cpp` - Real EEPROM/SdFat wrapper

**Week 5: Mock Implementations** (20 hours)
- `hal/mock/mock_i2c.cpp` - Simulated I2C for testing
- `hal/mock/mock_sensor.cpp` - Configurable sensor simulator
- `hal/mock/mock_gpio.cpp` - GPIO state tracking
- `hal/mock/mock_timer.cpp` - Accelerated time simulation
- `hal/mock/mock_storage.cpp` - In-memory storage

**Success Criteria**:
- [ ] All HAL interfaces defined with clear APIs
- [ ] Hardware wrappers compile and run on Teensy 4.1
- [ ] Mock implementations compile in native environment
- [ ] No breaking changes to existing firmware API
- [ ] Documentation for each interface

#### 3.2 Refactor Critical Systems for Testability
**Timeline**: Weeks 6-9 (4 weeks)
**Effort**: 80 hours

**Week 6: State Machine Refactoring** (20 hours)
- **Goal**: Reduce `ProcessFlightState()` from 782 lines to ~50 lines + handlers
- **Pattern**: State Pattern with handler functions
- Create: `StateHandler` interface
- Implement: 13 state handlers (STARTUP, CALIBRATION, PAD_IDLE, etc.)
- **File**: `src/state_handlers/` directory
- **Benefit**: Each state independently testable

**Week 7: Flight Logic Decoupling** (20 hours)
- **Goal**: Reduce global variable coupling from 21 externs to FlightContext struct
- Create: `FlightContext` struct with all flight state
- Refactor: Pass context by reference through call hierarchy
- Update: All state handlers to use context
- **Benefit**: Enables dependency injection for testing

**Week 8: Sensor Driver Abstraction** (20 hours)
- Modify sensor drivers to use HAL interfaces
- Create: Sensor factory pattern for instantiation
- Update: `icm_20948_functions.cpp`, `ms5611_functions.cpp`, etc.
- **Benefit**: Swap real sensors for mocks in tests

**Week 9: Testing Integration** (20 hours)
- Integrate HAL with refactored code
- Verify hardware builds still work
- Create first integration test with mocks
- **Benefit**: Validate refactoring didn't break anything

**Success Criteria**:
- [ ] State handlers isolated and independently testable
- [ ] FlightContext eliminates global coupling
- [ ] Sensor drivers use HAL, not direct Wire calls
- [ ] Hardware firmware still compiles and runs
- [ ] First integration test passes

#### 3.3 Implement Unit Test Suite
**Timeline**: Weeks 10-14 (5 weeks)
**Effort**: 100 hours

**Test Coverage Targets**:
- **Safety-Critical**: 85%+ coverage
  - Apogee detection (90%+)
  - Landing detection (85%+)
  - Pyro firing logic (100%)
  - State machine transitions (85%+)
- **Core Systems**: 70%+ coverage
  - Kalman filter (75%)
  - Guidance control (70%)
  - Sensor fusion (75%)
- **Supporting Systems**: 60%+ coverage
  - Data logging (70%)
  - Command processor (60%)
  - Utilities (60%)

**Week 10: Safety-Critical Tests** (25 hours)
- Apogee detection: 15 tests (all 4 methods + edge cases)
- Landing detection: 10 tests (stability, timeout, false positives)
- Pyro firing: 8 tests (timing, state guards, safety)
- **Target**: 33 tests, 85%+ coverage

**Week 11: State Machine Tests** (20 hours)
- State transitions: 15 tests (all valid transitions)
- Error recovery: 5 tests (grace periods, manual clear)
- State persistence: 5 tests (EEPROM save/restore)
- **Target**: 25 tests, 85%+ coverage

**Week 12: Sensor Fusion Tests** (20 hours)
- Kalman filter: 15 tests (init, predict, update, edge cases)
- Sensor switching: 5 tests (ICM↔KX134 handoff)
- Orientation accuracy: 5 tests (known inputs→expected outputs)
- **Target**: 25 tests, 75%+ coverage

**Week 13: Guidance & Control Tests** (20 hours)
- PID control: 12 tests (proportional, integral, derivative, limits)
- Stability monitoring: 8 tests (violation detection, recovery)
- **Target**: 20 tests, 70%+ coverage

**Week 14: Integration & Scenario Tests** (15 hours)
- Full flight simulation: 3 scenarios (nominal, apogee failure, sensor degradation)
- Command processor: 10 tests (parsing, validation, execution)
- Data logging: 5 tests (CSV format, field population, SD card)
- **Target**: 18 tests, scenario coverage

**Total Tests**: 137+ unit tests + 8 integration tests
**Estimated Coverage**: 60%+ overall, 85%+ safety-critical

**Success Criteria**:
- [ ] All safety-critical tests passing (100%)
- [ ] Coverage reports generated (gcov/lcov)
- [ ] Tests run in CI/CD pipeline
- [ ] No regressions in hardware builds

#### 3.4 CI/CD Pipeline Setup
**Timeline**: Week 10 (parallel with testing)
**Effort**: 8 hours

**GitHub Actions Workflow**:
1. **Build Verification** (compile for Teensy 4.1)
2. **Unit Test Execution** (native platform)
3. **Coverage Reporting** (gcov → Codecov)
4. **Static Analysis** (cppcheck)
5. **Documentation Build** (verify markdown links)

**Files to Create**:
- `.github/workflows/build.yml` - Main CI pipeline
- `.github/workflows/test.yml` - Test execution
- `.github/workflows/coverage.yml` - Coverage reporting

**Success Criteria**:
- [ ] All PRs trigger automated build
- [ ] Test failures block merges
- [ ] Coverage trends visible in GitHub
- [ ] Build badges in README.md

---

## Phase 4: Feature Completion (Weeks 15-22)

### Priority: MEDIUM - Functionality Gaps

#### 4.1 Implement Quaternion-Based Kalman Filter
**Timeline**: Weeks 15-17 (3 weeks)
**Effort**: 60 hours
**Responsibility**: Algorithms Engineer

**Current Issue**: Euler angle implementation suffers gimbal lock at ±90° pitch

**Implementation Plan**:
1. **Week 15**: Mathematical formulation
   - Design quaternion state representation
   - Derive quaternion prediction equations (gyro integration)
   - Derive quaternion update equations (accel, mag measurements)
   - **Reference**: `docs/QUATERNION_MIGRATION_PLAN.md`

2. **Week 16**: Code implementation
   - File: `src/kalman_filter.cpp` (refactor)
   - Implement quaternion prediction step
   - Implement quaternion update step
   - Add quaternion normalization
   - Provide Euler angle conversion for backward compatibility

3. **Week 17**: Testing & validation
   - Create 20+ quaternion filter tests
   - Validate against known orientation sequences
   - Bench test with hardware in all orientations
   - Compare with Euler implementation (ensure no regressions)

**Success Criteria**:
- [ ] No gimbal lock at steep angles
- [ ] Orientation accuracy ±2° (hardware test)
- [ ] All tests passing
- [ ] Quaternion fields properly logged

#### 4.2 Implement Trajectory SD Card Loading
**Timeline**: Weeks 18-19 (2 weeks)
**Effort**: 40 hours

**Current Issue**: Only hardcoded test trajectory; no SD card loading

**Implementation Plan**:
1. **Week 18**: File format & parser
   - Design CSV trajectory format (lat, lon, alt, waypoint_type)
   - Implement: `guidance_load_trajectory_from_sd(const char* filename)`
   - File: `src/guidance_control.cpp`
   - Add validation (check lat/lon bounds, altitude sanity)

2. **Week 19**: Integration & testing
   - Add command: `load_trajectory <filename>`
   - Create test trajectory files
   - Test loading, waypoint following, auto-advance
   - Document trajectory file format

**Success Criteria**:
- [ ] Load trajectories from SD card CSV files
- [ ] Validation catches malformed files
- [ ] Command interface works
- [ ] Documented in `docs/CONFIGURATION.md`

#### 4.3 Implement Cross-Track Error Control
**Timeline**: Week 20 (1 week)
**Effort**: 20 hours

**Current Issue**: XTE calculation and PID control commented out

**Implementation Plan**:
- File: `src/guidance_control.cpp:681-685`
- Uncomment and complete `calculate_crosstrack_error_m()`
- Implement great-circle XTE calculation
- Enable XTE PID controller (config.h:341-353)
- Test lateral correction during waypoint following

**Success Criteria**:
- [ ] XTE calculated correctly (bench test with known coords)
- [ ] PID control provides lateral corrections
- [ ] Guidance follows curved paths, not just direct bearing

#### 4.4 Live Telemetry System (Optional)
**Timeline**: Weeks 21-22 (2 weeks) - **OPTIONAL**
**Effort**: 40 hours
**Note**: Requires ESP32 hardware investment

**Implementation**: See `docs/TELEMETRY_IMPLEMENTATION_PLAN.md`

**High-Level Plan**:
1. **Week 21**: ESP32 firmware development
   - Implement LoRa transmission code
   - Implement data packetization
   - Test radio range

2. **Week 22**: Teensy integration
   - Add UART telemetry output
   - Implement message prioritization
   - Test end-to-end transmission

**Decision Point**: Evaluate priority vs. other features. Telemetry is valuable for test flights but not critical for basic operation.

---

## Phase 5: Code Quality Improvements (Weeks 15-20)

### Priority: MEDIUM - Maintainability

#### 5.1 Refactor String Class Usage
**Timeline**: Week 15 (parallel with quaternion work)
**Effort**: 6 hours

- Replace `String g_FileDateString` with `char[64]`
- Replace `String g_LogDataString` with fixed buffer
- Use `snprintf()` for formatting
- **File**: `src/TripleT_Flight_Firmware.cpp:72-73`

**Success Criteria**:
- [ ] No String class in production code
- [ ] Heap fragmentation eliminated
- [ ] Logging still works correctly

#### 5.2 Enforce Stability Monitoring
**Timeline**: Week 16
**Effort**: 4 hours

- Currently: `guidance_check_stability()` only warns
- **Action**: Make stability failures trigger ERROR state
- Add grace period to prevent oscillation
- **File**: `src/flight_logic.cpp`

**Success Criteria**:
- [ ] Stability violations trigger recovery action
- [ ] Grace period prevents false triggers
- [ ] Logged with specific violation reasons

#### 5.3 Add Comprehensive Error Logging
**Timeline**: Week 17
**Effort**: 8 hours

- Add error rate limiting
- Add error context logging (which sensor, what value)
- Implement error history buffer (last 10 errors)
- **File**: `src/utility_functions.cpp`

**Success Criteria**:
- [ ] All errors logged with context
- [ ] Error history retrievable via command
- [ ] Rate limiting prevents log spam

---

## Phase 6: Documentation & Release (Weeks 21-24)

### Priority: LOW - Polish

#### 6.1 Update All Documentation
**Timeline**: Week 21
**Effort**: 16 hours

- Update `README.md` with feature status
- Document all new commands
- Update hardware requirements (if ESP32 added)
- Create user manual (if needed)

#### 6.2 Create Release Package
**Timeline**: Week 22
**Effort**: 8 hours

- Tag v1.0 release
- Create release notes
- Package test trajectories
- Create installation guide

#### 6.3 Flight Test Procedures
**Timeline**: Weeks 23-24
**Effort**: Variable (depends on weather, range access)

- Conduct 3-5 test flights
- Collect telemetry data
- Validate all systems in flight
- Document lessons learned

---

## Resource Requirements

### Personnel

**Option A: Solo Developer**
- **Timeline**: 24 weeks (6 months)
- **Effort**: 10-15 hours/week
- **Total**: 240-360 hours

**Option B: Small Team (2-3 developers)**
- **Timeline**: 12 weeks (3 months)
- **Effort**: Parallel workstreams
- **Total**: Same hours, faster completion

**Recommended Roles**:
- Lead Developer (architecture, state machine, HAL)
- Test Engineer (unit tests, CI/CD, coverage)
- Algorithms Engineer (quaternion filter, guidance)

### Equipment

**Required (Already Have)**:
- Teensy 4.1 microcontroller
- ICM-20948, MS5611, GPS, KX134 sensors
- SD card, NeoPixel, buzzer
- Development PC

**Optional (For Enhanced Testing)**:
- Oscilloscope ($300) - for signal validation
- Logic analyzer ($50) - for I2C debugging
- Power supply ($100) - for bench testing
- ESP32 modules ($50) - for telemetry (if implementing)

**Total Additional Cost**: $500-$700 (optional)

### Software Tools

**All Free/Open Source**:
- PlatformIO (already installed)
- Unity test framework (already installed)
- GitHub Actions (free for public repos)
- gcov/lcov (coverage tools)
- cppcheck (static analysis)

**Total Software Cost**: $0

---

## Success Metrics

### Phase 1-2 (Weeks 1-3)
- [ ] Zero critical safety bugs remaining
- [ ] Documentation reduced by 40%+
- [ ] All documentation current and accurate

### Phase 3 (Weeks 2-14)
- [ ] HAL implemented and integrated
- [ ] Test coverage: 5% → 60%+
- [ ] Safety-critical coverage: 85%+
- [ ] CI/CD pipeline operational

### Phase 4 (Weeks 15-22)
- [ ] Quaternion filter eliminates gimbal lock
- [ ] Trajectory loading from SD card working
- [ ] Cross-track error control implemented

### Phase 5-6 (Weeks 15-24)
- [ ] No String class usage in production code
- [ ] Stability monitoring enforced
- [ ] Release v1.0 tagged and documented
- [ ] Successful test flights completed

---

## Risk Assessment & Mitigation

### High-Risk Items

**1. HAL Refactoring Introduces Bugs**
- **Risk**: Breaking existing functionality during abstraction
- **Mitigation**:
  - Maintain backward compatibility
  - Comprehensive regression testing
  - Incremental refactoring (one module at a time)
  - Keep hardware builds compiling at every step

**2. Test Coverage Doesn't Catch Real Issues**
- **Risk**: Tests pass but hardware fails
- **Mitigation**:
  - Hardware-in-loop (HIL) testing
  - Extensive bench testing before flight
  - Test flights in controlled environment
  - Maintain manual test procedures

**3. Schedule Overruns**
- **Risk**: Complexity underestimated, timeline slips
- **Mitigation**:
  - Phases are mostly independent (can re-prioritize)
  - Critical path is Phase 1 + Phase 3.1-3.2
  - Non-critical features (telemetry) can be deferred
  - Weekly progress reviews

**4. Resource Constraints**
- **Risk**: Limited developer time
- **Mitigation**:
  - Prioritization (safety first, features second)
  - Parallel work possible with team
  - Community contributions (if open source)
  - Defer non-essential features

---

## Decision Points

### Week 4: HAL Architecture Review
- **Decision**: Validate HAL design before full implementation
- **Criteria**: Can mock all critical hardware? No performance overhead?

### Week 10: Coverage Assessment
- **Decision**: Is 60% target achievable? Adjust if needed
- **Criteria**: Test execution speed, coverage tool effectiveness

### Week 14: Feature Prioritization
- **Decision**: Which Phase 4 features to implement first?
- **Criteria**: Flight requirements, available time, hardware availability

### Week 20: Telemetry Go/No-Go
- **Decision**: Implement telemetry or defer to v2.0?
- **Criteria**: Budget for ESP32 hardware, timeline pressure

---

## Maintenance Plan (Post-Launch)

### Ongoing Activities

**Weekly**:
- Review CI/CD failures
- Monitor test coverage trends
- Address critical bug reports

**Monthly**:
- Documentation updates for any code changes
- Review error logs from flights
- Update test cases based on lessons learned

**Quarterly**:
- Major feature releases (v1.1, v1.2, etc.)
- Dependency updates (library versions)
- Hardware compatibility testing

**Annually**:
- Full architecture review
- Performance optimization pass
- Security audit

---

## Appendices

### A. File Structure After Phase 2

```
TripleT-Flight-Firmware/
├── README.md (updated)
├── CLAUDE.md (updated)
├── FEATURE_STATUS.md (consolidation of gap analysis + implementation plan)
├── CODE_REVIEW_FINDINGS_2026.md (consolidation of reviews)
├── COMPETITIVE_ANALYSIS.md
├── BENCH_TEST_PROCEDURE.md
├── PROJECT_PLAN_2026.md (this document)
├── docs/
│   ├── FLIGHT_STATE_MACHINE.md (merged from State Machine + FLIGHT_OPERATIONS)
│   ├── SYSTEM_ARCHITECTURE.md (merged from TripleT_Flight_Firmware_Doc + Feature_Usage)
│   ├── COMMANDS.md
│   ├── DEVELOPMENT_STATUS.md
│   ├── ERROR_CODES.md
│   ├── HARDWARE.md
│   ├── CONFIGURATION.md
│   ├── TESTING.md
│   ├── QUATERNION_MIGRATION_PLAN.md
│   ├── STM32_MIGRATION_ANALYSIS.md
│   ├── TELEMETRY_IMPLEMENTATION_PLAN.md
│   └── DOCS_MAINTENANCE.md
├── test/
│   ├── README.md
│   ├── test_apogee_detection.cpp (new)
│   ├── test_landing_detection.cpp (new)
│   ├── test_state_machine.cpp (new)
│   └── ... (more tests)
├── hal/ (new directory)
│   ├── hal_i2c.h
│   ├── hal_sensor.h
│   ├── hal_gpio.h
│   ├── hal_timer.h
│   ├── hal_serial.h
│   ├── hal_storage.h
│   ├── teensy/ (hardware implementations)
│   └── mock/ (test implementations)
└── web_interface/
    ├── README.md
    └── ... (existing files)
```

### B. Test Execution Commands

```bash
# Run all tests
pio test -e native

# Run specific test
pio test -e native -f test_apogee_detection

# Generate coverage report
pio test -e native --with-coverage

# View coverage HTML
open .pio/coverage/index.html
```

### C. References

- Unity Testing Framework: https://github.com/ThrowTheSwitch/Unity
- PlatformIO Testing: https://docs.platformio.org/en/latest/advanced/unit-testing/
- GitHub Actions: https://docs.github.com/en/actions
- Kalman Filter Theory: Welch & Bishop, "An Introduction to the Kalman Filter"
- Quaternion Orientation: Kuipers, "Quaternions and Rotation Sequences"

---

## Conclusion

This comprehensive project plan transforms the TripleT Flight Firmware from a functional prototype (70% feature complete, 5% test coverage) into production-ready software (95%+ feature complete, 60%+ test coverage) over 24 weeks.

**Key Milestones**:
- **Week 3**: All documentation consolidated and current
- **Week 9**: HAL implemented, critical systems refactored
- **Week 14**: 60%+ test coverage achieved
- **Week 20**: All major features completed
- **Week 24**: v1.0 release with flight validation

**Total Effort**: 240-360 hours (6 months solo, 3 months with team)
**Investment**: $0-$700 (optional equipment)
**Outcome**: Reliable, well-tested, maintainable flight control software

The firmware is already suitable for hobby flights. This plan brings it to competition and commercial quality standards.
