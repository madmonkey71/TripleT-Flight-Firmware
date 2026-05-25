# TripleT Flight Firmware - Testing Quick Reference

**One-Page Cheat Sheet** | Full docs: `COMPREHENSIVE_TESTING_STRATEGY.md`

---

## Running Tests

```bash
# Run all native tests
pio test -e native

# Run specific test file
pio test -e native -f test_apogee

# Run with verbose output
pio test -e native -v

# Build firmware without testing
pio run -e teensy41

# Clean and rebuild
pio run -t clean && pio run -e teensy41
```

---

## Test Files & Locations

| Component | Test File | Status | Coverage |
|-----------|-----------|--------|----------|
| Kalman Filter | `test/unit/test_kalman_*.cpp` | ✓ Exists (2 tests) | 5% |
| Apogee Detection | `test/unit/test_apogee_*.cpp` | ✓ Partial | 10% |
| Flight Logic | `test/test_flight_logic.cpp` | ✓ Minimal | <1% |
| State Machine | `test/unit/test_state_*.cpp` | ⚠ Planned | 0% |
| Landing Detection | `test/unit/test_landing_*.cpp` | ⚠ Planned | 0% |
| Guidance Control | `test/unit/test_guidance_*.cpp` | ⚠ Planned | 0% |

---

## Adding a New Test (5-Minute Guide)

### 1. Create Test File
```cpp
// test/unit/test_my_feature.cpp
#include <unity.h>

void setUp(void) {
    // Initialize before each test
}

void tearDown(void) {
    // Clean up after each test
}

void test_my_scenario(void) {
    // Arrange, Act, Assert
    int result = functionUnderTest();
    TEST_ASSERT_EQUAL(expected, result);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_my_scenario);
    UNITY_END();
    return 0;
}
```

### 2. Run Test
```bash
pio test -e native
```

### 3. Check Results
✓ Test passes → Commit
✗ Test fails → Debug → Repeat

---

## Common Assertions

```cpp
TEST_ASSERT_EQUAL(expected, actual);              // Integers
TEST_ASSERT_EQUAL_FLOAT(expected, actual);        // Floats
TEST_ASSERT_TRUE(condition);                      // Boolean true
TEST_ASSERT_FALSE(condition);                     // Boolean false
TEST_ASSERT_NOT_NULL(pointer);                    // Not null
TEST_ASSERT_FLOAT_WITHIN(tol, expected, actual);  // Float with tolerance
```

---

## Coverage Goals by Phase

| Phase | Timeline | Coverage Target | Status |
|-------|----------|-----------------|--------|
| Phase 1 | Months 1-3 | 25% → 35% | In Progress |
| Phase 2 | Months 4-6 | 35% → 60% | Planned |
| Target | Production | 60%+ | Goal |

**Safety-Critical (Must be 85%+):**
- Apogee detection
- Landing detection
- Pyro firing logic
- State transitions

---

## Pre-Flight Checklist

### Bench Testing (Before Hardware Flight)

- [ ] Phase 1: Sensor health check ✓
- [ ] Phase 2: State machine transitions ✓
- [ ] Phase 3: Apogee detection ✓
- [ ] Phase 4: Landing detection ✓
- [ ] Phase 5: Guidance system (if enabled) ✓
- [ ] Phase 6: Data logging ✓
- [ ] Phase 7: Command processor ✓

### Software Testing (Automated)

- [ ] All unit tests passing (pio test -e native)
- [ ] Firmware compiles without warnings (pio run -e teensy41)
- [ ] No high-severity code analysis warnings
- [ ] Code coverage at minimum thresholds
- [ ] CI/CD pipeline green on main branch

### Hardware Validation (Flight-Day)

- [ ] Barometer calibrated at pad altitude
- [ ] GPS 3D fix acquired
- [ ] All sensor health checks pass
- [ ] Pyro channels continuity verified
- [ ] SD card inserted and tested
- [ ] Battery voltage nominal
- [ ] System boots to PAD_IDLE state

---

## Troubleshooting

### Tests Won't Compile
```
Error: undefined reference to 'g_maxAltitudeReached'
→ Add to test file: float g_maxAltitudeReached = 0.0f;
```

### Tests Hang/Freeze
```
Infinite loop in test?
→ Add iteration counter: for(int i=0; i<100; i++)
→ Increase timeout: pio test -e native --timeout 60
```

### Unexpected Test Failure
```
→ Run with verbose output: pio test -e native -v
→ Check setUp/tearDown for state leakage
→ Verify mocks initialized correctly
```

### CI/CD Fails but Local Tests Pass
```
→ Might be timing-sensitive (flaky test)
→ Add small delays: delay(10);
→ Reset global state in setUp()
```

---

## Git Workflow

### Before Committing
```bash
pio test -e native        # Run tests
pio run -e teensy41       # Verify firmware builds
git status                # Check what changed
```

### Commit Message Format
```
Brief description of change

- Specific change 1
- Specific change 2
Tests: test_feature.cpp (5/5 passing)
```

### Create Pull Request
```bash
git checkout -b feature/my-feature
# ... make changes ...
git add .
git commit -m "description"
gh pr create --title "Feature description"
```

---

## Performance Targets

| Metric | Target | Current |
|--------|--------|---------|
| Test suite execution time | <10 sec | 6 sec ✓ |
| Firmware build time | <30 sec | 15 sec ✓ |
| CI/CD pipeline time | <5 min | 3 min ✓ |
| Code coverage | 60%+ | 5% ⚠ |
| Test pass rate | 100% | 95% ⚠ |

---

## Key Contacts

- **Test Lead:** [Name] - Sets testing standards
- **CI/CD Owner:** [Name] - Maintains GitHub Actions
- **Flight Safety Officer:** [Name] - Approves flight readiness

---

## Important Dates

- **Bench Testing Start:** [Date]
- **Target Flight Date:** [Date]
- **Code Freeze:** [Date - 2 weeks before flight]
- **Test Completion:** [Date - 1 week before flight]

---

## Resources

### Documentation
- Full Strategy: `COMPREHENSIVE_TESTING_STRATEGY.md`
- Implementation Guide: `TESTING_IMPLEMENTATION_GUIDE.md`
- Architecture Docs: `Firmware Function Documentation.md`

### Tools
- Test Framework: Unity (in `.pio/libdeps/native/`)
- Build System: PlatformIO (`platformio.ini`)
- CI/CD: GitHub Actions (`.github/workflows/`)

### External References
- Unity Framework: http://www.throwtheswitch.org/unity
- ArduinoFake: https://github.com/FabioBatSilva/ArduinoFake
- PlatformIO Testing: https://docs.platformio.org/en/latest/advanced/unit-testing/

---

## Common Commands Reference

```bash
# Testing
pio test -e native                    # Run all tests
pio test -e native -f test_name       # Run specific test
pio test -e native -v                 # Verbose output

# Building
pio run -e teensy41                   # Build firmware
pio run -e teensy41 -t upload         # Upload to Teensy
pio run -t clean                      # Clean build artifacts
pio device monitor --baud 115200      # Serial monitor

# Git/GitHub
git status                            # Check changes
git add .                             # Stage all changes
git commit -m "message"               # Commit
git push origin branch-name           # Push to GitHub
gh pr create --title "Title"          # Create PR

# Coverage (after Phase 1)
gcov test/unit/*.cpp                 # Generate coverage
lcov -d . -c -o coverage.info        # Collect coverage
genhtml coverage.info -o html/        # HTML report
```

---

## Success Criteria

### Unit Tests
- [ ] 85%+ coverage of safety-critical code
- [ ] 100% of pyro firing logic tests passing
- [ ] All state transitions tested
- [ ] Apogee detection: 90%+ coverage

### Integration Tests
- [ ] Flight simulation matches bench test predictions ±10%
- [ ] All 8 scenarios tested and documented
- [ ] Sensor failure recovery verified

### Bench Tests
- [ ] All 7 phases completed successfully
- [ ] Hardware behavior matches simulation
- [ ] No unintended pyro firing

### Flight-Ready
- [ ] All test suites passing
- [ ] Code coverage > 60%
- [ ] No critical bugs in flight logs
- [ ] Team confidence level 5/5

---

## Weekly Status Update Template

```markdown
# Test Status - Week of [DATE]

## Summary
- Tests Passing: X/Y (XX%)
- Coverage: XX% (target: XX%)
- New Tests Added: N
- Bugs Found: N
- Status: [ON TRACK / AT RISK / BLOCKED]

## This Week's Accomplishments
- [Item 1]
- [Item 2]

## Blockers/Issues
- [Issue and owner]

## Next Week's Goals
- [Goal 1]
- [Goal 2]
```

---

## Red Flags ⚠

**Stop and investigate if:**
- Tests suddenly start failing (regression)
- Coverage drops (untested code being added)
- New test won't pass (logic error or bad test?)
- CI/CD flakes (intermittent failures)
- Firmware size exceeds 220KB limit
- Any test takes >1 second to run (slowness)

**Never fly if:**
- Any apogee detection test failing
- Any pyro firing test failing
- Any state transition test failing
- Code coverage below 60% for safety-critical code
- Benchmark shows firmware slower than expected
- Any critical bugs unfixed

---

**Last Updated:** 2025-02-14
**Version:** 1.0
**Status:** Ready for Use

For detailed information, see `COMPREHENSIVE_TESTING_STRATEGY.md`
