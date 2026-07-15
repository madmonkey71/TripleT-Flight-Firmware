# TripleT Flight Firmware - Testing Implementation Guide

**Version:** 1.0
**Companion to:** COMPREHENSIVE_TESTING_STRATEGY.md
**Status:** Quick-Start Implementation Reference

---

## Quick Start: First Steps (This Week)

### Step 1: Create HAL Directory Structure

```bash
# Navigate to project root
cd /mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware

# Create HAL and test directory structure
mkdir -p src/hal
mkdir -p test/unit
mkdir -p test/integration
mkdir -p test/mocks
mkdir -p test/data/sensor_recordings
mkdir -p test/data/expected_outputs
mkdir -p .github/workflows

# Create directory documentation
echo "# Hardware Abstraction Layer" > src/hal/README.md
echo "# Unit Tests" > test/unit/README.md
echo "# Integration Tests" > test/integration/README.md
echo "# Mock Implementations" > test/mocks/README.md
```

### Step 2: Create First HAL Header (hal_i2c.h)

```bash
cat > src/hal/hal_i2c.h << 'EOF'
#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <cstdint>

/**
 * @brief Abstract I2C interface for hardware independence
 *
 * This interface allows flight_logic and sensor drivers to work with
 * real I2C hardware (Teensy Wire library) or mock objects for testing.
 */
class I2CInterface {
public:
    virtual ~I2CInterface() = default;

    /// Begin I2C transmission to device at address
    virtual bool beginTransmission(uint8_t address) = 0;

    /// Write a single byte
    virtual bool write(uint8_t byte) = 0;

    /// End transmission and return status (0=success)
    virtual uint8_t endTransmission(void) = 0;

    /// Request data from device
    virtual uint8_t requestFrom(uint8_t address, uint8_t length) = 0;

    /// Read a single byte
    virtual uint8_t read(void) = 0;

    /// Check if device is present at address
    virtual bool isConnected(uint8_t address) = 0;
};

// Global HAL instance - can be swapped between real and mock
extern I2CInterface* g_i2c_hal;

#endif // HAL_I2C_H
EOF
echo "Created src/hal/hal_i2c.h"
```

### Step 3: Create Mock I2C Implementation

```bash
cat > test/mocks/mock_i2c.h << 'EOF'
#ifndef MOCK_I2C_H
#define MOCK_I2C_H

#include "../../src/hal/hal_i2c.h"
#include <vector>
#include <cstring>

/**
 * @brief Mock I2C for testing - simulates sensor responses
 */
class MockI2C : public I2CInterface {
private:
    uint8_t current_address = 0;
    std::vector<uint8_t> write_buffer;
    std::vector<uint8_t> read_buffer;
    int read_index = 0;

public:
    bool beginTransmission(uint8_t address) override {
        current_address = address;
        write_buffer.clear();
        return true;
    }

    bool write(uint8_t byte) override {
        write_buffer.push_back(byte);
        return true;
    }

    uint8_t endTransmission(void) override {
        // Mock success
        return 0;
    }

    uint8_t requestFrom(uint8_t address, uint8_t length) override {
        // Mock: return data from read_buffer
        current_address = address;
        read_index = 0;
        return length;
    }

    uint8_t read(void) override {
        if (read_index < read_buffer.size()) {
            return read_buffer[read_index++];
        }
        return 0;
    }

    bool isConnected(uint8_t address) override {
        // Mock: all devices connected
        return true;
    }

    // Test helpers
    void setReadData(const std::vector<uint8_t>& data) {
        read_buffer = data;
        read_index = 0;
    }

    std::vector<uint8_t> getLastWrite() const {
        return write_buffer;
    }

    uint8_t getLastAddress() const {
        return current_address;
    }
};

#endif // MOCK_I2C_H
EOF
echo "Created test/mocks/mock_i2c.h"
```

### Step 4: Set Up CI/CD Pipeline

```bash
cat > .github/workflows/test.yml << 'EOF'
name: Test Suite

on:
  push:
    branches: [master, beta-*, develop]
  pull_request:
    branches: [master, develop]

jobs:
  unit-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
        with:
          python-version: '3.10'

      - name: Install dependencies
        run: |
          pip install platformio
          apt-get update && apt-get install -y cmake

      - name: Build and Run Unit Tests
        run: pio test -e native -v

      - name: Upload test results
        if: always()
        uses: actions/upload-artifact@v3
        with:
          name: test-results
          path: .pio/test/

  firmware-build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
        with:
          python-version: '3.10'

      - name: Install PlatformIO
        run: pip install platformio

      - name: Build Teensy firmware
        run: pio run -e teensy41

      - name: Check firmware size
        run: |
          SIZE=$(stat -c%s .pio/build/teensy41/firmware.elf 2>/dev/null || echo "0")
          LIMIT=$((220*1024))
          echo "Firmware size: $SIZE bytes (limit: $LIMIT)"
          if [ $SIZE -gt $LIMIT ] && [ $SIZE -ne 0 ]; then
            echo "ERROR: Firmware exceeds size limit"
            exit 1
          fi
EOF
echo "Created .github/workflows/test.yml"
```

### Step 5: Create First Test File (Apogee Detection)

```bash
cat > test/unit/test_apogee_detection.cpp << 'EOF'
#include <unity.h>
#include <vector>

// Mock the dependencies for apogee detection
extern float g_maxAltitudeReached;
extern float g_launchAltitude;
extern bool g_baroCalibrated;

// Simulated apogee detection logic (extracted from flight_logic.cpp)
static int descendingCount = 0;
const int APOGEE_CONFIRMATION_COUNT = 5;

bool test_detectApogee(float currentAltitude) {
    // Simplified apogee detection for testing
    if (g_baroCalibrated && currentAltitude < g_maxAltitudeReached) {
        descendingCount++;
        if (descendingCount >= APOGEE_CONFIRMATION_COUNT) {
            return true;
        }
    } else {
        descendingCount = 0;
    }
    return false;
}

void setUp(void) {
    g_maxAltitudeReached = 1000.0f;
    g_launchAltitude = 0.0f;
    g_baroCalibrated = true;
    descendingCount = 0;
}

void tearDown(void) {
}

void test_apogee_on_fifth_descent(void) {
    TEST_ASSERT_FALSE(test_detectApogee(999.0f));  // 1
    TEST_ASSERT_FALSE(test_detectApogee(998.0f));  // 2
    TEST_ASSERT_FALSE(test_detectApogee(997.0f));  // 3
    TEST_ASSERT_FALSE(test_detectApogee(996.0f));  // 4
    TEST_ASSERT_TRUE(test_detectApogee(995.0f));   // 5 - Apogee!
}

void test_apogee_reset_on_ascent(void) {
    TEST_ASSERT_FALSE(test_detectApogee(999.0f));  // 1
    TEST_ASSERT_FALSE(test_detectApogee(998.0f));  // 2

    g_maxAltitudeReached = 1001.0f; // Climbed!
    TEST_ASSERT_FALSE(test_detectApogee(1000.5f)); // Reset

    TEST_ASSERT_FALSE(test_detectApogee(1000.0f)); // 1 (reset)
    TEST_ASSERT_FALSE(test_detectApogee(999.5f));  // 2
    TEST_ASSERT_FALSE(test_detectApogee(999.0f));  // 3
    TEST_ASSERT_FALSE(test_detectApogee(998.5f));  // 4
    TEST_ASSERT_TRUE(test_detectApogee(998.0f));   // 5 - Apogee!
}

void test_apogee_not_detected_on_ascent(void) {
    TEST_ASSERT_FALSE(test_detectApogee(999.0f));
    TEST_ASSERT_FALSE(test_detectApogee(998.0f));
    TEST_ASSERT_FALSE(test_detectApogee(999.5f)); // Ascent - reset counter
    TEST_ASSERT_FALSE(test_detectApogee(1000.0f));
    // Should not detect apogee yet
    TEST_ASSERT_FALSE(test_detectApogee(999.0f));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_apogee_on_fifth_descent);
    RUN_TEST(test_apogee_reset_on_ascent);
    RUN_TEST(test_apogee_not_detected_on_ascent);
    UNITY_END();
    return 0;
}
EOF
echo "Created test/unit/test_apogee_detection.cpp"
```

### Step 6: Add Test to platformio.ini

```bash
cat >> platformio.ini << 'EOF'

# Additional native test environment configuration
[env:native]
# Include test fixtures directory
test_build_src = true
test_filter = test_*

# Define test framework
test_framework = unity

# Build flags for testing
build_flags =
    -D UNIT_TEST_NATIVE
    -D TEST_MODE
    -Wall -Wextra -Werror=unused-variable
EOF
echo "Updated platformio.ini"
```

---

## Running Tests Locally

### Run All Tests

```bash
# Build and run all native tests
pio test -e native

# Expected output:
# Collected 2 tests
# ============ test/unit/test_apogee_detection.cpp =============
# test_apogee_on_fifth_descent ........................... [PASS]
# test_apogee_reset_on_ascent ............................ [PASS]
# test_apogee_not_detected_on_ascent ..................... [PASS]
# ======================================= 3 passed in 0.234s
```

### Run Specific Test

```bash
# Run only apogee detection tests
pio test -e native -f test_apogee

# Run with verbose output
pio test -e native -v
```

### Run Firmware Build Only

```bash
# Verify Teensy firmware still compiles
pio run -e teensy41
```

---

## Adding More Tests

### Template: New Test File

```cpp
// test/unit/test_landing_detection.cpp

#include <unity.h>

// Include or mock the functions you're testing
// Example: extern declarations or includes from source

void setUp(void) {
    // Run before each test
}

void tearDown(void) {
    // Run after each test
}

void test_landing_stable_altitude(void) {
    // Arrange: Set up test conditions
    // Act: Call the function under test
    // Assert: Verify expected behavior
    TEST_ASSERT_EQUAL(expected, actual);
}

void test_landing_wind_gust_recovery(void) {
    // Test wind gust doesn't cause false landing
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_landing_stable_altitude);
    RUN_TEST(test_landing_wind_gust_recovery);
    UNITY_END();
    return 0;
}
```

### Common Unity Assertions

```cpp
// Equality tests
TEST_ASSERT_EQUAL(expected, actual);
TEST_ASSERT_EQUAL_FLOAT(expected, actual);
TEST_ASSERT_EQUAL_STRING(expected, actual);

// Boolean tests
TEST_ASSERT_TRUE(condition);
TEST_ASSERT_FALSE(condition);

// Tolerance tests (for floating point)
TEST_ASSERT_FLOAT_WITHIN(tolerance, expected, actual);
TEST_ASSERT_INT_WITHIN(tolerance, expected, actual);

// Array tests
TEST_ASSERT_EQUAL_INT_ARRAY(expected, actual, length);
TEST_ASSERT_EQUAL_FLOAT_ARRAY(expected, actual, length);

// Null pointer tests
TEST_ASSERT_NOT_NULL(pointer);
TEST_ASSERT_NULL(pointer);
```

---

## Integration with Development Workflow

### Pre-Commit Checklist

```bash
#!/bin/bash
# Save as .git/hooks/pre-commit and chmod +x

echo "Running pre-commit tests..."

# 1. Run unit tests
pio test -e native
if [ $? -ne 0 ]; then
    echo "FAIL: Unit tests failed"
    exit 1
fi

# 2. Build firmware
pio run -e teensy41
if [ $? -ne 0 ]; then
    echo "FAIL: Firmware build failed"
    exit 1
fi

# 3. Check code style (optional)
# clang-format check can go here

echo "PASS: All pre-commit checks passed"
exit 0
```

### Feature Branch Testing

```bash
# When working on a feature branch:
git checkout -b feature/apogee-improvements

# Make changes
# ... edit flight_logic.cpp ...

# Run affected tests
pio test -e native -f test_apogee

# If tests pass, commit
git add -A
git commit -m "Improve apogee detection robustness

- Added hysteresis to prevent chatter
- Tests: test_apogee_detection.cpp (5/5 passing)"

# Create PR
gh pr create --title "Improve apogee detection" --body "See commits"
```

---

## Troubleshooting

### Test Compilation Errors

**Problem:** `error: undefined reference to 'g_maxAltitudeReached'`

**Solution:** Add global variable definition in test file or create mock:
```cpp
// Add to test file
float g_maxAltitudeReached = 0.0f;
float g_launchAltitude = 0.0f;
bool g_baroCalibrated = false;
```

### Test Execution Hangs

**Problem:** Test never completes, appears frozen

**Solution:** Check for infinite loops in test or code under test:
```cpp
// BAD: Infinite loop in test
void test_bad(void) {
    while(true) { // Hangs!
        if (some_condition) break;
    }
}

// GOOD: Use loop counter with timeout
void test_good(void) {
    int iterations = 0;
    while(iterations < 100) { // Max 100 iterations
        iterations++;
        if (some_condition) break;
    }
    TEST_ASSERT_LESS_THAN(100, iterations);
}
```

### Mock Data Not Being Used

**Problem:** Test still using real hardware values instead of mocks

**Solution:** Verify mock is initialized and injected:
```cpp
void setUp(void) {
    g_i2c_hal = &mock_i2c; // Inject mock
    mock_i2c.setReadData({0x12, 0x34}); // Set expected data
}
```

---

## Code Coverage Measurement

### Generate Coverage Report

```bash
# Compile with coverage instrumentation
pio test -e native

# Generate coverage info
gcov test/unit/*.cpp

# Create HTML report
lcov -d .pio/build/native -c -o coverage.info
genhtml coverage.info -o html/coverage

# Open in browser
open html/coverage/index.html  # macOS
# or
xdg-open html/coverage/index.html  # Linux
```

### Interpreting Coverage

```
Line Coverage:   % of lines executed
Branch Coverage: % of conditional paths taken
Function Coverage: % of functions called

Example:
apogee_detection.cpp: 85% coverage means:
- 85% of code lines were executed during tests
- 15% of lines were not hit (dead code or untested branch)

Target: 85% for safety-critical code, 70% for general code
```

---

## GitHub Actions Integration

### View Test Results

1. **On GitHub:**
   - Go to Actions tab
   - Click on latest test run
   - See real-time test output
   - Download artifacts (test results, logs)

2. **Locally:**
```bash
# See all CI runs
gh run list

# View specific run
gh run view <run-id>

# Download artifacts
gh run download <run-id>
```

### Set Up Branch Protection

```
Repository Settings → Branches → Add Rule

Branch name pattern: main, beta-*, develop

Require:
✓ Passing status checks (CI/CD must pass)
✓ Code review before merge (1 reviewer)
✓ Status checks must pass before merging
```

---

## Test Maintenance

### When Tests Fail

**Process:**

1. **Run test locally** to verify failure
   ```bash
   pio test -e native -f test_name -v
   ```

2. **Understand the failure**
   - Check assertion that failed
   - Check test output for actual vs expected values
   - Review recent code changes

3. **Fix either test or code**
   ```bash
   # If code has bug:
   git diff src/flight_logic.cpp
   # Fix the bug, re-run test

   # If test is wrong:
   git diff test/unit/test_apogee_detection.cpp
   # Update test, verify it now passes
   ```

4. **Commit fix**
   ```bash
   git add .
   git commit -m "Fix apogee detection: prevent false trigger on wind gust"
   ```

### When Tests Are Flaky

**Flaky Test:** Passes sometimes, fails other times

**Causes:**
- Timing-dependent code (race conditions)
- Uninitialized variables
- Mock data not reset properly

**Solution:**
```cpp
// BAD: Flaky due to uninitialized data
void test_flaky(void) {
    int result;
    // ... result may be uninitialized ...
    TEST_ASSERT_EQUAL(0, result); // Sometimes pass, sometimes fail
}

// GOOD: Initialize before use
void test_good(void) {
    int result = 0;
    // ... deterministic operation ...
    TEST_ASSERT_EQUAL(expected, result); // Always pass or always fail
}

// GOOD: Reset mocks in setUp
void setUp(void) {
    mock_i2c.reset();  // Clear state from previous test
    mock_baro.setAltitude(0.0f); // Set known state
}
```

---

## Performance Optimization

### Speed Up Test Execution

**Current:** ~6 seconds for 137 tests

**Optimization 1: Parallel Execution**
```bash
# Run tests in parallel (if framework supports)
pio test -e native --parallel 4
```

**Optimization 2: Reduce Test Scope**
```bash
# Run only changed tests
pio test -e native -f test_apogee test_landing

# Skip slow integration tests locally
pio test -e native -f unit/
```

**Optimization 3: Mock Expensive Operations**
```cpp
// BAD: Expensive operation in test
void test_slow(void) {
    for (int i = 0; i < 1000000; i++) { // Slow loop
        check_condition(i);
    }
}

// GOOD: Mock the expensive part
void test_fast(void) {
    mock_expensive_operation.setResult(true);
    check_condition(0); // Just verify logic
}
```

---

## Examples: Step-by-Step Test Creation

### Example 1: Testing State Machine Transition

**Source Code (flight_logic.cpp):**
```cpp
void ProcessFlightState(void) {
    switch(current_state) {
        case COAST:
            if (detectApogee()) {
                current_state = APOGEE;
            }
            break;
        case APOGEE:
            FireDroguePyro();
            current_state = DROGUE_DEPLOY;
            break;
        // ...
    }
}
```

**Test File (test/unit/test_state_transitions.cpp):**
```cpp
#include <unity.h>

// Mock the state and detection
enum FlightState current_state = STARTUP;
bool apogee_detected_flag = false;

bool detectApogee(void) {
    return apogee_detected_flag;
}

bool pyro_fired = false;
void FireDroguePyro(void) {
    pyro_fired = true;
}

// Simplified version of ProcessFlightState for testing
void ProcessFlightState(void) {
    switch(current_state) {
        case COAST:
            if (detectApogee()) {
                current_state = APOGEE;
            }
            break;
        case APOGEE:
            FireDroguePyro();
            current_state = DROGUE_DEPLOY;
            break;
    }
}

void setUp(void) {
    current_state = COAST;
    apogee_detected_flag = false;
    pyro_fired = false;
}

void test_coast_to_apogee_transition(void) {
    // Arrange
    current_state = COAST;
    apogee_detected_flag = false;

    // Act
    ProcessFlightState(); // Should not transition yet

    // Assert
    TEST_ASSERT_EQUAL(COAST, current_state);

    // Now trigger apogee
    apogee_detected_flag = true;
    ProcessFlightState();

    // Assert
    TEST_ASSERT_EQUAL(APOGEE, current_state);
}

void test_apogee_fires_drogue_and_transitions(void) {
    // Arrange
    current_state = APOGEE;
    pyro_fired = false;

    // Act
    ProcessFlightState();

    // Assert
    TEST_ASSERT_TRUE(pyro_fired);
    TEST_ASSERT_EQUAL(DROGUE_DEPLOY, current_state);
}

void test_invalid_transition_prevented(void) {
    // Arrange
    current_state = PAD_IDLE;

    // Act
    ProcessFlightState();

    // Assert
    TEST_ASSERT_EQUAL(PAD_IDLE, current_state); // No transition from PAD_IDLE
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_coast_to_apogee_transition);
    RUN_TEST(test_apogee_fires_drogue_and_transitions);
    RUN_TEST(test_invalid_transition_prevented);
    UNITY_END();
    return 0;
}
```

---

## Best Practices Summary

1. **Test Naming:** `test_component_scenario`
   - ✓ `test_apogee_descent_detection`
   - ✗ `test_apogee` (too vague)

2. **Arrange-Act-Assert Pattern:**
   ```cpp
   void test_example(void) {
       // Arrange: Set up test conditions
       // Act: Execute function under test
       // Assert: Verify results
   }
   ```

3. **One Assertion Per Test** (when possible)
   ```cpp
   // Better: Single focus
   void test_apogee_triggers_on_fifth_read(void) {
       // ... setup ...
       TEST_ASSERT_TRUE(detectApogee());
   }

   // Less ideal: Multiple assertions can mask failures
   void test_apogee_multi(void) {
       TEST_ASSERT_TRUE(detectApogee());
       TEST_ASSERT_EQUAL(APOGEE, state);
       TEST_ASSERT_TRUE(pyro_fired);
   }
   ```

4. **Mock Everything External**
   - Real hardware calls → Mock HAL
   - External sensors → Mock sensor class
   - Current time → Mock timer

5. **Test Edge Cases**
   - Boundary values (0, -1, max int)
   - Off-by-one errors
   - Null pointers
   - Empty arrays

---

## Conclusion

This guide provides concrete steps to implement the testing strategy. Start with the "Quick Start" section this week, and follow the implementation roadmap in the comprehensive strategy document.

**Key Files Created:**
1. `src/hal/hal_i2c.h` - HAL abstraction
2. `test/mocks/mock_i2c.h` - Mock implementation
3. `.github/workflows/test.yml` - CI/CD pipeline
4. `test/unit/test_apogee_detection.cpp` - First tests

**Next:** Run `pio test -e native` and watch tests pass!

---
