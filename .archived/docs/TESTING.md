# Testing Guide

This project uses the **Unity** testing framework (integrated with PlatformIO) for unit testing.

## Prerequisites
* PlatformIO Core or IDE installed.
* `native` environment configured for running logic tests on your PC (faster).
* `teensy41` environment configured for running hardware tests on the device.

## Running Tests

### 1. Run Logic Tests (Native)
These tests verify math and state machine logic without requiring the hardware.
```bash
pio test -e native
```

### 2. Run Hardware Tests
These tests verify sensor connections and hardware-specific features. *Requires a Teensy 4.1 connected via USB.*
```bash
pio test -e teensy41
```

## Writing Tests

1.  Create a new file in `test/`, e.g., `test_kalman.cpp`.
2.  Include `<unity.h>`.
3.  Define `setUp()` and `tearDown()` if needed.
4.  Write test functions using `TEST_ASSERT_*` macros.
5.  Implement `setup()` and `loop()` to run `UNITY_BEGIN()`, your tests, and `UNITY_END()`.

**Example:**
```cpp
#include <unity.h>
#include "kalman_filter.h"

void test_kalman_init(void) {
    kalman_init(0, 0, 0);
    float r, p, y;
    kalman_get_orientation(r, p, y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r);
}

void setup() {
    delay(2000);
    UNITY_BEGIN();
    RUN_TEST(test_kalman_init);
    UNITY_END();
}

void loop() {}
```
