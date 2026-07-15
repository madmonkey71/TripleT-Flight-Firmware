---
title: Hardware Abstraction Layer (HAL)
type: concept
tags: [hal, testing, abstraction, architecture]
created: 2026-04-15
updated: 2026-07-02
related_files: [src/hal/hal_interfaces.h, src/hal/teensy_hal.h, src/hal/mock_hal.h, src/hal/hal_factory.h, src/hal/hal_config.cpp]
---

Pure-virtual interface layer designed to isolate all hardware calls so the firmware can be compiled and tested on desktop (Linux/Mac) without Teensy hardware.

> **Status (2026-07):** the HAL is implemented but **dormant** — the flight build never calls `hal_init()`, so the global interface pointers stay null and every hardware access in the compiled firmware still goes straight to the Arduino/Teensy APIs (`Wire`, `Serial`, `EEPROM`, `SdFat`, `PWMServo`, `WDT_T4`). Migrating the flight path onto these interfaces is outstanding work. See [[queries/system-workflow-audit-2026-07]] §10.

## Interfaces (8 total)

| Interface | Replaces | Key Methods |
|-----------|----------|-------------|
| `ITimer` | `millis()`, `micros()`, `delay()` | `millis()`, `delay()`, `delayMicroseconds()` |
| `ISerial` | `Serial.print/read` | `print()`, `println()`, `read()`, `available()` |
| `IGPIO` | `digitalWrite`, `digitalRead` | `write()`, `read()`, `pinMode()` |
| `II2C` | `Wire.begin/read` | `begin()`, `write()`, `requestFrom()` |
| `IEEPROM` | `EEPROM.read/write` | `read()`, `write()`, `update()` |
| `ISDCard` | `SD.open/write` | `open()`, `write()`, `exists()` |
| `IServo` | `PWMServo.write` | `attach()`, `write()`, `writeMicroseconds()` |
| `IWatchdog` | `WDT_T4` | `begin()`, `feed()` |

## Global Instances

`hal_init()` populates globals intended to be available throughout the firmware. **Note:** `setup()` does not currently call `hal_init()`, so in the flight build these pointers remain null and unused:

```cpp
// src/hal/hal_config.cpp
extern ITimer*    g_timer;
extern ISerial*   g_serial;
extern IGPIO*     g_gpio;
extern II2C*      g_i2c;
extern IEEPROM*   g_eeprom;
extern ISDCard*   g_sdcard;
extern IServo*    g_servo;
extern IWatchdog* g_watchdog;
```

## Factory Selection

Compile-time selection via `NATIVE_TEST_BUILD` preprocessor flag:

```cpp
// src/hal/hal_factory.h
#ifdef NATIVE_TEST_BUILD
  // Returns MockTimer, MockSerial, etc.
#else
  // Returns TeensyTimer, TeensySerial, etc.
#endif
```

**Caveat:** no PlatformIO environment currently defines `NATIVE_TEST_BUILD` — the `native` test environment defines `UNIT_TEST_NATIVE` instead. As wired today the factory would always return the Teensy implementations; native test suites use the mocks directly rather than through this factory.

## Singletons Pattern

Teensy implementations use singletons to avoid heap allocation:

```cpp
class TeensyTimer : public ITimer {
  static TeensyTimer& instance() {
    static TeensyTimer _instance;
    return _instance;
  }
  uint32_t millis() override { return ::millis(); }
};
```

Inline implementations in `.h` files avoid linker issues with templates.

## Binary Size Impact

Zero overhead — but only because the HAL is currently unreferenced: the flight build makes no calls through the interfaces, so the linker drops them entirely. The inlined-singleton design should keep overhead near zero once the firmware is actually migrated onto the HAL, but that claim is unverified until the migration happens.
