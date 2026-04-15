---
title: Hardware Abstraction Layer (HAL)
type: concept
tags: [hal, testing, abstraction, architecture]
created: 2026-04-15
updated: 2026-04-15
related_files: [src/hal/hal_interfaces.h, src/hal/teensy_hal.h, src/hal/mock_hal.h, src/hal/hal_factory.h, src/hal/hal_config.cpp]
---

Pure-virtual interface layer that isolates all hardware calls so the firmware can be compiled and tested on desktop (Linux/Mac) without Teensy hardware.

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

After `hal_init()` in `setup()`, globals are available throughout firmware:

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

The `native` PlatformIO environment sets `-DNATIVE_TEST_BUILD` automatically.

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

Zero overhead: HAL interfaces are inlined in Teensy build. Binary size unchanged (145KB Flash) vs pre-HAL baseline.
