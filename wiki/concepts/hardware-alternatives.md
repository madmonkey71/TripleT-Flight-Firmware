---
title: Flight Computer Hardware Alternatives
type: concept
tags: [hardware, mcu, migration, alternatives, teensy, stm32, esp32]
created: 2026-07-01
updated: 2026-07-02
related_files: [src/config.h, src/hal/teensy_hal.h, src/hal/hal_interfaces.h, platformio.ini, .archived/docs/STM32_MIGRATION_ANALYSIS.md]
---

Evaluates whether a different main-controller board could match or beat the current [[entities/hardware-platform|Teensy 4.1]] outcome. Scope is the flight computer MCU only — sensor loadout is covered separately in [[concepts/sensor-evaluation]].

## Why This Question Even Has a Good Answer

The [[concepts/hal-abstraction]] layer already isolates every hardware touchpoint behind 8 pure-virtual interfaces (`ITimer`, `ISerial`, `IGPIO`, `II2C`, `IEEPROM`, `ISDCard`, `IServo`, `IWatchdog`). All flight-critical logic (`flight_logic.cpp`, `kalman_filter.cpp`, `guidance_control.cpp`, `state_management.cpp`) is standard C++ with zero direct Arduino/Teensy calls. A hardware swap is bounded to writing a new HAL backend plus `sensor_factory` wiring — not touching flight logic. This was validated once already: `.archived/docs/STM32_MIGRATION_ANALYSIS.md` (2025-05-18) concluded an STM32 port was "highly viable," ~2-3 days of engineering, with no architectural blockers.

## Baseline: What Teensy 4.1 Currently Delivers

| Property | Value | Why it matters here |
|---|---|---|
| Core | Cortex-M7 @ 600 MHz, hardware FPU | Kalman filter + PID guidance run with wide timing margin at 20-100 ms sensor cadence |
| RAM / Flash | 1 MB / 8 MB (firmware ~145 KB) | No memory pressure; room to grow (trajectory buffers, telemetry) |
| Storage | Native SDIO, `FIFO_SDIO` | Fast, low-CPU-overhead SD logging ([[concepts/data-logging]]) |
| Watchdog | `WDT_T4`, 5000 ms (`WATCHDOG_TIMEOUT_MS`) | Used for crash recovery via [[entities/state-management]] |
| Ecosystem | Arduino + PlatformIO, SparkFun libs used as-is | Zero driver-porting cost for ICM-20948/KX134/MS5611/u-blox |
| Wireless | None on-board | Requires the separate [[entities/esp32-telemetry|ESP32 bridge]] (implemented, behind `ENABLE_TELEMETRY`) |

This is a strong baseline for a single-board, deterministic, Arduino-native flight computer. Any alternative has to beat it on a *specific* axis (cost, wireless integration, procurement, future headroom) without giving back real-time margin, driver compatibility, or SD throughput.

## Alternatives Considered

### 1. STM32H747 / STM32H743 (e.g. Portenta H7, custom board)

| | Detail |
|---|---|
| Core | Dual Cortex-M7 (480 MHz) + M4 (240 MHz), or single M7 on H743 |
| Wireless | Portenta H7 has an on-board Murata WiFi/BLE module — could absorb the planned ESP32 telemetry role onto the flight computer itself |
| Arduino support | `arduino_core_stm32` framework; `platformio.ini` env already sketched in the archived migration analysis |
| Driver risk | SparkFun I2C libraries mostly portable (I2C is I2C), but `PWMServo` and `SdFat` `FIFO_SDIO` mode are Teensy-specific and must be replaced (standard `Servo.h`, STM32 SDMMC config) |
| SD storage | SDMMC peripheral exists but needs its own init path — not a drop-in HAL backend swap, it's new code in `teensy_hal.h`'s STM32 sibling |
| Cost / availability | Portenta H7 ≈ 3-4× Teensy 4.1 price; custom STM32H743 board is cheaper in volume but adds board-design + procurement lead time |

**Verdict:** Viable and the only alternative that could genuinely consolidate wireless telemetry onto one board. Main cost is re-implementing SD/servo glue and validating SDMMC reliability under vibration/power-brownout — exactly the scenario the current Teensy SDIO path is already field-proven for.

### 2. ESP32-S3 (single-board, replaces Teensy entirely)

| | Detail |
|---|---|
| Core | Dual-core Xtensa LX7 @ 240 MHz, no hardware FPU (S3 has vector instructions but float math is materially slower than Cortex-M7) |
| Wireless | WiFi + BLE built in — eliminates the separate ESP32 telemetry board and its ESP-NOW hop entirely |
| Arduino support | Strong (`espressif32` PlatformIO platform), but SparkFun driver compatibility varies per library and needs re-verification |
| SD storage | SPI-mode SD only (no native SDIO controller comparable to Teensy's) — meaningfully slower log-write throughput, a regression for [[concepts/data-logging]]'s 62-field CSV stream |
| Determinism | FreeRTOS scheduler + WiFi stack sharing the core budget makes tight, jitter-free timing for the flight-state loop harder to guarantee than a bare-metal Cortex-M7 loop |
| Watchdog | Has one, but not validated against this project's recovery flow |

**Verdict:** Attractive for cost and integrated wireless, but the FPU and SD-throughput regressions are real risks for a safety-critical apogee/guidance loop. Better suited as *what the existing `esp32_telemetry_transmitter` stub already is* — a co-processor for the wireless bridge — not a Teensy replacement.

### 3. STM32F4 (e.g. STM32F405, "cheap and cheerful")

| | Detail |
|---|---|
| Core | Single Cortex-M7... no — Cortex-M4F @ 168 MHz | Adequate for the 50 Hz control loop per the archived analysis, but far less thermal/compute headroom than M7 for future feature growth (trajectory following, quaternion Kalman) |
| Cost | Cheapest option, widest STM32 second-source availability | Good for a from-scratch custom board at scale |
| Everything else | Same porting burden as STM32H7 (pins, `PWMServo`, SDIO) without the RAM/Flash headroom | |

**Verdict:** Only makes sense if the goal is unit cost at volume on a custom board; gives up the comfortable compute margin the project currently enjoys for no capability gain.

### 4. Stay on Teensy 4.1, add ESP32-S3 co-processor (status quo path)

This is what the roadmap already plans via [[entities/esp32-telemetry]]: keep the M7 for deterministic flight logic and SD logging, use a second, cheap MCU purely as a wireless bridge. No HAL changes, no driver re-validation, no SD-throughput regression — just finishing the existing stub.

## Comparison Summary

| Option | Real-time margin | Driver/library risk | SD logging | Wireless | Relative cost | Net vs. Teensy 4.1 |
|---|---|---|---|---|---|---|
| Teensy 4.1 (current) | Best | None (proven) | Best (native SDIO) | External board required | Baseline | — |
| Portenta H7 (STM32H747) | Comparable | Medium (SD/servo glue) | Medium (SDMMC, unproven here) | On-board | 3-4× | Consolidates wireless; adds porting + validation cost |
| ESP32-S3 only | Lower (no FPU, shared RTOS) | Medium-high | Lower (SPI SD) | On-board | Lowest | Cheaper + wireless, but regresses timing/logging margins |
| STM32F405 custom | Lower (M4 vs M7) | Medium (same as H7 port) | Needs SDIO impl | External board required | Lowest (at volume) | Cheaper only at scale; no capability gain |
| Teensy 4.1 + ESP32-S3 bridge (planned) | Best | None (matches current work) | Best | On companion board | Baseline + cheap 2nd MCU | Recommended: finishes existing roadmap item |

## Recommendation

Keep Teensy 4.1 as the flight-critical MCU. It already outperforms every alternative on the axes that matter most for a safety-critical apogee/guidance loop: real-time margin, proven SD logging, and zero driver-porting risk. The one legitimate gap — no on-board wireless — is already the subject of a planned, low-risk fix ([[entities/esp32-telemetry]]) rather than a reason to re-platform the whole flight computer.

If procurement cost or a hard requirement for on-board WiFi/BLE ever forces a change, **Portenta H7** is the next-best option — it preserves Cortex-M7 compute margin while adding wireless, at the cost of reimplementing the SD/servo HAL backend and re-validating it under real flight conditions before it could be trusted over the current Teensy path.

## Related

- [[entities/hardware-platform]] — current physical loadout and pin layout
- [[concepts/hal-abstraction]] — why a swap is bounded to HAL + sensor_factory work
- [[concepts/sensor-evaluation]] — sensor-level alternatives (separate from MCU choice)
- [[entities/esp32-telemetry]] — the wireless-bridge plan referenced above
- `.archived/docs/STM32_MIGRATION_ANALYSIS.md` — prior STM32 port feasibility study this page builds on
