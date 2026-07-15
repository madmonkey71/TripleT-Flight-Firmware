# TripleT Flight Firmware

**Current Version:** v0.10.0
**Current State:** Pre-Release (Phases 1-5 Complete, Phase 6 Planning)
**Last Updated:** February 2026

## Project Lead
**Matthew Thom** - Project Lead and Primary Developer

## Overview

This firmware is designed for the **Teensy 4.1** microcontroller and provides comprehensive flight control capabilities for model rockets.
The system manages all phases of flight from launch detection through recovery, utilizing a Kalman filter for sensor fusion, robust data logging, and extensive safety features.

It's been a while since I merged once of the dev branches back into the master but I thought it was time.
The current state is ready for more extensive testing and is now held back by the hardware reference design more than the completeness of this code.

**I'm hoping to have the hardware in a better state in the coming few weeks**

## Key Features

- **Multi-Phase Flight Management**: Handles all 14 flight phases from pad idle through recovery.
- **Advanced Sensor Fusion**: Kalman filter combining data from ICM-20948 IMU, KX134 high-G accelerometer, MS5611 barometer, and u-blox GPS.
- **Hardware Abstraction Layer (HAL)**: Modular architecture enabling desktop testing without hardware.
- **Sensor Modularity**: Interface-based sensor design supporting ICM-20948, BNO085, and KX134 with automatic failover.
- **Dual Accelerometer Strategy**: Intelligently uses KX134 for high-G events and ICM-20948 for general flight.
- **Multi-Path Apogee Detection**: Four methods (barometric, accelerometer, GPS, backup timer) with 2-of-3 voting consensus.
- **Comprehensive Recovery System**: SOS audio beacon, LED strobe patterns, and GPS coordinate transmission (Serial).
- **Watchdog Recovery**: Automatic state recovery from power loss or watchdog reset via EEPROM persistence.
- **Real-time Data Logging**: 62 data points logged to SD card in CSV format.
- **Wireless Telemetry (opt-in)**: Optional Teensy → ESP32 → ESP-NOW → ground-station bridge, gated by `ENABLE_TELEMETRY` in `src/config.h`. 40-byte packed binary on the radio, CSV-shaped text on the ground-side USB. See [Wireless telemetry](wiki/entities/esp32-telemetry.md). *Status: code shipped, hardware bench-test pending before v1.0.0.*
- **CI/CD Testing**: Unit tests run on every push via GitHub Actions.
- **Interactive Command Interface**: Rich serial command system for diagnostics and control.

## Project Status & Roadmap

See [`wiki/queries/roadmap-2026.md`](wiki/queries/roadmap-2026.md) for the full phase plan and [`wiki/queries/development-status-2026-04.md`](wiki/queries/development-status-2026-04.md) for outstanding gaps.

**Completed Phases:**
- Phase 1: HAL Foundation (v0.7.0)
- Phase 2: Sensor Modularity (v0.8.0)
- Phase 3: Testing Infrastructure (v0.9.0)
- Phase 4: Safety & Reliability (v0.10.0)
- Phase 5: Documentation

**Remaining Work for v1.0.0:**
1. **Live Telemetry:** Code shipped (Teensy + both ESP32 firmwares); needs a hardware bench-test and a flight on the radio link.
2. **Trajectory Following:** Waypoint navigation present but SD waypoint loader missing; cross-track error still a placeholder.
3. **Flight Validation:** 5 successful flights on v0.10.0+ firmware before tagging v1.0.0.

`PowerManager`, `PreflightChecker`, thermal management, and the quaternion Kalman migration are deferred to v1.1 — see [`wiki/queries/v1-release-gate-2026-05.md`](wiki/queries/v1-release-gate-2026-05.md).

## Quick Start

1. **Hardware Setup**: Connect sensors via I2C, GPS via serial, insert SD card.
2. **Upload Firmware**: Use PlatformIO with `teensy41` environment.
3. **Initialize**: System performs startup checks and enters `PAD_IDLE` state.
4. **Arm**: Use `arm` command when ready for flight.
5. **Recovery**: If lost, the rocket will emit an SOS beacon and strobe light.

### Optional: Enable Wireless Telemetry

1. Flash [`esp32_ground_station_receiver`](esp32_ground_station_receiver/) to a USB-connected ESP32; note the MAC address it prints at boot.
2. Paste that MAC into `ground_station_mac` in [`esp32_telemetry_transmitter/esp32_telemetry_transmitter.cpp`](esp32_telemetry_transmitter/esp32_telemetry_transmitter.cpp); flash this firmware to the onboard ESP32 and wire its UART2 RX (default GPIO16) to Teensy `Serial5 TX` (pin 20), plus GND in common.
3. Set `ENABLE_TELEMETRY 1` in [`src/config.h`](src/config.h) and rebuild the Teensy firmware.
4. Open the ground-station ESP32 on USB serial at 115200 baud; you'll see `TELEM,...` lines every ~200 ms once the Teensy starts logging.

Protocol details and packet layout: [Wireless telemetry](wiki/entities/esp32-telemetry.md).

## Documentation

All project documentation lives in the [`wiki/`](wiki/) folder. Start at [`wiki/index.md`](wiki/index.md).

- 🗺️ **[Architecture overview](wiki/overview.md)** — Tech stack, directory layout, state machine
- 🔧 **[Hardware platform](wiki/entities/hardware-platform.md)** — Teensy 4.1, sensors, wiring, pyro
- 🚀 **[Flight state transitions](wiki/concepts/flight-state-transitions.md)** — All 14 states and guards
- ⚙️ **[Configuration system](wiki/entities/configuration-system.md)** — `config.h` parameters and compile flags
- 💻 **[Serial commands](wiki/entities/command-processor.md)** — Full command catalogue
- 🛡️ **[Safety / robustness](wiki/concepts/system-robustness.md)** — 4-layer defence, redundancy
- 🛠️ **[Developer workflow](wiki/concepts/developer-workflow.md)** — Build / test / flash cycle
- 📋 **[Roadmap](wiki/queries/roadmap-2026.md)** — Phase plan v0.6.0 → v1.0.0

Historical/superseded documents are preserved under `.archived/`.

## License

This project is licensed under the MIT License.
