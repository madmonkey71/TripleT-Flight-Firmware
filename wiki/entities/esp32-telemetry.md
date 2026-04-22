---
title: ESP32 Telemetry Link (Wireless Bridge)
type: entity
tags: [esp32, telemetry, wireless, esp-now]
created: 2026-04-22
updated: 2026-04-22
related_files: [esp32_telemetry_transmitter/esp32_telemetry_transmitter.cpp, esp32_ground_station_receiver/esp32_ground_station_receiver.cpp, docs/TELEMETRY_IMPLEMENTATION_PLAN.md]
---

Planned wireless telemetry path: Teensy → ESP32 transmitter (onboard) → ESP-NOW radio → ESP32 receiver (ground station) → USB serial → [[entities/web-interface|web console]]. Firmware stubs live in `esp32_telemetry_transmitter/` and `esp32_ground_station_receiver/`; integration is tracked as Phase 6 work.

## Status

- ✅ Protocol and packet layout specified in `docs/TELEMETRY_IMPLEMENTATION_PLAN.md`.
- ✅ ESP32 firmware skeletons exist (both TX and RX).
- ⏳ Not yet enabled in Teensy main loop (gate on `ENABLE_TELEMETRY` flag + Serial5 output in `WriteLogData`).
- ⏳ End-to-end integration test pending.

Treat this page as **the design**, not the current shipped feature. [[queries/development-status-2026-04]] tracks real status.

## Link Topology

```
Teensy 4.1                     ESP32 (TX, onboard)              ESP32 (RX, ground)         Host PC
───────────                    ─────────────────                 ──────────────────         ───────
LogData ──Serial5──▶  UART in ─▶ pack/checksum ─ESP-NOW▶  ESP-NOW▶ unpack/validate ─▶ CSV ─USB─▶ browser
         (115200 baud)          (250 B max)                                                 (web_interface/)
```

ESP-NOW is chosen for low-latency peer-to-peer 2.4 GHz broadcast without Wi-Fi association. 250-byte packet limit drives the compact binary format.

## Packet Layout (planned)

Fixed-size binary struct (`TelemetryPacket`) covering the high-value subset of `LogData`:

| Field | Size | Notes |
|-------|------|-------|
| timestamp | 4 B | ms since boot |
| flight_state | 1 B | enum |
| error_code | 1 B | `ErrorCode_t` |
| lat, lon, alt_gps | 3×4 B | fixed-point |
| altitude (baro) | 4 B | meters AGL |
| accel x/y/z | 3×2 B | int16, scaled |
| quat q0..q3 | 4×2 B | int16, scaled |
| battery_v | 2 B | mV |
| stability_flags | 1 B | bitfield |
| checksum | 1 B | XOR or CRC-8 |

Total well under 250 B, leaving headroom for growth.

## Teensy-Side Integration Points

1. Add `Serial5.begin(115200)` in `setup()` behind `ENABLE_TELEMETRY`.
2. In `WriteLogData()` (or a dedicated `sendTelemetryPacket()`), pack the `LogData` subset and write to `Serial5`.
3. Preserve the existing USB-serial CSV path — the two are independent outputs.

## Ground-Side Integration

The RX-side ESP32 turns received packets back into a CSV-compatible line and emits them on USB serial. The [[entities/web-interface|web console]] parses either source identically — local or radio — because the field mapping is the same.

## Failure Modes

| Failure | Result | Mitigation |
|---------|--------|------------|
| ESP32 TX power-loss | Silent gap in telemetry | SD card log on Teensy remains authoritative |
| Radio range exceeded | Packets drop silently | Acceptable — recovery beacon + GPS covers recovery |
| Checksum fail | Packet discarded on RX | Next packet (100 ms later) fills the hole |
| UART overrun | Teensy drops a packet | Telemetry is lossy-by-design; SD log is loss-tolerant |

Telemetry is **advisory**, not flight-critical. Never wire pyro arming or flight decisions to telemetry acknowledgement.

## Related

- [[concepts/data-logging]] — authoritative record lives on SD card
- [[entities/web-interface]] — consumer of the telemetry stream
- [[queries/roadmap-2026]] — Phase 6 placement
