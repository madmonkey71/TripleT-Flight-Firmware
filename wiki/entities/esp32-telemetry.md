---
title: ESP32 Telemetry Link (Wireless Bridge)
type: entity
tags: [esp32, telemetry, wireless, esp-now]
created: 2026-04-22
updated: 2026-05-25
related_files: [src/telemetry.h, src/telemetry.cpp, src/TripleT_Flight_Firmware.cpp, esp32_telemetry_transmitter/esp32_telemetry_transmitter.cpp, esp32_ground_station_receiver/esp32_ground_station_receiver.cpp, .archived/docs/TELEMETRY_IMPLEMENTATION_PLAN.md]
---

Wireless telemetry path: Teensy → ESP32 transmitter (onboard) → ESP-NOW radio → ESP32 receiver (ground station) → USB serial → [[entities/web-interface|web console]].

## Status (2026-05-25 — telemetry PR)

- ✅ Teensy side implemented: `ENABLE_TELEMETRY` flag in `src/config.h`; `src/telemetry.h` / `src/telemetry.cpp` define the 40-byte binary packet, CRC-8, and framing; `Serial5.begin(115200)` and a packed-frame write are wired into `WriteLogData()`.
- ✅ Unit tests: `test/test_telemetry/` covers struct size, scaling, NaN handling, CRC reference vectors, frame layout, corruption detection (16 cases).
- ✅ ESP32 TX firmware: `esp32_telemetry_transmitter/esp32_telemetry_transmitter.cpp` parses UART frames, validates CRC, broadcasts the bare payload over ESP-NOW.
- ✅ ESP32 RX firmware: `esp32_ground_station_receiver/esp32_ground_station_receiver.cpp` receives ESP-NOW packets and emits a `TELEM,...` CSV-shaped line on USB Serial. Status / diagnostic lines start with `#`.
- ⏳ End-to-end bench test (Teensy + 2 ESP32s + web console) pending — needs hardware.
- ⏳ Web console parser: today it parses the 62-field SD-card CSV; needs a small extension to recognise the `TELEM,` prefix.

Required for v1.0.0 ([[queries/v1-release-gate-2026-05]]): bench-test round-trip + at least one flight on radio link.

## Link Topology

```
Teensy 4.1                     ESP32 (TX, onboard)              ESP32 (RX, ground)         Host PC
───────────                    ─────────────────                 ──────────────────         ───────
LogData ──Serial5──▶  UART in ─▶ pack/checksum ─ESP-NOW▶  ESP-NOW▶ unpack/validate ─▶ CSV ─USB─▶ browser
         (115200 baud)          (250 B max)                                                 (web_interface/)
```

ESP-NOW is chosen for low-latency peer-to-peer 2.4 GHz broadcast without Wi-Fi association. 250-byte packet limit drives the compact binary format.

## Packet Layout (shipped)

Fixed-size 40-byte binary struct `TelemetryPacket` (see `src/telemetry.h`). All multi-byte integers are little-endian; floats are scaled to fixed-point so the wire format is deterministic.

| Field | Offset | Size | Encoding |
|-------|--------|------|----------|
| `timestamp_ms` | 0 | 4 | uint32 — ms since boot |
| `flight_state` | 4 | 1 | uint8 — `FlightState` enum |
| `error_code` | 5 | 1 | uint8 — last `ErrorCode_t` |
| `latitude_e7` | 6 | 4 | int32 — degrees × 1e7 |
| `longitude_e7` | 10 | 4 | int32 — degrees × 1e7 |
| `altitude_gps_mm` | 14 | 4 | int32 — GPS MSL altitude, mm |
| `altitude_baro_mm` | 18 | 4 | int32 — barometer altitude, mm |
| `accel_x_mg`/`_y`/`_z` | 22–27 | 3 × 2 | int16 — g × 1000 (milli-g), clamped |
| `q0_q14`/`q1`/`q2`/`q3` | 28–35 | 4 × 2 | int16 — quaternion × 16384 (Q14 fixed-point) |
| `battery_mv` | 36 | 2 | uint16 — battery voltage, mV |
| `stability_flags` | 38 | 1 | uint8 — bitfield (rate / attitude / saturation) |
| `guidance_active` | 39 | 1 | uint8 — 1 = guidance commanding, 0 = degraded |

UART framing (Teensy → onboard ESP32, Serial5):

```
[0xA5 SYNC][0x28 LEN=40][... 40-byte payload ...][CRC8 over payload]
```

CRC-8/SMBUS (poly 0x07, init 0x00). 43 bytes per UART frame.

ESP-NOW payload: just the 40-byte struct, no framing — ESP-NOW provides radio-level integrity.

## Teensy-Side Integration (shipped)

1. `ENABLE_TELEMETRY 0|1` in `src/config.h` gates the whole link. Default 0 — zero behavioural change to existing builds.
2. `setup()` calls `Serial5.begin(TELEMETRY_BAUD)` when enabled.
3. Inside `WriteLogData()` (same call site, same 5 Hz cadence as the SD log), `telemetry_pack(logEntry, packet)` builds the packet and `telemetry_frame()` writes the framed bytes to `Serial5`.
4. USB serial CSV and SD-card logging are unchanged.

## Ground-Side Integration (shipped)

The RX-side ESP32 receives ESP-NOW packets and emits one text line per packet on USB serial:

```
TELEM,<timestamp_ms>,<flight_state>,<error_code>,<lat>,<lon>,<alt_gps_m>,<alt_baro_m>,<ax_g>,<ay_g>,<az_g>,<q0>,<q1>,<q2>,<q3>,<battery_v>,<stability_flags>,<guidance_active>
```

The `TELEM,` prefix lets the [[entities/web-interface|web console]] distinguish radio packets from the 62-field SD-card CSV pass-through. Status / diagnostic lines begin with `#` (e.g., the receiver's MAC address printed at boot) — parsers should ignore them.

> ⚠️ The web console's current CSV parser only knows the 62-field SD-card format. Adding a `TELEM,` branch is a small follow-up before flight validation can use the radio link as the sole data source.

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
