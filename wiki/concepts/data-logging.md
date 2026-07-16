---
title: Data Logging & Telemetry
type: concept
tags: [logging, sd-card, web-interface, csv, telemetry]
created: 2026-04-15
updated: 2026-07-16
related_files: [src/data_structures.h, src/log_format_definition.cpp, src/TripleT_Flight_Firmware.cpp, web_interface/]
---

Flight data is logged to CSV on the Teensy 4.1's built-in SDIO SD card and optionally streamed in real-time via USB to a web interface. Authoritative schema lives in `LogData` (`src/data_structures.h`); CSV column order lives in `src/log_format_definition.cpp` — the two **must** stay in lockstep.

## LogData Struct

Defined in `src/data_structures.h`. 63 fields per row (`LOG_COLUMN_COUNT`). Logging runs at **5 Hz** (hard-coded 200 ms gate in `WriteLogData()`; the `LOG_INTERVAL` constant is unused) in every flight state; the same 5 Hz row is echoed to USB serial when `enableSerialCSV` is on (`debug_serial_csv on`). SD writes are direct `println` calls with a flush every 10 rows (~2 s) — there is no RAM ring buffer or batching.

| Category | Fields |
|----------|--------|
| Identification | `seqNum`, `timestamp` |
| State | `flightState`, `last_error_code` |
| GPS | `fixType`, `sats`, `latitude`, `longitude`, `altitude`, `altitudeMSL`, `speed`, `heading`, `pDOP`, `rtk` |
| Barometer | `raw_altitude`, `calibrated_altitude`, `pressure`, `temperature` |
| KX134 (high-G) | `kx134_accel[3]` (g) |
| ICM-20948 | `icm_accel[3]`, `icm_gyro[3]`, `icm_mag[3]`, `icm_temp` |
| AHRS | `q0/q1/q2/q3`, `euler_roll/pitch/yaw`, `gyro_bias_x/y/z` |
| Guidance | `target_roll/pitch/yaw`, `pid_roll/pitch/yaw_integral`, `actuator_output_roll/pitch/yaw`, `guidance_active` |
| Stability | `stability_flags`, `max_*_rate_dps_so_far`, `max_*_att_err_deg_so_far` |
| Trajectory | `current_target_wp_idx`, `distance_to_target_wp_m`, `bearing_to_target_wp_rad`, `altitude_error_to_wp_m` |
| System | `battery_voltage` |

## SD Card Configuration

```cpp
// src/config.h
#define SD_CARD_MIN_FREE_SPACE  5 * 1024 * 1024  // 5 MB minimum — live (low-space warning)
#define SD_CACHE_SIZE           8                  // defined but UNUSED
#define LOG_PREALLOC_SIZE       5000000            // defined but UNUSED (no preallocation happens)
```

Only the minimum-free-space check is live; the cache and preallocation constants are never referenced by the code.

## Log Format

CSV with headers defined in `src/log_format_definition.cpp` (`LOG_COLUMNS[]`, 63 columns — single source of truth for the header). Headers and struct fields must be kept manually synchronized — there is no compile-time enforcement.

Log files are named `DATA_YYYYMMDD_HHMMSS.csv` when a GPS time fix is available, else `LOG_<millis>.csv` (built in — and stored in — 64-byte buffers).

## Web Interface

Browser-based real-time dashboard in `web_interface/index.html` using the **Web Serial API** (Chrome/Edge only). Parses CSV stream and renders:
- 3D rocket orientation visualization (quaternion-driven)
- Altitude and acceleration graphs
- GPS map display
- Flight state indicator

Data field mapping: `web_interface/js/flight_console_data_mapping.json`, with a matching hardcoded fallback in `data_parser.js` (used when running over `file://` and `fetch` is blocked) — both carry the full 63 columns including `GuidanceActive`. A stale duplicate used to live at the repo root; it was never fetched by any code path and was deleted.

Test harness for parser: `web_interface/test_message_filtering.html`

## Telemetry (ESP32)

Two companion ESP32 projects for wireless telemetry over ESP-NOW:
- `esp32_telemetry_transmitter/` — onboard, reads Teensy UART and RF-transmits
- `esp32_ground_station_receiver/` — ground station, receives and forwards to PC

Current status: Teensy side shipped (`src/telemetry.h`, `src/telemetry.cpp`, `ENABLE_TELEMETRY` flag in `src/config.h` — default `0`, `Serial5` write inside `WriteLogData()`); both ESP32 firmwares replaced with real code; bench-test on hardware and a flight on radio still needed. Sending piggybacks on the 5 Hz log cadence and sits after the SD early-returns, so **no SD card ⇒ no telemetry packets**. See [[entities/esp32-telemetry]] for the 40-byte packet layout and integration steps.

## Post-Flight Analysis

CSV logs are the authoritative flight record. Typical analyses:

- Apogee altitude vs prediction (expect ±10 %).
- Descent rate under drogue vs main (sanity-check parachute deployment).
- Peak acceleration vs motor thrust curve.
- `flightState` column timeline — verify every transition fired.
- Guidance review: `target_*` vs `euler_*`, PID integrals, `stability_flags`, `guidance_active`.
- Error-code post-mortem: any non-zero `last_error_code`, timing relative to flight events.
- Watchdog reset markers: data-gap > 1 s around a boot banner line.

## Related

- [[entities/web-interface]] — CSV consumer
- [[entities/esp32-telemetry]] — wireless path
- [[concepts/kalman-filter]] — source of orientation fields
- [[entities/flight-logic]] — source of state field
- [[entities/error-handling]] — `error_code` interpretation
