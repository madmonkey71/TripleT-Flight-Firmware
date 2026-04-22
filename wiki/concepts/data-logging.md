---
title: Data Logging & Telemetry
type: concept
tags: [logging, sd-card, web-interface, csv, telemetry]
created: 2026-04-15
updated: 2026-04-15
related_files: [src/data_structures.h, src/log_format_definition.cpp, src/TripleT_Flight_Firmware.cpp, web_interface/]
---

Flight data is logged to CSV on the Teensy 4.1's built-in SDIO SD card and optionally streamed in real-time via USB to a web interface.

## LogData Struct

Defined in `src/data_structures.h`. ~50 fields written every 200ms (configurable):

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
#define SD_CARD_MIN_FREE_SPACE  5 * 1024 * 1024  // 5 MB minimum
#define SD_CACHE_SIZE           8                  // Cache factor
#define LOG_PREALLOC_SIZE       5000000            // Pre-allocate 5 MB
```

## Log Format

CSV with headers defined in `src/log_format_definition.cpp`. Headers and struct fields must be kept manually synchronized — there is no compile-time enforcement.

## Web Interface

Browser-based real-time dashboard in `web_interface/index.html` using the **Web Serial API** (Chrome/Edge only). Parses CSV stream and renders:
- 3D rocket orientation visualization (quaternion-driven)
- Altitude and acceleration graphs
- GPS map display
- Flight state indicator

Data field mapping: `flight_console_data_mapping.json` in repo root.

Test harness for parser: `web_interface/test_message_filtering.html`

## Telemetry (ESP32)

Two companion ESP32 projects for wireless telemetry:
- `esp32_telemetry_transmitter/` — Onboard, reads Teensy serial and RF-transmits
- `esp32_ground_station_receiver/` — Ground station, receives and forwards to PC

Protocol details in each project's README.
