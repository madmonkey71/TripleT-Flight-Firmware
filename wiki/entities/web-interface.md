---
title: Web Interface — Browser-Based Flight Console
type: entity
tags: [web-serial, telemetry, visualization, dashboard]
created: 2026-04-22
updated: 2026-04-22
related_files: [web_interface/index.html, web_interface/js/data_parser.js, web_interface/js/serial_handler.js, web_interface/js/flight_console_data_mapping.json]
---

Browser-based real-time flight console that speaks to the Teensy over [Web Serial API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Serial_API). Parses the 62-field CSV telemetry stream into charts, gauges, and a 3D orientation model — no flashing or install required.

## Entry Point

`web_interface/index.html` — open directly via `file://` or serve locally.

```bash
# Options for serving (Chrome requires secure context for some features)
cd web_interface && python3 -m http.server 8000   # or use bundled run_local_server.{sh,bat}
```

Chrome / Edge only — Web Serial is not available in Firefox or Safari.

## Connection Flow

1. Click **Connect**.
2. Browser prompts for a serial port — select the Teensy.
3. Console auto-negotiates 115200 baud.
4. Telemetry stream begins; panels populate within a few seconds.

## Data Pipeline

```
Teensy USB Serial (115200 baud)
  ├─ CSV data line    (62 fields, ~100 Hz when enabled)
  ├─ INFO / WARN / ERROR / DEBUG lines
  └─ command echoes

Web console
  ├─ serial_handler.js       → line reader, error guards
  ├─ data_parser.js          → classify line, split CSV, map to named fields
  ├─ flight_console_data_mapping.json → field order ↔ semantic names
  └─ ui/charts/3d model      → render
```

The mapping JSON must stay in sync with `LogData` in `src/data_structures.h` and the CSV header produced by `src/log_format_definition.cpp`. If fields drift, the parser will mis-map — see [[concepts/data-logging]].

## Dashboard Panels

- **Flight state + error code** — from CSV columns
- **Altitude charts** — baro, GPS, calibrated; overlaid
- **Acceleration** — X/Y/Z, both IMU and high-G sensor
- **Orientation** — quaternion → 3D rocket model; Euler angles as text
- **GPS** — lat/lon, fix type, satellite count, PDOP
- **Battery** — voltage, estimated time remaining
- **Guidance** — target vs actual attitude, PID integrals, servo output, stability flag

## Command Entry

Same serial port is used to send commands back to the Teensy (e.g. `arm`, `calibrate`, `debug_serial_csv on`). Responses appear in a message log panel with colour-coded severity. See [[entities/command-processor]] for the catalogue.

## Testing

`web_interface/test_message_filtering.html` feeds canned serial lines into the parser — useful after changing field order or adding a new log column.

## Limitations & Gotchas

- **HTTPS restriction**: some browsers block Web Serial on `http://` except `localhost`. Use `file://` or `localhost` during development.
- **Field-count drift**: if `LogData` gains a field but the mapping JSON is not updated, all downstream fields shift. Integration test via `test_message_filtering.html` before flight.
- **Backpressure**: at 100 Hz, charts need throttling; heavy tabs can stall the reader and cause buffer overrun on the Teensy side.

## Related

- [[concepts/data-logging]] — CSV format / `LogData` struct
- [[entities/command-processor]] — commands available over the same link
- [[entities/esp32-telemetry]] — wireless path to the same dashboard; ground-side ESP32 emits `TELEM,...` text lines on USB serial (parser extension still needed in the web console)
