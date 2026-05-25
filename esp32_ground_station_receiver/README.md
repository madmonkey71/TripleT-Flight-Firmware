# ESP32 Ground Station Receiver

Ground-side ESP32 that receives TripleT telemetry over ESP-NOW and emits a text line on USB Serial for the web console.

## Files

- `esp32_ground_station_receiver.cpp` — Arduino-style firmware (ESP-NOW receive → CSV-shaped USB output).
- `telemetry_packet.h` — wire-format struct. **Must stay in sync with `src/telemetry.h`** in the parent project and the matching file in `esp32_telemetry_transmitter/`.

## Build

This directory is intended as the `src/` of a separate PlatformIO project for the ESP32. Recommended `platformio.ini`:

```ini
[env:esp32_telemetry_rx]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
```

## USB output format

One line per received packet:

```
TELEM,<timestamp_ms>,<flight_state>,<error_code>,<lat_deg>,<lon_deg>,<alt_gps_m>,<alt_baro_m>,<ax_g>,<ay_g>,<az_g>,<q0>,<q1>,<q2>,<q3>,<battery_v>,<stability_flags>,<guidance_active>
```

The `TELEM,` prefix lets the [web console](../web_interface/) distinguish radio packets from any pass-through CSV. Lines beginning with `#` are status / diagnostic messages — the parser should ignore them.

Example:

```
# ESP32 RX MAC: 24:6F:28:AB:CD:EF
# ESP32 RX: received=0 dropped=0
TELEM,1234567,5,0,34.0000123,-118.0000456,1234.567,1234.500,0.012,-0.003,0.998,0.99988,0.00012,0.01540,0.00220,11.100,0,1
```

## Setup

1. Build and flash this firmware to a second ESP32.
2. Open the USB Serial monitor at 115200 baud — the firmware prints its MAC address at startup.
3. Copy that MAC into `ground_station_mac[6]` in `../esp32_telemetry_transmitter/esp32_telemetry_transmitter.cpp` and rebuild the TX firmware.
4. Flash the TX firmware to the onboard ESP32, wire it to Teensy `Serial5`, and set `ENABLE_TELEMETRY=1` in `../src/config.h`.

## Protocol

See `../wiki/entities/esp32-telemetry.md` for the full link topology and the rationale for each field in the packet.
