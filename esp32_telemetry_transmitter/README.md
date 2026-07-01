# ESP32 Telemetry Transmitter (Onboard)

Onboard ESP32 that bridges the Teensy 4.1 to the ground-station ESP32 via ESP-NOW.

## Files

- `esp32_telemetry_transmitter.cpp` — Arduino-style firmware (UART2 → frame parser → ESP-NOW broadcast).
- `telemetry_packet.h` — wire-format struct definition. **Must stay in sync with `src/telemetry.h` in the parent project** and with the matching file in `esp32_ground_station_receiver/`.

## Build

This directory is intended as the `src/` of a separate PlatformIO project for the ESP32. Recommended `platformio.ini`:

```ini
[env:esp32_telemetry_tx]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
```

Copy `esp32_telemetry_transmitter.cpp` and `telemetry_packet.h` into the new project's `src/` directory and build with `pio run`.

## Wiring (Teensy 4.1 → ESP32)

| Teensy 4.1 | ESP32 | Notes |
|------------|-------|-------|
| `Serial5 TX` (pin 20) | UART2 RX (default GPIO16) | Telemetry frames in, 115200 baud |
| `GND` | `GND` | Required common reference |
| `5V` (optional) | `VIN` | Power the ESP32 from the flight battery |

The Teensy is `Serial5` because `Serial1–4` are reserved for sensors. The link is one-way (Teensy → ESP32 only).

## Configuration

Edit the top of `esp32_telemetry_transmitter.cpp`:

- `ground_station_mac[6]` — set to the receiver ESP32's MAC. The receiver prints its MAC on USB Serial during setup; copy that value here.
- `TELEMETRY_UART_RX_PIN` — defaults to GPIO16; change if your wiring differs.
- `STATUS_LED_PIN` — onboard LED; defaults to GPIO2.

## Status output

USB Serial (115200 baud) prints a status line every 5 s:

```
ESP32 TX: ok=1234 dropped=2
```

`ok` counts ESP-NOW sends that returned ESP_OK; `dropped` counts CRC failures and ESP-NOW send errors.

## Protocol

See `../wiki/entities/esp32-telemetry.md` for the full link topology and packet layout. Summary:

- UART frame: `[SYNC=0xA5][LEN=40][... 40-byte payload ...][CRC8]` — 43 bytes total.
- ESP-NOW payload: bare `TelemetryPacket` struct (40 bytes), no framing (ESP-NOW provides its own integrity).
- CRC-8/SMBUS (poly 0x07) over the payload only.
