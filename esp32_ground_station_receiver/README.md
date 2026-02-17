# ESP32 Ground Station Receiver

This directory is a placeholder for the ESP32 Ground Station Receiver firmware.

## Purpose

The ESP32 Ground Station Receiver will be responsible for:
1.  Receiving telemetry data wirelessly from the onboard ESP32 Telemetry Transmitter (likely via ESP-NOW).
2.  Forwarding this data to a connected computer via USB serial.
3.  The data format over USB serial should be compatible with the existing web interface or other ground control software (e.g., CSV).

## Additional Components (Future)

This ground station might eventually include:
-   A small OLED display for status information.
-   Physical controls or buttons.
-   Its own web interface for configuration or direct data viewing if not connected to the main ground control software.

## Project Status

-   This is currently a placeholder for the C++ source code (`esp32_ground_station_receiver.cpp`).
-   A dedicated PlatformIO project will be created for this ESP32 firmware at a later stage.
-   The exact wireless protocol (assumed ESP-NOW) and data handling are yet to be finalized.
