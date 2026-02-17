# ESP32 Telemetry Transmitter

This directory is a placeholder for the ESP32 Telemetry Transmitter firmware.

## Purpose

The ESP32 Telemetry Transmitter will be responsible for:
1.  Receiving telemetry data from the main flight computer (Teensy 4.1) via a serial (UART) connection or other bus (I2C/SPI).
2.  Transmitting this data wirelessly to the ESP32 Ground Station Receiver, likely using the ESP-NOW protocol for a direct, low-latency link.

## Project Status

-   This is currently a placeholder for the C++ source code (`esp32_telemetry_transmitter.cpp`).
-   A dedicated PlatformIO project will be created for this ESP32 firmware at a later stage when requirements are further defined.
-   The communication protocol between the Teensy and this ESP32, and the exact data format, are yet to be finalized.
-   Hardware setup will involve connecting one of the ESP32's UARTs (or I2C/SPI pins) to a corresponding port on the Teensy. Due to the single available UART constraint on the Teensy for this purpose, careful planning of shared UART usage or selection of an alternative bus is required.
