# Telemetry System Implementation Plan

## 1. Overview
The goal is to implement a robust, real-time telemetry link from the flying rocket (Teensy 4.1) to a Ground Station (ESP32) and finally to a PC for visualization.

**Architecture:**
`Teensy 4.1 (UART)` -> `ESP32 Transmitter (ESP-NOW)` -> `ESP32 Receiver (USB Serial)` -> `Web Interface`

## 2. Hardware Architecture

### 2.1. Connections
*   **Onboard:**
    *   **Teensy 4.1:** Connect `Serial1` (Pins 0/1) or `Serial5` (Pins 21/20) to the ESP32.
        *   *Recommendation:* Use `Serial5` (Tx: 21, Rx: 20) if free, to keep `Serial1` available for GPS or debugging.
        *   *Pins:* Teensy TX -> ESP32 RX, Teensy RX -> ESP32 TX, GND -> GND.
    *   **ESP32 Transmitter:** Powered by the Teensy's 3.3V rail or VBAT.
*   **Ground:**
    *   **ESP32 Receiver:** Connected via USB to the Ground Station Laptop.

### 2.2. ESP-NOW Protocol
*   **Mode:** One-way broadcast (Rocket -> Ground) or Peer-to-Peer. Broadcast is simpler for reconnection.
*   **Frequency:** 2.4GHz.
*   **Packet Size:** Max 250 bytes. We must ensure our telemetry packet fits or is split.

## 3. Software Architecture

### 3.1. Data Packet Structure (Shared C Struct)
To minimize bandwidth, transmit binary structs, not CSV strings.
```cpp
struct TelemetryPacket {
    uint32_t timestamp;
    uint8_t state;
    float lat, lon, alt;
    float accel[3];
    float gyro[3];
    float orientation[3]; // Roll, Pitch, Yaw
    float battery_voltage;
    uint8_t checksum;
} __attribute__((packed));
```

### 3.2. Teensy Firmware Updates
1.  **Define Protocol:** Create `telemetry_protocol.h` with the struct above.
2.  **Serial Output:** In `WriteLogData()`, populate this struct and write it to `Serial5.write((uint8_t*)&packet, sizeof(packet))`.
3.  **Config:** Add `#define ENABLE_TELEMETRY` and `#define TELEMETRY_SERIAL Serial5` in `config.h`.

### 3.3. ESP32 Transmitter Firmware
1.  **Loop:** Listen on `Serial2` (HardwareSerial).
2.  **Buffer:** Read bytes until `sizeof(TelemetryPacket)` is received.
3.  **Verify:** Check checksum/magic byte.
4.  **Send:** `esp_now_send(broadcastAddress, data, len)`.

### 3.4. ESP32 Receiver Firmware
1.  **Callback:** `OnDataRecv` receives the packet.
2.  **Forward:** Convert the binary struct to the CSV format expected by the Web Interface (`LogDataToString` logic) or forward raw binary if the Web Interface is updated to parse it.
    *   *Recommendation:* Forward as CSV line (`Serial.println("...,...,...")`) to maintain compatibility with the existing Web Interface without modifying it immediately.

## 4. Implementation Steps

### Step 1: Protocol Definition
- [ ] Create `src/telemetry_protocol.h` with the packed struct.

### Step 2: Teensy Implementation
- [ ] Initialize `TELEMETRY_SERIAL` in `setup()`.
- [ ] Create `sendTelemetryPacket()` function in `TripleT_Flight_Firmware.cpp`.
- [ ] Call it inside `WriteLogData` (or a separate task to avoid blocking logging).

### Step 3: ESP32 Transmitter
- [ ] Create PlatformIO environment for `esp32_telemetry_transmitter`.
- [ ] Implement UART reading and ESP-NOW broadcasting.

### Step 4: ESP32 Receiver
- [ ] Create PlatformIO environment for `esp32_ground_station_receiver`.
- [ ] Implement ESP-NOW receiving and Serial CSV formatting.

### Step 5: Integration Test
- [ ] Verify data flows from Teensy -> ESP32 -> Ground ESP32 -> Serial Monitor.
