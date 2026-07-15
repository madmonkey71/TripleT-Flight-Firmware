// ESP32 Telemetry Transmitter (Onboard)
//
// Role: receive framed telemetry packets from the Teensy 4.1 over UART2,
// validate the CRC-8, and broadcast the unframed payload via ESP-NOW to the
// ground-station ESP32.
//
// Wire format: see telemetry_packet.h (must match src/telemetry.h on the Teensy).
//
// Build: this is intended to live in its own PlatformIO project for the ESP32
// (board: esp32dev or similar). Drop telemetry_packet.h and this .cpp into the
// project's src/ directory.

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "telemetry_packet.h"

// --- Configuration -----------------------------------------------------------
// Adjust pin numbers / MAC to match your wiring + ground-station MAC.

#define TELEMETRY_UART_RX_PIN 16   // ESP32 UART2 RX (connect to Teensy Serial5 TX)
#define TELEMETRY_UART_TX_PIN 17   // ESP32 UART2 TX (unused — Teensy doesn't read back)
#define TELEMETRY_BAUD        115200
#define STATUS_LED_PIN        2    // Onboard LED on most ESP32 dev boards

// Replace with the ground-station ESP32's MAC address.
static uint8_t ground_station_mac[6] = {0x24, 0x6F, 0x28, 0x00, 0x00, 0x00};

// --- State machine for the UART framing parser -------------------------------
enum ParseState {
    WAIT_SYNC,
    WAIT_LEN,
    WAIT_PAYLOAD,
    WAIT_CRC
};

static ParseState  parse_state   = WAIT_SYNC;
static uint8_t     payload_buf[TELEMETRY_PACKET_SIZE];
static uint8_t     payload_idx   = 0;
static uint8_t     expected_len  = 0;
static uint32_t    packets_ok    = 0;
static uint32_t    packets_dropped = 0;

static void on_data_sent(const uint8_t *, esp_now_send_status_t status) {
    // Optional: extend for retry / link-quality reporting.
    digitalWrite(STATUS_LED_PIN, status == ESP_NOW_SEND_SUCCESS ? HIGH : LOW);
}

static void handle_complete_packet() {
    if (esp_now_send(ground_station_mac, payload_buf, TELEMETRY_PACKET_SIZE) == ESP_OK) {
        ++packets_ok;
    } else {
        ++packets_dropped;
    }
}

static void process_byte(uint8_t b) {
    switch (parse_state) {
        case WAIT_SYNC:
            if (b == TELEMETRY_SYNC_BYTE) {
                parse_state = WAIT_LEN;
            }
            break;
        case WAIT_LEN:
            if (b == TELEMETRY_PACKET_SIZE) {
                expected_len = b;
                payload_idx  = 0;
                parse_state  = WAIT_PAYLOAD;
            } else {
                // Bad length — resync.
                parse_state = WAIT_SYNC;
            }
            break;
        case WAIT_PAYLOAD:
            payload_buf[payload_idx++] = b;
            if (payload_idx >= expected_len) {
                parse_state = WAIT_CRC;
            }
            break;
        case WAIT_CRC: {
            uint8_t expected = telemetry_crc8(payload_buf, expected_len);
            if (expected == b) {
                handle_complete_packet();
            } else {
                ++packets_dropped;
            }
            parse_state = WAIT_SYNC;
            break;
        }
    }
}

void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    Serial.begin(115200);                // USB debug
    Serial2.begin(TELEMETRY_BAUD, SERIAL_8N1, TELEMETRY_UART_RX_PIN, TELEMETRY_UART_TX_PIN);
    Serial.println(F("ESP32 TX: UART2 listening for TripleT telemetry frames"));

    // ESP-NOW init
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println(F("ESP32 TX: esp_now_init() failed"));
        return;
    }
    esp_now_register_send_cb(on_data_sent);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, ground_station_mac, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) != ESP_OK) {
        Serial.println(F("ESP32 TX: esp_now_add_peer() failed"));
    }
}

void loop() {
    while (Serial2.available()) {
        process_byte((uint8_t)Serial2.read());
    }

    // Periodic status print, throttled.
    static uint32_t last_report = 0;
    if (millis() - last_report >= 5000) {
        Serial.printf("ESP32 TX: ok=%lu dropped=%lu\n",
                      (unsigned long)packets_ok, (unsigned long)packets_dropped);
        last_report = millis();
    }
}
