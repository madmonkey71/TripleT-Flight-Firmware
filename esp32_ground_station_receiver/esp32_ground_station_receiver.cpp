// ESP32 Ground Station Receiver
//
// Role: receive TelemetryPacket frames over ESP-NOW from the onboard ESP32
// TX, unpack them, and emit a CSV-shaped text line on USB Serial for the
// web console.
//
// Output line format (one per packet, no header):
//
//   TELEM,<timestamp_ms>,<flight_state>,<error_code>,<lat_deg>,<lon_deg>,
//   <alt_gps_m>,<alt_baro_m>,<ax_g>,<ay_g>,<az_g>,<q0>,<q1>,<q2>,<q3>,
//   <battery_v>,<stability_flags>,<guidance_active>
//
// The `TELEM,` prefix distinguishes radio packets from any pass-through
// CSV on USB. The web interface can parse this format separately from
// the 62-field SD-card CSV.
//
// Build: this is intended to live in its own PlatformIO project for the
// ESP32 (board: esp32dev or similar).

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include "telemetry_packet.h"

#define STATUS_LED_PIN 2

static volatile uint32_t packets_received = 0;
static volatile uint32_t packets_dropped  = 0;

static void emit_packet(const TelemetryPacket &p) {
    // Scale-back constants must match src/telemetry.h.
    float lat   = p.latitude_e7      / 1.0e7f;
    float lon   = p.longitude_e7     / 1.0e7f;
    float alt_g = p.altitude_gps_mm  / 1000.0f;
    float alt_b = p.altitude_baro_mm / 1000.0f;
    float ax    = p.accel_x_mg       / 1000.0f;
    float ay    = p.accel_y_mg       / 1000.0f;
    float az    = p.accel_z_mg       / 1000.0f;
    float q0    = p.q0_q14           / 16384.0f;
    float q1    = p.q1_q14           / 16384.0f;
    float q2    = p.q2_q14           / 16384.0f;
    float q3    = p.q3_q14           / 16384.0f;
    float vbat  = p.battery_mv       / 1000.0f;

    // printf format keeps this short and predictable for the web parser.
    Serial.printf(
        "TELEM,%lu,%u,%u,%.7f,%.7f,%.3f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.5f,%.3f,%u,%u\n",
        (unsigned long)p.timestamp_ms,
        (unsigned)p.flight_state,
        (unsigned)p.error_code,
        lat, lon,
        alt_g, alt_b,
        ax, ay, az,
        q0, q1, q2, q3,
        vbat,
        (unsigned)p.stability_flags,
        (unsigned)p.guidance_active);
}

static void on_data_recv(const esp_now_recv_info_t * /*info*/,
                         const uint8_t *data, int len) {
    if (len != TELEMETRY_PACKET_SIZE) {
        ++packets_dropped;
        return;
    }
    TelemetryPacket p;
    memcpy(&p, data, TELEMETRY_PACKET_SIZE);
    emit_packet(p);
    ++packets_received;
    digitalWrite(STATUS_LED_PIN, !digitalRead(STATUS_LED_PIN));
}

void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW);

    Serial.begin(115200);
    delay(100);
    Serial.println(F("# ESP32 RX: starting ESP-NOW receiver"));

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println(F("# ESP32 RX: esp_now_init() failed"));
        return;
    }
    esp_now_register_recv_cb(on_data_recv);

    // Print this MAC so the user knows what to put in the TX firmware.
    Serial.print(F("# ESP32 RX MAC: "));
    Serial.println(WiFi.macAddress());
}

void loop() {
    // Periodic status to USB (commented lines so the web parser ignores them).
    static uint32_t last_report = 0;
    if (millis() - last_report >= 5000) {
        Serial.printf("# ESP32 RX: received=%lu dropped=%lu\n",
                      (unsigned long)packets_received,
                      (unsigned long)packets_dropped);
        last_report = millis();
    }
}
