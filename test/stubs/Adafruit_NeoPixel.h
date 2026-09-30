#pragma once
// Native stub of the NeoPixel driver. Records the last colour written to each
// pixel so tests can assert on LED state (e.g. "degraded" orange).
#include "Arduino.h"
#define NEO_GRB 0x0052
#define NEO_KHZ800 0x0000
class Adafruit_NeoPixel {
public:
  Adafruit_NeoPixel(uint16_t n = 1, int16_t = 2, uint16_t = 0) : n_(n) {}
  void begin() {}
  void show() { shows++; }
  void setBrightness(uint8_t) {}
  void setPixelColor(uint16_t i, uint32_t c) { if (i < 8) color[i] = c; }
  static uint32_t Color(uint8_t r, uint8_t g, uint8_t b) { return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b; }
  uint32_t color[8] = {0};
  int shows = 0;
private:
  uint16_t n_;
};
