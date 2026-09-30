#pragma once
// Native stub: I2C bus is never touched by code under test.
#include "Arduino.h"
class TwoWire {
public:
  void begin() {}
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t) {}
  uint8_t endTransmission() { return 0; }
};
inline TwoWire Wire;
