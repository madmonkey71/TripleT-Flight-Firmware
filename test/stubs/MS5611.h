#pragma once
// Native stub of the MS5611 barometer library. Only the members referenced by
// headers included from the sources under test are provided.
#include "Arduino.h"
#define MS5611_READ_OK 0
#define OSR_HIGH 4
#define MS5611_LIB_VERSION "stub"
class MS5611 {
public:
  MS5611(uint8_t = 0x77) {}
  bool begin() { return true; }
  bool isConnected() { return connected; }
  int read() { return MS5611_READ_OK; }
  float getPressure() { return 1013.25f; }
  float getTemperature() { return 20.0f; }
  uint8_t getAddress() { return 0x77; }
  void setOversampling(int) {}
  bool connected = true;
};
