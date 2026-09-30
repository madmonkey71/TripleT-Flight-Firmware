#pragma once
// Native RAM-backed EEPROM with write accounting. Mirrors the Teensy EEPROM
// get/put API used by state_management.cpp. `put` uses update semantics per byte
// (like Teensy's EEPROM.update) and counts bytes actually changed so tests can
// verify wear behaviour and inspect exactly what is persisted.
#include "Arduino.h"
class EEPROMClass {
public:
  static constexpr int SIZE = 4096;
  uint8_t mem[SIZE];
  unsigned long put_calls = 0;      // number of put() calls
  unsigned long bytes_changed = 0;  // bytes whose value actually changed
  EEPROMClass() { memset(mem, 0xFF, SIZE); }
  void wipe() { memset(mem, 0xFF, SIZE); put_calls = 0; bytes_changed = 0; }
  uint8_t read(int addr) const { return mem[addr]; }
  void update(int addr, uint8_t v) { if (mem[addr] != v) { mem[addr] = v; bytes_changed++; } }
  void write(int addr, uint8_t v) { update(addr, v); }
  template <typename T> T& get(int addr, T& t) const { memcpy(&t, mem + addr, sizeof(T)); return t; }
  template <typename T> const T& put(int addr, const T& t) {
    put_calls++;
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&t);
    for (size_t i = 0; i < sizeof(T); i++) update(addr + (int)i, p[i]);
    return t;
  }
};
inline EEPROMClass EEPROM;
