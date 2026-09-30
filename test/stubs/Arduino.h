// Native (host) stand-in for the Arduino core, used ONLY by the unit-test build
// ([env:native], added via `-I test/stubs`). It lets the REAL firmware sources
// (flight_logic.cpp, state_management.cpp, kalman_filter.cpp, pyro_control.cpp ...)
// compile and run on a desktop with:
//   * a fully controllable millis()/micros() clock (advance with test_advance_ms()),
//   * recorded pin modes / levels / write history (so tests can prove a pyro pin
//     was never driven HIGH),
//   * captured Serial output (so tests can assert on operator-visible messages).
// It deliberately implements only what the firmware sources under test use.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>
#include <cstdarg>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef uint8_t byte;
typedef bool boolean;

#define PI 3.1415926535897932384626433832795
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define HEX 16
#define DEC 10
#define A7 21

#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

// --- PROGMEM string helper --------------------------------------------------
class __FlashStringHelper;
#define F(str) (reinterpret_cast<const __FlashStringHelper*>(str))
#define PSTR(str) (str)

// --- Test clock ---------------------------------------------------------------
inline unsigned long g_test_millis = 0;
inline unsigned long millis() { return g_test_millis; }
inline unsigned long micros() { return g_test_millis * 1000UL; }
inline void test_set_ms(unsigned long ms) { g_test_millis = ms; }
inline void test_advance_ms(unsigned long ms) { g_test_millis += ms; }
// delay() advances the fake clock so blocking firmware code paths are observable.
inline void delay(unsigned long ms) { g_test_millis += ms; }
inline void delayMicroseconds(unsigned int) {}

// --- Pins ----------------------------------------------------------------------
constexpr int TEST_NUM_PINS = 64;
inline int g_pin_mode[TEST_NUM_PINS];          // last pinMode()
inline int g_pin_level[TEST_NUM_PINS];         // last digitalWrite()
inline bool g_pin_ever_high[TEST_NUM_PINS];    // any digitalWrite(pin, HIGH) since test_pins_reset()
inline unsigned g_pin_high_writes[TEST_NUM_PINS];
inline unsigned long g_pin_last_high_ms[TEST_NUM_PINS];
inline void test_pins_reset() {
  for (int i = 0; i < TEST_NUM_PINS; i++) {
    g_pin_mode[i] = INPUT; g_pin_level[i] = LOW; g_pin_ever_high[i] = false;
    g_pin_high_writes[i] = 0; g_pin_last_high_ms[i] = 0;
  }
}
inline void pinMode(uint8_t pin, uint8_t mode) { if (pin < TEST_NUM_PINS) g_pin_mode[pin] = mode; }
inline void digitalWrite(uint8_t pin, uint8_t val) {
  if (pin >= TEST_NUM_PINS) return;
  g_pin_level[pin] = val;
  if (val == HIGH) { g_pin_ever_high[pin] = true; g_pin_high_writes[pin]++; g_pin_last_high_ms[pin] = g_test_millis; }
}
inline int digitalRead(uint8_t pin) { return pin < TEST_NUM_PINS ? g_pin_level[pin] : LOW; }
inline int analogRead(uint8_t) { return 0; }
inline int g_tone_calls = 0;
inline void tone(uint8_t, unsigned int, unsigned long = 0) { g_tone_calls++; }
inline void noTone(uint8_t) {}

// --- Serial ----------------------------------------------------------------------
// Captures everything printed so tests can assert on operator-visible output.
class SerialStub {
public:
  std::string out;
  void begin(unsigned long) {}
  operator bool() const { return true; }
  int available() { return 0; }
  int read() { return -1; }
  void clear() { out.clear(); }
  bool contains(const char* needle) const { return out.find(needle) != std::string::npos; }
  size_t write(const uint8_t*, size_t n) { return n; }
  size_t print(const char* s) { if (s) out += s; return s ? strlen(s) : 0; }
  size_t print(const __FlashStringHelper* s) { return print(reinterpret_cast<const char*>(s)); }
  size_t print(char c) { out += c; return 1; }
  size_t print(int v, int base = 10) { return printInt((long)v, base); }
  size_t print(unsigned int v, int base = 10) { return printInt((long)v, base); }
  size_t print(long v, int base = 10) { return printInt(v, base); }
  size_t print(unsigned long v, int base = 10) { return printInt((long)v, base); }
  size_t print(unsigned char v, int base = 10) { return printInt((long)v, base); }
  size_t print(double v, int digits = 2) { char b[48]; snprintf(b, sizeof b, "%.*f", digits, v); return print(b); }
  size_t println() { out += "\n"; return 1; }
  template <typename T> size_t println(T v) { size_t n = print(v); out += "\n"; return n + 1; }
  size_t println(double v, int digits) { size_t n = print(v, digits); out += "\n"; return n + 1; }
  size_t println(int v, int base) { size_t n = print(v, base); out += "\n"; return n + 1; }
  int printf(const char* fmt, ...) {
    char b[512]; va_list ap; va_start(ap, fmt); int n = vsnprintf(b, sizeof b, fmt, ap); va_end(ap); print(b); return n;
  }
private:
  size_t printInt(long v, int base) { char b[40]; snprintf(b, sizeof b, base == 16 ? "%lX" : "%ld", v); return print(b); }
};
inline SerialStub Serial;

// Minimal Stream base so headers that mention it still parse.
class Stream : public SerialStub {};

// --- String ------------------------------------------------------------------------
class String {
public:
  String() {}
  String(const char* s) : s_(s ? s : "") {}
  String(const std::string& s) : s_(s) {}
  String(int v) : s_(std::to_string(v)) {}
  const char* c_str() const { return s_.c_str(); }
  size_t length() const { return s_.length(); }
private:
  std::string s_;
};
