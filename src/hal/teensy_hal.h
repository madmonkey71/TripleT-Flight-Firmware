#ifndef TEENSY_HAL_H
#define TEENSY_HAL_H

#include "hal_interfaces.h"
#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Watchdog_t4.h>

// ============================================================================
// TEENSY TIMER IMPLEMENTATION
// ============================================================================
class TeensyTimer : public ITimer {
public:
  static TeensyTimer& instance() {
    static TeensyTimer _instance;
    return _instance;
  }

  uint32_t millis() override {
    return ::millis();
  }

  uint32_t micros() override {
    return ::micros();
  }

  void delay(uint32_t ms) override {
    ::delay(ms);
  }

  void delayMicroseconds(uint16_t us) override {
    ::delayMicroseconds(us);
  }

private:
  TeensyTimer() = default;
};

// ============================================================================
// TEENSY SERIAL IMPLEMENTATION
// ============================================================================
class TeensySerial : public ISerial {
public:
  static TeensySerial& instance() {
    static TeensySerial _instance;
    return _instance;
  }

  void print(const char* str) override {
    Serial.print(str);
  }

  void println(const char* str) override {
    Serial.println(str);
  }

  void write(uint8_t byte) override {
    Serial.write(byte);
  }

  int read() override {
    return Serial.read();
  }

  bool available() override {
    return Serial.available() > 0;
  }

  int peek() override {
    return Serial.peek();
  }

  void flush() override {
    Serial.flush();
  }

private:
  TeensySerial() = default;
};

// ============================================================================
// TEENSY GPIO IMPLEMENTATION
// ============================================================================
class TeensyGPIO : public IGPIO {
public:
  static TeensyGPIO& instance() {
    static TeensyGPIO _instance;
    return _instance;
  }

  void pinMode(uint8_t pin, uint8_t mode) override {
    ::pinMode(pin, mode);
  }

  void digitalWrite(uint8_t pin, uint8_t value) override {
    ::digitalWrite(pin, value);
  }

  int digitalRead(uint8_t pin) override {
    return ::digitalRead(pin);
  }

  void analogWrite(uint8_t pin, uint8_t value) override {
    ::analogWrite(pin, value);
  }

  int analogRead(uint8_t pin) override {
    return ::analogRead(pin);
  }

private:
  TeensyGPIO() = default;
};

// ============================================================================
// TEENSY I2C IMPLEMENTATION
// ============================================================================
class TeensyI2C : public II2C {
public:
  static TeensyI2C& instance() {
    static TeensyI2C _instance;
    return _instance;
  }

  bool begin(uint8_t sda_pin = 0, uint8_t scl_pin = 0) override {
    // Teensy uses default I2C pins, sda_pin and scl_pin are ignored
    Wire.begin();
    return true;
  }

  void beginTransmission(uint8_t address) override {
    Wire.beginTransmission(address);
  }

  size_t write(const uint8_t* data, size_t length) override {
    return Wire.write(data, length);
  }

  size_t write(uint8_t byte) override {
    return Wire.write(byte);
  }

  uint8_t endTransmission(bool sendStop = true) override {
    return Wire.endTransmission(sendStop);
  }

  size_t requestFrom(uint8_t address, size_t quantity, bool sendStop = true) override {
    return Wire.requestFrom(address, quantity, sendStop);
  }

  int read() override {
    return Wire.read();
  }

  int peek() override {
    return Wire.peek();
  }

  bool available() override {
    return Wire.available() > 0;
  }

private:
  TeensyI2C() = default;
};

// ============================================================================
// TEENSY EEPROM IMPLEMENTATION
// ============================================================================
class TeensyEEPROM : public IEEPROM {
public:
  static TeensyEEPROM& instance() {
    static TeensyEEPROM _instance;
    return _instance;
  }

  bool begin() override {
    // Teensy EEPROM is always available
    return true;
  }

  uint8_t read(uint16_t address) override {
    return EEPROM.read(address);
  }

  void write(uint16_t address, uint8_t value) override {
    EEPROM.write(address, value);
  }

  void write(uint16_t address, const uint8_t* data, size_t length) override {
    for (size_t i = 0; i < length; i++) {
      EEPROM.write(address + i, data[i]);
    }
  }

  void commit() override {
    // Teensy EEPROM is automatically committed
  }

  size_t length() override {
    return EEPROM.length();
  }

private:
  TeensyEEPROM() = default;
};

// ============================================================================
// TEENSY SD CARD IMPLEMENTATION (Placeholder - uses existing SdFat)
// ============================================================================
class TeensySDCard : public ISDCard {
public:
  static TeensySDCard& instance() {
    static TeensySDCard _instance;
    return _instance;
  }

  bool begin(uint8_t cs_pin) override {
    // TODO: Integrate with existing SD card initialization
    return true;
  }

  bool openFile(const char* filename) override {
    // TODO: Use existing SdFat file operations
    return true;
  }

  bool writeData(const uint8_t* data, size_t length) override {
    // TODO: Write to current file
    return true;
  }

  bool closeFile() override {
    // TODO: Close file and flush
    return true;
  }

  bool isHealthy() override {
    return true;
  }

  uint32_t getLastError() override {
    return 0;
  }

private:
  TeensySDCard() = default;
};

// ============================================================================
// TEENSY SERVO IMPLEMENTATION
// ============================================================================
class TeensyServo : public IServo {
public:
  static TeensyServo& instance() {
    static TeensyServo _instance;
    return _instance;
  }

  bool attach(uint8_t pin, uint16_t min_us = 1000, uint16_t max_us = 2000) override {
    // For now, just set as output - full servo library integration later
    pinMode(pin, OUTPUT);
    return true;
  }

  void detach(uint8_t pin) override {
    pinMode(pin, INPUT);
  }

  void writeAngle(uint8_t pin, uint8_t angle) override {
    // TODO: Map 0-180 degrees to PWM
    analogWrite(pin, (angle * 255) / 180);
  }

  void writeMicroseconds(uint8_t pin, uint16_t us) override {
    // TODO: Map microseconds to PWM value
    // analogWrite expects 0-255, not microseconds
    analogWrite(pin, (us - 1000) / 4);  // Rough approximation
  }

  uint8_t readAngle(uint8_t pin) override {
    // TODO: Read current angle from servo library
    return 90;
  }

  uint16_t readMicroseconds(uint8_t pin) override {
    // TODO: Read current microseconds from servo library
    return 1500;
  }

private:
  TeensyServo() = default;
};

// ============================================================================
// TEENSY WATCHDOG IMPLEMENTATION
// ============================================================================
class TeensyWatchdog : public IWatchdog {
public:
  static TeensyWatchdog& instance() {
    static TeensyWatchdog _instance;
    return _instance;
  }

  bool begin(uint32_t timeout_ms) override {
    extern WDT_T4<WDT1> wdt;
    // WDT_timings_t is a struct with timeout in milliseconds
    WDT_timings_t wdt_config = {timeout_ms};
    wdt.begin(wdt_config);
    return true;
  }

  void feed() override {
    extern WDT_T4<WDT1> wdt;
    wdt.feed();
  }

  void reset() override {
    extern WDT_T4<WDT1> wdt;
    wdt.reset();
  }

  uint32_t getTimeout() override {
    // Not easily queryable from Teensy WDT API
    return 5000;  // Default 5-second timeout
  }

  bool isWatchdogReset() override {
    // Check if system was reset by watchdog (implementation platform-specific)
    return false;
  }

private:
  TeensyWatchdog() = default;
};

#endif // TEENSY_HAL_H
