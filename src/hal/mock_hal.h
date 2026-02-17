#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#include "hal_interfaces.h"
#include <stdint.h>
#include <string.h>

// ============================================================================
// MOCK TIMER IMPLEMENTATION (Controllable for testing)
// ============================================================================
class MockTimer : public ITimer {
public:
  static MockTimer& instance() {
    static MockTimer _instance;
    return _instance;
  }

  uint32_t millis() override {
    return current_time_ms;
  }

  uint32_t micros() override {
    return current_time_ms * 1000 + current_time_us;
  }

  void delay(uint32_t ms) override {
    // In testing, advance time instead of blocking
    current_time_ms += ms;
  }

  void delayMicroseconds(uint16_t us) override {
    // In testing, advance time instead of blocking
    current_time_us += us;
    if (current_time_us >= 1000) {
      current_time_ms += current_time_us / 1000;
      current_time_us %= 1000;
    }
  }

  // Testing utilities
  void setTime(uint32_t ms) {
    current_time_ms = ms;
    current_time_us = 0;
  }

  void advanceTime(uint32_t ms) {
    current_time_ms += ms;
  }

  void reset() {
    current_time_ms = 0;
    current_time_us = 0;
  }

private:
  MockTimer() : current_time_ms(0), current_time_us(0) {}
  uint32_t current_time_ms;
  uint16_t current_time_us;
};

// ============================================================================
// MOCK SERIAL IMPLEMENTATION (Buffered for testing)
// ============================================================================
class MockSerial : public ISerial {
public:
  static MockSerial& instance() {
    static MockSerial _instance;
    return _instance;
  }

  void print(const char* str) override {
    if (str) {
      output_buffer += str;
      print_count++;
    }
  }

  void println(const char* str) override {
    if (str) {
      output_buffer += str;
      output_buffer += "\n";
      println_count++;
    }
  }

  void write(uint8_t byte) override {
    output_buffer += (char)byte;
    write_count++;
  }

  int read() override {
    if (input_position < input_buffer.length()) {
      return input_buffer[input_position++];
    }
    return -1;
  }

  bool available() override {
    return input_position < input_buffer.length();
  }

  int peek() override {
    if (input_position < input_buffer.length()) {
      return input_buffer[input_position];
    }
    return -1;
  }

  void flush() override {
    // No-op for mock
  }

  // Testing utilities
  std::string getOutput() const {
    return output_buffer;
  }

  void setInput(const char* data) {
    input_buffer = data;
    input_position = 0;
  }

  void clearOutput() {
    output_buffer.clear();
    print_count = 0;
    println_count = 0;
    write_count = 0;
  }

  int getPrintCount() const { return print_count; }
  int getPrintlnCount() const { return println_count; }
  int getWriteCount() const { return write_count; }

private:
  MockSerial() : input_position(0), print_count(0), println_count(0), write_count(0) {}
  std::string output_buffer;
  std::string input_buffer;
  size_t input_position;
  int print_count;
  int println_count;
  int write_count;
};

// ============================================================================
// MOCK GPIO IMPLEMENTATION (State tracking for testing)
// ============================================================================
class MockGPIO : public IGPIO {
public:
  static MockGPIO& instance() {
    static MockGPIO _instance;
    return _instance;
  }

  void pinMode(uint8_t pin, uint8_t mode) override {
    pin_modes[pin] = mode;
  }

  void digitalWrite(uint8_t pin, uint8_t value) override {
    digital_values[pin] = value;
    digital_write_count++;
  }

  int digitalRead(uint8_t pin) override {
    digital_read_count++;
    return digital_values[pin];
  }

  void analogWrite(uint8_t pin, uint8_t value) override {
    analog_values[pin] = value;
    analog_write_count++;
  }

  int analogRead(uint8_t pin) override {
    analog_read_count++;
    return analog_values[pin];
  }

  // Testing utilities
  int getPinMode(uint8_t pin) const {
    return pin_modes[pin];
  }

  int getDigitalValue(uint8_t pin) const {
    return digital_values[pin];
  }

  int getAnalogValue(uint8_t pin) const {
    return analog_values[pin];
  }

  void setDigitalValue(uint8_t pin, uint8_t value) {
    digital_values[pin] = value;
  }

  void setAnalogValue(uint8_t pin, uint8_t value) {
    analog_values[pin] = value;
  }

  int getDigitalWriteCount() const { return digital_write_count; }
  int getDigitalReadCount() const { return digital_read_count; }
  int getAnalogWriteCount() const { return analog_write_count; }
  int getAnalogReadCount() const { return analog_read_count; }

  void reset() {
    memset(pin_modes, 0, sizeof(pin_modes));
    memset(digital_values, 0, sizeof(digital_values));
    memset(analog_values, 0, sizeof(analog_values));
    digital_write_count = 0;
    digital_read_count = 0;
    analog_write_count = 0;
    analog_read_count = 0;
  }

private:
  MockGPIO() : digital_write_count(0), digital_read_count(0),
               analog_write_count(0), analog_read_count(0) {
    memset(pin_modes, 0, sizeof(pin_modes));
    memset(digital_values, 0, sizeof(digital_values));
    memset(analog_values, 0, sizeof(analog_values));
  }

  uint8_t pin_modes[60];
  uint8_t digital_values[60];
  uint8_t analog_values[60];
  int digital_write_count;
  int digital_read_count;
  int analog_write_count;
  int analog_read_count;
};

// ============================================================================
// MOCK I2C IMPLEMENTATION (Device simulation for testing)
// ============================================================================
class MockI2C : public II2C {
public:
  static MockI2C& instance() {
    static MockI2C _instance;
    return _instance;
  }

  bool begin(uint8_t sda_pin = 0, uint8_t scl_pin = 0) override {
    is_initialized = true;
    return true;
  }

  void beginTransmission(uint8_t address) override {
    current_address = address;
    transmit_buffer.clear();
  }

  size_t write(const uint8_t* data, size_t length) override {
    for (size_t i = 0; i < length; i++) {
      transmit_buffer.push_back(data[i]);
    }
    return length;
  }

  size_t write(uint8_t byte) override {
    transmit_buffer.push_back(byte);
    return 1;
  }

  uint8_t endTransmission(bool sendStop = true) override {
    // In mock, simulate successful transmission
    transmit_count++;
    return 0;  // I2C_SUCCESS
  }

  size_t requestFrom(uint8_t address, size_t quantity, bool sendStop = true) override {
    current_address = address;
    receive_buffer_position = 0;
    request_count++;
    return receive_buffer.size();
  }

  int read() override {
    if (receive_buffer_position < receive_buffer.size()) {
      return receive_buffer[receive_buffer_position++];
    }
    return -1;
  }

  int peek() override {
    if (receive_buffer_position < receive_buffer.size()) {
      return receive_buffer[receive_buffer_position];
    }
    return -1;
  }

  bool available() override {
    return receive_buffer_position < receive_buffer.size();
  }

  // Testing utilities
  void setReceiveData(const uint8_t* data, size_t length) {
    receive_buffer.clear();
    receive_buffer.insert(receive_buffer.begin(), data, data + length);
    receive_buffer_position = 0;
  }

  const std::vector<uint8_t>& getTransmitBuffer() const {
    return transmit_buffer;
  }

  int getTransmitCount() const { return transmit_count; }
  int getRequestCount() const { return request_count; }

  void reset() {
    transmit_buffer.clear();
    receive_buffer.clear();
    receive_buffer_position = 0;
    transmit_count = 0;
    request_count = 0;
    current_address = 0;
  }

private:
  MockI2C() : is_initialized(false), receive_buffer_position(0),
              transmit_count(0), request_count(0), current_address(0) {}

  bool is_initialized;
  std::vector<uint8_t> transmit_buffer;
  std::vector<uint8_t> receive_buffer;
  size_t receive_buffer_position;
  int transmit_count;
  int request_count;
  uint8_t current_address;
};

// ============================================================================
// MOCK EEPROM IMPLEMENTATION (Memory buffer for testing)
// ============================================================================
class MockEEPROM : public IEEPROM {
public:
  static MockEEPROM& instance() {
    static MockEEPROM _instance;
    return _instance;
  }

  bool begin() override {
    return true;
  }

  uint8_t read(uint16_t address) override {
    if (address < MOCK_EEPROM_SIZE) {
      return eeprom_data[address];
    }
    return 0xFF;
  }

  void write(uint16_t address, uint8_t value) override {
    if (address < MOCK_EEPROM_SIZE) {
      eeprom_data[address] = value;
      write_count++;
    }
  }

  void write(uint16_t address, const uint8_t* data, size_t length) override {
    for (size_t i = 0; i < length && (address + i) < MOCK_EEPROM_SIZE; i++) {
      eeprom_data[address + i] = data[i];
    }
    write_count++;
  }

  void commit() override {
    // No-op for mock
  }

  size_t length() override {
    return MOCK_EEPROM_SIZE;
  }

  // Testing utilities
  uint8_t* getRawData() {
    return eeprom_data;
  }

  int getWriteCount() const { return write_count; }

  void reset() {
    memset(eeprom_data, 0xFF, MOCK_EEPROM_SIZE);
    write_count = 0;
  }

private:
  static constexpr size_t MOCK_EEPROM_SIZE = 4096;

  MockEEPROM() : write_count(0) {
    memset(eeprom_data, 0xFF, MOCK_EEPROM_SIZE);
  }

  uint8_t eeprom_data[MOCK_EEPROM_SIZE];
  int write_count;
};

// ============================================================================
// MOCK SD CARD IMPLEMENTATION (Buffer for testing)
// ============================================================================
class MockSDCard : public ISDCard {
public:
  static MockSDCard& instance() {
    static MockSDCard _instance;
    return _instance;
  }

  bool begin(uint8_t cs_pin) override {
    is_healthy = true;
    return true;
  }

  bool openFile(const char* filename) override {
    current_filename = filename ? filename : "";
    return true;
  }

  bool writeData(const uint8_t* data, size_t length) override {
    for (size_t i = 0; i < length; i++) {
      file_buffer.push_back(data[i]);
    }
    write_count++;
    return true;
  }

  bool closeFile() override {
    close_count++;
    return true;
  }

  bool isHealthy() override {
    return is_healthy;
  }

  uint32_t getLastError() override {
    return 0;
  }

  // Testing utilities
  void setHealthy(bool healthy) {
    is_healthy = healthy;
  }

  const std::vector<uint8_t>& getFileBuffer() const {
    return file_buffer;
  }

  int getWriteCount() const { return write_count; }
  int getCloseCount() const { return close_count; }

  void reset() {
    file_buffer.clear();
    current_filename.clear();
    write_count = 0;
    close_count = 0;
    is_healthy = true;
  }

private:
  MockSDCard() : write_count(0), close_count(0), is_healthy(true) {}

  std::vector<uint8_t> file_buffer;
  std::string current_filename;
  int write_count;
  int close_count;
  bool is_healthy;
};

// ============================================================================
// MOCK SERVO IMPLEMENTATION
// ============================================================================
class MockServo : public IServo {
public:
  static MockServo& instance() {
    static MockServo _instance;
    return _instance;
  }

  bool attach(uint8_t pin, uint16_t min_us = 1000, uint16_t max_us = 2000) override {
    attached_pins[pin] = true;
    return true;
  }

  void detach(uint8_t pin) override {
    attached_pins[pin] = false;
  }

  void writeAngle(uint8_t pin, uint8_t angle) override {
    angles[pin] = angle;
    angle_write_count++;
  }

  void writeMicroseconds(uint8_t pin, uint16_t us) override {
    microseconds[pin] = us;
    us_write_count++;
  }

  uint8_t readAngle(uint8_t pin) override {
    return angles[pin];
  }

  uint16_t readMicroseconds(uint8_t pin) override {
    return microseconds[pin];
  }

  // Testing utilities
  bool isAttached(uint8_t pin) const {
    return attached_pins[pin];
  }

  int getAngleWriteCount() const { return angle_write_count; }
  int getUSWriteCount() const { return us_write_count; }

  void reset() {
    memset(attached_pins, 0, sizeof(attached_pins));
    memset(angles, 0, sizeof(angles));
    memset(microseconds, 0, sizeof(microseconds));
    angle_write_count = 0;
    us_write_count = 0;
  }

private:
  MockServo() : angle_write_count(0), us_write_count(0) {
    memset(attached_pins, 0, sizeof(attached_pins));
    memset(angles, 0, sizeof(angles));
    memset(microseconds, 0, sizeof(microseconds));
  }

  bool attached_pins[20];
  uint8_t angles[20];
  uint16_t microseconds[20];
  int angle_write_count;
  int us_write_count;
};

// ============================================================================
// MOCK WATCHDOG IMPLEMENTATION
// ============================================================================
class MockWatchdog : public IWatchdog {
public:
  static MockWatchdog& instance() {
    static MockWatchdog _instance;
    return _instance;
  }

  bool begin(uint32_t timeout_ms) override {
    wdt_timeout = timeout_ms;
    is_running = true;
    return true;
  }

  void feed() override {
    feed_count++;
    time_since_feed = 0;
  }

  void reset() override {
    reset_count++;
  }

  uint32_t getTimeout() override {
    return wdt_timeout;
  }

  bool isWatchdogReset() override {
    return was_wdt_reset;
  }

  // Testing utilities
  int getFeedCount() const { return feed_count; }
  int getResetCount() const { return reset_count; }
  void setWasWdtReset(bool was_reset) { was_wdt_reset = was_reset; }

  void reset_stats() {
    feed_count = 0;
    reset_count = 0;
    time_since_feed = 0;
    was_wdt_reset = false;
  }

private:
  MockWatchdog() : wdt_timeout(5000), is_running(false), feed_count(0),
                   reset_count(0), time_since_feed(0), was_wdt_reset(false) {}

  uint32_t wdt_timeout;
  bool is_running;
  int feed_count;
  int reset_count;
  uint32_t time_since_feed;
  bool was_wdt_reset;
};

#endif // MOCK_HAL_H
