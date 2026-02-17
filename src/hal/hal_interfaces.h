#ifndef HAL_INTERFACES_H
#define HAL_INTERFACES_H

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

// ============================================================================
// ITIMER - Replace millis(), micros(), delay()
// ============================================================================
class ITimer {
public:
  virtual ~ITimer() = default;

  // Get elapsed milliseconds since system start
  virtual uint32_t millis() = 0;

  // Get elapsed microseconds since system start
  virtual uint32_t micros() = 0;

  // Blocking delay in milliseconds
  virtual void delay(uint32_t ms) = 0;

  // Blocking delay in microseconds
  virtual void delayMicroseconds(uint16_t us) = 0;
};

// ============================================================================
// ISERIAL - Replace Serial.print, Serial.read, Serial.available
// ============================================================================
class ISerial {
public:
  virtual ~ISerial() = default;

  // Print null-terminated string (no newline)
  virtual void print(const char* str) = 0;

  // Print with newline
  virtual void println(const char* str) = 0;

  // Print a single byte
  virtual void write(uint8_t byte) = 0;

  // Read one byte from input buffer (-1 if none available)
  virtual int read() = 0;

  // Check if data available in input buffer
  virtual bool available() = 0;

  // Peek at next byte without consuming it (-1 if none available)
  virtual int peek() = 0;

  // Clear output buffer (flush)
  virtual void flush() = 0;
};

// ============================================================================
// IGPIO - Replace digitalWrite, digitalRead, pinMode
// ============================================================================
class IGPIO {
public:
  virtual ~IGPIO() = default;

  // Set pin mode (INPUT, OUTPUT, INPUT_PULLUP, etc)
  virtual void pinMode(uint8_t pin, uint8_t mode) = 0;

  // Write digital value to pin (HIGH or LOW)
  virtual void digitalWrite(uint8_t pin, uint8_t value) = 0;

  // Read digital value from pin (returns HIGH or LOW)
  virtual int digitalRead(uint8_t pin) = 0;

  // Analog write (PWM) - value 0-255
  virtual void analogWrite(uint8_t pin, uint8_t value) = 0;

  // Analog read - value 0-1023 (or 0-4095 depending on resolution)
  virtual int analogRead(uint8_t pin) = 0;
};

// ============================================================================
// II2C - Replace Wire.begin, Wire.write, Wire.read, Wire.beginTransmission
// ============================================================================
class II2C {
public:
  virtual ~II2C() = default;

  // Initialize I2C bus (SDA, SCL pins - 0 means default)
  virtual bool begin(uint8_t sda_pin = 0, uint8_t scl_pin = 0) = 0;

  // Start transmission to slave address
  virtual void beginTransmission(uint8_t address) = 0;

  // Write bytes to I2C bus (during transmission)
  virtual size_t write(const uint8_t* data, size_t length) = 0;

  // Write single byte
  virtual size_t write(uint8_t byte) = 0;

  // End transmission and return status (0 = success)
  virtual uint8_t endTransmission(bool sendStop = true) = 0;

  // Request bytes from slave address, returns number received
  virtual size_t requestFrom(uint8_t address, size_t quantity, bool sendStop = true) = 0;

  // Read single byte from received data
  virtual int read() = 0;

  // Peek at next byte without consuming
  virtual int peek() = 0;

  // Check if data available
  virtual bool available() = 0;
};

// ============================================================================
// IEEPROM - Replace EEPROM.read/write for persistent storage
// ============================================================================
class IEEPROM {
public:
  virtual ~IEEPROM() = default;

  // Initialize EEPROM (some platforms need this)
  virtual bool begin() = 0;

  // Read single byte from address
  virtual uint8_t read(uint16_t address) = 0;

  // Write single byte to address
  virtual void write(uint16_t address, uint8_t value) = 0;

  // Write multiple bytes
  virtual void write(uint16_t address, const uint8_t* data, size_t length) = 0;

  // Commit writes to persistent storage (some platforms batch writes)
  virtual void commit() = 0;

  // Get total EEPROM size in bytes
  virtual size_t length() = 0;
};

// ============================================================================
// ISDCARD - Replace SdFat operations for data logging
// ============================================================================
class ISDCard {
public:
  virtual ~ISDCard() = default;

  // Initialize SD card (chip select pin)
  virtual bool begin(uint8_t cs_pin) = 0;

  // Open/create a file for writing
  virtual bool openFile(const char* filename) = 0;

  // Write data to current file
  virtual bool writeData(const uint8_t* data, size_t length) = 0;

  // Close current file (flush to disk)
  virtual bool closeFile() = 0;

  // Check if file operation was successful
  virtual bool isHealthy() = 0;

  // Get last error code
  virtual uint32_t getLastError() = 0;
};

// ============================================================================
// ISERVO - Replace analogWrite for PWM servo control
// ============================================================================
class IServo {
public:
  virtual ~IServo() = default;

  // Attach servo to pin and set pulse range (min/max microseconds)
  virtual bool attach(uint8_t pin, uint16_t min_us = 1000, uint16_t max_us = 2000) = 0;

  // Detach servo from pin
  virtual void detach(uint8_t pin) = 0;

  // Write angle 0-180 degrees
  virtual void writeAngle(uint8_t pin, uint8_t angle) = 0;

  // Write raw microsecond pulse width
  virtual void writeMicroseconds(uint8_t pin, uint16_t us) = 0;

  // Read current angle
  virtual uint8_t readAngle(uint8_t pin) = 0;

  // Read current microsecond value
  virtual uint16_t readMicroseconds(uint8_t pin) = 0;
};

// ============================================================================
// IWATCHDOG - Replace WDT feeding and reset
// ============================================================================
class IWatchdog {
public:
  virtual ~IWatchdog() = default;

  // Initialize watchdog with timeout in milliseconds
  virtual bool begin(uint32_t timeout_ms) = 0;

  // Feed the watchdog (pet it, prevent reset)
  virtual void feed() = 0;

  // Force a watchdog reset
  virtual void reset() = 0;

  // Get remaining time before reset (milliseconds)
  virtual uint32_t getTimeout() = 0;

  // Check if this was a watchdog reset (true = recovering from WDT)
  virtual bool isWatchdogReset() = 0;
};

#endif // HAL_INTERFACES_H
