#ifndef HAL_FACTORY_H
#define HAL_FACTORY_H

#include "hal_interfaces.h"

#ifdef NATIVE_TEST_BUILD
  // For desktop testing - use Mock implementations
  #include "mock_hal.h"

  class HALFactory {
  public:
    static ITimer* createTimer() {
      return &MockTimer::instance();
    }

    static ISerial* createSerial() {
      return &MockSerial::instance();
    }

    static IGPIO* createGPIO() {
      return &MockGPIO::instance();
    }

    static II2C* createI2C() {
      return &MockI2C::instance();
    }

    static IEEPROM* createEEPROM() {
      return &MockEEPROM::instance();
    }

    static ISDCard* createSDCard() {
      return &MockSDCard::instance();
    }

    static IServo* createServo() {
      return &MockServo::instance();
    }

    static IWatchdog* createWatchdog() {
      return &MockWatchdog::instance();
    }
  };

#else
  // For production Teensy hardware - use real implementations
  #include "teensy_hal.h"

  class HALFactory {
  public:
    static ITimer* createTimer() {
      return &TeensyTimer::instance();
    }

    static ISerial* createSerial() {
      return &TeensySerial::instance();
    }

    static IGPIO* createGPIO() {
      return &TeensyGPIO::instance();
    }

    static II2C* createI2C() {
      return &TeensyI2C::instance();
    }

    static IEEPROM* createEEPROM() {
      return &TeensyEEPROM::instance();
    }

    static ISDCard* createSDCard() {
      return &TeensySDCard::instance();
    }

    static IServo* createServo() {
      return &TeensyServo::instance();
    }

    static IWatchdog* createWatchdog() {
      return &TeensyWatchdog::instance();
    }
  };

#endif

// Global HAL instances (easier than factory calls everywhere)
extern ITimer*    g_timer;
extern ISerial*   g_serial;
extern IGPIO*     g_gpio;
extern II2C*      g_i2c;
extern IEEPROM*   g_eeprom;
extern ISDCard*   g_sdcard;
extern IServo*    g_servo;
extern IWatchdog* g_watchdog;

#endif // HAL_FACTORY_H
