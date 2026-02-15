#include "hal_factory.h"

// ============================================================================
// GLOBAL HAL INSTANCES
// These are initialized by hal_init() which should be called in main setup()
// ============================================================================

ITimer*    g_timer    = nullptr;
ISerial*   g_serial   = nullptr;
IGPIO*     g_gpio     = nullptr;
II2C*      g_i2c      = nullptr;
IEEPROM*   g_eeprom   = nullptr;
ISDCard*   g_sdcard   = nullptr;
IServo*    g_servo    = nullptr;
IWatchdog* g_watchdog = nullptr;

// ============================================================================
// HAL INITIALIZATION FUNCTION
// Call this from main setup() to initialize all HAL instances
// ============================================================================
void hal_init() {
  g_timer    = HALFactory::createTimer();
  g_serial   = HALFactory::createSerial();
  g_gpio     = HALFactory::createGPIO();
  g_i2c      = HALFactory::createI2C();
  g_eeprom   = HALFactory::createEEPROM();
  g_sdcard   = HALFactory::createSDCard();
  g_servo    = HALFactory::createServo();
  g_watchdog = HALFactory::createWatchdog();
}

// ============================================================================
// HELPER MACROS FOR CONVENIENCE
// Can use in code instead of factory calls
// ============================================================================
// #define TIMER     (*g_timer)
// #define SERIAL    (*g_serial)
// #define GPIO      (*g_gpio)
// #define I2C       (*g_i2c)
// #define EEPROM    (*g_eeprom)
// #define SDCARD    (*g_sdcard)
// #define SERVO     (*g_servo)
// #define WATCHDOG  (*g_watchdog)
