#ifndef WATCHDOG_RECOVERY_H
#define WATCHDOG_RECOVERY_H

#include <Arduino.h>
#include "data_structures.h"

// ============================================================================
// WATCHDOG RECOVERY - System Recovery After Watchdog Reset
//
// When the watchdog timer triggers (system hangs/crash), the Teensy resets.
// This module detects the reset and recovers the flight state from EEPROM.
//
// Recovery procedure:
// 1. Detect if this is a watchdog reset vs normal startup
// 2. Read last known flight state from EEPROM
// 3. Re-initialize only essential sensors
// 4. Resume flight logic from last state (not from beginning)
// 5. Log recovery event for analysis
//
// This ensures flight continues even if firmware has bugs that cause hangs.
// ============================================================================

class WatchdogRecovery {
public:
  enum RecoveryReason {
    NORMAL_STARTUP,      // Normal boot, no reset
    WATCHDOG_RESET,      // Watchdog timer fired
    BROWNOUT_RESET,      // Low voltage reset
    UNKNOWN_RESET        // Unknown reset reason
  };

  struct RecoveryState {
    RecoveryReason reason;
    FlightState last_state;
    uint32_t last_state_time_ms;
    float last_altitude_m;
    bool sensors_healthy;
    const char* status_msg;
  };

  // ========================================================================
  // INITIALIZATION & DETECTION
  // ========================================================================

  static RecoveryReason detectResetReason() {
#ifdef ARDUINO
    // Teensy 4.1 (NXP i.MX RT1062) System Reset Controller Status Register
    volatile uint32_t* SRC_SRSR = (volatile uint32_t*)0x400F8008;
    uint32_t resetFlags = *SRC_SRSR;

    RecoveryReason reason = NORMAL_STARTUP;

    // Bit 3: WDOG1 reset, Bit 5: WDOG3 reset (Teensy WDT)
    if (resetFlags & ((1 << 3) | (1 << 5))) {
      reason = WATCHDOG_RESET;
    }
    // Bit 1: brownout reset
    else if (resetFlags & (1 << 1)) {
      reason = BROWNOUT_RESET;
    }
    // Bit 0: power-on reset
    else if (resetFlags & (1 << 0)) {
      reason = NORMAL_STARTUP;
    }
    // Bit 16: software reset
    else if (resetFlags & (1 << 16)) {
      reason = UNKNOWN_RESET;
    }

    // Clear status register by writing 1s to detected bits
    *SRC_SRSR = resetFlags;

    return reason;
#else
    // Desktop/test builds
    return NORMAL_STARTUP;
#endif
  }

  static bool isWatchdogReset() {
    return detectResetReason() == WATCHDOG_RESET;
  }

  // ========================================================================
  // STATE RECOVERY FROM EEPROM
  // ========================================================================

  static RecoveryState recoverState(uint16_t eeprom_state_addr = 0) {
    RecoveryState recovery = {};
    recovery.reason = detectResetReason();

    if (recovery.reason == NORMAL_STARTUP) {
      recovery.status_msg = "Normal startup";
      recovery.sensors_healthy = true;
      return recovery;
    }

    // ====== Watchdog Reset Detected ======

    recovery.status_msg = "Recovering from watchdog reset";

    // Read last flight state from EEPROM
    // TODO: Implement actual EEPROM read:
    // uint8_t state_byte = EEPROM.read(eeprom_state_addr);
    // recovery.last_state = (FlightState)state_byte;
    //
    // uint32_t time_bytes = read_uint32(eeprom_state_addr + 1);
    // recovery.last_state_time_ms = time_bytes;

    // Temporary placeholder
    recovery.last_state = PAD_IDLE;
    recovery.last_state_time_ms = 0;
    recovery.sensors_healthy = true;

    return recovery;
  }

  // ========================================================================
  // SENSOR RE-INITIALIZATION
  // ========================================================================

  static bool reinitializeSensors(IMUInterface* imu, bool full_init = false) {
    // If full_init = false: Quick init only critical sensors (recovery mode)
    // If full_init = true: Full initialization (normal startup)

    if (!imu) {
      return false;
    }

    // Quick sensor check - don't recalibrate, just verify communication
    bool sensor_ok = imu->begin();

    if (sensor_ok) {
      // Verify sensor is responding
      sensor_ok = imu->isHealthy();
    }

    if (!sensor_ok) {
      // Sensor initialization failed - log and continue anyway
      // Flight can continue in degraded mode
      return false;
    }

    return true;
  }

  // ========================================================================
  // RECOVERY LOGGING
  // ========================================================================

  static void logRecoveryEvent(const char* recovery_msg, uint32_t recovery_time_ms) {
    // Log to SD card or serial for analysis
    // Format: [RECOVERY] timestamp=T, reason=W, state=BOOST, msg="recovery message"

    // TODO: Implement actual logging:
    // File log;
    // if (log.open("recovery.log", FILE_WRITE)) {
    //   log.print("[RECOVERY] timestamp=");
    //   log.print(recovery_time_ms);
    //   log.print(", reason=");
    //   log.print(recovery_msg);
    //   log.println();
    //   log.close();
    // }

    // For now, print to serial
    Serial.print("[RECOVERY] ");
    Serial.print(recovery_msg);
    Serial.print(" at time ");
    Serial.println(recovery_time_ms);
  }

  // ========================================================================
  // GRACE PERIOD MECHANISM
  // ========================================================================

  // Prevent error state oscillation: Once in error, stay there for grace period
  static bool isInGracePeriod(uint32_t error_entered_time_ms, uint32_t grace_period_ms = 5000) {
    uint32_t time_in_error = millis() - error_entered_time_ms;
    return time_in_error < grace_period_ms;
  }

  // ========================================================================
  // RECOVERY STRATEGY BY STATE
  // ========================================================================

  static bool canRecoverFromState(FlightState state) {
    // Some states are recoverable after watchdog, some are not
    switch (state) {
      // Before launch - safe to recover
      case STARTUP:
      case CALIBRATION:
      case PAD_IDLE:
      case ARMED:
        return true;

      // During flight - risky to resume mid-flight, but possible
      case BOOST:
      case COAST:
      case APOGEE:
      case DROGUE_DEPLOY:
      case DROGUE_DESCENT:
      case MAIN_DEPLOY:
      case MAIN_DESCENT:
      case LANDED:
      case RECOVERY:
        // These states are recoverable but require sensor verification
        return true;

      // Error state - recovery already active
      case ERROR:
        return false;

      default:
        return false;
    }
  }

  static const char* getRecoveryStrategy(FlightState state) {
    switch (state) {
      case BOOST:
        return "Mid-boost recovery: Verify altitude/velocity, may miss apogee";
      case COAST:
        return "Coast phase recovery: Resume with current sensor readings";
      case APOGEE:
        return "Recovery at apogee: Proceed with drogue deployment";
      case DROGUE_DESCENT:
        return "Under drogue: Resume descent monitoring";
      case MAIN_DESCENT:
        return "Under main: Resume landing monitoring";
      default:
        return "Safe recovery: No flight-critical actions pending";
    }
  }

  // ========================================================================
  // DIAGNOSTIC INFORMATION
  // ========================================================================

  struct DiagnosticInfo {
    RecoveryReason last_reset_reason;
    FlightState recovered_state;
    uint32_t recovery_count;
    uint32_t last_recovery_time_ms;
  };

  static DiagnosticInfo getDiagnostics() {
    // TODO: Track actual recovery count in EEPROM
    return {
      detectResetReason(),
      PAD_IDLE,  // Placeholder
      0,         // Placeholder
      0          // Placeholder
    };
  }
};

#endif // WATCHDOG_RECOVERY_H
