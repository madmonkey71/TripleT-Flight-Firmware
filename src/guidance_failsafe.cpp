#include "guidance_control.h"
#include "stability_monitor.h"
#include "servo_smoother.h"
#include "config.h"
#include "data_structures.h"
#include <cstring>

// External flight state (g_guidance_active is declared in guidance_control.h)
extern FlightState g_currentFlightState;

// Global instances (defined in guidance_control.cpp)
// These are declared extern so guidance_failsafe.cpp can access the stability monitor
extern StabilityMonitor g_stability_monitor;
extern ServoSmoother g_servo_smoother;

// Failsafe state tracking
static struct {
  bool failsafe_active;
  uint32_t failsafe_start_ms;
  float gain_reduction_factor;  // 1.0 = full gain, 0.3 = 30% (minimum)
  bool passive_mode_active;      // Servos centered, no control
  uint8_t escalation_level;      // 0=none, 1=gain reduced, 2=passive, 3=error
} failsafe_state = {
  false, 0, 1.0f, false, 0
};

/**
 * @brief Check stability and apply failsafe actions if needed
 *
 * Called from main loop at 10 Hz (or every guidance_update)
 * Implements 3-level escalation: gain reduction → passive mode → ERROR state
 *
 * @param current_time_ms Current system time
 * @return true if failsafe was triggered, false otherwise
 */
bool guidance_failsafe_check(uint32_t current_time_ms) {
#if ENABLE_GUIDANCE != 1
  return false;
#endif

  const StabilityMonitor::StabilityMetrics metrics = g_stability_monitor.getMetrics();

  // Check for persistent violation (exceeds duration threshold)
  if (metrics.is_stable == false &&
      g_stability_monitor.isViolationPersistent()) {

    if (!failsafe_state.failsafe_active) {
      // First time entering failsafe
      failsafe_state.failsafe_active = true;
      failsafe_state.failsafe_start_ms = current_time_ms;
      failsafe_state.gain_reduction_factor = 1.0f;
      failsafe_state.escalation_level = 0;

      Serial.print(F("[Failsafe] ACTIVATED - Violation type: "));
      if (metrics.violation_type) {
        Serial.println(metrics.violation_type);
      } else {
        Serial.println(F("UNKNOWN"));
      }
    }

    uint32_t failsafe_duration = current_time_ms - failsafe_state.failsafe_start_ms;

    // LEVEL 1: Reduce PID gains gradually
    if (failsafe_duration >= GUIDANCE_FAILSAFE_LEVEL1_MS &&
        failsafe_state.escalation_level == 0) {
      failsafe_state.escalation_level = 1;
      Serial.println(F("[Failsafe] LEVEL 1: Reducing PID gains"));
    }

    // Apply continuous gain reduction while in level 1
    if (failsafe_state.escalation_level == 1) {
      if (failsafe_state.gain_reduction_factor > GUIDANCE_FAILSAFE_MIN_GAIN) {
        // Reduce by 2% per 100ms (20% reduction per second)
        failsafe_state.gain_reduction_factor -= 0.02f;
        if (failsafe_state.gain_reduction_factor < GUIDANCE_FAILSAFE_MIN_GAIN) {
          failsafe_state.gain_reduction_factor = GUIDANCE_FAILSAFE_MIN_GAIN;
        }
      }
    }

    // LEVEL 2: Enter passive mode (center servos, disable control)
    if ((failsafe_duration >= GUIDANCE_FAILSAFE_LEVEL2_MS &&
        failsafe_state.escalation_level < 2) ||
        metrics.actuator_saturation_percent > 95.0f) {
      failsafe_state.escalation_level = 2;
      failsafe_state.passive_mode_active = true;
      guidance_center_servos();  // Center all servos
      Serial.println(F("[Failsafe] LEVEL 2: Entering passive mode (servos centered)"));
    }

    // LEVEL 3: Permanently disable guidance (do NOT transition to ERROR - reserved for hardware failures)
    if (failsafe_duration >= GUIDANCE_FAILSAFE_LEVEL3_MS &&
        failsafe_state.escalation_level < 3) {
      failsafe_state.escalation_level = 3;
      g_guidance_active = false;
      guidance_center_servos();
      Serial.println(F("[Failsafe] LEVEL 3: CRITICAL - Guidance permanently disabled for this flight"));
      Serial.println(F("[Failsafe] Flight continues without active guidance (apogee/parachute unaffected)"));
      return true;
    }

    return true;  // Failsafe active but not critical yet
  }

  // If stability restored and failsafe was active, recover gracefully
  if (failsafe_state.failsafe_active && metrics.is_stable == true) {
    // Recover PID gains at slower rate (5% per 100ms = 50% per second)
    if (failsafe_state.gain_reduction_factor < 1.0f) {
      failsafe_state.gain_reduction_factor += 0.05f;
      if (failsafe_state.gain_reduction_factor > 1.0f) {
        failsafe_state.gain_reduction_factor = 1.0f;
      }
    }

    // Only declare recovery complete when not in passive mode and gains restored
    if (failsafe_state.escalation_level < 2 && failsafe_state.gain_reduction_factor >= 1.0f) {
      failsafe_state.failsafe_active = false;
      failsafe_state.failsafe_start_ms = 0;
      failsafe_state.escalation_level = 0;
      Serial.println(F("[Failsafe] Stability recovered, resuming normal operation"));
    }
  }

  return false;
}

/**
 * @brief Reset failsafe state
 * Called when entering new flight state or manually resetting system
 */
void guidance_failsafe_reset() {
  failsafe_state.failsafe_active = false;
  failsafe_state.failsafe_start_ms = 0;
  failsafe_state.gain_reduction_factor = 1.0f;
  failsafe_state.passive_mode_active = false;
  failsafe_state.escalation_level = 0;

  Serial.println(F("[Failsafe] State reset"));
}

/**
 * @brief Get current failsafe gain reduction factor
 * Multiply PID gains by this factor in guidance_update()
 *
 * @return Factor from 0.0 to 1.0
 */
float guidance_failsafe_get_gain_factor() {
  return failsafe_state.gain_reduction_factor;
}

/**
 * @brief Check if currently in passive mode
 * @return true if servos are centered and control disabled
 */
bool guidance_failsafe_is_passive_mode() {
  return failsafe_state.passive_mode_active;
}

/**
 * @brief Check if failsafe is currently active
 * @return true if any failsafe mechanism is engaged
 */
bool guidance_failsafe_is_active() {
  return failsafe_state.failsafe_active;
}

/**
 * @brief Get current failsafe escalation level
 * @return 0=normal, 1=gain reduction, 2=passive mode, 3=error state
 */
uint8_t guidance_failsafe_get_level() {
  return failsafe_state.escalation_level;
}

/**
 * @brief Print failsafe status to serial
 */
void guidance_failsafe_print_status() {
  Serial.println(F("=== FAILSAFE STATUS ==="));
  Serial.print(F("Active: "));
  Serial.println(failsafe_state.failsafe_active ? F("YES") : F("NO"));

  Serial.print(F("Level: "));
  Serial.println(failsafe_state.escalation_level);

  Serial.print(F("Gain Factor: "));
  Serial.print(failsafe_state.gain_reduction_factor, 2);
  Serial.println(F("x"));

  Serial.print(F("Passive Mode: "));
  Serial.println(failsafe_state.passive_mode_active ? F("YES") : F("NO"));

  if (failsafe_state.failsafe_active && failsafe_state.failsafe_start_ms > 0) {
    Serial.print(F("Duration: "));
    Serial.print(millis() - failsafe_state.failsafe_start_ms);
    Serial.println(F(" ms"));
  }
  Serial.println(F("========================"));
}
