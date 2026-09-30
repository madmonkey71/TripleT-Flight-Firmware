// Second half of the flight-logic test harness: helpers that need the REAL
// source files to have been included first (see flight_harness.h).
#pragma once

// Reset every global and every piece of real-code runtime state to power-on
// defaults, with the fake clock parked well away from 0 so `millis() - t`
// arithmetic in the firmware behaves as it does after minutes of uptime.
inline void harness_reset() {
  test_set_ms(100000);
  test_pins_reset();
  Serial.clear();
  EEPROM.wipe();
  g_currentFlightState = STARTUP;
  g_previousFlightState = STARTUP;
  g_stateEntryTime = 0;
  g_launchAltitude = 0.0f;
  g_maxAltitudeReached = 0.0f;
  g_currentAltitude = 0.0f;
  g_baroCalibrated = false;
  g_kx134_initialized_ok = true;
  g_icm20948_ready = true;
  g_useKalmanFilter = true;
  g_main_deploy_altitude_m_agl = 0.0f;
  g_last_error_code = NO_ERROR;
  g_guidance_active = true;
  g_debugFlags = {};
  ms5611_initialized_ok = true;
  baro_altitude_offset = 0.0f;
  baro_calibration_done = false;
  for (int i = 0; i < 3; i++) { kx134_accel[i] = 0; icm_accel[i] = 0; icm_gyro[i] = 0; }
  isStationary = false;
  GPS_fixType = 0; pDOP = 999; GPS_altitude = 0; GPS_altitudeMSL = 0;
  g_test_sensors_healthy = true;
  g_test_raw_alt = 0.0f;
  g_test_gps_alt_m = 0.0f;
  g_test_log_writes = 0;
  g_test_guidance_center_calls = 0;
  g_test_stability_compromised = false;
  g_pixels = Adafruit_NeoPixel(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
  flightLogicReset();
}

// --- Sensor injection -------------------------------------------------------
// Set the barometric altitude the firmware will compute (absolute, after offset).
inline void harness_set_baro_alt(float absolute_alt_m) { g_test_raw_alt = absolute_alt_m - baro_altitude_offset; }
// Set the specific-force magnitude (in g) seen by BOTH accelerometers (on the Z axis).
inline void harness_set_accel_g(float mag_g) {
  icm_accel[0] = 0; icm_accel[1] = 0; icm_accel[2] = mag_g;
  kx134_accel[0] = 0; kx134_accel[1] = 0; kx134_accel[2] = mag_g;
}

// One pass of the firmware main loop's flight-logic call.
inline void harness_pass() { ProcessFlightState(); }

// Advance `ms` of simulated time in 10 ms loop passes.
inline void harness_run_ms(unsigned long ms) {
  for (unsigned long t = 0; t < ms; t += 10) { test_advance_ms(10); harness_pass(); }
}

// Put the vehicle on the pad, calibrated, at `launch_alt` metres, in PAD_IDLE.
inline void harness_on_pad(float launch_alt = 100.0f) {
  g_baroCalibrated = true;
  baro_calibration_done = true;
  harness_set_baro_alt(launch_alt);
  harness_set_accel_g(1.0f);
  g_currentFlightState = PAD_IDLE;
  harness_pass();          // state-entry actions (launch altitude capture, pins LOW)
}
