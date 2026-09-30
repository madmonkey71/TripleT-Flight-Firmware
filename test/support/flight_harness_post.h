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
  g_sdCardAvailable = true;
  g_loggingEnabled = true;
  g_baroSample = {0, 0}; g_icmSample = {0, 0}; g_kx134Sample = {0, 0}; g_gpsSample = {0, 0};
  GPS_fixType = 0;
  pressure = 1013.25f;
  flightLogicReset();
  stateManagementResetRuntime();
  handleInitialStateManagementReset();
  s_resetToken = 0;            // flight_commands.cpp: no reset_flight token outstanding
  s_resetTokenIssuedMs = 0;
}

// --- Sensor injection -------------------------------------------------------
// Set the barometric altitude the firmware will compute (absolute, after offset).
inline void harness_set_baro_alt(float absolute_alt_m) { g_test_raw_alt = absolute_alt_m - baro_altitude_offset; }
// Set the specific-force magnitude (in g) seen by BOTH accelerometers (on the Z axis).
inline void harness_set_accel_g(float mag_g) {
  icm_accel[0] = 0; icm_accel[1] = 0; icm_accel[2] = mag_g;
  kx134_accel[0] = 0; kx134_accel[1] = 0; kx134_accel[2] = mag_g;
}

// One pass of the firmware main loop's flight-logic calls (same order as loop()).
inline void harness_pass() {
  handleInitialStateManagement();
  ProcessFlightState();
}

// Deliver one fresh sample from a sensor (what its driver does when it stores new data).
inline void harness_new_baro_sample() { sample_mark(g_baroSample, millis()); }
inline void harness_new_accel_samples() { sample_mark(g_icmSample, millis()); sample_mark(g_kx134Sample, millis()); }
inline void harness_new_gps_sample() { sample_mark(g_gpsSample, millis()); }
// All sensors at once (10 Hz cadence). GPS only delivers with a 3D fix.
inline void harness_new_samples() {
  harness_new_baro_sample();
  harness_new_accel_samples();
  if (GPS_fixType >= 3) harness_new_gps_sample();
}
inline float harness_baro_alt() { return g_test_raw_alt + baro_altitude_offset; }

// Advance `ms` of simulated time in 10 ms loop passes. Like the firmware, the
// sensors deliver a FRESH sample every 100 ms (10 Hz) while the loop runs 10x
// faster than that in this model (far faster still on hardware): values cached
// between samples are re-read on every pass.
inline void harness_run_ms(unsigned long ms) {
  static unsigned long phase = 0;
  for (unsigned long t = 0; t < ms; t += 10) {
    test_advance_ms(10);
    if ((++phase % 10) == 0) harness_new_samples();
    harness_pass();
  }
}

// Move the barometric altitude linearly to `to_alt` over `ms`, delivering fresh
// samples every 100 ms, running the flight logic on every 10 ms pass.
inline void harness_ramp_baro(float to_alt, unsigned long ms) {
  const float from = harness_baro_alt();
  const unsigned long steps = ms / 10;
  for (unsigned long i = 1; i <= steps; i++) {
    harness_set_baro_alt(from + (to_alt - from) * (float)i / (float)steps);
    harness_run_ms(10);
  }
}

// Put the vehicle on the pad, calibrated, at `launch_alt` metres, in PAD_IDLE.
inline void harness_on_pad(float launch_alt = 100.0f) {
  g_baroCalibrated = true;
  baro_calibration_done = true;
  harness_set_baro_alt(launch_alt);
  harness_set_accel_g(1.0f);
  harness_new_samples();
  g_currentFlightState = PAD_IDLE;
  harness_pass();          // state-entry actions (launch altitude capture, pins LOW)
}
