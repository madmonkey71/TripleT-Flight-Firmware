// Shared harness for unit tests that compile the REAL flight sources
// (src/flight_logic.cpp, src/state_management.cpp, ...) natively.
//
// Include order in a suite:
//     #include <unity.h>
//     #include "../support/flight_harness.h"        // stubs + firmware globals + helpers
//     #include "../../src/state_management.cpp"      // real code under test
//     #include "../../src/flight_logic.cpp"          // real code under test
//     #include "../support/flight_harness_post.h"    // helpers that need the real code
//
// What is faked (hardware/IO boundary only, never flight decisions):
//   * time (millis), pins, Serial, EEPROM        -> test/stubs/*.h
//   * sensor values                              -> plain globals set by the helpers below
//   * sensor-suite health (isSensorSuiteHealthy) -> g_test_sensors_healthy
//   * barometer altitude conversion              -> g_test_raw_alt + baro_altitude_offset
//   * guidance, logging, GPS accessors           -> recording stubs
// Everything else (state machine, detectors, recovery, persistence) is the shipped code.
#pragma once

#include "Arduino.h"
#include "Adafruit_NeoPixel.h"
#include "MS5611.h"
#include "../../src/config.h"
#include "../../src/constants.h"
#include "../../src/data_structures.h"
#include "../../src/debug_flags.h"
#include "../../src/error_codes.h"

// ---- Firmware globals normally defined in TripleT_Flight_Firmware.cpp ---------
FlightState g_currentFlightState = STARTUP;
FlightState g_previousFlightState = STARTUP;
unsigned long g_stateEntryTime = 0;
Adafruit_NeoPixel g_pixels(NEOPIXEL_COUNT, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
float g_launchAltitude = 0.0f;
float g_maxAltitudeReached = 0.0f;
float g_currentAltitude = 0.0f;
bool g_baroCalibrated = false;
MS5611 g_ms5611Sensor;
bool g_kx134_initialized_ok = false;
bool g_icm20948_ready = false;
bool g_useKalmanFilter = true;
bool g_usingKX134ForKalman = false;
DebugFlags g_debugFlags = {};
float g_main_deploy_altitude_m_agl = 0.0f;
ErrorCode_t g_last_error_code = NO_ERROR;
float g_kalmanRoll = 0, g_kalmanPitch = 0, g_kalmanYaw = 0;
float g_kalmanRollRate = 0, g_kalmanPitchRate = 0, g_kalmanYawRate = 0;
bool g_guidance_active = true;
float g_battery_voltage = 0.0f;

// ---- Sensor-driver globals (ms5611_/icm_/kx134_/gps_functions.cpp) -----------
bool ms5611_initialized_ok = true;
float pressure = 1013.25f;
float temperature = 15.0f;
float baro_altitude_offset = 0.0f;
bool baro_calibration_done = false;
float kx134_accel[3] = {0, 0, 0};
float icm_accel[3] = {0, 0, 0};
float icm_gyro[3] = {0, 0, 0};
float icm_q0 = 1, icm_q1 = 0, icm_q2 = 0, icm_q3 = 0;
bool isStationary = false;
uint8_t GPS_fixType = 0;
int pDOP = 999;
long GPS_altitude = 0;
long GPS_altitudeMSL = 0;
long GPS_latitude = 0;
long GPS_longitude = 0;
uint8_t SIV = 0;

// ---- Test controls ------------------------------------------------------------
bool g_test_sensors_healthy = true;   // what isSensorSuiteHealthy() reports
float g_test_raw_alt = 0.0f;          // barometric altitude before calibration offset
float g_test_gps_alt_m = 0.0f;        // what getGPSAltitude() reports
int g_test_log_writes = 0;            // WriteLogData() call count
int g_test_guidance_center_calls = 0; // guidance_center_servos() call count
int g_test_guidance_target_sets = 0;
bool g_test_stability_compromised = false;

// ---- Stubs for functions the real sources call across module boundaries -----
#include "../../src/utility_functions.h"   // declares WriteLogData, getStateName, isSensorSuiteHealthy ... (real header)
#include "../../src/guidance_control.h"    // real header
#include "../../src/gps_functions.h"
#include "../../src/ms5611_functions.h"
#include "../../src/icm_20948_functions.h"
#include "../../src/kx134_functions.h"

void WriteLogData(bool) { g_test_log_writes++; }
float ms5611_get_altitude(float) { return g_test_raw_alt + baro_altitude_offset; }
int ms5611_read() { return MS5611_READ_OK; }
uint8_t getFixType() { return GPS_fixType; }
float getGPSAltitude() { return g_test_gps_alt_m; }
bool isSensorSuiteHealthy(FlightState, bool) { return g_test_sensors_healthy; }
void convertQuaternionToEuler(float, float, float, float, float& r, float& p, float& y) { r = p = y = 0; }
const char* getStateName(FlightState state) {
  static const char* names[] = {"STARTUP", "CALIBRATION", "PAD_IDLE", "ARMED", "BOOST", "COAST", "APOGEE",
                                "DROGUE_DEPLOY", "DROGUE_DESCENT", "MAIN_DEPLOY", "MAIN_DESCENT", "LANDED",
                                "RECOVERY", "ERROR"};
  return (unsigned)state <= (unsigned)ERROR ? names[state] : "UNKNOWN";
}
const char* getErrorCodeName(ErrorCode_t) { return "ERR"; }
// Same selection rule as src/utility_functions.cpp: prefer the KX134 when it is
// initialised and non-zero, else the ICM-20948, else 0.
float get_accel_magnitude(bool kx_ok, const float* kx, bool icm_ok, const float* icm, bool) {
  auto mag = [](const float* v) { return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); };
  if (kx_ok && kx && (kx[0] != 0 || kx[1] != 0 || kx[2] != 0)) return mag(kx);
  if (icm_ok && icm && (icm[0] != 0 || icm[1] != 0 || icm[2] != 0)) return mag(icm);
  return 0.0f;
}

// guidance API recorded/neutral stubs
void guidance_reset_stability_status() {}
void guidance_get_target_euler_angles(float& r, float& p, float& y) { r = p = y = 0; }
void guidance_get_actuator_outputs(float& a, float& b, float& c) { a = b = c = 0; }
void guidance_check_stability(float, float, float, float, float, float, float, float, float, unsigned long) {}
bool guidance_is_stability_compromised() { return g_test_stability_compromised; }
void guidance_log_stability_diagnostics() {}
void guidance_center_servos() { g_test_guidance_center_calls++; }
void guidance_set_target_orientation_euler(float, float, float) { g_test_guidance_target_sets++; }
