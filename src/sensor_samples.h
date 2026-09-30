#ifndef SENSOR_SAMPLES_H
#define SENSOR_SAMPLES_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Fresh-sample bookkeeping (audit #1, #4)
//
// The sensor drivers cache their latest reading in globals, while the flight
// state machine runs on every main-loop pass (kHz), far faster than the sensors
// update (10 Hz). Any "N consecutive readings" counter driven by the loop was
// therefore satisfied by ONE sample re-read N times within a millisecond.
//
// Each driver now stamps a SampleClock whenever it stores a genuinely new sample.
// Confirmation logic must only count when `seq` has advanced.
// ---------------------------------------------------------------------------
struct SampleClock {
  uint32_t seq;     // incremented once per fresh sample
  uint32_t lastMs;  // millis() when the last fresh sample was stored
};

extern SampleClock g_baroSample;   // ms5611_read()
extern SampleClock g_icmSample;    // ICM_20948_read()
extern SampleClock g_kx134Sample;  // kx134_read()
extern SampleClock g_gpsSample;    // gps_read()

// Record that a fresh sample has just been stored.
inline void sample_mark(SampleClock& c, uint32_t nowMs) {
  c.seq++;
  c.lastMs = nowMs;
}

#endif // SENSOR_SAMPLES_H
