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

// ---------------------------------------------------------------------------
// FreshCounter: "N consecutive samples satisfy a condition" that only advances
// when the sensor has produced a NEW sample (its `seq` moved). Calls made between
// samples neither increment nor reset the count, so the confirmation time is
// N sensor periods, not N main-loop passes.
// ---------------------------------------------------------------------------
struct FreshCounter {
  uint32_t lastSeq = 0;
  bool primed = false;
  int count = 0;

  void reset() { lastSeq = 0; primed = false; count = 0; }

  // Returns true if `seq` was a new sample (and was therefore evaluated).
  bool feed(uint32_t seq, bool condition) {
    if (primed && seq == lastSeq) return false;
    primed = true;
    lastSeq = seq;
    count = condition ? count + 1 : 0;
    return true;
  }
};

// ---------------------------------------------------------------------------
// BaroTrack: ring buffer of the most recent FRESH barometric altitude samples
// (absolute altitude, metres, with timestamps). Used for vertical-rate and
// stationarity estimates and for the pad launch-altitude average.
// ---------------------------------------------------------------------------
struct BaroTrack {
  static const int N = 20;
  float alt[N];
  unsigned long ms[N];
  int head = 0;      // next write index
  int n = 0;         // valid samples (<= N)
  uint32_t lastSeq = 0;
  bool primed = false;

  BaroTrack() { reset(); }
  void reset() { head = 0; n = 0; lastSeq = 0; primed = false; for (int i = 0; i < N; i++) { alt[i] = 0; ms[i] = 0; } }

  // Store a sample if `seq` is new. Returns true if stored.
  bool push(uint32_t seq, unsigned long nowMs, float altAbs) {
    if (primed && seq == lastSeq) return false;
    primed = true;
    lastSeq = seq;
    alt[head] = altAbs;
    ms[head] = nowMs;
    head = (head + 1) % N;
    if (n < N) n++;
    return true;
  }
  int size() const { return n; }
  // i = 0 is the newest sample, i = size()-1 the oldest.
  float at(int i) const { return alt[(head - 1 - i + 2 * N) % N]; }
  unsigned long msAt(int i) const { return ms[(head - 1 - i + 2 * N) % N]; }

  // Vertical speed (m/s, +up) over the newest `window` samples; false if fewer than
  // `minSamples` samples or the span is under `minSpanMs`.
  bool verticalSpeed(int window, int minSamples, unsigned long minSpanMs, float& vs) const {
    int m = window < n ? window : n;
    if (m < minSamples || m < 2) return false;
    unsigned long span = msAt(0) - msAt(m - 1);
    if (span < minSpanMs || span == 0) return false;
    vs = (at(0) - at(m - 1)) / (span / 1000.0f);
    return true;
  }
  // max - min over the newest `m` samples (false if fewer than m stored).
  bool range(int m, float& r) const {
    if (m < 1 || n < m) return false;
    float lo = at(0), hi = at(0);
    for (int i = 1; i < m; i++) { float v = at(i); if (v < lo) lo = v; if (v > hi) hi = v; }
    r = hi - lo;
    return true;
  }
  // Mean of the newest `m` samples (false if none stored).
  bool mean(int m, float& out) const {
    if (n == 0) return false;
    if (m > n) m = n;
    float sum = 0;
    for (int i = 0; i < m; i++) sum += at(i);
    out = sum / m;
    return true;
  }
};

#endif // SENSOR_SAMPLES_H
