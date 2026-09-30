#include "pyro_control.h"
#include <Arduino.h>
#include "config.h"
#include "state_management.h" // g_pyroFiredMask, PYRO_FIRED_*, saveStateToEEPROM()
#include "debug_flags.h"

extern DebugFlags g_debugFlags;

static bool s_firing[2] = {false, false};
static unsigned long s_startMs[2] = {0, 0};

static uint8_t pinFor(PyroChannel ch) { return ch == PYRO_CH_DROGUE ? PYRO_CHANNEL_1 : PYRO_CHANNEL_2; }
static uint8_t firedBit(PyroChannel ch) { return ch == PYRO_CH_DROGUE ? PYRO_FIRED_DROGUE : PYRO_FIRED_MAIN; }
static const char* nameFor(PyroChannel ch) { return ch == PYRO_CH_DROGUE ? "1 (Drogue)" : "2 (Main)"; }

void pyro_init_safe() {
  // Latch LOW first, then enable the output driver, then assert LOW again, so the
  // pin can never present a transient HIGH while it changes from input to output.
  digitalWrite(PYRO_CHANNEL_1, LOW);
  digitalWrite(PYRO_CHANNEL_2, LOW);
  pinMode(PYRO_CHANNEL_1, OUTPUT);
  pinMode(PYRO_CHANNEL_2, OUTPUT);
  digitalWrite(PYRO_CHANNEL_1, LOW);
  digitalWrite(PYRO_CHANNEL_2, LOW);
  s_firing[0] = s_firing[1] = false;
  s_startMs[0] = s_startMs[1] = 0;
}

bool pyro_request_fire(PyroChannel ch) {
  if (g_pyroFiredMask & firedBit(ch)) return false;  // already completed: never re-fire
  if (s_firing[ch]) return true;                     // window already open
  if (g_debugFlags.enableSystemDebug) {
    Serial.print(F("Firing Pyro Channel "));
    Serial.println(nameFor(ch));
  }
  s_firing[ch] = true;
  s_startMs[ch] = millis();
  digitalWrite(pinFor(ch), HIGH);
  return true;
}

void pyro_service() {
  for (int i = 0; i < 2; i++) {
    const PyroChannel ch = static_cast<PyroChannel>(i);
    if (s_firing[i]) {
      if (millis() - s_startMs[i] >= PYRO_FIRE_DURATION) {
        digitalWrite(pinFor(ch), LOW);
        s_firing[i] = false;
        // Record completion BEFORE anything else can reset us, so a reset during
        // descent does not fire this channel again.
        g_pyroFiredMask |= firedBit(ch);
        saveStateToEEPROM();
        if (g_debugFlags.enableSystemDebug) {
          Serial.print(F("Pyro Channel "));
          Serial.print(nameFor(ch));
          Serial.println(F(" Fired."));
        }
      } else {
        digitalWrite(pinFor(ch), HIGH); // keep asserting for the whole window
      }
    } else {
      digitalWrite(pinFor(ch), LOW);    // an idle channel is always driven low
    }
  }
}

bool pyro_is_firing(PyroChannel ch) { return s_firing[ch]; }
bool pyro_fire_complete(PyroChannel ch) { return (g_pyroFiredMask & firedBit(ch)) != 0; }
