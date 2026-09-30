#ifndef PYRO_CONTROL_H
#define PYRO_CONTROL_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Pyrotechnic output ownership (audit #1, #10)
//
// Everything that drives a pyro pin goes through this module. The flight state
// machine only *requests* a fire; pyro_service() - called on EVERY main-loop
// pass, independent of the flight state - owns the pins:
//   * a requested channel is driven HIGH for exactly PYRO_FIRE_DURATION and then
//     LOW, no matter which state the vehicle is in by then (the old inline code
//     lived inside the DROGUE_DEPLOY / MAIN_DEPLOY cases, so a state change during
//     the window left the pin HIGH);
//   * completion is recorded in the persisted fired mask (state_management.h);
//   * a channel that is not firing is actively driven LOW every pass;
//   * a channel whose fired bit is set can never be requested again.
// There are no function-local "has fired" flags to get stuck between flights.
// ---------------------------------------------------------------------------

enum PyroChannel {
  PYRO_CH_DROGUE = 0,  // PYRO_CHANNEL_1
  PYRO_CH_MAIN = 1     // PYRO_CHANNEL_2
};

// Force both pyro pins to OUTPUT / LOW and abort any fire window in progress.
// The very first thing setup() runs (audit #1); also used on PAD_IDLE entry.
void pyro_init_safe();

// Ask for a channel to be fired. Returns true if the channel is firing now (or
// already was); false if it already completed a fire this flight (never re-fires).
bool pyro_request_fire(PyroChannel ch);

// Call on every main-loop pass.
void pyro_service();

bool pyro_is_firing(PyroChannel ch);
// True once this channel's fire window has completed (persisted across resets).
bool pyro_fire_complete(PyroChannel ch);

#endif // PYRO_CONTROL_H
