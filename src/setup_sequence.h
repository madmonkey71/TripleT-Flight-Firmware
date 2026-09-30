#ifndef SETUP_SEQUENCE_H
#define SETUP_SEQUENCE_H

#include <stddef.h>

// ---------------------------------------------------------------------------
// Watchdog-safe setup sequencing (audit #14)
//
// The hardware watchdog (WATCHDOG_TIMEOUT_MS, 5 s) is started early in setup() so a hang during
// bring-up - including a bring-up after a mid-flight reset - is recovered by another reset. But
// the slow steps (SD card init, log file creation, sensor inits) used to run back to back with
// only some of them feeding it, so their SUM could exceed the timeout and reset-loop the vehicle.
//
// runSetupSteps() feeds the watchdog before the first step and after every step, so the
// watchdog window only ever has to cover ONE step (a single blocking library call). It is used
// only from setup(); loop() keeps its own single feed per pass, so the in-flight watchdog is
// unchanged.
// ---------------------------------------------------------------------------

typedef bool (*SetupStepFn)(void* ctx);   // returns false if the step failed (bring-up continues)
typedef void (*SetupFeedFn)();

struct SetupStep {
  const char* name;
  SetupStepFn fn;
  void* ctx;
};

// Returns the number of steps that reported failure. Every step runs regardless.
inline int runSetupSteps(const SetupStep* steps, size_t count, SetupFeedFn feed) {
  int failures = 0;
  if (feed) feed();
  for (size_t i = 0; i < count; i++) {
    if (steps[i].fn && !steps[i].fn(steps[i].ctx)) failures++;
    if (feed) feed();
  }
  return failures;
}

#endif // SETUP_SEQUENCE_H
