#ifndef STARTUP_STATE_H
#define STARTUP_STATE_H

// Resolves the very first flight state after boot (runs once from loop(), after
// setup() has brought the sensors up): completes any pending in-flight recovery
// decision (audit #1) and moves STARTUP -> CALIBRATION / PAD_IDLE or ERROR.
void handleInitialStateManagement();

// Forget that the initial state was handled (used by unit tests).
void handleInitialStateManagementReset();

#endif // STARTUP_STATE_H
