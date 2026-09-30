#ifndef STATE_MANAGEMENT_H
#define STATE_MANAGEMENT_H

#include <stdint.h>
#include "data_structures.h" // For FlightStateData and the FlightState enum

// ---------------------------------------------------------------------------
// Persisted flight record (EEPROM) and boot-time recovery
// ---------------------------------------------------------------------------

// Bits of g_pyroFiredMask / FlightStateData::pyroFiredMask.
#define PYRO_FIRED_DROGUE 0x01
#define PYRO_FIRED_MAIN   0x02

// True from the moment BOOST is entered until an explicit reset_flight.
// Persisted. While set, ERROR auto-recovery / clear_errors must not send the
// vehicle back to PAD_IDLE (audit #3).
extern bool g_flightInProgress;
// Channels whose fire window has COMPLETED this flight. Persisted so a reset
// mid-descent never re-fires a channel that already fired (audit #1).
extern uint8_t g_pyroFiredMask;

// Clear the flight-safety flags (flight-in-progress, pyro-fired mask, resume
// count, pending recovery) in RAM only - no EEPROM write. Used by the
// reset_flight command (which then saves) and by unit tests.
void stateManagementResetRuntime();

// Save the live flight record to EEPROM. Never throttled (audit #6): call it on every state
// change and every safety-flag change. Put-if-changed: if nothing but the uptime timestamp
// differs from what is stored, nothing is written (flash wear).
void saveStateToEEPROM();

// Call every main-loop pass: during BOOST/COAST refreshes the record every
// EEPROM_PROGRESS_SAVE_INTERVAL_MS so a reset resumes with a current max altitude and
// burnout age. Does nothing in other states.
void saveFlightProgressPeriodic();

// Phase 1 of boot recovery (call early in setup(), before sensors are up):
// loads the record, restores the pyro-fired mask and flight-in-progress flag,
// and resolves every saved state that needs no sensor evidence. In-flight saved
// states are left PENDING and the vehicle stays in the pyro-inert STARTUP state.
void recoverFromPowerLoss();

// True while an in-flight saved state is waiting for sensor evidence.
bool recoveryPending();

// Phase 2 (call every loop pass until it returns true, once sensors are up):
// observes the live barometer for RECOVERY_EVIDENCE_WINDOW_MS, then decides
// whether the saved in-flight state is plausible and either resumes it or falls
// back to RECOVERY. Returns true once the decision has been applied (or nothing
// was pending). While it returns false the vehicle stays in pyro-inert STARTUP.
//   baroValid        - the barometer produced a sane pressure reading
//   baroSeq          - g_baroSample.seq (only fresh samples are counted)
//   rawAltNoOffsetM  - barometric altitude with NO calibration offset applied
//   nowMs            - millis()
bool recoveryEvidenceStep(bool baroValid, uint32_t baroSeq, float rawAltNoOffsetM, unsigned long nowMs);

// Pure decision function (no globals touched) so the recovery table is testable.
// verticalRateMps is the barometric vertical speed measured over the evidence window.
struct RecoveryDecision {
  FlightState state;   // state to restart in
  bool resume;         // true if this resumes an in-flight state
  bool plausible;      // true if the in-flight plausibility check passed
};
RecoveryDecision decideRecovery(const FlightStateData& record, bool baroValid, float rawAltNoOffsetM, float verticalRateMps);

#endif // STATE_MANAGEMENT_H
