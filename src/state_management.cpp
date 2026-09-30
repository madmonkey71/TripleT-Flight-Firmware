#include "state_management.h"
#include <EEPROM.h>
#include "constants.h" // For EEPROM_UPDATE_INTERVAL and other timing constants
#include "config.h" // For EEPROM configuration defines
#include "utility_functions.h" // For getStateName
#include "debug_flags.h" // For g_debugFlags
#include "flight_logic.h" // For flightLogicReset()
#include <Arduino.h> // For Serial, millis()

// Global instance of FlightStateData, mirroring State Machine.md
FlightStateData stateData; // This holds the data read from/written to EEPROM
unsigned long lastStateSaveTime = 0; // Renamed from lastStateSave to avoid conflict if State Machine.md meant a global in main .cpp

// Persisted flight-safety flags (see state_management.h)
bool g_flightInProgress = false;
uint8_t g_pyroFiredMask = 0;
static uint8_t s_resumeCount = 0;

// Record loaded at boot whose in-flight state still awaits sensor evidence.
static FlightStateData s_pendingRecord;
static bool s_recoveryPending = false;

// Extern variables from TripleT_Flight_Firmware.cpp that are needed by these functions.
// These are the live globals the state machine actually mutates (flight_logic.cpp).
extern FlightState g_currentFlightState;
extern FlightState g_previousFlightState;
extern unsigned long g_stateEntryTime;
extern float g_launchAltitude;     // Ground altitude recorded at PAD_IDLE
extern float g_maxAltitudeReached; // Peak altitude tracked during flight
extern float g_currentAltitude;    // Latest altitude from sensors
extern float g_main_deploy_altitude_m_agl; // Added for saving state
extern bool g_baroCalibrated;      // Barometer calibration status
// From ms5611_functions.cpp — persisted so in-flight recovery keeps its altitude reference.
extern float baro_altitude_offset;
extern bool baro_calibration_done;
extern unsigned long boostEndTime; // flight_logic.cpp: millis() at motor burnout (backup apogee timer base)
extern DebugFlags g_debugFlags;     // Declare g_debugFlags

// States between liftoff and main descent (the vehicle is, or was, in the air).
static bool isInFlightState(FlightState s) { return s >= BOOST && s <= MAIN_DESCENT; }

// Evidence gathering state for the pending decision (see recoveryEvidenceStep).
static unsigned long s_evStartMs = 0;
static unsigned long s_evFirstMs = 0, s_evLastMs = 0;
static float s_evFirstAlt = 0.0f, s_evLastAlt = 0.0f;
static uint32_t s_evLastSeq = 0;
static int s_evCount = 0;

void stateManagementResetRuntime() {
  g_flightInProgress = false;
  g_pyroFiredMask = 0;
  s_resumeCount = 0;
  s_recoveryPending = false;
  s_evStartMs = 0;
  s_evCount = 0;
}

// Every persisted field except the uptime timestamp (which changes on every save and is not
// comparable across reboots) - used for put-if-changed.
static bool sameRecord(const FlightStateData& a, const FlightStateData& b) {
  return a.state == b.state && a.launchAltitude == b.launchAltitude && a.maxAltitude == b.maxAltitude &&
         a.currentAltitude == b.currentAltitude && a.mainDeployAltitudeAgl == b.mainDeployAltitudeAgl &&
         a.baroAltitudeOffset == b.baroAltitudeOffset && a.baroCalibrated == b.baroCalibrated &&
         a.flightInProgress == b.flightInProgress && a.pyroFiredMask == b.pyroFiredMask &&
         a.resumeCount == b.resumeCount && a.burnoutAgeMs == b.burnoutAgeMs && a.signature == b.signature;
}

void saveStateToEEPROM() {
  const unsigned long now = millis();

  FlightStateData rec;
  memset(&rec, 0, sizeof rec);
  rec.state = static_cast<uint8_t>(g_currentFlightState); // Cast enum to uint8_t
  rec.launchAltitude = g_launchAltitude;
  rec.maxAltitude = g_maxAltitudeReached;
  rec.currentAltitude = g_currentAltitude; // This should be the latest available altitude
  rec.mainDeployAltitudeAgl = g_main_deploy_altitude_m_agl;
  rec.baroAltitudeOffset = baro_altitude_offset;
  rec.baroCalibrated = g_baroCalibrated ? 1 : 0;
  rec.flightInProgress = g_flightInProgress ? 1 : 0;
  rec.pyroFiredMask = g_pyroFiredMask;
  rec.resumeCount = s_resumeCount;
  // Time since burnout (0 before burnout / outside COAST) so the backup timer survives a reset.
  rec.burnoutAgeMs = (g_currentFlightState == COAST && boostEndTime > 0) ? static_cast<uint32_t>(now - boostEndTime) : 0;
  rec.signature = EEPROM_SIGNATURE_VALUE;

  lastStateSaveTime = now;

  FlightStateData stored;
  EEPROM.get(EEPROM_STATE_ADDR, stored);
  if (sameRecord(stored, rec)) {
    return; // identical to what is already stored: no flash write
  }

  rec.timestamp = now;
  EEPROM.put(EEPROM_STATE_ADDR, rec);
  stateData = rec;

  if (g_debugFlags.enableSystemDebug) {
    Serial.print(F("Flight state saved to EEPROM: "));
    Serial.println(getStateName(g_currentFlightState));
  }
}

void saveFlightProgressPeriodic() {
  if (g_currentFlightState != BOOST && g_currentFlightState != COAST) return;
  if (millis() - lastStateSaveTime < EEPROM_PROGRESS_SAVE_INTERVAL_MS) return;
  saveStateToEEPROM();
}

static bool loadStateFromEEPROM() { // Changed to static
  EEPROM.get(EEPROM_STATE_ADDR, stateData);

  if (stateData.signature != EEPROM_SIGNATURE_VALUE) {
    if (g_debugFlags.enableSystemDebug) { // Only print if debug is enabled
        Serial.println(F("No valid flight state found in EEPROM. Initializing with default values."));
    }
    // Optionally initialize stateData to defaults if signature is bad
    stateData.state = static_cast<uint8_t>(STARTUP); // Default to STARTUP
    stateData.launchAltitude = 0.0f;
    stateData.maxAltitude = 0.0f;
    stateData.currentAltitude = 0.0f;
    stateData.mainDeployAltitudeAgl = 0.0f; // Default for new field
    stateData.baroAltitudeOffset = 0.0f;
    stateData.baroCalibrated = 0;
    stateData.flightInProgress = 0;
    stateData.pyroFiredMask = 0;
    stateData.resumeCount = 0;
    stateData.burnoutAgeMs = 0;
    stateData.timestamp = 0;
    // Do not set signature here, as it indicates invalid data
    return false;
  }

  if (g_debugFlags.enableSystemDebug) {
      Serial.println(F("Found valid flight state in EEPROM:"));
      Serial.print(F("State: "));
      Serial.println(getStateName(static_cast<FlightState>(stateData.state)));
      Serial.print(F("Launch altitude: "));
      Serial.println(stateData.launchAltitude);
      Serial.print(F("Max altitude: "));
      Serial.println(stateData.maxAltitude);
      Serial.print(F("Last altitude: "));
      Serial.println(stateData.currentAltitude);
      Serial.print(F("Main Deploy AGL: ")); Serial.println(stateData.mainDeployAltitudeAgl); // Print new field
      Serial.print(F("Flight in progress: ")); Serial.println(stateData.flightInProgress);
      Serial.print(F("Pyro fired mask: ")); Serial.println(stateData.pyroFiredMask);
      Serial.print(F("Timestamp: "));
      Serial.println(stateData.timestamp);
  }
  return true;
}

// ---------------------------------------------------------------------------
// Recovery decision table (audit #1, #6)
//
//  saved state                 in-flight evidence?  restart state
//  --------------------------  -------------------  ------------------------------------
//  STARTUP/CALIBRATION/PAD_IDLE  n/a                STARTUP (normal boot sequence)
//  ARMED                         n/a                PAD_IDLE (disarmed)
//  BOOST, COAST                  plausible          COAST  (apogee detection re-armed; backup timer restored from
//                                                   the persisted burnout age + RECOVERY_BACKUP_TIMER_ALLOWANCE_MS)
//  APOGEE, DROGUE_DEPLOY         plausible          drogue already fired ? DROGUE_DESCENT : APOGEE
//  DROGUE_DESCENT                plausible          DROGUE_DESCENT
//  MAIN_DEPLOY                   plausible          main already fired ? MAIN_DESCENT : DROGUE_DESCENT
//  MAIN_DESCENT                  plausible          MAIN_DESCENT
//  any of BOOST..MAIN_DESCENT    NOT plausible      RECOVERY (pyro-inert; needs reset_flight)
//  LANDED, RECOVERY              n/a                RECOVERY
//  ERROR                         n/a                flight was in progress ? RECOVERY : ERROR
//  invalid value                 n/a                STARTUP
//
// No row restores DROGUE_DEPLOY or MAIN_DEPLOY directly: the pyro-firing states
// are only ever reached through the normal APOGEE / altitude logic, after the
// evidence check, and never for a channel whose fired flag is set.
// ---------------------------------------------------------------------------
RecoveryDecision decideRecovery(const FlightStateData& rec, bool baroValid, float rawAltNoOffsetM, float verticalRateMps) {
  RecoveryDecision d = {STARTUP, false, false};
  if (static_cast<uint8_t>(rec.state) > static_cast<uint8_t>(ERROR)) {
    return d; // invalid enum value -> normal startup
  }
  const FlightState saved = static_cast<FlightState>(rec.state);

  switch (saved) {
    case STARTUP:
    case CALIBRATION:
    case PAD_IDLE:
      d.state = STARTUP;
      return d;
    case ARMED:
      d.state = PAD_IDLE; // was armed but never launched
      return d;
    case LANDED:
    case RECOVERY:
      d.state = RECOVERY;
      return d;
    case ERROR:
      d.state = rec.flightInProgress ? RECOVERY : ERROR;
      return d;
    default:
      break; // BOOST .. MAIN_DESCENT handled below
  }

  // In-flight saved state: only resume with live evidence that we are airborne.
  const float aglNow = rawAltNoOffsetM + rec.baroAltitudeOffset - rec.launchAltitude;
  const bool plausible =
      baroValid && rec.baroCalibrated &&
      isfinite(aglNow) && isfinite(rec.maxAltitude) && isfinite(verticalRateMps) &&
      rec.resumeCount < RECOVERY_MAX_RESUMES &&
      fabsf(verticalRateMps) >= RECOVERY_MIN_VERTICAL_RATE_MPS &&
      aglNow >= RECOVERY_MIN_AGL_M &&
      aglNow <= rec.maxAltitude + RECOVERY_ALT_MARGIN_M;
  if (!plausible) {
    d.state = RECOVERY;
    return d;
  }

  d.plausible = true;
  d.resume = true;
  const bool drogueFired = (rec.pyroFiredMask & PYRO_FIRED_DROGUE) != 0;
  const bool mainFired = (rec.pyroFiredMask & PYRO_FIRED_MAIN) != 0;
  switch (saved) {
    case BOOST:
    case COAST:          d.state = COAST; break;
    case APOGEE:
    case DROGUE_DEPLOY:  d.state = drogueFired ? DROGUE_DESCENT : APOGEE; break;
    case DROGUE_DESCENT: d.state = DROGUE_DESCENT; break;
    case MAIN_DEPLOY:    d.state = mainFired ? MAIN_DESCENT : DROGUE_DESCENT; break;
    case MAIN_DESCENT:   d.state = MAIN_DESCENT; break;
    default:             d.state = RECOVERY; d.resume = false; break; // unreachable
  }
  return d;
}

// Apply a decision to the live globals.
static void applyRecoveryDecision(const RecoveryDecision& d, const FlightStateData& rec) {
  const FlightState saved = static_cast<FlightState>(rec.state <= ERROR ? rec.state : STARTUP);

  g_currentFlightState = d.state;
  g_stateEntryTime = millis();

  // The saved record proves a flight happened if it was in an in-flight state
  // (or later), even if we cannot resume it.
  if (isInFlightState(saved) || saved == LANDED || saved == RECOVERY) g_flightInProgress = true;

  if (saved == ARMED && rec.baroCalibrated) {
    // Disarmed to PAD_IDLE: keep the ground reference so no re-calibration is needed.
    baro_altitude_offset = rec.baroAltitudeOffset;
    baro_calibration_done = true;
    g_baroCalibrated = true;
  }

  if (d.resume) {
    flightLogicReset(); // detectors must start clean; nothing carries over from a previous run
    flightSetLaunchAltitude(rec.launchAltitude);
    g_maxAltitudeReached = rec.maxAltitude;
    g_main_deploy_altitude_m_agl = rec.mainDeployAltitudeAgl;
    baro_altitude_offset = rec.baroAltitudeOffset;
    baro_calibration_done = true;
    g_baroCalibrated = true;
    s_resumeCount = static_cast<uint8_t>(rec.resumeCount + 1);
    if (d.state == COAST) {
      // Re-arm the backup apogee timer from the burnout age persisted at the last save (0 if the
      // reset happened during BOOST), plus an allowance for the time the reset itself cost. The
      // restored age is therefore >= the true age, so the timer never fires LATER than nominal.
      const unsigned long now = millis();
      const unsigned long age = (saved == COAST ? rec.burnoutAgeMs : 0UL) + RECOVERY_BACKUP_TIMER_ALLOWANCE_MS;
      boostEndTime = now > age ? now - age : 1; // never 0: 0 means "unset"
    }
  }

  if (g_debugFlags.enableSystemDebug || d.state != STARTUP) {
    Serial.print(F("Recovery: saved "));
    Serial.print(getStateName(saved));
    Serial.print(F(" -> restart in "));
    Serial.print(getStateName(d.state));
    Serial.println(d.resume ? F(" (RESUMED in flight)") : (isInFlightState(saved) ? F(" (in-flight evidence NOT plausible)") : F("")));
  }

  if (d.state != STARTUP || g_flightInProgress) {
    saveStateToEEPROM(); // persist the decision (resume count, RECOVERY, flight flag)
  }
}

void recoverFromPowerLoss() {
  s_recoveryPending = false;
  if (!loadStateFromEEPROM()) {
    // No valid data, or data is explicitly invalid. Start fresh.
    // g_currentFlightState is already STARTUP by default.
    if (g_debugFlags.enableSystemDebug) {
        Serial.println(F("Proceeding with normal startup sequence."));
    }
    return;
  }

  if (static_cast<uint8_t>(stateData.state) > static_cast<uint8_t>(ERROR)) {
    Serial.println(F("WARNING: Invalid FlightState in EEPROM, defaulting to STARTUP"));
    stateData.state = static_cast<uint8_t>(STARTUP);
  }

  // Safety flags are restored unconditionally, before anything else can run:
  // a channel that already fired must never fire again.
  g_pyroFiredMask = stateData.pyroFiredMask & (PYRO_FIRED_DROGUE | PYRO_FIRED_MAIN);
  g_flightInProgress = stateData.flightInProgress != 0;
  s_resumeCount = stateData.resumeCount;

  const FlightState saved = static_cast<FlightState>(stateData.state);
  g_currentFlightState = STARTUP; // pyro-inert until the decision is made

  if (isInFlightState(saved)) {
    // Needs live barometer evidence: wait for resolvePendingRecovery().
    s_pendingRecord = stateData;
    s_recoveryPending = true;
    if (g_debugFlags.enableSystemDebug) {
      Serial.print(F("Recovery: saved "));
      Serial.print(getStateName(saved));
      Serial.println(F(" is an in-flight state - waiting for sensor evidence."));
    }
    return;
  }

  applyRecoveryDecision(decideRecovery(stateData, false, 0.0f, 0.0f), stateData);
}

bool recoveryPending() { return s_recoveryPending; }

bool recoveryEvidenceStep(bool baroValid, uint32_t baroSeq, float rawAltNoOffsetM, unsigned long nowMs) {
  if (!s_recoveryPending) return true;

  if (s_evStartMs == 0) {
    s_evStartMs = nowMs > 0 ? nowMs : 1;
    s_evCount = 0;
  }

  // Only genuinely new barometer samples count as evidence.
  if (baroValid && (s_evCount == 0 || baroSeq != s_evLastSeq)) {
    if (s_evCount == 0) { s_evFirstMs = nowMs; s_evFirstAlt = rawAltNoOffsetM; }
    s_evLastMs = nowMs;
    s_evLastAlt = rawAltNoOffsetM;
    s_evLastSeq = baroSeq;
    s_evCount++;
  }

  const bool windowDone = s_evCount >= RECOVERY_EVIDENCE_MIN_SAMPLES &&
                          (s_evLastMs - s_evFirstMs) >= RECOVERY_EVIDENCE_WINDOW_MS;
  const bool timedOut = (nowMs - s_evStartMs) >= RECOVERY_EVIDENCE_TIMEOUT_MS;
  if (!windowDone && !timedOut) return false;

  s_recoveryPending = false;
  const bool haveEvidence = windowDone;
  float vs = 0.0f;
  if (haveEvidence) {
    vs = (s_evLastAlt - s_evFirstAlt) / ((s_evLastMs - s_evFirstMs) / 1000.0f);
  }
  s_evStartMs = 0;
  applyRecoveryDecision(decideRecovery(s_pendingRecord, haveEvidence, s_evLastAlt, vs), s_pendingRecord);
  return true;
}
