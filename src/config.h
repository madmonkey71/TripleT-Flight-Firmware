#pragma once

// This file contains central configuration parameters for the TripleT Flight
// Firmware

// ============================================================================
// FIRMWARE VERSION - Update on each release
// ============================================================================
#define FIRMWARE_VERSION "v0.58.0-beta" // beta-0.58: flight-logic safety audit fixes (bench-test before flight)

// Define the board type - Teensy 4.1 only
#ifndef BOARD_TEENSY41
#if defined(__IMXRT1062__) && defined(ARDUINO_TEENSY41)
#define BOARD_TEENSY41
#endif
#endif

// --- Features & Hardware Presence ---

// --- Guidance System Configuration ---
// Enable/disable the entire guidance system including servos and stability
// control Set to 1 to enable guidance (requires servos/actuators), 0 to disable
// for passive rockets
#define ENABLE_GUIDANCE                                                        \
  1 // 1=Enable guidance system, 0=Disable for passive flights

// Configure parachute presence. At least MAIN must be present.
#define DROGUE_PRESENT true // Set to true if drogue deployment is needed
#define MAIN_PRESENT true   // Set to true if main deployment is needed
// HARDWARE NOTE: PYRO_CHANNEL_1 was previously defined as pin 2, which
// collided with NEOPIXEL_PIN (also 2) — NeoPixel data writes would have
// toggled the drogue pyro output. Moved to pin 4 (previously unused).
// Verify the drogue channel is physically wired to pin 4 before flight.
#define PYRO_CHANNEL_1 4    // GPIO pin for drogue deployment (moved off pin 2)
#define PYRO_CHANNEL_2 3    // GPIO pin for main deployment

// --- Automatically derive deployment type and check for errors ---
#if MAIN_PRESENT == 0
// Invalid: Main MUST be present
#error "Invalid Parachute Configuration: MAIN_PRESENT must be 1."
#elif DROGUE_PRESENT == 1 && MAIN_PRESENT == 1
// Both Drogue and Main are present
#define DUAL_DEPLOY 1
#define SINGLE_DEPLOY 0
#elif DROGUE_PRESENT == 0 && MAIN_PRESENT == 1
// Only Main is present
#define DUAL_DEPLOY 0
#define SINGLE_DEPLOY 1
#else // DROGUE_PRESENT == 1 && MAIN_PRESENT == 0 (Handled by first #if)
// This case should technically not be reached due to the first #if, but
// included for completeness
#error "Invalid Parachute Configuration: Logic error - Main must be present."
#endif

#define BUZZER_OUTPUT 1 // Enable buzzer output (1=Enabled, 0=Disabled)
#define USE_KX134                                                              \
  1 // Use KX134 high-g sensor alongside ICM20948 (1=Use, 0=Don't Use)

// --- Telemetry (Phase 6, v1.0.0 release gate) ---
// Set to 1 to enable the Serial5 -> ESP32 -> ESP-NOW radio link.
// When 0, the firmware behaves exactly as before — Serial5 is untouched, no
// packets are emitted, no extra CPU cost in the main loop.
// See wiki/entities/esp32-telemetry.md for the link topology and
// wiki/queries/v1-release-gate-2026-05.md for v1.0.0 acceptance criteria.
#define ENABLE_TELEMETRY 0
#define TELEMETRY_BAUD 115200      // Serial5 baud rate to onboard ESP32 TX
#define TELEMETRY_PERIOD_MS 100    // Send a packet every N ms (10 Hz default)

// --- GPS Interface Configuration ---
// Set GPS_USE_SPI to 1 for SPI connection, 0 for I2C connection
#define GPS_USE_SPI 0 // 0=I2C (default), 1=SPI

// GPS SPI Pins (Targeting NXP MIMXRT1062 BGA Balls: E8=MOSI, E7=MISO, D7=CS,
// D8=SCK) These correspond to Teensy 4.1 hardware SPI0: 11(MOSI), 12(MISO),
// 10(CS), 13(SCK)
#define GPS_SPI_CS_PIN 10   // BGA D7 matches Teensy Pin 10 (CS0)
#define GPS_SPI_MOSI_PIN 11 // BGA E8 matches Teensy Pin 11 (MOSI0)
#define GPS_SPI_MISO_PIN 12 // BGA E7 matches Teensy Pin 12 (MISO0)
#define GPS_SPI_SCK_PIN 13  // BGA D8 matches Teensy Pin 13 (SCK0)

// GPS Control Pins (Targeting NXP BGA Balls: D14=PPS, D13=ONOFF)
// D14 BGA ball = GPIO_AD_B1_14 = Teensy Pin 26
// D13 BGA ball = GPIO_AD_B1_13 = Teensy Pin 39
// Note: We'll just define the Teensy pins here.
// These are included for reference but the library might handle them or they
// are wired directly
#define GPS_PPS_PIN 26
#define GPS_ONOFF_PIN 39

#define GPS_SPI_SPEED 4000000 // SPI clock speed in Hz (default 4MHz)

// --- Pin Definitions ---
#define FLASH_CS_PIN 6 // CS pin for Serial Flash (if used)
#define NEOPIXEL_PIN 2 // Pin for NeoPixel
#define BUZZER_PIN 9   // Pin for buzzer

// --- NeoPixel Configuration ---
#define NEOPIXEL_COUNT 2 // Number of NeoPixels

// --- Flight Logic Parameters ---
// #define MAIN_DEPLOY_ALTITUDE 300   // Deploy main at this height (meters)
// above ground - Now dynamic, see g_main_deploy_altitude_m_agl
#define MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M                                      \
  100.0f // Main parachute deployment height above ground level (meters)
#define BOOST_ACCEL_THRESHOLD                                                  \
  2.0f // Acceleration threshold (in g) to detect liftoff.
#define COAST_ACCEL_THRESHOLD                                                  \
  0.5f // Acceleration threshold (in g) to detect the end of the boost phase
       // (motor burnout).
#define LAUNCH_CONFIRMATION_COUNT                                              \
  5 // Consecutive FRESH accelerometer samples above BOOST_ACCEL_THRESHOLD to
    // detect liftoff (audit #8). A pad bump shorter than this cannot launch.
// BOOST timeout: if burnout is never detected the state machine is forced to COAST after this
// long, with the burnout timestamp set, so the sensor gates and the backup apogee timer are
// always reachable (audit #8). Set above your longest motor burn.
#define BOOST_TIMEOUT_MS 12000
// Burnout on high-drag vehicles: after burnout the specific force is drag deceleration, which
// can stay above COAST_ACCEL_THRESHOLD, so the absolute test alone never fires. Burnout is
// therefore also declared when the specific force magnitude falls below this fraction of the
// peak (smoothed) boost level. Axis-independent on purpose: the IMU mounting is not assumed.
#define BOOST_BURNOUT_PEAK_FRACTION 0.35f
// ...and only once the drop has SETTLED: the reading must be within this fraction of the peak of
// what it was 3 fresh samples earlier. A motor's gradual tail-off (the H125W's thrust falls from 150 N to
// 0 over 1.7 s) crosses the fraction long before burnout but is still falling steeply, so it is
// not mistaken for burnout; drag-only coast after a real burnout is steady.
#define BOOST_BURNOUT_SETTLE_FRACTION 0.05f
#define BOOST_ACCEL_EMA_ALPHA 0.3f                   // smoothing of the boost-level tracker (per fresh sample)
#define COAST_CONFIRMATION_COUNT                                               \
  3 // Consecutive FRESH accelerometer samples below COAST_ACCEL_THRESHOLD to
    // confirm burnout (sensor samples, not main-loop passes - audit #4).
#define APOGEE_CONFIRMATION_COUNT                                              \
  5 // Consecutive FRESH barometer samples required to confirm apogee.
    // Confirmation counts everywhere now advance only when the sensor has
    // produced a new sample (see sensor_samples.h), so 5 = 5 sensor periods.
#define LANDING_CONFIRMATION_COUNT                                             \
  10 // Consecutive FRESH barometer samples of stationarity required to confirm landing.
// Launch altitude / ground reference (audit #11): the mean of the last LAUNCH_ALT_AVG_SAMPLES
// fresh barometer samples on the pad, re-zeroed when ARMED is entered (needs at least
// LAUNCH_ALT_MIN_SAMPLES, otherwise the previous reference is kept).
#define LAUNCH_ALT_AVG_SAMPLES 20
#define LAUNCH_ALT_MIN_SAMPLES 5
// Stationarity window for landing detection: the newest N fresh baro samples must span less
// than LANDING_ALTITUDE_STABLE_THRESHOLD metres.
#define LANDING_WINDOW_SAMPLES 10
// --- Main deployment safeguards (audit #5) ---
#define MAIN_DEPLOY_CONFIRMATION_COUNT 3  // Consecutive FRESH baro samples below the deploy altitude
// If the barometer stops delivering fresh samples during drogue descent, main is deployed after an
// ESTIMATED descent time: (last max AGL - main altitude) / assumed drogue rate * margin, floored at
// MAIN_DEPLOY_FALLBACK_MIN_MS. If the apogee is unknown (baro dead all flight) the fixed
// MAIN_DEPLOY_FALLBACK_TIME_MS is used instead. Deploying a little early is survivable; deploying after
// ground impact is not, hence margin < 1 and an assumed rate above the real one.
// TUNE PER AIRFRAME. Defaults suit the H125W flight in flight_simulation_h125w.py (drogue ~8.5 m/s,
// apogee ~1900 m AGL, ~216 s from apogee to 100 m AGL, ~272 s total): the estimate is then ~147 s,
// i.e. main at roughly 690 m AGL if the barometer fails - early, but well clear of the ground.
#define MAIN_DEPLOY_FALLBACK_TIME_MS 60000
#define MAIN_DEPLOY_ASSUMED_DROGUE_RATE_MPS 10.0f
#define MAIN_DEPLOY_FALLBACK_MARGIN 0.8f
#define MAIN_DEPLOY_FALLBACK_MIN_MS 3000
// Even with a barometer that keeps delivering (possibly wrong) data, main is deployed once this long
// has passed since drogue descent began. It must lie BETWEEN the nominal time from apogee to the main
// altitude (216 s for the H125W flight) and the time to landing (~260 s), or it either fires early on a
// slow-but-healthy descent (harmless: main early) or can never help. TUNE PER AIRFRAME.
#define MAIN_DEPLOY_MAX_DROGUE_TIME_MS 240000UL
// A descent state that never sees touchdown is forced to LANDED after this long.
#define DESCENT_STATE_TIMEOUT_MS 600000UL
#define BACKUP_APOGEE_TIME_MS                                                  \
  20000 // Failsafe time in ms after motor burnout to trigger apogee.
#define APOGEE_GPS_CONFIRMATION_COUNT                                          \
  3 // Consecutive FRESH GPS altitude samples showing descent to confirm
    // apogee.

// Redundant Sensing Apogee Detection
#ifndef APOGEE_BARO_DESCENT_THRESHOLD
#define APOGEE_BARO_DESCENT_THRESHOLD                                          \
  1.0 // Meters change to confirm descent for apogee
#endif
// Accelerometer (free-fall) apogee method. The old test (`icm_accel[2] < 0`) depended on
// the IMU's mounting/axis sign and on noise around zero. Near apogee drag ~ 0, so the
// MAGNITUDE of specific force collapses towards 0 g whichever way the IMU is mounted.
#define APOGEE_ACCEL_FREEFALL_G 0.3f  // |a| below this (g) counts as free fall
#ifndef APOGEE_ACCEL_SAMPLES
#define APOGEE_ACCEL_SAMPLES                                                   \
  5 // Consecutive FRESH accelerometer samples in free fall required
#endif
#define APOGEE_ACCEL_FREEFALL_WINDOW_MS 500          // ...sustained at least this long
#define APOGEE_ACCEL_FREEFALL_WINDOW_NO_BARO_MS 1500 // ...and this long when no fresh barometer can corroborate

// Independent plausibility gates for the sensor-based apogee methods (audit #4).
// The backup timer is deliberately NOT gated by any of them.
#define APOGEE_MIN_TIME_AFTER_BURNOUT_MS 2000   // No sensor method may fire earlier than this after burnout
#define APOGEE_MIN_ALTITUDE_GAIN_M 15.0f        // ...nor before max AGL has reached this (only checked with a working baro)
// Barometric method: ignore the pressure disturbance while the vehicle may still be
// transonic (shock over the static ports gives false "descents"). The descent
// reference restarts when the lockout ends, so a spike during it cannot poison it.
#define APOGEE_BARO_TRANSONIC_LOCKOUT_MS 3000
#define APOGEE_CLIMB_VETO_MPS 10.0f     // Baro still climbing faster than this vetoes the accel/GPS methods
#define APOGEE_HIGH_FORCE_VETO_G 1.5f   // Specific force above this (still decelerating hard) vetoes the baro/GPS methods
#define APOGEE_GPS_DESCENT_THRESHOLD_M 5.0f // GPS altitude drop from its max that counts as descent
// Sensor freshness: data older than this is treated as unavailable.
#define ACCEL_STALE_TIMEOUT_MS 500
#define GPS_STALE_TIMEOUT_MS 2000

// Redundant Sensing Landing Detection
#ifndef LANDING_ACCEL_MIN_G
#define LANDING_ACCEL_MIN_G                                                    \
  0.9f // Minimum acceleration for landing detection (g)
#endif
#ifndef LANDING_ACCEL_MAX_G
#define LANDING_ACCEL_MAX_G                                                    \
  1.1f // Maximum acceleration for landing detection (g)
#endif
#ifndef LANDING_CONFIRMATION_TIME_MS
#define LANDING_CONFIRMATION_TIME_MS                                           \
  2000 // Time in ms of stable conditions to confirm landing
#endif
#ifndef LANDING_ALTITUDE_STABLE_THRESHOLD
#define LANDING_ALTITUDE_STABLE_THRESHOLD                                      \
  1.0 // Metres: max-min of the barometric altitude over the landing window (stationarity)
#endif

// --- State Machine Timeouts & Durations ---
#ifndef PYRO_FIRE_DURATION
#define PYRO_FIRE_DURATION 1000 // Milliseconds for pyro channel to be active
#endif
#ifndef LANDED_TIMEOUT_MS
#define LANDED_TIMEOUT_MS                                                      \
  10000 // Milliseconds to stay in LANDED state before RECOVERY
#endif
#ifndef RECOVERY_TIMEOUT_MS
#define RECOVERY_TIMEOUT_MS                                                    \
  300000 // Milliseconds in RECOVERY before auto-shutdown (5 mins)
#endif
#ifndef ERROR_RECOVERY_ATTEMPT_MS
#define ERROR_RECOVERY_ATTEMPT_MS                                              \
  10000 // Milliseconds in ERROR state before attempting recovery
#endif
#ifndef CALIBRATION_AUTO_TIMEOUT_MS
#define CALIBRATION_AUTO_TIMEOUT_MS                                            \
  120000 // Milliseconds in CALIBRATION before fallback calibration (2 mins)
#endif

// --- Bench-only test commands (audit #13) ---
// TEST_FREEZE deliberately hangs the firmware for 6 s to prove the watchdog resets it. A serial
// command that can freeze the flight computer must not exist in a flight build, so it is compiled
// out unless this is set to 1 (e.g. `-D ENABLE_TEST_COMMANDS=1` in a bench environment). Even
// when compiled in it is refused outside PAD_IDLE.
#ifndef ENABLE_TEST_COMMANDS
#define ENABLE_TEST_COMMANDS 0
#endif

// --- Boot-time recovery plausibility (audit #1) ---
// After a reset the saved flight state is only RESUMED in flight if the live
// barometer proves the vehicle is really up in the air. Otherwise the vehicle
// restarts into a safe, pyro-inert state. See wiki/concepts/state-management.
#define RECOVERY_MIN_AGL_M 30.0f       // Must be at least this far above the saved launch altitude to resume
#define RECOVERY_ALT_MARGIN_M 300.0f   // ...and no higher than saved max altitude + this margin
#define RECOVERY_MAX_RESUMES 3         // Give up resuming after this many resets in one flight
// A vehicle that reset in flight is moving vertically; one sitting on the pad with a
// stale record is not. Recovery therefore watches the fresh barometer for a short
// window and requires a real vertical rate before it will resume.
// The backup apogee timer is restored from the burnout age persisted at the last save, plus this
// allowance for the time lost to the reset itself (watchdog timeout + boot). Restoring a slightly
// LARGER age fires the backup timer slightly EARLIER than nominal: the safe direction (audit #6).
#define RECOVERY_BACKUP_TIMER_ALLOWANCE_MS 3000
// While in BOOST/COAST the flight record is refreshed at this interval (max altitude, burnout age).
#define EEPROM_PROGRESS_SAVE_INTERVAL_MS 1000
#define RECOVERY_MIN_VERTICAL_RATE_MPS 2.0f   // |dz/dt| needed to count as airborne
#define RECOVERY_EVIDENCE_WINDOW_MS 800       // Observe the baro at least this long...
#define RECOVERY_EVIDENCE_MIN_SAMPLES 5       // ...and collect at least this many fresh samples
#define RECOVERY_EVIDENCE_TIMEOUT_MS 3000     // No usable baro by then -> treat evidence as absent

// --- ERROR state / ground proof (audit #2, #3) ---
// ERROR may only be entered from pre-flight states. Leaving ERROR automatically (or via
// clear_errors / clear_to_calibration / skip_calibration) additionally requires proof
// the vehicle is on the ground: never flown, and (if the barometer is calibrated) within
// this many metres of the launch altitude.
#define GROUND_AGL_TOLERANCE_M 30.0f
// reset_flight (audit #7): clears the persisted "flight in progress" state so a landed vehicle can be re-armed.
#define RESET_FLIGHT_TOKEN_TIMEOUT_MS 30000    // The confirmation token expires after this long
#define RESET_FLIGHT_MAX_VERTICAL_SPEED_MPS 1.0f // Baro vertical speed must be below this ("on the ground, not moving")
// A barometer with no fresh sample for this long is treated as failed/unavailable.
#define BARO_STALE_TIMEOUT_MS 500

// --- Sensor Error & Timeout Thresholds ---
#define MAX_SENSOR_FAILURES                                                    \
  3 // Maximum number of consecutive sensor failures before error state
// Hardware watchdog reset timeout. This value is what setup() actually
// programs into WDT_T4 (it was previously 1000 here while setup() hard-coded
// 5 s — the two now agree). Must be a whole number of seconds >= 1.
#define WATCHDOG_TIMEOUT_MS 5000 // Watchdog timer reset timeout in milliseconds
#define BAROMETER_ERROR_THRESHOLD                                              \
  10.0 // Barometer error threshold (m) between readings
#define ACCEL_ERROR_THRESHOLD                                                  \
  10.0 // Accelerometer error threshold (g) between readings
#define GPS_TIMEOUT_MS 5000 // GPS timeout in milliseconds

// --- Storage & Logging Configuration ---
// SD Card
#define SD_CARD_MIN_FREE_SPACE 5 * 1024 * 1024 // 5MB minimum free space
#define SD_CACHE_SIZE 8                        // Cache factor for SD operations
#define LOG_PREALLOC_SIZE 5000000              // Pre-allocate 5MB for log file
#define DISABLE_SDCARD_LOGGING                                                 \
  false // Disable SD card logging by default (only for use in testing)

// Logging Buffers (RAM)
#define MAX_LOG_ENTRIES 1   // Max log entries to buffer in RAM
#define MAX_BUFFER_SIZE 300 // Max size (bytes) of a single log entry string

// External Flash (if used)
#define EXTERNAL_FLASH_MIN_FREE_SPACE 1024 * 1024 // 1MB minimum free space

// --- EEPROM Configuration ---
#define EEPROM_STATE_ADDR                                                      \
  0 // EEPROM address for flight state (now holds the entire FlightStateData
    // struct)
// #define EEPROM_ALTITUDE_ADDR 4             // REMOVED - Part of
// FlightStateData struct #define EEPROM_TIMESTAMP_ADDR 8            // REMOVED
// - Part of FlightStateData struct #define EEPROM_SIGNATURE_ADDR 12 // REMOVED
// - Part of FlightStateData struct
#define EEPROM_SIGNATURE_VALUE                                                 \
  0xBEEF // Signature to validate EEPROM data (16-bit), used within
         // FlightStateData

// --- Madgwick Filter Configuration --- (REMOVED: Madgwick filter no longer
// supported)

// --- Orientation System Configuration ---
// Determines which orientation system is active at startup.
// Kalman filter is now the only option.
#define KALMAN_FILTER_ACTIVE_BY_DEFAULT true

// --- Guidance loop timing / authority (audit #12) ---
#define GUIDANCE_UPDATE_INTERVAL_MS 20   // 50 Hz control loop
#define GUIDANCE_MAX_DT_S 0.1f           // Clamp for a stalled loop so PID integrators/derivatives are not kicked

// --- Kalman filter accelerometer gating (audit #9) ---
// The accelerometer is only a valid tilt (gravity) reference when the vehicle is not accelerating:
// |a| ~ 1 g and not rotating fast. Under thrust, drag or in free fall the "gravity" vector points
// wherever the net force does, and feeding it in corrupts roll/pitch. Outside the band the update
// is SKIPPED and the covariance keeps growing (the gyro integration carries the estimate).
#define KALMAN_ACCEL_GATE_LOW_G 0.9f            // Accept |a| between these (g)...
#define KALMAN_ACCEL_GATE_HIGH_G 1.1f
#define KALMAN_ACCEL_GATE_MAX_GYRO_RPS 2.0f     // ...and only while the angular rate is below this (rad/s)

// --- PID Controller Gains ---

// Roll Axis PID
// Reduced from 1.0/0.1/0.05 to reduce oscillations and improve stability
#define PID_ROLL_KP 0.3f
#define PID_ROLL_KI 0.1f
#define PID_ROLL_KD 0.01f

// Pitch Axis PID
// Reduced from 1.0/0.1/0.05 to reduce oscillations and improve stability
#define PID_PITCH_KP 0.3f
#define PID_PITCH_KI 0.1f
#define PID_PITCH_KD 0.01f

// Yaw Axis PID (e.g., for reaction wheel or differential thrust)
// Reduced from 0.8/0.08/0.03 to reduce oscillations and improve stability
#define PID_YAW_KP 0.2f
#define PID_YAW_KI 0.08f
#define PID_YAW_KD 0.005f

// PID Output Limits (example)
#define PID_OUTPUT_MIN -1.0f          // Min actuator command
#define PID_OUTPUT_MAX 1.0f           // Max actuator command
#define PID_INTEGRAL_LIMIT_ROLL 0.5f  // Anti-windup for roll
#define PID_INTEGRAL_LIMIT_PITCH 0.5f // Anti-windup for pitch
#define PID_INTEGRAL_LIMIT_YAW 0.3f   // Anti-windup for yaw

// --- Actuator Configuration ---
#define ACTUATOR_PITCH_PIN 21      // Teensy pin 21
#define ACTUATOR_ROLL_PIN 23       // Teensy pin 23. Not in hardware design yet
#define ACTUATOR_YAW_PIN 20        // Teensy pin 20
#define SERVO_MIN_PULSE_WIDTH 1000 // Microseconds (adjust as needed)
#define SERVO_MAX_PULSE_WIDTH 2000 // Microseconds (adjust as needed)
#define SERVO_DEFAULT_ANGLE 90     // Default angle for servos (degrees)

// --- SD Card Driver Configuration (Teensy 4.1 Only) ---

// For Teensy 4.1, use the built-in SD card socket with SDIO mode and optimized
// settings
#ifndef SD_CONFIG
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#endif
#ifndef SD_BUF_SIZE       // Buffer size for SdFat library operations
#define SD_BUF_SIZE 65535 // 16KB buffer for SD card operations
#endif
// Note: Teensy 4.1 built-in SDIO SD card slot does not use a card detect pin

// State Management Configuration
// EEPROM_UPDATE_INTERVAL is now defined in constants.h

// Sensor Configuration

// --- Magnetometer Calibration Persistence ---
#define MAG_CAL_EEPROM_ADDR                                                    \
  100 // Starting address in EEPROM for mag calibration
#define MAG_CAL_MAGIC_NUMBER 0xBAADF00D // Magic number to validate stored data

// --- Flight State Machine Configuration ---
#define ARMED_TIMEOUT_MS 300000UL // 5 minutes in ARMED state before disarming

#define APOGEE_DELAY                                                           \
  1000 // Milliseconds to wait after apogee before deploying parachute

// --- Sensor Selections and Configurations ---

#define USE_KX134_FOR_LAUNCH_DETECTION // Use KX134 for launch detection,
                                       // otherwise use ICM20948

#ifdef USE_KX134_FOR_LAUNCH_DETECTION

#endif

// --- Servo Configuration ---
#define NUM_SERVOS 4 // Total number of servos

// --- Phase 6.2: Stability Monitoring & Servo Smoothing ---

// Stability Monitor Thresholds
#define GUIDANCE_STABILITY_ROLL_RATE_LIMIT_DPS 180.0f  // Max roll rate (deg/s)
#define GUIDANCE_STABILITY_PITCH_RATE_LIMIT_DPS 180.0f // Max pitch rate (deg/s)
#define GUIDANCE_STABILITY_YAW_RATE_LIMIT_DPS                                  \
  360.0f // Max yaw rate (deg/s) - higher for yaw
#define GUIDANCE_STABILITY_ROLL_ERROR_LIMIT_DEG                                \
  30.0f // Max roll attitude error (deg)
#define GUIDANCE_STABILITY_PITCH_ERROR_LIMIT_DEG                               \
  20.0f // Max pitch attitude error (deg)
#define GUIDANCE_STABILITY_YAW_ERROR_LIMIT_DEG                                 \
  20.0f // Max yaw attitude error (deg)
#define GUIDANCE_STABILITY_SATURATION_LIMIT_PERCENT                            \
  95.0f // Max servo command saturation (%)
#define GUIDANCE_STABILITY_VIOLATION_DURATION_MS                               \
  500 // Min duration to trigger failsafe (ms)

// Servo Smoother Parameters
#define SERVO_RATE_LIMIT_DPS 10.0f   // Max rate of change (deg/100ms)
#define SERVO_DEADBAND_DEG 0.5f      // Deadband threshold (degrees)
#define SERVO_LOWPASS_CUTOFF_HZ 2.0f // Low-pass filter cutoff (Hz)

// Guidance Failsafe Parameters
#define GUIDANCE_FAILSAFE_LEVEL1_MS 1000 // Duration before gain reduction (ms)
#define GUIDANCE_FAILSAFE_LEVEL2_MS 2000 // Duration before passive mode (ms)
#define GUIDANCE_FAILSAFE_LEVEL3_MS 5000 // Duration before ERROR state (ms)
#define GUIDANCE_FAILSAFE_MIN_GAIN 0.3f  // Minimum PID gain (30% of nominal)

// --- Recovery Beacon Configuration ---
#define RECOVERY_BEACON_SOS_DOT_MS 200  // Duration of an SOS dot
#define RECOVERY_BEACON_SOS_DASH_MS 600 // Duration of an SOS dash
#define RECOVERY_BEACON_SOS_SYMBOL_PAUSE_MS                                    \
  200 // Pause between dots/dashes within a letter
#define RECOVERY_BEACON_SOS_LETTER_PAUSE_MS 600 // Pause between S and O
#define RECOVERY_BEACON_SOS_WORD_PAUSE_MS 1400  // Pause after SOS sequence
#define RECOVERY_BEACON_FREQUENCY_HZ 2500 // Frequency of the recovery beep

// --- Recovery LED Strobe Configuration ---
#define RECOVERY_STROBE_ON_MS 100 // Duration LED is ON for strobe
#define RECOVERY_STROBE_OFF_MS                                                 \
  900 // Duration LED is OFF for strobe (Total cycle 1s)
#define RECOVERY_STROBE_BRIGHTNESS 255 // Brightness of strobe (0-255)
// Strobe color (RGB)
#define RECOVERY_STROBE_R 255
#define RECOVERY_STROBE_G 255
#define RECOVERY_STROBE_B 255

// --- Recovery GPS Beacon Configuration ---
#define RECOVERY_GPS_BEACON_INTERVAL_MS                                        \
  10000 // Interval to print GPS beacon data (10 seconds)

// --- Battery Voltage Monitoring Configuration ---
#define ENABLE_BATTERY_MONITORING 1 // 1 to enable, 0 to disable
#define BATTERY_VOLTAGE_PIN                                                    \
  A7 // Analog pin for battery voltage sensing (example, ensure this is a valid
     // analog pin)
#define ADC_REFERENCE_VOLTAGE                                                  \
  3.3f // ADC reference voltage (e.g., 3.3V for Teensy 4.1)
#define ADC_RESOLUTION                                                         \
  1024.0f // ADC resolution (e.g., 1024 for 10-bit, 4096 for 12-bit. Teensy 4.1
          // default is 10-bit)
// Voltage Divider Resistors (if used) - R1 is connected to battery positive, R2
// to ground, ADC reads between R1 and R2
#define VOLTAGE_DIVIDER_R1 10000.0f // Ohms (e.g., 10k)
#define VOLTAGE_DIVIDER_R2                                                     \
  10000.0f // Ohms (e.g., 10k) - results in a 1/2 divider
#define BATTERY_VOLTAGE_READ_INTERVAL_MS                                       \
  5000 // How often to read and potentially print battery voltage (5 seconds)

// --- Guidance System Failsafe Mechanisms ---

// Maximum Control Surface Deflection Limits (degrees)
// These are absolute values; the control system will apply them symmetrically
// (e.g., MAX_FIN_DEFLECTION_PITCH_DEG of 15 means +/- 15 degrees from neutral)
#define MAX_FIN_DEFLECTION_PITCH_DEG                                           \
  15.0f // Max deflection for pitch control surfaces
#define MAX_FIN_DEFLECTION_YAW_DEG                                             \
  15.0f // Max deflection for yaw control surfaces
#define MAX_FIN_DEFLECTION_ROLL_DEG                                            \
  20.0f // Max deflection for roll control surfaces (if applicable, e.g.
        // ailerons or differential deflection)

// Stability Monitoring (legacy guidance_check_stability path, used in
// BOOST/COAST). These previously carried their own values that disagreed with
// the Phase 6.2 GUIDANCE_STABILITY_* set (roll/yaw rate limits were swapped
// and saturation was 90% vs 95%). They now alias the Phase 6.2 constants so
// both stability systems enforce a single, consistent set of limits.
#define STABILITY_MAX_PITCH_RATE_DPS GUIDANCE_STABILITY_PITCH_RATE_LIMIT_DPS
#define STABILITY_MAX_ROLL_RATE_DPS GUIDANCE_STABILITY_ROLL_RATE_LIMIT_DPS
#define STABILITY_MAX_YAW_RATE_DPS GUIDANCE_STABILITY_YAW_RATE_LIMIT_DPS

// Stability Monitoring: Attitude Error (when guidance is active and trying to
// hold/achieve a target) If the difference between target attitude and actual
// attitude exceeds this for STABILITY_VIOLATION_DURATION_MS, a stability
// failsafe may be triggered.
#define STABILITY_MAX_ATTITUDE_ERROR_PITCH_DEG                                 \
  GUIDANCE_STABILITY_PITCH_ERROR_LIMIT_DEG
#define STABILITY_MAX_ATTITUDE_ERROR_ROLL_DEG                                  \
  GUIDANCE_STABILITY_ROLL_ERROR_LIMIT_DEG
#define STABILITY_MAX_ATTITUDE_ERROR_YAW_DEG                                   \
  GUIDANCE_STABILITY_YAW_ERROR_LIMIT_DEG

// Stability Monitoring: Control Effort (Actuator Saturation)
#define STABILITY_ACTUATOR_SATURATION_LEVEL_PERCENT                            \
  GUIDANCE_STABILITY_SATURATION_LIMIT_PERCENT

// Common duration for stability violations
#define STABILITY_VIOLATION_DURATION_MS                                        \
  GUIDANCE_STABILITY_VIOLATION_DURATION_MS

// --- Trajectory Following Configuration ---
#define MAX_TRAJECTORY_WAYPOINTS 50 // Max number of waypoints in a trajectory
#define DEFAULT_WAYPOINT_ACCEPTANCE_RADIUS_M                                   \
  10.0f // Default radius in meters to consider a waypoint "reached"

// PID Gains for Cross-Track Error (XTE) Controller (outputs a heading/yaw rate
// adjustment or bank angle)
#define TRAJ_XTE_PID_KP 0.5f
#define TRAJ_XTE_PID_KI 0.05f
#define TRAJ_XTE_PID_KD 0.01f
#define TRAJ_XTE_PID_INTEGRAL_LIMIT 0.2f // Limit for the integral term
#define TRAJ_XTE_PID_OUTPUT_LIMIT                                              \
  0.5f // Max output (e.g., radians for heading adjustment, or normalized bank
       // command)

// PID Gains for Altitude Controller (along trajectory, outputs a pitch
// adjustment)
#define TRAJ_ALT_PID_KP 0.3f
#define TRAJ_ALT_PID_KI 0.03f
#define TRAJ_ALT_PID_KD 0.01f
#define TRAJ_ALT_PID_INTEGRAL_LIMIT 0.2f // Limit for the integral term
#define TRAJ_ALT_PID_OUTPUT_LIMIT                                              \
  0.3f // Max output (e.g., radians for pitch adjustment)
