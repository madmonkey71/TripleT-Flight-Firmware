# TripleT Flight State Machine

## Overview
This document outlines the state machine design for the TripleT Flight Controller. The state machine manages the flight sequence from initialization through landing and recovery, with appropriate sensor monitoring and recovery system deployment.

## Arduino Framework Alignment
The state machine is implemented within the Arduino framework:
- **`setup()`**: Handles the `STARTUP` and `CALIBRATION` states.
- **`loop()`**: Manages all flight states from `PAD_IDLE` through `RECOVERY`, and handles `ERROR` states.

## State Machine Diagram
```mermaid
graph TD
    A[Start] --> B(setup);

    subgraph setup
        direction TB
        S[STARTUP] --> C[CALIBRATION];
    end

    C -- "Setup OK" --> PI[PAD_IDLE];
    C -- "Sensor/Config Error" --> ERR[ERROR];
    C -- "Abort/Test/Data" --> LND[LANDED]; // Special case path for ground ops

    subgraph loop [LOOP FUNCTION]
        direction TD
        PI --> ARM[ARMED];
        ARM --> BST[BOOST];
        BST --> CST[COAST];
        CST --> APG[APOGEE];
        APG --> DD[DROGUE_DEPLOY];
        DD --> DDS[DROGUE_DESCENT];
        DDS --> MD[MAIN_DEPLOY];
        MD --> MDS[MAIN_DESCENT];
        MDS --> LND; // LANDED node shared with setup path
        LND --> RCV[RECOVERY];
        RCV --> PI; // Option to re-arm or stay in RECOVERY for data download

        // Generic Error Transitions (can occur from most states)
        PI --> ERR;
        ARM --> ERR;
        BST --> ERR;
        CST --> ERR;
        APG --> ERR;
        DD --> ERR;
        DDS --> ERR;
        MD --> ERR;
        MDS --> ERR;
        LND --> ERR;
        RCV --> ERR;
        ERR -- "Attempt Recovery/Clear" --> PI; // Attempt to return to PAD_IDLE
    end
```

**Note on CALIBRATION to LANDED direct path:**
The direct path from `CALIBRATION` to `LANDED` represents special scenarios:
1.  **Abort handling**: Used when critical errors are detected during calibration that don't warrant a full `ERROR` state but prevent flight.
2.  **Ground testing**: Allows testing recovery systems without a full flight sequence.
3.  **Data retrieval**: Enables downloading calibration data before a launch without power cycling.

This path ensures the system can safely close logs and prepare for inspection regardless of when an abort becomes necessary.

**Error State Handling:**
The `ERROR` state can be entered from most other states if a critical sensor failure, configuration issue, or unexpected event occurs. The system will attempt to enter a safe mode. Depending on the error's nature and system configuration, it might attempt to recover or require manual intervention (e.g., via a serial command to clear errors and return to `PAD_IDLE`).

## Implementation Details

### State Definition
The `FlightState` enum is defined in `src/data_structures.h`:
```cpp
// In src/data_structures.h
enum FlightState : uint8_t {
  STARTUP,        // Initial state during power-on
  CALIBRATION,    // Sensor calibration state
  PAD_IDLE,       // On pad waiting for arm command
  ARMED,          // Armed and ready for launch
  BOOST,          // Motor burning, accelerating
  COAST,          // Unpowered flight upward
  APOGEE,         // Peak altitude reached
  DROGUE_DEPLOY,  // Deploying drogue parachute
  DROGUE_DESCENT, // Descending under drogue
  MAIN_DEPLOY,    // Deploying main parachute
  MAIN_DESCENT,   // Descending under main
  LANDED,         // On ground after flight
  RECOVERY,       // Post-flight data collection
  ERROR           // Error condition
};
```

### Configuration Parameters
Key configuration parameters affecting the state machine are primarily located in `src/config.h`. These include, but are not limited to:

```cpp
// In src/config.h (example parameters, refer to actual file for complete list)

// Recovery system configuration
#define DROGUE_PRESENT true        // Set to true if drogue deployment is needed
#define MAIN_PRESENT true          // Set to true if main deployment is needed
#define PYRO_CHANNEL_1 2           // GPIO pin for drogue deployment (actual pin defined in config.h)
#define PYRO_CHANNEL_2 3           // GPIO pin for main deployment (actual pin defined in config.h)
#define MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M 100.0f // Default height in meters AGL for main parachute deployment (actual value from config.h)

// Flight detection thresholds (values from config.h)
#define BOOST_ACCEL_THRESHOLD 2.0f  // Acceleration threshold for liftoff detection (g)
#define COAST_ACCEL_THRESHOLD 0.5f  // Acceleration threshold indicating motor burnout (g)
// Note: APOGEE_VELOCITY_THRESHOLD is not directly used in the current apogee detection logic.
// Landing detection uses LANDING_ACCEL_MIN_G, LANDING_ACCEL_MAX_G, LANDING_CONFIRMATION_TIME_MS, LANDING_ALTITUDE_STABLE_THRESHOLD.

// Error detection configuration (values from config.h)
#define MAX_SENSOR_FAILURES 3      // Maximum number of consecutive sensor failures before error state
// Note: Watchdog is not currently implemented in the v0.48 codebase provided. WATCHDOG_TIMEOUT_MS is in config.h but not used.

// EEPROM configuration for non-volatile storage (values from config.h and constants.h)
#define EEPROM_STATE_ADDR 0                // EEPROM address for FlightStateData struct
#define EEPROM_SIGNATURE_VALUE 0xBEEF      // Signature to validate EEPROM data
#define EEPROM_UPDATE_INTERVAL 5000        // Save state every 5 seconds (from constants.h)

// Redundant sensing configuration (values from config.h)
#define APOGEE_CONFIRMATION_COUNT 5        // Barometer readings for apogee
#define APOGEE_ACCEL_CONFIRMATION_COUNT 5  // Accelerometer readings for apogee
#define APOGEE_GPS_CONFIRMATION_COUNT 3    // GPS readings for apogee
#define BACKUP_APOGEE_TIME_MS 20000        // Backup time-based apogee detection (ms after boost)
// Note: LANDING_CONFIRMATION_COUNT is in config.h, but flight_logic.cpp uses LANDING_CONFIRMATION_TIME_MS.
```
**Note:** The C++ code blocks for `Non-Volatile Storage Implementation`, `Redundant Sensing for Critical Events`, `Enhanced Flight State Processing`, `Setup Function Enhancement`, and `Testing Considerations` are illustrative examples based on the structure of the provided firmware files (`src/state_management.cpp`, `src/flight_logic.cpp`, etc.). Refer to the actual source code for the most up-to-date and complete implementation details.

### Non-Volatile Storage Implementation
(Corresponds to `src/state_management.cpp`)

EEPROM storage is used for critical flight data to enable recovery from power loss.
- The `FlightStateData` struct (defined in `src/data_structures.h`) stores the current `FlightState` (as `uint8_t`), `launchAltitude`, `maxAltitude`, `currentAltitude`, `mainDeployAltitudeAgl`, a `timestamp`, and a validation `signature`.
- `saveStateToEEPROM()` updates this struct with current global values and writes it to EEPROM address `EEPROM_STATE_ADDR`. Saves occur periodically (defined by `EEPROM_UPDATE_INTERVAL` in `constants.h`) and immediately during critical state transitions like `APOGEE`, `DROGUE_DEPLOY`, `MAIN_DEPLOY`, and `LANDED`.
- `loadStateFromEEPROM()` (static helper in `state_management.cpp`) reads the struct from EEPROM and validates it using `EEPROM_SIGNATURE_VALUE`.
- `recoverFromPowerLoss()` is called during `setup()` (via `handleInitialStateManagement()` in `TripleT_Flight_Firmware.cpp`).
    - If valid data is loaded, it restores `currentFlightState`, `launchAltitude`, `maxAltitudeReached`, and `g_main_deploy_altitude_m_agl`.
    - It then determines the appropriate state to resume in based on the saved state (e.g., `BOOST`/`COAST` resumes to `DROGUE_DESCENT`, `ARMED` resumes to `PAD_IDLE`).

```cpp
// In src/data_structures.h
struct FlightStateData {
  uint8_t state;            // Current flight state (cast to/from FlightState)
  float launchAltitude;     // Stored launch altitude
  float maxAltitude;        // Maximum altitude reached
  float currentAltitude;    // Current altitude
  float mainDeployAltitudeAgl; // Dynamic main deployment altitude
  unsigned long timestamp;  // Timestamp of last save
  uint16_t signature;       // Validation signature (EEPROM_SIGNATURE_VALUE from config.h)
};

// In src/state_management.cpp
// extern FlightState currentFlightState; // (Actually g_currentFlightState from TripleT_Flight_Firmware.cpp)
// extern float launchAltitude;
// extern float maxAltitudeReached;
// extern float currentAltitude; // (Actually g_currentAltitude from TripleT_Flight_Firmware.cpp)
// extern float g_main_deploy_altitude_m_agl;

void saveStateToEEPROM(); // Uses global flight variables
void recoverFromPowerLoss(); // Modifies global flight variables
```

### Redundant Sensing for Critical Events
(Corresponds to `src/flight_logic.cpp`)

Multiple methods are used to detect critical flight events to improve reliability.

**Apogee Detection (`detectApogee()` in `src/flight_logic.cpp`):**
1.  **Barometric (Primary):** Checks if the current barometric altitude (`ms5611_get_altitude()`) is less than `g_maxAltitudeReached` for `APOGEE_CONFIRMATION_COUNT` consecutive readings.
2.  **Accelerometer (Secondary):** Checks if the Z-axis of the ICM-20948 accelerometer (`icm_accel[2]`) indicates negative Gs (freefall) for `APOGEE_ACCEL_CONFIRMATION_COUNT` consecutive readings.
3.  **GPS Altitude (Tertiary):** Checks if the GPS altitude (`getGPSAltitude()`) is less than its recorded maximum by more than 5 meters for `APOGEE_GPS_CONFIRMATION_COUNT` consecutive readings.
4.  **Backup Timer (Failsafe):** If `BACKUP_APOGEE_TIME_MS` has elapsed since `boostEndTime` (motor burnout) and no other method has detected apogee, it's triggered.

**Landing Detection (`detectLanding()` in `src/flight_logic.cpp`):**
- Uses a moving average of barometric altitude (`ms5611_get_altitude()`).
- Condition 1: Average altitude is close to `g_launchAltitude` (within `LANDING_ALTITUDE_STABLE_THRESHOLD`).
- Condition 2: Accelerometer magnitude (`get_accel_magnitude()`) is within a stable G-force range (`LANDING_ACCEL_MIN_G` to `LANDING_ACCEL_MAX_G`).
- If both conditions are met for `LANDING_CONFIRMATION_TIME_MS`, landing is confirmed.

**Motor Burnout Detection (`detectBoostEnd()` in `src/flight_logic.cpp`):**
- Called during `BOOST` state.
- If accelerometer magnitude (`get_accel_magnitude()`) drops below `COAST_ACCEL_THRESHOLD`, `boostEndTime` is recorded, and state transitions to `COAST`.

```cpp
// In src/flight_logic.cpp
// extern float g_maxAltitudeReached;
// extern bool g_baroCalibrated; (used by ms5611_get_altitude)
// extern float icm_accel[3];
// extern bool g_icm20948_ready;
// extern SFE_UBLOX_GNSS myGNSS; (used by getFixType, getGPSAltitude)
// extern unsigned long boostEndTime;
// extern float g_launchAltitude;
// extern bool g_kx134_initialized_ok;
// extern float kx134_accel[3];


bool detectApogee();
bool detectLanding();
void detectBoostEnd();
```

### Enhanced Flight State Processing with Redundant Sensing
(Corresponds to `ProcessFlightState()` in `src/flight_logic.cpp`)

The `ProcessFlightState()` function in `src/flight_logic.cpp` orchestrates the state machine:
- **Health Checks:** Periodically calls `isSensorSuiteHealthy()` (from `utility_functions.cpp`). If unhealthy for the current operational state (and not within the error clear grace period), transitions to `ERROR`.
- **Automatic Error Recovery:** If in `ERROR` state, periodically checks if health is restored to transition back to `PAD_IDLE` or `CALIBRATION`. A grace period (`errorClearGracePeriod`) prevents immediate re-entry to `ERROR`.
- **State Transitions:**
    - On entering a new state, `g_stateEntryTime` is recorded, `setFlightStateLED()` updates NeoPixel status, data is logged, and state is saved to EEPROM.
    - **CALIBRATION:** Waits for `g_baroCalibrated` to become true (via `calibrate` command), then moves to `PAD_IDLE`.
    - **PAD_IDLE:** Sets `g_launchAltitude`, resets flight metrics. Awaits `arm` command.
    - **ARMED:** Calculates `g_main_deploy_altitude_m_agl`. Transitions to `BOOST` if `get_accel_magnitude()` exceeds `BOOST_ACCEL_THRESHOLD`.
    - **BOOST:** Calls `detectBoostEnd()`. If burnout detected, captures current orientation (Kalman or ICM) using `guidance_set_target_orientation_euler()` for attitude hold, then transitions to `COAST`. Updates `g_maxAltitudeReached`.
    - **COAST:** Calls `detectApogee()`. If apogee detected, transitions to `APOGEE`. Updates `g_maxAltitudeReached`.
    - **APOGEE:** Transitions to `DROGUE_DEPLOY` (if `DROGUE_PRESENT`), else `MAIN_DEPLOY` (if `MAIN_PRESENT`), else `DROGUE_DESCENT` (warning if no chutes).
    - **DROGUE_DEPLOY:** Activates `PYRO_CHANNEL_1` for `PYRO_FIRE_DURATION`. Transitions to `DROGUE_DESCENT`.
    - **DROGUE_DESCENT:** If `MAIN_PRESENT`, transitions to `MAIN_DEPLOY` if current AGL altitude is below `g_main_deploy_altitude_m_agl`. If no main, calls `detectLanding()`.
    - **MAIN_DEPLOY:** Activates `PYRO_CHANNEL_2` for `PYRO_FIRE_DURATION`. Transitions to `MAIN_DESCENT`.
    - **MAIN_DESCENT:** Calls `detectLanding()`. If landing detected, transitions to `LANDED`.
    - **LANDED:** After `LANDED_TIMEOUT_MS`, transitions to `RECOVERY`.
    - **RECOVERY:** Activates buzzer pattern (`RECOVERY_BEEP_DURATION_MS`, `RECOVERY_SILENCE_DURATION_MS`, `RECOVERY_BEEP_FREQUENCY_HZ`).
    - **ERROR:** Manages error buzzer. Automatic recovery logic is active.
- **LED Indicators:** `setFlightStateLED()` in `flight_logic.cpp` sets NeoPixel colors based on the current state.

```cpp
// In src/flight_logic.cpp
// Uses many global variables like g_currentFlightState, g_launchAltitude, g_maxAltitudeReached,
// g_baroCalibrated, g_main_deploy_altitude_m_agl, g_pixels, etc.
// and constants from config.h and constants.h

void ProcessFlightState();
void setFlightStateLED(FlightState state); // Helper within flight_logic.cpp
```

### Arduino Framework Alignment & Setup Function Enhancement
(Corresponds to `setup()` and `handleInitialStateManagement()` in `src/TripleT_Flight_Firmware.cpp`, and `recoverFromPowerLoss()` in `src/state_management.cpp`)

- **`setup()` (in `TripleT_Flight_Firmware.cpp`):**
    - Initializes hardware: Serial, I2C, NeoPixel (`initNeoPixel`), pyro pins, servos, buzzer.
    - Calls `recoverFromPowerLoss()` to load and process any saved state from EEPROM. This sets `g_currentFlightState` based on saved data.
    - Initializes SD card (`initSDCard`), GPS (`gps_init`), sensors (`ms5611_init`, `ICM_20948_init`, `kx134_init`), Kalman filter (`kalman_init`), and guidance system (`guidance_init`).
    - Attempts to create an initial log file.
    - Prints initial sensor status.
- **`handleInitialStateManagement()` (in `TripleT_Flight_Firmware.cpp`, called once at the start of `loop()`):**
    - Performs a system health check (`isSensorSuiteHealthy()`).
    - If `g_currentFlightState` is `STARTUP` (fresh start or EEPROM indicated restart):
        - If healthy and barometer calibrated, transitions to `PAD_IDLE`.
        - If healthy but barometer not calibrated, transitions to `CALIBRATION`.
        - If unhealthy, transitions to `ERROR`.
    - If `g_currentFlightState` was `ERROR` (from EEPROM or previous issue) and system is now healthy:
        - Transitions to `PAD_IDLE` or `CALIBRATION` as appropriate.
    - If system becomes unhealthy during this initial management, transitions to `ERROR`.
- **`loop()` (in `TripleT_Flight_Firmware.cpp`):**
    - Calls `handleInitialStateManagement()` once.
    - Processes serial commands (`processCommand()`).
    - Reads sensor data periodically.
    - Updates Kalman filter and guidance system.
    - Calls `ProcessFlightState()` to manage the main state machine logic.
    - Writes log data (`WriteLogData()`).

```cpp
// In TripleT_Flight_Firmware.cpp
void setup();
void handleInitialStateManagement();
void loop();

// In src/state_management.cpp
void recoverFromPowerLoss(); // Called by setup() via handleInitialStateManagement()
```

### Testing Considerations for Additional Features
The illustrative C++ code blocks for testing (`testStateRecovery`, `testApogeeDetection`, etc.) are examples of how features *could* be tested via serial commands. The actual `processCommand` function in `src/command_processor.cpp` does not currently include these specific "test" commands. The "Testing Considerations" section below the code blocks provides more general advice on testing.

## Testing Considerations
(General advice, not specific test functions currently in `command_processor.cpp`)

1.  **Error Detection and Recovery:**
    *   Test by simulating sensor failures (e.g., disconnecting a sensor) to ensure transition to `ERROR` state.
    *   Verify automatic recovery from `ERROR` state when sensor health is restored.
    *   Test manual recovery commands (`clear_errors`, `clear_to_calibration`).
    *   Check the 5-second grace period after error clearing.
2.  **State Timeouts:** While specific state timeouts (like `ARMED_TIMEOUT_MS`) are defined in `config.h`, their direct implementation in `ProcessFlightState` for automatic transitions (other than `LANDED_TIMEOUT_MS` and `RECOVERY_TIMEOUT_MS`) is not explicitly detailed in the provided `flight_logic.cpp`. Testing would involve verifying implemented timeouts.
3.  **Self-Test Sequence:** The firmware does not currently have an explicit "daily or pre-flight self-test sequence" command. Testing individual components is done via status commands and debug flags.

## Additional Enhancements
This section reflects the status based on the `UPDATED_GAP_ANALYSIS_2025.md` and the v0.48 codebase.

1. ✅ Add error detection for sensor failures and recovery actions. (Implemented: `isSensorSuiteHealthy`, `ERROR` state, auto/manual recovery)
2. ➖ Implement watchdog timer for system stability. (Watchdog mentioned in old docs/config but not actively used/fed in main loop of current `TripleT_Flight_Firmware.cpp` or `flight_logic.cpp`. `Watchdog_t4` library was removed.)
3. ✅ Add non-volatile storage of flight state to recover from power loss. (Implemented: `FlightStateData` in EEPROM via `state_management.cpp`)
4. ✅ Consider adding redundant sensing for critical events like apogee detection. (Implemented: Multi-method apogee and landing detection in `flight_logic.cpp`)
5. ✅ Implement timeout fallbacks for critical actions (e.g., if apogee isn't detected in a reasonable time). (Implemented: `BACKUP_APOGEE_TIME_MS`)
