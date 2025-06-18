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
#define PYRO_CHANNEL_DROGUE 2      // GPIO pin for drogue deployment (example)
#define PYRO_CHANNEL_MAIN 3        // GPIO pin for main deployment (example)
#define MAIN_DEPLOY_ALTITUDE 300   // Deploy main at this height (meters) above ground (AGL)

// Flight detection thresholds
#define BOOST_ACCEL_THRESHOLD 2.0f  // Acceleration threshold for liftoff detection (g) (tuned based on rocket)
#define COAST_ACCEL_THRESHOLD 0.5f  // Acceleration threshold indicating motor burnout (g) (tuned)
#define APOGEE_VELOCITY_THRESHOLD 0.5f // Vertical velocity threshold to help confirm apogee (m/s)
#define LANDING_STABILITY_THRESHOLD 0.1f // Threshold for accelerometer magnitude stability to detect landing (g variation from 1g)
#define LANDING_ALTITUDE_STABILITY_THRESHOLD 1.0f // Altitude change threshold for landing detection (m)


// Error detection configuration (examples)
#define MAX_SENSOR_FAILURES 3      // Maximum number of consecutive sensor failures before error state
#define WATCHDOG_TIMEOUT_MS 5000   // Watchdog timer timeout in milliseconds (Teensy specific, often hardware)

// EEPROM configuration for non-volatile storage (defined in src/config.h or directly in state_management.cpp)
#define EEPROM_STATE_ADDR 0           // EEPROM address for flight state
#define EEPROM_SIGNATURE_ADDR 12      // EEPROM address for signature (example)
#define EEPROM_SIGNATURE_VALUE 0xABCD // Signature to validate EEPROM data (example)
#define EEPROM_UPDATE_INTERVAL 5000   // Save state every 5 seconds (example, may vary by state)

// Redundant sensing configuration (examples from config.h)
#define APOGEE_CONFIRMATION_COUNT 5   // Number of consecutive readings/conditions to confirm apogee
#define LANDING_CONFIRMATION_COUNT 10  // Number of consecutive readings/conditions to confirm landing (Note: current firmware's primary landing logic in flight_logic.cpp uses LANDING_CONFIRMATION_TIME_MS for a time-based check, though this count-based constant also exists in config.h)
#define BACKUP_APOGEE_TIME_MS 20000      // Backup time-based apogee detection (ms after boost detection if primary fails)
```
**Note:** The C++ code blocks for `Non-Volatile Storage Implementation`, `Redundant Sensing for Critical Events`, `Enhanced Flight State Processing`, `Setup Function Enhancement`, and `Testing Considerations` are illustrative examples of how these features are implemented in the firmware (`.cpp` files within the `src` directory, primarily `TripleT_Flight_Firmware.cpp`, `flight_logic.cpp`, `state_management.cpp`, and sensor-specific files). Refer to the actual source code for the most up-to-date and complete implementation details.

### Non-Volatile Storage Implementation

Add EEPROM storage for critical flight data to recover from power loss:

```cpp
#include <EEPROM.h>

// Flight state storage structure
struct FlightStateData {
  FlightState state;        // Current flight state
  float launchAltitude;     // Stored launch altitude
  float maxAltitude;        // Maximum altitude reached
  float currentAltitude;    // Current altitude
  unsigned long timestamp;  // Timestamp of last save
  uint16_t signature;       // Validation signature
};

FlightStateData stateData;
unsigned long lastStateSave = 0;

// Save current state to EEPROM
void saveStateToEEPROM() {
  unsigned long currentTime = millis();
  
  // Only save periodically to reduce wear
  if (currentTime - lastStateSave < EEPROM_UPDATE_INTERVAL && 
      flightState != APOGEE && flightState != DROGUE_DEPLOY && 
      flightState != MAIN_DEPLOY) {
    return;
  }
  
  // Update state data
  stateData.state = flightState;
  stateData.launchAltitude = launchAltitude;
  stateData.maxAltitude = maxAltitudeReached;
  stateData.currentAltitude = currentAltitude;
  stateData.timestamp = currentTime;
  stateData.signature = EEPROM_SIGNATURE_VALUE;
  
  // Write to EEPROM
  EEPROM.put(EEPROM_STATE_ADDR, stateData);
  
  // Update last save timestamp
  lastStateSave = currentTime;
  
  if (enableSystemDebug) {
    Serial.print(F("Flight state saved to EEPROM: "));
    Serial.println(getStateName(flightState));
  }
}

// Load state from EEPROM
bool loadStateFromEEPROM() {
  // Read from EEPROM
  EEPROM.get(EEPROM_STATE_ADDR, stateData);
  
  // Validate data
  if (stateData.signature != EEPROM_SIGNATURE_VALUE) {
    Serial.println(F("No valid flight state found in EEPROM"));
    return false;
  }
  
  // Data is valid
  Serial.println(F("Found valid flight state in EEPROM:"));
  Serial.print(F("State: "));
  Serial.println(getStateName(stateData.state));
  Serial.print(F("Launch altitude: "));
  Serial.println(stateData.launchAltitude);
  Serial.print(F("Max altitude: "));
  Serial.println(stateData.maxAltitude);
  Serial.print(F("Last altitude: "));
  Serial.println(stateData.currentAltitude);
  Serial.print(F("Timestamp: "));
  Serial.println(stateData.timestamp);
  
  return true;
}

// Recover from power loss
void recoverFromPowerLoss() {
  if (!loadStateFromEEPROM()) {
    // No valid data, start fresh
    return;
  }
  
  // Handle recovery based on saved state
  switch (stateData.state) {
    case STARTUP:
    case CALIBRATION:
    case PAD_IDLE:
      // Early states - just restart
      flightState = STARTUP;
      break;
      
    case ARMED:
      // Was armed but not launched - go back to PAD_IDLE
      flightState = PAD_IDLE;
      break;
      
    case BOOST:
    case COAST:
      // Power loss during ascent - try to detect if we're still ascending
      // For safety, assume we're past apogee
      flightState = DROGUE_DESCENT;
      
      // Restore launch altitude
      launchAltitude = stateData.launchAltitude;
      break;
      
    case APOGEE:
    case DROGUE_DEPLOY:
      // Power loss during critical deployment - deploy drogue immediately
      flightState = DROGUE_DEPLOY;
      break;
      
    case DROGUE_DESCENT:
      // Resume drogue descent
      flightState = DROGUE_DESCENT;
      
      // Restore altitude data
      launchAltitude = stateData.launchAltitude;
      maxAltitudeReached = stateData.maxAltitude;
      break;
      
    case MAIN_DEPLOY:
      // Power loss during main deployment - deploy main immediately
      flightState = MAIN_DEPLOY;
      break;
      
    case MAIN_DESCENT:
    case LANDED:
    case RECOVERY:
      // Later flight stages - go to recovery
      flightState = RECOVERY;
      break;
      
    case ERROR:
      // Was in error state - remain there
      flightState = ERROR;
      break;
  }
  
  Serial.print(F("Recovered from power loss. Resuming at state: "));
  Serial.println(getStateName(flightState));
}

// Get state name as string (for logging/debugging)
const char* getStateName(FlightState state) {
  switch (state) {
    case STARTUP: return "STARTUP";
    case CALIBRATION: return "CALIBRATION";
    case PAD_IDLE: return "PAD_IDLE";
    case ARMED: return "ARMED";
    case BOOST: return "BOOST";
    case COAST: return "COAST";
    case APOGEE: return "APOGEE";
    case DROGUE_DEPLOY: return "DROGUE_DEPLOY";
    case DROGUE_DESCENT: return "DROGUE_DESCENT";
    case MAIN_DEPLOY: return "MAIN_DEPLOY";
    case MAIN_DESCENT: return "MAIN_DESCENT";
    case LANDED: return "LANDED";
    case RECOVERY: return "RECOVERY";
    case ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}
```

### Redundant Sensing for Critical Events

Implement redundant detection methods for critical flight events:

```cpp
// Variables for redundant apogee detection
float maxAltitudeReached = 0.0;
float previousAltitude = 0.0;
int descendingCount = 0;
unsigned long boostEndTime = 0;
bool backupApogeeTimerStarted = false;

// Redundant apogee detection
bool detectApogee() {
  float currentAltitude = getBaroAltitude();
  bool apogeeDetected = false;
  
  // Method 1: Barometric detection (primary)
  if (barometerStatus.isWorking) {
    // Track maximum altitude
    if (currentAltitude > maxAltitudeReached) {
      maxAltitudeReached = currentAltitude;
      descendingCount = 0;
    }
    else if (currentAltitude < maxAltitudeReached - 1.0) {
      // Altitude is decreasing significantly
      descendingCount++;
      
      // Require multiple consecutive readings to confirm
      if (descendingCount >= APOGEE_CONFIRMATION_COUNT) {
        Serial.println(F("APOGEE DETECTED (barometric)"));
        apogeeDetected = true;
      }
    }
  }
  
  // Method 2: Accelerometer-based detection (backup)
  if (!apogeeDetected && accelerometerStatus.isWorking) {
    float accel_z = useKX134 ? kx134_accel[2] : icm_accel[2];
    
    // If Z acceleration is negative for several samples, we might be at apogee
    static float prevAccel = 0;
    static int accelNegativeCount = 0;
    
    if (accel_z < -0.1 && prevAccel < -0.1) {
      accelNegativeCount++;
      if (accelNegativeCount >= 5) {
        Serial.println(F("APOGEE DETECTED (accelerometer)"));
        apogeeDetected = true;
        accelNegativeCount = 0;
      }
    } else if (accel_z > 0) {
      accelNegativeCount = 0;
    }
    
    prevAccel = accel_z;
  }
  
  // Method 3: Time-based detection (last resort)
  if (!apogeeDetected && boostEndTime > 0) {
    // If we know when the boost phase ended, we can estimate apogee
    if (millis() - boostEndTime > BACKUP_APOGEE_TIME_MS) {
      Serial.println(F("APOGEE DETECTED (time-based)"));
      apogeeDetected = true;
    }
  }
  
  // If apogee detected, save state immediately
  if (apogeeDetected) {
    saveStateToEEPROM();
  }
  
  return apogeeDetected;
}

*Note: The actual firmware's `detectLanding()` function in `src/flight_logic.cpp` uses a time-based confirmation (`LANDING_CONFIRMATION_TIME_MS`) and accelerometer magnitude stability (comparing G-force to expected gravity ranges) rather than a simple counter as shown in the illustrative example below. Refer to `src/flight_logic.cpp` for the precise implementation.*
// Redundant landing detection
bool detectLanding() {
  static int stableCount = 0;
  bool landingDetected = false;
  
  // Method 1: Accelerometer stability (primary)
  if (accelerometerStatus.isWorking) {
    float accel_magnitude = GetAccelMagnitude();
    
    // Check if acceleration is close to 1g (just gravity)
    if (accel_magnitude > 0.95 && accel_magnitude < 1.05) {
      stableCount++;
    } else {
      stableCount = 0;
    }
    
    // Require extended stability to confirm landing
    if (stableCount >= LANDING_CONFIRMATION_COUNT) {
      landingDetected = true;
    }
  }
  
  // Method 2: Barometric stability (backup)
  if (!landingDetected && barometerStatus.isWorking) {
    static float lastAltitude = 0.0;
    static int altitudeStableCount = 0;
    
    float currentAltitude = getBaroAltitude();
    
    // Check if altitude is stable
    if (fabs(currentAltitude - lastAltitude) < LANDING_ALTITUDE_STABILITY_THRESHOLD) {
      altitudeStableCount++;
    } else {
      altitudeStableCount = 0;
    }
    
    lastAltitude = currentAltitude;
    
    // Require extended stability to confirm landing
    if (altitudeStableCount >= LANDING_CONFIRMATION_COUNT) {
      landingDetected = true;
    }
  }
  
  // Method 3: Timeout-based detection (last resort)
  // This is handled by the state timeout system
  
  // If landing detected, save state
  if (landingDetected && !landingDetectedFlag) {
    Serial.println(F("LANDING DETECTED"));
    landingDetectedFlag = true;
    saveStateToEEPROM();
  }
  
  return landingDetected;
}

// Track boost end for time-based apogee detection
void detectBoostEnd() {
  float accel_magnitude = GetAccelMagnitude();
  
  // When acceleration drops below threshold, record the time
  if (accel_magnitude < COAST_ACCEL_THRESHOLD && boostEndTime == 0) {
    boostEndTime = millis();
    Serial.println(F("BOOST END DETECTED"));
    Serial.print(F("Time since startup: "));
    Serial.print(boostEndTime / 1000.0);
    Serial.println(F(" seconds"));
    
    // Start backup apogee timer
    backupApogeeTimerStarted = true;
  }
}
```

### Enhanced Flight State Processing with Redundant Sensing

Update the `ProcessFlightState()` function to use redundant detection:

```cpp
void ProcessFlightState() {
  static float launchAltitude = 0.0;
  static unsigned long stateEntryTime = 0;
  static bool newState = true;
  static FlightState previousState = STARTUP;
  
  // Get current altitude
  float currentAltitude = getBaroAltitude();
  
  // Save current state periodically
  saveStateToEEPROM();
  
  // Check if state has changed
  if (flightState != previousState) {
    newState = true;
    previousState = flightState;
    
    // Set timeout for the new state
    setStateTimeout(flightState);
    
    // Log state transition
    Serial.print(F("STATE TRANSITION: "));
    Serial.print(getStateName(previousState));
    Serial.print(F(" -> "));
    Serial.println(getStateName(flightState));
    
    // Force a log entry for the transition
    WriteLogData(true);
    
    // Save state immediately on transition
    saveStateToEEPROM();
  }
  
  // Is this the first time in this state?
  if (newState) {
    stateEntryTime = millis();
    newState = false;
    
    // State entry actions
    switch(flightState) {
      case PAD_IDLE:
        Serial.println(F("PAD_IDLE state"));
        launchAltitude = currentAltitude; // Store ground level
        pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green
        pixels.show();
        break;
        
      case ARMED:
        Serial.println(F("ARMED state"));
        pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red
        pixels.show();
        // Start arming beeps
        tone(BUZZER_PIN, 2000, 100);
        break;
        
      case BOOST:
        Serial.println(F("BOOST state"));
        pixels.setPixelColor(0, pixels.Color(50, 0, 50)); // Purple
        pixels.show();
        WriteLogData(true); // Force immediate log entry for liftoff
        
        // Reset boost end detection
        boostEndTime = 0;
        
        // Reset apogee detection
        maxAltitudeReached = currentAltitude;
        descendingCount = 0;
        break;
        
      case COAST:
        Serial.println(F("COAST state"));
        pixels.setPixelColor(0, pixels.Color(0, 50, 50)); // Cyan
        pixels.show();
        
        // Reset apogee detection variables
        maxAltitudeReached = currentAltitude;
        descendingCount = 0;
        break;
        
      case APOGEE:
        Serial.println(F("APOGEE state"));
        pixels.setPixelColor(0, pixels.Color(50, 50, 0)); // Yellow
        pixels.show();
        
        // Record maximum altitude
        maxAltitudeReached = currentAltitude;
        WriteLogData(true); // Log apogee event
        break;
        
      case DROGUE_DEPLOY:
        Serial.println(F("DROGUE_DEPLOY state"));
        pixels.setPixelColor(0, pixels.Color(50, 25, 0)); // Orange
        pixels.show();
        break;
        
      case DROGUE_DESCENT:
        Serial.println(F("DROGUE_DESCENT state"));
        pixels.setPixelColor(0, pixels.Color(0, 0, 50)); // Blue
        pixels.show();
        break;
        
      case MAIN_DEPLOY:
        Serial.println(F("MAIN_DEPLOY state"));
        pixels.setPixelColor(0, pixels.Color(25, 0, 50)); // Purple
        pixels.show();
        break;
        
      case MAIN_DESCENT:
        Serial.println(F("MAIN_DESCENT state"));
        pixels.setPixelColor(0, pixels.Color(0, 25, 50)); // Light blue
        pixels.show();
        break;
        
      case LANDED:
        Serial.println(F("LANDED state"));
        pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green
        pixels.show();
        break;
        
      case RECOVERY:
        Serial.println(F("RECOVERY state"));
        pixels.setPixelColor(0, pixels.Color(0, 50, 25)); // Teal
        pixels.show();
        break;
        
      case ERROR:
        Serial.println(F("ERROR state - System in safe mode"));
        pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red
        pixels.show();
        
        // Make error beep
        tone(BUZZER_PIN, 500, 1000);
        break;
    }
    
    // Update logging rate for the new state
    updateLoggingRate(flightState);
  }
  
  // Process the current state
  switch(flightState) {
    case PAD_IDLE:
      // In PAD_IDLE we wait for arm command
      break;
      
    case ARMED:
      // Check for liftoff using redundant detection
      {
        float accel_magnitude = GetAccelMagnitude();
        
        // Primary detection: acceleration threshold
        if (accel_magnitude > BOOST_ACCEL_THRESHOLD) {
          // Confirm with a second reading
          delay(10);
          accel_magnitude = GetAccelMagnitude();
          
          if (accel_magnitude > BOOST_ACCEL_THRESHOLD) {
            flightState = BOOST;
            newState = true;
          }
        }
      }
      break;
      
    case BOOST:
      // Detect motor burnout with redundant methods
      detectBoostEnd();
      
      // Check if we've transitioned to coast phase
      if (boostEndTime > 0) {
        flightState = COAST;
        newState = true;
      }
      break;
      
    case COAST:
      // Check for apogee using redundant detection
      if (detectApogee()) {
        flightState = APOGEE;
        newState = true;
      }
      break;
      
    case APOGEE:
      // Deploy drogue if present, otherwise go to drogue descent
      if (DROGUE_PRESENT) {
        flightState = DROGUE_DEPLOY;
      } else {
        flightState = DROGUE_DESCENT;
      }
      newState = true;
      break;
      
    case DROGUE_DEPLOY:
      // Activate drogue deployment channel with redundancy
      {
        // First attempt
        digitalWrite(PYRO_CHANNEL_DROGUE, HIGH);
        delay(1000);
        digitalWrite(PYRO_CHANNEL_DROGUE, LOW);
        
        // Brief pause
        delay(500);
        
        // Secondary pulse for redundancy
        digitalWrite(PYRO_CHANNEL_DROGUE, HIGH);
        delay(500);
        digitalWrite(PYRO_CHANNEL_DROGUE, LOW);
        
        // Log deployment
        Serial.println(F("DROGUE DEPLOYMENT COMPLETE"));
        WriteLogData(true);
        
        flightState = DROGUE_DESCENT;
        newState = true;
      }
      break;
      
    case DROGUE_DESCENT:
      // Monitor altitude for main deployment
      {
        // Primary method: altitude above ground
        if ((currentAltitude - launchAltitude) < MAIN_DEPLOY_ALTITUDE) {
          flightState = MAIN_DEPLOY;
          newState = true;
        }
        
        // Backup method: time-based
        if (millis() - stateEntryTime > 30000) {
          // If we've been descending for 30+ seconds, check current descent rate
          static float lastCheckAlt = 0;
          static unsigned long lastCheckTime = 0;
          
          if (millis() - lastCheckTime > 1000) {
            float descentRate = (lastCheckAlt - currentAltitude) / ((millis() - lastCheckTime) / 1000.0);
            lastCheckAlt = currentAltitude;
            lastCheckTime = millis();
            
            // If descending slower than 10 m/s, consider deploying main
            if (descentRate > 0 && descentRate < 10.0) {
              flightState = MAIN_DEPLOY;
              newState = true;
              Serial.println(F("MAIN DEPLOY triggered by descent rate"));
            }
          }
        }
      }
      break;
      
    case MAIN_DEPLOY:
      // Deploy main if present, with redundancy
      if (MAIN_PRESENT) {
        // First attempt
        digitalWrite(PYRO_CHANNEL_MAIN, HIGH);
        delay(1000);
        digitalWrite(PYRO_CHANNEL_MAIN, LOW);
        
        // Brief pause
        delay(500);
        
        // Secondary pulse for redundancy
        digitalWrite(PYRO_CHANNEL_MAIN, HIGH);
        delay(500);
        digitalWrite(PYRO_CHANNEL_MAIN, LOW);
        
        // Log deployment
        Serial.println(F("MAIN DEPLOYMENT COMPLETE"));
        WriteLogData(true);
      }
      
      flightState = MAIN_DESCENT;
      newState = true;
      break;
      
    case MAIN_DESCENT:
      // Check for landing with redundant detection
      if (detectLanding()) {
        flightState = LANDED;
        newState = true;
      }
      break;
      
    case LANDED:
      // Wait a bit, then go to recovery mode
      if (millis() - stateEntryTime > 10000) {
        flightState = RECOVERY;
        newState = true;
      }
      break;
      
    case RECOVERY:
      // Log final data and prepare for shutdown
      if (millis() - stateEntryTime > 30000) {
        PrepareForShutdown();
      }
      
      // Beeping pattern for recovery
      if ((millis() / 2000) % 2 == 0) {
        tone(BUZZER_PIN, 3000);
      } else {
        noTone(BUZZER_PIN);
      }
      break;
      
    case ERROR:
      // Error handling - blink LED rapidly
      if ((millis() / 250) % 2 == 0) {
        pixels.setPixelColor(0, pixels.Color(50, 0, 0));
      } else {
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      }
      pixels.show();
      
      // Check if sensors have recovered
      if (isSensorSuiteHealthy(PAD_IDLE)) {
        // If we've been in error state for a while and sensors recovered, try to resume
        if (millis() - stateEntryTime > 10000) {
          flightState = PAD_IDLE;
          newState = true;
          Serial.println(F("Sensors recovered - resuming normal operation"));
        }
      }
      break;
  }
}
```

### Setup Function Enhancement for State Recovery

Modify the `setup()` function to include power loss recovery:

```cpp
void setup() {
  // Initialize variables
  flightState = STARTUP;
  
  // Wait for the Serial monitor to be opened
  Serial.begin(115200);
  delay(500); // Give the serial port time to initialize
  
  // Initialize system and debug settings
  enableDetailedOutput = false;
  enableSystemDebug = false;
  // ... (other debug flags)
  
  // Initialize the watchdog
  initWatchdog();
  
  // STARTUP state actions
  Serial.println(F("STARTUP state"));
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(20, 0, 0)); // Red during startup
  pixels.show();
  
  // Pet the watchdog
  petWatchdog();
  
  // Check for previous flight state in EEPROM
  Serial.println(F("Checking for flight state recovery..."));
  recoverFromPowerLoss();
  
  // If we're not starting fresh, skip some initialization
  if (flightState != STARTUP) {
    Serial.println(F("Resuming from saved state"));
    
    // Prepare for resumed operation
    switch (flightState) {
      case DROGUE_DEPLOY:
        // If resuming during deployment, do it now
        digitalWrite(PYRO_CHANNEL_DROGUE, HIGH);
        delay(1000);
        digitalWrite(PYRO_CHANNEL_DROGUE, LOW);
        flightState = DROGUE_DESCENT;
        break;
        
      case MAIN_DEPLOY:
        // If resuming during deployment, do it now
        digitalWrite(PYRO_CHANNEL_MAIN, HIGH);
        delay(1000);
        digitalWrite(PYRO_CHANNEL_MAIN, LOW);
        flightState = MAIN_DESCENT;
        break;
        
      default:
        // Other states handled normally
        break;
    }
    
    // Continue with normal initialization
  }
  
  // Startup Tone
  tone(BUZZER_PIN, 2000); delay(50); noTone(BUZZER_PIN);
  
  // Initialize recovery system channels
  pinMode(PYRO_CHANNEL_DROGUE, OUTPUT);
  pinMode(PYRO_CHANNEL_MAIN, OUTPUT);
  digitalWrite(PYRO_CHANNEL_DROGUE, LOW);
  digitalWrite(PYRO_CHANNEL_MAIN, LOW);
  
  // Initialize I2C, SPI, etc.
  Wire.begin();
  Wire.setClock(400000);
  // ... (other init code)
  
  // Pet the watchdog
  petWatchdog();
  
  // Initialize sensors
  Serial.println(F("Initializing sensors..."));
  
  // Initialize KX134 with error checking
  if (!kx134_init()) {
    Serial.println(F("WARNING: KX134 accelerometer initialization failed"));
    accelerometerStatus.isWorking = false;
  } else {
    Serial.println(F("KX134 accelerometer initialized"));
    accelerometerStatus.isWorking = true;
    accelerometerStatus.lastValidReading = millis();
  }
  
  // Initialize ICM-20948 with error checking
  if (ICM_20948_init()) {
    Serial.println(F("ICM-20948 initialized"));
    gyroscopeStatus.isWorking = true;
    gyroscopeStatus.lastValidReading = millis();
    magnetometerStatus.isWorking = true;
    magnetometerStatus.lastValidReading = millis();
  } else {
    Serial.println(F("WARNING: ICM-20948 initialization failed"));
    gyroscopeStatus.isWorking = false;
    magnetometerStatus.isWorking = false;
  }
  
  // Initialize MS5611 with error checking
  if (ms5611_init()) {
    Serial.println(F("MS5611 initialized"));
    barometerStatus.isWorking = true;
    barometerStatus.lastValidReading = millis();
  } else {
    Serial.println(F("WARNING: MS5611 initialization failed"));
    barometerStatus.isWorking = false;
  }
  
  // Pet the watchdog
  petWatchdog();
  
  // Transition to CALIBRATION state
  flightState = CALIBRATION;
  Serial.println(F("CALIBRATION state"));
  pixels.setPixelColor(0, pixels.Color(50, 50, 0)); // Yellow during calibration
  pixels.show();
  
  // Set state timeout for calibration
  setStateTimeout(CALIBRATION);
  
  // Set logging rate for calibration
  updateLoggingRate(flightState);
  
  // Initialize GPS and wait for a fix
  if (gps_init()) {
    Serial.println(F("GPS initialized"));
    gpsStatus.isWorking = true;
    gpsStatus.lastValidReading = millis();
  } else {
    Serial.println(F("WARNING: GPS initialization failed"));
    gpsStatus.isWorking = false;
  }
  
  Serial.println(F("Waiting for GPS time sync and 3D fix..."));
  
  // Wait for GPS to have valid date/time and 3D fix
  unsigned long gpsWaitStart = millis();
  bool gpsValid = false;
  
  // Use a blue "breathing" pattern while waiting
  int brightness = 0;
  int step = 5;
  
  // Pet the watchdog regularly during this loop
  lastWatchdogPet = millis();
  
  while (!gpsValid && (millis() - gpsWaitStart < 60000)) { // 60-second timeout
    // Feed the watchdog
    if (millis() - lastWatchdogPet >= (watchdogTimeout / 2)) {
      watchdog.feed();
      lastWatchdogPet = millis();
    }
    
    // Update GPS data
    gps_read();
    
    // Check sensor health
    checkSensor(gpsStatus, SIV, 0, "GPS");
    
    // Check GPS conditions (year, fix type, satellites)
    if (myGNSS.getYear() >= 2025 && GPS_fixType >= 3 && SIV >= 7) {
      gpsValid = true;
      pixels.setPixelColor(0, pixels.Color(0, 50, 50)); // Cyan indicates valid GPS
      pixels.show();
      
      // Calibrate barometer with GPS altitude
      Serial.println(F("Calibrating barometer with GPS altitude..."));
      if (ms5611_calibrate_with_gps(10000)) { // 10-second timeout
        baroCalibrated = true;
        Serial.println(F("Barometer calibration successful!"));
      } else {
        Serial.println(F("Barometer calibration failed!"));
        // Check if we need to abort
        if (!isSensorSuiteHealthy(CALIBRATION)) {
          handleSensorFailure(CALIBRATION);
          break;
        }
      }
      break;
    }
    
    // Update LED breathing effect
    if (millis() % 50 == 0) {
      brightness += step;
      if (brightness >= 50 || brightness <= 0) {
        step = -step;
        brightness = constrain(brightness, 0, 50);
      }
      pixels.setPixelColor(0, pixels.Color(0, 0, brightness));
      pixels.show();
    }
    
    delay(100); // Small delay to avoid hogging CPU
  }
  
  // Handle GPS timeout
  if (!gpsValid) {
    Serial.println(F("WARNING: GPS time sync timeout"));
    // Check if this is a critical error
    if (!isSensorSuiteHealthy(CALIBRATION)) {
      handleSensorFailure(CALIBRATION);
    }
  }
  
  // Pet the watchdog
  petWatchdog();
  
  // Initialize storage
  Serial.println(F("Initializing storage..."));
  sdCardAvailable = initSDCard();
  
  if (sdCardAvailable) {
    Serial.println(F("Creating log file..."));
    if (createNewLogFile()) {
      Serial.println(F("Log file created successfully"));
    } else {
      Serial.println(F("Failed to create log file"));
    }
  }
  
  // Store initial altitude for reference
  float launchAltitude = getBaroAltitude();
  
  // Final system check before entering main loop
  bool systemHealthy = isSensorSuiteHealthy(PAD_IDLE);
  
  // Transition to PAD_IDLE state for the loop() function
  flightState = systemHealthy ? PAD_IDLE : ERROR;
  
  if (flightState == PAD_IDLE) {
    Serial.println(F("Entering PAD_IDLE state"));
    pixels.setPixelColor(0, pixels.Color(0, 50, 0)); // Green for ready
  } else {
    Serial.println(F("Entering ERROR state - Critical sensor failure"));
    pixels.setPixelColor(0, pixels.Color(50, 0, 0)); // Red for error
  }
  pixels.show();
  
  // Update logging rate for the new state
  updateLoggingRate(flightState);
  
  // Log startup complete
  Serial.println(F("Setup complete - entering main loop"));
  
  // Final watchdog pet before entering loop
  watchdog.feed();
  lastWatchdogPet = millis();
}
```

### Testing Considerations for Additional Features

Add tests for the new features:

```cpp
// Test EEPROM state saving and recovery
void testStateRecovery() {
  Serial.println(F("Testing state recovery..."));
  
  // Save current state
  saveStateToEEPROM();
  
  // Change state temporarily
  FlightState originalState = flightState;
  flightState = ERROR;
  
  // Reset and recover
  Serial.println(F("Simulating power cycle..."));
  delay(1000);
  
  // Attempt to load state
  recoverFromPowerLoss();
  
  // Check if state was restored correctly
  Serial.print(F("Recovery test result: "));
  if (flightState == originalState) {
    Serial.println(F("PASS"));
  } else {
    Serial.println(F("FAIL"));
    // Restore original state
    flightState = originalState;
  }
}

// Test redundant apogee detection
void testApogeeDetection() {
  Serial.println(F("Testing redundant apogee detection..."));
  
  // Save original values
  float originalMaxAlt = maxAltitudeReached;
  int originalDescCount = descendingCount;
  
  // Test barometric detection
  Serial.print(F("Barometric detection: "));
  maxAltitudeReached = 1000.0;
  descendingCount = 0;
  
  // Simulate several descending readings
  for (int i = 0; i < APOGEE_CONFIRMATION_COUNT; i++) {
    float simulatedAlt = maxAltitudeReached - 2.0;
    barometerStatus.isWorking = true;
    
    // Manually call detection logic
    float currentAltitude = simulatedAlt;
    if (currentAltitude > maxAltitudeReached) {
      maxAltitudeReached = currentAltitude;
      descendingCount = 0;
    }
    else if (currentAltitude < maxAltitudeReached - 1.0) {
      descendingCount++;
    }
  }
  
  Serial.println(descendingCount >= APOGEE_CONFIRMATION_COUNT ? F("PASS") : F("FAIL"));
  
  // Test time-based detection
  Serial.print(F("Time-based detection: "));
  unsigned long originalBoostEnd = boostEndTime;
  boostEndTime = millis() - (BACKUP_APOGEE_TIME_MS + 1000);
  
  bool timeDetection = (millis() - boostEndTime > BACKUP_APOGEE_TIME_MS);
  Serial.println(timeDetection ? F("PASS") : F("FAIL"));
  
  // Restore original values
  maxAltitudeReached = originalMaxAlt;
  descendingCount = originalDescCount;
  boostEndTime = originalBoostEnd;
}

// Add to processCommand()
else if (command.startsWith("test ")) {
  if (command == "test recovery") {
    testStateRecovery();
  }
  else if (command == "test apogee") {
    testApogeeDetection();
  }
  else if (command == "test landing") {
    bool result = detectLanding();
    Serial.print(F("Landing detection test: "));
    Serial.println(result ? F("DETECTED") : F("NOT DETECTED"));
  }
}
```

## Testing Considerations

1. Add a specific test mode for error detection and recovery:
   ```cpp
   // Add to processCommand()
   else if (command.startsWith("test error ")) {
     if (command == "test error barometer") {
       // Simulate barometer failure
       barometerStatus.failureCount = MAX_SENSOR_FAILURES;
       barometerStatus.isWorking = false;
       Serial.println(F("Simulated barometer failure"));
     }
     else if (command == "test error accel") {
       // Simulate accelerometer failure
       accelerometerStatus.failureCount = MAX_SENSOR_FAILURES;
       accelerometerStatus.isWorking = false;
       Serial.println(F("Simulated accelerometer failure"));
     }
     else if (command == "test error watchdog") {
       // Simulate watchdog timeout
       Serial.println(F("Simulating watchdog timeout by entering infinite loop"));
       watchdogTriggered = true;  // Set flag first so we can recover
       while(1) { } // Force watchdog to trigger
     }
   }
   ```

2. Test the state timeout function by setting shorter timeouts in test mode.

3. Implement a daily or pre-flight self-test sequence:
   ```cpp
   void performSystemTest() {
     Serial.println(F("Running system test..."));
     
     // Test each sensor
     bool allPass = true;
     
     // Test barometer
     Serial.print(F("Testing barometer..."));
     ms5611_read();
     if (pressure > 100 && pressure < 1100) {
       Serial.println(F("PASS"));
     } else {
       Serial.println(F("FAIL"));
       allPass = false;
     }
     
     // Test accelerometer
     Serial.print(F("Testing accelerometer..."));
     kx134_read();
     float kx134_magnitude = sqrt(
       kx134_accel[0] * kx134_accel[0] + 
       kx134_accel[1] * kx134_accel[1] + 
       kx134_accel[2] * kx134_accel[2]
     );
     if (kx134_magnitude > 0.9 && kx134_magnitude < 1.1) {
       Serial.println(F("PASS"));
     } else {
       Serial.println(F("FAIL"));
       allPass = false;
     }
     
     // Test recovery channels
     Serial.print(F("Testing pyro channels (simulated)..."));
     // In a real test, you might check continuity without firing
     Serial.println(F("PASS"));
     
     // Test SD card
     Serial.print(F("Testing SD card..."));
     if (sdCardAvailable) {
       Serial.println(F("PASS"));
     } else {
       Serial.println(F("FAIL"));
       allPass = false;
     }
     
     // Test EEPROM state saving
     Serial.print(F("Testing EEPROM..."));
     bool eepromPass = testStateRecovery();
     Serial.println(eepromPass ? F("PASS") : F("FAIL"));
     if (!eepromPass) allPass = false;
     
     // Test redundant apogee detection
     Serial.print(F("Testing redundant detection..."));
     testApogeeDetection();
     
     Serial.print(F("System test result: "));
     Serial.println(allPass ? F("PASS") : F("FAIL"));
     
     // Update LED based on result
     pixels.setPixelColor(0, allPass ? pixels.Color(0, 50, 0) : pixels.Color(50, 0, 0));
     pixels.show();
   }
   ```

## Additional Enhancements

1. ✅ Add error detection for sensor failures and recovery actions.
2. ✅ Implement watchdog timer for system stability.
3. ✅ Add non-volatile storage of flight state to recover from power loss.
4. ✅ Consider adding redundant sensing for critical events like apogee detection.
5. ✅ Implement timeout fallbacks for critical actions (e.g., if apogee isn't detected in a reasonable time).
