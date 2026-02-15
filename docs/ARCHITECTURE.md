# System Architecture - TripleT Flight Firmware v0.9.0

**Last Updated:** February 15, 2026
**Target Audience:** System architects, senior developers, technical reviewers

---

## Table of Contents

1. [Overview](#overview)
2. [Data Flow Pipeline](#data-flow-pipeline)
3. [Hardware Abstraction Layer](#hardware-abstraction-layer)
4. [Sensor Subsystem](#sensor-subsystem)
5. [Flight State Machine](#flight-state-machine)
6. [Class Hierarchy](#class-hierarchy)
7. [Design Patterns](#design-patterns)
8. [Module Dependencies](#module-dependencies)
9. [Critical Paths](#critical-paths)

---

## Overview

The TripleT Flight Firmware follows a **modular, testable architecture** with clear separation of concerns:

```
┌─────────────────────────────────────────────────────────────┐
│              Application Layer                              │
│  (Flight Logic, Guidance, Commands, Logging)               │
└─────────────────────────────────────────────────────────────┘
                         │
┌─────────────────────────────────────────────────────────────┐
│              Sensor Fusion Layer                             │
│  (Kalman Filter, IMUManager, Redundancy)                   │
└─────────────────────────────────────────────────────────────┘
                         │
┌─────────────────────────────────────────────────────────────┐
│              Sensor Drivers Layer                            │
│  (ICM-20948, KX134, MS5611, GPS, etc.)                     │
└─────────────────────────────────────────────────────────────┘
                         │
┌─────────────────────────────────────────────────────────────┐
│              Hardware Abstraction Layer (HAL)                │
│  ITimer, ISerial, IGPIO, II2C, IEEPROM, ISDCard, IServo   │
└─────────────────────────────────────────────────────────────┘
                         │
┌─────────────────────────────────────────────────────────────┐
│              Hardware (Teensy 4.1, Sensors, etc.)            │
└─────────────────────────────────────────────────────────────┘
```

### Key Architectural Decisions

1. **Hardware Abstraction Layer (HAL)**
   - Pure virtual interfaces for all hardware interaction
   - Enables desktop testing without physical hardware
   - Supports easy platform migration (Teensy → STM32)

2. **Sensor Interface Pattern (IMUInterface)**
   - All motion sensors implement common interface
   - Easy sensor substitution (ICM-20948 ↔ BNO085)
   - Redundancy manager for automatic failover

3. **State Machine Core**
   - 14-state flight sequence
   - Clear state transitions with preconditions
   - Persistent state in EEPROM for power-loss recovery

4. **Dual-Sensor Redundancy**
   - Primary: ICM-20948 (±16G, gyro + accel + mag)
   - Backup: KX134 (±64G) or BNO085 (alternative)
   - Automatic switching based on health checks

---

## Data Flow Pipeline

### Main Processing Loop

```
┌──────────────────────────────────────────────────────────────┐
│                     Main Loop                                │
│              (Executes ~100 times/second)                    │
└──────────────────┬───────────────────────────────────────────┘
                   │
        ┌──────────▼─────────────┐
        │ 1. Read Sensors (20ms) │
        │ - IMU (Accel/Gyro)    │
        │ - Barometer            │
        │ - GPS                  │
        └──────────┬─────────────┘
                   │
        ┌──────────▼──────────────────┐
        │ 2. Sensor Fusion (Kalman)   │
        │ - Fuse gyro+accel          │
        │ - Generate quaternion      │
        │ - Extract Euler angles     │
        └──────────┬──────────────────┘
                   │
        ┌──────────▼─────────────────┐
        │ 3. Flight Logic Update     │
        │ - Check state transitions  │
        │ - Detect launch/apogee     │
        │ - Monitor sensor health    │
        │ - Make recovery decisions  │
        └──────────┬─────────────────┘
                   │
        ┌──────────▼──────────────────┐
        │ 4. Guidance Control        │
        │ - Calculate servo angles   │
        │ - Send servo commands      │
        │ - Apply stability control  │
        └──────────┬──────────────────┘
                   │
        ┌──────────▼─────────────────┐
        │ 5. Actuator Output        │
        │ - Update servo PWM        │
        │ - Fire pyro charges       │
        │ - Control LEDs/buzzer     │
        └──────────┬─────────────────┘
                   │
        ┌──────────▼────────────────┐
        │ 6. Data Logging           │
        │ - Format CSV line         │
        │ - Write to SD card        │
        │ - Update statistics       │
        └──────────┬────────────────┘
                   │
        ┌──────────▼──────────────────┐
        │ 7. Telemetry & Diagnostics │
        │ - Serial debug output      │
        │ - Wireless telemetry       │
        │ - Command processing       │
        └──────────┬──────────────────┘
                   │
        ┌──────────▼────────────────┐
        │ 8. Watchdog Pet           │
        │ - Feed watchdog timer     │
        │ - Monitor loop time       │
        └──────────┬────────────────┘
                   │
        └──────────┬──────────────────────┐
                   │                      │
                   └──────────┬───────────┘
                              │
                    Repeat (~10ms)
```

### Detailed Data Transformations

#### Phase 1: Sensor Reading (20-100ms)

```
Raw Sensor Data (LSBs or engineering units)
    ↓
sensor.read() ─── ICM20948Sensor::read()
    ├─ I2C read accelerometer: int16[3] → float[3] g
    ├─ I2C read gyroscope: int16[3] → float[3] rad/s
    ├─ I2C read magnetometer: int16[3] → float[3] μT
    └─ I2C read temperature: int16 → float °C
    ↓
Sensor values cached in globals:
    icm_accel[], icm_gyro[], icm_mag[], icm_temp
```

#### Phase 2: Kalman Filtering

```
Accelerometer (accel_x, accel_y, accel_z)
Gyroscope (gyro_x, gyro_y, gyro_z)
    ↓
KalmanFilter::update()
    ├─ Predict: θ' = θ + ω·Δt
    ├─ Measure: Compare accel magnitude
    ├─ Correct: Blend gyro & accel
    └─ Output: Quaternion (qw, qx, qy, qz)
    ↓
Orientation Quaternion
    ↓
Convert to Euler Angles (optional):
    roll  = atan2(2(qw·qx + qy·qz), 1 - 2(qx² + qy²))
    pitch = asin(2(qw·qy - qz·qx))
    yaw   = atan2(2(qw·qz + qx·qy), 1 - 2(qy² + qz²))
```

#### Phase 3: Flight Logic Decision

```
Current State + Sensor Input
    ↓
updateFlightState()
    ├─ ARMED → BOOST: if accel > 2.0g
    ├─ BOOST → COAST: if accel < 0.5g
    ├─ COAST → APOGEE: if detectApogee()
    │   └─ Check: (baro descent) && (accel descent) && (GPS descent) || timeout
    ├─ APOGEE → DROGUE_DEPLOY: immediate
    ├─ DROGUE_DESCENT → MAIN_DEPLOY: if altitude < 300m AGL
    └─ ... other transitions ...
    ↓
Potential State Change + Actions
    └─ Log state change
    └─ Fire pyro if needed
    └─ Update recovery system
```

#### Phase 4: Guidance Control

```
Current State + Orientation + Sensor Data
    ↓
guidance_controller.update()
    ├─ Check if COAST state (guidance active)
    ├─ Calculate desired vs. actual attitude
    ├─ PID control loop:
    │   ├─ error = desired_pitch - actual_pitch
    │   ├─ output = Kp·error + Ki·integral + Kd·derivative
    │   └─ Clamp to ±20° servo deflection
    └─ Send servo command (0-180°)
    ↓
Servo Actuator Position
```

#### Phase 5: Data Logging

```
All sensor data + flight state + decisions
    ↓
formatLogEntry()
    ├─ Timestamp
    ├─ Flight state
    ├─ Acceleration (XYZ)
    ├─ Gyroscope (XYZ)
    ├─ Altitude
    ├─ Velocity
    ├─ Orientation (quaternion or Euler)
    ├─ Temperature
    ├─ GPS (lat, lon, altitude)
    ├─ Servo position
    └─ Event markers (launch, apogee, deploy)
    ↓
CSV Line (62 fields)
    ↓
Write to SD Card
```

---

## Hardware Abstraction Layer

### Interface Hierarchy

```
      IHALProvider (virtual base)
           │
    ┌──────┴──────┬──────────┬─────────┬────────┬─────────┬────────┐
    │             │          │         │        │         │        │
  ITimer      ISerial      IGPIO     II2C   IEEPROM   ISDCard   IServo
    │             │          │         │        │         │        │
    └────────────┬──────────┴─────────┴────────┴─────────┴────────┘
                 │
        ┌────────▼──────────────┐
        │  Concrete HAL Class    │
        │  (TeensyHAL or MockHAL)
        └───────────────────────┘
```

### Interface Implementations

```
Interface Method     │ Production (Teensy)  │ Testing (Mock)
─────────────────────┼──────────────────────┼──────────────────
millis()             │ Read ARM SysTick     │ Simulate clock
delay(ms)            │ delayMicroseconds()  │ Track wall clock
pinMode(pin, mode)   │ GPIO_PinConfigure()  │ Record in map
digitalWrite()       │ GPIO_WriteBit()      │ Mock pin state
write(i2c)           │ I2C Send via Wire    │ Return mock data
read(eeprom)         │ EEPROM_ReadByte()    │ Simulated storage
openFile(filename)   │ SD card FatFs        │ In-memory buffer
```

### HAL Factory Pattern

```cpp
// Compile-time selection
#ifdef PLATFORM_TEENSY41
  #define GET_HAL() TeensyHAL::getInstance()
#else
  #define GET_HAL() MockHAL::getInstance()
#endif

// Usage in application code
ITimer* timer = GET_HAL()->getTimer();
uint32_t now = timer->millis();
```

---

## Sensor Subsystem

### Sensor Hierarchy

```
          IMUInterface (Pure Virtual)
                 │
     ┌───────────┼───────────┐
     │           │           │
ICM20948Sensor  KX134Sensor  BNO085Sensor
     │           │           │
     └───────────┼───────────┘
                 │
          IMUManager
          (Redundancy & Failover)
```

### Dual-Sensor Architecture

```
Primary Sensor (ICM-20948)
  ├─ ±16G accelerometer
  ├─ ±2000 dps gyroscope
  ├─ Magnetometer
  └─ Temperature sensor
      │
      ├─ Healthy? Use primary data
      │
      └─ Failed? Switch to backup

Backup Sensor (KX134 or BNO085)
  ├─ ±64G high-G accelerometer (KX134)
  │  OR
  ├─ ±16G with built-in fusion (BNO085)
  ├─ No gyroscope (KX134)
  │  OR
  ├─ Quaternion output (BNO085)
  └─ Fallback data source
```

### Sensor Manager Failover Logic

```cpp
IMUManager::read() {
  if (primary_sensor && primary_healthy) {
    bool ok = primary_sensor->read();
    if (ok) {
      return true;  // Use primary
    }
    primary_healthy = false;  // Mark failed
  }

  if (backup_sensor && backup_healthy) {
    bool ok = backup_sensor->read();
    if (ok) {
      return true;  // Use backup
    }
    backup_healthy = false;  // Mark failed
  }

  return false;  // No sensors available
}

// Data access always checks health
float IMUManager::getAccelX() {
  if (primary_sensor && primary_healthy) {
    return primary_sensor->getAccelX();
  }
  if (backup_sensor && backup_healthy) {
    return backup_sensor->getAccelX();
  }
  return 0.0f;
}
```

### Sensor Health Monitoring

```
Every Main Loop:
  ├─ Call sensor.read()
  ├─ Check return value
  ├─ Validate output ranges:
  │   ├─ Acceleration: ±100 m/s² (±10g)
  │   ├─ Gyroscope: ±360 deg/s
  │   └─ Temperature: -50°C to +150°C
  ├─ Check data freshness
  │   └─ If stale for > 1 second, mark unhealthy
  └─ If unhealthy:
      ├─ Increment failure counter
      ├─ Log error event
      └─ If counter > 3:
          └─ Enter ERROR state
```

---

## Flight State Machine

### State Diagram with Transitions

```
                    ┌─────────────┐
                    │   STARTUP   │
                    └────────┬────┘
                             │ Sensors OK
                    ┌────────▼────────┐
                    │  CALIBRATION    │
                    └────────┬────────┘
                             │ Calibration complete
                    ┌────────▼────────┐
                    │    PAD_IDLE     │
                    └────────┬────────┘
                             │ Arm command
                    ┌────────▼────────┐
                    │     ARMED       │
                    └────────┬────────┘
                             │ Accel > 2.0g
                    ┌────────▼────────┐
                    │     BOOST       │
                    └────────┬────────┘
                             │ Accel < 0.5g
                    ┌────────▼────────┐
                    │     COAST       │
                    └────────┬────────┘
                             │ Apogee detected
                    ┌────────▼────────┐
                    │    APOGEE       │
                    └────────┬────────┘
                             │ Immediate
              ┌──────────────▼──────────────┐
              │    DROGUE_DEPLOY           │
              │    (Fire pyro charge)      │
              └──────────────┬──────────────┘
                             │ Drogue deployed
                    ┌────────▼────────┐
                    │ DROGUE_DESCENT  │
                    └────────┬────────┘
                             │ Alt < 300m AGL
              ┌──────────────▼──────────────┐
              │    MAIN_DEPLOY             │
              │    (Fire main pyro)        │
              └──────────────┬──────────────┘
                             │ Main deployed
                    ┌────────▼────────┐
                    │  MAIN_DESCENT   │
                    └────────┬────────┘
                             │ Velocity < 1 m/s
                    ┌────────▼────────┐
                    │    LANDED       │
                    └────────┬────────┘
                             │ 10s timeout
                    ┌────────▼────────┐
                    │   RECOVERY      │
                    └────────┬────────┘
                             │ Beacon active
                             │ GPS TX
                             └─ Manual power-off
```

### State Transition Preconditions

```cpp
enum class TransitionResult {
  OK,           // Transition allowed
  BLOCKED,      // Condition not met
  ERROR         // Safety violation
};

// Example: BOOST → COAST transition
TransitionResult canTransitionBoostToCoast() {
  // 1. Preconditions
  if (current_state != BOOST) return TransitionResult::BLOCKED;

  // 2. Safety checks
  if (!sensor_healthy) return TransitionResult::ERROR;

  // 3. Physical conditions
  if (accel_magnitude > COAST_ACCEL_THRESHOLD) {
    return TransitionResult::BLOCKED;
  }

  // 4. All checks passed
  return TransitionResult::OK;
}
```

---

## Class Hierarchy

### Core Classes

```cpp
// Sensor classes
IMUInterface
  ├─ ICM20948Sensor
  ├─ KX134Sensor
  ├─ BNO085Sensor
  └─ MockIMUSensor (testing)

// Management
IMUManager                // Dual-sensor redundancy
SensorFactory             // Factory pattern

// Flight logic
FlightLogic               // State machine
GuidanceControl           // Servo control
KalmanFilter              // Orientation fusion

// Data
LogData                   // CSV record structure
SystemStatusContext       // Global state context

// Communication
CommandProcessor          // Serial commands
TelemetryManager          // Wireless data
```

### Data Structures

```cpp
// Core flight data
struct LogData {
  uint32_t timestamp;
  FlightState state;
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float altitude;
  float velocity;
  float quat_w, quat_x, quat_y, quat_z;
  // ... 56 more fields ...
};

// State persistence
struct FlightStateData {
  FlightState state;
  uint32_t timestamp;
  float altitude;
  uint16_t signature;
};

// System context (global state)
struct SystemStatusContext {
  FlightLogic* flight_logic;
  IMUManager* imu_manager;
  ISDCard* sd_card;
  ISerial* serial;
  // ... other system pointers ...
};
```

---

## Design Patterns

### 1. Factory Pattern (Sensor Creation)

```cpp
// Create sensors at compile-time
class SensorFactory {
  static IMUInterface* createPrimarySensor() {
#ifdef USE_BNO055_VARIANT
    return new BNO055Sensor();
#else
    return new ICM20948Sensor();
#endif
  }
};
```

**Benefits:**
- Easy sensor substitution without code changes
- Clear configuration via compile flags
- Symmetric primary/backup creation

### 2. Adapter Pattern (IMUInterface)

```cpp
// Wrap legacy driver with interface
class ICM20948Sensor : public IMUInterface {
  bool read() override {
    ICM_20948_read();  // Call legacy function
    return true;       // Adapt to interface
  }
};
```

**Benefits:**
- Use existing drivers without rewrite
- Convert old API to new interface
- Enable sensor swaps

### 3. Singleton Pattern (HAL)

```cpp
class TeensyHAL {
private:
  static TeensyHAL* instance;
  TeensyHAL() { }

public:
  static TeensyHAL* getInstance() {
    if (!instance) {
      instance = new TeensyHAL();
    }
    return instance;
  }
};
```

**Benefits:**
- Single global HAL instance
- Lazy initialization
- Easy testing (swap implementation)

### 4. Observer Pattern (Events)

```cpp
// Flight logic publishes state changes
class FlightLogic {
  void transitionState(FlightState new_state) {
    // Notify listeners
    for (Observer* obs : observers) {
      obs->onStateChange(new_state);
    }
  }
};
```

**Benefits:**
- Decoupled state change handling
- Multiple subscribers (logging, GUI, etc.)
- Easy to add new listeners

### 5. State Machine Pattern (Core Flight)

```cpp
class FlightLogic {
  void update() {
    switch (current_state) {
      case ARMED:
        if (isLaunchDetected()) {
          setState(BOOST);
        }
        break;
      case BOOST:
        if (isMotorBurnout()) {
          setState(COAST);
        }
        break;
      // ...
    }
  }
};
```

**Benefits:**
- Clear state transitions
- Easy to add new states
- Self-documenting code

---

## Module Dependencies

### Dependency Graph

```
TripleT_Flight_Firmware.cpp (main)
  ├─ flight_logic.cpp
  │   ├─ data_structures.h
  │   ├─ config.h
  │   ├─ sensors/imu_manager.h
  │   ├─ ms5611_functions.h
  │   ├─ gps_functions.h
  │   ├─ state_management.cpp
  │   └─ command_processor.cpp
  │
  ├─ guidance_control.cpp
  │   ├─ sensors/imu_manager.h
  │   ├─ kalman_filter.h
  │   └─ hal/hal_interfaces.h
  │
  ├─ sensors/sensor_factory.h
  │   ├─ sensors/imu_manager.h
  │   ├─ sensors/icm20948_sensor.h
  │   ├─ sensors/kx134_sensor.h
  │   └─ sensors/bno085_sensor.h
  │
  ├─ kalman_filter.cpp
  │   └─ data_structures.h
  │
  └─ log_format_definition.cpp
      └─ data_structures.h
```

### Acyclic Dependency Principle (ADP)

All dependencies flow **upward** toward application layer:

```
Application Layer (flight_logic, guidance_control)
         ↑
Sensor Fusion (kalman_filter, imu_manager)
         ↑
Sensor Drivers (icm20948_sensor, ms5611_functions)
         ↑
HAL (hal_interfaces, hal_implementations)
         ↑
Hardware (Teensy, I2C, SPI)
```

**No circular dependencies** - enables:
- Easy testing via HAL mocks
- Clear module boundaries
- Reduced coupling

---

## Critical Paths

### Critical Path 1: Launch Detection

```
Sensor Read (ICM-20948)
  └─ accel_z updated (50 ms)
      └─ flight_logic.update()
          └─ if (accel > BOOST_THRESHOLD) {
              └─ setState(BOOST)
                  └─ Log launch event
                      └─ Enable guidance
                          └─ Total: < 100ms
```

**Timing:** Must detect launch within 100ms
**Redundancy:** Two accel confirms before boost
**Backup:** Barometer detects altitude gain

### Critical Path 2: Apogee Detection

```
Barometer Read (MS5611)
  └─ altitude updated (100ms)
      └─ detectApogee()
          ├─ baro_descent? (altitude_n < altitude_n-1)
          ├─ accel_descent? (accel_z < threshold)
          ├─ gps_descent? (gps_alt < gps_alt_prev)
          └─ if (2 of 3 vote YES) {
              └─ setState(APOGEE)
                  └─ Log apogee event
                      └─ Fire drogue charge
                          └─ Total: < 500ms
```

**Timing:** Must trigger drogue within 500ms of apogee
**Redundancy:** 2-of-3 voting (baro + accel + GPS)
**Backup:** Timeout failsafe (20s after motor burnout)

### Critical Path 3: Main Deployment

```
Barometer Read
  └─ altitude updated (100ms)
      └─ in DROGUE_DESCENT state?
          └─ if (altitude_AGL < 300m) {
              └─ setState(MAIN_DEPLOY)
                  └─ Fire main charge
                      └─ Total: < 200ms
```

**Timing:** Must deploy main before ground impact
**Redundancy:** GPS altitude as backup
**Backup:** Manual deployment command

---

## Performance Characteristics

### Timing Budget (Main Loop ~10ms)

```
Task                    │ Duration  │ Frequency
────────────────────────┼───────────┼─────────────
Sensor Read             │ 2-5ms     │ Every loop
Kalman Filter           │ 1-2ms     │ Every loop
Flight Logic Update     │ 2-3ms     │ Every loop
Guidance Control        │ 1-2ms     │ Every loop (COAST only)
Logging                 │ 0-1ms     │ Every loop
Serial Processing       │ 0-1ms     │ Only if available
────────────────────────┼───────────┼─────────────
TOTAL                   │ 7-15ms    │ Every loop
```

### Memory Usage (Teensy 4.1: 512KB RAM)

```
Component               │ Size (bytes)
────────────────────────┼──────────────
Stack                   │ 10KB (estimated)
Global Variables        │ 50KB (sensor buffers, state)
Sensor Data Buffer      │ 15KB
Log Buffer              │ 10KB
Dynamic Allocation      │ 20KB (mostly drivers)
Free (headroom)         │ 395KB
```

### Flash Usage (Teensy 4.1: 1MB Flash)

```
Component               │ Size (bytes)
────────────────────────┼──────────────
Bootloader              │ 64KB
Firmware Code           │ 200KB
Data (config, strings)  │ 50KB
Free                    │ 686KB
```

---

## Extensibility Points

### How to Add New Subsystems

1. **Define interface** (e.g., `ITelemetry`)
2. **Implement concrete classes** (e.g., `XBeeRadio`)
3. **Register with factory** (optional)
4. **Inject via HAL** (dependency injection)
5. **Call from main loop**

Example:

```cpp
// 1. Define interface
class ITelemetry {
  virtual void send(const char* data) = 0;
};

// 2. Implement
class XBeeRadio : public ITelemetry {
  void send(const char* data) override {
    // Send via UART
  }
};

// 3. Use in flight logic
void loop() {
  if (current_state == COAST) {
    telemetry->send("BOOST_TO_COAST");
  }
}
```

---

## References

- **Design Patterns:** Gang of Four (GoF) patterns used
- **HAL Architecture:** MISRA C++ embedded guidelines
- **State Machine:** UML state diagrams
- **Testing:** Mock Object Pattern

---

**For implementation details, see:**
- `DEVELOPER_GUIDE.md` - Code patterns and examples
- `src/hal/hal_interfaces.h` - HAL interface definitions
- `src/sensors/imu_interface.h` - Sensor interface definition
- `src/flight_logic.cpp` - State machine implementation
