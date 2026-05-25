# Developer Guide - TripleT Flight Firmware v0.9.0

**Last Updated:** February 15, 2026
**Target Audience:** Firmware developers, hardware engineers, system integrators

---

## Table of Contents

1. [Getting Started](#getting-started)
2. [Project Structure](#project-structure)
3. [Hardware Abstraction Layer (HAL)](#hardware-abstraction-layer-hal)
4. [Adding New Sensors](#adding-new-sensors)
5. [Testing Strategy](#testing-strategy)
6. [Code Organization](#code-organization)
7. [Flight State Machine](#flight-state-machine)
8. [Common Development Tasks](#common-development-tasks)
9. [Best Practices](#best-practices)

---

## Getting Started

### Prerequisites

- **Teensy 4.1** microcontroller board
- **PlatformIO** CLI or IDE (`pio` command)
- **Git** for version control
- **Python 3.7+** (for PlatformIO)
- Sensors: ICM-20948, MS5611 barometer, u-blox GPS
- Optional: KX134 high-G accelerometer

### Initial Setup

1. **Clone Repository**
   ```bash
   git clone <repository-url>
   cd TripleT-Flight-Firmware
   ```

2. **Install Dependencies**
   ```bash
   pio lib update
   ```

3. **Verify Build**
   ```bash
   pio run -e teensy41
   ```

4. **Upload to Teensy**
   ```bash
   pio run -t upload -e teensy41
   ```

5. **Monitor Serial Output**
   ```bash
   pio device monitor --baud 115200
   ```

---

## Project Structure

```
TripleT-Flight-Firmware/
├── src/
│   ├── hal/                          # Hardware Abstraction Layer
│   │   ├── hal_interfaces.h          # Pure virtual interfaces
│   │   ├── hal_factory.h             # Dependency injection
│   │   ├── teensy_hal.h              # Real implementations
│   │   ├── mock_hal.h                # Testing implementations
│   │   └── hal_config.cpp
│   ├── sensors/                      # Sensor drivers & managers
│   │   ├── imu_interface.h           # IMU base interface
│   │   ├── imu_manager.h             # Dual-sensor redundancy
│   │   ├── icm20948_sensor.h         # ICM-20948 adapter
│   │   ├── kx134_sensor.h            # KX134 adapter
│   │   ├── bno085_sensor.h           # BNO085 adapter (alternative)
│   │   └── sensor_factory.h          # Factory pattern
│   ├── config.h                      # Central configuration
│   ├── data_structures.h             # Shared types & structs
│   ├── debug_flags.h                 # Debug output control
│   ├── flight_logic.cpp/h            # Core flight state machine
│   ├── state_management.cpp/h        # State persistence
│   ├── guidance_control.cpp/h        # Servo control & stability
│   ├── icm_20948_functions.cpp/h     # IMU sensor driver
│   ├── kx134_functions.cpp/h         # High-G accelerometer
│   ├── ms5611_functions.cpp/h        # Barometer driver
│   ├── gps_functions.cpp/h           # GPS receiver
│   ├── kalman_filter.cpp/h           # Orientation fusion
│   ├── command_processor.cpp/h       # Serial commands
│   ├── log_format_definition.cpp/h   # Data logging
│   └── TripleT_Flight_Firmware.cpp   # Main entry point
├── test/
│   ├── unit/                         # Unit tests
│   │   ├── test_state_machine.cpp
│   │   ├── test_apogee_detection.cpp
│   │   └── test_math_functions.cpp
│   ├── mocks/
│   │   └── mock_sensors.h            # Mock implementations
│   └── fixtures/                     # Test data
├── docs/
│   ├── DEVELOPER_GUIDE.md            # This file
│   ├── ARCHITECTURE.md               # System design
│   ├── SAFETY.md                     # Safety procedures
│   ├── SENSOR_EVALUATION.md          # Hardware comparison
│   ├── FLIGHT_STATE_MACHINE.md       # State definitions
│   └── CONFIGURATION.md              # Parameter guide
└── platformio.ini                    # PlatformIO configuration

```

---

## Hardware Abstraction Layer (HAL)

The HAL decouples hardware-specific code from business logic, enabling:
- **Cross-platform development** (desktop testing without Teensy)
- **Easy hardware swaps** (sensor alternatives)
- **Comprehensive testing** with mock implementations

### HAL Interfaces

The `src/hal/hal_interfaces.h` defines 8 pure virtual interfaces:

#### 1. ITimer - Timing & Delays

```cpp
class ITimer {
  virtual uint32_t millis() = 0;              // Milliseconds since boot
  virtual uint32_t micros() = 0;              // Microseconds since boot
  virtual void delay(uint32_t ms) = 0;        // Blocking millisecond delay
  virtual void delayMicroseconds(uint16_t us) = 0;  // Blocking microsecond delay
};
```

**Usage Example:**
```cpp
// Don't do this:
uint32_t start = millis();

// Do this instead:
uint32_t start = hal->timer()->millis();
```

#### 2. ISerial - Serial Communication

```cpp
class ISerial {
  virtual void print(const char* str) = 0;    // Print without newline
  virtual void println(const char* str) = 0;  // Print with newline
  virtual void write(uint8_t byte) = 0;       // Write single byte
  virtual int read() = 0;                     // Read byte (-1 if none)
  virtual bool available() = 0;               // Check if data available
  virtual int peek() = 0;                     // Peek at next byte
  virtual void flush() = 0;                   // Flush output buffer
};
```

**Usage Example:**
```cpp
// Print diagnostic info
hal->serial()->println("Sensor initialized");
if (hal->serial()->available()) {
  char cmd = hal->serial()->read();
}
```

#### 3. IGPIO - Digital I/O

```cpp
class IGPIO {
  virtual void pinMode(uint8_t pin, uint8_t mode) = 0;
  virtual void digitalWrite(uint8_t pin, uint8_t value) = 0;
  virtual int digitalRead(uint8_t pin) = 0;
  virtual void analogWrite(uint8_t pin, uint8_t value) = 0;  // PWM
  virtual int analogRead(uint8_t pin) = 0;
};
```

**Usage Example:**
```cpp
// Fire pyro charge
hal->gpio()->digitalWrite(PYRO_CHANNEL_1, HIGH);
hal->timer()->delay(500);
hal->gpio()->digitalWrite(PYRO_CHANNEL_1, LOW);
```

#### 4. II2C - I2C Bus Communication

```cpp
class II2C {
  virtual bool begin(uint8_t sda_pin = 0, uint8_t scl_pin = 0) = 0;
  virtual void beginTransmission(uint8_t address) = 0;
  virtual size_t write(const uint8_t* data, size_t length) = 0;
  virtual size_t write(uint8_t byte) = 0;
  virtual uint8_t endTransmission(bool sendStop = true) = 0;
  virtual size_t requestFrom(uint8_t address, size_t quantity, bool sendStop = true) = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  virtual bool available() = 0;
};
```

**Usage Example:**
```cpp
// Read from I2C sensor (register 0x3B, 6 bytes)
hal->i2c()->beginTransmission(0x68);
hal->i2c()->write(0x3B);
hal->i2c()->endTransmission();

hal->i2c()->requestFrom(0x68, 6);
uint8_t data[6];
for (int i = 0; i < 6 && hal->i2c()->available(); i++) {
  data[i] = hal->i2c()->read();
}
```

#### 5. IEEPROM - Persistent Storage

```cpp
class IEEPROM {
  virtual bool begin() = 0;
  virtual uint8_t read(uint16_t address) = 0;
  virtual void write(uint16_t address, uint8_t value) = 0;
  virtual void write(uint16_t address, const uint8_t* data, size_t length) = 0;
  virtual void commit() = 0;
  virtual size_t length() = 0;
};
```

**Usage Example:**
```cpp
// Save flight state to EEPROM
FlightStateData state_data;
state_data.state = APOGEE;
state_data.timestamp = hal->timer()->millis();
hal->eeprom()->write(EEPROM_STATE_ADDR, (uint8_t*)&state_data, sizeof(state_data));
hal->eeprom()->commit();
```

#### 6. ISDCard - SD Card Logging

```cpp
class ISDCard {
  virtual bool begin(uint8_t cs_pin) = 0;
  virtual bool openFile(const char* filename) = 0;
  virtual bool writeData(const uint8_t* data, size_t length) = 0;
  virtual bool closeFile() = 0;
  virtual bool isHealthy() = 0;
  virtual uint32_t getLastError() = 0;
};
```

**Usage Example:**
```cpp
// Write flight data to CSV
if (hal->sdcard()->openFile("flight.csv")) {
  const char* header = "timestamp,accel_x,accel_y,accel_z\n";
  hal->sdcard()->writeData((uint8_t*)header, strlen(header));
  // ... write data ...
  hal->sdcard()->closeFile();
}
```

#### 7. IServo - Servo Control (PWM)

```cpp
class IServo {
  virtual bool attach(uint8_t pin, uint16_t min_us = 1000, uint16_t max_us = 2000) = 0;
  virtual void detach(uint8_t pin) = 0;
  virtual void writeAngle(uint8_t pin, uint8_t angle) = 0;
  virtual void writeMicroseconds(uint8_t pin, uint16_t us) = 0;
  virtual uint8_t readAngle(uint8_t pin) = 0;
  virtual uint16_t readMicroseconds(uint8_t pin) = 0;
};
```

**Usage Example:**
```cpp
// Control canard servo for guidance
hal->servo()->attach(CANARD_PIN, 1000, 2000);
hal->servo()->writeAngle(CANARD_PIN, 90);  // Neutral
hal->servo()->writeAngle(CANARD_PIN, 120); // 30° deflection
```

#### 8. IWatchdog - Reset Protection

```cpp
class IWatchdog {
  virtual bool begin(uint32_t timeout_ms) = 0;
  virtual void feed() = 0;          // Pet the watchdog
  virtual void reset() = 0;         // Force watchdog reset
  virtual uint32_t getTimeout() = 0;
  virtual bool isWatchdogReset() = 0;  // Detect watchdog recovery
};
```

**Usage Example:**
```cpp
// Initialize watchdog with 5-second timeout
hal->watchdog()->begin(5000);

// In main loop, feed watchdog regularly
if (flight_ok) {
  hal->watchdog()->feed();
} else {
  // Let watchdog reset the system
}
```

### HAL Factory Pattern

Use dependency injection to select HAL implementation at runtime:

```cpp
#ifdef PLATFORM_TEENSY
  HAL* hal = new TeensyHAL();
#else
  HAL* hal = new MockHAL();  // For desktop testing
#endif
```

---

## Adding New Sensors

### Overview: IMUInterface Pattern

All motion sensors implement `IMUInterface`:

```cpp
class IMUInterface {
public:
  virtual bool begin() = 0;                           // Initialize
  virtual bool isHealthy() = 0;                       // Health check
  virtual const char* getErrorMessage() = 0;         // Error info
  virtual bool read() = 0;                           // Read latest data

  // Acceleration (m/s²)
  virtual float getAccelX() = 0;
  virtual float getAccelY() = 0;
  virtual float getAccelZ() = 0;
  virtual float getAccelMagnitude() = 0;

  // Rotation rate (deg/s)
  virtual float getGyroX() = 0;
  virtual float getGyroY() = 0;
  virtual float getGyroZ() = 0;

  // Magnetic field (μT)
  virtual float getMagX() = 0;
  virtual float getMagY() = 0;
  virtual float getMagZ() = 0;

  // Orientation (quaternion)
  virtual void getQuaternion(float& qw, float& qx, float& qy, float& qz) = 0;

  // Temperature (°C)
  virtual float getTemperature() = 0;

  // Configuration
  virtual void setAccelScale(uint16_t g_range) = 0;
  virtual void setGyroScale(uint16_t dps_range) = 0;
  virtual bool calibrate() = 0;
};
```

### Step-by-Step: Adding BNO055 Sensor

This example demonstrates adding a new sensor by following the established patterns.

#### Step 1: Create Sensor Adapter Header

Create `/mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware/src/sensors/bno055_sensor.h`:

```cpp
#ifndef BNO055_SENSOR_H
#define BNO055_SENSOR_H

#include "imu_interface.h"
#include <Adafruit_BNO055.h>

class BNO055Sensor : public IMUInterface {
private:
  Adafruit_BNO055 bno;
  bool healthy;
  char error_message[64];

public:
  BNO055Sensor() : bno(55, 0x28), healthy(false) {
    memset(error_message, 0, sizeof(error_message));
  }

  // Initialize the sensor
  bool begin() override {
    if (!bno.begin()) {
      snprintf(error_message, sizeof(error_message),
               "BNO055: Initialization failed");
      healthy = false;
      return false;
    }

    bno.setExtCrystalUse(true);
    healthy = true;
    return true;
  }

  // Check sensor health
  bool isHealthy() override {
    if (!healthy) return false;

    uint8_t system_status = bno.getSystemStatus();
    if (system_status != 0) {  // 0 = Idle, 1 = System Error
      snprintf(error_message, sizeof(error_message),
               "BNO055: System status=%d", system_status);
      healthy = false;
      return false;
    }

    return true;
  }

  const char* getErrorMessage() override {
    return error_message;
  }

  // Read latest sensor data
  bool read() override {
    if (!isHealthy()) return false;
    // BNO055 updates continuously, just verify we can access data
    bno.getEvent(NULL);  // Trigger internal update
    return true;
  }

  // Acceleration in m/s²
  float getAccelX() override {
    if (!isHealthy()) return 0.0f;
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    return accel.x();
  }

  float getAccelY() override {
    if (!isHealthy()) return 0.0f;
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    return accel.y();
  }

  float getAccelZ() override {
    if (!isHealthy()) return 0.0f;
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    return accel.z();
  }

  float getAccelMagnitude() override {
    if (!isHealthy()) return 0.0f;
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    return accel.magnitude();
  }

  // Rotation in deg/s
  float getGyroX() override {
    if (!isHealthy()) return 0.0f;
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    return gyro.x();
  }

  float getGyroY() override {
    if (!isHealthy()) return 0.0f;
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    return gyro.y();
  }

  float getGyroZ() override {
    if (!isHealthy()) return 0.0f;
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    return gyro.z();
  }

  // Magnetic field in μT
  float getMagX() override {
    if (!isHealthy()) return 0.0f;
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    return mag.x();
  }

  float getMagY() override {
    if (!isHealthy()) return 0.0f;
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    return mag.y();
  }

  float getMagZ() override {
    if (!isHealthy()) return 0.0f;
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    return mag.z();
  }

  // Quaternion (BNO055 provides directly!)
  void getQuaternion(float& qw, float& qx, float& qy, float& qz) override {
    if (!isHealthy()) {
      qw = 1.0f; qx = 0.0f; qy = 0.0f; qz = 0.0f;
      return;
    }

    imu::Quaternion q = bno.getQuat();
    qw = q.w();
    qx = q.x();
    qy = q.y();
    qz = q.z();
  }

  float getTemperature() override {
    if (!isHealthy()) return 0.0f;
    return bno.getTemp();
  }

  void setAccelScale(uint16_t g_range) override {
    // BNO055 has fixed ranges, not user-configurable
  }

  void setGyroScale(uint16_t dps_range) override {
    // BNO055 has fixed ranges, not user-configurable
  }

  bool calibrate() override {
    // BNO055 auto-calibrates, just trigger a full calibration
    return bno.beginConfig();
  }
};

#endif // BNO055_SENSOR_H
```

#### Step 2: Update Sensor Factory

Edit `/mnt/GAMES_SSD/matt/Code/TripleT-Flight-Firmware/src/sensors/sensor_factory.h`:

```cpp
#ifndef SENSOR_FACTORY_H
#define SENSOR_FACTORY_H

#include "imu_interface.h"
#include "icm20948_sensor.h"
#include "kx134_sensor.h"
#include "bno085_sensor.h"
#include "bno055_sensor.h"  // NEW
#include "imu_manager.h"

class SensorFactory {
public:
  // Create primary IMU sensor
  static IMUInterface* createPrimarySensor() {
#ifdef USE_BNO055_PRIMARY
    return new BNO055Sensor();  // NEW OPTION
#elif defined(USE_BNO085_VARIANT)
    return new BNO085Sensor();
#else
    return new ICM20948Sensor();  // Default
#endif
  }

  // Create backup IMU sensor
  static IMUInterface* createBackupSensor() {
#ifdef USE_BNO055_BACKUP
    return new BNO055Sensor();  // NEW OPTION
#elif defined(USE_BONO85_BACKUP)
    return new BNO085Sensor();
#else
    return new KX134Sensor();  // Default
#endif
  }

  // ... rest of factory methods ...
};
#endif
```

#### Step 3: Add Compile-Time Selection

Edit `platformio.ini`:

```ini
[env:teensy41_bno055]
extends = teensy41
build_flags =
  ${teensy41.build_flags}
  -D USE_BNO055_PRIMARY
  ; Comment out for KX134: -D USE_BNO055_BACKUP
lib_deps =
  ${teensy41.lib_deps}
  adafruit/Adafruit BNO055 @ ^1.1.2
```

#### Step 4: Create Unit Tests

Create `test/unit/test_bno055_sensor.cpp`:

```cpp
#include <unity.h>
#include "../../src/sensors/bno055_sensor.h"

BNO055Sensor sensor;

void setUp() {
  // Mock I2C before sensor.begin()
  // For desktop testing, mock hardware
}

void tearDown() {
}

void test_bno055_initialization() {
  TEST_ASSERT_TRUE(sensor.begin());
  TEST_ASSERT_TRUE(sensor.isHealthy());
}

void test_bno055_acceleration_read() {
  sensor.begin();
  TEST_ASSERT_NOT_NULL(sensor.getAccelX());
  TEST_ASSERT_NOT_NULL(sensor.getAccelMagnitude());
}

void test_bno055_quaternion_normalized() {
  sensor.begin();
  float qw, qx, qy, qz;
  sensor.getQuaternion(qw, qx, qy, qz);
  float magnitude = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, magnitude);
}

void test_bno055_temperature_reasonable() {
  sensor.begin();
  float temp = sensor.getTemperature();
  TEST_ASSERT_TRUE(temp > -50.0f && temp < 150.0f);
}

// Run tests
int main() {
  UNITY_BEGIN();
  RUN_TEST(test_bno055_initialization);
  RUN_TEST(test_bno055_acceleration_read);
  RUN_TEST(test_bno055_quaternion_normalized);
  RUN_TEST(test_bno055_temperature_reasonable);
  return UNITY_END();
}
```

#### Step 5: Update Flight Logic

The flight logic automatically uses whichever sensor is created via the factory:

```cpp
// In TripleT_Flight_Firmware.cpp setup()
void setup() {
  // ... initialization ...
  g_imu_manager = SensorFactory::createIMUManager();
  // Flight logic just calls g_imu_manager methods
}
```

#### Step 6: Add to Configuration Documentation

Edit `docs/CONFIGURATION.md`:

```markdown
### Sensor Selection (platformio.ini)

#### Build Variant: BNO055 (Alternative Sensor)
```ini
[env:teensy41_bno055]
extends = teensy41
build_flags = -D USE_BNO055_PRIMARY
lib_deps = adafruit/Adafruit BNO055 @ ^1.1.2
```

**Advantages:** Built-in sensor fusion, no separate gyro/accel
**Disadvantages:** Only ±16G, requires external library
```

---

## Testing Strategy

### Three-Tier Approach

#### 1. Unit Tests (Desktop, No Hardware)

Run with: `pio test -e native_test`

Tests isolated functions without hardware:

```cpp
// test/unit/test_math_functions.cpp
void test_kalman_update() {
  KalmanFilter kf;
  kf.begin();

  float accel = 50.0f;  // 5G acceleration
  kf.update(accel);

  float estimate = kf.getEstimate();
  TEST_ASSERT_TRUE(estimate > 0);
  TEST_ASSERT_TRUE(estimate < accel);  // Filter smooths it
}
```

**Test Coverage Targets:**
- Overall: > 70%
- Flight-critical code: > 95%
  - Apogee detection
  - Landing detection
  - State transitions
  - Error recovery

#### 2. Integration Tests (Mock Hardware)

Mock sensors simulate real flight data:

```cpp
// test/mocks/mock_sensors.h
class MockIMUSensor : public IMUInterface {
public:
  void setAcceleration(float ax, float ay, float az) {
    accel_x = ax;
    accel_y = ay;
    accel_z = az;
  }

  bool read() override {
    // Simulate sensor read with optional noise
    accel_x += noise();
    return true;
  }
  // ... implement other methods ...
};
```

Run full flight simulation:

```cpp
void test_nominal_flight_sequence() {
  MockIMUSensor sensor;
  FlightLogic logic(&sensor);

  // Simulate launch
  for (int t = 0; t < 1000; t++) {
    sensor.setAcceleration(0, 0, 50);  // High G during boost
    logic.update();
  }

  // Simulate coast phase
  for (int t = 0; t < 500; t++) {
    sensor.setAcceleration(0, 0, -10);  // Deceleration
    logic.update();
  }

  TEST_ASSERT_EQUAL(APOGEE, logic.getFlightState());
}
```

#### 3. Flight Tests (Real Hardware)

Validate on actual test flights before production release.

**Pre-Flight Checklist:**
- All sensors healthy
- GPS lock obtained
- SD card writable
- Parachute deployment confirmed
- Battery level adequate

**Test Flight Sequence:**
1. Calibration flight (passive, no deployment)
2. Apogee detection flight (verify altitude accuracy)
3. Drogue deployment flight (test recovery system 1)
4. Main deployment flight (test recovery system 2)
5. Full-power flight (validate all systems)

### CI/CD Pipeline

GitHub Actions automatically runs unit tests on every push:

```yaml
# .github/workflows/test.yml
name: Tests
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - uses: platformio/setup-platformio@v3
      - run: pio test -e native_test
      - run: pio run -e teensy41  # Verify build
```

### Local Test Execution

```bash
# Run all unit tests
pio test -e native_test

# Run specific test file
pio test -e native_test -f test_apogee_detection

# Run with verbose output
pio test -e native_test -v

# Run on Teensy (requires hardware)
pio run -e teensy41 -t upload
pio device monitor --baud 115200
```

---

## Code Organization

### Module Responsibilities

| Module | Responsibility | Key Files |
|--------|----------------|-----------|
| **Flight State Machine** | Core flight logic, state transitions | `flight_logic.cpp`, `state_management.cpp` |
| **HAL (Hardware Abstraction)** | Hardware independence | `src/hal/*.h` |
| **Sensor Management** | IMU drivers, redundancy | `src/sensors/*.h`, `*_functions.cpp` |
| **Guidance Control** | Servo commands, stability | `guidance_control.cpp` |
| **Sensor Fusion** | Kalman filter | `kalman_filter.cpp` |
| **Data Logging** | SD card, CSV format | `log_format_definition.cpp` |
| **Command Interface** | Serial commands | `command_processor.cpp` |
| **Configuration** | Runtime parameters | `config.h`, `data_structures.h` |

### Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                      Hardware Layer (HAL)                        │
│  ITimer, ISerial, IGPIO, II2C, IEEPROM, ISDCard, IServo, IWatchdog
└────────────────────────┬────────────────────────────────────────┘
                         │
          ┌──────────────┼──────────────┐
          │              │              │
    ┌─────▼──────┐ ┌────▼─────┐ ┌─────▼──────┐
    │ Sensors    │ │ Barometer│ │    GPS     │
    │(IMUManager)│ │(MS5611)  │ │ (u-blox)   │
    └─────┬──────┘ └────┬─────┘ └─────┬──────┘
          │             │             │
          │     ┌───────▼─────────┐   │
          │     │  Kalman Filter  │   │
          └────▶│  (Orientation)  │◀──┘
                └───────┬─────────┘
                        │
              ┌─────────▼─────────┐
              │ Flight Logic      │
              │ (State Machine)   │
              └─────────┬─────────┘
                        │
         ┌──────────────┼──────────────┐
         │              │              │
    ┌────▼─────┐  ┌────▼─────┐  ┌────▼─────┐
    │ Guidance  │  │ Logging  │  │ Serial   │
    │ Control   │  │ (SD Card)│  │ Commands │
    └──────────┘  └──────────┘  └──────────┘
         │              │              │
         └──────────────┼──────────────┘
                        │
              ┌─────────▼──────────┐
              │ Actuators          │
              │ (Servos, Pyro)    │
              └────────────────────┘
```

---

## Flight State Machine

### 14-State Flow

```
STARTUP → CALIBRATION → PAD_IDLE → ARMED → BOOST → COAST → APOGEE →
           DROGUE_DEPLOY → DROGUE_DESCENT → MAIN_DEPLOY → MAIN_DESCENT →
           LANDED → RECOVERY

                           ↓ (on critical error)
                         ERROR
```

### State Descriptions

| State | Duration | Actions | Exit Condition |
|-------|----------|---------|----------------|
| **STARTUP** | ~2s | Initialize HAL, sensors | All systems healthy |
| **CALIBRATION** | ~5s | Calibrate barometer, gyro | Calibration complete |
| **PAD_IDLE** | Until arm | Monitor sensors | Serial `arm` command |
| **ARMED** | Until launch | Monitor accelerometer | Acceleration > threshold |
| **BOOST** | 10-30s | Log all data | Acceleration < threshold |
| **COAST** | 5-15s | Detect apogee | Barometer descent detected |
| **APOGEE** | 1s | Signal apogee reached | Drogue deployment |
| **DROGUE_DEPLOY** | 1s | Fire pyro charge | Parachute deployed |
| **DROGUE_DESCENT** | 30-60s | Descending under drogue | Altitude < threshold |
| **MAIN_DEPLOY** | 1s | Fire main pyro | Main parachute deployed |
| **MAIN_DESCENT** | Until landing | Fine descent | Velocity < 1 m/s |
| **LANDED** | 10s | Wait for recovery | Timeout to RECOVERY |
| **RECOVERY** | Until power-off | Beacon active, GPS tx | Manual power-off |
| **ERROR** | Varies | Attempt recovery | Manual clear or reset |

### State Transitions

Add state transitions in `flight_logic.cpp`:

```cpp
void updateFlightState() {
  switch (current_state) {
    case ARMED:
      if (accel_z > BOOST_ACCEL_THRESHOLD) {
        setFlightState(BOOST);
        log_event("Launch detected");
      }
      break;

    case COAST:
      if (detectApogee()) {
        setFlightState(APOGEE);
        deployDrogue();
      }
      break;

    // ... other transitions ...
  }
}
```

---

## Common Development Tasks

### Building

```bash
# Build for Teensy
pio run -e teensy41

# Build for desktop testing
pio run -e native_test

# Clean build
pio run -t clean -e teensy41
```

### Uploading

```bash
# Upload to Teensy (interactive mode detection)
pio run -t upload -e teensy41

# Specify port explicitly
pio run -t upload -e teensy41 --upload-port /dev/ttyACM0
```

### Serial Monitoring

```bash
# Monitor at 115200 baud (default)
pio device monitor --baud 115200

# Monitor with timestamp
pio device monitor --baud 115200 --quiet --no-reset

# Pipe to file
pio device monitor --baud 115200 > flight_output.txt
```

### Adding Configuration Parameters

1. **Define in config.h:**
   ```cpp
   #define NEW_PARAM_VALUE 42
   ```

2. **Use throughout code:**
   ```cpp
   if (sensor_reading > NEW_PARAM_VALUE) {
     // Take action
   }
   ```

3. **Add to command processor:**
   ```cpp
   void handleStatusCommand() {
     hal->serial()->print("NEW_PARAM_VALUE: ");
     hal->serial()->println(NEW_PARAM_VALUE);
   }
   ```

4. **Update documentation:**
   Edit `docs/CONFIGURATION.md` with explanation

### Adding Logged Data Fields

1. **Extend LogData struct** in `src/data_structures.h`:
   ```cpp
   struct LogData {
     uint32_t timestamp;
     float new_field;
     // ... other fields ...
   };
   ```

2. **Update CSV headers** in `log_format_definition.cpp`:
   ```cpp
   const char* csv_headers[] = {
     "timestamp",
     "new_field",
     // ...
   };
   ```

3. **Populate in main loop** in `TripleT_Flight_Firmware.cpp`:
   ```cpp
   void loop() {
     // ...
     log_data.new_field = getNewValue();
     logToSD(log_data);
   }
   ```

### Creating a New Command

1. **Add handler** in `command_processor.cpp`:
   ```cpp
   void handleMyCommand(const SystemStatusContext& context) {
     hal->serial()->println("My command executed");
   }
   ```

2. **Register in command dispatcher**:
   ```cpp
   if (strcmp(cmd, "mycommand") == 0) {
     handleMyCommand(context);
   }
   ```

3. **Add help text**:
   ```cpp
   void printHelpText() {
     hal->serial()->println("mycommand - Does something specific");
   }
   ```

---

## Best Practices

### Code Style

- **Naming:** CamelCase for functions, snake_case for variables
- **Comments:** Explain WHY, not WHAT
- **Line length:** Keep under 100 characters
- **Indentation:** 2 spaces (Arduino standard)

### Safety-Critical Code

For state transitions and apogee detection:

1. **Validate inputs:**
   ```cpp
   if (!sensor || sensor->isFailed()) {
     return false;
   }
   ```

2. **Check assumptions:**
   ```cpp
   if (altitude < 0.0f || altitude > 100000.0f) {
     // Unreasonable value
     handleSensorError();
   }
   ```

3. **Use redundancy:**
   ```cpp
   bool apogee_detected = (baro_descent && accel_descent) || gps_descent || timeout;
   ```

4. **Test thoroughly:**
   - Mock sensors with edge cases
   - Test error recovery paths
   - Verify failsafe behavior

### Memory Management

- **Avoid dynamic allocation:** Use stack or static buffers
- **Check memory usage:** `pio run -e native_test` shows RAM/Flash
- **Pre-allocate buffers:** Especially for logging

### Testing New Features

```bash
# 1. Write tests first (TDD)
# test/unit/test_new_feature.cpp

# 2. Run locally
pio test -e native_test -f test_new_feature

# 3. Build for Teensy
pio run -e teensy41

# 4. Verify no memory issues
pio run -e teensy41 -v | grep -E "Memory|RAM|Flash"

# 5. Test on real hardware
# Upload and fly test mission
```

---

## Resources

- **Arduino HAL Documentation:** [docs.platformio.org](https://docs.platformio.org)
- **Teensy 4.1 Reference:** [pjrc.com/teensy](https://www.pjrc.com/teensy/teensy41.html)
- **Git Workflow:** See `CLAUDE.md` for branch strategy
- **Hardware Setup:** See `docs/HARDWARE.md`
- **Safety Procedures:** See `docs/SAFETY.md`

---

**Questions? Consult:**
- `CLAUDE.md` - Project conventions
- `docs/ARCHITECTURE.md` - System design
- `docs/CONFIGURATION.md` - Parameter guide
- `test/unit/` - Example test code
- `src/hal/hal_interfaces.h` - HAL documentation
