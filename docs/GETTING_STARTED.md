# Getting Started Guide - TripleT Flight Firmware

**Last Updated:** February 15, 2026
**For:** First-time developers, hardware integrators, test pilots

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Initial Setup](#initial-setup)
4. [First Build](#first-build)
5. [Uploading to Teensy](#uploading-to-teensy)
6. [Serial Communication](#serial-communication)
7. [Troubleshooting](#troubleshooting)
8. [Next Steps](#next-steps)

---

## Prerequisites

### Hardware Required

- **Teensy 4.1** microcontroller board
  - Includes USB cable
  - Can be purchased from: [pjrc.com/teensy](https://www.pjrc.com/teensy/teensy41.html)

- **Sensors** (for full flight computer):
  - ICM-20948 6-axis IMU (I2C address 0x68)
  - KX134 high-G accelerometer (I2C address 0x1E)
  - MS5611 barometer (I2C address 0x76 or 0x77)
  - u-blox NEO-8M GPS receiver (UART serial or SPI)
  - SD card (microSD or full-size with adapter)

- **Development Computer**:
  - Linux (recommended), macOS, or Windows
  - At least 2GB free disk space
  - USB port for Teensy connection

### Software Required

All software is **free and open-source**.

#### 1. Git

**Purpose:** Version control and repository management

**Installation:**
```bash
# Ubuntu/Debian
sudo apt-get install git

# macOS (with Homebrew)
brew install git

# Windows: Download from https://git-scm.com/download/win
# Then run installer
```

**Verify:**
```bash
git --version
# Expected: git version 2.x.x
```

#### 2. Python 3

**Purpose:** PlatformIO build system

**Installation:**
```bash
# Ubuntu/Debian
sudo apt-get install python3 python3-pip

# macOS
brew install python3

# Windows: Download from https://www.python.org/
# Check "Add Python to PATH" during installation
```

**Verify:**
```bash
python3 --version
# Expected: Python 3.7 or higher
```

#### 3. PlatformIO

**Purpose:** Teensy firmware build and upload tool

**Installation:**
```bash
# Install via pip
pip install platformio

# Or if pip3:
pip3 install platformio
```

**Verify:**
```bash
pio --version
# Expected: PlatformIO Core X.X.X
```

**Initialize PlatformIO home:**
```bash
pio system info
# This creates ~/.platformio/ directory
```

### Recommended Tools (Optional)

- **VS Code** - Code editor with PlatformIO plugin
  - Download: [code.visualstudio.com](https://code.visualstudio.com)
  - Install PlatformIO Extension

- **Serial Monitor** - Monitor firmware output
  - Built into PlatformIO
  - Or: `minicom`, `screen`, `putty` (Windows)

- **Git GUI** (optional)
  - GitKraken, SourceTree, or GitHub Desktop

---

## Installation

### Step 1: Clone Repository

```bash
# Choose a directory for your projects
mkdir -p ~/projects
cd ~/projects

# Clone the TripleT repository
git clone https://github.com/madmonkey71/TripleT-Flight-Firmware.git
cd TripleT-Flight-Firmware

# Verify directory structure
ls -la
# Expected: src/, test/, docs/, platformio.ini, README.md, etc.
```

### Step 2: Verify PlatformIO Setup

```bash
# Inside TripleT-Flight-Firmware directory
pio project config
# Should show configuration for teensy41 and native_test environments
```

### Step 3: Update Libraries

```bash
# Download all dependencies (Unity test framework, sensor libraries, etc.)
pio lib update

# Verify libraries installed
pio lib list
# Expected: Several Arduino libraries listed
```

### Step 4: Verify Initial Build

```bash
# Attempt to build for Teensy 4.1
pio run -e teensy41

# Expected output (final lines):
#
# Environment teensy41 Status: SUCCESS
# ====== 1 succeeded in X.XXs ======
```

**If build fails**, see [Troubleshooting](#troubleshooting)

---

## Initial Setup

### Understanding the Project Structure

```
TripleT-Flight-Firmware/
├── src/                          # Main firmware source code
│   ├── TripleT_Flight_Firmware.cpp    # Entry point (setup + loop)
│   ├── flight_logic.cpp          # Core state machine
│   ├── config.h                  # Configuration parameters
│   ├── data_structures.h         # Data types
│   ├── hal/                      # Hardware abstraction layer
│   │   ├── hal_interfaces.h      # HAL interface definitions
│   │   ├── teensy_hal.h          # Teensy implementations
│   │   └── mock_hal.h            # Testing implementations
│   └── sensors/                  # Sensor drivers
│       ├── imu_interface.h       # IMU base interface
│       ├── sensor_factory.h      # Sensor creation
│       └── *_sensor.h            # Individual sensors
├── test/                         # Tests (see Testing section)
├── docs/                         # Documentation
├── platformio.ini                # PlatformIO configuration
└── README.md                     # Project overview
```

### Key Configuration File

Open `src/config.h`:

```cpp
// Most important settings:

#define FIRMWARE_VERSION "v0.9.0"
// ↑ Current version

#define BOARD_TEENSY41
// ↑ Target board (only option: Teensy 4.1)

#define BOOST_ACCEL_THRESHOLD 2.0f
// ↑ Launch detection: 2G acceleration = liftoff

#define COAST_ACCEL_THRESHOLD 0.5f
// ↑ Motor burnout detection: acceleration drops below 0.5G

#define MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M 100.0f
// ↑ Deploy main parachute at 100m above ground

#define USE_KX134 1
// ↑ Enable high-G accelerometer (1=yes, 0=no)

#define GPS_USE_SPI 1
// ↑ GPS connection: 1=SPI, 0=I2C
```

**Modify settings as needed for your hardware setup.**

---

## First Build

### Build for Teensy

```bash
# Navigate to project directory
cd ~/projects/TripleT-Flight-Firmware

# Build for production (Teensy 4.1)
pio run -e teensy41

# Output will show:
# - Compiling source files
# - Linking
# - Final output
# - Success/Failure status
```

**Expected Output (last 10 lines):**
```
Compiling .pio/build/teensy41/src/flight_logic.cpp.o
Compiling .pio/build/teensy41/src/guidance_control.cpp.o
Linking .pio/build/teensy41/firmware.elf
Checking size .pio/build/teensy41/firmware.elf
  text       data     bss     dec     hex filename
 208942     4084  45732  258758  3f286 .pio/build/teensy41/firmware.elf
  RAM:   32.5% (used 16746 bytes from 53248 bytes)
  Flash: 19.9% (used 208942 bytes from 1044480 bytes)
====== [SUCCESS] Took X.XXs ======
```

**Key Metrics:**
- **RAM:** Should be < 50% (headroom for runtime allocations)
- **Flash:** Should be < 80% (room for future features)

### Build for Testing (Desktop)

```bash
# Build unit tests for your computer (not Teensy)
pio run -e native_test

# Run tests immediately after building
pio test -e native_test

# Expected output:
#
# test/unit/test_state_machine.cpp::<test_name> [PASSED]
# test/unit/test_apogee_detection.cpp::<test_name> [PASSED]
# ...
# ===== X passed in Y.YYs ======
```

---

## Uploading to Teensy

### Hardware Connection

1. **Connect Teensy 4.1 to your computer**:
   - Micro-USB cable → Teensy USB port
   - Status LED should light up (blue/red)

2. **Verify Connection**:
   ```bash
   # Check if system recognizes Teensy
   ls /dev/ttyACM*   # Linux/Mac
   # or
   # Device Manager → Ports (Windows)

   # Should see: /dev/ttyACM0 (Linux) or /dev/cu.usbmodem* (Mac)
   ```

### Upload Firmware

```bash
# With Teensy connected via USB
pio run -t upload -e teensy41

# Expected output:
# Looking for upload port...
# Upload port: /dev/ttyACM0
# Uploading .pio/build/teensy41/firmware.elf
# ........ (dots as upload progresses)
# ====== [SUCCESS] Took X.XXs ======

# Teensy LED should blink during upload
```

### Verify Upload Success

After upload completes:

```bash
# Open serial monitor to see output
pio device monitor --baud 115200

# Expected output (first lines):
# TripleT Flight Firmware v0.9.0
# Initializing HAL...
# Initializing sensors...
# Sensor initialization: OK
# System ready
#
# >  (command prompt)
```

---

## Serial Communication

### Opening Serial Monitor

```bash
# Default speed: 115200 baud
pio device monitor --baud 115200

# Or with more options:
pio device monitor --baud 115200 --eol LF --raw
```

### Basic Commands

Once in monitor, type these commands:

```
help ↵
  → Lists all available commands

status_sensors ↵
  → Shows all sensor health and current readings

calibrate ↵
  → Calibrates barometer and gyroscope

arm ↵
  → Arms system for launch (changes state to ARMED)

log_test ↵
  → Tests SD card logging with dummy data

reset ↵
  → Soft reset (via watchdog)
```

### Interpreting Status Output

```
> status_sensors

=== SENSOR STATUS ===
IMU Primary (ICM-20948):  HEALTHY
  Accel: +0.1, -0.2, +9.81 m/s²
  Gyro:  +0.0, -0.1, +0.0 deg/s
  Temp:  +25.3°C
  Quat:  +1.00, +0.00, -0.00, +0.01

IMU Backup (KX134):       HEALTHY
  Accel: +0.0, -0.1, +9.81 m/s²

Barometer (MS5611):       HEALTHY
  Pressure: 101.325 kPa (1013 mbar)
  Altitude: 125m
  Temp: +25.0°C

GPS (u-blox):             HEALTHY (3D fix)
  Latitude: 40.1234°
  Longitude: -105.5678°
  Altitude: 1234m MSL
  SNR: 28 dB
  Sats: 12
```

**Green Light Checklist:**
- [ ] All sensors show HEALTHY (not FAIL)
- [ ] Acceleration at rest ≈ 9.81 m/s² (±0.5)
- [ ] Temperature in reasonable range (0°C to 50°C typical)
- [ ] GPS shows "3D fix" with valid position
- [ ] Altitude reasonable (within ±50m of actual)

---

## Troubleshooting

### Build Fails: "teensy41 environment not found"

**Cause:** PlatformIO not properly initialized

**Solution:**
```bash
# Reinstall platformio
pip uninstall platformio
pip install platformio

# Reinitialize
pio system info

# Try build again
pio run -e teensy41
```

### Build Fails: "Cannot find ArduinoFake"

**Cause:** Libraries not downloaded

**Solution:**
```bash
# Update libraries
pio lib update

# Clean and rebuild
pio run -t clean -e teensy41
pio run -e teensy41
```

### Build Fails: Memory exceeded

**Cause:** Too much code for Teensy 4.1

**Solution:**
1. Check feature flags in `src/config.h`
2. Disable non-essential logging
3. See memory usage:
   ```bash
   pio run -e teensy41 -v | grep -E "RAM|Flash"
   ```

### Teensy Not Found During Upload

**Cause:** USB connection issue or driver missing

**Solution:**
```bash
# Linux: Check USB device
lsusb | grep Teensy
# Should show: Teensy device

# Restart udev rules
sudo udevadm control --reload

# macOS: Just try again (usually works on retry)
# Windows: Install TeensyduinoInstaller from https://www.pjrc.com/

# Try upload with explicit port:
pio run -t upload -e teensy41 --upload-port /dev/ttyACM0
```

### Serial Monitor Shows Garbage Text

**Cause:** Baud rate mismatch

**Solution:**
```bash
# Firmware uses 115200 baud (confirmed in code)
pio device monitor --baud 115200

# Not 9600 or other rates
```

### Sensors Show "FAIL"

**Cause:** Hardware connection issue

**Steps:**
1. Verify I2C connections (SDA/SCL to Teensy pins)
2. Check pull-up resistors (typically 4.7kΩ on I2C bus)
3. Power supply adequate (3.3V, >500mA available)
4. I2C address conflicts (see `docs/HARDWARE.md`)

---

## Next Steps

### After First Successful Upload

1. **Read the documentation:**
   - `README.md` - Project overview
   - `docs/DEVELOPER_GUIDE.md` - Development patterns
   - `docs/ARCHITECTURE.md` - System design
   - `docs/CONFIGURATION.md` - Advanced settings

2. **Explore the code:**
   - `src/flight_logic.cpp` - Core state machine
   - `src/sensors/` - Sensor implementations
   - `src/hal/` - Hardware abstraction layer
   - `test/unit/` - Example tests

3. **Run tests:**
   ```bash
   # Unit tests (desktop, no hardware)
   pio test -e native_test

   # See what tests are available
   ls test/unit/test_*.cpp
   ```

4. **Modify configuration:**
   - Edit `src/config.h`
   - Rebuild and upload
   - Test behavior

5. **Add your own feature:**
   - Create a test in `test/unit/test_my_feature.cpp`
   - Implement in `src/my_feature.cpp`
   - Follow patterns in existing code

### Common Development Workflow

```bash
# 1. Make code changes
nano src/flight_logic.cpp

# 2. Run tests (quick desktop check)
pio test -e native_test

# 3. Build for Teensy
pio run -e teensy41

# 4. Upload if build successful
pio run -t upload -e teensy41

# 5. Monitor output
pio device monitor --baud 115200

# 6. Type commands to test behavior
# > status_sensors
# > arm
# > log_test
# > reset
```

### Getting Help

**Documentation:** Check `/docs/` directory

**Questions:** Look in `DEVELOPER_GUIDE.md`:
- "How do I add a new sensor?" → See BNO055 example
- "What's the HAL architecture?" → See ARCHITECTURE.md
- "How do I run tests?" → See DEVELOPER_GUIDE.md Testing section
- "What's a safe change?" → See SAFETY.md

**Specific Issues:**
- Build problems → `CLAUDE.md` (project conventions)
- Hardware setup → `docs/HARDWARE.md`
- Sensor calibration → `docs/CONFIGURATION.md`
- Flight procedures → `docs/FLIGHT_STATE_MACHINE.md`

---

## Quick Reference

### Most Common Commands

```bash
# Build for Teensy
pio run -e teensy41

# Upload to Teensy
pio run -t upload -e teensy41

# Run tests on desktop
pio test -e native_test

# Monitor serial output (after uploading)
pio device monitor --baud 115200

# Clean build
pio run -t clean -e teensy41

# See all environments
cat platformio.ini | grep "\[env"
```

### Most Common Serial Commands

```
help              - List all commands
status_sensors    - Sensor health report
arm               - Arm for launch
calibrate         - Calibrate sensors
log_test          - Test SD card logging
reset             - Restart Teensy
```

### Most Important Files

| File | Purpose |
|------|---------|
| `src/config.h` | Configuration parameters |
| `src/flight_logic.cpp` | State machine logic |
| `src/TripleT_Flight_Firmware.cpp` | Entry point (setup/loop) |
| `platformio.ini` | Build configuration |
| `docs/DEVELOPER_GUIDE.md` | Development reference |

---

## Success Indicators

You've successfully set up TripleT when:

- [ ] `pio run -e teensy41` completes with SUCCESS
- [ ] Teensy accepts upload without errors
- [ ] Serial monitor shows system ready message
- [ ] `status_sensors` shows all sensors HEALTHY
- [ ] `pio test -e native_test` passes all tests
- [ ] You can send `arm` command and system enters ARMED state

---

## Next: Flight Preparation

Once development setup is complete:

1. Read `docs/FLIGHT_STATE_MACHINE.md` for flight phases
2. Review `docs/SAFETY.md` for pre-flight checklist
3. Examine `docs/HARDWARE.md` for wiring diagram
4. Plan your first test flight

---

**Questions?** Check the documentation in `/docs/` or refer to CLAUDE.md for project conventions.

**Ready to develop?** See `DEVELOPER_GUIDE.md` for code patterns and examples.

**Ready to fly?** See `docs/SAFETY.md` for pre-flight procedures.
