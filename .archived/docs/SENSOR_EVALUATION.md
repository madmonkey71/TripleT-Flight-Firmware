# Sensor Evaluation & Hardware Comparison - TripleT Flight Firmware v0.9.0

**Last Updated:** February 15, 2026
**Target Audience:** Hardware engineers, system integrators, procurement specialists

---

## Table of Contents

1. [Executive Summary](#executive-summary)
2. [Primary IMU Comparison](#primary-imu-comparison)
3. [High-G Accelerometer Options](#high-g-accelerometer-options)
4. [Barometer Options](#barometer-options)
5. [GPS Receiver Comparison](#gps-receiver-comparison)
6. [Hardware Compatibility Matrix](#hardware-compatibility-matrix)
7. [Migration Path for Sensor Changes](#migration-path-for-sensor-changes)
8. [Recommendations for v1.0.0](#recommendations-for-v1.0.0)

---

## Executive Summary

### Current Hardware (v0.9.0)

The TripleT Flight Firmware v0.9.0 is designed around:

| Subsystem | Current (Primary) | Current (Backup) | Status |
|-----------|------------------|------------------|--------|
| **IMU** | ICM-20948 | KX134 | Production |
| **Barometer** | MS5611 | N/A | Production |
| **GPS** | u-blox NEO-8M | N/A | Production |
| **Microcontroller** | Teensy 4.1 | N/A | Production |

### Architecture Supports Alternatives

The modular IMUInterface pattern allows easy sensor substitution:

```cpp
// Compile-time sensor selection
#ifdef USE_BNO085_VARIANT
  return new BNO085Sensor();  // Alternative
#else
  return new ICM20948Sensor();  // Default
#endif
```

This enables evaluation of:
- BNO055 / BNO085 (integrated 9-DOF with fusion)
- ICM-20649 (Successor to ICM-20948)
- LSM6DSRX / LSM9DS1 (ST Microelectronics alternatives)

---

## Primary IMU Comparison

### ICM-20948 (Current Primary)

**Status:** Production, proven reliable

**Specifications:**
```
Accelerometer Range:        ±2, ±4, ±8, ±16 G (selectable)
Accelerometer Output:       14-bit resolution
Accelerometer Noise:        20 mg RMS @ ±16G
Accelerometer Bandwidth:    Single-pole 246 Hz, 111 Hz, 50 Hz

Gyroscope Range:            ±250, ±500, ±1000, ±2000 dps (selectable)
Gyroscope Output:           16-bit resolution
Gyroscope Noise:            4 dps RMS (no root)
Gyroscope Bandwidth:        Same as accel (246 Hz)

Magnetometer:               14-bit resolution
Operating Temp:             -40°C to +85°C
Supply Voltage:             2.4V to 3.6V (I2C pull-ups at 3.3V)
I2C Address:                0x68 or 0x69 (selectable via AD0)
Output Data Rate:           8 Hz to 1.125 kHz
Dimensions:                 4.0mm x 4.0mm x 0.9mm
```

**Advantages:**
- ✓ Proven in multiple flights
- ✓ Good balance of range (±16G) and resolution
- ✓ Built-in magnetometer (calibration data in EEPROM)
- ✓ Kalman filter integrates gyro + accel well
- ✓ Multiple range options (can reduce noise with ±2G for recovery)
- ✓ Well-supported library (Invensense)

**Disadvantages:**
- ✗ ±16G limit (insufficient for small high-power rockets)
- ✗ Requires KX134 backup for high-G detection
- ✗ No built-in sensor fusion (requires external Kalman filter)

**Use Case:** ✓ Excellent for general flight dynamics, moderate rockets

**Cost:** ~$15-25 per unit

---

### BNO085 / BNO055 (Alternative)

**Status:** Alternative evaluation, not currently in use

**Specifications (BNO085):**
```
Accelerometer Range:        ±4, ±8, ±16 G
Accelerometer Output:       14-bit resolution
Accelerometer Noise:        24 mg RMS @ ±8G

Gyroscope Range:            ±2000 dps
Gyroscope Output:           16-bit resolution

Magnetometer:               14-bit resolution

QUARTERNION OUTPUT:         Direct from sensor (major advantage!)
Sensor Fusion:              Bosch proprietary (9-axis)

Operating Temp:             -40°C to +85°C
Supply Voltage:             1.71V to 3.6V
I2C Address:                0x28 or 0x29 (selectable)
Output Data Rate:           1 Hz to 1000 Hz
Dimensions:                 2.5mm x 3.0mm x 1.0mm (tiny!)
```

**Advantages:**
- ✓ **Built-in quaternion output** (eliminates Kalman filter complexity)
- ✓ Built-in 9-axis sensor fusion
- ✓ Very compact size
- ✓ Same ±16G range but better integration
- ✓ Potentially more accurate orientation
- ✓ Internal temperature compensation

**Disadvantages:**
- ✗ ±16G limit (same high-G issue as ICM-20948)
- ✗ Proprietary sensor fusion (hard to debug if it fails)
- ✗ Cost higher (~$25-35 per unit)
- ✗ Smaller community and fewer examples
- ✗ Library support varies by repository

**Use Case:** ⚠ Good alternative if quaternion output preferred, still needs high-G backup

**Cost:** ~$25-35 per unit

---

### ICM-20649 (Newer Successor)

**Status:** Available, not evaluated yet

**Specifications:**
```
Accelerometer Range:        ±4, ±8, ±16, ±30 G (30G option!)
Accelerometer Output:       16-bit resolution
Accelerometer Noise:        Better than ICM-20948

Gyroscope Range:            ±250, ±500, ±1000, ±2000, ±4000 dps
Gyroscope Output:           16-bit resolution

Magnetometer:               None (compass built-in to some boards)

Supply Voltage:             1.71V to 3.6V
I2C Address:                0x68
Operating Temp:             -40°C to +85°C
Output Data Rate:           0.98 Hz to 11.5 kHz
```

**Advantages:**
- ✓ ±30G option (reduces need for high-G backup!)
- ✓ Pin-compatible with ICM-20948 (easy swap)
- ✓ Better noise performance
- ✓ Higher output data rate

**Disadvantages:**
- ✗ No built-in magnetometer (external compass required)
- ✗ Not yet integrated into TripleT codebase
- ✗ Requires testing to validate performance

**Recommendation:** Consider for v1.1.0, pending field evaluation

---

### LSM9DS1 / LSM6DSRX (ST Microelectronics)

**Status:** Competitor option, not integrated

**Key Specs (LSM9DS1):**
```
Accel Range:                ±2, ±4, ±8, ±16 G
Gyro Range:                 ±245, ±500, ±2000 dps
Mag Range:                  ±4, ±8, ±12, ±16 Gauss
I2C Address:                0x6B (accel/gyro), 0x1D (mag)
Supply:                     1.9V to 3.6V
Temp Range:                 -40°C to +85°C
```

**Comparison:**
- Similar performance to ICM-20948
- Two separate I2C addresses (more complex)
- Good alternative if Invensense chips unavailable
- Stronger community in robotics (not rockets specifically)

**Status:** Not recommended for TripleT (no advantage over ICM-20948)

---

## High-G Accelerometer Options

### KX134 (Current Backup)

**Status:** Production, proven reliable

**Specifications:**
```
Accelerometer Range:        ±8, ±16, ±32, ±64 G (selectable)
Accelerometer Output:       16-bit resolution
Accelerometer Noise:        5 mg RMS @ ±64G
Accelerometer Bandwidth:    ~1.6 kHz

Gyroscope:                  None (accel-only sensor)
Magnetometer:               None
Temperature:                -40°C to +85°C
Supply Voltage:             1.71V to 3.6V
I2C Address:                0x1E (fixed)
Output Data Rate:           0.781 Hz to 10 kHz
Dimensions:                 3.0mm x 3.0mm x 0.9mm
```

**Usage Pattern in TripleT:**
- Dual-sensor mode: ICM-20948 (primary) + KX134 (backup)
- Automatic failover if primary fails
- Redundancy for critical accel-based decisions
- High-G events (can handle up to 64G spikes)

**Advantages:**
- ✓ Very sensitive to high acceleration (64G cap!)
- ✓ Proven in flight
- ✓ Simple I2C interface
- ✓ Good balance of cost/performance
- ✓ IMUManager seamlessly switches between primary/backup

**Disadvantages:**
- ✗ No gyroscope (can't measure rotation)
- ✗ No magnetometer (can't measure magnetic field)
- ✗ Less common than other accelerometers
- ✗ When used as backup, loses gyro/mag data temporarily

**Use Case:** ✓ Excellent as high-G backup, proven in rockets

**Cost:** ~$10-15 per unit

---

### BNO085 as Backup

**Alternative:** Use BNO085 for both primary AND backup

**Advantages:**
- ✓ Eliminates need for two different sensor types
- ✓ Always have 9-DOF (no loss on failover)
- ✓ Quaternion always available
- ✓ Simpler component inventory

**Disadvantages:**
- ✗ ±16G limit (no high-G capability improvement)
- ✗ Not actually a backup then (redundancy but not diversification)
- ✗ Both fail if BNO085 manufacturing issue

**Recommendation:** Not preferred - keep KX134 for true high-G backup

---

### H3LIS100DL (Alternative High-G)

**Status:** Available, not integrated

**Specifications:**
```
Accelerometer Range:        ±100 G (!!)
Accelerometer Output:       16-bit resolution
Accelerometer Bandwidth:    ~400 Hz
I2C Address:                0x18 or 0x19
Supply:                     2.4V to 3.6V
Temperature:                -40°C to +85°C
```

**Advantages:**
- ✓ Extreme ±100G capability (overkill for rockets, but safe)
- ✓ Can handle any conceivable rocket acceleration

**Disadvantages:**
- ✗ Lower bandwidth (~400 Hz vs KX134's 1.6 kHz)
- ✗ Larger noise floor at lower accelerations
- ✗ No advantage over KX134 for typical rockets
- ✗ Not integrated into TripleT

**Recommendation:** Not necessary for TripleT's mission profile (overkill)

---

## Barometer Options

### MS5611 (Current)

**Status:** Production, proven reliable

**Specifications:**
```
Pressure Range:             300 to 1100 mbar
Pressure Accuracy:          ±1.5 mbar (absolute)
Pressure Noise:             ~2.5 Pa (0.025 mbar)
Temperature Range:          -40°C to +85°C
Supply Voltage:             1.8V to 3.6V
I2C Address:                0x76 or 0x77 (selectable)
Altitude Resolution:        ~0.3m (from 24-bit output)
Output Data Rate:           0-40 Hz
Dimensions:                 5.0mm x 3.5mm x 1.0mm
Typical Accuracy:           ±1m altitude (field tested)
```

**Flight Advantages:**
- ✓ Proven in TripleT flights
- ✓ ±1m altitude accuracy in recovery range
- ✓ Good temporal resolution (10 Hz updates)
- ✓ Simple I2C protocol
- ✓ Supports multiple I2C addresses (easy to differentiate)

**Use Case:** ✓ Production quality, recommended for v1.0.0

**Cost:** ~$3-5 per unit

---

### BMP388 (Higher Resolution Alternative)

**Status:** Available, not integrated

**Specifications:**
```
Pressure Range:             300 to 1250 mbar
Pressure Accuracy:          ±0.5 Pa (better!)
Pressure Noise:             ~0.9 Pa (much lower!)
Temperature Range:          -40°C to +85°C
Supply:                     1.2V to 3.6V
I2C Address:                0x77 (same as MS5611!)
Altitude Accuracy:          ±0.2m (better resolution)
Output Data Rate:           0-200 Hz (higher!)
Dimensions:                 2.0mm x 2.0mm x 0.8mm (much smaller!)
```

**Comparison to MS5611:**
```
Parameter              MS5611           BMP388
─────────────────────────────────────────────────
Altitude Accuracy:     ±1.0m            ±0.2m (5x better!)
Noise Floor:           2.5 Pa           0.9 Pa (2.8x better!)
Output Rate:           40 Hz            200 Hz (5x faster!)
Cost:                  ~$4              ~$6-8
Size:                  5x3.5mm          2x2mm (4x smaller!)
```

**Advantages:**
- ✓ Much better altitude resolution (±0.2m vs ±1m)
- ✓ Significantly lower noise
- ✓ Higher update rate enables faster apogee detection
- ✓ Physically smaller
- ✓ Same I2C address as MS5611 (backward compatible)

**Disadvantages:**
- ✗ Not yet integrated into TripleT codebase
- ✗ Slightly higher cost
- ✗ More power consumption (minimal impact)
- ✗ Requires library evaluation

**Recommendation:** Consider for v1.1.0 if altitude precision becomes critical

---

### BMP390L (Extreme Altitude Range)

**Status:** Available, specialized

**Key Specs:**
```
Pressure Range:             30 to 1250 mbar (extreme range!)
Altitude Range:             ~0m to ~9000m (covers extreme altitude flights)
Supply:                     1.2V to 3.6V
Accuracy:                   ±0.5 Pa
```

**Use Case:** High-altitude flights (sounding rockets)
**Recommendation:** Not needed for typical model rockets (TripleT's domain)

---

## GPS Receiver Comparison

### u-blox NEO-8M (Current)

**Status:** Production, proven reliable

**Specifications:**
```
Receiver Type:              GPS/GLONASS dual constellation
Position Accuracy:          ~2.5m CEP (typical)
Altitude Accuracy:          ~5-10m (typical)
Speed Accuracy:             ~0.1 m/s
Time Accuracy:              ~100ns (for time sync)
Cold Start Time:            ~30-35 seconds
Warm Start Time:            ~5 seconds
Update Rate:                Up to 10 Hz
Supply Voltage:             2.7V to 3.6V
Supply Current:             40 mA @ 10 Hz
Operating Temp:             -40°C to +85°C
Interface:                  UART serial (or SPI with adapter)
```

**Flight Characteristics:**
- ✓ Proven reliable in multiple TripleT flights
- ✓ Adequate altitude accuracy (±5-10m) for deployment decisions
- ✓ GPS lock typically achieved 30-60 seconds before launch
- ✓ 10 Hz update rate sufficient for apogee detection
- ✓ Good balance of cost/performance

**Use Case:** ✓ Production quality, recommended for v1.0.0

**Cost:** ~$30-50 per unit

---

### u-blox NEO-M8N (Improved Variant)

**Status:** Available, minor improvement

**Specifications:**
```
Receiver Type:              GPS/GLONASS/Galileo tri-constellation
Position Accuracy:          ~2.0m CEP (slightly better)
Altitude Accuracy:          ±5m (same as NEO-8M)
Update Rate:                Up to 18 Hz
Supply Current:             50 mA @ 10 Hz (slightly higher)
```

**Comparison:**
```
Parameter           NEO-8M          NEO-M8N         Improvement
─────────────────────────────────────────────────────────────
Position Accuracy:  2.5m            2.0m            20% better
Constellations:     GPS+GLONASS     GPS+GLONASS+Gal 3 sources
Update Rate:        10 Hz           18 Hz           1.8x faster
Cost:               $40             $50-60          +25% cost
```

**Recommendation:** Marginal improvement; stick with NEO-8M for v1.0.0

---

### u-blox ZED-F9P (RTK/PPP Capable)

**Status:** Available, high-end option

**Key Specs:**
```
Position Accuracy:          ~0.02m (RTK) or ~0.1m (PPP)
Altitude Accuracy:          ~0.03m (RTK)
Update Rate:                Up to 25 Hz
Supply Current:             500 mA (much higher!)
Cost:                       $200+++ (10x more expensive!)
```

**Recommendation:** **NOT recommended for TripleT**
- Overkill for model rocket accuracy needs
- Extremely high power consumption
- Significantly higher cost
- RTK requires ground station setup

---

## Hardware Compatibility Matrix

### Sensor Combinations Tested

```
Configuration           Primary IMU      Backup          Status
─────────────────────────────────────────────────────────────────
Current (v0.9.0)       ICM-20948        KX134           ✓ Production
Alternative A           BNO085          KX134           ⚠ Untested
Alternative B           ICM-20648       None            ✗ Not evaluated
Alternative C           LSM9DS1         None            ✗ Not evaluated
Future (v1.1.0?)       ICM-20948        KX134           ✓ Recommended
```

### I2C Address Configuration

```
Sensor           Default I2C         Alt Address      Notes
─────────────────────────────────────────────────────────────
ICM-20948        0x68               0x69             AD0 pin
KX134            0x1E               None             Fixed
MS5611           0x76               0x77             CSB pin
BNO085           0x28               0x29             ADDR pin
BMP388           0x77               0x76             SDO pin
```

**Current TripleT Configuration:**
- ICM-20948: 0x68 (default)
- KX134: 0x1E (fixed)
- MS5611: 0x76 (default)
- GPS: UART serial

**No I2C conflicts** ✓

---

## Migration Path for Sensor Changes

### Scenario 1: Replace ICM-20948 with BNO085

**Steps:**

1. **Create new sensor adapter:**
   ```cpp
   // src/sensors/bno085_sensor.h (NEW)
   class BNO085Sensor : public IMUInterface {
     bool begin() override { /* ... */ }
     float getAccelX() override { /* ... */ }
     void getQuaternion(...) override { /* ... */ }
     // Implement IMUInterface methods
   };
   ```

2. **Update sensor factory:**
   ```cpp
   // src/sensors/sensor_factory.h
   #ifdef USE_BNO085_VARIANT
     return new BNO085Sensor();
   #else
     return new ICM20948Sensor();
   #endif
   ```

3. **Add compile flag to platformio.ini:**
   ```ini
   [env:teensy41_bno085]
   extends = teensy41
   build_flags = -D USE_BNO085_VARIANT
   ```

4. **Test flight sequence:**
   - Build: `pio run -e teensy41_bno085`
   - Upload: `pio run -t upload -e teensy41_bno085`
   - Test flight: Monitor `status_sensors` during flight
   - Analyze: Compare apogee detection timing with baseline

5. **Update documentation:**
   - `docs/CONFIGURATION.md` - Add BNO085 build instructions
   - `docs/SENSOR_EVALUATION.md` - Add flight test results

**Timeline:** ~2-3 hours (mostly testing)

**Risk Level:** LOW (modular design, backward compatible)

---

### Scenario 2: Add ICM-20649 with ±30G Range

**Steps:**

1. **Create ICM-20649 adapter:**
   ```cpp
   class ICM20649Sensor : public IMUInterface {
     void begin() override {
       ICM_20649_init();  // Existing driver
       setAccelScale(30);  // Use ±30G range
     }
     // ... implement IMUInterface ...
   };
   ```

2. **Update I2C address (same as ICM-20948, 0x68):**
   - Both sensors share same address
   - Can't use both simultaneously
   - Pin-compatible replacement only

3. **Evaluate high-G capability:**
   - Remove KX134 dependency (±30G sufficient)
   - Simplify to single primary sensor
   - Test with high-power motor

4. **Configuration option:**
   ```ini
   [env:teensy41_icm20649]
   build_flags = -D USE_ICM20649_PRIMARY
   lib_deps = invensense/MPU6050 @ ^1.0.11  ; Updated library
   ```

**Advantage:** Potential to eliminate need for KX134 backup

**Timeline:** ~4-6 hours (needs new library testing)

**Risk Level:** MEDIUM (significant hardware change)

---

### Scenario 3: Upgrade to BMP388 Barometer

**Steps:**

1. **BMP388 is drop-in replacement (same I2C address):**
   ```cpp
   // src/ms5611_functions.cpp
   #ifdef USE_BMP388
     BMP388 baro;  // Instead of MS5611
   #else
     MS5611 baro;  // Current
   #endif
   ```

2. **Update barometer driver:**
   ```cpp
   void barometer_init() {
   #ifdef USE_BMP388
     baro.begin(0x77);  // Same address!
   #else
     baro.begin(0x76);
   #endif
   }
   ```

3. **Test improved precision:**
   - Measure altitude steps (stairs)
   - Verify ±0.2m precision vs ±1.0m baseline
   - Confirm apogee detection timing improvement

4. **Configuration:**
   ```ini
   [env:teensy41_bmp388]
   build_flags = -D USE_BMP388_BAROMETER
   ```

**Advantage:** 5x better altitude resolution

**Timeline:** ~2-3 hours (I2C address compatible)

**Risk Level:** LOW (same I2C address, library similar)

---

## Recommendations for v1.0.0

### Recommended Hardware Configuration

```
Subsystem           Recommended        Alternative       Status
────────────────────────────────────────────────────────────────
Primary IMU         ICM-20948          BNO085            ✓ Proven
Backup IMU          KX134              BNO085            ✓ Proven
Barometer           MS5611             BMP388            ✓ Reliable
GPS                 u-blox NEO-8M      NEO-M8N           ✓ Adequate
Microcontroller     Teensy 4.1         None              ✓ Locked
```

### Rationale

**ICM-20948 (Primary):**
- ✓ Proven in flight (multiple successful missions)
- ✓ Good balance of range and precision
- ✓ Well-supported library and community
- ✓ Magnetometer enables full 9-DOF

**KX134 (Backup):**
- ✓ Proven as redundant accelerometer
- ✓ ±64G capability provides true high-G failover
- ✓ Simple I2C protocol
- ✓ Clear failover logic in IMUManager

**MS5611 (Barometer):**
- ✓ Proven flight history in TripleT
- ✓ Adequate ±1m altitude accuracy
- ✓ Robust I2C protocol
- ✓ No need to change what works

**u-blox NEO-8M (GPS):**
- ✓ Proven in flight
- ✓ Good position/altitude accuracy
- ✓ 10 Hz update rate sufficient
- ✓ Reliable lock before launch

---

### Future Evaluation Path (v1.1.0 and beyond)

**Near-term (3-6 months):**
1. Flight evaluate BMP388 barometer (better precision)
2. Evaluate BNO085 for future quaternion-only variant
3. Assess cost/benefit of ICM-20649 (eliminate KX134?)

**Medium-term (6-12 months):**
1. Consider NEO-M8N GPS if extra precision needed
2. Evaluate ST Microelectronics sensors if component shortages
3. Test extreme-altitude variants (BMP390L) for high-power rockets

**Long-term (>12 months):**
1. Monitor emerging sensor technology
2. Plan migration to next-generation IMU (when available)
3. Consider modular sensor board design

---

### Cost Analysis

**Current Hardware Cost (v0.9.0):**
```
Component           Unit Cost       Qty    Total
─────────────────────────────────────────────────
ICM-20948 IMU       $20.00          1      $20.00
KX134 Accel         $12.00          1      $12.00
MS5611 Baro         $4.00           1      $4.00
NEO-8M GPS          $40.00          1      $40.00
Teensy 4.1          $25.00          1      $25.00
PCB + Assembly      $50.00          1      $50.00
Connectors/Misc     $20.00          1      $20.00
─────────────────────────────────────────────────
TOTAL SENSOR COST:                          $171.00
```

**Alternative Configurations:**

```
Option 1 (BNO085 + KX134):
  BNO085 (Primary)     $30.00
  KX134 (Backup)       $12.00
  Other sensors        $44.00
  TOTAL:              $176.00 (+$5, marginal)

Option 2 (All redundancy, dual BNO085):
  BNO085 (Primary)     $30.00
  BNO085 (Backup)      $30.00
  Other sensors        $44.00
  TOTAL:              $194.00 (+$23, not recommended)

Option 3 (Budget: Single sensor):
  ICM-20948           $20.00
  MS5611              $4.00
  NEO-8M              $40.00
  (No backup IMU)
  Other               $50.00
  TOTAL:              $114.00 (-$57, NO REDUNDANCY)
```

**Recommendation:** Stay with current configuration (cost optimal + redundant)

---

## References & Datasheets

### Primary Documents
- ICM-20948: [TDK InvenSense datasheet](https://invensense.tdk.com/)
- BNO085: [Bosch Sensortec](https://www.bosch-sensortec.com/)
- KX134: [Rohm Semiconductor](https://www.rohm.com/)
- MS5611: [TE Connectivity](https://www.te.com/)
- NEO-8M: [u-blox](https://www.u-blox.com/)

### Evaluation Tools
- CAD model compatibility
- Power budget analysis
- PCB layout review
- Flight data comparison

---

## Approval Matrix

For sensor changes to v1.0.0 release:

```
Change              Authority      Status      Notes
────────────────────────────────────────────────────────
Hardware swap       PM review       APPROVED    Follow migration steps
New sensor variant  Technical lead  PENDING     Field test required
Cost increase > 10% Budget owner    PENDING     Justify on performance
```

---

**For detailed sensor integration, see:**
- `DEVELOPER_GUIDE.md` - Adding new sensors (BNO055 example)
- `src/sensors/imu_interface.h` - Interface definition
- `src/sensors/sensor_factory.h` - Factory pattern
- Sensor datasheets (links above)
