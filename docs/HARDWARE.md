# Hardware Requirements

**Version:** v0.51  
**Target Platform:** Teensy 4.1

## Core Components

### Microcontroller
- **Teensy 4.1** microcontroller (ARM Cortex-M7 @ 600MHz)
  - 512KB RAM, 7.75MB Flash
  - Built-in SDIO interface for SD card
  - Multiple hardware serial ports
  - I2C, SPI interfaces

### Required Sensors

#### Inertial Measurement
- **ICM-20948** 9-DOF IMU
  - 3-axis accelerometer (±16G range for normal flight)
  - 3-axis gyroscope
  - 3-axis magnetometer
  - I2C interface
  - Temperature sensor

- **KX134** High-G Accelerometer
  - 3-axis accelerometer (±64G range for high-G events)
  - I2C interface
  - Used for launch detection and high-acceleration phases

#### Environmental Sensors
- **MS5611** Barometric Pressure Sensor
  - High-precision pressure and temperature measurement
  - Primary altitude reference
  - I2C interface

#### Navigation
- **u-blox GPS Module** (e.g., ZOE-M8Q, SAM-M8Q)
  - UBX protocol configured
  - GPS + GLONASS recommended
  - I2C or UART interface
  - External antenna connection

### Data Storage
- **SD Card** (FAT32 formatted)
  - Class 10 or better recommended
  - 8GB+ capacity
  - Uses Teensy 4.1 built-in SDIO interface

### User Interface & Feedback

#### Visual Indicators
- **NeoPixel LEDs** (2x recommended)
  - WS2812B or compatible
  - Flight state indication
  - Recovery strobe pattern
  - Connected to `NEOPIXEL_PIN` (default: Pin 2)

#### Audio Feedback
- **Buzzer** (Piezo or magnetic)
  - State change notifications
  - SOS recovery beacon
  - Error alerts
  - Connected to `BUZZER_PIN` (default: Pin 9)

### Deployment System
- **Pyrotechnic Channels** (2x)
  - Drogue parachute deployment (Pin 2, default)
  - Main parachute deployment (Pin 3, default)
  - MOSFETs and current limiting resistors required
  - Proper electrical isolation essential

### Attitude Control (Optional)
- **Servo Actuators** (3x)
  - Pitch control: `ACTUATOR_PITCH_PIN` (default: Pin 21)
  - Roll control: `ACTUATOR_ROLL_PIN` (default: Pin 23)
  - Yaw control: `ACTUATOR_YAW_PIN` (default: Pin 20)
  - Standard PWM servo interface

## Pin Configuration

### Digital Pins
| Pin | Function | Component | Notes |
|-----|----------|-----------|-------|
| 2 | NeoPixel Data | LED Strip | WS2812B compatible |
| 9 | Buzzer | Audio Feedback | PWM capable |
| 2 | Pyro Channel 1 | Drogue Deploy | High current capability |
| 3 | Pyro Channel 2 | Main Deploy | High current capability |
| 20 | Actuator Yaw | Servo Control | PWM output |
| 21 | Actuator Pitch | Servo Control | PWM output |
| 23 | Actuator Roll | Servo Control | PWM output |

### I2C Interface
| Pin | Function | Notes |
|-----|----------|-------|
| 18 | SDA | I2C Data Line |
| 19 | SCL | I2C Clock Line |

**I2C Devices:**
- ICM-20948 IMU
- KX134 Accelerometer  
- MS5611 Barometer
- GPS Module (if I2C variant)

### Serial Interfaces
| Port | Function | Component |
|------|----------|-----------|
| Serial1 | GPS Communication | u-blox GPS |
| Serial (USB) | Debug/Commands | Computer Interface |

### Analog Pins
| Pin | Function | Component | Notes |
|-----|----------|-----------|-------|
| A7 | Battery Voltage | Voltage Divider | Optional monitoring |

### SD Card
- **Built-in SDIO Interface** (Teensy 4.1)
- No additional pins required
- High-speed data logging

## Power Requirements

### Supply Voltage
- **Primary**: 3.3V regulated supply
- **Input Range**: 3.0V - 3.6V (depending on sensors)
- **Current Consumption**: ~200-500mA (depending on active components)

### Power Distribution
- **Sensors**: 3.3V regulated supply required
- **Servos**: May require separate 5V supply
- **Pyro Channels**: Separate high-current supply for deployment charges

### Battery Monitoring
- **Voltage Divider** circuit for battery voltage sensing
- **Configuration**:
  - `VOLTAGE_DIVIDER_R1`: Upper resistor value
  - `VOLTAGE_DIVIDER_R2`: Lower resistor value
  - Formula: `V_battery = V_adc * (R1 + R2) / R2`
  - Ensure `V_adc ≤ 3.3V` (ADC reference voltage)

## Circuit Design Considerations

### I2C Bus
- **Pull-up Resistors**: 4.7kΩ on SDA and SCL lines
- **Bus Speed**: 400kHz (Fast I2C) recommended
- **Address Conflicts**: Verify no I2C address conflicts between sensors

### Power Supply
- **Filtering**: Adequate filtering capacitors for sensor supplies
- **Isolation**: Separate analog and digital supplies if possible
- **Backup Power**: Consider backup power for critical systems during deployment

### Pyrotechnic Safety
- **Current Limiting**: Appropriate current limiting for deployment charges
- **Isolation**: Optical isolation recommended for pyro channels
- **Safety Switches**: Hardware safety switches for deployment circuits
- **Test Points**: Provision for continuity testing without firing

### Environmental Protection
- **Conformal Coating**: Protect circuits from moisture and debris
- **Shock Mounting**: Secure mounting considering high-G forces
- **Thermal Management**: Consider temperature extremes during flight

## Assembly Notes

### Sensor Mounting
- **IMU Alignment**: Mount ICM-20948 with axes aligned to rocket body frame
- **Vibration Isolation**: Consider soft mounting for sensitive sensors
- **Magnetic Interference**: Keep magnetometer away from current-carrying conductors

### Wiring
- **Wire Gauge**: Appropriate gauge for current requirements
- **Strain Relief**: Secure connections against vibration and shock
- **Redundancy**: Consider redundant wiring for critical functions

### Testing
- **Continuity Testing**: Verify all connections before integration
- **Sensor Verification**: Use `scan_i2c` command to verify sensor detection
- **Pyro Testing**: Safe testing procedures for deployment circuits

## Recommended Development Setup

### Breadboard/Prototype
- **Teensy 4.1** on breadboard or development board
- **Sensor Breakout Boards**: Use commercial breakout boards for initial testing
- **Level Shifters**: If mixing 3.3V and 5V components
- **External Supplies**: Bench power supplies for development

### PCB Design (Future)
- **Custom PCB**: Integrated design for production units
- **Connector Standards**: Standardized connectors for sensors and actuators
- **Test Points**: Adequate test points for debugging and calibration
- **Documentation**: Complete schematic and layout documentation

This hardware configuration provides a robust foundation for comprehensive flight computer capabilities with extensive sensor fusion, reliable deployment systems, and comprehensive data logging.