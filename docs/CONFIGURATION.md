# Configuration Guide

**Version:** v0.51

## Overview

The firmware is highly configurable through parameters in `src/config.h`. This document covers key configuration options and their effects on system behavior.

## Flight Parameters

### Launch Detection
```cpp
#define BOOST_ACCEL_THRESHOLD 2.0f  // G-force threshold for liftoff detection
```

### Motor Burnout Detection
```cpp
#define COAST_ACCEL_THRESHOLD 0.5f  // G-force threshold for burnout detection
```

### Apogee Detection
```cpp
#define APOGEE_CONFIRMATION_COUNT 5          // Barometer readings to confirm apogee
#define APOGEE_ACCEL_CONFIRMATION_COUNT 5    // Accelerometer readings for apogee
#define APOGEE_GPS_CONFIRMATION_COUNT 3      // GPS readings for apogee
#define BACKUP_APOGEE_TIME_MS 20000         // Failsafe timer (20 seconds)
```

### Parachute Deployment
```cpp
#define MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M 100  // Default main deployment altitude AGL
#define DROGUE_PRESENT true                    // Enable drogue parachute
#define MAIN_PRESENT true                      // Enable main parachute
```

## Hardware Configuration

### Sensor Control
```cpp
#define USE_KX134 1           // Enable KX134 high-G accelerometer
#define BUZZER_OUTPUT 1       // Enable buzzer functionality
#define NEOPIXEL_COUNT 2      // Number of status LEDs
```

### Pin Assignments
```cpp
#define NEOPIXEL_PIN 2        // NeoPixel data pin
#define BUZZER_PIN 9          // Buzzer control pin
#define PYRO_CHANNEL_1 2      // Drogue deployment pin
#define PYRO_CHANNEL_2 3      // Main deployment pin
#define ACTUATOR_PITCH_PIN 21 // Pitch servo pin
#define ACTUATOR_ROLL_PIN 23  // Roll servo pin
#define ACTUATOR_YAW_PIN 20   // Yaw servo pin
```

## PID Controller Gains

### Roll Axis
```cpp
#define PID_ROLL_KP 1.0f      // Proportional gain
#define PID_ROLL_KI 0.1f      // Integral gain
#define PID_ROLL_KD 0.05f     // Derivative gain
```

### Pitch Axis
```cpp
#define PID_PITCH_KP 1.0f     // Proportional gain
#define PID_PITCH_KI 0.1f     // Integral gain
#define PID_PITCH_KD 0.05f    // Derivative gain
```

### Yaw Axis
```cpp
#define PID_YAW_KP 0.5f       // Proportional gain
#define PID_YAW_KI 0.05f      // Integral gain
#define PID_YAW_KD 0.02f      // Derivative gain
```

## Recovery System

### SOS Beacon Timing
```cpp
#define RECOVERY_BEACON_SOS_DOT_MS 200        // Dot duration
#define RECOVERY_BEACON_SOS_DASH_MS 600       // Dash duration
#define RECOVERY_BEACON_SOS_SYMBOL_PAUSE_MS 200   // Between symbols
#define RECOVERY_BEACON_SOS_LETTER_PAUSE_MS 600   // Between letters
#define RECOVERY_BEACON_SOS_WORD_PAUSE_MS 1400    // Between words
#define RECOVERY_BEACON_FREQUENCY_HZ 2000     // Buzzer frequency
```

### LED Strobe Pattern
```cpp
#define RECOVERY_STROBE_ON_MS 100             // Strobe on duration
#define RECOVERY_STROBE_OFF_MS 900            // Strobe off duration
#define RECOVERY_STROBE_BRIGHTNESS 255        // LED brightness
#define RECOVERY_STROBE_R 0                   // Red component
#define RECOVERY_STROBE_G 255                 // Green component
#define RECOVERY_STROBE_B 0                   // Blue component
```

### GPS Beacon
```cpp
#define RECOVERY_GPS_BEACON_INTERVAL_MS 10000 // GPS coordinate output interval
```

## Battery Monitoring

### Enable/Disable
```cpp
#define ENABLE_BATTERY_MONITORING 1           // 1 to enable, 0 to disable
```

### Hardware Configuration
```cpp
#define BATTERY_VOLTAGE_PIN A7                // Analog input pin
#define ADC_REFERENCE_VOLTAGE 3.3f            // Teensy 4.1 ADC reference
#define ADC_RESOLUTION 1024.0f                // 10-bit ADC resolution
```

### Voltage Divider
```cpp
#define VOLTAGE_DIVIDER_R1 10000.0f           // Upper resistor (Ohms)
#define VOLTAGE_DIVIDER_R2 10000.0f           // Lower resistor (Ohms)
```

### Update Rate
```cpp
#define BATTERY_VOLTAGE_READ_INTERVAL_MS 5000 // Read interval
```

## Safety & Error Handling

### Error Thresholds
```cpp
#define MAX_SENSOR_FAILURES 3                // Failure threshold for ERROR state
```

### EEPROM Configuration
```cpp
#define EEPROM_STATE_ADDR 0                  // Flight state data address
#define EEPROM_SIGNATURE_VALUE 0xBEEF        // Data validity signature
#define MAG_CAL_EEPROM_ADDR 100             // Magnetometer calibration address
```

### Timing Constants
```cpp
#define LANDING_CONFIRMATION_TIME_MS 5000    // Landing confirmation delay
#define LANDED_TIMEOUT_MS 30000             // Transition to RECOVERY delay
```

## Altitude Thresholds

### Landing Detection
```cpp
#define LANDING_ALTITUDE_THRESHOLD_M 50.0f   // Landing altitude threshold
#define LANDING_ACCEL_MIN_G 0.8f            // Minimum acceleration for landing
#define LANDING_ACCEL_MAX_G 1.2f            // Maximum acceleration for landing
```

## Data Logging

### SD Card Control
```cpp
#define DISABLE_SDCARD_LOGGING false         // Set true to disable logging
```

## Configuration Guidelines

### Sensor Thresholds
- **Launch Detection**: Adjust `BOOST_ACCEL_THRESHOLD` based on expected motor acceleration
- **Burnout Detection**: Set `COAST_ACCEL_THRESHOLD` to detect motor cutoff
- **Apogee Confirmation**: Higher counts provide more reliability but slower response

### PID Tuning
- **Start Conservative**: Begin with lower gains and increase gradually
- **Proportional**: Primary response gain - affects speed of correction
- **Integral**: Eliminates steady-state error - use sparingly to avoid windup
- **Derivative**: Provides damping - helps prevent overshoot

### Recovery System
- **Audio Pattern**: Adjust timing for local regulations and preferences
- **Visual Pattern**: Modify colors and timing for visibility conditions
- **GPS Interval**: Balance between battery life and update frequency

### Battery Monitoring
- **Voltage Divider**: Calculate based on maximum battery voltage
- **Formula**: `V_battery = V_adc * (R1 + R2) / R2`
- **Safety**: Ensure `V_adc ≤ 3.3V` to protect Teensy ADC

## Testing Configurations

### Bench Testing
```cpp
#define BOOST_ACCEL_THRESHOLD 0.1f   // Lower threshold for shake testing
#define COAST_ACCEL_THRESHOLD 0.05f  // Lower threshold for gentle movement
```

### Ground Testing
```cpp
#define BACKUP_APOGEE_TIME_MS 5000   // Shorter timer for testing
#define LANDED_TIMEOUT_MS 10000      // Faster transition to recovery
```

## Production vs Development

### Development Settings
- Lower acceleration thresholds for testing
- Shorter timeouts for faster iteration
- Enhanced debug output enabled
- Relaxed sensor requirements

### Production Settings
- Flight-tested acceleration thresholds
- Conservative safety margins
- Optimized for flight performance
- Minimal debug output for performance

## Common Configuration Issues

### Sensor Detection
- Verify I2C addresses don't conflict
- Check pull-up resistor values (4.7kΩ recommended)
- Ensure proper power supply voltages

### PID Instability
- Start with derivative gain = 0
- Increase proportional gain until oscillation
- Add derivative gain to dampen oscillation
- Add integral gain carefully to eliminate offset

### Recovery System
- Test audio patterns for local noise ordinances
- Verify LED visibility in expected recovery conditions
- Adjust GPS beacon interval for battery life requirements

This configuration system provides extensive customization while maintaining safe default values for typical rocket applications.