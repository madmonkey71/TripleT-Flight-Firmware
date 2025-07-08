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

## Advanced Guidance Configuration
This section covers parameters related to newer guidance features, including stability failsafes and trajectory following. These values are defined in `src/config.h`.

### Attitude PID Output & Integral Limits
These limits apply to the primary attitude stabilization PIDs (Roll, Pitch, Yaw).
```cpp
#define PID_OUTPUT_MIN -1.0f // Min actuator command (normalized, e.g., -1.0 for full negative deflection)
#define PID_OUTPUT_MAX  1.0f // Max actuator command (normalized, e.g., +1.0 for full positive deflection)

#define PID_INTEGRAL_LIMIT_ROLL 0.5f  // Anti-windup limit for the roll PID integral term
#define PID_INTEGRAL_LIMIT_PITCH 0.5f // Anti-windup limit for the pitch PID integral term
#define PID_INTEGRAL_LIMIT_YAW 0.3f   // Anti-windup limit for the yaw PID integral term
```

### Guidance Failsafe Mechanisms
These parameters control the automatic detection of unstable flight conditions. If a failsafe condition is met (a threshold is exceeded for `STABILITY_VIOLATION_DURATION_MS`), the system may transition to an `ERROR` state (logging error code `GUIDANCE_STABILITY_FAIL`) and disengage active guidance.

#### Maximum Control Surface Deflection Limits (Informational for Stability Monitoring)
These values define the expected maximum deflections for control surfaces in degrees. The stability monitor checks if commanded outputs (normalized PID outputs) represent a large percentage of these maximums (see `STABILITY_ACTUATOR_SATURATION_LEVEL_PERCENT`), which could indicate the system is struggling to maintain control.
```cpp
#define MAX_FIN_DEFLECTION_PITCH_DEG 15.0f // Expected max deflection for pitch control surfaces (degrees)
#define MAX_FIN_DEFLECTION_YAW_DEG   15.0f // Expected max deflection for yaw control surfaces (degrees)
#define MAX_FIN_DEFLECTION_ROLL_DEG  20.0f // Expected max deflection for roll control surfaces (degrees, if applicable)
```

#### Stability Monitoring Thresholds
```cpp
#define STABILITY_MAX_PITCH_RATE_DPS    180.0f // Max pitch angular velocity (degrees per second)
#define STABILITY_MAX_ROLL_RATE_DPS     360.0f // Max roll angular velocity (degrees per second)
#define STABILITY_MAX_YAW_RATE_DPS      180.0f // Max yaw angular velocity (degrees per second)

#define STABILITY_MAX_ATTITUDE_ERROR_PITCH_DEG 20.0f // Max allowable error between target and actual pitch (degrees)
#define STABILITY_MAX_ATTITUDE_ERROR_ROLL_DEG  30.0f // Max allowable error between target and actual roll (degrees)
#define STABILITY_MAX_ATTITUDE_ERROR_YAW_DEG   20.0f // Max allowable error between target and actual yaw (degrees)

// This checks if actuator commands are consistently at a high percentage of their maximum possible output (PID_OUTPUT_MAX),
// indicating control saturation or insufficient control authority.
#define STABILITY_ACTUATOR_SATURATION_LEVEL_PERCENT 90.0f // Actuator output as percentage of PID_OUTPUT_MAX
```

#### Common Failsafe Duration
```cpp
#define STABILITY_VIOLATION_DURATION_MS 500 // Duration (milliseconds) a condition must persist to trigger a failsafe
```

### Trajectory Following (Path Guidance) Configuration
Parameters for the trajectory following feature. This feature allows the rocket to follow a pre-defined series of waypoints. Currently, it uses a simplified go-to-waypoint approach for horizontal and vertical guidance by adjusting the targets for the main attitude PIDs.

#### General Trajectory Parameters
```cpp
#define MAX_TRAJECTORY_WAYPOINTS 50      // Maximum number of waypoints that can be defined in a single trajectory.
#define DEFAULT_WAYPOINT_ACCEPTANCE_RADIUS_M 10.0f // Distance (meters) to a target waypoint to consider it "reached" and advance to the next.
```

#### Trajectory Heading PID Controller Gains
This PID controller aims to minimize the heading error between the vehicle's current yaw and the bearing to the target waypoint. *(Note: Current simple implementation sets `target_yaw_rad_g` directly to bearing. These PID gains are for a more advanced XTE controller not yet fully active for yaw target setting, but are available for tuning if that logic is enabled/refined).*
```cpp
#define TRAJ_XTE_PID_KP 0.5f     // Proportional gain for trajectory heading control
#define TRAJ_XTE_PID_KI 0.05f    // Integral gain
#define TRAJ_XTE_PID_KD 0.01f    // Derivative gain
#define TRAJ_XTE_PID_INTEGRAL_LIMIT 0.2f // Anti-windup for XTE PID integral term
#define TRAJ_XTE_PID_OUTPUT_LIMIT 0.5f   // Max output for this PID (e.g., radians for yaw adjustment or target).
```

#### Trajectory Altitude PID Controller Gains
This PID controller aims to minimize the altitude error between the vehicle's current altitude (MSL) and the target waypoint's altitude (MSL). Its output directly sets the `target_pitch_rad_g` for the primary attitude (pitch) PID controller.
```cpp
#define TRAJ_ALT_PID_KP 0.3f     // Proportional gain for trajectory altitude control
#define TRAJ_ALT_PID_KI 0.03f    // Integral gain
#define TRAJ_ALT_PID_KD 0.01f    // Derivative gain
#define TRAJ_ALT_PID_INTEGRAL_LIMIT 0.2f // Anti-windup for Altitude PID integral term
#define TRAJ_ALT_PID_OUTPUT_LIMIT 0.3f   // Max output (radians, e.g., approx +/-17 degrees for target_pitch_rad_g)
```

## Guidance System Configuration

The TripleT Flight Firmware includes an advanced guidance and control system that can provide active flight stabilization using servo-controlled fins or canards. This system can be completely disabled for passive rockets that don't have actuators.

### Guidance Enable/Disable

**Location:** `src/config.h`

```cpp
// --- Guidance System Configuration ---
// Enable/disable the entire guidance system including servos and stability control
// Set to 1 to enable guidance (requires servos/actuators), 0 to disable for passive rockets
#define ENABLE_GUIDANCE 1  // 1=Enable guidance system, 0=Disable for passive flights
```

### Configuration Options

#### Enabled Guidance (`ENABLE_GUIDANCE 1`)
When guidance is enabled, the system will:
- Initialize servo/actuator objects for pitch, roll, and yaw control
- Run PID controllers for attitude stabilization
- Perform stability monitoring during BOOST and COAST phases
- Command actuators during appropriate flight phases
- Log guidance-related data (target angles, PID integrals, actuator outputs)
- Monitor guidance stability and transition to ERROR state if compromised

**Hardware Requirements:**
- Servo-controlled fins or canards connected to:
  - Pitch actuator: Pin 21 (`ACTUATOR_PITCH_PIN`)
  - Roll actuator: Pin 23 (`ACTUATOR_ROLL_PIN`) 
  - Yaw actuator: Pin 20 (`ACTUATOR_YAW_PIN`)
- ICM20948 IMU for orientation feedback
- Optional: GPS for navigation guidance

#### Disabled Guidance (`ENABLE_GUIDANCE 0`)
When guidance is disabled, the system will:
- Skip all guidance-related initialization
- Disable servo/actuator control
- Skip stability monitoring and PID calculations
- Zero out guidance-related log fields
- Operate as a passive rocket with full data logging capability

**Use Cases:**
- Small rockets without room for actuators
- Initial test flights before adding guidance hardware
- Backup/recovery mode for guidance hardware failures
- Research flights focused on aerodynamics without control

### PID Controller Configuration

**Location:** `src/config.h`

When guidance is enabled, you can tune the PID controllers:

```cpp
// --- PID Controller Gains ---

// Roll Axis PID
#define PID_ROLL_KP 1.0f
#define PID_ROLL_KI 0.1f
#define PID_ROLL_KD 0.05f

// Pitch Axis PID
#define PID_PITCH_KP 1.0f
#define PID_PITCH_KI 0.1f
#define PID_PITCH_KD 0.05f

// Yaw Axis PID
#define PID_YAW_KP 0.8f
#define PID_YAW_KI 0.08f
#define PID_YAW_KD 0.03f
```

### Actuator Configuration

**Location:** `src/config.h`

```cpp
// --- Actuator Configuration ---
#define ACTUATOR_PITCH_PIN 21 // Teensy pin 21
#define ACTUATOR_ROLL_PIN  23 // Teensy pin 23
#define ACTUATOR_YAW_PIN   20 // Teensy pin 20
#define SERVO_MIN_PULSE_WIDTH 1000 // Microseconds
#define SERVO_MAX_PULSE_WIDTH 2000 // Microseconds
#define SERVO_DEFAULT_ANGLE 90     // Default angle for servos (degrees)
```

### Active Flight Phases

When guidance is enabled, active control occurs during:
- **COAST**: Attitude hold based on orientation at motor burnout
- **DROGUE_DESCENT**: Continued stabilization under drogue chute
- **MAIN_DESCENT**: Stabilization under main parachute

### Safety Features

- **Stability Monitoring**: Continuous monitoring of attitude errors and rates
- **Error Detection**: Automatic transition to ERROR state if stability is compromised
- **Sensor Validation**: Guidance requires healthy ICM20948 IMU
- **Ground Safety**: No actuator commands when rocket is stationary on ground

### Migration Guide

To convert from guided to passive rocket configuration:
1. Set `ENABLE_GUIDANCE 0` in `src/config.h`
2. Remove servo hardware if desired
3. Recompile and flash firmware
4. All other functionality (logging, recovery, etc.) remains unchanged

To add guidance to an existing passive rocket:
1. Install servo-controlled fins/canards
2. Connect servos to designated pins
3. Set `ENABLE_GUIDANCE 1` in `src/config.h`
4. Tune PID parameters for your specific airframe
5. Test guidance on ground before flight