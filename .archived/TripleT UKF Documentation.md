# UKF Implementation Documentation

## Overview

The TripleT Flight Firmware now incorporates an Unscented Kalman Filter (UKF) for sensor fusion. This implementation fuses acceleration data from two separate accelerometers (KX134 and ICM20948) to provide more accurate and reliable measurements of position, velocity, and acceleration in the vertical axis.

## UKF State Variables

The UKF implementation tracks a state vector with three components:

1. **Position** (meters) - Vertical position relative to starting point
2. **Velocity** (meters/second) - Vertical velocity
3. **Acceleration** (meters/second²) - Vertical acceleration

## Implementation Details

### Core Components

- **State Vector** (`x_[3]`): Stores the current estimates for position, velocity, and acceleration
- **Covariance Matrix** (`P_[3][3]`): Represents the uncertainty in the state estimates
- **Process Noise** (`Q_[3][3]`): Models the noise in the state transition (physics model)
- **Measurement Noise** (`R_[2][2]`): Models the noise in sensor readings
  - KX134 measurement noise: 0.5
  - ICM20948 measurement noise: 0.8 (slightly higher uncertainty)

### Key Parameters

- **Processing Rate**: 20Hz (50ms intervals)
- **Alpha**: 0.3 (Controls spread of sigma points)
- **Beta**: 2.0 (Optimal for Gaussian distributions)
- **Kappa**: 0.0 (Secondary scaling parameter)

### Data Flow

1. Raw acceleration data is collected from both the KX134 and ICM20948 sensors
2. The UKF processes these measurements at regular intervals (20Hz)
3. The filter predicts the state using a physics model (position, velocity, acceleration)
4. Measurements are incorporated to correct the predictions
5. The updated state estimates are available through getter methods

### Physics Model

The implementation uses a simple physics model for state transition:
- Position += velocity * dt + 0.5 * acceleration * dt²
- Velocity += acceleration * dt
- Acceleration remains constant between updates (plus process noise)

## Available Data

The UKF implementation provides three key outputs:

1. **Position** (`ukf.getPosition()`)
   - In meters, relative to the initialization point
   - Provides integrated position based on acceleration and velocity

2. **Velocity** (`ukf.getVelocity()`)
   - In meters per second
   - More accurate than simple differentiation of barometer readings
   - Available via `getUkfVelocity()` function

3. **Acceleration** (`ukf.getAcceleration()`)
   - In meters per second squared
   - Fused from both accelerometer sensors
   - Available via `getUkfAcceleration()` function

## Data Logging

The UKF data is included in the flight logs as:
- `UKF_Position` - Estimated vertical position (m)
- `UKF_Velocity` - Estimated vertical velocity (m/s)
- `UKF_Acceleration` - Estimated vertical acceleration (m/s²)

## Next Steps and Considerations

1. **Altitude Fusion**: Currently, the UKF only processes accelerometer data. Future enhancements could incorporate barometric altitude measurements for improved accuracy.

2. **Performance Assessment**: Compare UKF velocity estimates with those calculated from barometric altitude differences to evaluate relative accuracy.

3. **Tuning Opportunities**: The process and measurement noise parameters (Q and R matrices) could be further tuned based on field data.

4. **Extended State Vector**: The current implementation tracks only the vertical axis. This could be extended to 6 or 9 states for full 3D tracking.

5. **State Initialization**: Currently initialized with zero position and velocity. Consider initializing with barometric data.

6. **Gravity Compensation**: The implementation could benefit from explicit gravity compensation rather than treating it as part of the acceleration.

7. **Failure Mode Handling**: Add explicit handling of sensor failures and divergence detection.

## Conclusion

The UKF implementation provides enhanced state estimation for flight trajectory analysis. The fused data offers potentially more accurate velocity and position information than what's available from single-sensor sources, particularly in high-dynamic situations where barometric altitude readings may lag or have noise issues. This data will be valuable for more precise apogee detection, deployment timing, and post-flight analysis.