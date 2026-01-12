# Competitive Feature Analysis

This document evaluates the current TripleT Flight Firmware against the specified set of competitive features.

## Altimeter Data Options

| Feature | Requirement | Current Status | Notes |
| :--- | :--- | :--- | :--- |
| **Field Data Output** | Beeps, AltOS Software | **Partial** | **Beeps:** Supported (SOS, State indicators).<br>**AltOS:** Not supported. Project uses a custom Web Interface (telemetry currently unimplemented). |
| **Records Peak Altitude** | Yes | **Yes** | Tracks `g_maxAltitudeReached` and records in log. |
| **Records Peak Speed** | Yes | **Yes** | Recorded in CSV log (`Speed` from GPS). |
| **Records Peak/Avg Accel** | Yes | **Yes** | Instantaneous acceleration (IMU & High-G) recorded in CSV log. Peak/Average deriveable. |
| **Records Booster Burn Time** | Derivable | **Yes** | Derivable from log state transitions (`BOOST` -> `COAST`). |
| **Records Coast to Apogee Time** | Derivable | **Yes** | Derivable from log state transitions (`COAST` -> `APOGEE`). |
| **Records Apogee to Ejection** | Derivable | **Yes** | Derivable from log state transitions (`APOGEE` -> `DROGUE_DEPLOY`/`MAIN_DEPLOY`). |
| **Records Time to Apogee** | Derivable | **Yes** | Derivable from log timestamps. |
| **Records Ejection Altitudes** | Derivable | **Yes** | Derivable from log data at ejection events. |
| **Records Ejection Times** | Derivable | **Yes** | Derivable from log timestamps. |
| **Records Flight Duration** | Derivable | **Yes** | Derivable from log (`LIFTOFF` to `LANDED`). |
| **Displays Current Altitude** | via AltOS Software | **No** | Live telemetry is currently "NOT STARTED" per gap analysis. System has GPS/Baro but cannot transmit real-time data to ground station yet. |
| **# of Flights Stored** | 40min | **Exceeds** | Uses SD Card (multi-GB capacity), capable of storing hours of flight data. |
| **Max Altitude** | 100,000ft (30480 m) | **Yes** | MS5611 barometer operational range exceeds 100k ft (approx 30km limit). |
| **Sampling Rate** | 100/ascent; 10/descent | **No** | Current global `LOG_INTERVAL` is 100ms (10Hz). High-speed ascent logging (100Hz) is not currently configured. |
| **Measurement Units** | Imperial / Metric | **Metric** | Firmware uses Metric units internally (meters, m/s). Log output is Metric. |

## Item Functions

| Feature | Requirement | Current Status | Notes |
| :--- | :--- | :--- | :--- |
| **Barometric Pressure Sensor** | Yes | **Yes** | MS5611 implemented. |
| **Accelerometer** | 1-axis 200G (Motor)<br>3-axis 16G (Gyro Cal) | **Partial** | **High-G:** KX134 implemented, but range is set to 64g, not 200g.<br>**IMU:** ICM-20948 implemented (16g 3-axis). |
| **Gyro Tilt Sensors** | 3-axis 2000 deg/sec | **Yes** | ICM-20948 Gyro implemented (config supports 2000 dps range, currently set to 250 dps for precision). |
| **Magnetic Directional Sensor** | 6-axis IMU | **Yes** | ICM-20948 Magnetometer implemented (9-axis total with Accel/Gyro). |
| **Event Timer** | Yes | **Yes** | System millis and state entry timers implemented. |
| **Telemetry** | 70cm ham-band | **No** | Hardware support exists (ESP32 placeholders), but firmware logic is "NOT STARTED". |
| **Field Locator** | GPS Telemetry | **Partial** | GPS is implemented and logged to SD card. Beacon mode (Audio SOS/Strobe) implemented. Real-time location transmission is missing. |

## Altimeter Dual Deployment Capability

| Feature | Requirement | Current Status | Notes |
| :--- | :--- | :--- | :--- |
| **Dual-Deployment Capable?** | Yes | **Yes** | `DROGUE_DEPLOY` and `MAIN_DEPLOY` states implemented. |
| **Dual Deployment Pyro Channels** | 6 | **No** | Configuration defines 2 channels (`PYRO_CHANNEL_1`, `PYRO_CHANNEL_2`). Requirement asks for 6. |
| **Peak Velocity Eject Option?** | Yes | **No** | Deployment logic is Apogee-based or Altitude-based. No velocity-trigger for ejection. |
| **Apogee Eject Option?** | Yes | **Yes** | Standard drogue deployment at Apogee (`detectApogee`). |
| **Descending Altitude Eject** | Yes | **Yes** | Main deployment at configurable altitude (`g_main_deploy_altitude_m_agl`). |
| **Timer Eject Option?** | Yes | **Yes** | Backup timer `BACKUP_APOGEE_TIME_MS` triggers apogee event if other sensors fail. |
| **Manual Eject Option?** | No | **No** | No manual fire command found in command processor. |
| **Configurable Velocity Eject** | Yes | **No** | No configurable velocity threshold for ejection. |
| **Time Delay for Redundancy** | Configurable | **Yes** | `APOGEE_DELAY` is defined in config. |
