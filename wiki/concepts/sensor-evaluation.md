---
title: Sensor Selection & Evaluation
type: concept
tags: [sensors, hardware-evaluation, migration, alternatives]
created: 2026-04-22
updated: 2026-04-22
related_files: [src/sensors/imu_interface.h, src/sensors/sensor_factory.h, src/sensors/bno085_sensor.h, docs/SENSOR_EVALUATION.md]
---

Rationale for the current sensor loadout (ICM-20948 + KX134 + MS5611 + u-blox GNSS), shortlist of alternatives evaluated, and upgrade paths. Hardware details are in [[entities/hardware-platform]]; this page focuses on *why* these parts and what could replace them.

## Current Loadout

| Role | Part | Reason chosen |
|------|------|---------------|
| Primary IMU | ICM-20948 | 9-DoF, magnetometer built in, ±16 g range sufficient for most phases, well-supported SparkFun driver |
| Backup accel | KX134 | ±64 g covers extreme boost + hard landings; cheap; simple I2C interface |
| Barometric altimeter | MS5611 | Low noise, wide altitude range (0–50 km), robust I2C |
| GNSS | u-blox NEO-M8 / ZOE-M8Q | Good fix under cover, MSL altitude, time sync |

Design decision: keep ICM + KX134 as *two physical accels* so that a saturation event on the primary doesn't blind apogee detection; the KX134 catches the high-g window.

## Alternatives Considered

| Alternative | For | Pros | Cons |
|-------------|-----|------|------|
| **BNO085** | Primary IMU | Hardware sensor fusion (gives quaternion directly); no Kalman needed on MCU | Proprietary fusion is opaque; evaluation stub exists (`src/sensors/bno085_sensor.h`) but not wired in |
| **ICM-20649** | Primary IMU | ±30 g accel (higher than 20948) | No onboard mag; would require separate magnetometer |
| **BMP388** | Barometer | Better precision + lower noise than MS5611 | Similar power/size; minor improvement only |

## Migration Path Model

Because sensors sit behind `IMUInterface` and are selected via [[concepts/sensor-redundancy|`sensor_factory`]], swapping primary or backup requires:

1. Implement the adapter (`src/sensors/[name]_sensor.h`).
2. Add a compile flag in `platformio.ini` (e.g. `-DUSE_BNO085_VARIANT`).
3. Branch in `sensor_factory.cpp` on the flag.
4. Add unit tests against `IMUInterface` contract.
5. Field-validate with at least one test flight before making default.

No flight-logic code changes. See [[entities/configuration-system]] for the flag list.

## Flight-Critical vs Nice-to-Have

| Sensor | Flight-critical? | If missing |
|--------|------------------|------------|
| Primary IMU (ICM/BNO) | **Yes** | Cannot orient → guidance disabled; apogee still works via baro + GPS + timer |
| Barometer | Effectively yes | Apogee accuracy degrades; GPS + timer still deploy chute |
| GPS | No (for flight) | Recovery becomes harder but flight/apogee fine |
| KX134 backup | No | Lose high-g coverage; most flights never need it |
| Magnetometer | No (optional in Kalman) | Heading drift, but pitch/roll still good |

This ordering is enforced by [[concepts/apogee-detection]] and [[concepts/system-robustness]].

## Hardware Selection Checklist

When adding a new sensor option:

- [ ] I2C or SPI? Will it conflict on existing bus addresses?
- [ ] Operating range covers flight envelope? (Don't pick a ±8 g accel for a solid motor.)
- [ ] Driver available in Arduino/PlatformIO library registry?
- [ ] Can it be mocked behind `IMUInterface`?
- [ ] Power draw fits the battery budget?

## Related

- [[entities/hardware-platform]] — current physical loadout
- [[concepts/sensor-redundancy]] — failover logic
- [[concepts/hal-abstraction]] — why swaps are cheap
- [[queries/roadmap-2026]] — where sensor upgrades sit on the timeline
