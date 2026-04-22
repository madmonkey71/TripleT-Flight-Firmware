# Theoretical Flight Profile vs State Machine Analysis

## Aerotech H125W in a 1.5 kg Rocket — Nominal Flight

*Generated: 2026-02-22*
*Simulation script: `flight_simulation_h125w.py`*

---

## 1. Rocket & Motor Configuration

| Parameter | Value |
|-----------|-------|
| **Motor** | Aerotech H125W (White Lightning) |
| **Total Impulse** | 299.4 N·s |
| **Average Thrust** | 125 N |
| **Peak Thrust** | 276 N (at T+0.05s) |
| **Burn Time** | 2.77 s |
| **Motor Mass** | 323 g (188 g propellant) |
| **Dry Mass (no motor)** | 1.18 kg |
| **Liftoff Mass** | 1.50 kg |
| **Burnout Mass** | 1.315 kg |
| **Airframe Diameter** | 29 mm |
| **Cd (rocket)** | 0.5 |
| **Drogue** | 0.5 m dia, Cd=1.5 |
| **Main** | 1.2 m dia, Cd=2.2 |

---

## 2. Flight Profile Summary

| Event | Time | Altitude | Velocity | Accel | Firmware State |
|-------|------|----------|----------|-------|----------------|
| Ignition | T+0.0s | 0 m | 0 m/s | **17.7g** | ARMED |
| Liftoff detected | T+0.1s | ~1 m | ~17 m/s | 17.7g | **ARMED -> BOOST** |
| Peak velocity | T+2.5s | ~380 m | ~196 m/s | ~0g | BOOST |
| Motor burnout | T+2.8s | 426 m | 195 m/s | -1.6g | **BOOST -> COAST** |
| Coast midpoint | T+10s | 1,466 m | 100 m/s | -1.3g | COAST |
| Apogee | T+19.8s | **1,936 m** | 0 m/s | -1.0g | **COAST -> APOGEE** |
| Drogue fires | T+19.8s | 1,936 m | 0 m/s | — | **APOGEE -> DROGUE_DEPLOY** |
| Drogue opens | T+20.8s | 1,928 m | -8.5 m/s | 7.5g | **DROGUE_DEPLOY -> DROGUE_DESCENT** |
| Main deploys | T+237s | 100 m | -8.5 m/s | — | **DROGUE_DESCENT -> MAIN_DEPLOY** |
| Main opens | T+238s | 92 m | -2.9 m/s | 7.5g | **MAIN_DEPLOY -> MAIN_DESCENT** |
| Touchdown | T+272s | 0 m | -2.9 m/s | 1.0g | **MAIN_DESCENT -> LANDED** |
| Recovery mode | T+282s | 0 m | 0 m/s | 1.0g | **LANDED -> RECOVERY** |

**Total flight time: ~4 min 32 sec**

---

## 3. State Machine Mapping — Detailed Analysis

### 3.1 STARTUP -> CALIBRATION -> PAD_IDLE (Pre-Flight)

**What happens:** Power on, sensors initialize, barometer calibrates.

| Firmware Threshold | Value | This Flight |
|--------------------|-------|-------------|
| `CALIBRATION_AUTO_TIMEOUT_MS` | 120,000 ms | GPS fix within ~60s outdoors |
| GPS fix type required | >= 3 (3D fix) | Should achieve within 30-60s |
| GPS pDOP required | < 3.0 | Typically 1.0-2.0 outdoors |

**Outcome:** Auto-calibration succeeds when GPS acquires fix. Transitions CALIBRATION -> PAD_IDLE. If no GPS after 2 minutes, fallback calibration (offset=0) kicks in.

**Note on calibration:** The GPS calibration offset only affects absolute altitude in logs. All flight-critical decisions use AGL (altitude above ground level), calculated as `current_alt - launch_alt`. Since the offset cancels out in this subtraction, calibration has **no impact on flight safety**. Launch detection, apogee, main deployment at 100m AGL, and landing detection all work identically with or without GPS calibration.

---

### 3.2 PAD_IDLE -> ARMED (User Command)

**What happens:** User sends `arm` command. Health check runs.

| Health Check | Requirement | Expected |
|-------------|-------------|----------|
| MS5611 initialized | Yes | PASS |
| MS5611 calibrated | Yes | PASS (auto-calibrated) |
| IMU (ICM-20948 or KX134) | At least one | PASS |
| GPS | Not required | N/A |

**Outcome:** System arms successfully. Launch altitude recorded.

---

### 3.3 ARMED -> BOOST (Launch Detection)

**Firmware threshold:** `BOOST_ACCEL_THRESHOLD = 2.0g`
**Sensor rate:** 10 Hz (100 ms polling)

| Time | Accel (g) | Detected? |
|------|-----------|-----------|
| T+0.0s | 17.7g | Not yet read (same cycle as ignition) |
| T+0.1s | ~17g | **YES — 17g >> 2.0g threshold** |

**Analysis:** The H125W produces 276N at ignition on a 1.5kg rocket = **18.8g instantaneous** (minus 1g gravity = 17.7g sensed). This massively exceeds the 2.0g threshold. Detection occurs on the **first sensor read after ignition** (within 100ms).

**Potential concern:** The 2.0g threshold is quite low. On the pad with vibration or wind gusts, could a false trigger occur? At 2.0g, you'd need ~30N of external force on a 1.5kg rocket. Unlikely but worth noting. The threshold is appropriate for this motor class.

**ICM-20948 range:** +/-16g. Peak acceleration is 17.7g, which **exceeds the ICM-20948 range**. The firmware should auto-switch to KX134 (+/-64g) at the 16g threshold. If KX134 is not present, the ICM-20948 will clip/saturate at 16g but launch detection still succeeds since 16g >> 2.0g.

---

### 3.4 BOOST -> COAST (Burnout Detection)

**Firmware threshold:** `COAST_ACCEL_THRESHOLD = 0.5g`
**Detection function:** `detectBoostEnd()` — single reading below threshold

| Time | Accel (g) | Below 0.5g? |
|------|-----------|-------------|
| T+2.0s | 2.0g | No |
| T+2.3s | 0.7g | No |
| T+2.5s | -0.3g | **YES — magnitude 0.3g < 0.5g** |
| T+2.8s | -1.6g | Magnitude 1.6g > 0.5g |

**Wait — important subtlety.** The firmware uses `get_accel_magnitude()` which computes the **vector magnitude** sqrt(ax² + ay² + az²). During coast, the rocket is in near-freefall, so the magnitude approaches **0g** (not 1g). This is correct — a rocket in freefall (no thrust, ignoring drag) reads ~0g on all axes.

However, with aerodynamic drag at 195 m/s, there's significant deceleration (~1.6g). The accelerometer would actually read this drag deceleration. Let me reconsider:

- During powered flight: accel magnitude = thrust/mass - g ≈ 17g to 2g (decreasing)
- At burnout: thrust drops to 0, rocket experiences only gravity + drag
- In freefall reference: accel reads **drag deceleration only** ≈ 0.6g at burnout velocity

**The magnitude at burnout depends on sensor reference frame.** If the accelerometer reads body-frame acceleration (typical), then:
- On the pad: magnitude = 1.0g (gravity)
- During boost: magnitude = thrust/mass ≈ 17g to 2g
- At burnout: magnitude drops suddenly to ~0.5-0.8g (drag only in freefall)

The 0.5g threshold should trigger within 1-2 sensor reads after burnout. **Detection latency: ~100-200ms after actual burnout.**

**Discussion point:** The single-reading detection could false-trigger if there's a thrust hiccup during the tail-off phase (T+2.0 to T+2.8s where thrust drops from 47N to 0N). At T+2.5s, thrust is ~15N on a 1.3kg rocket = 1.2g minus gravity = ~0.2g magnitude. This could trigger burnout detection ~0.3s early. Consider requiring 2-3 consecutive readings below threshold for more robust detection.

---

### 3.5 COAST (Apogee Detection)

**Duration:** ~17 seconds (T+2.8s to T+19.8s)
**Firmware uses 4 detection methods (first one to trigger wins):**

#### Method 1: Barometric (Primary)
- **Threshold:** `APOGEE_CONFIRMATION_COUNT = 5` consecutive readings where altitude < max altitude
- **Sensor rate:** 10 Hz (100ms)
- **At apogee:** Velocity crosses zero. Altitude changes by ~0.5m per 100ms near apogee.
- **Detection time:** 5 readings × 100ms = **500ms after true apogee**
- **Altitude loss before detection:** ~1-2m (negligible at 1,936m)

#### Method 2: Accelerometer (Secondary)
- **Threshold:** `APOGEE_ACCEL_CONFIRMATION_COUNT = 5` consecutive negative Z-axis readings
- **At apogee:** Z-axis acceleration is negative (decelerating upward, then accelerating downward)
- **Issue:** Z-axis is negative for the **entire coast phase** (gravity + drag always pulling down). This would trigger almost immediately after entering COAST, which seems like a bug or the check is more nuanced.
- **Actually:** The check is `icm_accel[2] < 0.0f`. In body frame during coast, the Z-axis reads near 0g (freefall minus drag). The sign depends on the sensor orientation and whether drag is positive or negative in the body Z-axis. If the rocket is nose-up, Z-axis drag is positive (opposing upward motion), so `icm_accel[2]` would be **positive** during ascent and **negative** only after apogee when the rocket starts descending and drag reverses. This is correct behavior.
- **Detection time:** ~500ms after apogee

#### Method 3: GPS (Tertiary)
- **Threshold:** `APOGEE_GPS_CONFIRMATION_COUNT = 3` readings showing descent > 5m
- **GPS rate:** 10 Hz
- **GPS altitude accuracy:** ±5-10m at best
- **Detection time:** Likely 1-3 seconds after apogee (GPS latency + accuracy)

#### Method 4: Backup Timer (Failsafe)
- **Threshold:** `BACKUP_APOGEE_TIME_MS = 20,000ms` after motor burnout
- **Expected apogee:** 17s after burnout
- **Timer fires:** 20s after burnout = T+22.8s
- **Altitude at timer fire:** ~1,920m (16m below apogee, descending at ~5 m/s under drogue)

**Analysis:** Barometric detection will trigger first, ~500ms after true apogee at T+20.3s. The backup timer at 20s provides excellent margin — only 3s after expected apogee.

**Discussion point:** The backup timer is well-tuned for this flight. For higher-performance motors with longer coast phases, the 20s timer might fire prematurely. It's currently a fixed constant. Consider making it configurable or computing it from expected burn time.

---

### 3.6 APOGEE -> DROGUE_DEPLOY -> DROGUE_DESCENT

**Transition:** Immediate (same loop cycle)
**Pyro fire duration:** `PYRO_FIRE_DURATION = 1,000ms`

| Time | Event | State |
|------|-------|-------|
| T+20.3s | Apogee detected | APOGEE |
| T+20.3s | Drogue pyro fires (CH1 HIGH) | DROGUE_DEPLOY |
| T+21.3s | Pyro off (CH1 LOW) | DROGUE_DESCENT |

**Drogue terminal velocity:** 8.5 m/s (19 mph)

**Analysis:** The 1-second pyro fire is standard. The drogue deployment shock at ~7.5g (decelerating from freefall to 8.5 m/s) is well within structural limits. Shock cord should be rated for 10-15g.

---

### 3.7 DROGUE_DESCENT -> MAIN_DEPLOY -> MAIN_DESCENT

**Firmware threshold:** `MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M = 100.0m` AGL
**Detection:** Barometric altitude < main deploy altitude (checked every 100ms)

| Time | Altitude | Velocity | Event |
|------|----------|----------|-------|
| T+237s | 100.5m | -8.5 m/s | Above threshold |
| T+237.1s | 99.6m | -8.5 m/s | **Below 100m — MAIN_DEPLOY** |
| T+238.1s | ~91m | -2.9 m/s | Pyro off, **MAIN_DESCENT** |

**Descent under drogue:** 1,836m at 8.5 m/s = **216 seconds** (3 min 36 sec)

**Analysis:** The main deployment altitude of 100m AGL is appropriate. At 8.5 m/s descent rate, the rocket passes through 100m within one sensor cycle. No risk of missing the altitude target.

**Discussion point:** The barometer accuracy matters here. A ±5m error means deployment between 95-105m AGL. At 8.5 m/s, that's ±0.6s timing difference — negligible. But if calibration used the fallback (offset=0), the "100m AGL" could be significantly wrong if the launch site isn't near sea level.

---

### 3.8 MAIN_DESCENT -> LANDED

**Firmware thresholds:**
- `LANDING_ALTITUDE_STABLE_THRESHOLD = 1.0m` (within 1m of launch altitude)
- `LANDING_ACCEL_MIN_G = 0.9g`, `LANDING_ACCEL_MAX_G = 1.1g` (sitting at 1g ± 0.1g)
- `LANDING_CONFIRMATION_TIME_MS = 2,000ms` (stable for 2 seconds)

**Landing velocity:** 2.9 m/s (6.5 mph) — safe

| Time | Altitude AGL | Accel | Condition Met? |
|------|-------------|-------|----------------|
| T+272s | 0.0m | 1.0g | Altitude within 1m: YES |
| T+272s | 0.0m | 1.0g | Accel 0.9-1.1g: YES |
| T+274s | 0.0m | 1.0g | Stable for 2s: **YES -> LANDED** |

**Analysis:** Landing detection works well. The 10-reading altitude moving average in `detectLanding()` prevents false triggers from barometer noise. The 2-second confirmation window is appropriate — the rocket won't bounce at 2.9 m/s.

---

### 3.9 LANDED -> RECOVERY

**Threshold:** `LANDED_TIMEOUT_MS = 10,000ms` (10 seconds)
**Transition:** T+284s — automatic, no user action needed.

**In RECOVERY state:**
- SOS buzzer pattern plays
- LED strobe activates
- GPS beacon broadcasts every 10 seconds

---

## 4. Timeline Summary — State vs Flight Event

```
T-120s  ┌─STARTUP──────────────────────────────────┐
        │ Sensors initializing, GPS acquiring fix   │
T-60s   └──►CALIBRATION─────────────────────────────┤
        │ Auto-calibrate with GPS (or timeout 120s) │
T-30s   └──►PAD_IDLE────────────────────────────────┤
        │ User sends 'arm' command                  │
T-10s   └──►ARMED───────────────────────────────────┤
        │ Waiting for >2.0g acceleration            │
T+0.0s  │ IGNITION ================================ │ 17.7g peak
T+0.1s  └──►BOOST──────────────────────────────────┤
        │ 276N thrust, 195 m/s burnout velocity     │
T+2.8s  └──►COAST──────────────────────────────────┤ Burnout at 426m
        │ 17s coast, decelerating under drag        │
T+19.8s │ TRUE APOGEE at 1,936m ================== │
T+20.3s └──►APOGEE──────────────────────────────────┤ Detected 0.5s late
T+20.3s └──►DROGUE_DEPLOY──────────────────────────┤ CH1 fires
T+21.3s └──►DROGUE_DESCENT─────────────────────────┤ 8.5 m/s descent
        │ 216 seconds descending under drogue       │
T+237s  └──►MAIN_DEPLOY────────────────────────────┤ At 100m AGL, CH2 fires
T+238s  └──►MAIN_DESCENT───────────────────────────┤ 2.9 m/s descent
        │ 34 seconds descending under main          │
T+272s  │ TOUCHDOWN =============================== │
T+274s  └──►LANDED──────────────────────────────────┤ Confirmed after 2s
T+284s  └──►RECOVERY───────────────────────────────┤ SOS buzzer + GPS beacon
```

---

## 5. Potential Issues & Discussion Points

### 5.1 IMU Saturation During Boost
- **Peak acceleration: 17.7g** exceeds ICM-20948 range (+/-16g)
- Firmware switches to KX134 (+/-64g) at the 16g threshold
- **If KX134 not installed:** ICM-20948 clips at 16g. Launch detection still works (16g >> 2.0g threshold). But logged acceleration data will be inaccurate during the first ~0.5s of boost.
- **Recommendation:** KX134 is important for this motor class

### 5.2 Burnout Detection Robustness — FIXED
- **Previously:** Single reading below 0.5g triggered COAST (could false-trigger during tail-off)
- **Now:** Requires `COAST_CONFIRMATION_COUNT = 3` consecutive readings below 0.5g
- At 10 Hz sensor rate, this adds 200-300ms detection latency (acceptable)
- Prevents false burnout detection during thrust hiccups or tail-off

### 5.3 Backup Apogee Timer Margin
- Timer: 20s after burnout. Expected apogee: 17s after burnout.
- Margin: only 3s. If burnout is detected early (see 5.2), the timer reference shifts.
- If burnout detected at T+2.5s instead of T+2.8s, timer fires at T+22.5s (2.7s after apogee)
- **Status:** Acceptable margin for this flight. Would need tuning for different motors.

### 5.4 Barometer Calibration — Not Safety-Critical
- All flight decisions use AGL: `current_altitude - launch_altitude`
- The calibration offset cancels out in this subtraction
- Main deployment at 100m AGL works correctly regardless of calibration
- GPS calibration only improves absolute altitude accuracy in logs
- **Status:** Nice to have for post-flight analysis, not required for safe flight

### 5.5 Health Check During Flight
- Health check runs every 1 second during BOOST/COAST/descent
- If barometer has a momentary I2C bus error, `ms5611_initialized_ok` remains true (set at init time)
- No risk of false ERROR transition from transient sensor read failures
- **Status:** Safe for this flight profile

### 5.6 Descent Time vs Battery
- Total flight: 4.5 minutes (drogue descent is 3.5 min)
- RECOVERY state begins at T+284s with SOS buzzer and GPS beacon
- Typical LiPo with Teensy 4.1 + sensors: 2-4 hours battery life
- **Status:** No battery concern for this flight

### 5.7 Sensor Sample Rates
- All sensors: 10 Hz (100ms intervals)
- During BOOST (2.8s), the firmware captures ~28 data points
- During COAST (17s), ~170 data points
- This is sufficient for state transition detection
- **Note:** Data logging at 5 Hz (200ms) means ~14 logged points during boost. For post-flight analysis of the thrust curve, 10-20 Hz logging would be better.

---

## 6. Firmware Threshold Validation

| Threshold | Current Value | Appropriate for H125W? | Notes |
|-----------|--------------|----------------------|-------|
| `BOOST_ACCEL_THRESHOLD` | 2.0g | **YES** | 17.7g >> 2.0g, no false trigger risk |
| `COAST_ACCEL_THRESHOLD` | 0.5g (x3) | **YES** | Now requires 3 consecutive readings |
| `APOGEE_CONFIRMATION_COUNT` | 5 | **YES** | 500ms delay, ~2m altitude loss |
| `BACKUP_APOGEE_TIME_MS` | 20,000ms | **YES** | 3s margin after expected apogee |
| `MAIN_DEPLOY_HEIGHT_ABOVE_GROUND_M` | 100m | **YES** | Standard for dual-deploy |
| `LANDING_CONFIRMATION_TIME_MS` | 2,000ms | **YES** | Safe at 2.9 m/s landing |
| `PYRO_FIRE_DURATION` | 1,000ms | **YES** | Standard e-match timing |
| `LANDED_TIMEOUT_MS` | 10,000ms | **YES** | Adequate before recovery mode |

---

## 7. Thrust Curve Data (Aerotech H125W)

```
Time(s)  Thrust(N)
0.053    276.0
0.161    241.5
0.270    216.3
0.378    199.5
0.488    188.8
0.597    182.3
0.705    175.9
0.814    169.4
0.922    162.3
1.031    154.3
1.141    143.9
1.249    133.5
1.357    123.0
1.466    113.1
1.575    101.8
1.684     88.4
1.793     73.5
1.901     60.4
2.009     46.6
2.119     36.8
2.228     29.5
2.336     23.6
2.445     18.8
2.553     14.7
2.663     11.0
2.772      0.0
```

Source: ThrustCurve.org (TMT test stand data)

---

## 8. Simulation Limitations

- **No wind modeling** — drift not calculated
- **Constant air density** — actual density decreases ~12% at 2,000m
- **No parachute inflation delay** — real deployment takes 0.5-1.0s
- **No thrust misalignment** — assumes vertical flight
- **Euler integration** (dt=0.1s) — adequate accuracy for this analysis
- **Linear propellant consumption** — real burn rate varies with thrust

For production flight planning, use OpenRocket or RASAero with this data as a cross-reference.
