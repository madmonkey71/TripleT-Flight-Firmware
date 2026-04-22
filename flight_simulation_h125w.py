#!/usr/bin/env python3
"""
Flight Simulation for Aerotech H125W Motor
Calculates complete flight profile from launch to landing
"""

import math

# Motor data - Aerotech H125W
THRUST_CURVE = [
    (0.053, 276), (0.161, 241), (0.270, 216), (0.378, 199), (0.488, 189),
    (0.597, 182), (0.705, 176), (0.814, 169), (0.922, 162), (1.031, 154),
    (1.141, 144), (1.249, 133), (1.357, 123), (1.466, 113), (1.575, 102),
    (1.684, 88), (1.793, 74), (1.901, 60), (2.009, 47), (2.119, 37),
    (2.228, 30), (2.336, 24), (2.445, 19), (2.553, 15), (2.663, 11), (2.772, 0)
]
BURN_TIME = 2.77  # seconds
PROPELLANT_MASS = 0.188  # kg
MOTOR_CASE_MASS = 0.323 - 0.188  # kg

# Rocket parameters
DRY_MASS = 1.18  # kg (without motor)
CD_ROCKET = 0.5
AREA_ROCKET = 0.00066  # m² (29mm diameter)

# Parachute parameters
CD_DROGUE = 1.5
DIAMETER_DROGUE = 0.5  # m
AREA_DROGUE = math.pi * (DIAMETER_DROGUE / 2) ** 2

CD_MAIN = 2.2
DIAMETER_MAIN = 1.2  # m
AREA_MAIN = math.pi * (DIAMETER_MAIN / 2) ** 2

# Environment
RHO = 1.225  # kg/m³ (sea level air density)
G = 9.81  # m/s²

# Deployment altitudes
MAIN_DEPLOY_ALT = 100  # m AGL

# Simulation parameters
DT = 0.1  # seconds


def interpolate_thrust(t):
    """Linear interpolation of thrust curve"""
    if t <= 0:
        return THRUST_CURVE[0][1]
    if t >= BURN_TIME:
        return 0

    # Find bracketing points
    for i in range(len(THRUST_CURVE) - 1):
        t1, thrust1 = THRUST_CURVE[i]
        t2, thrust2 = THRUST_CURVE[i + 1]
        if t1 <= t <= t2:
            # Linear interpolation
            frac = (t - t1) / (t2 - t1)
            return thrust1 + frac * (thrust2 - thrust1)

    return 0


def get_current_mass(t):
    """Calculate current rocket mass (propellant burns linearly)"""
    base_mass = DRY_MASS + MOTOR_CASE_MASS
    if t >= BURN_TIME:
        return base_mass  # All propellant consumed

    # Linear propellant consumption
    propellant_remaining = PROPELLANT_MASS * (1 - t / BURN_TIME)
    return base_mass + propellant_remaining


def simulate_flight():
    """Run complete flight simulation"""
    # State variables
    t = 0.0
    altitude = 0.0
    velocity = 0.0
    phase = "BOOST"

    # Tracking key events
    peak_accel = 0.0
    peak_accel_time = 0.0
    apogee = 0.0
    apogee_time = 0.0
    burnout_velocity = 0.0
    burnout_time = 0.0

    # Output table
    results = []
    results.append(("Time(s)", "Alt(m)", "Vel(m/s)", "Accel(g)", "Phase", "Notes"))
    results.append(("-" * 80, "", "", "", "", ""))

    timestep = 0

    while True:
        # Current state
        mass = get_current_mass(t)
        thrust = interpolate_thrust(t)

        # Determine drag coefficient and area based on phase
        if phase in ["BOOST", "COAST"]:
            cd = CD_ROCKET
            area = AREA_ROCKET
        elif phase == "DROGUE":
            cd = CD_DROGUE
            area = AREA_DROGUE
        else:  # MAIN
            cd = CD_MAIN
            area = AREA_MAIN

        # Forces
        weight = mass * G
        drag = 0.5 * cd * RHO * area * velocity ** 2

        # Drag opposes motion
        if velocity > 0:  # Ascending
            net_force = thrust - weight - drag
        else:  # Descending
            net_force = thrust - weight + drag

        acceleration = net_force / mass
        accel_g = acceleration / G

        # Track peak acceleration
        if abs(accel_g) > abs(peak_accel):
            peak_accel = accel_g
            peak_accel_time = t

        # Euler integration
        velocity += acceleration * DT
        altitude += velocity * DT

        # Determine phase transitions and events
        notes = ""

        if timestep == 0:
            notes = "LIFTOFF"

        if phase == "BOOST" and thrust == 0:
            phase = "COAST"
            burnout_time = t
            burnout_velocity = velocity
            notes = f"BURNOUT (v={velocity:.1f} m/s)"

        if phase == "COAST" and velocity <= 0:
            phase = "DROGUE"
            apogee = altitude
            apogee_time = t
            notes = f"APOGEE (alt={altitude:.1f} m)"

        if phase == "DROGUE" and altitude <= MAIN_DEPLOY_ALT:
            phase = "MAIN"
            notes = f"MAIN DEPLOY (alt={altitude:.1f} m, v={abs(velocity):.1f} m/s)"

        if altitude <= 0 and t > 1:
            notes = f"LANDING (v={abs(velocity):.1f} m/s)"
            results.append((f"{t:.1f}", f"{max(0, altitude):.1f}", f"{velocity:.2f}",
                          f"{accel_g:.2f}", phase, notes))
            break

        # Output every 0.5 seconds or on key events
        if timestep % 5 == 0 or notes:
            results.append((f"{t:.1f}", f"{altitude:.1f}", f"{velocity:.2f}",
                          f"{accel_g:.2f}", phase, notes))

        # Advance time
        t += DT
        timestep += 1

        # Safety check
        if t > 300:  # 5 minutes max
            print("Simulation timeout")
            break

    # Print results
    print("\n" + "=" * 100)
    print("AEROTECH H125W FLIGHT SIMULATION - COMPLETE PROFILE")
    print("=" * 100)
    print()

    # Format table
    for row in results:
        if len(row) == 6:
            print(f"{row[0]:<10} {row[1]:<10} {row[2]:<12} {row[3]:<12} {row[4]:<10} {row[5]}")
        else:
            print(row[0])

    # Summary statistics
    print("\n" + "=" * 100)
    print("KEY FLIGHT EVENTS SUMMARY")
    print("=" * 100)
    print(f"Peak Acceleration:     {peak_accel:.2f} g at T+{peak_accel_time:.1f}s")
    print(f"Motor Burnout:         T+{burnout_time:.1f}s at {burnout_velocity:.1f} m/s")
    print(f"Apogee:                {apogee:.1f} m AGL at T+{apogee_time:.1f}s")
    print(f"Drogue Descent Rate:   ~{get_terminal_velocity(DRY_MASS + MOTOR_CASE_MASS, CD_DROGUE, AREA_DROGUE):.1f} m/s")
    print(f"Main Descent Rate:     ~{get_terminal_velocity(DRY_MASS + MOTOR_CASE_MASS, CD_MAIN, AREA_MAIN):.1f} m/s")
    print(f"Total Flight Time:     {t:.1f} seconds")
    print()

    # Verify total impulse
    total_impulse = sum([interpolate_thrust(t * DT) * DT for t in range(int(BURN_TIME / DT) + 1)])
    print(f"Calculated Total Impulse: {total_impulse:.1f} N·s (Expected: 299.4 N·s)")
    print()


def get_terminal_velocity(mass, cd, area):
    """Calculate terminal velocity for given drag parameters"""
    # Terminal velocity when drag = weight
    # 0.5 * cd * rho * A * v^2 = m * g
    # v = sqrt(2 * m * g / (cd * rho * A))
    return math.sqrt(2 * mass * G / (cd * RHO * area))


if __name__ == "__main__":
    simulate_flight()
