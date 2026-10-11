"""A whole flight: takeoff roll, laps under the course autopilot, landing and roll-out.

The ground rolls are one-dimensional (the aircraft is held on the runway line, wings
level, at its ground attitude); the airborne part is the nonlinear model. Assumptions:
docs/decisions/2026-10-11-takeoff-landing-wind.md.
"""

import math
from dataclasses import dataclass

import numpy as np

from .nonlinear import Trajectory, quaternion_from_euler, simulate_nonlinear


@dataclass(frozen=True)
class MissionResult:
    trajectory: Trajectory
    liftoff_time: float
    liftoff_distance: float
    liftoff_speed: float           # airspeed, m/s
    touchdown_time: float          # None if the flight did not include a landing
    touchdown_north: float
    stop_time: float
    stop_north: float


def _ground_row(model, table, t, north, airspeed, throttle):
    """Log entry for a point of a ground roll along the runway (heading north)."""
    thrust = table.thrust(throttle, airspeed) if throttle > 0.0 else 0.0
    state = np.array([north, 0.0, 0.0, max(airspeed, 1e-6), 0.0, 0.0,
                      *quaternion_from_euler(0.0, 0.0, 0.0), 0.0, 0.0, 0.0])
    out = model.outputs(state, (thrust, 0.0, 0.0, 0.0))
    out.update({"throttle": throttle, "current": table.current(throttle, airspeed)})
    return state, out


def _ground_acceleration(model, airspeed, thrust, friction):
    """Along-runway acceleration at the ground attitude (angle of attack zero)."""
    qS = 0.5 * model.rho * airspeed * airspeed * model.S
    lift = qS * model.CL0
    drag = qS * (model.CDmin + model.k * model.CL0 ** 2)
    normal = max(model.m * model.g - lift, 0.0)
    return (thrust - drag - friction * normal) / model.m


def takeoff_roll(model, table, liftoff_speed, friction=0.04, dt=0.01):
    """Full-throttle roll from rest at the start line, into the wind, to the liftoff airspeed."""
    headwind = -model.wind[0]
    t, north, ground_speed = 0.0, 0.0, 0.0
    times, states, rows = [], [], []
    while True:
        airspeed = ground_speed + headwind
        state, out = _ground_row(model, table, t, north, airspeed, 1.0)
        times.append(t), states.append(state), rows.append(out)
        if airspeed >= liftoff_speed:
            break
        acceleration = _ground_acceleration(model, max(airspeed, 0.0),
                                            table.max_thrust(max(airspeed, 0.0)), friction)
        if acceleration <= 0.0 or t > 120.0:
            raise RuntimeError("the aircraft cannot accelerate to its liftoff speed")
        ground_speed += acceleration * dt
        north += ground_speed * dt
        t += dt
    return times, states, rows


def landing_roll(model, table, t0, north, ground_speed, friction=0.04, dt=0.01):
    """Throttle-closed roll-out from touchdown to a stop."""
    headwind = -model.wind[0]
    t = t0
    times, states, rows = [], [], []
    while ground_speed > 0.2 and t - t0 < 120.0:
        airspeed = max(ground_speed + headwind, 0.0)
        ground_speed += _ground_acceleration(model, airspeed, 0.0, friction) * dt
        north += max(ground_speed, 0.0) * dt
        t += dt
        state, out = _ground_row(model, table, t, north, airspeed, 0.0)
        times.append(t), states.append(state), rows.append(out)
    return times, states, rows


def fly_mission(model, autopilot, table, dt=0.01, friction=0.04, liftoff_factor=1.2,
                t_limit=1500.0):
    """Takeoff roll, then the autopilot's flight, then (if it lands) the roll-out.

    Liftoff at liftoff_factor x stall speed, the convention of the design report (eq. 25).
    Time zero is throttle-up, so lap times count from the start of the flight window.
    """
    performance = model.performance
    stall = math.sqrt(2.0 * model.m * model.g / (model.rho * model.S * performance.CL_max))
    liftoff_speed = liftoff_factor * stall
    times, states, rows = takeoff_roll(model, table, liftoff_speed, friction, dt)
    liftoff_time, liftoff_distance = times[-1], states[-1][0]

    # airborne from the end of the roll: level, at the angle of attack that carries the weight
    qS = 0.5 * model.rho * liftoff_speed ** 2 * model.S
    alpha = (model.m * model.g / qS - model.CL0) / model.CLa
    x0 = np.array([liftoff_distance, 0.0, 0.0, liftoff_speed * math.cos(alpha), 0.0,
                   liftoff_speed * math.sin(alpha), *quaternion_from_euler(0.0, alpha, 0.0),
                   0.0, 0.0, 0.0])
    air = simulate_nonlinear(model, t_limit, dt, controller=autopilot, x0=x0,
                             stop=lambda: autopilot.finished, t0=liftoff_time)
    keys = [k for k in air.data if k in rows[0]]
    t = np.concatenate([times[:-1], air.t])
    x = np.concatenate([np.array(states[:-1]).reshape(-1, 13), air.x])
    data = {k: np.concatenate([[row[k] for row in rows[:-1]], air.data[k]]) for k in keys}

    touchdown_time = touchdown_north = stop_time = stop_north = None
    if autopilot.touchdown is not None:
        touchdown_time, touchdown_north = autopilot.touchdown
        r_times, r_states, r_rows = landing_roll(model, table, air.t[-1], air["north"][-1],
                                                 air["ground_speed"][-1], friction, dt)
        if r_times:
            t = np.concatenate([t, r_times])
            x = np.concatenate([x, np.array(r_states)])
            data = {k: np.concatenate([data[k], [row[k] for row in r_rows]]) for k in keys}
            stop_time, stop_north = r_times[-1], r_states[-1][0]
    return MissionResult(Trajectory(t, x, data), liftoff_time, liftoff_distance, liftoff_speed,
                         touchdown_time, touchdown_north, stop_time, stop_north)
