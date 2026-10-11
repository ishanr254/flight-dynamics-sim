"""Flying manoeuvres with the nonlinear model: scripted controls, or a simple pilot.

What is flagged and why: docs/decisions/2026-10-11-manoeuvre-mode.md. Control signs:
docs/decisions/2026-10-11-control-surfaces.md.
"""

import math
from dataclasses import dataclass

DEFLECTIONS = ("delta_m", "delta_l", "delta_n")


@dataclass(frozen=True)
class Input:
    """One control held at a value between two times. Deflections in radians, added to
    trim; "throttle" is a setting from 0 to 1 and replaces the trim thrust."""

    control: str
    start: float
    end: float
    value: float


def doublet(control, start, half, value):
    """A deflection one way for `half` seconds, then the other way for the same time."""
    return [Input(control, start, start + half, value),
            Input(control, start + half, start + 2.0 * half, -value)]


class ScriptedControls:
    """Open-loop control inputs. Thrust stays at its trim value unless a throttle input is
    active, in which case it comes from the thrust table."""

    def __init__(self, inputs, thrust_table=None):
        self.inputs = list(inputs)
        for item in self.inputs:
            if item.control not in DEFLECTIONS + ("throttle",):
                raise ValueError(f"unknown control '{item.control}'")
        if thrust_table is None and any(i.control == "throttle" for i in self.inputs):
            raise ValueError("a throttle input needs a thrust table")
        self.thrust_table = thrust_table
        self.telemetry = {}

    def __call__(self, t, out, model):
        deflection = dict.fromkeys(DEFLECTIONS, 0.0)
        throttle = None
        for item in self.inputs:
            if item.start <= t < item.end:
                if item.control == "throttle":
                    throttle = item.value
                else:
                    deflection[item.control] += item.value
        thrust = model.thrust_trim
        if throttle is not None:
            thrust = self.thrust_table.thrust(throttle, out["V"]) if throttle > 0.0 else 0.0
        if self.thrust_table is not None:
            setting = (throttle if throttle is not None
                       else self.thrust_table.throttle_for(thrust, out["V"]))
            self.telemetry = {"throttle": setting,
                              "current": self.thrust_table.current(setting, out["V"])}
        return thrust, deflection["delta_m"], deflection["delta_l"], deflection["delta_n"]


@dataclass(frozen=True)
class Target:
    """What the manoeuvre pilot aims for from `start` until the next target.

    bank: bank angle to hold, rad; None leaves the wings alone and only damps roll rate
          (needed for looping flight, where bank angle is not continuous).
    load_factor: load factor to hold; None holds height (the load factor a level turn at
          the present bank needs, corrected for height error).
    throttle: 0 to 1.
    """

    start: float
    bank: float = 0.0
    load_factor: float = None
    throttle: float = 1.0


class ManoeuvrePilot:
    """Follows a schedule of Targets. Its gains are choices, not properties of the aircraft."""

    def __init__(self, targets, thrust_table, bank_rate=math.radians(240.0), roll_gain=0.15,
                 roll_damping=0.02, alpha_gain=1.0, pitch_rate_gain=0.05, sideslip_gain=1.0,
                 altitude_gain=0.5, climb_rate_limit=5.0, flight_path_gain=1.5,
                 load_factor_limit=math.inf):
        """load_factor_limit caps the load factor the pilot asks for when holding height."""
        self.load_factor_limit = load_factor_limit
        self.targets = sorted(targets, key=lambda target: target.start)
        self.thrust_table = thrust_table
        self.bank_rate, self.roll_gain, self.roll_damping = bank_rate, roll_gain, roll_damping
        self.alpha_gain, self.pitch_rate_gain = alpha_gain, pitch_rate_gain
        self.sideslip_gain, self.altitude_gain = sideslip_gain, altitude_gain
        self.climb_rate_limit, self.flight_path_gain = climb_rate_limit, flight_path_gain
        self.bank_command = None
        self.height = None
        self._last_t = None
        self.telemetry = {}

    def _target(self, t):
        current = self.targets[0]
        for target in self.targets:
            if target.start <= t:
                current = target
        return current

    def __call__(self, t, out, model):
        target = self._target(t)
        dt = 0.0 if self._last_t is None else t - self._last_t
        self._last_t = t
        if self.height is None:
            self.height, self.bank_command = out["altitude"], out["phi"]
        V, phi = out["V"], out["phi"]
        qS = 0.5 * model.rho * V * V * model.S
        weight = model.m * model.g
        gamma = math.asin(max(-1.0, min(1.0, out["climb_rate"] / V)))

        if target.bank is None:
            delta_l = self.roll_damping * out["p"]
            self.bank_command = phi
        else:
            step = self.bank_rate * dt
            self.bank_command += max(-step, min(step, target.bank - self.bank_command))
            delta_l = -self.roll_gain * (self.bank_command - phi)

        if target.load_factor is None:
            climb = max(-self.climb_rate_limit,
                        min(self.climb_rate_limit,
                            self.altitude_gain * (self.height - out["altitude"])))
            n = (1.0 / max(math.cos(phi), 0.2)
                 + self.flight_path_gain * (climb / V - gamma) * V / model.g)
            n = max(0.0, min(self.load_factor_limit, n))
        else:
            n = target.load_factor
        alpha_c = (n * weight / qS - model.CL0) / model.CLa
        # the pull-up rate that goes with n, for any attitude: gravity along the lift axis
        q_c = model.g / V * (n - math.cos(phi) * math.cos(out["theta"]))
        lon = model.aircraft.longitudinal
        delta_m = (-(lon.Cm_alpha * alpha_c + lon.Cm_q * q_c * model.l / V) / lon.Cm_dm
                   + self.alpha_gain * (out["alpha"] - alpha_c)
                   + self.pitch_rate_gain * (out["q"] - q_c))

        delta_n = -self.sideslip_gain * out["beta"]
        throttle = min(max(target.throttle, 0.0), 1.0)
        thrust = self.thrust_table.thrust(throttle, V) if throttle > 0.0 else 0.0
        self.telemetry = {"throttle": throttle,
                          "current": self.thrust_table.current(throttle, V)}
        return thrust, delta_m, delta_l, delta_n
