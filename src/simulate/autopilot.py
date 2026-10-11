"""A simple autopilot that flies a racetrack course with the nonlinear model.

It stands in for the pilot: bank to turn, lift to hold altitude, thrust to hold speed. Its
gains are choices made here, not properties of the aircraft, so lap times depend on it as
well as on the aircraft. Sign conventions for the controls:
docs/decisions/2026-10-11-control-surfaces.md.
"""

import math
import tomllib
from dataclasses import dataclass, field
from pathlib import Path


@dataclass(frozen=True)
class Course:
    """Racetrack pattern. The start line is at north = 0; the first leg heads north."""

    name: str
    half_length: float          # start line to each end of the straight, m
    turn_direction: int         # +1 right-hand 180 deg turns, -1 left-hand
    full_turn: bool             # 360 deg turn at mid-downwind, opposite to turn_direction
    laps: int
    speed: float                # m/s to hold; math.inf means full throttle throughout
    altitude: float
    load_factor_limit: float    # load factor to aim for in turns
    thrust_max: float
    deflection_max: float


def load_course(path):
    raw = tomllib.loads(Path(path).read_text())
    pattern, flight = raw["pattern"], raw["flight"]
    direction = {"right": 1, "left": -1}[pattern["turn_direction"]]
    return Course(
        name=raw.get("name", Path(path).stem), half_length=float(pattern["half_length"]),
        turn_direction=direction, full_turn=bool(pattern["full_turn"]),
        laps=int(pattern["laps"]),
        speed=math.inf if flight["speed"] == "max" else float(flight["speed"]),
        altitude=float(flight["altitude"]),
        load_factor_limit=float(flight["load_factor_limit"]),
        thrust_max=float(flight["thrust_max"]),
        deflection_max=math.radians(float(flight["deflection_max_deg"])))


@dataclass(frozen=True)
class Gains:
    heading: float = 0.8          # 1/s: turn rate commanded per rad of heading error
    cross_track: float = 0.015    # rad of heading correction per m off the line
    cross_track_limit: float = math.radians(30.0)
    capture: float = math.radians(12.0)   # heading error at which a turn counts as complete
    bank_rate: float = math.radians(120.0)  # how fast the bank command may change, rad/s
    roll: float = 0.15            # aileron per rad of bank error
    altitude: float = 0.5         # 1/s: climb rate commanded per m of altitude error
    climb_rate_limit: float = 3.0   # m/s
    flight_path: float = 1.5      # 1/s: load factor per rad of flight-path error, times V/g
    alpha: float = 1.0            # elevator per rad of angle-of-attack error
    pitch_rate: float = 0.05      # elevator per rad/s of pitch-rate error
    sideslip: float = 1.0         # rudder per rad of sideslip
    speed: float = 1.0            # 1/s: acceleration commanded per m/s of speed error


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class CourseAutopilot:
    """Without `performance` it flies to the course's own numbers. With it (and a thrust
    table from the powertrain) thrust comes from the propulsion model and turns respect
    the structural and lift limits."""

    course: Course
    gains: Gains = field(default_factory=Gains)
    performance: object = None
    thrust_table: object = None
    structure_margin: float = 0.93   # fraction of the structural limit aimed for in turns
    lift_margin: float = 0.85        # fraction of maximum lift aimed for in turns
    # Optional speed management: close the throttle on the approach so the aircraft enters
    # each turn at this speed, hold it through the turn, then go back to full throttle.
    turn_entry_speed: float = None

    def __post_init__(self):
        self.phase = "upwind"
        self.heading = 0.0            # unwrapped heading, rad
        self._last_psi = None
        self._last_t = None
        self._last_north = None
        self.base = 0.0               # unwrapped heading of the current upwind leg
        self.target = 0.0             # unwrapped heading target
        self.downwind_east = None
        self.bank_command = 0.0
        self.lap_times = []           # times of crossing the start line northbound
        self.events = [(0.0, "upwind")]
        self.finished = False
        self.telemetry = {}

    # ---------------------------------------------------------------- guidance
    def _advance(self, t, out):
        c, north, half = self.course, out["north"], self.course.half_length
        captured = abs(self.target - self.heading) < self.gains.capture
        new = None
        if self.phase == "upwind":
            if (self._last_north is not None and self._last_north < 0.0 <= north
                    and t > 0.0):
                self.lap_times.append(t)
                if len(self.lap_times) >= c.laps:
                    self.finished = True
            if north >= half:
                new, self.target = "turn_1", self.base + c.turn_direction * math.pi
        elif self.phase == "turn_1" and captured:
            new, self.downwind_east = "downwind_1", out["east"]
        elif self.phase == "downwind_1" and north <= 0.0:
            if c.full_turn:
                new, self.target = "full_turn", self.target - c.turn_direction * 2 * math.pi
            else:
                new = "downwind_2"
        elif self.phase == "full_turn" and captured:
            new = "downwind_2"
        elif self.phase == "downwind_2" and north <= -half:
            new, self.target = "turn_2", self.target + c.turn_direction * math.pi
        elif self.phase == "turn_2" and captured:
            new, self.base = "upwind", self.target
        if new:
            self.phase = new
            self.events.append((t, new))

    def _heading_command(self, out):
        """Unwrapped heading to steer: the turn target, or the leg heading plus a correction
        that brings the aircraft back onto the leg's line."""
        if self.phase in ("upwind", "downwind_1", "downwind_2"):
            line_east = 0.0 if self.phase == "upwind" else self.downwind_east
            northbound = self.phase == "upwind"
            right_of_line = (out["east"] - line_east) * (1.0 if northbound else -1.0)
            g = self.gains
            correction = max(-g.cross_track_limit,
                             min(g.cross_track_limit, -g.cross_track * right_of_line))
            return self.target + correction
        return self.target

    def _distance_to_turn(self, out):
        """Metres left on the current straight before the next turn starts, or None in a turn."""
        north, half = out["north"], self.course.half_length
        if self.phase == "upwind":
            return half - north
        if self.phase == "downwind_1":
            return north if self.course.full_turn else north + half
        if self.phase == "downwind_2":
            return north + half
        return None

    def _speed_target(self, out, model, drag_level):
        """Speed to hold (math.inf for full throttle, 0.0 for throttle closed)."""
        target = self.course.speed
        if self.turn_entry_speed is None:
            return target
        distance = self._distance_to_turn(out)
        if distance is None:
            return self.turn_entry_speed                      # in a turn: hold the entry speed
        V = out["V"]
        if V > self.turn_entry_speed:
            # distance needed to slow down with the throttle closed, at the present drag
            slowing = (V * V - self.turn_entry_speed ** 2) / (2.0 * drag_level / model.m)
            if distance <= slowing:
                return 0.0
        return target

    # ----------------------------------------------------------------- control
    def __call__(self, t, out, model):
        g, c = self.gains, self.course
        dt = 0.0 if self._last_t is None else t - self._last_t
        if self._last_psi is not None:
            self.heading += wrap(out["psi"] - self._last_psi)
        self._advance(t, out)
        self._last_psi, self._last_t, self._last_north = out["psi"], t, out["north"]

        V, phi = out["V"], out["phi"]
        qS = 0.5 * model.rho * V * V * model.S
        weight = model.m * model.g
        turn_target, ceiling = c.load_factor_limit, c.load_factor_limit * 1.1
        if self.performance is not None:
            lift_limit = self.performance.CL_max * qS / weight
            turn_target = min(turn_target,
                              self.structure_margin * self.performance.load_factor_limit,
                              self.lift_margin * lift_limit)
            ceiling = min(self.performance.load_factor_limit, 0.97 * lift_limit)
        bank_limit = math.acos(1.0 / max(turn_target, 1.0001))
        turn_rate = g.heading * (self._heading_command(out) - self.heading)
        wanted = max(-bank_limit, min(bank_limit, math.atan(V * turn_rate / model.g)))
        step = g.bank_rate * dt
        self.bank_command += max(-step, min(step, wanted - self.bank_command))
        delta_l = -g.roll * (self.bank_command - phi)

        climb = max(-g.climb_rate_limit,
                    min(g.climb_rate_limit, g.altitude * (c.altitude - out["altitude"])))
        gamma = math.asin(max(-1.0, min(1.0, out["climb_rate"] / V)))
        n = (1.0 / max(math.cos(phi), 0.2)
             + g.flight_path * (climb / V - gamma) * V / model.g)
        n = max(0.0, min(ceiling, n))
        CL = n * weight / qS
        alpha_c = (CL - model.CL0) / model.CLa
        q_c = model.g / V * (n - math.cos(phi) * math.cos(gamma))     # steady pull-up/turn
        lon = model.aircraft.longitudinal
        feedforward = -(lon.Cm_alpha * alpha_c + lon.Cm_q * q_c * model.l / V) / lon.Cm_dm
        delta_m = feedforward + g.alpha * (out["alpha"] - alpha_c) + g.pitch_rate * (out["q"] - q_c)

        delta_n = -g.sideslip * out["beta"]

        CL_level = weight / qS
        speed_target = self._speed_target(
            out, model, qS * (model.CDmin + model.k * CL_level * CL_level))
        if math.isinf(speed_target):
            thrust = math.inf                                   # full throttle
        elif speed_target <= 0.0:
            thrust = 0.0                                        # throttle closed
        else:
            drag = qS * (model.CDmin + model.k * CL * CL)
            thrust = drag + model.m * (g.speed * (speed_target - V)
                                       + model.g * math.sin(gamma))
        if self.thrust_table is not None:
            thrust = max(0.0, min(thrust, self.thrust_table.max_thrust(V)))
            # zero thrust means the throttle is closed, which draws no current
            throttle = self.thrust_table.throttle_for(thrust, V) if thrust > 0.0 else 0.0
            self.telemetry = {"throttle": throttle,
                              "current": self.thrust_table.current(throttle, V)}
        return thrust, delta_m, delta_l, delta_n
