"""Steady-flight performance limits from the drag polar, lift limit, structure and thrust.

Point-performance relations (level flight and level turns), used to say where the limits
are and which one binds; the nonlinear simulation then flies at them.
"""

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import brentq


@dataclass(frozen=True)
class Envelope:
    """mass, S, rho, g describe the aircraft and air; performance and thrust table the rest."""

    m: float
    S: float
    rho: float
    g: float
    performance: object
    thrust_table: object

    def _qS(self, V):
        return 0.5 * self.rho * V * V * self.S

    def drag(self, V, load_factor=1.0):
        qS = self._qS(V)
        CL = load_factor * self.m * self.g / qS
        return qS * (self.performance.CD0 + self.performance.k * CL * CL)

    def max_thrust(self, V):
        return self.thrust_table.max_thrust(V)

    @property
    def stall_speed(self):
        return math.sqrt(2.0 * self.m * self.g / (self.rho * self.S * self.performance.CL_max))

    @property
    def corner_speed(self):
        """Lowest speed at which the structural load factor can be reached without stalling."""
        return self.stall_speed * math.sqrt(self.performance.load_factor_limit)

    @property
    def top_speed(self):
        """Highest speed at which full-throttle thrust equals level-flight drag."""
        excess = lambda V: self.max_thrust(V) - self.drag(V)   # noqa: E731
        speeds = np.linspace(self.stall_speed, self.thrust_table.speeds[-1], 400)
        values = np.array([excess(V) for V in speeds])
        positive = np.nonzero(values > 0)[0]
        if not len(positive):
            return math.nan
        last = positive[-1]
        if last == len(speeds) - 1:
            return float(speeds[-1])
        return float(brentq(excess, speeds[last], speeds[last + 1]))

    def lift_limited_load_factor(self, V):
        return self.performance.CL_max * self._qS(V) / (self.m * self.g)

    def instantaneous_load_factor(self, V):
        """What can be pulled right now: the smaller of the structural and lift limits."""
        return min(self.performance.load_factor_limit, self.lift_limited_load_factor(V))

    def sustained_load_factor(self, V):
        """What can be held without losing speed, also capped by structure and lift."""
        qS = self._qS(V)
        spare = self.max_thrust(V) / qS - self.performance.CD0
        if spare <= 0:
            return 0.0
        n = math.sqrt(spare / self.performance.k) * qS / (self.m * self.g)
        return min(n, self.instantaneous_load_factor(V))

    @staticmethod
    def turn_rate(V, load_factor, g=9.81):
        return g * math.sqrt(max(load_factor ** 2 - 1.0, 0.0)) / V

    @staticmethod
    def turn_radius(V, load_factor, g=9.81):
        root = math.sqrt(max(load_factor ** 2 - 1.0, 0.0))
        return V * V / (g * root) if root > 0 else math.inf

    def climb_rate(self, V):
        return (self.max_thrust(V) - self.drag(V)) * V / (self.m * self.g)

    def level_current(self, V):
        """Battery current to hold level flight at V, or None if beyond full throttle."""
        need = self.drag(V)
        if need > self.max_thrust(V):
            return None
        return self.thrust_table.current(self.thrust_table.throttle_for(need, V), V)

    def endurance(self, V):
        """Seconds of level flight at V on the usable battery capacity."""
        current = self.level_current(V)
        usable = self.thrust_table.powertrain.battery.usable_ah
        return None if not current else usable / current * 3600.0
