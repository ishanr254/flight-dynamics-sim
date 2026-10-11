"""Electric powertrain: battery, motor and propeller solved together.

Ported from sizing-scoring-26-27/src/prop (prop_classes.py, prop_helper_functions.py).
Given throttle and airspeed, the operating point is the RPM at which the voltage the motor
needs equals throttle times the battery voltage under load.
"""

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import brentq

from .propeller import Propeller

CELL_VOLTAGE = 3.7                 # nominal volts per LiPo cell
CELL_RESISTANCE_AH = 0.013         # ohm * Ah per cell: resistance = this / capacity


@dataclass(frozen=True)
class Motor:
    kv: float              # rpm per volt
    rated_power: float     # W

    @property
    def kt(self):
        """Torque constant, N m per A."""
        return 60.0 / (2.0 * math.pi * self.kv)

    def _regression(self, c):
        kv, p = self.kv, self.rated_power
        return c[0] + c[1] * kv + c[2] * p + c[3] * kv * kv + c[4] * kv * p + c[5] * p * p

    @property
    def resistance(self):
        """Winding resistance, ohm (regression on Kv and rated power from the sizing code)."""
        return self._regression((0.3517732388, -0.0005385476, -0.0001855504,
                                 0.0000002999, 0.0000000776, 0.0000000380))

    @property
    def no_load_current(self):
        """No-load current, A (same regression source)."""
        return self._regression((-0.5621009279, 0.0005335965, 0.0016292435,
                                 0.0000005495, 0.0000006015, -0.0000004552))


@dataclass(frozen=True)
class Battery:
    cells: int
    capacity_ah: float
    c_rating: float
    usable_fraction: float = 0.85

    @property
    def voltage(self):
        return self.cells * CELL_VOLTAGE

    @property
    def resistance(self):
        return CELL_RESISTANCE_AH / self.capacity_ah * self.cells

    @property
    def usable_ah(self):
        return self.capacity_ah * self.usable_fraction

    @property
    def max_current(self):
        return self.c_rating * self.capacity_ah


@dataclass(frozen=True)
class OperatingPoint:
    throttle: float
    speed: float
    rpm: float
    thrust: float          # N
    current: float         # A
    voltage: float         # V at the battery terminals under load
    electrical_power: float  # W
    current_limited: bool


@dataclass(frozen=True)
class Powertrain:
    motor: Motor
    battery: Battery
    propeller: Propeller
    rho: float
    current_limit: float   # A: the smaller of the fuse and what the battery can deliver
    thrust_factor: float = 1.0   # installed thrust / model thrust, from a static test

    def _current(self, rpm, speed):
        return self.propeller.torque(rpm, speed, self.rho) / self.motor.kt + self.motor.no_load_current

    def operating_point(self, throttle, speed):
        throttle = min(max(throttle, 0.0), 1.0)
        m, b = self.motor, self.battery
        if throttle <= 0.0:
            return OperatingPoint(0.0, speed, 0.0, 0.0, 0.0, b.voltage, 0.0, False)

        def voltage_shortfall(rpm):
            current = self._current(rpm, speed)
            return rpm / m.kv + current * m.resistance - throttle * (b.voltage - current * b.resistance)

        top = m.kv * b.voltage
        rpm = brentq(voltage_shortfall, 1.0, top) if voltage_shortfall(1.0) < 0 else 1.0
        limited = self._current(rpm, speed) > self.current_limit
        if limited:
            rpm = brentq(lambda r: self._current(r, speed) - self.current_limit, 1.0, rpm)
        current = max(self._current(rpm, speed), 0.0)
        voltage = b.voltage - current * b.resistance
        thrust = self.thrust_factor * self.propeller.thrust(rpm, speed, self.rho)
        return OperatingPoint(throttle, speed, rpm, thrust, current, voltage,
                              current * voltage, limited)

    def tabulate(self, speed_max=60.0, n_speed=31, n_throttle=21):
        return PowertrainTable(self, np.linspace(0.0, speed_max, n_speed),
                               np.linspace(0.0, 1.0, n_throttle))


class PowertrainTable:
    """Thrust and current on a throttle-speed grid, for fast lookup inside a simulation."""

    def __init__(self, powertrain, speeds, throttles):
        self.powertrain, self.speeds, self.throttles = powertrain, speeds, throttles
        points = [[powertrain.operating_point(th, v) for th in throttles] for v in speeds]
        self._thrust = np.array([[p.thrust for p in row] for row in points])
        self._current = np.array([[p.current for p in row] for row in points])

    def _rows(self, table, speed):
        v = min(max(speed, self.speeds[0]), self.speeds[-1])
        i = int(np.clip(np.searchsorted(self.speeds, v) - 1, 0, len(self.speeds) - 2))
        w = (v - self.speeds[i]) / (self.speeds[i + 1] - self.speeds[i])
        return table[i] + w * (table[i + 1] - table[i])

    def thrust(self, throttle, speed):
        return float(np.interp(throttle, self.throttles, self._rows(self._thrust, speed)))

    def current(self, throttle, speed):
        return float(np.interp(throttle, self.throttles, self._rows(self._current, speed)))

    def max_thrust(self, speed):
        return float(self._rows(self._thrust, speed)[-1])

    def throttle_for(self, thrust, speed):
        """Throttle that gives a thrust at a speed (0 to 1; thrust rises with throttle)."""
        row = np.maximum.accumulate(self._rows(self._thrust, speed))
        return float(np.interp(thrust, row, self.throttles))
