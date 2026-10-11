"""Propeller thrust and torque from tabulated coefficients (APC performance data)."""

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

INCH = 0.0254


@dataclass(frozen=True)
class Propeller:
    """Ct and Cp against advance ratio J = V / (n D), one table per tabulated RPM."""

    name: str
    diameter: float            # m
    rpms: np.ndarray
    tables: tuple              # per RPM: (J, Ct, Cp) arrays

    def coefficients(self, rpm, speed):
        """(Ct, Cp) at an RPM and airspeed, interpolated between the two nearest RPM tables."""
        J = speed / (max(rpm, 1.0) / 60.0 * self.diameter)
        hi = int(np.clip(np.searchsorted(self.rpms, rpm), 1, len(self.rpms) - 1))
        lo = hi - 1
        w = float(np.clip((rpm - self.rpms[lo]) / (self.rpms[hi] - self.rpms[lo]), 0.0, 1.0))
        ct_lo, cp_lo = self._at(lo, J)
        ct_hi, cp_hi = self._at(hi, J)
        return ct_lo + w * (ct_hi - ct_lo), cp_lo + w * (cp_hi - cp_lo)

    def _at(self, index, J):
        Js, Ct, Cp = self.tables[index]
        if J <= Js[-1]:
            return float(np.interp(J, Js, Ct)), float(np.interp(J, Js, Cp))
        # past the end of the table thrust keeps falling (windmilling); power goes to zero
        slope = (Ct[-1] - Ct[-2]) / (Js[-1] - Js[-2])
        return float(Ct[-1] + slope * (J - Js[-1])), float(max(Cp[-1] * (1 - (J - Js[-1])), 0.0))

    def thrust(self, rpm, speed, rho):
        n = rpm / 60.0
        return self.coefficients(rpm, speed)[0] * rho * n * n * self.diameter ** 4

    def torque(self, rpm, speed, rho):
        n = rpm / 60.0
        return (self.coefficients(rpm, speed)[1] * rho * n * n * self.diameter ** 5
                / (2.0 * math.pi))


def load_propeller(path):
    raw = json.loads(Path(path).read_text())
    rpms = sorted(raw["rpm"], key=float)
    tables = tuple((np.array(raw["rpm"][k]["J"]), np.array(raw["rpm"][k]["Ct"]),
                    np.array(raw["rpm"][k]["Cp"])) for k in rpms)
    return Propeller(raw["name"], raw["diameter_in"] * INCH,
                     np.array([float(k) for k in rpms]), tables)
