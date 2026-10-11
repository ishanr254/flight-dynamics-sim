"""Natural modes of a linear system from the eigenvalues of its A matrix."""

import math
from dataclasses import dataclass

import numpy as np

LN2 = math.log(2.0)


@dataclass(frozen=True)
class Mode:
    """One aperiodic mode (real eigenvalue) or one oscillatory mode (conjugate pair)."""

    eigenvalue: complex
    eigenvector: np.ndarray

    @property
    def oscillatory(self):
        return self.eigenvalue.imag != 0.0

    @property
    def stable(self):
        return self.eigenvalue.real < 0.0

    @property
    def wn(self):
        return abs(self.eigenvalue) if self.oscillatory else None

    @property
    def wd(self):
        return abs(self.eigenvalue.imag) if self.oscillatory else None

    @property
    def zeta(self):
        return -self.eigenvalue.real / abs(self.eigenvalue) if self.oscillatory else None

    @property
    def period(self):
        return 2.0 * math.pi / self.wd if self.oscillatory else None

    @property
    def time_constant(self):
        """tau = -1/Re(lambda): positive when stable, negative when unstable."""
        re = self.eigenvalue.real
        return -1.0 / re if re != 0.0 else math.inf

    @property
    def time_to_half_or_double(self):
        re = self.eigenvalue.real
        return LN2 / abs(re) if re != 0.0 else math.inf


def modes(A, imag_tol=1e-9):
    """Modes of A, fastest first. Conjugate pairs collapse to one oscillatory Mode."""
    values, vectors = np.linalg.eig(np.asarray(A, float))
    scale = max(1.0, float(np.max(np.abs(values))))
    out = []
    for value, vector in zip(values, vectors.T):
        if abs(value.imag) <= imag_tol * scale:
            out.append(Mode(complex(value.real, 0.0), np.real_if_close(vector)))
        elif value.imag > 0:
            out.append(Mode(complex(value), vector))
    return sorted(out, key=lambda m: -abs(m.eigenvalue))


def name_lateral_modes(mode_list):
    """Label the classic lateral pattern (roll, Dutch roll, spiral) when it is present."""
    oscillatory = [m for m in mode_list if m.oscillatory]
    aperiodic = sorted((m for m in mode_list if not m.oscillatory),
                       key=lambda m: -abs(m.eigenvalue))
    if len(oscillatory) != 1 or len(aperiodic) != 2:
        return {f"mode_{i + 1}": m for i, m in enumerate(mode_list)}
    return {"roll": aperiodic[0], "dutch_roll": oscillatory[0], "spiral": aperiodic[1]}


def name_longitudinal_modes(mode_list):
    """Label short period (faster pair) and phugoid (slower pair) when both oscillate."""
    oscillatory = sorted((m for m in mode_list if m.oscillatory),
                         key=lambda m: -abs(m.eigenvalue))
    if len(oscillatory) != 2 or len(mode_list) != 2:
        return {f"mode_{i + 1}": m for i, m in enumerate(mode_list)}
    return {"short_period": oscillatory[0], "phugoid": oscillatory[1]}
