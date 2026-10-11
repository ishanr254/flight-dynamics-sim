"""Aircraft and flight-condition data containers (SI units, radians)."""

from dataclasses import dataclass

STANDARD_GRAVITY = 9.81


@dataclass(frozen=True)
class LateralCoefficients:
    """Lateral-directional derivatives; rate derivatives are per (p*l/V), (r*l/V)."""

    CY_beta: float
    CY_p: float
    CY_r: float
    CY_dl: float
    CY_dn: float
    Cl_beta: float
    Cl_p: float
    Cl_r: float
    Cl_dl: float
    Cl_dn: float
    Cn_beta: float
    Cn_p: float
    Cn_r: float
    Cn_dl: float
    Cn_dn: float


@dataclass(frozen=True)
class Aircraft:
    name: str
    source: str
    m: float
    S: float
    l: float
    A: float
    C: float
    E: float
    lateral: LateralCoefficients


@dataclass(frozen=True)
class FlightCondition:
    V: float
    rho: float
    alpha: float = 0.0
    theta: float = 0.0
    g: float = STANDARD_GRAVITY
