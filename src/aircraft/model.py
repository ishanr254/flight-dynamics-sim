"""Aircraft and flight-condition data containers (SI units, radians).

Conventions are fixed in docs/decisions/ and described for users in docs/aircraft-file.md.
"""

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
class LongitudinalCoefficients:
    """Stability-axis longitudinal derivatives; rate derivatives are per (q*l/V).

    Each is the dimensional derivative divided by qS (and by l for the moment), times V
    for speed derivatives, so CZ_u is about -2*CL at trim.
    """

    CX_u: float
    CX_alpha: float
    CZ_u: float
    CZ_alpha: float
    CZ_q: float
    Cm_u: float
    Cm_alpha: float
    Cm_q: float
    # elevator derivatives per radian (trailing edge down positive); zero if not supplied
    CX_dm: float = 0.0
    CZ_dm: float = 0.0
    Cm_dm: float = 0.0


@dataclass(frozen=True)
class Aircraft:
    """Inertias A (roll), B (pitch), C (yaw), E (xz product); None where not supplied."""

    name: str
    source: str
    m: float
    S: float
    l: float
    A: float | None = None
    B: float | None = None
    C: float | None = None
    E: float = 0.0
    lateral: LateralCoefficients | None = None
    longitudinal: LongitudinalCoefficients | None = None


@dataclass(frozen=True)
class FlightCondition:
    V: float
    rho: float
    alpha: float = 0.0
    theta: float = 0.0
    g: float = STANDARD_GRAVITY
