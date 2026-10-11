"""Trim point and stability derivatives of a geometry, from the vortex-lattice solver.

Dimensional derivatives (Xu ... Nr) are in stability axes about the centre of gravity.
Non-dimensional ones follow the common b/2V, c/2V form (the one XFLR5 and AVL print), so
they can be compared directly; conversion to this tool's l/V aircraft file is done by
src.aircraft.xflr5.course_lateral.
"""

import math

import numpy as np
from scipy.optimize import root_scalar

from ..aircraft.mass import to_stability_axes
from .lattice import build_lattice
from .vlm import VortexLattice

MOTIONS = ("u", "v", "w", "p", "q", "r")
LOADS = ("X", "Y", "Z", "L", "M", "N")
METHOD = "built-in vortex lattice"


class TrimError(ValueError):
    pass


def trim(geometry, lattice, limit_deg=25.0):
    """Solver at the angle of attack where the pitching moment about the CG is zero."""
    solvers = {}

    def pitching_moment(alpha):
        solvers[alpha] = VortexLattice(lattice, geometry.mass.cg, alpha)
        return solvers[alpha].loads((1, 0, 0, 0, 0, 0))[4]

    # the moment is very nearly linear in alpha, so a secant search needs few solves
    result = root_scalar(pitching_moment, method="secant", x0=0.0, x1=math.radians(2.0),
                         xtol=1e-7, maxiter=12)
    if not result.converged or abs(result.root) > math.radians(limit_deg):
        raise TrimError(f"{geometry.name}: no pitching-moment trim within "
                        f"+/-{limit_deg:g} deg angle of attack")
    return solvers.get(result.root) or VortexLattice(lattice, geometry.mass.cg, result.root)


def stability_derivatives(geometry, n_chord=10, n_span=24):
    g = geometry
    solver = trim(g, build_lattice(g, n_chord, n_span))
    unit = solver.loads((1, 0, 0, 0, 0, 0))             # rho = 1, U = 1
    CL, CDi = -unit[2] / (0.5 * g.S), -unit[0] / (0.5 * g.S)
    if CL <= 0:
        raise TrimError(f"{g.name}: trimmed lift coefficient is {CL:.4f}; the aircraft "
                        "cannot balance its weight at the pitching-moment trim point")
    U = math.sqrt(2.0 * g.mass.m * g.g / (g.rho * g.S * CL))
    qS = 0.5 * g.rho * U ** 2 * g.S

    def loads(state):
        out = solver.loads(state, g.rho)
        velocity = np.asarray(state[:3], float)
        out[:3] -= 0.5 * g.rho * g.S * g.CD0 * np.linalg.norm(velocity) * velocity
        return out

    base = np.array([U, 0.0, 0.0, 0.0, 0.0, 0.0])
    steps = np.array([0.01 * U] * 3 + [0.01 * U / g.b] * 3)
    d = {}
    for j, motion in enumerate(MOTIONS):
        step = np.zeros(6)
        step[j] = steps[j]
        # loads are quadratic in the motion, so the central difference is exact
        slope = (loads(base + step) - loads(base - step)) / (2.0 * steps[j])
        for load, value in zip(LOADS, slope):
            d[load + motion] = float(value)

    per_p, per_q = 2.0 * U / g.b, 2.0 * U / g.c
    alpha = solver.alpha
    Isxx, Iszz, Isxz = to_stability_axes(g.mass.Ixx, g.mass.Izz, g.mass.Ixz, alpha)
    d.update({
        "Cxu": d["Xu"] * U / qS, "Cxa": d["Xw"] * U / qS,
        "Czu": d["Zu"] * U / qS + 2.0 * CL,
        "CLa": -d["Zw"] * U / qS, "CLq": -d["Zq"] * per_q / qS,
        "Cmu": d["Mu"] * U / (qS * g.c), "Cma": d["Mw"] * U / (qS * g.c),
        "Cmq": d["Mq"] * per_q / (qS * g.c),
        "CYb": d["Yv"] * U / qS, "CYp": d["Yp"] * per_p / qS, "CYr": d["Yr"] * per_p / qS,
        "Clb": d["Lv"] * U / (qS * g.b), "Clp": d["Lp"] * per_p / (qS * g.b),
        "Clr": d["Lr"] * per_p / (qS * g.b),
        "Cnb": d["Nv"] * U / (qS * g.b), "Cnp": d["Np"] * per_p / (qS * g.b),
        "Cnr": d["Nr"] * per_p / (qS * g.b),
    })
    d.update({
        "name": g.name, "version": METHOD, "panels": len(solver.lattice),
        "alpha_trim_deg": math.degrees(alpha), "V": U, "CL": CL, "CDi": CDi,
        "neutral_point": g.mass.cg[0] - d["Cma"] / d["CLa"] * g.c,
        "mass": g.mass.m, "rho": g.rho, "S": g.S, "b": g.b, "c": g.c,
        "Ibxx": g.mass.Ixx, "Ibzz": g.mass.Izz, "Ibxz": g.mass.Ixz,
        "Isxx": Isxx, "Iszz": Iszz, "Isxz": Isxz,
    })
    return d
