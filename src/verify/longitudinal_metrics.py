"""Scalar longitudinal metrics derived from a state-space model, and textbook estimates."""

import math

import numpy as np

from ..analysis import modes, name_longitudinal_modes


def modal_metrics(ss):
    """Short-period and phugoid characteristics; raises if either is not oscillatory."""
    named = name_longitudinal_modes(modes(ss.A))
    if "short_period" not in named:
        raise ValueError("system does not have two oscillatory modes")
    sp, ph = named["short_period"], named["phugoid"]
    return {
        "short_period_natural_frequency_rad_s": sp.wn,
        "short_period_damping_ratio": sp.zeta,
        "phugoid_natural_frequency_rad_s": ph.wn,
        "phugoid_period_s": ph.period,
        "phugoid_damping_ratio": ph.zeta,
    }


def short_period_approximation(ss):
    """(wn, zeta) of the decoupled alpha-q system, the approximation of course Ch. 5 sec. 5."""
    i, j = ss.states.index("alpha"), ss.states.index("q")
    sub = ss.A[np.ix_([i, j], [i, j])]
    wn = math.sqrt(np.linalg.det(sub))
    return wn, -np.trace(sub) / (2.0 * wn)


def lanchester_phugoid_period(V, g=9.81):
    """T = sqrt(2) * pi * V / g, about 0.45 V in SI (course Ch. 5 sec. 3.3)."""
    return math.sqrt(2.0) * math.pi * V / g
