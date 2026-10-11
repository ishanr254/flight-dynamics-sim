"""Scalar lateral-directional metrics derived from a state-space model."""

from dataclasses import replace

from scipy.optimize import brentq

from ..analysis import modes, name_lateral_modes
from ..dynamics import lateral_state_space


def modal_metrics(ss):
    """Roll / Dutch-roll / spiral characteristics; raises if the classic pattern is absent."""
    named = name_lateral_modes(modes(ss.A))
    if "dutch_roll" not in named:
        raise ValueError("system does not have one oscillatory and two aperiodic modes")
    dutch = named["dutch_roll"]
    return {
        "roll_time_constant_s": named["roll"].time_constant,
        "dutch_roll_period_s": dutch.period,
        "dutch_roll_damping_ratio": dutch.zeta,
        "dutch_roll_natural_frequency_rad_s": dutch.wn,
        "spiral_time_constant_s": named["spiral"].time_constant,
    }


def decoupled_steady_roll_rate(ss, delta_l):
    """p_inf = -(l_dl / l_p) * delta_l from the decoupled roll equation (Ch. 9 sec. 2)."""
    p, dl = ss.states.index("p"), ss.inputs.index("delta_l")
    return -ss.B[p, dl] / ss.A[p, p] * delta_l


def spiral_neutral_factor(aircraft, condition, coefficient="Cn_beta", bracket=(0.1, 10.0)):
    """Multiplier on a lateral coefficient at which the spiral eigenvalue crosses zero."""
    def spiral_eigenvalue(factor):
        lateral = replace(aircraft.lateral,
                          **{coefficient: factor * getattr(aircraft.lateral, coefficient)})
        ss = lateral_state_space(replace(aircraft, lateral=lateral), condition)
        return min(modes(ss.A), key=lambda m: abs(m.eigenvalue)).eigenvalue.real
    return brentq(spiral_eigenvalue, *bracket)
