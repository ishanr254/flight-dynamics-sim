"""Linearized longitudinal equations in the four-state small-perturbation form.

State [u, alpha, q, theta], controls [delta_x, delta_m], stability axes, about steady
straight flight at climb angle theta_0 = condition.theta. Derivation and coefficient
definitions: docs/decisions/2026-10-11-longitudinal-convention.md.
"""

import math

import numpy as np

from .statespace import StateSpace

STATES = ("u", "alpha", "q", "theta")
INPUTS = ("delta_x", "delta_m")


def longitudinal_state_space(aircraft, condition):
    ac, c, fc = aircraft, aircraft.longitudinal, condition
    if c is None:
        raise ValueError(f"{ac.name}: the aircraft file has no [longitudinal] section")

    V = fc.V
    qS = 0.5 * fc.rho * V ** 2 * ac.S
    force = qS / ac.m
    moment = qS * ac.l / ac.B
    rate = ac.l / V  # rate derivatives are per (q*l/V)

    A = np.array([
        [force * c.CX_u / V, force * c.CX_alpha, 0.0, -fc.g * math.cos(fc.theta)],
        [force * c.CZ_u / V ** 2, force * c.CZ_alpha / V, 1.0 + force * rate * c.CZ_q / V,
         -fc.g * math.sin(fc.theta) / V],
        [moment * c.Cm_u / V, moment * c.Cm_alpha, moment * rate * c.Cm_q, 0.0],
        [0.0, 0.0, 1.0, 0.0],
    ])
    # throttle (delta_x) has no derivative in the aircraft file yet
    B = np.array([
        [0.0, force * c.CX_dm],
        [0.0, force * c.CZ_dm / V],
        [0.0, moment * c.Cm_dm],
        [0.0, 0.0],
    ])
    return StateSpace(A, B, STATES, INPUTS)
