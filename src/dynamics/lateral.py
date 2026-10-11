"""Linearized lateral-directional equations, MAE 4070/5070 Ch. 9 eq. (5).

State [beta, r, p, phi], controls [delta_l, delta_n], about straight level flight.
The notes take the product of inertia E as zero; here it is kept, through
A*dp/dt - E*dr/dt = L and C*dr/dt - E*dp/dt = N, which reduces to eq. (5) when E = 0.
"""

import math

import numpy as np

from .statespace import StateSpace

STATES = ("beta", "r", "p", "phi")
INPUTS = ("delta_l", "delta_n")


def lateral_state_space(aircraft, condition):
    ac, c, fc = aircraft, aircraft.lateral, condition
    qS = 0.5 * fc.rho * fc.V ** 2 * ac.S
    rate = ac.l / fc.V  # rate derivatives are per (p*l/V), (r*l/V)
    force = qS / (ac.m * fc.V)
    yaw = qS * ac.l / ac.C
    roll = qS * ac.l / ac.A

    A = np.array([
        [force * c.CY_beta,
         force * rate * c.CY_r - math.cos(fc.alpha),
         force * rate * c.CY_p + math.sin(fc.alpha),
         fc.g * math.cos(fc.theta) / fc.V],
        [yaw * c.Cn_beta, yaw * rate * c.Cn_r, yaw * rate * c.Cn_p, 0.0],
        [roll * c.Cl_beta, roll * rate * c.Cl_r, roll * rate * c.Cl_p, 0.0],
        [0.0, math.tan(fc.theta), 1.0, 0.0],
    ])
    B = np.array([
        [force * c.CY_dl, force * c.CY_dn],
        [yaw * c.Cn_dl, yaw * c.Cn_dn],
        [roll * c.Cl_dl, roll * c.Cl_dn],
        [0.0, 0.0],
    ])
    if ac.E != 0.0:
        coupling = np.eye(4)
        coupling[1, 2] = -ac.E / ac.C
        coupling[2, 1] = -ac.E / ac.A
        A, B = np.linalg.solve(coupling, A), np.linalg.solve(coupling, B)
    return StateSpace(A, B, STATES, INPUTS)
