"""Steady vortex-lattice solver.

Geometry axes G: x aft, y starboard, z up. Stability axes S at the reference angle of
attack: x_s forward along the flight path, y_s starboard, z_s down. The wake leaves the
trailing edges along the reference freestream and stays there, so the circulation is
linear in the motion (u, v, w, p, q, r) and the loads are quadratic in it.

Loads are Kutta-Joukowski forces on the spanwise (bound) segments, using the local
relative wind plus the velocity induced by the whole lattice, so induced drag is included.
Forces on chordwise segments are neglected, as in AVL: they are small and, at junctions
between surfaces, numerically ill-conditioned.

A vortex acting on a different surface has a finite core (a fraction of its ring's span
width), which keeps junctions and wake crossings between surfaces well behaved. Within a
surface the vortices are singular lines, so the basic lifting solution is unaffected.
"""

import math

import numpy as np
from scipy.linalg import lu_factor, lu_solve

FAR = 1.0e5        # wake length, metres
CUTOFF = 1.0e-8    # points closer than this to a vortex line see no velocity from it
CORE_FRACTION = 0.5    # core radius / ring span width, between different surfaces


def segment_velocity(P, A, B, core2=0.0):
    """Velocity at points P (M,3) from unit-strength straight vortices A->B (N,3): (M,N,3).

    core2 is the squared core radius, a scalar or an (M,N) array.
    """
    r1 = P[:, None, :] - A[None, :, :]
    r2 = P[:, None, :] - B[None, :, :]
    r0 = (B - A)[None, :, :]
    cross = np.cross(r1, r2)
    cross2 = np.einsum("mnk,mnk->mn", cross, cross)
    n1, n2 = np.linalg.norm(r1, axis=-1), np.linalg.norm(r2, axis=-1)
    length2 = np.einsum("mnk,mnk->mn", r0, r0)
    near = (cross2 <= CUTOFF ** 2 * length2) | (n1 < CUTOFF) | (n2 < CUTOFF)
    with np.errstate(divide="ignore", invalid="ignore"):
        k = (np.einsum("mnk,mnk->mn", r0, r1) / n1
             - np.einsum("mnk,mnk->mn", r0, r2) / n2) / (4.0 * math.pi
                                                             * (cross2 + core2 * length2))
    k[near] = 0.0
    return cross * k[:, :, None]


def ring_velocity(P, point_surface, lattice, wake_direction, core_fraction=CORE_FRACTION):
    """Velocity at P from each unit-strength ring, including its wake: (M, N, 3)."""
    lat = lattice
    te = lat.trailing_edge
    width = np.linalg.norm(lat.B - lat.A, axis=-1)
    core2 = np.where(point_surface[:, None] != lat.surface[None, :],
                     (core_fraction * width[None, :]) ** 2, 0.0)
    V = segment_velocity(P, lat.A, lat.B, core2)
    V += segment_velocity(P, lat.B, lat.C, core2)
    V += segment_velocity(P, lat.D, lat.A, core2)
    V[:, ~te] += segment_velocity(P, lat.C[~te], lat.D[~te], core2[:, ~te])
    far = FAR * np.asarray(wake_direction)
    V[:, te] += segment_velocity(P, lat.C[te], lat.C[te] + far, core2[:, te])
    V[:, te] += segment_velocity(P, lat.D[te] + far, lat.D[te], core2[:, te])
    return V


class VortexLattice:
    def __init__(self, lattice, cg, alpha):
        self.lattice, self.cg, self.alpha = lattice, np.asarray(cg, float), alpha
        flow = np.array([math.cos(alpha), 0.0, math.sin(alpha)])
        # columns are the stability axes expressed in geometry axes
        self.axes = np.column_stack([-flow, [0.0, 1.0, 0.0],
                                     [math.sin(alpha), 0.0, -math.cos(alpha)]])
        influence = ring_velocity(lattice.control, lattice.surface, lattice, flow)
        self._lu = lu_factor(np.einsum("mnk,mk->mn", influence, lattice.normal))

        # bound segments: every ring's leading edge, and the trailing edge of each ring
        # that is not at the surface trailing edge (there the wake cancels it)
        lat, te = lattice, lattice.trailing_edge
        index = np.arange(len(lat))
        self._owner = np.concatenate([index, index[~te]])
        start = np.concatenate([lat.A, lat.C[~te]])
        end = np.concatenate([lat.B, lat.D[~te]])
        self._mid, self._vector = 0.5 * (start + end), end - start
        self._induced = ring_velocity(self._mid, lat.surface[self._owner], lattice, flow)

    def relative_wind(self, points, state):
        """Air velocity relative to the aircraft at points, in G. state = (U,v,w,p,q,r) in S."""
        velocity = self.axes @ np.asarray(state[:3], float)
        rate = self.axes @ np.asarray(state[3:], float)
        return -(velocity + np.cross(rate, points - self.cg))

    def circulation(self, state):
        rhs = -np.einsum("mk,mk->m", self.relative_wind(self.lattice.control, state),
                         self.lattice.normal)
        return lu_solve(self._lu, rhs)

    def loads(self, state, rho=1.0):
        """Force (X, Y, Z) and moment (L, M, N) about the CG, in stability axes."""
        gamma = self.circulation(state)
        wind = (self.relative_wind(self._mid, state)
                + np.einsum("knc,n->kc", self._induced, gamma))
        force = rho * gamma[self._owner, None] * np.cross(wind, self._vector)
        moment = np.cross(self._mid - self.cg, force)
        return np.concatenate([self.axes.T @ force.sum(axis=0),
                               self.axes.T @ moment.sum(axis=0)])
