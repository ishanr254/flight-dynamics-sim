"""Vortex-ring lattice on the mean camber surface of each lifting surface."""

import math
from dataclasses import dataclass

import numpy as np

from .airfoil import camber_line


@dataclass(frozen=True)
class Lattice:
    """N vortex rings A->B->C->D (A-B is the bound segment at the panel quarter chord)."""

    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    D: np.ndarray
    trailing_edge: np.ndarray   # ring sheds the wake from C and D
    control: np.ndarray         # collocation points at the panel three-quarter chord
    normal: np.ndarray
    surface: np.ndarray         # index of the owning surface

    def __len__(self):
        return len(self.A)


def _half_stations(sections, n_span):
    """Spanwise stations from root to tip, uniform within each section interval."""
    total = sections[-1].span - sections[0].span
    stations = [(0, 0.0)]
    for k, (a, b) in enumerate(zip(sections, sections[1:])):
        n = max(1, round(n_span * (b.span - a.span) / total))
        stations += [(k, (i + 1) / n) for i in range(n)]
    return stations


def surface_grid(surface, n_chord, n_span):
    """Camber-surface points, shape (n_chord + 1, stations, 3), tip to tip if symmetric."""
    sections = surface.sections
    cambers = [camber_line(s.airfoil) for s in sections]
    xi = np.linspace(0.0, 1.0, n_chord + 1)
    ex = np.array([1.0, 0.0, 0.0])

    # leading-edge position of every section, accumulated along the dihedral of each panel
    roots = [np.zeros(3)]
    span_dirs = []
    for a, b in zip(sections, sections[1:]):
        d = math.radians(a.dihedral_deg)
        e_s = (np.array([0.0, 0.0, 1.0]) if surface.vertical
               else np.array([0.0, math.cos(d), math.sin(d)]))
        span_dirs.append(e_s)
        roots.append(roots[-1] + (b.span - a.span) * e_s)

    columns = []
    for k, t in _half_stations(sections, n_span):
        a, b = sections[k], sections[min(k + 1, len(sections) - 1)]
        k_dir = min(k, len(span_dirs) - 1)
        e_s = span_dirs[k_dir]
        e_n = np.cross(ex, e_s)
        chord = a.chord + t * (b.chord - a.chord)
        twist = math.radians(a.twist_deg + t * (b.twist_deg - a.twist_deg))
        zc = (1 - t) * cambers[k](xi) + t * cambers[min(k + 1, len(sections) - 1)](xi)
        le = (roots[k] + t * (roots[min(k + 1, len(roots) - 1)] - roots[k])
              + (a.x_offset + t * (b.x_offset - a.x_offset)) * ex)
        along = xi * math.cos(twist) + zc * math.sin(twist)
        up = -xi * math.sin(twist) + zc * math.cos(twist)
        columns.append(le + chord * (along[:, None] * ex + up[:, None] * e_n))
    grid = np.stack(columns, axis=1)

    inc = math.radians(surface.incidence_deg)
    x, z = grid[..., 0].copy(), grid[..., 2].copy()
    grid[..., 0] = x * math.cos(inc) + z * math.sin(inc)
    grid[..., 2] = -x * math.sin(inc) + z * math.cos(inc)
    grid += np.asarray(surface.position)

    if surface.symmetric:
        mirror = grid[:, :0:-1].copy()
        mirror[..., 1] *= -1.0
        grid = np.concatenate([mirror, grid], axis=1)
    return grid


def build_lattice(geometry, n_chord=8, n_span=16):
    """n_span is the panel count per half of the largest surface; others scale by span."""
    largest = max(s.sections[-1].span - s.sections[0].span for s in geometry.surfaces)
    parts = {name: [] for name in ("A", "B", "C", "D", "te", "cp", "n", "surf")}
    for index, surface in enumerate(geometry.surfaces):
        extent = surface.sections[-1].span - surface.sections[0].span
        P = surface_grid(surface, n_chord, max(2, round(n_span * extent / largest)))
        quarter = P[:-1] + 0.25 * (P[1:] - P[:-1])
        aft = np.concatenate([quarter[1:], P[-1:]], axis=0)
        A, B = quarter[:, :-1], quarter[:, 1:]
        D, C = aft[:, :-1], aft[:, 1:]
        three_quarter = P[:-1] + 0.75 * (P[1:] - P[:-1])
        cp = 0.5 * (three_quarter[:, :-1] + three_quarter[:, 1:])
        chordwise = 0.5 * ((P[1:, :-1] - P[:-1, :-1]) + (P[1:, 1:] - P[:-1, 1:]))
        spanwise = 0.5 * ((P[:-1, 1:] - P[:-1, :-1]) + (P[1:, 1:] - P[1:, :-1]))
        normal = np.cross(chordwise, spanwise)
        normal /= np.linalg.norm(normal, axis=-1, keepdims=True)
        te = np.zeros(A.shape[:2], bool)
        te[-1] = True
        for key, value in (("A", A), ("B", B), ("C", C), ("D", D), ("cp", cp), ("n", normal)):
            parts[key].append(value.reshape(-1, 3))
        parts["te"].append(te.ravel())
        parts["surf"].append(np.full(te.size, index))
    cat = {key: np.concatenate(value) for key, value in parts.items()}
    return Lattice(cat["A"], cat["B"], cat["C"], cat["D"], cat["te"], cat["cp"], cat["n"],
                   cat["surf"])
