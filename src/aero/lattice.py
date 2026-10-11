"""Vortex-ring lattice on the mean camber surface of each lifting surface."""

import math
from dataclasses import dataclass, field

import numpy as np

from .airfoil import camber_line

# sign of the normal rotation for a positive deflection, by role and by side (+1 starboard)
# "+1" turns the trailing edge away from the panel normal, which adds lift along the normal
DEFLECTION_SIGN = {
    "delta_m": lambda side: 1.0,     # trailing edge down on both halves
    "delta_l": lambda side: side,    # starboard trailing edge down, port up
    "delta_n": lambda side: -1.0,    # fin normal points to port; trailing edge to port
}


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
    # role -> (N, 3) change of each panel normal per radian of that control
    deflection: dict = field(default_factory=dict)

    def __len__(self):
        return len(self.A)


def _distribute(breaks, n_total):
    """Nodes with one at every break and about n_total uniform panels in between."""
    breaks = sorted(breaks)
    total = breaks[-1] - breaks[0]
    nodes = [breaks[0]]
    for a, b in zip(breaks, breaks[1:]):
        n = max(1, round(n_total * (b - a) / total))
        nodes += [a + (b - a) * (i + 1) / n for i in range(n)]
    return np.array(nodes)


def _unique(values, tolerance=1e-9):
    out = []
    for v in sorted(values):
        if not out or v - out[-1] > tolerance:
            out.append(v)
    return out


def surface_mesh(surface, n_chord, n_span):
    """Camber-surface points (chord nodes, stations, 3), tip to tip if symmetric.

    Also returns the chordwise node fractions and the spanwise node positions of one half.
    Panel edges are placed on every hinge line and every control's spanwise edge.
    """
    sections = surface.sections
    cambers = [camber_line(s.airfoil) for s in sections]
    root, tip = sections[0].span, sections[-1].span
    xi = _distribute(_unique([0.0, 1.0] + [1.0 - c.chord_fraction for c in surface.controls
                                           if c.chord_fraction < 1.0]), n_chord)
    spans = _distribute(_unique([s.span for s in sections]
                                + [e for c in surface.controls
                                   for e in (c.span_from, c.span_to)]), n_span)
    ex = np.array([1.0, 0.0, 0.0])

    # leading-edge position of every section, accumulated along the dihedral of each panel
    roots, span_dirs = [np.zeros(3)], []
    for a, b in zip(sections, sections[1:]):
        d = math.radians(a.dihedral_deg)
        e_s = (np.array([0.0, 0.0, 1.0]) if surface.vertical
               else np.array([0.0, math.cos(d), math.sin(d)]))
        span_dirs.append(e_s)
        roots.append(roots[-1] + (b.span - a.span) * e_s)

    columns = []
    for s in spans:
        k = min(max(np.searchsorted([sec.span for sec in sections], s, side="right") - 1, 0),
                len(sections) - 2)
        a, b = sections[k], sections[k + 1]
        t = (s - a.span) / (b.span - a.span)
        e_s = span_dirs[k]
        e_n = np.cross(ex, e_s)
        chord = a.chord + t * (b.chord - a.chord)
        twist = math.radians(a.twist_deg + t * (b.twist_deg - a.twist_deg))
        zc = (1 - t) * cambers[k](xi) + t * cambers[k + 1](xi)
        le = (roots[k] + t * (roots[k + 1] - roots[k])
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
    return grid, xi, spans


def _deflection_gains(surface, xi, spans, chordwise, normal):
    """role -> (chord panels, span panels, 3): normal change per radian of deflection."""
    n_chord, n_half = len(xi) - 1, len(spans) - 1
    xi_start = xi[:-1]
    mid = 0.5 * (spans[:-1] + spans[1:])
    if surface.symmetric:       # columns run port tip -> root -> starboard tip
        mid = np.concatenate([mid[::-1], mid])
        side = np.concatenate([-np.ones(n_half), np.ones(n_half)])
    else:
        side = np.ones(n_half)
    tangent = chordwise - np.einsum("ijk,ijk->ij", chordwise, normal)[..., None] * normal
    tangent /= np.linalg.norm(tangent, axis=-1, keepdims=True)

    gains = {}
    for control in surface.controls:
        behind_hinge = xi_start >= 1.0 - control.chord_fraction - 1e-9
        within = (mid > control.span_from) & (mid < control.span_to)
        sign = np.array([DEFLECTION_SIGN[control.role](s) for s in side])
        weight = behind_hinge[:, None] * (within * sign)[None, :]
        gains[control.role] = (gains.get(control.role, np.zeros((n_chord, len(mid), 3)))
                               + weight[..., None] * tangent)
    return gains


def build_lattice(geometry, n_chord=8, n_span=16):
    """n_span is the panel count per half of the largest surface; others scale by span."""
    largest = max(s.sections[-1].span - s.sections[0].span for s in geometry.surfaces)
    parts = {name: [] for name in ("A", "B", "C", "D", "te", "cp", "n", "surf")}
    gains, sizes = [], []
    for index, surface in enumerate(geometry.surfaces):
        extent = surface.sections[-1].span - surface.sections[0].span
        P, xi, spans = surface_mesh(surface, n_chord, max(2, round(n_span * extent / largest)))
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
        gains.append({role: gain.reshape(-1, 3) for role, gain in
                      _deflection_gains(surface, xi, spans, chordwise, normal).items()})
        sizes.append(te.size)
    cat = {key: np.concatenate(value) for key, value in parts.items()}

    deflection = {}
    for role in sorted({role for gain in gains for role in gain}):
        deflection[role] = np.concatenate([gain.get(role, np.zeros((size, 3)))
                                           for gain, size in zip(gains, sizes)])
    return Lattice(cat["A"], cat["B"], cat["C"], cat["D"], cat["te"], cat["cp"], cat["n"],
                   cat["surf"], deflection)
