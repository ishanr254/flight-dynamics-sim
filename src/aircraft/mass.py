"""Inertia bookkeeping for the aircraft-data layer (x, z in the plane of symmetry)."""

import math
import tomllib
from pathlib import Path


def box_inertia(mass, extent):
    """(Ixx, Iyy, Izz) of a uniform box about its own centre; extent = full [x, y, z] lengths."""
    lx, ly, lz = extent
    return (mass * (ly ** 2 + lz ** 2) / 12.0,
            mass * (lx ** 2 + lz ** 2) / 12.0,
            mass * (lx ** 2 + ly ** 2) / 12.0)


def load_spread(path):
    """Entries of a mass-distribution file, validated."""
    entries = tomllib.loads(Path(path).read_text()).get("spread", [])
    for entry in entries:
        name = entry.get("component", "?")
        extent = entry.get("extent")
        if not isinstance(entry.get("mass"), (int, float)) or entry["mass"] <= 0:
            raise ValueError(f"{path}: '{name}' needs a positive mass")
        if (not isinstance(extent, list) or len(extent) != 3
                or any(not isinstance(v, (int, float)) or v < 0 for v in extent)):
            raise ValueError(f"{path}: '{name}' needs extent = [x, y, z] with lengths >= 0")
    return entries


def spread_increment(entries):
    """Total (dIxx, dIyy, dIzz) from replacing each point mass by its box. dIxz is zero."""
    totals = [0.0, 0.0, 0.0]
    for entry in entries:
        for i, value in enumerate(box_inertia(entry["mass"], entry["extent"])):
            totals[i] += value
    return tuple(totals)


def to_stability_axes(Ixx, Izz, Ixz, alpha):
    """Rotate body-axis (Ixx, Izz, Ixz) about y by the trim angle of attack (radians)."""
    c, s = math.cos(alpha), math.sin(alpha)
    return (Ixx * c * c + Izz * s * s - 2.0 * Ixz * s * c,
            Izz * c * c + Ixx * s * s + 2.0 * Ixz * s * c,
            Ixz * (c * c - s * s) + (Ixx - Izz) * s * c)
