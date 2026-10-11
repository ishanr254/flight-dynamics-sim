"""Mean camber lines: NACA 4-digit by formula, anything else from a Selig .dat file."""

import re
from pathlib import Path

import numpy as np

AIRFOIL_DIR = Path("data/airfoils")


class AirfoilError(ValueError):
    pass


def naca4_camber(code):
    m, p = int(code[0]) / 100.0, int(code[1]) / 10.0

    def camber(x):
        x = np.asarray(x, float)
        if m == 0.0 or p == 0.0:
            return np.zeros_like(x)
        front = m / p ** 2 * (2 * p * x - x ** 2)
        rear = m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x ** 2)
        return np.where(x < p, front, rear)
    return camber


def selig_camber(path):
    """Camber line of a Selig-format file (upper TE -> LE -> lower TE), chord-normalized."""
    rows = []
    for line in Path(path).read_text().splitlines():
        parts = line.split()
        try:
            if len(parts) == 2:
                rows.append((float(parts[0]), float(parts[1])))
        except ValueError:
            continue  # header line
    points = np.array(rows)
    if len(points) < 10:
        raise AirfoilError(f"{path}: expected airfoil coordinates, found {len(points)} points")
    nose = int(np.argmin(points[:, 0]))
    upper, lower = points[:nose + 1][::-1], points[nose:]
    if len(upper) < 3 or len(lower) < 3:
        raise AirfoilError(f"{path}: not in Selig order (upper surface first, from the "
                           "trailing edge)")
    x0, chord = points[nose, 0], points[:, 0].max() - points[nose, 0]

    def camber(x):
        xa = x0 + np.asarray(x, float) * chord
        mean = 0.5 * (np.interp(xa, upper[:, 0], upper[:, 1])
                      + np.interp(xa, lower[:, 0], lower[:, 1]))
        return (mean - points[nose, 1]) / chord
    return camber


def camber_line(name, airfoil_dir=AIRFOIL_DIR):
    """Function x/c -> z/c for the named airfoil."""
    naca = re.fullmatch(r"\s*NACA[\s-]*(\d{4})\s*", name, flags=re.IGNORECASE)
    if naca:
        return naca4_camber(naca.group(1))
    path = Path(airfoil_dir) / f"{name}.dat"
    if not path.exists():
        raise AirfoilError(f"airfoil '{name}': no file {path} (only NACA 4-digit sections "
                           "are built in)")
    return selig_camber(path)
