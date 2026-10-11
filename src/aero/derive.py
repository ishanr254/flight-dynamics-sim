"""Stability derivatives from geometry: python3 -m src.aero.derive GEOMETRY.toml --aircraft OUT.toml

Trims the aircraft (zero pitching moment, lift = weight), computes the stability
derivatives with the built-in vortex-lattice solver, prints them, and writes an aircraft
file that src.analyze and the rest of the tool read directly.
"""

import argparse
import sys
from pathlib import Path

from ..aircraft.mass import load_spread
from ..aircraft.xflr5 import aircraft_toml
from .airfoil import AirfoilError
from .derivatives import TrimError, stability_derivatives
from .geometry import GeometryError, load_geometry

LONGITUDINAL = ("CLa", "Cma", "CLq", "Cmq", "Cxu", "Cxa")
LATERAL = ("CYb", "CYp", "CYr", "Clb", "Clp", "Clr", "Cnb", "Cnp", "Cnr")


def summary(d):
    lines = [
        f"{d['name']}  ({d['version']}, {d['panels']} panels)",
        f"trim: alpha = {d['alpha_trim_deg']:.3f} deg, V = {d['V']:.2f} m/s, "
        f"CL = {d['CL']:.4f}, CDi = {d['CDi']:.5f}",
        f"neutral point x = {d['neutral_point']:.4f} m",
        "",
        "longitudinal (per rad, per q*c/2V):  "
        + "  ".join(f"{k} = {d[k]:.4f}" for k in LONGITUDINAL),
        "lateral (per rad, per p*b/2V):       "
        + "  ".join(f"{k} = {d[k]:.4f}" for k in LATERAL),
    ]
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("geometry")
    parser.add_argument("--aircraft", help="aircraft TOML to write")
    parser.add_argument("--spread", help="mass-distribution TOML to add to the inertia")
    parser.add_argument("--panels", default="10x24", metavar="CHORDxSPAN",
                        help="chordwise x spanwise (per half of the largest surface) panels")
    args = parser.parse_args(argv)
    try:
        n_chord, n_span = (int(v) for v in args.panels.lower().split("x"))
    except ValueError:
        parser.error("--panels must look like 10x24")
    try:
        geometry = load_geometry(args.geometry)
        d = stability_derivatives(geometry, n_chord, n_span)
    except (GeometryError, AirfoilError, TrimError, OSError) as exc:
        print(exc, file=sys.stderr)
        return 1
    print(summary(d))
    if args.aircraft:
        spread = load_spread(args.spread) if args.spread else None
        Path(args.aircraft).write_text(aircraft_toml(
            d, geometry.c, args.geometry, spread, args.spread or "",
            generator="src.aero.derive"))
        print(f"\nwrote {args.aircraft}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
