"""DF1 derivatives from geometry vs XFLR5: python3 -m cases.df1_vlm

For each mission, runs the built-in vortex-lattice solver on data/geometry/df1_m*.toml and
compares (1) every stability derivative with the XFLR5 log and (2) the lateral modes that
follow from them with XFLR5's eigenvalues. Inertia is XFLR5's in both, so differences are
aerodynamic only.
"""

from pathlib import Path

from src.aero.derivatives import stability_derivatives
from src.aero.geometry import load_geometry
from src.aircraft import load_aircraft
from src.aircraft.xflr5 import aircraft_toml, log_derivatives
from src.dynamics import lateral_state_space
from src.verify import compare, format_table, modal_metrics

MISSIONS = ("m1", "m2", "m3")
TRIM = ("alpha_trim_deg", "V", "neutral_point")
LONGITUDINAL = ("CLa", "Cma", "CLq", "Cmq", "Cxu", "Cxa")
LATERAL = ("CYb", "CYp", "CYr", "Clb", "Clp", "Clr", "Cnb", "Cnp", "Cnr")
NOTES = {
    "alpha_trim_deg": "absolute difference matters here, not the ratio",
    "Cxu": "set by the zero-lift drag supplied as input",
    "Cnp": "method difference, see docs/decisions (induced velocity in the force sum)",
    "CYb": "includes -CD0 from the supplied drag; about 5% without it",
    "CYp": "small number", "Clb": "small number (no dihedral)",
}


def geometry_path(mission):
    return f"data/geometry/df1_{mission}.toml"


def vlm_aircraft_path(mission):
    return f"data/aircraft/df1_{mission}_vlm.toml"


def derivative_table(computed, reference):
    rows = [f"{'':16}{'lattice':>11}{'XFLR5':>11}{'diff':>8}", "-" * 46]
    for key in TRIM + LONGITUDINAL + LATERAL:
        diff = (computed[key] - reference[key]) / abs(reference[key])
        note = f"  {NOTES[key]}" if key in NOTES else ""
        rows.append(f"{key:16}{computed[key]:11.5f}{reference[key]:11.5f}{diff:>+8.1%}{note}")
    return "\n".join(rows)


def main():
    for mission in MISSIONS:
        geometry = load_geometry(geometry_path(mission))
        computed = stability_derivatives(geometry)
        reference = log_derivatives(
            Path(f"data/aircraft/{mission}Log.rtf").read_text(errors="replace"))
        print(f"=== {geometry.name}  ({computed['panels']} panels) ===\n")
        print(derivative_table(computed, reference))

        Path(vlm_aircraft_path(mission)).write_text(aircraft_toml(
            computed, geometry.c, geometry_path(mission), generator="src.aero.derive"))
        ss = lateral_state_space(*load_aircraft(vlm_aircraft_path(mission)))
        print("\nLateral modes from the lattice derivatives, against XFLR5's eigenvalues:")
        print(format_table(compare(modal_metrics(ss),
                                   f"data/reference/df1_{mission}_lateral.toml")))
        print()


if __name__ == "__main__":
    main()
