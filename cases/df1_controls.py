"""DF1 control derivatives and control responses: python3 -m cases.df1_controls

1. Control derivatives from the built-in lattice for Mission 1, against the design
   report's Table 4.4.4.
2. The lattice's all-moving-tail derivative against the one in the XFLR5 log (XFLR5
   rotated the whole tailplane, which is not the real elevator but is a like-for-like
   check of the method).
3. Writes aircraft files with lattice derivatives, controls and the distributed-mass
   inertia, and simulates elevator, aileron and rudder inputs. Figures in output/df1/.
"""

from dataclasses import replace
from math import radians
from pathlib import Path

import numpy as np

from cases.cs7_lateral import plot_responses
from src.aero.derivatives import stability_derivatives
from src.aero.geometry import Control, load_geometry
from src.aircraft import load_aircraft
from src.aircraft.mass import load_spread
from src.aircraft.xflr5 import aircraft_toml, log_derivatives
from src.dynamics import lateral_state_space, longitudinal_state_space
from src.simulate import simulate
from src.verify import compare, format_table

MISSIONS = ("m1", "m2", "m3")
SPREAD = "data/aircraft/df1_mass_distribution.toml"
REFERENCE = "data/reference/df1_m1_controls.toml"
OUTPUT = Path("output/df1")


def geometry_path(mission):
    return f"data/geometry/df1_{mission}.toml"


def model_path(mission):
    """Lattice derivatives with controls, distributed-mass inertia."""
    return f"data/aircraft/df1_{mission}_vlm_distributed.toml"


def control_metrics(d):
    """Control derivatives in the form the design report tabulates (span/chord based)."""
    qS = 0.5 * d["rho"] * d["V"] ** 2 * d["S"]
    return {
        "aileron_rolling_power_Cl_per_rad": d["Ldelta_l"] / (qS * d["b"]),
        "elevator_lift_CL_per_rad": -d["Zdelta_m"] / qS,
        "elevator_pitching_power_Cm_per_rad": d["Mdelta_m"] / (qS * d["c"]),
        "rudder_side_force_CY_per_rad": d["Ydelta_n"] / qS,
        "rudder_yawing_power_Cn_per_rad": d["Ndelta_n"] / (qS * d["b"]),
    }


def all_moving_tail(geometry):
    """The geometry with the whole tailplane as the elevator, as XFLR5 analysed it."""
    surfaces = []
    for surface in geometry.surfaces:
        if any(c.role == "delta_m" for c in surface.controls):
            whole = Control("all-moving tail", "delta_m", 1.0, surface.sections[0].span,
                            surface.sections[-1].span)
            surfaces.append(replace(surface, controls=(whole,)))
        else:
            surfaces.append(replace(surface, controls=()))
    return replace(geometry, surfaces=tuple(surfaces))


def pulse(ss, name, amplitude, start, end):
    """u(t): one input held at amplitude between start and end, zero otherwise."""
    index = ss.inputs.index(name)

    def u(t):
        out = np.zeros(len(ss.inputs))
        if start <= t < end:
            out[index] = amplitude
        return out
    return u


def doublet(ss, name, amplitude, start, half):
    index = ss.inputs.index(name)

    def u(t):
        out = np.zeros(len(ss.inputs))
        if start <= t < start + half:
            out[index] = amplitude
        elif start + half <= t < start + 2 * half:
            out[index] = -amplitude
        return out
    return u


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    geometry = load_geometry(geometry_path("m1"))
    derivatives = stability_derivatives(geometry)
    print(f"=== {geometry.name}: control derivatives from the lattice ===\n")
    print(format_table(compare(control_metrics(derivatives), REFERENCE)))

    tail = stability_derivatives(all_moving_tail(geometry))
    log = log_derivatives(Path("data/aircraft/m1Log.rtf").read_text(errors="replace"))
    qS = 0.5 * tail["rho"] * tail["V"] ** 2 * tail["S"]
    print("\nAll-moving tailplane, against the XFLR5 log (per rad):")
    print(f"{'':10}{'lattice':>10}{'XFLR5':>10}{'diff':>8}")
    for label, value, key in (("CZde", tail["Zdelta_m"] / qS, "CZde"),
                              ("CMde", tail["Mdelta_m"] / (qS * tail["c"]), "CMde")):
        print(f"{label:10}{value:10.4f}{log[key]:10.4f}{value / log[key] - 1:>+8.1%}")

    spread = load_spread(SPREAD)
    for mission in MISSIONS:
        g = load_geometry(geometry_path(mission))
        d = derivatives if mission == "m1" else stability_derivatives(g)
        Path(model_path(mission)).write_text(aircraft_toml(
            d, g.c, geometry_path(mission), spread, SPREAD, generator="src.aero.derive"))
    print(f"\nwrote {', '.join(model_path(m) for m in MISSIONS)}")

    aircraft, condition = load_aircraft(model_path("m1"))
    lon = longitudinal_state_space(aircraft, condition)
    lat = lateral_state_space(aircraft, condition)

    def run(ss, t_end, u):
        return [("lattice derivatives, distributed mass", simulate(ss, t_end, u=u), "-")]

    plot_responses(run(lon, 2.0, pulse(lon, "delta_m", radians(-1.0), 0.2, 2.0)),
                   [("alpha", "deg"), ("q", "deg/s"), ("theta", "deg"), ("u", "m/s")],
                   "DF1 Mission 1: elevator step of -1 deg (trailing edge up) at 0.2 s",
                   OUTPUT / "control_elevator_step.png")
    plot_responses(run(lat, 3.0, pulse(lat, "delta_l", radians(-2.0), 0.2, 0.7)),
                   [("p", "deg/s"), ("phi", "deg"), ("beta", "deg"), ("r", "deg/s")],
                   "DF1 Mission 1: aileron pulse of -2 deg for 0.5 s (roll to starboard)",
                   OUTPUT / "control_aileron_pulse.png")
    plot_responses(run(lat, 5.0, doublet(lat, "delta_n", radians(5.0), 0.2, 0.4)),
                   [("beta", "deg"), ("r", "deg/s"), ("p", "deg/s"), ("phi", "deg")],
                   "DF1 Mission 1: rudder doublet of +/-5 deg, 0.4 s each way",
                   OUTPUT / "control_rudder_doublet.png")
    print(f"Figures written to {OUTPUT}/")


if __name__ == "__main__":
    main()
