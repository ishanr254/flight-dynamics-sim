"""Modal analysis of any aircraft data file: python3 -m src.analyze <aircraft.toml>

Prints the state matrices and natural modes for whichever of the lateral and longitudinal
sections the file contains.
"""

import sys

import numpy as np

from .aircraft import AircraftDataError, load_aircraft
from .analysis import modes, name_lateral_modes, name_longitudinal_modes
from .dynamics import lateral_state_space, longitudinal_state_space


def format_modes(named):
    lines = [f"{'mode':<12} {'eigenvalue [1/s]':>24} {'wn [rad/s]':>11} {'zeta':>7} "
             f"{'period [s]':>11} {'tau [s]':>10}  stable",
             "-" * 86]
    for name, m in named.items():
        ev = m.eigenvalue
        eig = f"{ev.real:+.4f} ± {abs(ev.imag):.4f}j" if m.oscillatory else f"{ev.real:+.5f}"
        osc = (f"{m.wn:>11.4f} {m.zeta:>7.3f} {m.period:>11.3f}" if m.oscillatory
               else f"{'-':>11} {'-':>7} {'-':>11}")
        lines.append(f"{name:<12} {eig:>24} {osc} {m.time_constant:>10.3f}  "
                     f"{'yes' if m.stable else 'NO'}")
    return "\n".join(lines)


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        print(__doc__, file=sys.stderr)
        return 2
    try:
        aircraft, condition = load_aircraft(args[0])
    except (AircraftDataError, OSError) as exc:
        print(exc, file=sys.stderr)
        return 1

    np.set_printoptions(precision=5, suppress=True, linewidth=120)
    print(f"{aircraft.name}  ({aircraft.source})")
    print(f"V = {condition.V:g} m/s, rho = {condition.rho:g} kg/m^3, "
          f"alpha = {condition.alpha:g} rad, theta = {condition.theta:g} rad")
    sections = (("Longitudinal", aircraft.longitudinal, longitudinal_state_space,
                 name_longitudinal_modes),
                ("Lateral", aircraft.lateral, lateral_state_space, name_lateral_modes))
    for title, present, build, name_modes in sections:
        if present is None:
            continue
        ss = build(aircraft, condition)
        print(f"\n{title} A, states {ss.states}:\n{ss.A}\n")
        print(f"{title} B, inputs {ss.inputs}:\n{ss.B}\n")
        print(format_modes(name_modes(modes(ss.A))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
