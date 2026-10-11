"""DF1 lateral dynamics for the three mission configurations: python3 -m cases.df1_lateral

Aircraft and reference files are generated from the XFLR5 logs by src.aircraft.xflr5.
Two versions of each mission are analysed:
  as analysed by XFLR5   -- checked against XFLR5's own eigenvalues (solver verification)
  distributed mass       -- inertia corrected with data/aircraft/df1_mass_distribution.toml
Free-response figures (controls fixed) are written to output/df1/.
"""

from math import radians
from pathlib import Path

from cases.cs7_lateral import plot_responses
from src.aircraft import load_aircraft
from src.analysis import modes, name_lateral_modes
from src.analyze import format_modes
from src.dynamics import lateral_state_space
from src.simulate import simulate
from src.verify import compare, format_table, modal_metrics

MISSIONS = ("m1", "m2", "m3")
OUTPUT = Path("output/df1")
STYLES = ("-", "--", ":")
RATES = [("beta", "deg"), ("p", "deg/s"), ("r", "deg/s")]


def paths(mission):
    return f"data/aircraft/df1_{mission}.toml", f"data/reference/df1_{mission}_lateral.toml"


def distributed_path(mission):
    return f"data/aircraft/df1_{mission}_distributed.toml"


def effect_table(as_analysed, distributed):
    rows = [f"{'metric':<36} {'XFLR5 inertia':>14} {'distributed':>12} {'change':>8}",
            "-" * 73]
    for metric, before in modal_metrics(as_analysed).items():
        after = modal_metrics(distributed)[metric]
        rows.append(f"{metric:<36} {before:>14.4g} {after:>12.4g} {after / before - 1:>+8.0%}")
    return "\n".join(rows)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    dutch_roll, spiral, m1_effect = [], [], []
    for mission, style in zip(MISSIONS, STYLES):
        aircraft_path, reference_path = paths(mission)
        aircraft, condition = load_aircraft(aircraft_path)
        ss = lateral_state_space(aircraft, condition)
        print(f"=== {aircraft.name}  (V = {condition.V:.2f} m/s) ===\n")
        print("As analysed by XFLR5, against XFLR5's eigenvalues:")
        print(format_table(compare(modal_metrics(ss), reference_path)))

        better, _ = load_aircraft(distributed_path(mission))
        ss_better = lateral_state_space(better, condition)
        print(f"\nWith distributed mass (A = {aircraft.A:.4f} -> {better.A:.4f}, "
              f"C = {aircraft.C:.4f} -> {better.C:.4f} kg m^2):")
        print(format_modes(name_lateral_modes(modes(ss_better.A))))
        print()
        print(effect_table(ss, ss_better))
        print()

        label = aircraft.name
        dutch_roll.append((label, simulate(ss_better, 4.0, x0={"beta": radians(4.0)}), style))
        spiral.append((label, simulate(ss_better, 15.0, x0={"phi": radians(5.0)}), style))
        if mission == "m1":
            x0 = {"beta": radians(4.0)}
            m1_effect = [("XFLR5 inertia (point masses on centreline)",
                          simulate(ss, 4.0, x0=x0), "--"),
                         ("distributed mass", simulate(ss_better, 4.0, x0=x0), "-")]

    plot_responses(dutch_roll, RATES, "DF1 Dutch roll, beta_0 = 4 deg, distributed mass",
                   OUTPUT / "dutch_roll.png")
    plot_responses(spiral, [("phi", "deg"), ("beta", "deg")],
                   "DF1 spiral divergence, phi_0 = 5 deg, controls fixed, distributed mass",
                   OUTPUT / "spiral.png")
    plot_responses(m1_effect, RATES, "DF1 Mission 1: effect of the mass distribution",
                   OUTPUT / "m1_inertia_effect.png")
    print(f"Figures written to {OUTPUT}/")


if __name__ == "__main__":
    main()
