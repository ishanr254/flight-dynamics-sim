"""DF1 longitudinal dynamics for the three missions: python3 -m cases.df1_longitudinal

For each mission:
  1. XFLR5's derivatives through this tool's solver, against XFLR5's own eigenvalues
     (verifies the dynamics code).
  2. The same with the mass distribution from the design report (pitch inertia grows).
  3. The textbook estimates from the course notes, for comparison.
Free-response figures (controls fixed, distributed mass) are written to output/df1/.
"""

from math import radians
from pathlib import Path

from cases.cs7_lateral import plot_responses
from src.aircraft import load_aircraft
from src.analysis import modes, name_longitudinal_modes
from src.analyze import format_modes
from src.dynamics import longitudinal_state_space
from src.simulate import simulate
from src.verify import (compare, format_table, lanchester_phugoid_period,
                        longitudinal_modal_metrics, short_period_approximation)

MISSIONS = ("m1", "m2", "m3")
OUTPUT = Path("output/df1")
STYLES = ("-", "--", ":")
STATES = [("u", "m/s"), ("alpha", "deg"), ("q", "deg/s"), ("theta", "deg")]


def paths(mission):
    return (f"data/aircraft/df1_{mission}.toml",
            f"data/reference/df1_{mission}_longitudinal.toml")


def distributed_path(mission):
    return f"data/aircraft/df1_{mission}_distributed.toml"


def estimates_table(ss, condition):
    """Full-model values next to the decoupled estimates of course Ch. 5."""
    full = longitudinal_modal_metrics(ss)
    wn, zeta = short_period_approximation(ss)
    rows = [
        ("short-period natural frequency (rad/s)",
         full["short_period_natural_frequency_rad_s"], wn, "alpha-q subsystem"),
        ("short-period damping ratio", full["short_period_damping_ratio"], zeta,
         "alpha-q subsystem"),
        ("phugoid period (s)", full["phugoid_period_s"],
         lanchester_phugoid_period(condition.V, condition.g), "sqrt(2) pi V / g"),
    ]
    lines = [f"{'':40}{'full model':>11}{'estimate':>10}{'diff':>8}  estimate from",
             "-" * 86]
    for label, value, estimate, origin in rows:
        lines.append(f"{label:40}{value:11.4f}{estimate:10.4f}"
                     f"{estimate / value - 1:>+8.1%}  {origin}")
    return "\n".join(lines)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    short_period, phugoid = [], []
    for mission, style in zip(MISSIONS, STYLES):
        aircraft_path, reference_path = paths(mission)
        aircraft, condition = load_aircraft(aircraft_path)
        ss = longitudinal_state_space(aircraft, condition)
        print(f"=== {aircraft.name}  (V = {condition.V:.2f} m/s) ===\n")
        print("As analysed by XFLR5, against XFLR5's eigenvalues:")
        print(format_table(compare(longitudinal_modal_metrics(ss), reference_path)))

        better, _ = load_aircraft(distributed_path(mission))
        ss_better = longitudinal_state_space(better, condition)
        print(f"\nWith distributed mass (B = {aircraft.B:.4f} -> {better.B:.4f} kg m^2):")
        print(format_modes(name_longitudinal_modes(modes(ss_better.A))))
        print("\nAgainst the course-notes estimates (distributed mass):")
        print(estimates_table(ss_better, condition))
        print()

        short_period.append(
            (aircraft.name, simulate(ss_better, 1.5, x0={"alpha": radians(2.0)}), style))
        phugoid.append((aircraft.name, simulate(ss_better, 90.0, x0={"u": 2.0}), style))

    plot_responses(short_period, STATES[1:],
                   "DF1 short period: released from alpha = +2 deg, controls fixed",
                   OUTPUT / "short_period.png")
    plot_responses(phugoid, [STATES[0], STATES[3], STATES[1]],
                   "DF1 phugoid: released from u = +2 m/s, controls fixed",
                   OUTPUT / "phugoid.png")
    print(f"Figures written to {OUTPUT}/")


if __name__ == "__main__":
    main()
