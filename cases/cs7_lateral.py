"""Reproduce MAE 4070/5070 Case Study 7 (lateral dynamics): python3 -m cases.cs7_lateral

Prints the mode table and the comparison against the case study's quoted values, and
writes the CS7 figures to output/cs7/.
"""

from dataclasses import replace
from math import degrees, radians
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from src.aircraft import load_aircraft
from src.analysis import modes, name_lateral_modes
from src.analyze import format_modes
from src.dynamics import lateral_state_space
from src.simulate import simulate
from src.verify import (compare, decoupled_steady_roll_rate, format_table, modal_metrics,
                        spiral_neutral_factor)

AIRCRAFT = "data/aircraft/cs7_transport.toml"
REFERENCE = "data/reference/cs7_lateral.toml"
OUTPUT = Path("output/cs7")
YAW_DAMPER_GAIN = 1.8  # delta_n = 1.8 r, CS7 Q3


def scaled(aircraft, coefficient, factor):
    lateral = replace(aircraft.lateral,
                      **{coefficient: factor * getattr(aircraft.lateral, coefficient)})
    return replace(aircraft, lateral=lateral)


def computed_metrics(aircraft, condition):
    ss = lateral_state_space(aircraft, condition)
    K = np.zeros((len(ss.inputs), len(ss.states)))
    K[ss.inputs.index("delta_n"), ss.states.index("r")] = YAW_DAMPER_GAIN
    damped = modal_metrics(ss.closed_loop(K))
    spiral = simulate(ss, 200.0, x0={"phi": radians(30.0)})

    metrics = modal_metrics(ss)
    metrics.update({
        "roll_rate_steady_deg_s": degrees(decoupled_steady_roll_rate(ss, radians(-5.0))),
        "spiral_neutral_Cn_beta_factor": spiral_neutral_factor(aircraft, condition),
        "dutch_roll_damping_ratio_with_yaw_damper": damped["dutch_roll_damping_ratio"],
        "spiral_time_constant_with_yaw_damper_s": damped["spiral_time_constant_s"],
        "spiral_phi_at_200s_deg": degrees(spiral["phi"][-1]),
    })
    return metrics


def plot_responses(runs, states, title, path):
    """runs: list of (label, Response, linestyle)."""
    fig, axes = plt.subplots(len(states), 1, figsize=(8, 2.4 * len(states)), sharex=True,
                             squeeze=False)
    for ax, (state, unit) in zip(axes[:, 0], states):
        for label, response, style in runs:
            ax.plot(response.t, np.degrees(response[state]), style, label=label)
        ax.set_ylabel(f"{state} ({unit})")
        ax.grid(alpha=0.3)
    axes[0, 0].set_title(title)
    axes[-1, 0].set_xlabel("Time (s)")
    if len(runs) > 1:
        axes[0, 0].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def make_figures(aircraft, condition):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    ss = lateral_state_space(aircraft, condition)
    rates = [("beta", "deg"), ("p", "deg/s"), ("r", "deg/s")]

    plot_responses([("nominal", simulate(ss, 5.0, u={"delta_l": radians(-5.0)}), "b-")],
                   [("p", "deg/s")], "Roll response to delta_l = -5 deg (CS7 Q1a)",
                   OUTPUT / "q1a_roll_response.png")
    plot_responses([("nominal", simulate(ss, 50.0, u={"delta_n": radians(-5.0)}), "k-")],
                   rates, "Response to delta_n = -5 deg (CS7 Q1b)",
                   OUTPUT / "q1b_rudder_response.png")

    for coefficient, tag in (("Cn_beta", "q2a"), ("Cl_beta", "q2b")):
        doubled = lateral_state_space(scaled(aircraft, coefficient, 2.0), condition)
        x0 = {"beta": radians(4.0)}
        plot_responses([("nominal", simulate(ss, 50.0, x0=x0), "-"),
                        (f"2 x {coefficient}", simulate(doubled, 50.0, x0=x0), "--")],
                       rates, f"Dutch roll, beta_0 = 4 deg: {coefficient} comparison (CS7 Q2)",
                       OUTPUT / f"{tag}_dutch_roll_{coefficient}.png")

    for coefficient, tag in (("Cn_beta", "q4a"), ("Cl_beta", "q4b")):
        doubled = lateral_state_space(scaled(aircraft, coefficient, 2.0), condition)
        x0 = {"phi": radians(30.0)}
        plot_responses([("nominal", simulate(ss, 200.0, x0=x0), "-"),
                        (f"2 x {coefficient}", simulate(doubled, 200.0, x0=x0), "--")],
                       [("phi", "deg"), ("beta", "deg")],
                       f"Spiral, phi_0 = 30 deg: {coefficient} comparison (CS7 Q4)",
                       OUTPUT / f"{tag}_spiral_{coefficient}.png")

    K = np.zeros((len(ss.inputs), len(ss.states)))
    K[ss.inputs.index("delta_n"), ss.states.index("r")] = YAW_DAMPER_GAIN
    x0 = {"beta": radians(5.0)}
    plot_responses([("no control", simulate(ss, 30.0, x0=x0), "-"),
                    (f"delta_n = {YAW_DAMPER_GAIN} r", simulate(ss.closed_loop(K), 30.0, x0=x0),
                     "--")],
                   rates, "Yaw damper, beta_0 = 5 deg (CS7 Q3)",
                   OUTPUT / "q3_yaw_damper.png")


def main():
    aircraft, condition = load_aircraft(AIRCRAFT)
    ss = lateral_state_space(aircraft, condition)
    print(f"{aircraft.name}\n")
    print(format_modes(name_lateral_modes(modes(ss.A))))
    print()
    print(format_table(compare(computed_metrics(aircraft, condition), REFERENCE)))
    make_figures(aircraft, condition)
    print(f"\nFigures written to {OUTPUT}/")


if __name__ == "__main__":
    main()
