"""DF1 Mission 1 on the competition course: python3 -m cases.df1_course

Flies the nonlinear model round the AIAA DBF course under the course autopilot at a set
speed and turn load factor, prints a lap summary and a step-size convergence check, and
writes figures to output/df1/. The viewer data is written by cases.viewer_scenarios.
"""

import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from src.aircraft import load_aircraft
from src.simulate import CourseAutopilot, NonlinearAircraft, load_course, simulate_nonlinear
from src.viz import scenario

AIRCRAFT = "data/aircraft/df1_m1_vlm_distributed.toml"
GEOMETRY_SPAN = 1.182      # m, DF1 wingspan (data/geometry/df1_m1.toml)
COURSE = "data/courses/dbf_2026_m1.toml"
OUTPUT = Path("output/df1")
PHASE_LABELS = {"upwind": "Upwind leg", "turn_1": "180° turn", "downwind_1": "Downwind leg",
                "full_turn": "360° turn", "downwind_2": "Downwind leg", "turn_2": "180° turn"}


def fly(course, dt=0.01, t_limit=600.0):
    aircraft, condition = load_aircraft(AIRCRAFT)
    model = NonlinearAircraft(aircraft, condition, thrust_max=course.thrust_max,
                              deflection_max=course.deflection_max)
    autopilot = CourseAutopilot(course)
    trajectory = simulate_nonlinear(model, t_limit, dt, controller=autopilot,
                                    x0=model.trim_state(altitude=course.altitude),
                                    stop=lambda: autopilot.finished)
    return model, autopilot, trajectory


def lap_summary(course, autopilot, tr):
    crossings = [0.0] + autopilot.lap_times
    laps = [b - a for a, b in zip(crossings, crossings[1:])]
    turning = np.abs(tr["phi"]) > math.radians(45.0)
    turn_rate = np.abs(np.gradient(np.unwrap(tr["psi"]), tr.t))
    radius = float(np.median(tr["ground_speed"][turning] / turn_rate[turning]))
    return {
        "laps": laps,
        "total_time": autopilot.lap_times[-1],
        "max_bank_deg": math.degrees(np.abs(tr["phi"]).max()),
        "max_load_factor": float(tr["load_factor"].max()),
        "turn_radius_m": radius,
        "speed_range": (float(tr["V"].min()), float(tr["V"].max())),
        "altitude_range": (float(tr["altitude"].min()), float(tr["altitude"].max())),
        "max_alpha_deg": math.degrees(tr["alpha"].max()),
        "max_sideslip_deg": math.degrees(np.abs(tr["beta"]).max()),
        "max_elevator_deg": math.degrees(np.abs(tr["delta_m"]).max()),
        "max_aileron_deg": math.degrees(np.abs(tr["delta_l"]).max()),
        "max_rudder_deg": math.degrees(np.abs(tr["delta_n"]).max()),
        "thrust_range": (float(tr["thrust"].min()), float(tr["thrust"].max())),
        "outside_envelope": tr.fraction_outside_envelope,
        "downwind_offset_m": autopilot.downwind_east,
    }


def summary_lines(course, s):
    ideal = course.speed ** 2 / (9.81 * math.tan(math.acos(1.0 / course.load_factor_limit)))
    return [
        f"Laps: {len(s['laps'])}, total {s['total_time']:.1f} s "
        f"({', '.join(f'{lap:.1f}' for lap in s['laps'])} s)",
        f"Speed {s['speed_range'][0]:.1f}-{s['speed_range'][1]:.1f} m/s, "
        f"altitude {s['altitude_range'][0]:.1f}-{s['altitude_range'][1]:.1f} m",
        f"Max bank {s['max_bank_deg']:.0f}°, max load factor {s['max_load_factor']:.2f}",
        f"Turn radius {s['turn_radius_m']:.0f} m (steady-turn formula: {ideal:.0f} m)",
        f"Max angle of attack {s['max_alpha_deg']:.1f}°, max sideslip "
        f"{s['max_sideslip_deg']:.1f}°",
        f"Max deflections: elevator {s['max_elevator_deg']:.1f}°, aileron "
        f"{s['max_aileron_deg']:.1f}°, rudder {s['max_rudder_deg']:.1f}°",
        f"Thrust {s['thrust_range'][0]:.1f}-{s['thrust_range'][1]:.1f} N",
        f"Time outside the aerodynamic model's valid range: {s['outside_envelope']:.1%}",
    ]


def figures(course, tr, autopilot):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(tr["north"], tr["east"], "-", color="#b31b1b", lw=1.2)
    ax.axvline(0.0, color="k", ls="--", lw=0.8)
    ax.text(2, ax.get_ylim()[0], " start line", va="bottom", fontsize=8)
    for end in (-course.half_length, course.half_length):
        ax.axvline(end, color="0.6", ls=":", lw=0.8)
    ax.set_xlabel("north / upwind (m)")
    ax.set_ylabel("east (m)")
    ax.set_aspect("equal")
    ax.set_title(f"DF1 Mission 1 ground track, {course.laps} laps")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(OUTPUT / "course_ground_track.png", dpi=130)
    plt.close(fig)

    lap_end = autopilot.lap_times[0]
    first = tr.t <= lap_end
    rows = [("phi", "bank (deg)", True), ("load_factor", "load factor", False),
            ("altitude", "altitude (m)", False), ("V", "speed (m/s)", False),
            ("delta_l", "aileron (deg)", True), ("delta_m", "elevator (deg)", True)]
    fig, axes = plt.subplots(len(rows), 1, figsize=(9, 11), sharex=True)
    for ax, (key, label, degrees) in zip(axes, rows):
        ax.plot(tr.t[first], np.degrees(tr[key][first]) if degrees else tr[key][first])
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
        for time, phase in autopilot.events:
            if time <= lap_end:
                ax.axvline(time, color="0.7", lw=0.6)
    axes[0].set_title("DF1 Mission 1, first lap (grey lines: course phases)")
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    fig.savefig(OUTPUT / "course_first_lap.png", dpi=130)
    plt.close(fig)


def course_scenario():
    """The set-speed flight as a viewer scenario."""
    course = load_course(COURSE)
    _, autopilot, tr = fly(course)
    s = lap_summary(course, autopilot, tr)
    events = [(time, PHASE_LABELS[phase]) for time, phase in autopilot.events]
    events += [(time, f"Lap {i + 1} complete") for i, time in enumerate(autopilot.lap_times)]
    ground = {"half_length": course.half_length, "downwind_east": s["downwind_offset_m"],
              "altitude": course.altitude}
    return scenario(
        tr, "df1-m1-course", "DF1 Mission 1: set speed, 2 g turns", "Course",
        "Three laps of the AIAA DBF course at a set 27.9 m/s with turns limited to a load "
        "factor of 2. Speed and turn limit here are inputs, and drag is the low XFLR5-based "
        "estimate, so this shows the course and the autopilot, not the aircraft's limits.",
        span=GEOMETRY_SPAN, summary=summary_lines(course, s), events=sorted(events),
        ground=ground)


def main():
    course = load_course(COURSE)
    model, autopilot, tr = fly(course)
    s = lap_summary(course, autopilot, tr)
    lines = summary_lines(course, s)
    print(f"{model.aircraft.name} on {course.name}\n")
    print("\n".join(lines))

    print("\nStep-size convergence (total time for all laps):")
    for dt in (0.02, 0.01, 0.005):
        _, ap, _ = fly(course, dt)
        print(f"  dt = {dt:<6} {ap.lap_times[-1]:.3f} s")

    figures(course, tr, autopilot)
    print(f"\nFigures written to {OUTPUT}/")


if __name__ == "__main__":
    main()
