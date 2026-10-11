"""DF1 Mission 1 performance limits and limit laps: python3 -m cases.df1_performance

1. Propulsion model against the static thrust test.
2. Performance envelope (stall, top speed, turn limits, endurance) against the report.
3. Laps of the course for a range of structural load-factor limits, flown two ways: full
   throttle everywhere, and slowing to a chosen speed before each turn. Lap time, speed
   lost in the turns, peak loads and battery use.
Figures are written to output/df1/.
"""

import math
from dataclasses import replace
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from cases.df1_course import AIRCRAFT, COURSE, GEOMETRY_SPAN, PHASE_LABELS, lap_summary
from src.aircraft import load_aircraft
from src.performance import Envelope, load_performance
from src.simulate import (CourseAutopilot, Gains, NonlinearAircraft, load_course,
                          simulate_nonlinear)
from src.verify import compare, format_table
from src.viz import scenario

PERFORMANCE = "data/performance/df1_m1.toml"
OUTPUT = Path("output/df1")
CRUISE = 27.70                       # m/s: the report's cruise speed, 90.88 ft/s
LOAD_FACTOR_LIMITS = (2.0, 2.5, 3.0, 3.5, 4.0, 4.5)
# m/s, turn-entry speeds tried
ENTRY_SPEEDS = (28.0, 26.0, 24.0, 23.0, 22.0, 21.0, 20.0, 19.0, 18.0, 17.0, 16.0, 15.0, 14.0)
# quicker roll-in and roll-out than the default pilot, for flying at the limits
LIMIT_GAINS = Gains(heading=2.5, bank_rate=math.radians(180.0))


def setup(thrust_factor=None):
    aircraft, condition = load_aircraft(AIRCRAFT)
    performance = load_performance(PERFORMANCE, condition.rho)
    if thrust_factor is not None:
        performance = replace(performance, powertrain=replace(performance.powertrain,
                                                              thrust_factor=thrust_factor))
    table = performance.powertrain.tabulate()
    envelope = Envelope(aircraft.m, aircraft.S, condition.rho, condition.g, performance, table)
    return aircraft, condition, performance, table, envelope


def envelope_metrics(performance, envelope):
    static = performance.powertrain.operating_point(1.0, 0.0)
    return {
        "static_thrust_N": static.thrust,
        "static_current_A": static.current,
        "static_electrical_power_W": static.electrical_power,
        "stall_speed_m_s": envelope.stall_speed,
        "level_drag_at_cruise_N": envelope.drag(CRUISE),
        "top_speed_m_s": envelope.top_speed,
        "endurance_at_cruise_min": envelope.endurance(CRUISE) / 60.0,
    }


def fly_limit(turn_load_factor, laps=2, dt=0.01, thrust_factor=None, structural_limit=None,
              turn_entry_speed=None):
    """Full throttle; turns aim for turn_load_factor within the aircraft's limits.

    turn_entry_speed: if given, the throttle is closed before each turn to arrive at this
    speed, which is then held through the turn.
    """
    aircraft, condition, performance, table, _ = setup(thrust_factor)
    if structural_limit is not None:
        performance = replace(performance, load_factor_limit=structural_limit)
    course = replace(load_course(COURSE), speed=math.inf, load_factor_limit=turn_load_factor,
                     laps=laps)
    model = NonlinearAircraft(aircraft, condition, deflection_max=course.deflection_max,
                              performance=performance)
    autopilot = CourseAutopilot(course, LIMIT_GAINS, performance, table,
                                turn_entry_speed=turn_entry_speed)
    trajectory = simulate_nonlinear(model, 600.0, dt, controller=autopilot,
                                    x0=model.trim_state(altitude=course.altitude),
                                    stop=lambda: autopilot.finished)
    return course, performance, autopilot, trajectory


def lap_metrics(performance, autopilot, tr):
    """Figures for the last lap flown (the first starts from trim, not from a turn)."""
    start, end = ([0.0] + autopilot.lap_times)[-2:]
    lap = (tr.t >= start) & (tr.t <= end)
    amp_hours = float(np.trapezoid(tr["current"][lap], tr.t[lap]) / 3600.0)
    usable = performance.powertrain.battery.usable_ah
    distance = float(np.trapezoid(tr["ground_speed"][lap], tr.t[lap]))
    turning = lap & (np.abs(tr["phi"]) > math.radians(30.0))
    return {
        "lap_time": end - start,
        "distance": distance,
        "time_turning": float(turning.sum() * (tr.t[1] - tr.t[0])),
        "speed_in_turns": float(tr["V"][turning].mean()),
        "throttle_closed_time": float(((tr["throttle"] < 0.02) & lap).sum()
                                      * (tr.t[1] - tr.t[0])),
        "speed_min": float(tr["V"][lap].min()), "speed_max": float(tr["V"][lap].max()),
        "load_factor_max": float(tr["load_factor"][lap].max()),
        "bank_max_deg": math.degrees(np.abs(tr["phi"][lap]).max()),
        "alpha_max_deg": math.degrees(tr["alpha"][lap].max()),
        "CL_max": float(tr["CL"][lap].max()),
        "elevator_max_deg": math.degrees(np.abs(tr["delta_m"][lap]).max()),
        "altitude_min": float(tr["altitude"][lap].min()),
        "current_mean": float(tr["current"][lap].mean()),
        "amp_hours": amp_hours, "laps_on_battery": usable / amp_hours,
        "outside_envelope": float(1.0 - tr["valid"][lap].mean()),
    }


def fly_at_limit(limit, turn_entry_speed=None, **kwargs):
    """A lap with the structural limit set to `limit` and turns flown up to it."""
    _, performance, autopilot, tr = fly_limit(limit, structural_limit=limit,
                                              turn_entry_speed=turn_entry_speed, **kwargs)
    return lap_metrics(performance, autopilot, tr)


def best_entry_speed(limit):
    """(entry speed, metrics) of the fastest lap among ENTRY_SPEEDS."""
    return min(((speed, fly_at_limit(limit, speed)) for speed in ENTRY_SPEEDS),
               key=lambda pair: pair[1]["lap_time"])


def sweep_table(rows):
    """rows: (limit, full-throttle metrics, best entry speed, its metrics, corner speed)."""
    lines = [f"{'':>6} |{'full throttle everywhere':^36}|{'slowing for the turns':^53}",
             f"{'limit':>6} | {'lap (s)':>7} {'slowest':>8} {'peak n':>7} {'Ah/lap':>7}  | "
             f"{'entry':>6} {'corner':>7} {'lap (s)':>7} {'saved':>6} {'peak n':>7} {'Ah/lap':>7}",
             "-" * 98]
    for limit, full, entry, managed, corner in rows:
        lines.append(
            f"{limit:>5.1f}g | {full['lap_time']:>7.2f} {full['speed_min']:>7.1f}  "
            f"{full['load_factor_max']:>7.2f} {full['amp_hours']:>7.3f}  | {entry:>5.0f}  "
            f"{corner:>6.1f}  {managed['lap_time']:>7.2f} "
            f"{full['lap_time'] - managed['lap_time']:>6.2f} "
            f"{managed['load_factor_max']:>7.2f} {managed['amp_hours']:>7.3f}")
    return "\n".join(lines)


def entry_speed_table(limit):
    lines = [f"{'entry (m/s)':>11} {'lap (s)':>8} {'path (m)':>9} {'time turning':>13} "
             f"{'speed in turns':>15} {'throttle closed':>16} {'Ah/lap':>7} {'lowest alt':>11}",
             "-" * 97]
    for speed in (None,) + ENTRY_SPEEDS:
        m = fly_at_limit(limit, speed)
        lines.append(
            f"{'none' if speed is None else format(speed, '.0f'):>11} {m['lap_time']:>8.2f} "
            f"{m['distance']:>9.0f} {m['time_turning']:>11.1f} s {m['speed_in_turns']:>11.1f} m/s "
            f"{m['throttle_closed_time']:>14.1f} s {m['amp_hours']:>7.3f} {m['altitude_min']:>9.1f} m")
    return "\n".join(lines)


def envelope_figure(envelope, performance):
    OUTPUT.mkdir(parents=True, exist_ok=True)
    speeds = np.linspace(envelope.stall_speed, 34.0, 200)
    sustained = np.array([envelope.sustained_load_factor(V) for V in speeds])
    instant = np.array([envelope.instantaneous_load_factor(V) for V in speeds])
    fig, axes = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
    axes[0].plot(speeds, [envelope.max_thrust(V) for V in speeds], label="thrust, full throttle")
    for n, style in ((1.0, "-"), (2.0, "--"), (4.5, ":")):
        axes[0].plot(speeds, [envelope.drag(V, n) for V in speeds], style, color="C3",
                     label=f"drag at load factor {n:g}")
    axes[0].set_ylabel("force (N)")
    axes[0].set_ylim(0, 60)
    axes[1].plot(speeds, instant, label="instantaneous (lift and structure)")
    axes[1].plot(speeds, sustained, label="sustained (thrust equals drag)")
    axes[1].set_ylabel("load factor")
    rate = lambda n: np.degrees([envelope.turn_rate(V, x) for V, x in zip(speeds, n)])  # noqa: E731
    axes[2].plot(speeds, rate(instant), label="instantaneous")
    axes[2].plot(speeds, rate(sustained), label="sustained")
    axes[2].set_ylabel("turn rate (deg/s)")
    axes[2].set_xlabel("airspeed (m/s)")
    for ax in axes:
        ax.axvline(envelope.top_speed, color="0.5", ls="--", lw=0.8)
        ax.axvline(envelope.corner_speed, color="0.5", ls=":", lw=0.8)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    axes[0].set_title(f"{performance.name}: performance envelope "
                      "(dotted: corner speed, dashed: top speed)")
    fig.tight_layout()
    fig.savefig(OUTPUT / "performance_envelope.png", dpi=130)
    plt.close(fig)


def sweep_figure(rows):
    limits = [r[0] for r in rows]
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.6))
    for index, label in ((1, "full throttle everywhere"), (3, "slowing for the turns")):
        axes[0].plot(limits, [r[index]["lap_time"] for r in rows], "o-", label=label)
        axes[1].plot(limits, [r[index]["speed_min"] for r in rows], "o-", label=label)
        axes[2].plot(limits, [r[index]["amp_hours"] for r in rows], "o-", label=label)
    axes[0].set_ylabel("lap time (s)")
    axes[1].set_ylabel("slowest speed in the lap (m/s)")
    axes[2].set_ylabel("battery used per lap (Ah)")
    for ax in axes:
        ax.set_xlabel("structural load-factor limit")
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    fig.suptitle("DF1 Mission 1: effect of the load-factor limit and of slowing for the turns")
    fig.tight_layout()
    fig.savefig(OUTPUT / "limit_lap_sweep.png", dpi=130)
    plt.close(fig)


def limit_scenario(laps=3, turn_entry_speed=None):
    """A flight at the structural limit as a viewer scenario: full throttle everywhere, or
    slowing to turn_entry_speed before each turn."""
    _, _, performance, _, _ = setup()
    n = performance.load_factor_limit
    course, performance, autopilot, tr = fly_limit(n, laps=laps,
                                                   turn_entry_speed=turn_entry_speed)
    summary = lap_summary(course, autopilot, tr)
    m = lap_metrics(performance, autopilot, tr)
    lines = [
        f"Laps: {laps}, total {summary['total_time']:.1f} s "
        f"({', '.join(f'{lap:.1f}' for lap in summary['laps'])} s)",
        (f"Throttle closed before each turn to enter at {turn_entry_speed:g} m/s; "
         if turn_entry_speed else "Full throttle; ")
        + f"speed {m['speed_min']:.1f}-{m['speed_max']:.1f} m/s",
        f"Peak load factor {m['load_factor_max']:.2f} (structural limit {n:g}), bank "
        f"{m['bank_max_deg']:.0f}°",
        f"Peak angle of attack {m['alpha_max_deg']:.1f}°, peak lift coefficient "
        f"{m['CL_max']:.2f} of {performance.CL_max:g}",
        f"Battery {m['amp_hours']:.2f} Ah per lap; {m['laps_on_battery']:.1f} laps on a charge",
        f"Time outside the aerodynamic model's valid range: {summary['outside_envelope']:.1%}",
    ]
    events = [(time, PHASE_LABELS[phase]) for time, phase in autopilot.events]
    events += [(time, f"Lap {i + 1} complete") for i, time in enumerate(autopilot.lap_times)]
    ground = {"half_length": course.half_length, "downwind_east": summary["downwind_offset_m"],
              "altitude": course.altitude}
    if turn_entry_speed:
        identifier, title = "df1-m1-fastest", (
            f"DF1 Mission 1: fastest lap found (slow to {turn_entry_speed:g} m/s for turns)")
        description = (
            f"Three laps with the throttle closed before each turn so the aircraft enters "
            f"at {turn_entry_speed:g} m/s, turns at the {n:g} g structural limit, then full "
            "throttle. Slightly faster than full throttle everywhere and much lighter on "
            "the battery.")
    else:
        identifier, title = "df1-m1-limit", (
            f"DF1 Mission 1: full throttle, {n:g} g turns")
        description = (
            f"Three laps at full throttle, with turns flown at the {n:g} g structural limit. "
            "Speed, turn radius and lap time are outputs of the propulsion, drag and "
            "structural models.")
    return scenario(
        tr, identifier, title, "Course", description,
        span=GEOMETRY_SPAN, summary=lines, events=sorted(events), ground=ground)


def main():
    _, _, performance, _, envelope = setup()
    print(f"{performance.name}\n")
    print("Models against the static test and the design report "
          f"(thrust calibration factor {performance.powertrain.thrust_factor:g}):")
    print(format_table(compare(envelope_metrics(performance, envelope), PERFORMANCE)))
    _, _, raw, _, raw_envelope = setup(thrust_factor=1.0)
    print(f"\nWithout the thrust calibration: static thrust "
          f"{raw.powertrain.operating_point(1.0, 0.0).thrust:.1f} N, top speed "
          f"{raw_envelope.top_speed:.1f} m/s")

    print(f"\nEnvelope: stall {envelope.stall_speed:.1f} m/s, corner speed "
          f"{envelope.corner_speed:.1f} m/s, top speed {envelope.top_speed:.1f} m/s")
    best = max(np.linspace(envelope.stall_speed, envelope.top_speed, 300),
               key=lambda V: envelope.turn_rate(V, envelope.sustained_load_factor(V)))
    n_best = envelope.sustained_load_factor(best)
    print(f"Best sustained turn: {math.degrees(envelope.turn_rate(best, n_best)):.0f} deg/s at "
          f"{best:.1f} m/s (load factor {n_best:.2f}, radius "
          f"{envelope.turn_radius(best, n_best):.0f} m)")
    n_top = envelope.sustained_load_factor(0.95 * envelope.top_speed)
    print(f"Sustained load factor at 95% of top speed: {n_top:.2f}")

    print("\nLaps by structural load-factor limit (second lap of two):")
    rows = []
    for limit in LOAD_FACTOR_LIMITS:
        entry, managed = best_entry_speed(limit)
        # speed at which the lift the autopilot allows itself just reaches its turn target
        corner = envelope.stall_speed * math.sqrt(0.93 * limit / 0.85)
        rows.append((limit, fly_at_limit(limit), entry, managed, corner))
    print(sweep_table(rows))

    limit = performance.load_factor_limit
    print(f"\nTurn-entry speed at the {limit:g} g limit:")
    print(entry_speed_table(limit))

    print(f"\nSensitivity of the full-throttle lap at {limit:g} g:")
    for label, kwargs in (("thrust calibration removed", {"thrust_factor": 1.0}),
                          ("step halved (0.005 s)", {"dt": 0.005})):
        print(f"  {label:28} lap {fly_at_limit(limit, **kwargs)['lap_time']:.2f} s")

    envelope_figure(envelope, performance)
    sweep_figure(rows)
    print(f"\nFigures written to {OUTPUT}/")


if __name__ == "__main__":
    main()
