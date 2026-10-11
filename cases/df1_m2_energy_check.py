"""Check the model against a reported Mission 2 flight: python3 -m cases.df1_m2_energy_check

Ishan reported (2026-10-11, from memory, no log) that a Mission 2 flight of eight laps
left a 4500 mAh pack at about 2% and hot. This flies the whole mission in the model
(takeoff roll, eight laps, landing circuit, with wind) and compares the battery used, and
checks the propulsion model against all four static tests in the report.
See docs/flight-validation.md for what can and cannot be concluded.
"""

import math
from dataclasses import replace

import numpy as np

from cases.df1_performance import whole_mission
from src.aircraft import load_aircraft
from src.performance import Envelope, load_performance
from src.propulsion import Battery, Motor, Powertrain, load_propeller

AIRCRAFT = "data/aircraft/df1_m2_vlm_distributed.toml"
PERFORMANCE = "data/performance/df1_m2.toml"
MASS = 7.228 * 0.45359237      # kg: the report's Mission 2 weight. The XFLR5 model (and so the
#                                aircraft file) is 0.5 kg heavier; see the mass-distribution record
LAPS = 8
WINDOW = 300.0                 # s: the five-minute flight window
PACK_AH = 4.5
REPORTED_AH = 0.98 * PACK_AH   # "down to about 2%"
WIND = 5.0                     # m/s down the runway: assumed, the wind that day is not known
# report Table 4.4.5 (p.27), Mission 2 takeoff: 39.94 ft/s after 24.78 ft
REPORT_TAKEOFF_SPEED, REPORT_TAKEOFF_DISTANCE = 39.94 * 0.3048, 24.78 * 0.3048
LBF = 4.4482216
# report Table 8.1.2: propeller, pack capacity (Ah), C rating, thrust (lbf), current (A), power (W)
STATIC_TESTS = (("15x6E", 4.2, 35.0, 10.34, 60.39, 1358.0), ("15x6E", 4.0, 30.0, 9.800, 52.14, 1075.0),
                ("14x7E", 4.2, 35.0, 8.969, 52.71, 1162.0), ("14x7E", 4.0, 30.0, 8.426, 44.74, 918.5))


def static_checks():
    """(label, measured/modelled thrust, modelled/measured current) for each static test."""
    rows = []
    for name, capacity, c_rating, thrust_lbf, current, _ in STATIC_TESTS:
        powertrain = Powertrain(Motor(520.0, 1200.0), Battery(6, capacity, c_rating),
                                load_propeller(f"data/propulsion/apc_{name}.json"), 1.225, 100.0)
        point = powertrain.operating_point(1.0, 0.0)
        rows.append((f"{name[:-1]} with {capacity:g} Ah", thrust_lbf * LBF / point.thrust,
                     point.current / current))
    return rows


def setup():
    aircraft, condition = load_aircraft(AIRCRAFT)
    aircraft = replace(aircraft, m=MASS)
    performance = load_performance(PERFORMANCE, condition.rho)
    return aircraft, condition, performance


def fly(limit, wind=WIND, turn_entry_speed=None, dt=0.01, laps=LAPS, drag_factor=1.0,
        thrust_factor=None):
    """The whole flight at full throttle with the structural limit set to `limit`: takeoff
    roll into a headwind of `wind`, the laps, then a landing circuit.

    drag_factor scales the zero-lift drag; thrust_factor replaces the thrust calibration.
    Both are for asking what the aircraft would have to be like to match the flight.
    """
    aircraft, condition, performance = setup()
    performance = replace(performance, CD0=performance.CD0 * drag_factor)
    if thrust_factor is not None:
        performance = replace(performance, powertrain=replace(performance.powertrain,
                                                              thrust_factor=thrust_factor))
    _, result, autopilot = whole_mission(aircraft, condition, performance, limit, wind, laps,
                                         turn_entry_speed, dt)
    tr = result.trajectory
    used = np.concatenate([[0.0], np.cumsum(0.5 * (tr["current"][1:] + tr["current"][:-1])
                                            * np.diff(tr.t))]) / 3600.0
    last_lap = autopilot.lap_times[-1]
    airborne = (tr.t > result.liftoff_time) & (tr.t < last_lap)
    return {
        "result": result, "autopilot": autopilot,
        "liftoff_distance": result.liftoff_distance, "liftoff_speed": result.liftoff_speed,
        "laps_time": last_lap,
        "lap": last_lap - (autopilot.lap_times[-2] if len(autopilot.lap_times) > 1 else 0.0),
        "amp_hours_laps": float(np.interp(last_lap, tr.t, used)),
        "amp_hours_total": float(used[-1]),
        "mean_current": float(tr["current"][airborne].mean()),
        "speed_max": float(tr["V"].max()),
        "touchdown_north": result.touchdown_north, "stop_north": result.stop_north,
        "flight_time": float(tr.t[-1]),
        "outside_envelope": float(1.0 - tr["valid"].mean()),
    }


def main():
    print("Propulsion model against the report's static tests (no calibration applied):")
    print(f"{'':20}{'measured / modelled thrust':>28}{'modelled / measured current':>30}")
    for label, thrust_ratio, current_ratio in static_checks():
        print(f"{label:20}{thrust_ratio:>28.2f}{current_ratio:>30.2f}")

    aircraft, condition, performance = setup()
    envelope = Envelope(aircraft.m, aircraft.S, condition.rho, condition.g, performance,
                        performance.powertrain.tabulate())
    usable = performance.powertrain.battery.usable_ah
    print(f"\n{performance.name}: {aircraft.m:.3f} kg, top speed {envelope.top_speed:.1f} m/s, "
          f"pack {performance.powertrain.battery.capacity_ah:g} Ah ({usable:.2f} Ah at 85%)")
    base = fly(3.0)
    print(f"\nTakeoff roll (headwind {WIND:g} m/s): liftoff at {base['liftoff_speed']:.2f} m/s "
          f"after {base['liftoff_distance']:.1f} m.")
    calm = fly(3.0, wind=0.0, laps=2)
    print(f"In still air: {calm['liftoff_distance']:.2f} m; the report's Table 4.4.5 gives "
          f"{REPORT_TAKEOFF_DISTANCE:.2f} m at {REPORT_TAKEOFF_SPEED:.2f} m/s.")

    print(f"\nWhole flight: takeoff roll, {LAPS} laps at full throttle, landing circuit.")
    print(f"{'limit':>6} {'wind':>6} {'strategy':>16} {'8 laps done':>12} {'lap':>7} "
          f"{'used by then':>13} {'after landing':>14} {'of reported':>12} {'mean current':>13}  in window")
    print("-" * 118)

    def line(limit, wind, label, entry):
        f = fly(limit, wind, entry)
        print(f"{limit:>5.1f}g {wind:>4.0f}   {label:>16} {f['laps_time']:>10.0f} s {f['lap']:>5.1f} s "
              f"{f['amp_hours_laps']:>10.2f} Ah {f['amp_hours_total']:>11.2f} Ah "
              f"{f['amp_hours_total'] / REPORTED_AH:>11.0%} {f['mean_current']:>11.1f} A  "
              f"{'yes' if f['laps_time'] <= WINDOW else 'no'}")

    for limit in (2.5, 3.0, 3.5, 4.5):
        line(limit, WIND, "full throttle", None)
    for wind in (0.0, 8.0):
        line(3.0, wind, "full throttle", None)
    corner = envelope.stall_speed * math.sqrt(0.93 * 3.0 / 0.85)
    line(3.0, WIND, f"slow to {corner:.0f} m/s", round(corner))

    print(f"\nReported: {LAPS} laps, pack at about 2%: about {REPORTED_AH:.1f} Ah of {PACK_AH:g} Ah. "
          f"Over a {WINDOW:.0f} s flight that is a mean current of about "
          f"{REPORTED_AH * 3600 / WINDOW:.0f} A.")

    powertrain = performance.powertrain
    print("\nFull-throttle current against airspeed: "
          + ", ".join(f"{powertrain.operating_point(1.0, V).current:.0f} A at {V} m/s"
                      for V in (20, 25, 30)))
    print("\nWhat would close the gap (3.0 g, wind as above):")
    print(f"{'':36} {'top speed seen':>15} {'8 laps done':>12} {'after landing':>14} {'of reported':>12}")
    for label, kwargs in (("as modelled", {}),
                          ("zero-lift drag +50%", {"drag_factor": 1.5}),
                          ("zero-lift drag doubled", {"drag_factor": 2.0}),
                          ("thrust factor 0.65", {"thrust_factor": 0.65}),
                          ("drag +50% and thrust factor 0.65",
                           {"drag_factor": 1.5, "thrust_factor": 0.65})):
        f = fly(3.0, **kwargs)
        print(f"{label:36} {f['speed_max']:>11.1f} m/s {f['laps_time']:>10.0f} s "
              f"{f['amp_hours_total']:>11.2f} Ah {f['amp_hours_total'] / REPORTED_AH:>11.0%}")
    print(f"Landing in the base case: touchdown {base['touchdown_north']:.0f} m from the start "
          f"line, stopped at {base['stop_north']:.0f} m; whole flight {base['flight_time']:.0f} s.")


if __name__ == "__main__":
    main()
