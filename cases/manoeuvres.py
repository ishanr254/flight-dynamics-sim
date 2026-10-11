"""Manoeuvres, natural modes and the second aircraft: python3 -m cases.manoeuvres

Flies DF1 Mission 1 through a set of manoeuvres with the nonlinear model, releases it from
small disturbances to show its natural modes, and runs the Case Study 7 transport's
lateral responses from the linear model. Prints what each one reaches and how much of it
is flagged. The viewer scenarios built here are written by cases.viewer_scenarios.
"""

import math
from math import degrees, radians

import numpy as np

from cases.df1_course import GEOMETRY_SPAN
from cases.df1_performance import setup
from src.aircraft import load_aircraft
from src.dynamics import lateral_state_space
from src.simulate import (Input, ManoeuvrePilot, NonlinearAircraft, ScriptedControls, Target,
                          doublet, simulate, simulate_nonlinear)
from src.simulate.nonlinear import quaternion_from_euler
from src.viz import lateral_scenario, scenario

START_HEIGHT = 80.0      # m
STEP = 0.005             # s
LOOP_LOAD_FACTOR = 3.8
CS7 = "data/aircraft/cs7_transport.toml"
CS7_SPAN = 44.0          # m: not in the case study; a typical span for a 240 m^2 transport

ATTITUDE = [("phi", "bank (deg)", "deg"), ("theta", "pitch (deg)", "deg")]


def df1_model():
    aircraft, condition, performance, table, envelope = setup()
    model = NonlinearAircraft(aircraft, condition, performance=performance,
                              deflection_max=radians(30.0))
    return model, table, envelope


def summarise(tr):
    """Lines describing what a flight reached and how much of it is flagged."""
    lines = [
        f"Speed {tr['V'].min():.1f}-{tr['V'].max():.1f} m/s, height "
        f"{tr['altitude'].min():.0f}-{tr['altitude'].max():.0f} m",
        f"Load factor {tr['load_factor'].min():+.2f} to {tr['load_factor'].max():+.2f}",
        f"Angle of attack {degrees(tr['alpha'].min()):+.1f}° to {degrees(tr['alpha'].max()):+.1f}°, "
        f"sideslip up to {degrees(np.abs(tr['beta']).max()):.1f}°",
        f"Peak roll rate {degrees(np.abs(tr['p']).max()):.0f}°/s, peak pitch rate "
        f"{degrees(np.abs(tr['q']).max()):.0f}°/s",
        f"Outside the model's valid range: {1.0 - tr['valid'].mean():.0%} of the time",
        f"Over the structural limit: {tr['overload'].mean():.0%} of the time",
    ]
    return lines


def loop_duration(model, table):
    """Time at which the pull from level flight has taken the nose through 360 deg."""
    pilot = ManoeuvrePilot([Target(0.0), Target(0.5, None, LOOP_LOAD_FACTOR)], table)
    tr = simulate_nonlinear(model, 10.0, STEP, pilot, model.trim_state(altitude=START_HEIGHT))
    travel = np.cumsum(tr["q"]) * STEP
    return float(tr.t[np.argmax(travel >= 2.0 * math.pi)])


def df1_manoeuvres():
    """(identifier, title, description, trajectory, traces, playback) for each manoeuvre."""
    model, table, _ = df1_model()
    x0 = model.trim_state(altitude=START_HEIGHT)

    def scripted(inputs, duration):
        return simulate_nonlinear(model, duration, STEP, ScriptedControls(inputs, table), x0)

    limit = model.load_factor_limit

    def piloted(targets, duration):
        pilot = ManoeuvrePilot(targets, table, load_factor_limit=0.95 * limit)
        return simulate_nonlinear(model, duration, STEP, pilot, x0)

    t_loop = loop_duration(model, table)
    longitudinal = [("load_factor", "load factor", ""), ("alpha", "angle of attack (deg)", "deg"),
                    ("theta", "pitch (deg)", "deg")]
    return [
        ("elevator-stop", "Elevator to the stop",
         "Full up elevator (30°) held for one second from cruise, then released. The load "
         "factor goes far past the 4.5 g structural limit and the wing reaches its lift limit: "
         "both warnings show. Nothing after the first half second should be believed.",
         scripted([Input("delta_m", 0.5, 1.5, radians(-30.0))], 4.0), longitudinal, 0.5),
        ("aileron-roll", "Aileron roll",
         "Ten degrees of aileron held for 1.4 s from cruise: about one full roll at nearly "
         "300°/s. No elevator or rudder is used, so the nose drops and the aircraft loses "
         "height.",
         scripted([Input("delta_l", 0.5, 1.9, radians(-10.0))], 4.0),
         [("p", "roll rate (deg/s)", "deg"), ("phi", "bank (deg)", "deg"),
          ("beta", "sideslip (deg)", "deg")], 0.5),
        ("aileron-full", "Full aileron",
         "Full aileron (30°) for half a second. The model gives about 800°/s. That is past "
         "the roll rate at which its aerodynamics hold (the wing tips would stall), so the "
         "valid-range warning shows.",
         scripted([Input("delta_l", 0.5, 1.0, radians(-30.0))], 3.0),
         [("p", "roll rate (deg/s)", "deg"), ("phi", "bank (deg)", "deg"),
          ("beta", "sideslip (deg)", "deg")], 0.25),
        ("rudder-doublet", "Rudder doublet",
         "Six degrees of rudder one way then the other, half a second each, then hands off. "
         "The Dutch roll rings down over a few seconds.",
         scripted(doublet("delta_n", 0.5, 0.5, radians(6.0)), 6.0),
         [("beta", "sideslip (deg)", "deg"), ("r", "yaw rate (deg/s)", "deg"),
          ("phi", "bank (deg)", "deg")], 0.5),
        ("turn-reversal", "Hard turn reversal",
         "Full throttle, holding height: roll to 75° of bank one way (about 4 g), hold three "
         "seconds, reverse to 75° the other way, hold, and roll level.",
         piloted([Target(0.0), Target(0.5, radians(75.0)), Target(3.5, radians(-75.0)),
                  Target(6.5)], 9.0),
         [("phi", "bank (deg)", "deg"), ("load_factor", "load factor", ""),
          ("V", "speed (m/s)", "")], 1.0),
        ("loop", "Loop",
         f"Full throttle, a {LOOP_LOAD_FACTOR:g} g pull from level cruise held all the way "
         "round, then level off. Speed falls to under 20 m/s over the top, where the wing "
         "cannot give the load asked for: it reaches its lift limit and the valid-range "
         "warning shows.",
         piloted([Target(0.0), Target(0.5, None, LOOP_LOAD_FACTOR), Target(t_loop, 0.0)],
                 t_loop + 3.0),
         [("load_factor", "load factor", ""), ("V", "speed (m/s)", ""),
          ("alpha", "angle of attack (deg)", "deg")], 1.0),
        ("stall-approach", "Approach to the stall",
         "Throttle closed, holding height with the elevator. Speed decays until the wing "
         "reaches its lift limit at about 9.7 m/s and the aircraft starts to sink. The model "
         "has no real stall: lift simply stops rising, so what follows the warning is not a "
         "prediction.",
         piloted([Target(0.0, 0.0, None, 0.0)], 16.0),
         [("V", "speed (m/s)", ""), ("alpha", "angle of attack (deg)", "deg"),
          ("altitude", "height (m)", "")], 2.0),
    ]


def df1_modes():
    """Releases from a disturbance with the controls fixed at trim."""
    model, _, _ = df1_model()

    def released(duration, alpha=0.0, beta=0.0, bank=0.0, extra_speed=0.0):
        V = model.V0 + extra_speed
        x = model.trim_state(altitude=START_HEIGHT)
        x[3] = V * math.cos(alpha) * math.cos(beta)
        x[4] = V * math.sin(beta)
        x[5] = V * math.sin(alpha) * math.cos(beta)
        x[6:10] = quaternion_from_euler(bank, alpha, 0.0)
        return simulate_nonlinear(model, duration, STEP, x0=x)

    def aileron_pulse():
        table = setup()[3]
        return simulate_nonlinear(model, 2.0, STEP,
                                  ScriptedControls([Input("delta_l", 0.2, 0.5, radians(-5.0))],
                                                   table),
                                  model.trim_state(altitude=START_HEIGHT))

    return [
        ("mode-short-period", "Short period",
         "Released from 5° above the trim angle of attack. The nose settles in a fraction of "
         "a second with almost no overshoot. Shown in slow motion.",
         released(2.0, alpha=radians(5.0)),
         [("alpha", "angle of attack (deg)", "deg"), ("q", "pitch rate (deg/s)", "deg")], 0.25),
        ("mode-phugoid", "Phugoid",
         "Released 4 m/s faster than trim. Speed and height trade back and forth with a "
         "period of about 15 s while the angle of attack barely changes.",
         released(60.0, extra_speed=4.0),
         [("V", "speed (m/s)", ""), ("altitude", "height (m)", ""),
          ("theta", "pitch (deg)", "deg")], 4.0),
        ("mode-roll", "Roll mode",
         "A 5° aileron pulse for 0.3 s. Roll rate builds and dies away within a few "
         "hundredths of a second of the control; the bank angle reached then stays. Shown "
         "in slow motion.",
         aileron_pulse(),
         [("p", "roll rate (deg/s)", "deg"), ("phi", "bank (deg)", "deg")], 0.25),
        ("mode-dutch-roll", "Dutch roll",
         "Released from 8° of sideslip. The nose swings and the wings rock together, dying "
         "away over about four seconds. Shown at half speed.",
         released(6.0, beta=radians(8.0)),
         [("beta", "sideslip (deg)", "deg"), ("r", "yaw rate (deg/s)", "deg"),
          ("phi", "bank (deg)", "deg")], 0.5),
        ("mode-spiral", "Spiral",
         "Released from 10° of bank with the controls fixed. The bank slowly steepens "
         "(doubling in about 12 s) and the aircraft descends in a tightening turn. This mode "
         "is unstable, slowly enough for a pilot to correct without noticing.",
         released(25.0, bank=radians(10.0)),
         [("phi", "bank (deg)", "deg"), ("altitude", "height (m)", ""),
          ("V", "speed (m/s)", "")], 2.0),
    ]


def cs7_cases():
    """(identifier, title, description, Response, traces, playback, controls) for the transport."""
    aircraft, condition = load_aircraft(CS7)
    ss = lateral_state_space(aircraft, condition)
    lateral = [("beta", "sideslip (deg)", "deg"), ("phi", "bank (deg)", "deg"),
               ("r", "yaw rate (deg/s)", "deg")]
    rudder = simulate(ss, 20.0, u={"delta_n": radians(-2.0)}, n_samples=1201)
    return condition, [
        ("cs7-dutch-roll", "Transport: Dutch roll",
         "The Case Study 7 transport released from 4° of sideslip at 125 m/s. Period about "
         "5.7 s, damping ratio 0.25: slower and looser than DF1's.",
         simulate(ss, 30.0, x0={"beta": radians(4.0)}, n_samples=1801), lateral, 2.0, {}),
        ("cs7-spiral", "Transport: spiral",
         "Released from 30° of bank. This aircraft's spiral is stable: the bank decays with "
         "a time constant of about 129 s. Shown eight times faster than life.",
         simulate(ss, 200.0, x0={"phi": radians(30.0)}, n_samples=6001), lateral, 8.0, {}),
        ("cs7-rudder", "Transport: rudder step",
         "Two degrees of rudder applied and held for 20 s. Sideslip and yaw rate oscillate "
         "in the Dutch roll while the bank builds.",
         rudder, lateral, 2.0, {"delta_n": np.full(len(rudder.t), radians(-2.0))}),
    ]


def manoeuvre_scenarios():
    out = []
    for group, cases in (("Manoeuvres", df1_manoeuvres()), ("Natural modes (DF1)", df1_modes())):
        for identifier, title, description, tr, traces, playback in cases:
            out.append(scenario(tr, f"df1-{identifier}", f"DF1: {title}", group, description,
                                span=GEOMETRY_SPAN, summary=summarise(tr), traces=traces,
                                playback=playback, exaggeration=4,
                                rate=15.0 if tr.t[-1] > 20.0 else 30.0))
    condition, cases = cs7_cases()
    for identifier, title, description, response, traces, playback, controls in cases:
        lines = [f"Linear lateral model, {condition.V:g} m/s; path rebuilt at constant speed "
                 "and height",
                 f"Sideslip up to {degrees(np.abs(response['beta']).max()):.1f}°, bank up to "
                 f"{degrees(np.abs(response['phi']).max()):.1f}°",
                 f"Aircraft drawn with an assumed {CS7_SPAN:g} m span"]
        out.append(lateral_scenario(response, condition.V, identifier, title,
                                    "Second aircraft (Case Study 7 transport)", description,
                                    controls=controls, span=CS7_SPAN, summary=lines,
                                    traces=traces, playback=playback, exaggeration=1,
                                    rate=5.0 if response.t[-1] > 100.0 else 15.0))
    return out


def main():
    for group, cases in (("DF1 manoeuvres", df1_manoeuvres()), ("DF1 natural modes", df1_modes())):
        print(f"=== {group} ===")
        for _, title, _, tr, _, _ in cases:
            print(f"\n{title} ({tr.t[-1]:.0f} s)")
            print("  " + "\n  ".join(summarise(tr)))
        print()
    condition, cases = cs7_cases()
    print("=== Case Study 7 transport (linear lateral model) ===")
    for _, title, _, response, _, _, _ in cases:
        print(f"{title}: {response.t[-1]:.0f} s, sideslip up to "
              f"{degrees(np.abs(response['beta']).max()):.1f}°, bank up to "
              f"{degrees(np.abs(response['phi']).max()):.1f}°")


if __name__ == "__main__":
    main()
