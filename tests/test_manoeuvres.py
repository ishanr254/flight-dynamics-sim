"""Manoeuvre mode: scripted controls, the manoeuvre pilot, flags, modes and the second aircraft."""

import json
import math
from math import radians
from pathlib import Path

import numpy as np
import pytest

from cases.manoeuvres import (LOOP_LOAD_FACTOR, cs7_cases, df1_manoeuvres, df1_model, df1_modes,
                              manoeuvre_scenarios)
from src.aircraft import load_aircraft
from src.analysis import modes, name_lateral_modes
from src.dynamics import lateral_state_space
from src.simulate import (Input, ManoeuvrePilot, ScriptedControls, Target, doublet,
                          simulate_nonlinear)
from src.viz import lateral_scenario
from src.viz.export import GLOBAL


@pytest.fixture(scope="module")
def flown():
    return {case[0]: case[3] for case in df1_manoeuvres() + df1_modes()}


@pytest.fixture(scope="module")
def model_and_table():
    model, table, envelope = df1_model()
    return model, table, envelope


def invalid(tr):
    return 1.0 - tr["valid"].mean()


# ---- scripted controls

def test_inputs_add_to_trim_only_while_active(model_and_table):
    model, table, _ = model_and_table
    controls = ScriptedControls([Input("delta_m", 1.0, 2.0, radians(-5.0)),
                                 Input("delta_m", 1.5, 2.0, radians(-1.0))], table)
    out = model.outputs(model.trim_state(), tuple(model.trim_controls()))
    assert controls(0.5, out, model) == (model.thrust_trim, 0.0, 0.0, 0.0)
    assert controls(1.2, out, model)[1] == pytest.approx(radians(-5.0))
    assert controls(1.7, out, model)[1] == pytest.approx(radians(-6.0))
    assert controls(2.0, out, model)[1] == 0.0


def test_doublet_is_two_opposite_inputs():
    first, second = doublet("delta_n", 0.5, 0.4, 0.1)
    assert (first.start, first.end, first.value) == (0.5, 0.9, 0.1)
    assert (second.start, second.end, second.value) == (0.9, pytest.approx(1.3), -0.1)


def test_bad_inputs_are_rejected(model_and_table):
    with pytest.raises(ValueError, match="unknown control"):
        ScriptedControls([Input("flaps", 0.0, 1.0, 0.1)])
    with pytest.raises(ValueError, match="needs a thrust table"):
        ScriptedControls([Input("throttle", 0.0, 1.0, 0.5)])


def test_throttle_input_sets_thrust_from_the_table(model_and_table):
    model, table, _ = model_and_table
    controls = ScriptedControls([Input("throttle", 0.0, 1.0, 1.0)], table)
    out = model.outputs(model.trim_state(), tuple(model.trim_controls()))
    assert controls(0.5, out, model)[0] == pytest.approx(table.max_thrust(out["V"]))
    assert controls.telemetry["throttle"] == 1.0


# ---- flags

def test_elevator_to_the_stop_raises_both_flags(flown):
    tr = flown["elevator-stop"]
    assert tr["load_factor"].max() > 6.0
    assert tr["overload"].mean() > 0.05 and invalid(tr) > 0.1


def test_moderate_manoeuvres_raise_no_flags(flown):
    for name in ("aileron-roll", "rudder-doublet", "turn-reversal"):
        assert invalid(flown[name]) == 0.0 and flown[name]["overload"].mean() == 0.0, name


def test_full_aileron_is_flagged_for_roll_rate_alone(flown, model_and_table):
    model, _, _ = model_and_table
    tr = flown["aileron-full"]
    helix = np.abs(tr["p"]) * model.span / (2.0 * tr["V"])
    assert helix.max() > model.envelope.helix_max
    assert invalid(tr) > 0.05
    assert np.abs(tr["CL"]).max() < model.CL_max            # not the lift limit
    assert np.degrees(np.abs(tr["beta"]).max()) < 15.0      # not sideslip
    assert tr["overload"].mean() == 0.0


def test_span_comes_from_aspect_ratio_and_area(model_and_table):
    model, _, _ = model_and_table
    assert model.span == pytest.approx(1.182, abs=0.002)


# ---- manoeuvre pilot

def test_turn_reversal_holds_height_and_stays_within_the_structure(flown, model_and_table):
    tr = flown["turn-reversal"]
    assert np.degrees(tr["phi"].max()) == pytest.approx(75.0, abs=5.0)
    assert np.degrees(tr["phi"].min()) == pytest.approx(-75.0, abs=5.0)
    assert np.ptp(tr["altitude"]) < 3.0
    assert tr["load_factor"].max() < model_and_table[0].load_factor_limit
    assert abs(np.degrees(tr["phi"][-1])) < 5.0


def test_loop_goes_all_the_way_round_and_recovers(flown):
    tr = flown["loop"]
    pitch_travel = np.cumsum(tr["q"]) * (tr.t[1] - tr.t[0])
    assert pitch_travel.max() > 2.0 * math.pi
    assert tr["altitude"].max() - tr["altitude"][0] > 20.0
    assert abs(np.degrees(tr["theta"][-1])) < 15.0 and abs(np.degrees(tr["phi"][-1])) < 10.0
    assert tr["load_factor"].max() == pytest.approx(LOOP_LOAD_FACTOR, rel=0.2)
    assert tr["V"].min() > 12.0


def test_approach_to_the_stall_reaches_the_lift_limit_near_stall_speed(flown, model_and_table):
    _, _, envelope = model_and_table
    tr = flown["stall-approach"]
    first = int(np.argmax(tr["valid"] < 0.5))
    assert tr["valid"][0] == 1.0 and first > 0
    assert tr["V"][first] == pytest.approx(envelope.stall_speed, rel=0.12)
    assert tr["altitude"][-1] < tr["altitude"][0] - 5.0


def test_pilot_holds_a_steady_load_factor(model_and_table):
    model, table, _ = model_and_table
    pilot = ManoeuvrePilot([Target(0.0, None, 2.0)], table)
    tr = simulate_nonlinear(model, 1.5, 0.005, pilot, model.trim_state(altitude=80.0))
    assert tr["load_factor"][-60:].mean() == pytest.approx(2.0, rel=0.08)


# ---- natural modes from the nonlinear model

def test_dutch_roll_period_matches_the_linear_model(flown):
    tr = flown["mode-dutch-roll"]
    beta = tr["beta"]
    crossings = tr.t[1:][(beta[:-1] > 0) & (beta[1:] <= 0)]
    period = float(np.mean(np.diff(crossings)))
    linear = name_lateral_modes(modes(lateral_state_space(
        *load_aircraft("data/aircraft/df1_m1_asbuilt.toml")).A))["dutch_roll"].period
    assert period == pytest.approx(linear, rel=0.05)


def test_short_period_settles_and_spiral_diverges(flown):
    short = flown["mode-short-period"]
    assert abs(short["alpha"][-1]) < 0.1 * abs(short["alpha"][0])
    spiral = flown["mode-spiral"]
    assert abs(spiral["phi"][-1]) > 2.5 * abs(spiral["phi"][0])
    assert spiral["altitude"][-1] < spiral["altitude"][0]


def test_roll_mode_leaves_the_bank_where_it_got_to(flown):
    tr = flown["mode-roll"]
    after = tr.t > 1.0
    assert np.degrees(np.abs(tr["p"][after]).max()) < 0.1 * np.degrees(np.abs(tr["p"]).max())
    assert np.degrees(tr["phi"][-1]) > 20.0


def test_phugoid_period_is_about_fifteen_seconds(flown):
    tr = flown["mode-phugoid"]
    speed = tr["V"] - tr["V"].mean()
    crossings = tr.t[1:][(speed[:-1] > 0) & (speed[1:] <= 0)]
    assert float(np.mean(np.diff(crossings))) == pytest.approx(15.0, abs=2.5)


# ---- second aircraft and scenario contents

def test_lateral_scenario_rebuilds_heading_from_yaw_rate():
    condition, cases = cs7_cases()
    response = cases[0][3]
    s = lateral_scenario(response, condition.V, "x", "X", "G", "d", rate=15.0)
    yaw = np.trapezoid(response["r"], response.t)
    assert s["columns"]["psi"][-1] == pytest.approx(yaw, abs=0.01)
    assert set(s["columns"]["valid"]) == {1} and set(s["columns"]["overload"]) == {0}
    assert s["columns"]["V"][0] == condition.V
    assert s["columns"]["north"][-1] == pytest.approx(condition.V * response.t[-1], rel=0.01)


def test_scenarios_carry_what_the_viewer_needs():
    scenarios = manoeuvre_scenarios()
    assert [s["group"] for s in scenarios].count("Manoeuvres") == 7
    assert {s["group"] for s in scenarios} == {
        "Manoeuvres", "Natural modes (DF1)", "Second aircraft (Case Study 7 transport)"}
    for s in scenarios:
        assert 1 <= len(s["traces"]) <= 3 and s["playback"] > 0 and s["exaggeration"] >= 1
        assert all(trace[0] in s["columns"] for trace in s["traces"])
        assert len(s["summary"]) >= 3 and s["description"]


def test_shipped_viewer_data_has_all_four_groups_and_stays_small():
    path = Path("viz/data/scenarios.js")
    scenarios = json.loads(path.read_text().split(f"window.{GLOBAL} = ", 1)[1].rstrip().rstrip(";"))
    groups = []
    for s in scenarios:
        if s["group"] not in groups:
            groups.append(s["group"])
    assert groups == ["Course", "Manoeuvres", "Natural modes (DF1)",
                      "Second aircraft (Case Study 7 transport)"]
    assert path.stat().st_size < 3_000_000
