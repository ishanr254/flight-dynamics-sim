"""Performance envelope and limit laps for DF1 Mission 1.

Comparisons with the design report use illustrative tolerances (docs/requirements.md).
"""

import math

import numpy as np
import pytest

from cases.df1_performance import (CRUISE, PERFORMANCE, envelope_metrics, fly_limit,
                                   lap_metrics, setup)
from src.performance import PerformanceDataError, load_performance
from src.simulate import NonlinearAircraft, simulate_nonlinear
from src.verify import compare


@pytest.fixture(scope="module")
def data():
    return setup()


@pytest.fixture(scope="module")
def laps():
    out = {}
    for n in (2.0, 4.5):
        _, performance, autopilot, tr = fly_limit(n, dt=0.02)
        out[n] = (performance, autopilot, tr, lap_metrics(performance, autopilot, tr))
    return out


def test_performance_file_values(data):
    _, _, performance, _, _ = data
    assert performance.CD0 == 0.059 and performance.CL_max == 1.41
    assert performance.k == pytest.approx(1 / (math.pi * 3.85 * 0.75))
    assert performance.load_factor_limit == 4.5
    assert performance.powertrain.current_limit == 100.0
    assert performance.powertrain.thrust_factor == 0.83


def test_missing_field_is_reported(tmp_path):
    text = open(PERFORMANCE).read().replace("CL_max = 1.41", "")
    path = tmp_path / "bad.toml"
    path.write_text(text)
    with pytest.raises(PerformanceDataError, match="CL_max"):
        load_performance(path, 1.225)


def test_models_against_test_data_and_report(data):
    _, _, performance, _, envelope = data
    rows = {r.metric: r for r in compare(envelope_metrics(performance, envelope), PERFORMANCE)}
    assert abs(rows["static_thrust_N"].relative_difference) < 0.01          # calibrated to it
    assert abs(rows["level_drag_at_cruise_N"].relative_difference) < 0.05
    assert abs(rows["static_current_A"].relative_difference) < 0.15
    assert abs(rows["top_speed_m_s"].relative_difference) < 0.20
    assert abs(rows["endurance_at_cruise_min"].relative_difference) < 0.25


def test_envelope_relations(data):
    aircraft, condition, performance, _, envelope = data
    weight = aircraft.m * condition.g
    assert envelope.stall_speed == pytest.approx(
        math.sqrt(2 * weight / (condition.rho * aircraft.S * performance.CL_max)))
    assert envelope.corner_speed == pytest.approx(envelope.stall_speed * math.sqrt(4.5))
    top = envelope.top_speed
    assert envelope.max_thrust(top) == pytest.approx(envelope.drag(top), abs=0.05)
    assert envelope.sustained_load_factor(top) == pytest.approx(1.0, abs=0.1)
    assert envelope.climb_rate(top) == pytest.approx(0.0, abs=0.05)
    for V in np.linspace(envelope.stall_speed * 1.01, top, 12):
        assert envelope.sustained_load_factor(V) <= envelope.instantaneous_load_factor(V) + 1e-9
    assert envelope.instantaneous_load_factor(envelope.stall_speed) == pytest.approx(1.0)
    assert envelope.level_current(top * 1.1) is None


def test_turn_formulas():
    from src.performance import Envelope
    assert Envelope.turn_radius(20.0, 2.0) == pytest.approx(400 / (9.81 * math.sqrt(3)))
    assert Envelope.turn_rate(20.0, 2.0) == pytest.approx(9.81 * math.sqrt(3) / 20.0)
    assert Envelope.turn_radius(20.0, 1.0) == math.inf


def test_performance_drag_replaces_the_aircraft_file_drag(data):
    aircraft, condition, performance, _, envelope = data
    plain = NonlinearAircraft(aircraft, condition)
    limited = NonlinearAircraft(aircraft, condition, performance=performance)
    assert limited.thrust_trim == pytest.approx(envelope.drag(condition.V), rel=1e-6)
    assert limited.thrust_trim > 3.0 * plain.thrust_trim
    rates = limited.derivative(limited.trim_state(), limited.trim_controls())
    assert rates[1:] == pytest.approx(0.0, abs=1e-9)


def test_lift_is_capped_and_flagged_beyond_the_limit(data):
    aircraft, condition, performance, _, _ = data
    model = NonlinearAircraft(aircraft, condition, performance=performance)
    _, _, CL = model.aerodynamics(20.0, math.radians(25.0), 0, 0, 0, 0, 0, 0, 0)
    assert CL == pytest.approx(performance.CL_max)
    x = model.trim_state()
    x[3], x[5] = 20.0 * math.cos(math.radians(25.0)), 20.0 * math.sin(math.radians(25.0))
    assert model.outputs(x, tuple(model.trim_controls()))["valid"] == 0.0
    assert model.envelope.alpha_max == pytest.approx((1.41 - model.CL0) / model.CLa)


def test_full_throttle_reaches_top_speed_on_the_straights(data, laps):
    _, _, _, _, envelope = data
    assert laps[4.5][3]["speed_max"] == pytest.approx(envelope.top_speed, rel=0.02)


def test_structural_limit_is_respected(laps):
    performance, _, tr, metrics = laps[4.5]
    assert metrics["load_factor_max"] <= performance.load_factor_limit + 1e-6
    assert metrics["load_factor_max"] > 0.9 * performance.load_factor_limit
    assert tr["load_factor"].max() <= performance.load_factor_limit + 1e-6


def test_harder_turns_are_faster_but_lose_more_speed(laps):
    soft, hard = laps[2.0][3], laps[4.5][3]
    assert hard["lap_time"] < 0.8 * soft["lap_time"]
    assert hard["speed_min"] < soft["speed_min"] - 2.0
    assert hard["amp_hours"] < soft["amp_hours"]


def test_limit_lap_stays_inside_the_valid_range_and_below_maximum_lift(laps):
    performance, _, _, metrics = laps[4.5]
    assert metrics["outside_envelope"] == 0.0
    assert metrics["CL_max"] < performance.CL_max


def test_battery_covers_the_mission(laps):
    _, autopilot, _, metrics = laps[4.5]
    assert metrics["laps_on_battery"] > 3.0          # Mission 1 is three laps


def test_limit_lap_time_converges_with_step_size(laps):
    _, performance, autopilot, tr = fly_limit(4.5, dt=0.01)
    assert lap_metrics(performance, autopilot, tr)["lap_time"] == pytest.approx(
        laps[4.5][3]["lap_time"], rel=5e-3)
