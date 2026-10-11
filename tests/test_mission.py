"""Wind, takeoff roll, landing circuit and roll-out."""

import math

import numpy as np
import pytest

from cases.df1_performance import setup, whole_mission
from src.simulate import (CourseAutopilot, NonlinearAircraft, load_course,
                          simulate_nonlinear, takeoff_roll)

RUNWAY_HALF_LENGTH = 85.0     # m, as drawn in the viewer


@pytest.fixture(scope="module")
def data():
    aircraft, condition, performance, table, envelope = setup()
    return aircraft, condition, performance, table, envelope


@pytest.fixture(scope="module")
def flight(data):
    aircraft, condition, performance, _, _ = data
    return whole_mission(aircraft, condition, performance, 4.5, wind=5.0, laps=1, dt=0.02)


def test_wind_moves_the_ground_track_but_not_the_airspeed(data):
    aircraft, condition, performance, _, _ = data
    model = NonlinearAircraft(aircraft, condition, performance=performance, wind=(-5.0, 2.0))
    tr = simulate_nonlinear(model, 10.0, 0.01)
    assert tr["V"][-1] == pytest.approx(condition.V, abs=1e-6)
    assert tr["north"][-1] == pytest.approx((condition.V - 5.0) * 10.0, rel=1e-6)
    assert tr["east"][-1] == pytest.approx(20.0, rel=1e-6)
    assert tr["ground_speed"][-1] == pytest.approx(math.hypot(condition.V - 5.0, 2.0))


def test_headwind_shortens_the_takeoff_roll(data):
    aircraft, condition, performance, table, envelope = data
    liftoff = 1.2 * envelope.stall_speed
    distances = []
    for wind in (0.0, 5.0):
        model = NonlinearAircraft(aircraft, condition, performance=performance, wind=(-wind, 0.0))
        times, states, rows = takeoff_roll(model, table, liftoff)
        assert rows[-1]["V"] >= liftoff and rows[0]["V"] == pytest.approx(wind, abs=1e-3)
        assert rows[-1]["altitude"] == 0.0 and rows[-1]["throttle"] == 1.0
        distances.append(states[-1][0])
    assert 0.0 < distances[1] < 0.6 * distances[0]


def test_time_zero_is_throttle_up(flight):
    _, result, autopilot = flight
    assert result.liftoff_time > 0.0
    assert result.trajectory.t[0] == 0.0
    assert autopilot.lap_times[0] > result.liftoff_time
    assert np.all(np.diff(result.trajectory.t) > 0)


def test_liftoff_at_1_2_times_stall(flight, data):
    _, result, _ = flight
    assert result.liftoff_speed == pytest.approx(1.2 * data[4].stall_speed)


def test_phases_end_with_a_landing_circuit_without_the_360(flight):
    _, _, autopilot = flight
    phases = [phase for _, phase in autopilot.events]
    assert phases == ["upwind", "turn_1", "downwind_1", "full_turn", "downwind_2", "turn_2",
                      "upwind", "turn_1", "downwind_1", "downwind_2", "turn_2", "final"]


def test_aircraft_climbs_to_pattern_height_and_never_goes_underground(flight):
    course, result, _ = flight
    altitude = result.trajectory["altitude"]
    assert altitude.max() == pytest.approx(course.altitude, abs=2.0)
    assert altitude.min() > -0.3


def test_touchdown_is_on_the_runway_at_about_approach_speed(flight, data):
    _, result, _ = flight
    tr = result.trajectory
    assert abs(result.touchdown_north) < RUNWAY_HALF_LENGTH
    k = np.searchsorted(tr.t, result.touchdown_time)
    assert tr["V"][k] == pytest.approx(1.3 * data[4].stall_speed, rel=0.15)
    assert abs(tr["east"][k]) < 5.0
    assert -3.0 < tr["climb_rate"][k] < 0.0


def test_roll_out_ends_stopped_on_the_runway(flight):
    _, result, _ = flight
    tr = result.trajectory
    assert result.stop_time > result.touchdown_time
    assert result.touchdown_north < result.stop_north < RUNWAY_HALF_LENGTH
    assert tr["ground_speed"][-1] < 0.5 and tr["throttle"][-1] == 0.0


def test_approach_uses_little_battery(flight):
    _, result, autopilot = flight
    tr = result.trajectory
    used = np.concatenate([[0.0], np.cumsum(0.5 * (tr["current"][1:] + tr["current"][:-1])
                                            * np.diff(tr.t))]) / 3600.0
    lap = np.interp(autopilot.lap_times[-1], tr.t, used)
    assert 0.0 < used[-1] - lap < 0.6 * lap


def test_no_landing_unless_asked(data):
    aircraft, condition, performance, table, _ = data
    course = load_course("data/courses/dbf_2026_m1.toml")
    autopilot = CourseAutopilot(course, performance=performance, thrust_table=table)
    assert autopilot.land is False and autopilot.touchdown is None
