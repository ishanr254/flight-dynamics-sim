"""Autopilot and course flight for DF1 Mission 1."""

import math

import numpy as np
import pytest

from cases.df1_course import COURSE, fly, lap_summary
from src.simulate import load_course


@pytest.fixture(scope="module")
def course():
    return load_course(COURSE)


@pytest.fixture(scope="module")
def flight(course):
    model, autopilot, trajectory = fly(course, dt=0.02)
    return autopilot, trajectory, lap_summary(course, autopilot, trajectory)


def test_course_file_matches_the_rules_figure(course):
    assert course.half_length == pytest.approx(500 * 0.3048)
    assert course.laps == 3 and course.full_turn and course.turn_direction == 1


def test_all_laps_are_completed_and_repeatable(flight, course):
    autopilot, _, summary = flight
    assert len(autopilot.lap_times) == course.laps
    assert max(summary["laps"]) - min(summary["laps"]) < 0.01 * summary["laps"][0]


def test_every_lap_has_the_phases_in_order(flight):
    autopilot, _, _ = flight
    phases = [phase for _, phase in autopilot.events]
    lap = ["upwind", "turn_1", "downwind_1", "full_turn", "downwind_2", "turn_2"]
    assert phases[:len(lap) * 3] == lap * 3


def test_ground_track_has_the_shape_of_the_course(flight, course):
    _, tr, summary = flight
    assert tr["north"].max() > course.half_length and tr["north"].min() < -course.half_length
    assert summary["downwind_offset_m"] > 60.0          # right-hand turns put downwind to the east
    assert tr["east"].max() > summary["downwind_offset_m"] + 60.0   # the 360 is on the outside
    assert abs(tr["east"][np.abs(tr["north"]) < 20][0]) < 5.0       # crosses the start line on the runway line


def test_heading_change_per_lap_is_zero(flight):
    """Two right-hand 180 deg turns and one left-hand 360 deg turn cancel."""
    autopilot, tr, _ = flight
    heading = np.unwrap(tr["psi"])
    first_lap_end = np.searchsorted(tr.t, autopilot.lap_times[0])
    assert heading[first_lap_end] - heading[0] == pytest.approx(0.0, abs=math.radians(5))
    assert heading.max() - heading.min() == pytest.approx(2 * math.pi, abs=math.radians(15))


def test_turns_respect_the_load_factor_limit(flight, course):
    _, _, summary = flight
    assert summary["max_load_factor"] < 1.1 * course.load_factor_limit
    assert summary["max_bank_deg"] == pytest.approx(60.0, abs=3.0)


def test_turn_radius_matches_the_steady_turn_formula(flight, course):
    _, _, summary = flight
    formula = course.speed ** 2 / (9.81 * math.tan(math.radians(60.0)))
    assert summary["turn_radius_m"] == pytest.approx(formula, rel=0.08)


def test_speed_and_altitude_are_held(flight, course):
    _, _, summary = flight
    assert abs(summary["speed_range"][0] - course.speed) < 1.0
    assert abs(summary["speed_range"][1] - course.speed) < 1.0
    assert abs(summary["altitude_range"][0] - course.altitude) < 2.0
    assert abs(summary["altitude_range"][1] - course.altitude) < 2.0


def test_flight_stays_inside_the_model_envelope_and_control_limits(flight, course):
    _, tr, summary = flight
    assert summary["outside_envelope"] == 0.0
    limit = math.degrees(course.deflection_max)
    assert max(summary["max_elevator_deg"], summary["max_aileron_deg"],
               summary["max_rudder_deg"]) < 0.5 * limit
    assert summary["thrust_range"][1] < course.thrust_max


def test_lap_time_converges_with_step_size(flight, course):
    autopilot, _, _ = flight
    _, finer, _ = fly(course, dt=0.01)
    assert finer.lap_times[-1] == pytest.approx(autopilot.lap_times[-1], rel=2e-3)
