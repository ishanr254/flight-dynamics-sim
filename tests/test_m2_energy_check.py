"""Propulsion model against every static test, and the reported Mission 2 flight."""

import pytest

from cases.df1_m2_energy_check import (LAPS, MASS, PACK_AH, REPORT_TAKEOFF_DISTANCE,
                                       REPORT_TAKEOFF_SPEED, REPORTED_AH, WINDOW, fly, setup,
                                       static_checks)


def test_one_thrust_factor_is_roughly_right_for_all_four_static_tests():
    """Measured thrust is 75-83% of the model's across both propellers and both packs."""
    for label, thrust_ratio, current_ratio in static_checks():
        assert 0.72 < thrust_ratio < 0.86, label
        assert 0.85 < current_ratio < 1.20, label


def test_mission_2_configuration():
    aircraft, _, performance = setup()
    assert aircraft.m == pytest.approx(MASS) and MASS == pytest.approx(3.279, abs=0.001)
    assert performance.powertrain.propeller.name == "APC 14x7E"
    assert performance.powertrain.battery.capacity_ah == PACK_AH


@pytest.fixture(scope="module")
def eight_laps():
    return fly(3.0, dt=0.02)


def test_takeoff_roll_matches_the_report():
    """Still air, Mission 2: the report's Table 4.4.5 gives 24.78 ft at 39.94 ft/s."""
    calm = fly(3.0, wind=0.0, laps=1, dt=0.02)
    assert calm["liftoff_speed"] == pytest.approx(REPORT_TAKEOFF_SPEED, rel=0.02)
    assert calm["liftoff_distance"] == pytest.approx(REPORT_TAKEOFF_DISTANCE, rel=0.08)


def test_eight_laps_fit_the_window_as_flown(eight_laps):
    """The flight completed eight laps; the model agrees that is possible at 3 g."""
    assert LAPS == 8 and eight_laps["laps_time"] < WINDOW


def test_model_uses_about_three_quarters_of_what_the_flight_did(eight_laps):
    """Recorded gap: the flight left the pack at about 2% (about 4.4 Ah); the model's whole
    flight, with takeoff, landing and wind, uses about 3.2 Ah. This pins the model's number
    and the direction of the gap. It does not assert agreement."""
    assert REPORTED_AH == pytest.approx(4.41, abs=0.01)
    assert eight_laps["amp_hours_total"] == pytest.approx(3.23, abs=0.15)
    assert eight_laps["amp_hours_total"] > eight_laps["amp_hours_laps"]
    assert 0.65 < eight_laps["amp_hours_total"] / REPORTED_AH < 0.82


def test_more_drag_means_a_slower_aircraft_that_uses_more_battery(eight_laps):
    """The propeller draws more current at lower airspeed, so a draggier aircraft uses
    more energy per lap, not just more time."""
    draggy = fly(3.0, dt=0.02, drag_factor=1.5)
    assert draggy["speed_max"] < eight_laps["speed_max"] - 2.0
    assert draggy["mean_current"] > eight_laps["mean_current"] + 2.0
    assert draggy["amp_hours_total"] > 1.1 * eight_laps["amp_hours_total"]
