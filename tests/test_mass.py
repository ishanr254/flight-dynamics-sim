import math
from pathlib import Path

import pytest

from src.aircraft import load_aircraft
from src.aircraft.mass import box_inertia, load_spread, spread_increment, to_stability_axes
from src.aircraft.xflr5 import aircraft_toml, inertia, parse_log
from src.analysis import modes, name_lateral_modes
from src.dynamics import lateral_state_space

LOG = Path("data/aircraft/m1Log.rtf")
SPREAD = "data/aircraft/df1_mass_distribution.toml"


@pytest.fixture(scope="module")
def log():
    return parse_log(LOG.read_text(errors="replace"))


def test_spanwise_rod_is_m_b_squared_over_12():
    assert box_inertia(2.0, [0.0, 3.0, 0.0]) == pytest.approx((1.5, 0.0, 1.5))


def test_box_matches_textbook_formula():
    assert box_inertia(12.0, [1.0, 2.0, 3.0]) == pytest.approx((13.0, 10.0, 5.0))


def test_rotation_reproduces_xflr5_stability_axis_inertia(log):
    """XFLR5 prints both body- and stability-axis inertia; the rotation must link them."""
    rotated = to_stability_axes(log["Ibxx"], log["Ibzz"], log["Ibxz"],
                                math.radians(log["alpha_trim_deg"]))
    assert rotated == pytest.approx((log["Isxx"], log["Iszz"], log["Isxz"]), rel=2e-3)


def test_spread_increment_is_sum_of_boxes():
    entries = load_spread(SPREAD)
    wing = next(e for e in entries if e["component"] == "Main wing")
    assert box_inertia(wing["mass"], wing["extent"])[0] == pytest.approx(
        0.398 * 1.1814 ** 2 / 12)
    dxx, dyy, dzz = spread_increment(entries)
    assert dxx == pytest.approx(sum(box_inertia(e["mass"], e["extent"])[0] for e in entries))
    assert dzz > dxx > 0 and dyy > 0


def test_spread_masses_exist_in_the_xflr5_model():
    xml = Path("data/aircraft/Mission 1 Iteration 2.xml").read_text()
    for entry in load_spread(SPREAD):
        assert f"<Mass>  {entry['mass']:.3f}</Mass>" in xml, entry["component"]


def test_spread_changes_inertia_only(log, tmp_path):
    spread = load_spread(SPREAD)
    plain, with_spread = tmp_path / "plain.toml", tmp_path / "spread.toml"
    plain.write_text(aircraft_toml(log, 0.307, "test"))
    with_spread.write_text(aircraft_toml(log, 0.307, "test", spread, SPREAD))
    (a, fc_a), (b, fc_b) = load_aircraft(plain), load_aircraft(with_spread)
    assert (a.m, a.S, a.l, a.lateral, fc_a) == (b.m, b.S, b.l, b.lateral, fc_b)
    assert b.A > a.A and b.C > a.C
    assert (b.A, b.C, b.E) == pytest.approx(inertia(log, spread))


def test_distributed_mass_slows_the_roll_mode():
    def roll_rate(path):
        named = name_lateral_modes(modes(lateral_state_space(*load_aircraft(path)).A))
        return named["roll"].eigenvalue.real
    assert roll_rate("data/aircraft/df1_m1.toml") == pytest.approx(-147.9, rel=1e-3)
    assert -40.0 < roll_rate("data/aircraft/df1_m1_distributed.toml") < -15.0


def test_bad_spread_entry_is_rejected(tmp_path):
    path = tmp_path / "bad.toml"
    path.write_text('[[spread]]\ncomponent = "Wing"\nmass = 0.4\nextent = [1.0, 2.0]\n')
    with pytest.raises(ValueError, match="'Wing' needs extent"):
        load_spread(path)
