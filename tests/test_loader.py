import re
from pathlib import Path

import pytest

from src.aircraft import AircraftDataError, load_aircraft

CS7 = Path("data/aircraft/cs7_transport.toml")


def write_variant(tmp_path, pattern, replacement):
    text, count = re.subn(pattern, replacement, CS7.read_text(), flags=re.MULTILINE)
    assert count >= 1, f"pattern {pattern!r} not found in {CS7}"
    path = tmp_path / "variant.toml"
    path.write_text(text)
    return path


def test_loads_cs7_values_as_given():
    aircraft, condition = load_aircraft(CS7)
    assert aircraft.m == 1.2e5
    assert aircraft.l == 6.5
    assert aircraft.lateral.Cl_p == -18.6
    assert condition.V == 125.0
    assert condition.alpha == 0.0


def test_missing_field_is_named(tmp_path):
    path = write_variant(tmp_path, r"^Cn_r = .*$", "")
    with pytest.raises(AircraftDataError, match=r"\[lateral\] missing required field 'Cn_r'"):
        load_aircraft(path)


def test_all_problems_reported_together(tmp_path):
    path = write_variant(tmp_path, r"^Cn_r = .*$", "")
    path.write_text(re.sub(r"^m = .*$", "m = -5", path.read_text(), flags=re.MULTILINE))
    with pytest.raises(AircraftDataError) as excinfo:
        load_aircraft(path)
    assert len(excinfo.value.problems) == 2


def test_misspelled_field_is_flagged(tmp_path):
    path = write_variant(tmp_path, r"^Cl_beta = ", "Clbeta = ")
    with pytest.raises(AircraftDataError) as excinfo:
        load_aircraft(path)
    message = str(excinfo.value)
    assert "unknown field 'Clbeta'" in message
    assert "missing required field 'Cl_beta'" in message


def test_angle_in_degrees_is_caught(tmp_path):
    path = write_variant(tmp_path, r"^alpha = .*$", "alpha = 5.0")
    with pytest.raises(AircraftDataError, match="radians"):
        load_aircraft(path)


def test_unrealizable_inertia_is_rejected(tmp_path):
    path = write_variant(tmp_path, r"^E = .*$", "E = 9.0e6")
    with pytest.raises(AircraftDataError, match="not physically realizable"):
        load_aircraft(path)


def test_non_numeric_value_is_rejected(tmp_path):
    path = write_variant(tmp_path, r"^V = .*$", 'V = "125 m/s"')
    with pytest.raises(AircraftDataError, match="'V' must be a number"):
        load_aircraft(path)
