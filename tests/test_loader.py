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


def test_lateral_only_file_has_no_longitudinal_model():
    aircraft, _ = load_aircraft(CS7)
    assert aircraft.longitudinal is None and aircraft.B is None
    assert aircraft.lateral is not None


def test_file_with_both_motions_loads_both():
    aircraft, _ = load_aircraft("data/aircraft/df1_m1.toml")
    assert aircraft.lateral is not None and aircraft.longitudinal is not None
    assert aircraft.B == 0.1831


def test_longitudinal_section_needs_pitch_inertia(tmp_path):
    text = Path("data/aircraft/df1_m1.toml").read_text()
    path = tmp_path / "no_B.toml"
    path.write_text(re.sub(r"^B = .*$", "", text, flags=re.MULTILINE))
    with pytest.raises(AircraftDataError, match=r"'B' is required when \[longitudinal\]"):
        load_aircraft(path)


def test_longitudinal_only_file_loads(tmp_path):
    text = Path("data/aircraft/df1_m1.toml").read_text()
    start, end = text.index("[lateral]"), text.index("# Speed derivatives")
    path = tmp_path / "longitudinal_only.toml"
    path.write_text(text[:start] + text[end:])
    aircraft, _ = load_aircraft(path)
    assert aircraft.lateral is None and aircraft.longitudinal is not None


def test_file_with_no_motion_section_is_rejected(tmp_path):
    text = CS7.read_text()
    path = tmp_path / "nothing.toml"
    path.write_text(text[:text.index("# Rate derivatives")] + text[text.index("# Straight"):])
    with pytest.raises(AircraftDataError, match="nothing to analyse"):
        load_aircraft(path)
