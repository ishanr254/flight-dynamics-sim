from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from src.aircraft import load_aircraft
from src.aircraft.xflr5 import XflrLogError, aircraft_toml, course_lateral, parse_log
from src.analysis import modes
from src.dynamics import lateral_state_space

LOG = Path("data/aircraft/m1Log.rtf")


@pytest.fixture(scope="module")
def log():
    return parse_log(LOG.read_text(errors="replace"))


def test_parses_values_printed_in_the_log(log):
    assert log["name"] == "Mission 1 Iteration 2"
    assert log["mass"] == 2.973
    assert log["V"] == 27.76911
    assert log["b"] == 1.182
    assert log["Isxz"] == 0.01889
    assert log["Clp"] == -0.33636
    assert log["alpha_trim_deg"] == -1.08365
    assert complex(-1.279, 8.325) in log["lateral_eigenvalues"]


def test_incomplete_log_names_what_is_missing():
    with pytest.raises(XflrLogError, match="Clp"):
        parse_log(LOG.read_text(errors="replace").replace("Clp=", "Xlp="))


def test_conversion_reproduces_xflr5_dimensional_derivatives(log):
    """Lp, Nv etc. rebuilt from the converted coefficients must equal the log's own values."""
    chord = 0.307
    c = course_lateral(log, chord)
    qS = 0.5 * log["rho"] * log["V"] ** 2 * log["S"]
    V = log["V"]
    assert qS * chord * c["Cl_p"] * chord / V == pytest.approx(-1.4502, rel=1e-4)   # Lp
    assert qS * chord * c["Cn_r"] * chord / V == pytest.approx(-0.37048, rel=1e-4)  # Nr
    assert qS * chord * c["Cn_beta"] / V == pytest.approx(0.43607, rel=1e-4)        # Nv
    assert qS * c["CY_r"] * chord / V == pytest.approx(0.48071, rel=1e-4)           # Yr


def test_dynamics_do_not_depend_on_the_reference_length_chosen(log, tmp_path):
    spectra = []
    for chord in (0.307, 1.182):
        path = tmp_path / f"chord_{chord}.toml"
        path.write_text(aircraft_toml(log, chord, "test"))
        spectra.append(np.sort_complex(np.linalg.eigvals(
            lateral_state_space(*load_aircraft(path)).A)))
    assert spectra[0] == pytest.approx(spectra[1], rel=1e-9)


def test_inertia_coupling_is_needed_to_match_xflr5():
    aircraft, condition = load_aircraft("data/aircraft/df1_m1.toml")
    roll = lambda ac: min(m.eigenvalue.real for m in modes(lateral_state_space(ac, condition).A))
    assert roll(aircraft) == pytest.approx(-147.9, rel=1e-3)
    assert roll(replace(aircraft, E=0.0)) == pytest.approx(-123.0, rel=0.02)
