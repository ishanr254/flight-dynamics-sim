"""Longitudinal model checked against XFLR5's printed state matrix and by construction."""

import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from src.aircraft import Aircraft, FlightCondition, LongitudinalCoefficients, load_aircraft
from src.aircraft.xflr5 import aircraft_toml, course_longitudinal, parse_log
from src.analysis import modes, name_longitudinal_modes
from src.dynamics import lateral_state_space, longitudinal_state_space
from src.verify import (lanchester_phugoid_period, longitudinal_modal_metrics,
                        short_period_approximation)

LOG = Path("data/aircraft/m1Log.rtf")
# XFLR5's "Longitudinal state matrix" for Mission 1, states [u, w, q, theta]
XFLR5_MATRIX = np.array([
    [-0.0476594, 0.176211, 0.0, -9.81],
    [-0.706931, -8.08229, 25.5858, 0.0],
    [-0.0334432, -8.7701, -13.7714, 0.0],
    [0.0, 0.0, 1.0, 0.0]])


@pytest.fixture(scope="module")
def log():
    return parse_log(LOG.read_text(errors="replace"))


@pytest.fixture(scope="module")
def df1():
    return load_aircraft("data/aircraft/df1_m1.toml")


def test_state_and_input_order(df1):
    ss = longitudinal_state_space(*df1)
    assert ss.states == ("u", "alpha", "q", "theta")
    assert ss.inputs == ("delta_x", "delta_m")
    assert not ss.B.any()


def test_matrix_equals_xflr5_matrix_with_w_replaced_by_V_alpha(df1):
    """[u, w, q, theta] -> [u, alpha, q, theta] scales the w column by V and the w row by 1/V."""
    V = df1[1].V
    T = np.diag([1.0, V, 1.0, 1.0])
    expected = np.linalg.inv(T) @ XFLR5_MATRIX @ T
    assert longitudinal_state_space(*df1).A == pytest.approx(expected, rel=2e-4, abs=1e-9)


def test_coefficients_follow_the_documented_definitions(log):
    c = course_longitudinal(log, 0.307)
    V = log["V"]
    qS = 0.5 * log["rho"] * V ** 2 * log["S"]
    assert qS * c["CZ_alpha"] / V == pytest.approx(log["Zw"])
    assert qS * 0.307 / V * c["CZ_q"] == pytest.approx(log["Zq"])
    assert qS * 0.307 * 0.307 / V * c["Cm_q"] == pytest.approx(log["Mq"])
    assert c["CZ_u"] == pytest.approx(-2 * log["mass"] * 9.81 / qS, rel=2e-3)   # about -2 CL


def test_modes_do_not_depend_on_the_reference_length_chosen(log, tmp_path):
    spectra = []
    for chord in (0.307, 1.0):
        path = tmp_path / f"chord_{chord}.toml"
        path.write_text(aircraft_toml(log, chord, "test"))
        spectra.append(np.sort_complex(np.linalg.eigvals(
            longitudinal_state_space(*load_aircraft(path)).A)))
    assert spectra[0] == pytest.approx(spectra[1], rel=1e-9)


def test_climb_angle_enters_through_gravity_terms(df1):
    aircraft, condition = df1
    gamma = 0.1
    ss = longitudinal_state_space(aircraft, replace(condition, theta=gamma))
    assert ss.A[0, 3] == pytest.approx(-9.81 * math.cos(gamma))
    assert ss.A[1, 3] == pytest.approx(-9.81 * math.sin(gamma) / condition.V)


def test_mode_naming_and_fallback(df1):
    named = name_longitudinal_modes(modes(longitudinal_state_space(*df1).A))
    assert named["short_period"].wn > named["phugoid"].wn
    assert set(name_longitudinal_modes(modes(np.diag([-1.0, -2.0, -3.0, -4.0])))) == {
        "mode_1", "mode_2", "mode_3", "mode_4"}


def test_short_period_approximation_is_close_for_df1(df1):
    ss = longitudinal_state_space(*df1)
    full = longitudinal_modal_metrics(ss)
    wn, zeta = short_period_approximation(ss)
    assert wn == pytest.approx(full["short_period_natural_frequency_rad_s"], rel=0.01)
    assert zeta == pytest.approx(full["short_period_damping_ratio"], rel=0.01)


def test_lanchester_period_is_about_0_45_V():
    assert lanchester_phugoid_period(100.0) == pytest.approx(45.3, abs=0.1)   # CS5 eq. (2)


def test_asking_for_a_missing_motion_is_a_clear_error(df1):
    aircraft, condition = df1
    with pytest.raises(ValueError, match=r"no \[longitudinal\] section"):
        longitudinal_state_space(replace(aircraft, longitudinal=None), condition)
    with pytest.raises(ValueError, match=r"no \[lateral\] section"):
        lateral_state_space(replace(aircraft, lateral=None), condition)


def test_statically_unstable_aircraft_is_reported_unstable():
    c = LongitudinalCoefficients(CX_u=-0.05, CX_alpha=0.1, CZ_u=-0.6, CZ_alpha=-4.0,
                                 CZ_q=-2.0, Cm_u=0.0, Cm_alpha=+0.5, Cm_q=-4.0)
    aircraft = Aircraft("unstable", "", m=3.0, S=0.4, l=0.3, B=0.2, longitudinal=c)
    found = modes(longitudinal_state_space(aircraft, FlightCondition(V=20.0, rho=1.225)).A)
    assert any(not m.stable for m in found)
