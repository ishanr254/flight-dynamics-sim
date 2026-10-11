"""A/B entries checked against the Ch. 9 eq. (5) definitions, evaluated independently here."""

import math
from dataclasses import replace

import pytest

from src.aircraft import Aircraft, FlightCondition, LateralCoefficients
from src.dynamics import lateral_state_space

COEFFS = LateralCoefficients(
    CY_beta=-0.65, CY_p=0.0, CY_r=0.0, CY_dl=0.0, CY_dn=0.19,
    Cl_beta=-0.92, Cl_p=-18.6, Cl_r=5.89, Cl_dl=-0.56, Cl_dn=0.13,
    Cn_beta=0.8175, Cn_p=-1.37, Cn_r=-7.18, Cn_dl=-0.02, Cn_dn=-0.56)
AIRCRAFT = Aircraft(name="test", source="", m=1.2e5, S=240.0, l=6.5, A=5.88e6, C=11.1e6,
                    E=0.0, lateral=COEFFS)
LEVEL = FlightCondition(V=125.0, rho=1.225)


def test_state_and_input_order():
    ss = lateral_state_space(AIRCRAFT, LEVEL)
    assert ss.states == ("beta", "r", "p", "phi")
    assert ss.inputs == ("delta_l", "delta_n")


def test_entries_match_notes_definitions():
    rho, S, V, l, m, A_, C_ = 1.225, 240.0, 125.0, 6.5, 1.2e5, 5.88e6, 11.1e6
    ss = lateral_state_space(AIRCRAFT, LEVEL)
    expected = {
        (0, 0): rho * S * V ** 2 * -0.65 / (2 * m) / V,          # Y_beta / V
        (1, 0): rho * S * l * V ** 2 * 0.8175 / (2 * C_),        # n_beta
        (1, 1): l / (2 * C_ * V) * rho * S * l * V ** 2 * -7.18,  # n_r
        (1, 2): l / (2 * C_ * V) * rho * S * l * V ** 2 * -1.37,  # n_p
        (2, 0): rho * S * l * V ** 2 * -0.92 / (2 * A_),         # l_beta
        (2, 1): l / (2 * A_ * V) * rho * S * l * V ** 2 * 5.89,   # l_r
        (2, 2): l / (2 * A_ * V) * rho * S * l * V ** 2 * -18.6,  # l_p
    }
    for (i, j), value in expected.items():
        assert ss.A[i, j] == pytest.approx(value, rel=1e-12)
    assert ss.B[0, 1] == pytest.approx(rho * S * V ** 2 * 0.19 / (2 * m) / V, rel=1e-12)
    assert ss.B[2, 0] == pytest.approx(rho * S * l * V ** 2 * -0.56 / (2 * A_), rel=1e-12)
    assert ss.B[1, 1] == pytest.approx(rho * S * l * V ** 2 * -0.56 / (2 * C_), rel=1e-12)


def test_kinematic_and_gravity_terms_level_flight():
    ss = lateral_state_space(AIRCRAFT, LEVEL)
    assert ss.A[0, 1] == -1.0
    assert ss.A[0, 2] == 0.0
    assert ss.A[0, 3] == pytest.approx(9.81 / 125.0)
    assert list(ss.A[3]) == [0.0, 0.0, 1.0, 0.0]
    assert list(ss.B[3]) == [0.0, 0.0]


def test_alpha_and_theta_enter_as_in_eq_5():
    alpha, theta = 0.1, 0.05
    ss = lateral_state_space(AIRCRAFT, FlightCondition(V=125.0, rho=1.225, alpha=alpha,
                                                       theta=theta))
    assert ss.A[0, 1] == pytest.approx(-math.cos(alpha))
    assert ss.A[0, 2] == pytest.approx(math.sin(alpha))
    assert ss.A[0, 3] == pytest.approx(9.81 * math.cos(theta) / 125.0)
    assert ss.A[3, 1] == pytest.approx(math.tan(theta))


def test_rate_derivatives_scale_with_l_over_V_not_l_over_2V():
    slow = lateral_state_space(AIRCRAFT, LEVEL)
    doubled_l = lateral_state_space(replace(AIRCRAFT, l=13.0), LEVEL)
    # l_p ~ l^2 (one l from the moment arm, one from the p*l/V normalization)
    assert doubled_l.A[2, 2] == pytest.approx(4.0 * slow.A[2, 2])
    assert doubled_l.A[2, 0] == pytest.approx(2.0 * slow.A[2, 0])


def test_nonzero_product_of_inertia_warns():
    with pytest.warns(UserWarning, match="product of inertia"):
        lateral_state_space(replace(AIRCRAFT, E=1.0e5), LEVEL)
