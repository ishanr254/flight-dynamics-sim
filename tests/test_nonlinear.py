"""Nonlinear model: it must reduce to the linear models for small disturbances."""

import math

import numpy as np
import pytest

from src.aircraft import load_aircraft
from src.dynamics import lateral_state_space, longitudinal_state_space
from src.simulate import (Envelope, ModelError, NonlinearAircraft, simulate,
                          simulate_nonlinear)
from src.simulate.nonlinear import euler_from_quaternion, quaternion_from_euler

AIRCRAFT = "data/aircraft/df1_m1_asbuilt.toml"


@pytest.fixture(scope="module")
def loaded():
    return load_aircraft(AIRCRAFT)


@pytest.fixture(scope="module")
def model(loaded):
    return NonlinearAircraft(*loaded)


def jacobian(model):
    x0, u0 = model.trim_state(), model.trim_controls()
    J = np.zeros((13, 13))
    for j in range(13):
        step = np.zeros(13)
        step[j] = 1e-6 * max(1.0, abs(x0[j]))
        J[:, j] = (model.derivative(x0 + step, u0) - model.derivative(x0 - step, u0)) / (2 * step[j])
    return J


def test_trim_is_an_equilibrium(model):
    rates = model.derivative(model.trim_state(), model.trim_controls())
    assert rates[0] == pytest.approx(model.V0)          # flying north at the trim speed
    assert rates[1:] == pytest.approx(0.0, abs=1e-9)


def test_linearizing_about_trim_returns_every_linear_mode(model, loaded):
    nonlinear = np.linalg.eigvals(jacobian(model))
    linear = np.concatenate([np.linalg.eigvals(longitudinal_state_space(*loaded).A),
                             np.linalg.eigvals(lateral_state_space(*loaded).A)])
    for value in linear:
        assert np.abs(nonlinear - value).min() < 1e-6 * max(1.0, abs(value))


@pytest.mark.parametrize("state,index,builder,watch", [
    ("alpha", 5, longitudinal_state_space, "q"),
    ("beta", 4, lateral_state_space, "r"),
])
def test_small_disturbance_matches_the_linear_simulation(model, loaded, state, index,
                                                         builder, watch):
    angle = math.radians(1.0)
    x = model.trim_state()
    x[3], x[index] = model.V0 * math.cos(angle), model.V0 * math.sin(angle)
    nonlinear = simulate_nonlinear(model, 2.0, 0.002, x0=x)
    linear = simulate(builder(*loaded), 2.0, x0={state: angle}, n_samples=1001)
    peak = np.abs(linear[watch]).max()
    assert np.abs(nonlinear[watch] - linear[watch]).max() < 0.005 * peak
    assert np.abs(nonlinear[state] - linear[state]).max() < 0.005 * angle


def test_large_disturbance_departs_from_the_linear_simulation(model, loaded):
    """At 60 deg of bank the gravity term is no longer linear in the bank angle."""
    bank = math.radians(60.0)
    x = model.trim_state()
    x[6:10] = quaternion_from_euler(bank, 0.0, 0.0)
    nonlinear = simulate_nonlinear(model, 3.0, 0.005, x0=x)
    linear = simulate(lateral_state_space(*loaded), 3.0, x0={"phi": bank}, n_samples=601)
    assert abs(nonlinear["beta"][-1] - linear["beta"][-1]) > 0.1 * abs(linear["beta"]).max()


def test_drag_polar_reproduces_trim_drag_and_grows_with_lift(model):
    qS = 0.5 * model.rho * model.V0 ** 2 * model.S
    (X, _, _), _, _ = model.aerodynamics(model.V0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert -X == pytest.approx(model.thrust_trim)
    assert model.CDmin + model.k * model.CL0 ** 2 == pytest.approx(-X / qS)
    assert model.k > 0 and model.CDmin > 0


def test_controls_are_limited(model):
    limited = NonlinearAircraft(model.aircraft, model.condition, thrust_max=10.0,
                                deflection_max=math.radians(20.0))
    thrust, dm, dl, dn = limited.limit((99.0, 1.0, -1.0, 0.1))
    assert thrust == 10.0 and dm == pytest.approx(math.radians(20.0))
    assert dl == pytest.approx(-math.radians(20.0)) and dn == 0.1
    assert limited.limit((-5.0, 0, 0, 0))[0] == 0.0


def test_samples_outside_the_envelope_are_flagged(model):
    tight = NonlinearAircraft(model.aircraft, model.condition,
                              envelope=Envelope(alpha_max=math.radians(1.0)))
    pull = simulate_nonlinear(tight, 1.0, 0.005,
                              controller=lambda t, out, m: (m.thrust_trim, math.radians(-5), 0, 0))
    assert 0.0 < pull.fraction_outside_envelope < 1.0
    assert simulate_nonlinear(model, 1.0, 0.01).fraction_outside_envelope == 0.0


def test_quaternion_and_euler_angles_round_trip():
    angles = (0.4, -0.3, 2.0)
    assert euler_from_quaternion(*quaternion_from_euler(*angles)) == pytest.approx(angles)


def test_lateral_only_aircraft_is_rejected():
    with pytest.raises(ModelError, match="needs both"):
        NonlinearAircraft(*load_aircraft("data/aircraft/cs7_transport.toml"))


def test_halving_the_step_barely_changes_a_manoeuvre(model):
    def roll(t, out, m):
        return m.thrust_trim, 0.0, math.radians(-3.0) if t < 0.5 else 0.0, 0.0
    coarse = simulate_nonlinear(model, 2.0, 0.01, controller=roll)
    fine = simulate_nonlinear(model, 2.0, 0.005, controller=roll)
    assert coarse["phi"][-1] == pytest.approx(fine["phi"][-1], rel=1e-3)
