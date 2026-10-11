import numpy as np
import pytest
from scipy.linalg import expm

from src.dynamics import StateSpace
from src.simulate import simulate

SYSTEM = StateSpace(A=np.array([[0.0, 1.0], [-4.0, -0.8]]), B=np.array([[0.0], [2.0]]),
                    states=("x", "v"), inputs=("f",))


def test_free_response_matches_matrix_exponential():
    response = simulate(SYSTEM, 10.0, x0={"x": 1.0}, n_samples=11)
    for t, x in zip(response.t, response.x):
        assert x == pytest.approx(expm(SYSTEM.A * t) @ [1.0, 0.0], abs=1e-6)


def test_step_response_settles_to_static_gain():
    response = simulate(SYSTEM, 60.0, u={"f": 3.0})
    steady = -np.linalg.solve(SYSTEM.A, SYSTEM.B @ [3.0])
    assert response.x[-1] == pytest.approx(steady, abs=1e-6)


def test_time_varying_input_is_applied():
    pulse = simulate(SYSTEM, 5.0, u=lambda t: np.array([1.0 if t < 1.0 else 0.0]))
    assert abs(pulse["x"]).max() > 0.0
    assert simulate(SYSTEM, 5.0)["x"] == pytest.approx(0.0)


def test_result_converges_as_tolerance_tightens():
    exact = expm(SYSTEM.A * 10.0) @ [1.0, 0.0]
    errors = [abs(simulate(SYSTEM, 10.0, x0={"x": 1.0}, rtol=tol, atol=tol * 1e-2).x[-1]
                  - exact).max() for tol in (1e-4, 1e-7, 1e-10)]
    assert errors[0] > errors[1] > errors[2]


def test_unknown_state_name_is_an_error():
    with pytest.raises(ValueError):
        simulate(SYSTEM, 1.0, x0={"nope": 1.0})
