"""Time-domain response of a StateSpace model via scipy.integrate.solve_ivp."""

from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp


@dataclass(frozen=True)
class Response:
    t: np.ndarray
    x: np.ndarray  # shape (len(t), n_states)
    states: tuple

    def __getitem__(self, state):
        return self.x[:, self.states.index(state)]


def simulate(ss, t_end, x0=None, u=None, n_samples=1001, rtol=1e-8, atol=1e-10):
    """Integrate from t=0 to t_end.

    x0: dict of state name -> initial value (unlisted states start at 0).
    u:  dict of input name -> constant value, or a callable u(t) returning the input vector.
    """
    x_init = np.zeros(len(ss.states))
    for name, value in (x0 or {}).items():
        x_init[ss.states.index(name)] = value

    if callable(u):
        u_of_t = u
    else:
        u_const = np.zeros(len(ss.inputs))
        for name, value in (u or {}).items():
            u_const[ss.inputs.index(name)] = value
        u_of_t = lambda t: u_const  # noqa: E731

    t_eval = np.linspace(0.0, t_end, n_samples)
    sol = solve_ivp(lambda t, x: ss.A @ x + ss.B @ u_of_t(t), (0.0, t_end), x_init,
                    t_eval=t_eval, method="RK45", rtol=rtol, atol=atol)
    if not sol.success:
        raise RuntimeError(f"integration failed: {sol.message}")
    return Response(sol.t, sol.y.T, ss.states)
