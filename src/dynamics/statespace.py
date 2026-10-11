"""Linear state-space model dx/dt = A x + B u about an equilibrium."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class StateSpace:
    A: np.ndarray
    B: np.ndarray
    states: tuple
    inputs: tuple

    def __post_init__(self):
        n, m = len(self.states), len(self.inputs)
        if self.A.shape != (n, n) or self.B.shape != (n, m):
            raise ValueError(
                f"shape mismatch: A {self.A.shape}, B {self.B.shape} "
                f"for {n} states and {m} inputs")

    def closed_loop(self, K):
        """State space with u = K x + u_ext folded in (K is inputs x states)."""
        return StateSpace(self.A + self.B @ np.asarray(K, float), self.B,
                          self.states, self.inputs)
