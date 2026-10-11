import math

import numpy as np
import pytest

from src.analysis import modes, name_lateral_modes


def oscillator(wn, zeta):
    return np.array([[0.0, 1.0], [-wn ** 2, -2.0 * zeta * wn]])


def test_damped_oscillator_recovers_wn_and_zeta():
    (mode,) = modes(oscillator(wn=2.0, zeta=0.3))
    assert mode.oscillatory and mode.stable
    assert mode.wn == pytest.approx(2.0)
    assert mode.zeta == pytest.approx(0.3)
    assert mode.wd == pytest.approx(2.0 * math.sqrt(1 - 0.3 ** 2))
    assert mode.period == pytest.approx(2 * math.pi / mode.wd)


def test_unstable_oscillator_has_negative_damping():
    (mode,) = modes(oscillator(wn=1.5, zeta=-0.1))
    assert not mode.stable
    assert mode.zeta == pytest.approx(-0.1)


def test_aperiodic_modes_time_constant_sign_convention():
    fast, slow = modes(np.diag([-4.0, 0.05]))
    assert not fast.oscillatory and fast.stable
    assert fast.time_constant == pytest.approx(0.25)
    assert fast.time_to_half_or_double == pytest.approx(math.log(2) / 4.0)
    assert not slow.stable
    assert slow.time_constant == pytest.approx(-20.0)
    assert slow.wn is None and slow.zeta is None


def test_lateral_naming_by_speed():
    A = np.zeros((4, 4))
    A[:2, :2] = oscillator(wn=1.0, zeta=0.2)
    A[2, 2], A[3, 3] = -3.0, -0.01
    named = name_lateral_modes(modes(A))
    assert named["roll"].eigenvalue.real == pytest.approx(-3.0)
    assert named["spiral"].eigenvalue.real == pytest.approx(-0.01)
    assert named["dutch_roll"].wn == pytest.approx(1.0)


def test_lateral_naming_falls_back_when_pattern_absent():
    named = name_lateral_modes(modes(np.diag([-1.0, -2.0, -3.0, -4.0])))
    assert set(named) == {"mode_1", "mode_2", "mode_3", "mode_4"}
