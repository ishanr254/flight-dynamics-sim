"""Scaffold smoke test — confirms the test runner and core deps are wired up."""

import numpy
import scipy


def test_deps_importable():
    assert numpy.__version__
    assert scipy.__version__
