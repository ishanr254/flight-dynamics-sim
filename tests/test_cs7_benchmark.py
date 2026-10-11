"""CS7 lateral benchmark. Tolerances are illustrative, not adopted (docs/requirements.md)."""

import pytest

from cases.cs7_lateral import AIRCRAFT, REFERENCE, computed_metrics
from src.aircraft import load_aircraft
from src.verify import compare

# References the case study states precisely enough for a tight check.
PRECISE = {"roll_time_constant_s", "roll_rate_steady_deg_s", "spiral_neutral_Cn_beta_factor",
           "spiral_phi_at_200s_deg"}
# References quoted as "around X"; only order-of-agreement is checked.
APPROXIMATE = {"dutch_roll_period_s", "dutch_roll_damping_ratio",
               "dutch_roll_damping_ratio_with_yaw_damper",
               "spiral_time_constant_with_yaw_damper_s"}
# CS7 quotes ~700 s, but its own Q4 figure (phi 30 -> 6 deg in 200 s) implies ~125 s.
KNOWN_INCONSISTENT = {"spiral_time_constant_s"}


@pytest.fixture(scope="module")
def rows():
    aircraft, condition = load_aircraft(AIRCRAFT)
    return {r.metric: r for r in compare(computed_metrics(aircraft, condition), REFERENCE)}


def test_every_reference_is_classified(rows):
    assert set(rows) == PRECISE | APPROXIMATE | KNOWN_INCONSISTENT


@pytest.mark.parametrize("metric", sorted(PRECISE))
def test_precise_references(rows, metric):
    assert abs(rows[metric].relative_difference) < 0.02


@pytest.mark.parametrize("metric", sorted(APPROXIMATE))
def test_approximate_references(rows, metric):
    assert abs(rows[metric].relative_difference) < 0.25


def test_quoted_spiral_time_constant_still_disagrees(rows):
    """Fails if the model or the reference changes, so the open question gets revisited."""
    assert rows["spiral_time_constant_s"].relative_difference < -0.5
