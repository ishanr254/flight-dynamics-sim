"""Level top speed and excess power from the DF1 flight log, against the model.

This is the one comparison of the performance model with measured flight data. The data
are thin (a few seconds of usable full-throttle flight), so tolerances are wide.
"""

import numpy as np
import pytest

from cases.df1_flight_top_speed import (binned, flight_samples, model_envelope, selected,
                                        top_speed_from)


@pytest.fixture(scope="module")
def samples():
    return flight_samples()


@pytest.fixture(scope="module")
def rows(samples):
    return binned(samples, selected(samples))


def test_the_fastest_point_of_the_flight_was_a_dive(samples):
    peak = int(np.argmax(samples["V"]))
    assert samples["V"][peak] == pytest.approx(36.4, abs=0.5)          # about 81 mph
    assert np.gradient(samples["h"], samples["t"])[peak] < -8.0        # descending fast
    assert samples["throttle"][peak] > 0.97


def test_usable_data_are_few_seconds_only(samples):
    keep = selected(samples)
    assert 4.0 < keep.sum() * 0.1 < 15.0


def test_excess_power_falls_with_speed_and_changes_sign(rows):
    medians = [r[1] for r in rows]
    assert medians[0] > 3.0 and medians[-1] < -1.0
    assert all(b < a + 1.0 for a, b in zip(medians, medians[1:]))


def test_level_top_speed_from_flight_matches_the_model(samples, rows):
    measured, _ = top_speed_from(rows)
    model = model_envelope(samples["rho"]).top_speed
    assert 27.5 < measured < 30.5
    assert model == pytest.approx(measured, rel=0.06)


def test_model_has_more_excess_power_at_low_speed_than_the_flight_shows(samples, rows):
    """Recorded discrepancy: about twice the flight's value at 23-25 m/s. Pins the direction
    so a change in the model or the analysis is noticed; it does not assert agreement."""
    envelope = model_envelope(samples["rho"])
    V, median = rows[0][0], rows[0][1]
    assert V < 24.0
    assert 1.5 < envelope.climb_rate(V) / median < 2.5
