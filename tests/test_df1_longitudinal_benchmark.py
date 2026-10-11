"""DF1 vs XFLR5's own longitudinal eigenvalues. The log prints 4 significant figures."""

import pytest

from cases.df1_longitudinal import MISSIONS, paths
from src.aircraft import load_aircraft
from src.dynamics import longitudinal_state_space
from src.verify import compare, longitudinal_modal_metrics


@pytest.mark.parametrize("mission", MISSIONS)
def test_modes_match_xflr5(mission):
    aircraft_path, reference_path = paths(mission)
    ss = longitudinal_state_space(*load_aircraft(aircraft_path))
    for row in compare(longitudinal_modal_metrics(ss), reference_path):
        assert abs(row.relative_difference) < 2e-3, row.metric
