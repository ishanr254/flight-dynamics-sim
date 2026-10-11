"""The as-built DF1 geometry: the XFLR5 model with the fin at the design report's height."""

import math

import pytest

from src.aero.geometry import load_geometry
from src.aero.xflr5_plane import geometry_toml
from src.aircraft import load_aircraft
from src.dynamics import lateral_state_space, longitudinal_state_space
from src.verify import longitudinal_modal_metrics, modal_metrics

PLANE = "data/aircraft/Mission 1 Iteration 2.xml"
POLAR = "data/aircraft/Mission 1 Iteration 2_T7-VLM2-Elev(g1_0).xml"
REPORT_FIN_HEIGHT = 10.24 * 0.0254      # design report Table 5.1.1


def fin(path):
    return next(s for s in load_geometry(path).surfaces if s.name == "Fin")


def test_fin_heights_of_the_two_geometries():
    assert fin("data/geometry/df1_m1.toml").sections[-1].span == pytest.approx(0.133)
    assert fin("data/geometry/df1_m1_asbuilt.toml").sections[-1].span == pytest.approx(
        REPORT_FIN_HEIGHT, abs=1e-4)


def test_rudder_still_spans_the_whole_fin():
    surface = fin("data/geometry/df1_m1_asbuilt.toml")
    rudder = surface.controls[0]
    assert (rudder.span_from, rudder.span_to) == (0.0, surface.sections[-1].span)


def test_everything_but_the_fin_is_identical():
    a, b = load_geometry("data/geometry/df1_m1.toml"), load_geometry("data/geometry/df1_m1_asbuilt.toml")
    assert (a.S, a.b, a.c, a.mass, a.CD0) == (b.S, b.b, b.c, b.mass, b.CD0)
    assert a.surfaces[:2] == b.surfaces[:2]


def test_override_is_recorded_and_validated(tmp_path):
    text = geometry_toml(PLANE, POLAR, tip_spans={"Fin": 0.2601}, note="why")
    assert "Fin: tip span set to 0.2601 m" in text and "# why" in text
    with pytest.raises(ValueError, match="not in"):
        geometry_toml(PLANE, POLAR, tip_spans={"Winglet": 0.1})
    with pytest.raises(ValueError, match="must exceed"):
        geometry_toml(PLANE, POLAR, tip_spans={"Fin": 0.0})


def test_taller_fin_raises_dutch_roll_frequency_and_leaves_pitch_alone():
    built = load_aircraft("data/aircraft/df1_m1_asbuilt.toml")
    short = load_aircraft("data/aircraft/df1_m1_vlm.toml")       # XFLR5 geometry
    frequency = modal_metrics(lateral_state_space(*built))["dutch_roll_natural_frequency_rad_s"]
    assert frequency / (2 * math.pi) == pytest.approx(2.03, abs=0.05)
    assert built[0].lateral.Cn_beta > 2.5 * short[0].lateral.Cn_beta
    assert built[0].lateral.Cn_dn < 2.5 * short[0].lateral.Cn_dn < 0
    assert built[0].longitudinal.Cm_alpha == pytest.approx(short[0].longitudinal.Cm_alpha, rel=0.01)
    pitch = longitudinal_modal_metrics(longitudinal_state_space(*built))
    assert pitch["short_period_damping_ratio"] == pytest.approx(0.59, abs=0.02)
