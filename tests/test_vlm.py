"""Vortex-lattice solver checked against theory, then against XFLR5 for DF1.

XFLR5 tolerances are illustrative, not adopted (docs/requirements.md).
"""

import math
from pathlib import Path

import numpy as np
import pytest

from src.aero.airfoil import AirfoilError, camber_line
from src.aero.derivatives import stability_derivatives
from src.aero.geometry import (Geometry, GeometryError, MassProperties, Section, Surface,
                               load_geometry)
from src.aero.lattice import build_lattice
from src.aero.vlm import VortexLattice, segment_velocity
from src.aircraft import load_aircraft
from src.aircraft.xflr5 import aircraft_toml, log_derivatives
from src.dynamics import longitudinal_state_space
from src.verify import load_reference, longitudinal_modal_metrics

DF1 = "data/geometry/df1_m1.toml"


def rectangular_wing(aspect_ratio, airfoil="NACA 0012", vertical=False):
    span = aspect_ratio
    sections = (Section(0.0, 1.0, 0.0, 0.0, 0.0, airfoil),
                Section(span / 2, 1.0, 0.0, 0.0, 0.0, airfoil))
    surface = Surface("wing", (0.0, 0.0, 0.0), 0.0, True, vertical, sections)
    mass = MassProperties(1.0, (0.25, 0.0, 0.0), 1.0, 1.0, 1.0, 0.0)
    return Geometry("wing", span, span, 1.0, 1.0, 9.81, 0.0, mass, (surface,))


def lift_and_drag(geometry, alpha_deg, mesh=(8, 16)):
    solver = VortexLattice(build_lattice(geometry, *mesh), geometry.mass.cg,
                           math.radians(alpha_deg))
    loads = solver.loads((1, 0, 0, 0, 0, 0))
    return -loads[2] / (0.5 * geometry.S), -loads[0] / (0.5 * geometry.S), loads


def test_long_straight_vortex_gives_gamma_over_2_pi_h():
    v = segment_velocity(np.array([[0.0, 0.0, 0.5]]), np.array([[-1e4, 0.0, 0.0]]),
                         np.array([[1e4, 0.0, 0.0]]))
    assert np.linalg.norm(v) == pytest.approx(1.0 / (2 * math.pi * 0.5), rel=1e-6)


def test_point_on_the_vortex_line_sees_no_velocity():
    v = segment_velocity(np.array([[0.3, 0.0, 0.0]]), np.zeros((1, 3)),
                         np.array([[1.0, 0.0, 0.0]]))
    assert np.all(v == 0.0)


def test_naca_camber_and_file_camber():
    assert camber_line("NACA 0012")(np.array([0.3, 0.7])) == pytest.approx(0.0)
    assert camber_line("NACA 2412")(0.4) == pytest.approx(0.02)
    fx = camber_line("FX 60-126")(np.linspace(0.05, 0.95, 10))
    assert fx.min() > 0.0 and 0.02 < fx.max() < 0.06
    with pytest.raises(AirfoilError, match="no file"):
        camber_line("Not An Airfoil")


@pytest.mark.parametrize("aspect_ratio", [4.0, 8.0])
def test_lift_curve_slope_of_a_flat_rectangular_wing(aspect_ratio):
    """Within 5% of the Helmbold lifting-surface estimate."""
    wing = rectangular_wing(aspect_ratio)
    slope = (lift_and_drag(wing, 2.0)[0] - lift_and_drag(wing, 0.0)[0]) / math.radians(2.0)
    helmbold = 2 * math.pi * aspect_ratio / (2 + math.sqrt(4 + aspect_ratio ** 2))
    assert slope == pytest.approx(helmbold, rel=0.05)


def test_induced_drag_is_close_to_the_elliptic_value():
    wing = rectangular_wing(6.0)
    CL, CDi, _ = lift_and_drag(wing, 4.0, mesh=(8, 24))
    assert CDi == pytest.approx(CL ** 2 / (math.pi * 6.0), rel=0.10)


def test_symmetric_flight_has_no_lateral_loads():
    _, _, loads = lift_and_drag(load_geometry(DF1), 2.0, mesh=(4, 8))
    assert loads[[1, 3, 5]] == pytest.approx(0.0, abs=1e-12)


def test_uncambered_wing_makes_no_lift_at_zero_incidence_and_cambered_one_does():
    assert lift_and_drag(rectangular_wing(6.0), 0.0)[0] == pytest.approx(0.0, abs=1e-12)
    assert lift_and_drag(rectangular_wing(6.0, "NACA 4412"), 0.0)[0] > 0.2


def test_isolated_fin_side_force_matches_low_aspect_ratio_theory():
    """A vertical surface in sideslip behaves like a wing at incidence."""
    fin = rectangular_wing(2.0, vertical=True)
    fin = Geometry(**{**fin.__dict__, "surfaces": (
        Surface("fin", (0, 0, 0), 0.0, False, True, fin.surfaces[0].sections),)})
    solver = VortexLattice(build_lattice(fin, 8, 16), fin.mass.cg, 0.0)
    dY = (solver.loads((1, 0.01, 0, 0, 0, 0))[1] - solver.loads((1, -0.01, 0, 0, 0, 0))[1]) / 0.02
    area, aspect = 1.0, 1.0      # half of the mirrored AR-2 wing
    slope = -dY / (0.5 * area)
    assert slope == pytest.approx(2 * math.pi * aspect / (2 + math.sqrt(4 + aspect ** 2)),
                                  rel=0.08)


@pytest.fixture(scope="module")
def df1():
    reference = log_derivatives(Path("data/aircraft/m1Log.rtf").read_text(errors="replace"))
    return stability_derivatives(load_geometry(DF1), 8, 16), reference


@pytest.mark.parametrize("key,tolerance", [
    ("CLa", 0.05), ("Cma", 0.08), ("Cmq", 0.08), ("CLq", 0.10), ("neutral_point", 0.03),
    ("V", 0.04), ("Clp", 0.05), ("Clr", 0.08), ("Cnb", 0.12), ("CYr", 0.12), ("Cnr", 0.20),
])
def test_df1_derivatives_against_xflr5(df1, key, tolerance):
    computed, reference = df1
    assert computed[key] == pytest.approx(reference[key], rel=tolerance)


def test_df1_trim_angle_within_a_quarter_degree_of_xflr5(df1):
    computed, reference = df1
    assert abs(computed["alpha_trim_deg"] - reference["alpha_trim_deg"]) < 0.25


def test_df1_static_stability_signs(df1):
    computed, _ = df1
    assert computed["Cma"] < 0 and computed["Cnb"] > 0 and computed["Clb"] < 0
    assert computed["Clp"] < 0 and computed["Cmq"] < 0 and computed["Cnr"] < 0


def test_df1_longitudinal_modes_from_geometry_against_xflr5(df1, tmp_path):
    """Short period and phugoid frequency follow from lift and moment slopes; damping of
    the phugoid depends on drag, which is an input, so it is not checked here."""
    computed, _ = df1
    path = tmp_path / "df1.toml"
    path.write_text(aircraft_toml(computed, 0.307, "test"))
    metrics = longitudinal_modal_metrics(longitudinal_state_space(*load_aircraft(path)))
    reference = {r["metric"]: r["value"]
                 for r in load_reference("data/reference/df1_m1_longitudinal.toml")}
    for key, tolerance in (("short_period_natural_frequency_rad_s", 0.06),
                           ("short_period_damping_ratio", 0.06),
                           ("phugoid_natural_frequency_rad_s", 0.06)):
        assert metrics[key] == pytest.approx(reference[key], rel=tolerance)


def test_refining_the_mesh_changes_the_answer_less_each_time():
    geometry = load_geometry(DF1)
    coarse, medium, fine = (stability_derivatives(geometry, *mesh)["CLa"]
                            for mesh in ((4, 8), (6, 12), (8, 16)))
    assert abs(fine - medium) < abs(medium - coarse)


def test_geometry_problems_are_listed(tmp_path):
    text = Path(DF1).read_text().replace("rho = 1.225", "rho = -1.0").replace(
        'airfoil = "FX 60-126"', "", 1)
    path = tmp_path / "bad.toml"
    path.write_text(text)
    with pytest.raises(GeometryError) as excinfo:
        load_geometry(path)
    message = str(excinfo.value)
    assert "'rho' must be > 0" in message and "missing required field 'airfoil'" in message
