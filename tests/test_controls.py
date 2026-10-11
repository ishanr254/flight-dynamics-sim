"""Control surfaces: mesh, sign conventions, theory, and the DF1 references.

Reference tolerances are illustrative, not adopted (docs/requirements.md).
"""

import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from cases.df1_controls import REFERENCE, all_moving_tail, control_metrics
from src.aero.derivatives import stability_derivatives
from src.aero.geometry import (Control, Geometry, GeometryError, MassProperties, Section,
                               Surface, load_geometry)
from src.aero.lattice import build_lattice, surface_mesh
from src.aero.vlm import VortexLattice
from src.aircraft import load_aircraft
from src.aircraft.xflr5 import aircraft_toml, log_derivatives
from src.dynamics import lateral_state_space, longitudinal_state_space
from src.simulate import simulate
from src.verify import compare

DF1 = "data/geometry/df1_m1.toml"
MESH = (8, 16)


@pytest.fixture(scope="module")
def geometry():
    return load_geometry(DF1)


@pytest.fixture(scope="module")
def derivatives(geometry):
    return stability_derivatives(geometry, *MESH)


def flapped_wing(aspect_ratio, chord_fraction):
    span = aspect_ratio
    sections = (Section(0.0, 1.0, 0.0, 0.0, 0.0, "NACA 0012"),
                Section(span / 2, 1.0, 0.0, 0.0, 0.0, "NACA 0012"))
    flap = Control("flap", "delta_m", chord_fraction, 0.0, span / 2)
    surface = Surface("wing", (0.0, 0.0, 0.0), 0.0, True, False, sections, (flap,))
    mass = MassProperties(1.0, (0.25, 0.0, 0.0), 1.0, 1.0, 1.0, 0.0)
    return Geometry("wing", span, span, 1.0, 1.0, 9.81, 0.0, mass, (surface,))


def test_mesh_has_edges_on_hinge_line_and_control_ends(geometry):
    wing = geometry.surfaces[0]
    _, xi, spans = surface_mesh(wing, 8, 16)
    assert np.isclose(xi, 0.75).any()                      # aileron hinge at 75% chord
    assert np.isclose(spans, 0.3157).any() and np.isclose(spans, 0.5260).any()


def test_only_panels_behind_the_hinge_and_inside_the_span_deflect(geometry):
    lattice = build_lattice(geometry, *MESH)
    moving = np.linalg.norm(lattice.deflection["delta_l"], axis=1) > 0
    assert moving.any() and not moving[lattice.surface != 0].any()
    x_over_c = lattice.control[moving, 0] / 0.307
    assert x_over_c.min() > 0.75
    assert np.abs(lattice.control[moving, 1]).min() > 0.3157
    assert np.abs(lattice.control[moving, 1]).max() < 0.5260


def test_flap_effectiveness_matches_thin_airfoil_theory():
    """CL_delta / CL_alpha of a full-span flap on a high-aspect-ratio wing."""
    fraction = 0.3
    wing = flapped_wing(10.0, fraction)
    solver = VortexLattice(build_lattice(wing, 16, 20), wing.mass.cg, 0.0)
    lift = lambda state, deflection=None: -solver.loads(state, 1.0, deflection)[2]  # noqa: E731
    dCL_dalpha = (lift((1, 0, 0.01, 0, 0, 0)) - lift((1, 0, -0.01, 0, 0, 0))) / 0.02
    dCL_ddelta = (lift((1, 0, 0, 0, 0, 0), {"delta_m": 0.01})
                  - lift((1, 0, 0, 0, 0, 0), {"delta_m": -0.01})) / 0.02
    theta = math.acos(2 * fraction - 1)
    theory = 1 - (theta - math.sin(theta)) / math.pi
    assert dCL_ddelta / dCL_dalpha == pytest.approx(theory, rel=0.06)


def test_signs_follow_the_documented_convention(derivatives):
    d = derivatives
    assert d["Zdelta_m"] < 0 and d["Mdelta_m"] < 0      # elevator down: lift up, nose down
    assert d["Ldelta_l"] < 0                            # positive aileron: roll to port
    assert d["Ydelta_n"] > 0 and d["Ndelta_n"] < 0      # positive rudder: nose to port


def test_symmetric_and_antisymmetric_controls_do_not_cross_couple(derivatives):
    d = derivatives
    scale = abs(d["Zdelta_m"])
    for key in ("Ydelta_m", "Ldelta_m", "Ndelta_m", "Xdelta_l", "Zdelta_l", "Mdelta_l",
                "Zdelta_n", "Mdelta_n"):
        assert abs(d[key]) < 1e-6 * scale, key


def test_all_moving_tail_matches_the_xflr5_log(geometry):
    d = stability_derivatives(all_moving_tail(geometry), *MESH)
    log = log_derivatives(Path("data/aircraft/m1Log.rtf").read_text(errors="replace"))
    qS = 0.5 * d["rho"] * d["V"] ** 2 * d["S"]
    assert d["Zdelta_m"] / qS == pytest.approx(log["CZde"], rel=0.06)
    assert d["Mdelta_m"] / (qS * d["c"]) == pytest.approx(log["CMde"], rel=0.06)


def test_control_derivatives_against_the_design_report(derivatives):
    for row in compare(control_metrics(derivatives), REFERENCE):
        assert abs(row.relative_difference) < 0.25, row.metric


def test_unknown_control_role_is_an_error(geometry):
    solver = VortexLattice(build_lattice(geometry, 4, 8), geometry.mass.cg, 0.0)
    with pytest.raises(KeyError, match="no control with role"):
        solver.loads((1, 0, 0, 0, 0, 0), 1.0, {"flaps": 0.1})


def test_bad_control_definitions_are_reported(tmp_path):
    text = Path(DF1).read_text()
    text = text.replace('role = "delta_m"', 'role = "elevon"').replace(
        "span = [0.3157, 0.526]", "span = [0.3157, 0.9]")
    path = tmp_path / "bad.toml"
    path.write_text(text)
    with pytest.raises(GeometryError) as excinfo:
        load_geometry(path)
    message = str(excinfo.value)
    assert "'role' must be one of" in message and "lies outside the surface" in message


@pytest.fixture(scope="module")
def aircraft(derivatives, tmp_path_factory):
    path = tmp_path_factory.mktemp("controls") / "df1.toml"
    path.write_text(aircraft_toml(derivatives, 0.307, "test"))
    return load_aircraft(path)


def test_aircraft_file_carries_all_three_controls(aircraft):
    ac, _ = aircraft
    assert ac.longitudinal.Cm_dm < 0 and ac.longitudinal.CZ_dm < 0
    assert ac.lateral.Cl_dl < 0 and ac.lateral.Cn_dn < 0 and ac.lateral.CY_dn > 0


def test_files_without_elevator_derivatives_get_a_zero_control_column():
    ss = longitudinal_state_space(*load_aircraft("data/aircraft/df1_m1.toml"))
    assert not ss.B.any()


def test_elevator_up_pitches_the_nose_up(aircraft):
    ss = longitudinal_state_space(*aircraft)
    response = simulate(ss, 0.5, u={"delta_m": math.radians(-1.0)})
    assert response["q"][5] > 0 and response["alpha"][-1] > 0
    assert not ss.B[:, ss.inputs.index("delta_x")].any()


def test_aileron_and_rudder_responses_have_the_right_sense(aircraft):
    ss = lateral_state_space(*aircraft)
    roll = simulate(ss, 0.3, u={"delta_l": math.radians(-1.0)})
    assert roll["p"][-1] > 0                      # negative aileron: roll to starboard
    yaw = simulate(ss, 0.2, u={"delta_n": math.radians(1.0)})
    assert yaw["r"][-1] < 0                       # positive rudder: nose to port


def test_steady_roll_rate_is_control_power_over_roll_damping(aircraft):
    ss = lateral_state_space(*aircraft)
    p, dl = ss.states.index("p"), ss.inputs.index("delta_l")
    delta = math.radians(-1.0)
    response = simulate(ss, 0.25, u={"delta_l": delta})
    assert response["p"].max() == pytest.approx(-ss.B[p, dl] / ss.A[p, p] * delta, rel=0.10)


def test_longitudinal_only_surface_changes_do_not_alter_uncontrolled_results(geometry):
    bare = replace(geometry, surfaces=tuple(replace(s, controls=()) for s in geometry.surfaces))
    with_controls = stability_derivatives(geometry, *MESH)
    without = stability_derivatives(bare, *MESH)
    for key in ("CLa", "Cma", "Clp", "Cnb"):
        assert with_controls[key] == pytest.approx(without[key], rel=0.02)
