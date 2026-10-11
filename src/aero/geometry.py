"""Aircraft geometry and mass properties (TOML). Axes: x aft, y to starboard, z up; SI."""

import math
import tomllib
from dataclasses import dataclass
from pathlib import Path


class GeometryError(ValueError):
    def __init__(self, path, problems):
        self.problems = list(problems)
        lines = "\n".join(f"  - {p}" for p in self.problems)
        super().__init__(f"{path}: {len(self.problems)} problem(s) in geometry\n{lines}")


@dataclass(frozen=True)
class Section:
    span: float           # distance along the surface from its root
    chord: float
    x_offset: float       # leading-edge sweep-back relative to the surface position
    dihedral_deg: float   # of the panel outboard of this section
    twist_deg: float      # leading edge up, about the leading edge
    airfoil: str


@dataclass(frozen=True)
class Surface:
    name: str
    position: tuple
    incidence_deg: float
    symmetric: bool       # mirrored about the x-z plane
    vertical: bool        # sections run along +z instead of +y (a fin)
    sections: tuple


@dataclass(frozen=True)
class MassProperties:
    m: float
    cg: tuple
    Ixx: float
    Iyy: float
    Izz: float
    Ixz: float


@dataclass(frozen=True)
class Geometry:
    name: str
    S: float
    b: float
    c: float
    rho: float
    g: float
    CD0: float
    mass: MassProperties
    surfaces: tuple


def _number(table, key, where, problems, default=None, positive=False):
    value = table.get(key, default)
    if value is None:
        problems.append(f"{where} missing required field '{key}'")
        return math.nan
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        problems.append(f"{where} '{key}' must be a number, got {value!r}")
        return math.nan
    if positive and value <= 0:
        problems.append(f"{where} '{key}' must be > 0, got {value:g}")
    return float(value)


def _vector(table, key, where, problems):
    value = table.get(key)
    if (not isinstance(value, list) or len(value) != 3
            or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value)):
        problems.append(f"{where} '{key}' must be [x, y, z] in metres")
        return (math.nan,) * 3
    return tuple(float(v) for v in value)


def load_geometry(path):
    path = Path(path)
    try:
        raw = tomllib.loads(path.read_text())
    except tomllib.TOMLDecodeError as exc:
        raise GeometryError(path, [f"not valid TOML: {exc}"]) from exc
    problems = []

    ref = raw.get("reference", {})
    S = _number(ref, "S", "[reference]", problems, positive=True)
    b = _number(ref, "b", "[reference]", problems, positive=True)
    c = _number(ref, "c", "[reference]", problems, positive=True)

    cond = raw.get("condition", {})
    rho = _number(cond, "rho", "[condition]", problems, positive=True)
    g = _number(cond, "g", "[condition]", problems, default=9.81, positive=True)
    CD0 = _number(raw.get("drag", {}), "CD0", "[drag]", problems, default=0.0)

    mt = raw.get("mass", {})
    mass = MassProperties(
        m=_number(mt, "m", "[mass]", problems, positive=True),
        cg=_vector(mt, "cg", "[mass]", problems),
        Ixx=_number(mt, "Ixx", "[mass]", problems, positive=True),
        Iyy=_number(mt, "Iyy", "[mass]", problems, positive=True),
        Izz=_number(mt, "Izz", "[mass]", problems, positive=True),
        Ixz=_number(mt, "Ixz", "[mass]", problems, default=0.0))

    surfaces = []
    if not raw.get("surface"):
        problems.append("no [[surface]] defined")
    for st in raw.get("surface", []):
        name = str(st.get("name", f"surface {len(surfaces) + 1}"))
        where = f"surface '{name}'"
        sections = []
        for i, sec in enumerate(st.get("section", [])):
            sw = f"{where} section {i + 1}"
            if not isinstance(sec.get("airfoil"), str):
                problems.append(f"{sw} missing required field 'airfoil'")
            sections.append(Section(
                span=_number(sec, "span", sw, problems),
                chord=_number(sec, "chord", sw, problems, positive=True),
                x_offset=_number(sec, "x_offset", sw, problems, default=0.0),
                dihedral_deg=_number(sec, "dihedral_deg", sw, problems, default=0.0),
                twist_deg=_number(sec, "twist_deg", sw, problems, default=0.0),
                airfoil=str(sec.get("airfoil", ""))))
        if len(sections) < 2:
            problems.append(f"{where} needs at least two sections (root and tip)")
        elif any(b_.span <= a.span for a, b_ in zip(sections, sections[1:])):
            problems.append(f"{where} section 'span' values must increase from root to tip")
        surfaces.append(Surface(
            name=name,
            position=_vector(st, "position", where, problems),
            incidence_deg=_number(st, "incidence_deg", where, problems, default=0.0),
            symmetric=bool(st.get("symmetric", True)),
            vertical=bool(st.get("vertical", False)),
            sections=tuple(sections)))

    if problems:
        raise GeometryError(path, problems)
    return Geometry(name=str(raw.get("name", path.stem)), S=S, b=b, c=c, rho=rho, g=g,
                    CD0=CD0, mass=mass, surfaces=tuple(surfaces))
