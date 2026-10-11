"""Load and validate an aircraft data file (TOML). Format: docs/aircraft-file.md."""

import math
import tomllib
from dataclasses import fields
from pathlib import Path

from .model import Aircraft, FlightCondition, LateralCoefficients, LongitudinalCoefficients

# section -> (required fields, optional fields with defaults or None)
ALWAYS = {
    "mass": (("m",), {}),
    "reference": (("S", "l"), {}),
    "inertia": ((), {"A": None, "B": None, "C": None, "E": 0.0}),
    "condition": (("V", "rho"), {"alpha": 0.0, "theta": 0.0}),
}
MOTION = {
    "lateral": (tuple(f.name for f in fields(LateralCoefficients)), ("A", "C")),
    "longitudinal": (tuple(f.name for f in fields(LongitudinalCoefficients)), ("B",)),
}
STRICTLY_POSITIVE = (("mass", "m"), ("reference", "S"), ("reference", "l"),
                     ("inertia", "A"), ("inertia", "B"), ("inertia", "C"),
                     ("condition", "V"), ("condition", "rho"))


class AircraftDataError(ValueError):
    """Raised with every problem found in a data file, not just the first."""

    def __init__(self, path, problems):
        self.problems = list(problems)
        lines = "\n".join(f"  - {p}" for p in self.problems)
        super().__init__(f"{path}: {len(self.problems)} problem(s) in aircraft data\n{lines}")


def _read_section(raw, section, required, optional, problems):
    """Validated numbers of one section; missing optional values come back as given."""
    table = raw.get(section)
    if not isinstance(table, dict):
        if required:
            problems.append(f"missing section [{section}] (needs: {', '.join(required)})")
        table = {}
    known = set(required) | set(optional)
    for key in sorted(set(table) - known):
        problems.append(f"[{section}] unknown field '{key}' (check spelling)")
    values = {}
    for key in required + tuple(optional):
        if key not in table:
            if key in required and isinstance(raw.get(section), dict):
                problems.append(f"[{section}] missing required field '{key}'")
            elif key in optional:
                values[key] = optional[key]
            continue
        value = table[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            problems.append(f"[{section}] '{key}' must be a number, got {value!r}")
        elif not math.isfinite(value):
            problems.append(f"[{section}] '{key}' must be finite, got {value!r}")
        else:
            values[key] = float(value)
    return values


def load_aircraft(path):
    """Return (Aircraft, FlightCondition) from a TOML file, or raise AircraftDataError."""
    path = Path(path)
    try:
        raw = tomllib.loads(path.read_text())
    except tomllib.TOMLDecodeError as exc:
        raise AircraftDataError(path, [f"not valid TOML: {exc}"]) from exc

    problems = []
    data = {section: _read_section(raw, section, required, optional, problems)
            for section, (required, optional) in ALWAYS.items()}

    motions = {}
    for section, (required, needs_inertia) in MOTION.items():
        if section not in raw:
            continue
        motions[section] = _read_section(raw, section, required, {}, problems)
        for key in needs_inertia:
            if data["inertia"].get(key) is None:
                problems.append(f"[inertia] '{key}' is required when [{section}] is given")
    if not motions:
        problems.append("no [lateral] or [longitudinal] section: nothing to analyse")

    for section, key in STRICTLY_POSITIVE:
        v = data[section].get(key)
        if v is not None and v <= 0:
            problems.append(f"[{section}] '{key}' must be > 0, got {v:g}")

    A, C, E = (data["inertia"].get(k) for k in "ACE")
    if None not in (A, C, E) and A > 0 and C > 0 and E * E >= A * C:
        problems.append(
            f"[inertia] E^2 = {E * E:g} must be < A*C = {A * C:g}; "
            "this inertia tensor is not physically realizable")

    for key in ("alpha", "theta"):
        v = data["condition"].get(key)
        if v is not None and abs(v) > math.pi / 2:
            problems.append(
                f"[condition] '{key}' = {v:g} exceeds pi/2; angles must be in radians, "
                "is this value in degrees?")

    if problems:
        raise AircraftDataError(path, problems)

    aircraft = Aircraft(
        name=str(raw.get("name", path.stem)),
        source=str(raw.get("source", "")),
        m=data["mass"]["m"],
        **data["reference"],
        **data["inertia"],
        lateral=(LateralCoefficients(**motions["lateral"]) if "lateral" in motions else None),
        longitudinal=(LongitudinalCoefficients(**motions["longitudinal"])
                      if "longitudinal" in motions else None),
    )
    return aircraft, FlightCondition(**data["condition"])
