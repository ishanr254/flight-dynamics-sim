"""Load and validate an aircraft data file (TOML)."""

import math
import tomllib
from dataclasses import fields
from pathlib import Path

from .model import Aircraft, FlightCondition, LateralCoefficients

REQUIRED = {
    "mass": ("m",),
    "reference": ("S", "l"),
    "inertia": ("A", "C"),
    "lateral": tuple(f.name for f in fields(LateralCoefficients)),
    "condition": ("V", "rho"),
}
OPTIONAL = {
    "inertia": {"E": 0.0},
    "condition": {"alpha": 0.0, "theta": 0.0},
}
STRICTLY_POSITIVE = (("mass", "m"), ("reference", "S"), ("reference", "l"),
                     ("inertia", "A"), ("inertia", "C"), ("condition", "V"),
                     ("condition", "rho"))


class AircraftDataError(ValueError):
    """Raised with every problem found in a data file, not just the first."""

    def __init__(self, path, problems):
        self.problems = list(problems)
        lines = "\n".join(f"  - {p}" for p in self.problems)
        super().__init__(f"{path}: {len(self.problems)} problem(s) in aircraft data\n{lines}")


def load_aircraft(path):
    """Return (Aircraft, FlightCondition) from a TOML file, or raise AircraftDataError."""
    path = Path(path)
    try:
        raw = tomllib.loads(path.read_text())
    except tomllib.TOMLDecodeError as exc:
        raise AircraftDataError(path, [f"not valid TOML: {exc}"]) from exc

    problems = []
    values = {}
    for section, keys in REQUIRED.items():
        table = raw.get(section)
        if not isinstance(table, dict):
            problems.append(f"missing section [{section}] (needs: {', '.join(keys)})")
            continue
        known = set(keys) | set(OPTIONAL.get(section, {}))
        for key in sorted(set(table) - known):
            problems.append(f"[{section}] unknown field '{key}' (check spelling)")
        merged = {**OPTIONAL.get(section, {}), **table}
        for key in known:
            if key not in merged:
                problems.append(f"[{section}] missing required field '{key}'")
            elif isinstance(merged[key], bool) or not isinstance(merged[key], (int, float)):
                problems.append(f"[{section}] '{key}' must be a number, got {merged[key]!r}")
            elif not math.isfinite(merged[key]):
                problems.append(f"[{section}] '{key}' must be finite, got {merged[key]!r}")
            else:
                values[(section, key)] = float(merged[key])

    for section, key in STRICTLY_POSITIVE:
        v = values.get((section, key))
        if v is not None and v <= 0:
            problems.append(f"[{section}] '{key}' must be > 0, got {v:g}")

    A, C, E = (values.get(("inertia", k)) for k in "ACE")
    if None not in (A, C, E) and A > 0 and C > 0 and E * E >= A * C:
        problems.append(
            f"[inertia] E^2 = {E * E:g} must be < A*C = {A * C:g}; "
            "this inertia tensor is not physically realizable")

    for key in ("alpha", "theta"):
        v = values.get(("condition", key))
        if v is not None and abs(v) > math.pi / 2:
            problems.append(
                f"[condition] '{key}' = {v:g} exceeds pi/2; angles must be in radians, "
                "is this value in degrees?")

    if problems:
        raise AircraftDataError(path, problems)

    def section(name):
        return {k: v for (s, k), v in values.items() if s == name}

    aircraft = Aircraft(
        name=str(raw.get("name", path.stem)),
        source=str(raw.get("source", "")),
        m=values[("mass", "m")],
        **section("reference"),
        **section("inertia"),
        lateral=LateralCoefficients(**section("lateral")),
    )
    return aircraft, FlightCondition(**section("condition"))
