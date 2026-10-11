"""Performance data for one aircraft configuration (data/performance/*.toml)."""

import math
import tomllib
from dataclasses import dataclass
from pathlib import Path

from ..propulsion import Battery, Motor, Powertrain, load_propeller


class PerformanceDataError(ValueError):
    pass


@dataclass(frozen=True)
class Performance:
    name: str
    CD0: float
    k: float                    # induced-drag factor: CD = CD0 + k CL^2
    CL_max: float
    load_factor_limit: float
    never_exceed_speed: float
    powertrain: Powertrain
    reference: tuple            # entries of the file's [[reference]] list


def load_performance(path, rho):
    path = Path(path)
    raw = tomllib.loads(path.read_text())
    try:
        aero, structure = raw["aerodynamics"], raw["structure"]
        motor, battery = raw["motor"], raw["battery"]
        k = 1.0 / (math.pi * aero["aspect_ratio"] * aero["oswald"])
        pack = Battery(int(battery["cells"]), float(battery["capacity_ah"]),
                       float(battery["c_rating"]), float(battery.get("usable_fraction", 0.85)))
        powertrain = Powertrain(
            Motor(float(motor["kv"]), float(motor["rated_power"])), pack,
            load_propeller(raw["propeller"]["file"]), rho,
            min(float(raw["limits"]["fuse_current"]), pack.max_current),
            float(raw.get("calibration", {}).get("thrust_factor", 1.0)))
        performance = Performance(
            name=raw.get("name", path.stem), CD0=float(aero["CD0"]), k=k,
            CL_max=float(aero["CL_max"]),
            load_factor_limit=float(structure["load_factor_limit"]),
            never_exceed_speed=float(structure["never_exceed_speed"]),
            powertrain=powertrain, reference=tuple(raw.get("reference", [])))
    except KeyError as exc:
        raise PerformanceDataError(f"{path}: missing field {exc}") from exc
    for label, value in (("CD0", performance.CD0), ("CL_max", performance.CL_max),
                         ("load_factor_limit", performance.load_factor_limit - 1.0),
                         ("never_exceed_speed", performance.never_exceed_speed)):
        if not value > 0:
            raise PerformanceDataError(f"{path}: '{label}' is out of range")
    return performance
