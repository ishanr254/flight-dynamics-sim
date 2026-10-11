"""Export flights as scenarios for the web viewer (viz/).

The viewer is a static page that must open straight from disk, where browsers do not
allow loading JSON with fetch. Scenarios are therefore written as one JavaScript file that
assigns a global, which a plain <script> tag can load from disk or from any web host.

Each scenario holds uniformly sampled columns. Attitude is exported as the body-to-earth
quaternion (north-east-down earth axes) so playback can interpolate through any attitude.
"""

import json
import math
from pathlib import Path

import numpy as np

from ..simulate.nonlinear import quaternion_from_euler

GLOBAL = "FD_SCENARIOS"
# column -> (decimals kept, value used when the source does not provide it)
COLUMNS = {
    "north": (2, 0.0), "east": (2, 0.0), "altitude": (2, 0.0),
    "q0": (5, 1.0), "q1": (5, 0.0), "q2": (5, 0.0), "q3": (5, 0.0),
    "V": (2, 0.0), "phi": (4, 0.0), "theta": (4, 0.0), "psi": (4, 0.0),
    "alpha": (4, 0.0), "beta": (4, 0.0), "p": (4, 0.0), "q": (4, 0.0), "r": (4, 0.0),
    "load_factor": (3, 1.0), "thrust": (2, 0.0),
    "delta_m": (4, 0.0), "delta_l": (4, 0.0), "delta_n": (4, 0.0),
    "valid": (0, 1.0), "overload": (0, 0.0),
}
FLAGS = ("valid", "overload")


def scenario_from_series(t, series, identifier, title, group, description, rate=30.0,
                         span=1.0, summary=(), events=(), ground=None, traces=(),
                         playback=1.0, exaggeration=None):
    """A viewer scenario from time series sampled at times t.

    series: column name -> array (see COLUMNS; missing ones take their default). psi must
    be continuous (unwrapped). span: wingspan in metres, used to size the aircraft model.
    summary: lines of text shown beside the animation. events: (time, label) pairs.
    ground: optional course markings. traces: up to three (column, label, unit) to plot,
    unit "deg" converting from radians. playback: suggested playback speed.
    exaggeration: how many times life size to draw the aircraft (None lets the page choose).
    """
    t = np.asarray(t, float)
    samples = np.arange(t[0], t[-1] + 1e-9, 1.0 / rate)
    columns = {}
    for name, (decimals, default) in COLUMNS.items():
        values = np.asarray(series[name], float) if name in series else np.full(len(t), default)
        resampled = np.interp(samples, t, values)
        if name in FLAGS:
            columns[name] = [int(round(v)) for v in resampled]
        else:
            columns[name] = [round(float(v), decimals) for v in resampled]
    quaternion = np.array([columns[f"q{i}"] for i in range(4)])
    quaternion /= np.linalg.norm(quaternion, axis=0)
    for i in range(4):
        columns[f"q{i}"] = [round(float(v), 5) for v in quaternion[i]]
    out = {
        "id": identifier, "title": title, "group": group, "description": description,
        "rate": rate, "duration": float(samples[-1] - samples[0]), "span": span,
        "summary": list(summary),
        "events": [[round(float(time - t[0]), 2), label] for time, label in events],
        "ground": ground or {},
        "traces": [list(trace) for trace in traces],
        "playback": playback,
        "columns": columns,
    }
    if exaggeration is not None:
        out["exaggeration"] = exaggeration
    return out


def scenario(trajectory, identifier, title, group, description, **kwargs):
    """A viewer scenario from a nonlinear Trajectory."""
    series = {name: trajectory[name] for name in COLUMNS if name in trajectory.data}
    series["psi"] = np.unwrap(trajectory["psi"])
    for i in range(4):
        series[f"q{i}"] = trajectory.x[:, 6 + i]
    return scenario_from_series(trajectory.t, series, identifier, title, group, description,
                                **kwargs)


def lateral_scenario(response, speed, identifier, title, group, description, altitude=1000.0,
                     controls=None, **kwargs):
    """A viewer scenario from a linear lateral Response (states beta, r, p, phi).

    The linear model has no position or heading. They are rebuilt by kinematics at constant
    speed and height: heading is the integral of the yaw rate, and the aircraft moves along
    heading plus sideslip. This is the small-disturbance picture.
    """
    t = response.t
    dt = np.diff(t, prepend=t[0])
    psi = np.cumsum(response["r"] * dt)
    track = psi + response["beta"]
    series = {
        "north": np.cumsum(speed * np.cos(track) * dt),
        "east": np.cumsum(speed * np.sin(track) * dt),
        "altitude": np.full(len(t), altitude), "V": np.full(len(t), speed),
        "phi": response["phi"], "psi": psi, "beta": response["beta"],
        "p": response["p"], "r": response["r"],
        **(controls or {}),
    }
    quaternions = np.array([quaternion_from_euler(phi, 0.0, heading)
                            for phi, heading in zip(response["phi"], psi)])
    for i in range(4):
        series[f"q{i}"] = quaternions[:, i]
    return scenario_from_series(t, series, identifier, title, group, description, **kwargs)


def write_scenarios(path, scenarios, generator):
    """Write scenarios as a script the viewer loads with a <script> tag."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    body = json.dumps(list(scenarios), separators=(",", ":"))
    path.write_text(f"// Generated by {generator}. Do not edit by hand.\n"
                    f"window.{GLOBAL} = {body};\n")
    return path
