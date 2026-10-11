"""Convert an XFLR5 stability-analysis log into this tool's aircraft and reference files.

    python3 -m src.aircraft.xflr5 LOG --chord 0.307 --aircraft OUT.toml \
        [--reference REF.toml | --spread DISTRIBUTION.toml]

XFLR5 reports lateral coefficients with moments referred to the span b and rates per
p*b/(2V); this tool uses one reference length l and rates per p*l/V. With k = b/l:
    static moment coefficients  x k          rate moment coefficients  x k^2/2
    static force coefficients   unchanged    rate force coefficients   x k/2
Derivatives and inertias are taken in XFLR5's stability axes, so alpha = theta = 0.
--spread FILE adds the inertia of components the XFLR5 model lumps into point masses
(see src.aircraft.mass); the result no longer corresponds to XFLR5's own eigenvalues.
The log (.txt or the .rtf macOS produces on paste) is read as-is; no RTF tools needed.
"""

import argparse
import math
import re
from pathlib import Path

from .mass import load_spread, spread_increment, to_stability_axes

NUMBER = r"(-?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?)"
SCALARS = {
    "mass": rf"Mass=\s*{NUMBER}\s*kg",
    "rho": rf"Density\s*=\s*{NUMBER}\s*kg/m3",
    "S": rf"Reference Area\s*=\s*{NUMBER}",
    "b": rf"Reference length\s*=\s*{NUMBER}",
    "V": rf"VInf\s*=\s*{NUMBER}\s*m/s",
    "alpha_trim_deg": rf"zero-moment angle\.\.\.\s*\S*?=\s*{NUMBER}",
    "Ibxx": rf"Ibxx=\s*{NUMBER}",
    "Ibzz": rf"Ibzz=\s*{NUMBER}",
    "Ibxz": rf"Ibxz=\s*{NUMBER}",
    "Ibyy": rf"Ibyy=\s*{NUMBER}",
    "Isxx": rf"Isxx=\s*{NUMBER}",
    "Iszz": rf"Iszz=\s*{NUMBER}",
    "Isxz": rf"Isxz=\s*{NUMBER}",
}
LATERAL = ("CYb", "CYp", "CYr", "Clb", "Clp", "Clr", "Cnb", "Cnp", "Cnr")
LONGITUDINAL = ("Xu", "Xw", "Zu", "Zw", "Zq", "Mu", "Mw", "Mq")   # dimensional, SI


class XflrLogError(ValueError):
    pass


def clean(raw):
    """Undo the RTF wrapping macOS adds, leaving plain log lines."""
    raw = raw.replace("\\'b2", "²").replace("\\'b0", "°")
    raw = re.sub(r"\\uc0\\u945 ?", "α", raw)
    return "\n".join(line.rstrip().rstrip("\\").rstrip() for line in raw.splitlines())


def parse_log(raw):
    text = clean(raw)
    missing, out = [], {}
    for key, pattern in SCALARS.items():
        match = re.search(pattern, text)
        if match:
            out[key] = float(match.group(1))
        else:
            missing.append(key)
    for key in LATERAL + LONGITUDINAL:
        match = re.search(rf"\b{key}=\s*{NUMBER}", text)
        if match:
            out[key] = float(match.group(1))
        else:
            missing.append(key)

    name = re.search(r"Launching the 3D Panel Analysis\.+\s*\n(.+)", text)
    out["name"] = name.group(1).strip() if name else "unnamed"
    version = re.search(r"xflr5 v[\d.]+", text)
    out["version"] = version.group(0) if version else "xflr5"

    for motion in ("Lateral", "Longitudinal"):
        key = f"{motion.lower()}_eigenvalues"
        modes = re.search(rf"___{motion} modes____\s*\n\s*Eigenvalue:(.+)", text)
        out[key] = [complex(float(re_), float(im)) for re_, im in
                    re.findall(rf"{NUMBER}\+\s*{NUMBER}i", modes.group(1))] if modes else []
        if len(out[key]) != 4:
            missing.append(f"{motion.lower()} eigenvalues")

    if missing:
        raise XflrLogError("not a complete XFLR5 stability log; could not find: "
                           + ", ".join(missing))
    return out


def course_lateral(log, chord):
    """XFLR5 lateral coefficients re-expressed in the l/V, single-length convention."""
    k = log["b"] / chord
    return {
        "CY_beta": log["CYb"], "CY_p": log["CYp"] * k / 2, "CY_r": log["CYr"] * k / 2,
        "Cl_beta": log["Clb"] * k, "Cl_p": log["Clp"] * k * k / 2,
        "Cl_r": log["Clr"] * k * k / 2,
        "Cn_beta": log["Cnb"] * k, "Cn_p": log["Cnp"] * k * k / 2,
        "Cn_r": log["Cnr"] * k * k / 2,
    }


def course_longitudinal(log, chord):
    """Dimensional stability-axis derivatives as this tool's longitudinal coefficients."""
    V = log["V"]
    qS = 0.5 * log["rho"] * V ** 2 * log["S"]
    return {
        "CX_u": log["Xu"] * V / qS, "CX_alpha": log["Xw"] * V / qS,
        "CZ_u": log["Zu"] * V / qS, "CZ_alpha": log["Zw"] * V / qS,
        "CZ_q": log["Zq"] * V / (qS * chord),
        "Cm_u": log["Mu"] * V / (qS * chord), "Cm_alpha": log["Mw"] * V / (qS * chord),
        "Cm_q": log["Mq"] * V / (qS * chord ** 2),
    }


def inertia(log, spread=None):
    """(A, B, C, E) in stability axes: the source's own, or body-axis values plus a spread.

    Pitch inertia B is about the y axis, which the rotation to stability axes leaves alone.
    """
    if not spread:
        return log["Isxx"], log["Ibyy"], log["Iszz"], log["Isxz"]
    dxx, dyy, dzz = spread_increment(spread)
    A, C, E = to_stability_axes(log["Ibxx"] + dxx, log["Ibzz"] + dzz, log["Ibxz"],
                                math.radians(log["alpha_trim_deg"]))
    return A, log["Ibyy"] + dyy, C, E


def log_derivatives(raw):
    """Every 'name= value' pair in the log (dimensional and non-dimensional derivatives)."""
    text = clean(raw)
    out = {name: float(value) for name, value in
           re.findall(rf"\b([A-Za-z]\w*)=\s*{NUMBER}", text)}
    for key, pattern in (("alpha_trim_deg", SCALARS["alpha_trim_deg"]), ("V", SCALARS["V"]),
                         ("neutral_point", rf"Neutral Point position=\s*{NUMBER}")):
        out[key] = float(re.search(pattern, text).group(1))
    return out


def aircraft_toml(log, chord, source, spread=None, spread_source="",
                  generator="src.aircraft.xflr5"):
    c = course_lateral(log, chord)
    A, B, C, E = inertia(log, spread)
    lon = course_longitudinal(log, chord)
    longitudinal = "\n".join(f"{key} = {value!r}" for key, value in lon.items())
    name = log["name"] + (" (distributed mass)" if spread else "")
    inertia_note = (
        f"# XFLR5 body-axis inertia plus the component extents in {spread_source}:\n"
        f"# {', '.join(e['component'] for e in spread)}. XFLR5 gave A = {log['Isxx']:g}, "
        f"B = {log['Ibyy']:g}, C = {log['Iszz']:g}, E = {log['Isxz']:g}.\n" if spread else "")
    order = ("CY_beta", "CY_p", "CY_r", "Cl_beta", "Cl_p", "Cl_r", "Cn_beta", "Cn_p", "Cn_r")
    coefficients = "\n".join(f"{key} = {c[key]!r}" for key in order)
    return f"""# Generated by {generator} from {source} ({log['version']}). Do not edit by hand;
# re-run the generator. Stability axes; trim alpha was {log['alpha_trim_deg']:g} deg.

name = "{name}"
source = "{log['version']}, {source}"

[mass]
m = {log['mass']!r}

[reference]
S = {log['S']!r}
l = {chord!r}          # mean chord; XFLR5 span reference was {log['b']:g} m

[inertia]
{inertia_note}A = {A!r}
B = {B!r}
C = {C!r}
E = {E!r}

[lateral]
{coefficients}
# Aileron and rudder derivatives are not available from this source.
# Zero means "not available", not "no control authority".
CY_dl = 0.0
CY_dn = 0.0
Cl_dl = 0.0
Cl_dn = 0.0
Cn_dl = 0.0
Cn_dn = 0.0

# Speed derivatives are of the whole force, so CZ_u is about -2*CL at trim.
# Elevator and throttle derivatives are not part of the file yet.
[longitudinal]
{longitudinal}

[condition]
V = {log['V']!r}
rho = {log['rho']!r}
alpha = 0.0
theta = 0.0
"""


def reference_toml(log, aircraft_path, source):
    values = sorted(log["lateral_eigenvalues"], key=abs, reverse=True)
    real = [v for v in values if v.imag == 0.0]
    pair = [v for v in values if v.imag > 0.0]
    if len(real) != 2 or len(pair) != 1:
        raise XflrLogError("lateral eigenvalues are not roll + Dutch roll + spiral")
    roll, spiral, dutch = real[0], real[1], pair[0]
    metrics = {
        "roll_time_constant_s": -1.0 / roll.real,
        "dutch_roll_period_s": 2.0 * math.pi / dutch.imag,
        "dutch_roll_damping_ratio": -dutch.real / abs(dutch),
        "dutch_roll_natural_frequency_rad_s": abs(dutch),
        "spiral_time_constant_s": -1.0 / spiral.real,
    }
    kind = f"{log['version']} lateral eigenvalues (log prints 4 significant figures)"
    entries = "\n".join(
        f'[[reference]]\nmetric = "{name}"\nvalue = {value!r}\nkind = "{kind}"\n'
        f'source = "{source}"\n' for name, value in metrics.items())
    return f'# Generated by src.aircraft.xflr5 from {source}.\n\naircraft = "{aircraft_path}"\n\n{entries}'


def longitudinal_reference_toml(log, aircraft_path, source):
    pairs = sorted((v for v in log["longitudinal_eigenvalues"] if v.imag > 0.0),
                   key=abs, reverse=True)
    if len(pairs) != 2:
        raise XflrLogError("longitudinal eigenvalues are not short period + phugoid")
    short, phugoid = pairs
    metrics = {
        "short_period_natural_frequency_rad_s": abs(short),
        "short_period_damping_ratio": -short.real / abs(short),
        "phugoid_natural_frequency_rad_s": abs(phugoid),
        "phugoid_period_s": 2.0 * math.pi / phugoid.imag,
        "phugoid_damping_ratio": -phugoid.real / abs(phugoid),
    }
    kind = f"{log['version']} longitudinal eigenvalues (log prints 4 significant figures)"
    entries = "\n".join(
        f'[[reference]]\nmetric = "{name}"\nvalue = {value!r}\nkind = "{kind}"\n'
        f'source = "{source}"\n' for name, value in metrics.items())
    return f'# Generated by src.aircraft.xflr5 from {source}.\n\naircraft = "{aircraft_path}"\n\n{entries}'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("log")
    parser.add_argument("--chord", type=float, required=True,
                        help="reference length l in metres (mean chord)")
    parser.add_argument("--aircraft", required=True, help="aircraft TOML to write")
    parser.add_argument("--reference", help="lateral reference-values TOML to write")
    parser.add_argument("--reference-longitudinal",
                        help="longitudinal reference-values TOML to write")
    parser.add_argument("--spread", help="mass-distribution TOML to add to the inertia")
    args = parser.parse_args(argv)
    if args.spread and (args.reference or args.reference_longitudinal):
        parser.error("references hold XFLR5's eigenvalues, which do not apply with --spread")

    log = parse_log(Path(args.log).read_text(errors="replace"))
    spread = load_spread(args.spread) if args.spread else None
    Path(args.aircraft).write_text(
        aircraft_toml(log, args.chord, args.log, spread, args.spread or ""))
    if args.reference:
        Path(args.reference).write_text(reference_toml(log, args.aircraft, args.log))

    if args.reference_longitudinal:
        Path(args.reference_longitudinal).write_text(
            longitudinal_reference_toml(log, args.aircraft, args.log))


if __name__ == "__main__":
    main()
