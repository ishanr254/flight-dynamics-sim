"""Import an XFLR5 plane definition (and its analysis settings) as a geometry file.

    python3 -m src.aero.xflr5_plane PLANE.xml POLAR.xml -o GEOMETRY.toml [--controls FILE]
        [--tip-span "Fin=0.2601"]

The plane XML supplies the lifting surfaces; the polar XML supplies reference dimensions,
air density, mass, centre of gravity and inertia. XFLR5 plane files carry no control
surfaces; --controls merges them in from a separate TOML file. --tip-span replaces the
span position of a named surface's last section, for correcting a dimension in the XFLR5
model without editing it. This is a convenience for existing XFLR5 models; geometry files
can equally be written by hand.
"""

import argparse
import tomllib
import xml.etree.ElementTree as ET
from pathlib import Path


def _floats(text):
    return [float(v) for v in text.replace(",", " ").split()]


def geometry_toml(plane_path, polar_path, cd0=0.0, controls_path=None, tip_spans=None,
                  note=""):
    """tip_spans: {surface name: span position of its last section, m}, overriding the file.
    note: a line recorded in the header, e.g. why a dimension was overridden."""
    tip_spans = dict(tip_spans or {})
    plane = ET.parse(plane_path).getroot().find("Plane")
    polar = ET.parse(polar_path).getroot().find("Polar")
    inertia = polar.find("Inertia_Value")
    get = lambda node, tag: float(node.find(tag).text)  # noqa: E731

    controls = (tomllib.loads(Path(controls_path).read_text()).get("control", [])
                if controls_path else [])
    surface_names = {wing.find("Name").text.strip() for wing in plane.findall("wing")}
    for name in tip_spans:
        if name not in surface_names:
            raise ValueError(f"--tip-span names surface {name!r}, which is not in {plane_path}")
    for control in controls:
        if control.get("surface") not in surface_names:
            raise ValueError(f"{controls_path}: control '{control.get('name')}' names surface "
                             f"{control.get('surface')!r}, which is not in {plane_path}")

    lines = [
        f"# Imported by src.aero.xflr5_plane from {plane_path} and {polar_path}."
        + (f"\n# Control surfaces from {controls_path}." if controls_path else "")
        + "".join(f"\n# {name}: tip span set to {value:g} m, not the XFLR5 file's value."
                  for name, value in tip_spans.items())
        + (f"\n# {note}" if note else ""),
        "# Axes: x aft, y to starboard, z up. SI units.",
        "",
        f'name = "{plane.find("Name").text.strip()}"',
        "",
        "[reference]",
        f"S = {get(polar, 'Reference_Area')!r}",
        f"b = {get(polar, 'Reference_Span_Length')!r}",
        f"c = {get(polar, 'Reference_Chord_Length')!r}",
        "",
        "[condition]",
        f"rho = {get(polar, 'Density')!r}",
        "",
        "[drag]",
        f"CD0 = {cd0!r}   # zero-lift (viscous) drag coefficient; the lattice cannot predict it",
        "",
        "[mass]",
        f"m = {get(inertia, 'Mass')!r}",
        f"cg = {_floats(inertia.find('CoG').text)!r}",
        f"Ixx = {get(inertia, 'CoG_Ixx')!r}",
        f"Iyy = {get(inertia, 'CoG_Iyy')!r}",
        f"Izz = {get(inertia, 'CoG_Izz')!r}",
        f"Ixz = {get(inertia, 'CoG_Ixz')!r}",
    ]
    for wing in plane.findall("wing"):
        fin = wing.find("isFin").text.strip() == "true"
        if fin and (wing.find("isDoubleFin").text.strip() == "true"
                    or wing.find("isSymFin").text.strip() == "true"):
            raise ValueError(f"{plane_path}: double or symmetric fins are not supported yet")
        lines += [
            "",
            "[[surface]]",
            f'name = "{wing.find("Name").text.strip()}"',
            f"position = {_floats(wing.find('Position').text)!r}",
            f"incidence_deg = {get(wing, 'Tilt_angle')!r}",
            f"symmetric = {'false' if fin else wing.find('Symetric').text.strip()}",
            f"vertical = {'true' if fin else 'false'}",
        ]
        name = wing.find("Name").text.strip()
        sections = list(wing.find("Sections"))
        spans = [get(section, "y_position") for section in sections]
        if name in tip_spans:
            if tip_spans[name] <= spans[-2]:
                raise ValueError(f"--tip-span for {name!r} must exceed {spans[-2]:g} m")
            spans[-1] = tip_spans[name]
        for section, span_position in zip(sections, spans):
            lines += [
                "  [[surface.section]]",
                f"  span = {span_position!r}",
                f"  chord = {get(section, 'Chord')!r}",
                f"  x_offset = {get(section, 'xOffset')!r}",
                f"  dihedral_deg = {get(section, 'Dihedral')!r}",
                f"  twist_deg = {get(section, 'Twist')!r}",
                f'  airfoil = "{section.find("Right_Side_FoilName").text.strip()}"',
            ]
        for control in controls:
            if control["surface"] != name:
                continue
            span = [spans[0], spans[-1]] if control["span"] == "full" else control["span"]
            lines += [
                "  [[surface.control]]",
                f'  name = "{control["name"]}"',
                f'  role = "{control["role"]}"',
                f"  chord_fraction = {control['chord_fraction']!r}",
                f"  span = {[float(v) for v in span]!r}",
            ]
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("plane")
    parser.add_argument("polar")
    parser.add_argument("-o", "--output", required=True)
    parser.add_argument("--cd0", type=float, default=0.0,
                        help="zero-lift drag coefficient to record (default 0)")
    parser.add_argument("--controls", help="TOML file of control surfaces to merge in")
    parser.add_argument("--tip-span", action="append", default=[], metavar="NAME=METRES",
                        help="override the span position of a surface's last section")
    parser.add_argument("--note", default="", help="a line to record in the file header")
    args = parser.parse_args(argv)
    try:
        tip_spans = {name: float(value) for name, value in
                     (item.split("=", 1) for item in args.tip_span)}
    except ValueError:
        parser.error('--tip-span must look like "Fin=0.2601"')
    Path(args.output).write_text(geometry_toml(args.plane, args.polar, args.cd0, args.controls,
                                               tip_spans, args.note))


if __name__ == "__main__":
    main()
