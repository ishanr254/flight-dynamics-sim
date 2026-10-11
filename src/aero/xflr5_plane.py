"""Import an XFLR5 plane definition (and its analysis settings) as a geometry file.

    python3 -m src.aero.xflr5_plane PLANE.xml POLAR.xml -o GEOMETRY.toml

The plane XML supplies the lifting surfaces; the polar XML supplies reference dimensions,
air density, mass, centre of gravity and inertia. This is a convenience for existing XFLR5
models; geometry files can equally be written by hand.
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


def _floats(text):
    return [float(v) for v in text.replace(",", " ").split()]


def geometry_toml(plane_path, polar_path, cd0=0.0):
    plane = ET.parse(plane_path).getroot().find("Plane")
    polar = ET.parse(polar_path).getroot().find("Polar")
    inertia = polar.find("Inertia_Value")
    get = lambda node, tag: float(node.find(tag).text)  # noqa: E731

    lines = [
        f"# Imported by src.aero.xflr5_plane from {plane_path} and {polar_path}.",
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
        for section in wing.find("Sections"):
            lines += [
                "  [[surface.section]]",
                f"  span = {get(section, 'y_position')!r}",
                f"  chord = {get(section, 'Chord')!r}",
                f"  x_offset = {get(section, 'xOffset')!r}",
                f"  dihedral_deg = {get(section, 'Dihedral')!r}",
                f"  twist_deg = {get(section, 'Twist')!r}",
                f'  airfoil = "{section.find("Right_Side_FoilName").text.strip()}"',
            ]
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("plane")
    parser.add_argument("polar")
    parser.add_argument("-o", "--output", required=True)
    parser.add_argument("--cd0", type=float, default=0.0,
                        help="zero-lift drag coefficient to record (default 0)")
    args = parser.parse_args(argv)
    Path(args.output).write_text(geometry_toml(args.plane, args.polar, args.cd0))


if __name__ == "__main__":
    main()
