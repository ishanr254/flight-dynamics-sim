# Geometry file reference

A geometry file is a TOML file describing an aircraft's lifting surfaces, control surfaces
and mass properties. It is the input to the built-in aerodynamics:

    python3 -m src.aero.derive path/to/geometry.toml --aircraft my_aircraft.toml

which trims the aircraft, computes its stability and control derivatives, and writes an
aircraft file (`docs/aircraft-file.md`) for the rest of the tool. Examples are in
`data/geometry/`. Every problem in a file is reported at once.

## Axes and units

x aft, y to starboard, z up, origin wherever is convenient (the DF1 files use the wing
root leading edge). SI units; angles in degrees where the field name ends in `_deg`.

Note this differs from the aircraft file, which uses x forward, z down. The conversion is
done for you.

## Layout

```toml
name = "My aircraft"

[reference]
S = 0.36287          # m^2, reference area
b = 1.182            # m, span
c = 0.307            # m, mean chord (becomes the aircraft file's reference length l)

[condition]
rho = 1.225          # kg/m^3
g = 9.81             # m/s^2, optional

[drag]
CD0 = 0.0096         # zero-lift drag coefficient; optional, default 0

[mass]
m = 2.973            # kg
cg = [0.0585, 0.0, -0.0553]     # m
Ixx = 0.0111         # kg m^2 about the CG, in the geometry axes
Iyy = 0.1831
Izz = 0.1748
Ixz = 0.0158         # optional, default 0

[[surface]]
name = "Main Wing"
position = [0.0, 0.0, 0.0]      # root leading edge, m
incidence_deg = 0.0             # leading edge up positive
symmetric = true                # mirrored about the x-z plane
vertical = false                # true for a fin: sections run along +z
  [[surface.section]]
  span = 0.0                    # distance from the surface root, m
  chord = 0.307
  x_offset = 0.0                # leading-edge sweep-back at this section, m
  dihedral_deg = 0.0            # of the panel outboard of this section
  twist_deg = 0.0               # leading edge up, about the leading edge
  airfoil = "FX 60-126"
  [[surface.section]]
  span = 0.591
  chord = 0.307
  airfoil = "FX 60-126"
  [[surface.control]]
  name = "aileron"
  role = "delta_l"
  chord_fraction = 0.25         # fraction of the local chord behind the hinge
  span = [0.3157, 0.5260]       # from, to along the surface, m
```

Each surface needs at least two sections (root and tip), with `span` increasing.

## Airfoils

Only the mean camber line is used. `NACA` four-digit names (`"NACA 0012"`, `"NACA 2412"`)
are built in. Any other name must have a coordinate file `data/airfoils/<name>.dat` in
Selig order (upper surface from the trailing edge to the nose, then the lower surface
back).

## Controls

| `role` | Control | Positive deflection | Two halves of a symmetric surface |
|---|---|---|---|
| `delta_m` | elevator | trailing edge down | move together |
| `delta_l` | aileron | starboard trailing edge down | move opposite |
| `delta_n` | rudder (on a `vertical` surface) | trailing edge to port | n/a |

`chord_fraction = 1.0` makes the whole chord move (an all-moving surface). The mesh puts a
panel edge on every hinge line and control end automatically.

## What the drag input means

The vortex lattice predicts induced drag but not profile (viscous) drag, so `CD0` is an
input. It affects the speed derivatives and therefore phugoid damping, and little else.

## What is not modelled

Fuselage and propeller aerodynamics, viscous effects and stall, compressibility, hinge
moments, and derivatives with respect to rate of change of angle of attack. Method and
settings: `docs/decisions/2026-10-11-vortex-lattice-method.md`.
