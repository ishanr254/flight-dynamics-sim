# Control surfaces: sign conventions and how deflection is modelled

Date: 2026-10-11

## Decided

**Controls and their signs** follow the course notes (Ch. 5, Ch. 9) and the signs of the
derivatives tabulated in Case Studies 5 and 7:

| Control | Symbol | Positive deflection | Effect of a positive deflection |
|---|---|---|---|
| Elevator | `delta_m` | trailing edge down | more lift, nose-down pitching moment (`Cm_dm < 0`) |
| Aileron | `delta_l` | starboard trailing edge down, port up | roll to port (`Cl_dl < 0`) |
| Rudder | `delta_n` | trailing edge to port | side force to starboard, nose to port (`Cn_dn < 0`) |

Throttle (`delta_x`) stays in the control vector but has no derivative yet.

**In the geometry file** a control is attached to a surface with its role, the fraction of
the local chord behind the hinge, and its spanwise extent along the surface. On a
symmetric surface the control exists on both halves; the role sets whether the two halves
move together (elevator) or opposite (aileron).

**In the lattice** a deflection rotates the normals of the panels behind the hinge line
about the hinge axis; the lattice itself does not move. The mesh is built so that a panel
edge lies on every hinge line and every control's spanwise edge. Control derivatives are
central differences of the loads over +/-0.01 rad at the trim condition.

**In the aircraft file** the lateral control derivatives already exist (`CY_dl` ... `Cn_dn`).
The longitudinal section gains `CX_dm`, `CZ_dm`, `Cm_dm`, optional and zero by default so
existing files stay valid:

    X_dm = qS CX_dm      Z_dm = qS CZ_dm      M_dm = qS l Cm_dm      per radian

## Alternatives considered

- Moving the lattice geometry with the deflection instead of rotating normals: more
  faithful at large deflections, but the derivative at zero deflection is the same to
  first order and the wake and mesh would have to be rebuilt for each deflection.
- Taking control derivatives from the design report's table instead of computing them:
  kept as a comparison, not as the source, because the tool is meant to work from geometry.

## Limits

- Inviscid, attached flow. Real control effectiveness is lower, typically by 10-30%,
  because of boundary-layer thickening and hinge gaps; the lattice does not model that.
- Small deflections only (the derivative is taken at zero deflection).
- No hinge moments.

## Results

Implemented in `src/aero` (geometry, lattice, solver) and carried through the aircraft
file to the dynamics. Run `python3 -m cases.df1_controls`.

**Against theory.** For a full-span 30%-chord flap on an aspect-ratio-10 wing, the ratio
of lift per unit flap deflection to lift per unit angle of attack is within 6% of the
thin-airfoil value (0.66). Symmetric and antisymmetric controls do not cross-couple:
elevator produces no lateral load and aileron no lift or pitching moment.

**Against XFLR5 (like for like).** The Mission 1 XFLR5 log holds derivatives for rotating
the whole tailplane. The lattice with an all-moving tailplane gives (per radian):

| | Lattice | XFLR5 | Diff |
|---|---|---|---|
| CZde | -0.617 | -0.605 | +1.9% |
| CMde | -1.534 | -1.501 | +2.2% |

**Against the design report's Table 4.4.4** (Mission 1, real control geometry, per radian):

| | Lattice | Report | Lattice relative to report |
|---|---|---|---|
| Elevator lift, CL | 0.418 | 0.413 | 1% larger |
| Elevator pitching power, Cm | -1.101 | -1.180 | 7% smaller |
| Aileron rolling power, Cl | -0.194 | -0.229 | 15% smaller |
| Rudder side force, CY | 0.0708 | 0.0630 | 12% larger |
| Rudder yawing power, Cn | -0.0512 | -0.0458 | 12% larger |

The report's values were read off XFLR5 moment-coefficient curves at several deflections,
with control geometry that is not recorded in the repository, so this is a looser
comparison than the all-moving-tail one. Two known differences in what is being compared:
the aileron's spanwise position here was read from the three-view drawing, and the
report's fin is 10.24 in tall where the XFLR5 model (used here) has 5.2 in.

**Mesh sensitivity.** Elevator and aileron power change by under 2% between 400 and 1140
panels. Adding hinge lines to the mesh changes the stability derivatives by about 0.1% at
the default mesh.

**Control responses** for Mission 1 (lattice derivatives, distributed-mass inertia): a
2 deg aileron deflection gives a steady roll rate of about 54 deg/s; a 1 deg up-elevator
step raises angle of attack by about 1 deg within 0.3 s; a rudder doublet excites the
Dutch roll, which decays over about 4 s. These are model outputs with no measured
counterpart yet (`docs/flight-validation.md`).

## Follow-ups

- Trim at a chosen speed using the elevator (currently the trim speed is whatever balances
  the aircraft with controls neutral). Needed for predictions at flight-test speeds.
- Throttle derivative.
- An empirical effectiveness factor for viscous losses, if flight data supports one.
