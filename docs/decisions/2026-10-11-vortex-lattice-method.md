# Derivatives from geometry: vortex-lattice method and its settings

Date: 2026-10-11

## Decided

Stability derivatives are computed from geometry by a steady vortex-lattice solver written
for this tool (`src/aero`), with no external aerodynamics program at run time (R9).

- **Lattice:** vortex rings on the mean camber surface of each lifting surface, bound
  segment at each panel's quarter chord, collocation at three-quarter chord.
- **Wake:** leaves the trailing edges along the trim freestream and stays there. The
  circulation is then linear in the motion (u, v, w, p, q, r), the loads are quadratic,
  and central differences give the derivatives exactly.
- **Loads:** Kutta-Joukowski force on the spanwise (bound) segments, using the local
  relative wind plus the velocity induced by the whole lattice. This includes induced
  drag. Forces on chordwise segments are neglected, as AVL does.
- **Vortex core:** a vortex acting on a *different* surface has a finite core of 0.5 x its
  ring's span width. Within a surface vortices stay singular.
- **Trim:** angle of attack for zero pitching moment about the CG, then speed for lift =
  weight (the same definition XFLR5's stability analysis uses).
- **Drag:** zero-lift drag is an input (`[drag] CD0`); the lattice cannot predict it.
- **Input:** a geometry file of our own (`data/geometry/*.toml`). An importer reads XFLR5
  plane files as a convenience; it is not needed to run the solver.
- **Output:** the existing aircraft file format, so the dynamics code is untouched.

## Alternatives considered

- Handbook formulas (as in `sizing-scoring-25-26/src/aero/stability_derivatives.m`):
  simpler, but only rough agreement is possible and they assume a conventional layout.
- Wrapping AVL or XFLR5: breaks the standalone rule.
- Forces on every ring segment (chordwise ones too): tried first. Results did not settle
  with mesh refinement because chordwise segments at the fin/tailplane junction see
  near-singular induced velocities.
- No vortex core: tried. Fin side force swung between -8 and +59 (per fin area) as the mesh
  changed, because the tailplane at -2 deg incidence rises about 5 mm into the fin's first
  row of panels and the two surfaces' vortex lines nearly coincide.

## Verification

Against theory (tests in `tests/test_vlm.py`): lift-curve slope of flat rectangular wings
within 5% of the Helmbold estimate, induced drag within 10% of the elliptic value, an
isolated fin in sideslip within 8% of low-aspect-ratio theory.

Against XFLR5 for DF1 (`python3 -m cases.df1_vlm`, 710 panels), Mission 1:

| | Lattice | XFLR5 | Diff |
|---|---|---|---|
| Trim angle of attack | -0.95 deg | -1.08 deg | 0.14 deg |
| Trim speed | 28.05 m/s | 27.77 m/s | +1.0% |
| Neutral point | 0.1261 m | 0.1254 m | +0.6% |
| CLa | 3.894 | 3.893 | 0.0% |
| Cma | -0.857 | -0.848 | -1.1% |
| Cmq | -8.86 | -8.67 | -2.2% |
| CLq | 7.19 | 6.85 | +5.0% |
| Clp | -0.339 | -0.336 | -0.8% |
| Clr | 0.0637 | 0.0630 | +1.2% |
| Cnb | 0.0633 | 0.0598 | +6.0% |
| CYr | 0.140 | 0.132 | +6.6% |
| Cnr | -0.0965 | -0.0859 | -12.3% |
| CYb | -0.104 | -0.090 | -15.4% |

Lateral modes from the lattice derivatives vs XFLR5's eigenvalues, Mission 1: roll time
constant -1.1%, Dutch-roll frequency +4.0%, damping ratio +3.8%, spiral time constant
-3.5%. Missions 2 and 3 are similar (all modal quantities within about 10%).

## Known differences and what they mean

- **CYb** includes `-CD0`, the profile drag turning with the relative wind. Without it the
  value is -0.094 (5% from XFLR5).
- **Cnp** (-0.013 vs -0.039). When induced velocity is left out of the force sum the
  lattice gives -0.042, close to XFLR5, which suggests XFLR5's near-field forces omit it.
  This is an inference from that experiment, not something read from XFLR5's source. The
  version with induced velocity is kept because it is the more complete model.
- **Cmu, Czu** are zero here and small but non-zero in XFLR5. In inviscid flow at fixed
  angle of attack every load scales with speed squared, so they must vanish at trim;
  XFLR5's values come from its viscous corrections.
- **Cxu, Cxa** depend on the drag model; `CD0` was set to XFLR5's viscous drag coefficient.
- **Clb, CYp** are small numbers (the wing has no dihedral), so percentage differences are
  large while absolute differences are about 0.001-0.004.
- **Trim angle** differs by 0.1-0.15 deg, most likely from the airfoil coordinates: the
  FX 60-126 file here was copied from `end-of-semester-25-26`, not exported from XFLR5.

## Limits

- **Fin derivatives depend on the core setting.** With 0.25 instead of 0.5, CYb and Cnb
  are 10-20% larger. The cause is the fin/tailplane junction: fully sealed, the tailplane
  would act as an end plate and raise fin effectiveness further (about 2.4 per fin area
  against 1.4 isolated; the lattice and XFLR5 both give about 1.7). 0.5 was chosen because
  results are stable under mesh refinement there; it also happens to agree with XFLR5.
- Results still drift a few percent between 710 and 1140 panels.
- No fuselage, no propeller, no viscous or stall effects, no compressibility.
- Controls are not modelled yet (phase 3).
- Longitudinal derivatives are computed and compared but not yet written to the aircraft
  file, because the longitudinal convention is not locked.
