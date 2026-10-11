# Plan: stability derivatives from geometry (vortex lattice)

Approved 2026-10-11. Phases 1 and 2 run first, then stop for review.

Method: steady vortex-lattice (ring vortices on the mean camber surface, fixed wake along
the trim freestream), the same class of method XFLR5 used for the DF1 logs, so the check
is like for like. Limits: no drag prediction from first principles (zero-lift drag is an
input) and no fuselage aerodynamics.

## Phase 1 — geometry and core solver
- [x] Geometry file format (TOML): surfaces, sections, airfoils, mass properties
- [x] Airfoil camber lines (Selig .dat files, NACA 4-digit)
- [x] Importer from XFLR5 plane XML to the geometry format
- [x] Lattice + solver: forces and moments at given alpha, sideslip and rotation rates
- [x] Check lift-curve slope, pitching-moment slope, trim angle and speed against the logs

## Phase 2 — all stability derivatives
- [x] Static and rate derivatives, lateral and longitudinal, in stability axes
- [x] Write lateral derivatives into the existing aircraft file format
- [x] Derivative-by-derivative comparison against the three XFLR5 logs
- [x] Lateral modes from the predicted derivatives against XFLR5's eigenvalues
- [x] Mesh-convergence check

Status 2026-10-11: phases 1 and 2 done; stopped for review. Results and method decisions are in
`docs/decisions/2026-10-11-vortex-lattice-method.md`.

## Phase 3 — control derivatives (not started; needs approval)
- [ ] Elevator, aileron and rudder deflection
- [ ] Check against the elevator values in the logs and the report's Table 4.4.4

## Phase 4 — product packaging (not started; needs approval)
- [ ] One installable command: geometry to derivatives, modes, simulation, report
- [ ] User documentation
