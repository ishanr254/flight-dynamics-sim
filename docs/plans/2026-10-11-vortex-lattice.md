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

## Phase 3 — control derivatives (approved 2026-10-11)
- [x] Record sign conventions and the modelling approach
      (`docs/decisions/2026-10-11-control-surfaces.md`)
- [x] Control surfaces in the geometry file (hinge position, spanwise extent, role)
- [x] Lattice: mesh lines on hinge lines and control edges; deflection as a rotation of
      the panel normals aft of the hinge
- [x] Control derivatives for elevator, aileron and rudder
- [x] Aircraft file: longitudinal control derivatives; lateral ones filled in
- [x] Check against the all-moving-tail derivatives in the Mission 1 XFLR5 log
- [x] Check against the design report's Table 4.4.4
- [x] Control responses for DF1 (elevator, aileron, rudder inputs) with figures
- [x] Tests, file-format reference, status

Status 2026-10-11: phase 3 done; stopped for review. Results are in
`docs/decisions/2026-10-11-control-surfaces.md`.

## Phase 4 — product packaging (not started; needs approval)
- [ ] One installable command: geometry to derivatives, modes, simulation, report
- [ ] User documentation
