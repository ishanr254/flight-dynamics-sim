# Plan: longitudinal dynamics

Approved 2026-10-11. All eight steps run, then stop for review. Controls are held for
phase 3 of the vortex-lattice plan.

Goal: short-period and phugoid modes plus longitudinal time responses for any aircraft
file, through the same pipeline as lateral, verified on DF1 against XFLR5's longitudinal
eigenvalues for all three missions.

- [x] 1. Record the convention (`docs/decisions/2026-10-11-longitudinal-convention.md`)
- [x] 2. Aircraft file: optional `[longitudinal]` section and pitch inertia `B`;
        lateral-only files keep loading; same style of diagnostics
- [x] 3. Longitudinal state-space builder (`src/dynamics/longitudinal.py`)
- [x] 4. Mode naming: short period, phugoid, with a fallback
- [x] 5. Fill the longitudinal section from the XFLR5 log converter and from the lattice
- [x] 6. Verify in three layers:
        (a) XFLR5 derivatives through our solver vs XFLR5 eigenvalues
        (b) lattice derivatives through our solver vs XFLR5 eigenvalues
        (c) against the course notes: phugoid period ~ 0.45 V, short-period approximation
- [x] 7. DF1 time responses and figures: angle-of-attack release, speed release
- [x] 8. `src.analyze` prints lateral and longitudinal; tests; file-format reference;
        `CLAUDE.md` status

Status 2026-10-11: all eight steps done; stopped for review. Results are in
`docs/decisions/2026-10-11-longitudinal-convention.md`; the file format is described in
`docs/aircraft-file.md`.

Not included: the five-state course form (altitude, thrust model); derivatives with
respect to rate of change of angle of attack (taken as zero); thrust variation with speed.
