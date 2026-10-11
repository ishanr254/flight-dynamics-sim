# CLAUDE.md

Context for any Claude Code session working in this repo. Read this first.

## What this project is

A general-purpose, reusable linear flight-dynamics analysis tool for preliminary
aircraft stability and control assessment (Cornell MAE senior design, Fall 2026).
Full scope in `README.md`, requirements in `docs/requirements.md`, timeline in
`docs/timeline.md`.

The simulator itself is the engineering design artifact — not any one aircraft's
results. Two aircraft must both work through the same solver with zero
aircraft-specific edits to solver code (R2) — this is a hard architectural
constraint from day one, not a nice-to-have added later.

## Planning happens elsewhere

Requirements, timeline, and scope decisions are made in a separate Claude Project
(chat), not here. Don't re-derive or renegotiate scope inside a coding session —
if something in `docs/requirements.md` looks wrong or needs to change, flag it for
discussion there rather than editing requirements unilaterally in this repo.

## Architecture (starting point, not locked — expect this to evolve)

```
src/
  aircraft/     # Aircraft data class, loader, validation (R6)
  dynamics/     # state-space construction (A, B matrices), conventions
  analysis/     # modal/eigen analysis, mode classification
  simulate/     # time-domain integration (scipy.integrate.solve_ivp)
  viz/          # exports time-history JSON for the visualizer
  verify/       # benchmark comparison, reference cases (R3-R5)
  aero/         # geometry -> stability derivatives (built-in vortex lattice, R9)
  analyze.py    # generic entry point: modal analysis of any aircraft file
cases/          # aircraft-specific scenarios and figures (never imported by src/)
data/
  aircraft/     # per-aircraft data files (TOML)
  geometry/     # geometry + mass files, input to src.aero
  airfoils/     # Selig .dat coordinates (NACA 4-digit sections are built in)
  reference/    # benchmark reference values, with how each was obtained
tests/
docs/
  requirements.md
  timeline.md
  decisions/    # one file per real design decision, dated, as it happens
```

Aircraft-specific data must never touch generic solver code. Test logic stays out
of module files (separation of concerns).

Python 3.12, venv, `python3` not `python` — same convention as the
`sizing-scoring-26-27` repo.

**Standalone:** the tool must run for anyone with Python 3.12 and
`pip install -r requirements.txt`. No run-time dependence on MATLAB, XFLR5, AVL, or
sibling repos — copy code/data in, never import it from where it lives.

## Running

```
python3 -m venv venv && ./venv/bin/pip install -r requirements.txt
./venv/bin/pytest -q
./venv/bin/python -m src.analyze data/aircraft/cs7_transport.toml   # any aircraft file
./venv/bin/python -m cases.cs7_lateral      # CS7 benchmark table + figures in output/
./venv/bin/python -m cases.df1_lateral      # DF1 (3 missions) vs XFLR5 eigenvalues
./venv/bin/python -m cases.df1_longitudinal # DF1 short period and phugoid vs XFLR5
./venv/bin/python -m src.aircraft.xflr5 LOG --chord C --aircraft OUT.toml  # XFLR5 log -> aircraft file
# add --spread data/aircraft/df1_mass_distribution.toml for the corrected-inertia version
./venv/bin/python -m src.aero.derive data/geometry/df1_m1.toml --aircraft OUT.toml  # geometry -> derivatives
./venv/bin/python -m cases.df1_vlm          # built-in lattice vs XFLR5, derivative by derivative
./venv/bin/python -m cases.df1_flight_log   # inventory of the DF1 flight log (no model comparison yet)
```

## Visualization plan

Start from the existing `flight-dynamics-visual` page (sibling repo, same GitHub
folder) rather than building a new renderer — copied into this repo when that phase
starts, per the standalone rule above. It's a single static `index.html` —
vanilla JS, Three.js r128 via CDN, no build step, no server. It already has: a
generic Three.js scene, procedural low-poly aircraft mesh, chase/orbit camera, a
trail buffer, an HTML/CSS HUD, and a hand-drawn 2D canvas attitude indicator — all
already state-agnostic (they just consume px/py/pz/phi/theta/psi each frame and
don't care where those numbers came from).

What actually needs to change: right now, `phys()` computes each of 5 canonical
DF1 modes from hardcoded closed-form formulas (DF1's actual eigenvalues typed in
as literal constants), evaluated live every frame — it doesn't read any external
data. Replace `phys()` with a loader that reads an exported JSON time-history
(`{t, x, y, z, phi, theta, psi}` samples, ~30-60 samples/sec of playback time —
not every solver step) and interpolates at playback time. The five-mode-tab /
stable-unstable-toggle UI needs to become an aircraft/scenario picker — that's the
real rewrite (~30% of the file), not the render engine.

Performance is not expected to be a real constraint. The current scene is a
handful of low-poly primitives and one polyline trail — WebGL/Three.js handles far
more complex scenes than this at 60fps easily. Don't spend time on a performance
investigation here unless something concrete shows otherwise.

## Conventions

**Lateral — locked** (2026-10-11, `docs/decisions/`): MAE 4070/5070 Ch. 9 eq. (5).
State `[beta, r, p, phi]`, controls `[delta_l, delta_n]`, non-dimensional coefficients,
one reference length `l`, rate derivatives per `p*l/V` and `r*l/V` (ℓ/V, not ℓ/2V or
b/2V — the factor-of-2 trap), inertias `A`/`C`/`E` (the notes drop `E`; the solver keeps it), SI and radians.

**Longitudinal — locked** (2026-10-11, `docs/decisions/`): four-state small-perturbation
form, state `[u, alpha, q, theta]`, controls `[delta_x, delta_m]` (zeros until phase 3),
stability axes, coefficients non-dimensionalized with the same single `l` and ℓ/V rates.
No alpha-dot derivatives. The five-state Ch. 5 course form (altitude, thrust model) is
deferred, not rejected.

The aircraft file format is documented for users in `docs/aircraft-file.md`; keep it in
step with `src/aircraft/loader.py`.

Data from XFLR5/AVL uses b/2V and c/2V and body-axis forms: it must be converted in
the aircraft-data layer, never inside the solver.

## Working style

- For larger or delicate work (a new solver module, anything touching the R2
  zero-edit constraint, architecture changes) — plan first: lay out the approach
  and alternatives, get it approved, then implement. Small well-scoped fixes don't
  need this ceremony.
- Don't commit automatically after every change. Only commit when explicitly
  asked, once the change has actually been reviewed/tested — not mid-task.
- Keep the repo clean: one-off debug scripts and scratch files created while
  working shouldn't linger past the task that needed them. Anything worth keeping
  goes where it actually belongs (`docs/decisions/` for real decisions, `tests/`
  for real tests) rather than sitting loose in the tree.
- For visualizer/UI changes, a quick throwaway mockup before the full build is a
  good way to align on behavior first — cheaper to iterate on than the real thing.

## Session habits

- Update the Status section below and add a dated file to `docs/decisions/` before
  ending a session, for any real design decision made (not every commit — a real
  decision is something like "chose formulation X over Y because Z").
- Don't invent conventions or requirements not documented here — flag the gap for
  the planning chat instead of guessing.

## Status

2026-10-11. Lateral and longitudinal linear dynamics both work end to end: aircraft file
with validation, state-space, modal analysis, time simulation, reference comparison,
figures. 92 tests pass.

- **Dynamics code verified** against two independent sources: the CS7 transport (lateral;
  matches the case study's precisely stated values to <1%) and DF1 in three mission
  configurations (lateral and longitudinal; matches XFLR5's eigenvalues to 4 figures).
- **Derivatives from geometry** (R9) by the built-in vortex lattice in `src/aero`. For
  DF1, main derivatives are within about 5-10% of XFLR5; lateral modes within about 10%;
  short period and phugoid frequency within 2%; phugoid damping 20-45% high (drag-driven).

DF1 aircraft files per mission: `df1_m*.toml` (XFLR5 derivatives, for solver
verification), `df1_m*_distributed.toml` (same, inertia corrected from the design report;
use for results about the aircraft), `df1_m*_vlm.toml` (derivatives from the built-in
lattice, XFLR5 inertia).

- **Flight data:** one DF1 PX4 log is in `data/flight/`. It has been inventoried but not
  compared with the model: it has no hands-off segments, so it cannot confirm the modes.
  `docs/flight-validation.md` is the living record of this and must be kept honest about
  what has and has not been compared with real data. **Paused** until Ishan flies
  dedicated test manoeuvres; do not build on the existing log in the meantime.

Open, in order:
1. Vortex-lattice phase 3: control derivatives (elevator, aileron, rudder), then control
   responses. Needs approval. No control inputs are simulated for DF1 yet. This also
   unlocks the one comparison the flight log can support (response to recorded inputs).
2. Visualization export (`src/viz`): not started. R1 (animated demo) is the primary
   success criterion.
3. Vortex-lattice phase 4: product packaging (one installable command, user docs).
4. DF1 data: Missions 2 and 3 XFLR5 masses disagree with the design report (see
   `docs/decisions/2026-10-11-df1-mass-distribution.md`).
5. Scope questions for the planning chat: CS7's long-time figures need `sin(phi)` in the
   gravity term; the five-state course longitudinal form.
