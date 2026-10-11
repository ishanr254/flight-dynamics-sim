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
  simulate/     # linear responses; nonlinear 6-DOF model; course autopilot
  propulsion/   # propeller tables, motor and battery, solved operating points
  performance/  # performance data loading; envelope (stall, top speed, turn limits)
  viz/          # exports flights for the web viewer
  verify/       # benchmark comparison, reference cases (R3-R5)
  aero/         # geometry -> stability derivatives (built-in vortex lattice, R9)
  analyze.py    # generic entry point: modal analysis of any aircraft file
cases/          # aircraft-specific scenarios and figures (never imported by src/)
viz/            # standalone web viewer (index.html, vendor/, data/)
data/
  aircraft/     # per-aircraft data files (TOML)
  geometry/     # geometry + mass files, input to src.aero
  airfoils/     # Selig .dat coordinates (NACA 4-digit sections are built in)
  courses/      # course layouts and flight limits for the autopilot
  performance/  # per-configuration drag polar, CL_max, structure, powertrain, references
  propulsion/   # propeller coefficient tables (APC)
  flight/       # flight logs
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
./venv/bin/python -m cases.df1_controls     # control derivatives and control responses
./venv/bin/python -m cases.df1_course       # fly the course at a set speed and turn limit
./venv/bin/python -m cases.df1_performance  # propulsion check, envelope, full-throttle limit laps
./venv/bin/python -m cases.df1_m2_energy_check  # model vs the reported Mission 2 flight
./venv/bin/python -m cases.df1_flight_top_speed # level top speed from the flight log vs the model
./venv/bin/python -m cases.manoeuvres       # manoeuvres, natural modes, second aircraft
./venv/bin/python -m cases.viewer_scenarios # regenerate viz/data/scenarios.js
open viz/index.html                          # the viewer (no server needed)
./venv/bin/python -m src.aircraft.xflr5 LOG --chord C --aircraft OUT.toml  # XFLR5 log -> aircraft file
# add --spread data/aircraft/df1_mass_distribution.toml for the corrected-inertia version
./venv/bin/python -m src.aero.derive data/geometry/df1_m1.toml --aircraft OUT.toml  # geometry -> derivatives
./venv/bin/python -m cases.df1_vlm          # built-in lattice vs XFLR5, derivative by derivative
./venv/bin/python -m cases.df1_flight_log   # inventory of the DF1 flight log (no model comparison yet)
```

## Visualization

The viewer lives in `viz/` in this repo: one static page, vanilla JS, a vendored copy of
Three.js r128, no build step, no server, no network requests. It must stay that way: it
has to work by opening `viz/index.html` from disk, with no dependence on another repo, on
any AI assistant, or on an internet connection. `tests/test_viz_export.py` enforces this.

It does no physics. The simulator exports flights (`src/viz/export.py`) to
`viz/data/scenarios.js`, and the page plays them back. Its main purpose is to show the
aircraft flying the competition course; a second mode for extreme manoeuvres is planned.
It was ported from the sibling `flight-dynamics-visual` page; see `viz/README.md`.

Published by GitHub Pages from the repo root, so the viewer is at
`https://ishanr254.github.io/flight-dynamics-sim/viz/`.

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

**Controls — locked** (2026-10-11): `delta_m` elevator (trailing edge down positive),
`delta_l` aileron (starboard trailing edge down positive), `delta_n` rudder (trailing edge
to port positive), all per radian; signs follow the course case studies.

User-facing references: `docs/aircraft-file.md` (keep in step with
`src/aircraft/loader.py`) and `docs/geometry-file.md` (keep in step with
`src/aero/geometry.py`).

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

2026-10-11. Linear dynamics (lateral and longitudinal), aerodynamics from geometry,
control derivatives, a nonlinear 6-DOF model with a course autopilot, performance limits
from propulsion, drag and structure, and a standalone web viewer are all in place. 204
tests pass.

- **Dynamics code verified** against two independent sources: the CS7 transport (lateral;
  matches the case study's precisely stated values to <1%) and DF1 in three mission
  configurations (lateral and longitudinal; matches XFLR5's eigenvalues to 4 figures).
- **Derivatives from geometry** (R9) by the built-in vortex lattice in `src/aero`. For
  DF1, main stability derivatives are within about 5-10% of XFLR5; lateral modes within
  about 10%; short period and phugoid frequency within 2%; phugoid damping 20-45% high
  (drag-driven). Control derivatives: all-moving tail within about 2% of XFLR5; real
  surfaces within 1-15% of the design report's table.
- **Nonlinear model** (R10): linearizing it returns the linear modes to 1e-9. DF1
  Mission 1 flies three laps of the DBF course at 27.9 m/s in 44.4 s per lap, 61 deg bank,
  load factor 2.08, never leaving the model's valid range. The lap time follows from the
  chosen speed and load-factor limit; it is not a competition prediction.
- **Performance limits** (R11): propulsion (APC propeller tables, motor, battery;
  thrust calibrated to the static test), the report's drag polar, CL_max and a 4.5 g
  structural limit. DF1 Mission 1: top speed 29.6 m/s; full-throttle lap 32.1 s at the
  4.5 g limit, 34.4 s at the report's 3.5 g, 45.7 s at 2 g. The load-factor limit matters
  far more than thrust. Slowing to about the corner speed before each turn is faster
  (31.7 s at 4.5 g; worth more at lower limits) and uses much less battery. Not checked
  against a measured lap.
- **Whole missions:** steady wind, a takeoff roll per the rules, and a landing circuit
  with roll-out (`src/simulate/mission.py`). The Mission 2 takeoff roll matches the
  report's (7.46 m vs 7.55 m).
- **Two drag models coexist on purpose:** linear modes and the set-speed course flight
  use the aircraft file's (low, XFLR5-based) drag so they stay comparable with XFLR5;
  anything about performance uses `data/performance/*.toml`.
- **Manoeuvre mode:** scripted control inputs and a small manoeuvre pilot
  (`src/simulate/manoeuvre.py`). Every sample is flagged if outside the aerodynamic
  model's valid range (angle of attack, sideslip, lift limit, roll helix angle) or over
  the structural load limit. Seven DF1 manoeuvres, five natural modes from the nonlinear
  model, and three Case Study 7 responses from the linear model.
- **Viewer** (`viz/`): 19 scenarios in four groups (Course, Manoeuvres, Natural modes,
  second aircraft), with time traces, two warning labels and a suggested playback speed.
- **Flight data:** one DF1 PX4 log is in `data/flight/`. It has been inventoried but not
  compared with the model: it has no hands-off segments, so it cannot confirm the modes.
  `docs/flight-validation.md` is the living record of this and must be kept honest about
  what has and has not been compared with real data. Mode validation is paused until
  Ishan flies dedicated test manoeuvres.
  Two comparisons with real data exist. (1) Level top speed from the Mission 1 log,
  by excess power at full throttle: about 29.1 m/s against the model's 29.8 m/s (7 s of
  data). The same data suggest the model has about twice the real excess power at
  23-25 m/s. (2) A reported Mission 2 flight, eight laps, pack down to about 2% (about
  4.4 Ah): the model's whole flight uses about 3.2 Ah, a quarter low. Cause not
  identified; it is not speed or drag. No calibration has been changed on this evidence.

DF1 files. Two geometries exist on purpose:
- `data/geometry/df1_m*.toml`: exactly the XFLR5 model, whose fin is 5.24 in tall. Only
  for comparing the tool with XFLR5.
- `data/geometry/df1_m*_asbuilt.toml`: the same with the fin at the design report's
  10.24 in (Ishan's decision, 2026-10-11). For anything describing the aircraft.

Aircraft files per mission:
- `df1_m*.toml`: XFLR5 derivatives and inertia. For verifying the solver. No controls.
- `df1_m*_distributed.toml`: XFLR5 derivatives, inertia corrected from the design report.
- `df1_m*_vlm.toml`: lattice derivatives with controls from the XFLR5 geometry, XFLR5
  inertia. For comparing aerodynamics with XFLR5.
- `df1_m*_asbuilt.toml`: lattice derivatives with controls from the as-built geometry,
  corrected inertia. **The tool's description of the aircraft**; used by the course
  flights, performance limits, manoeuvres and natural modes. Dutch roll 2.03 Hz
  (1.17 Hz with the XFLR5 fin).

The design report's own stability and controllability tables came from the short-fin
XFLR5 model, so its Dutch-roll and rudder numbers differ from the as-built results.

**The repository is public** (Ishan's choice, 2026-10-11) and Pages serves the whole repo,
including `docs/courseNotes/` (course notes and case-study solutions) and the flight log.
He has said he will deal with that later; do not add further third-party material.

Open, in order:
0. Performance follow-ups, if wanted (items 5-6 of the usefulness list): Missions 2 and 3,
   scoring and the five-minute window, design sweeps.
1. Stage 5 of `docs/plans/2026-10-11-course-flight-and-visualizer.md`: publishing details
   and user documentation; then vortex-lattice phase 4 (product packaging: one installable
   command).
2. Write-up of R4, R5 and R7 as results; numeric thresholds for the requirements.
3. Trim at a chosen speed using the elevator; throttle derivative; thrust lapse; wind.
4. DF1 data: Missions 2 and 3 XFLR5 masses disagree with the design report.
5. Scope question still open: the five-state course longitudinal form.
