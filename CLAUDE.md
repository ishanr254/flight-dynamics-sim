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
  analyze.py    # generic entry point: modal analysis of any aircraft file
cases/          # aircraft-specific scenarios and figures (never imported by src/)
data/
  aircraft/     # per-aircraft data files (TOML)
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
b/2V — the factor-of-2 trap), inertias `A`/`C` with `E` neglected, SI and radians.

**Longitudinal — not locked.** Ch. 5 fixes state `[V, gamma, alpha, q, h]` and controls
`[delta_x, delta_m]`; the remaining matrix entries and a complete benchmark data set are
still missing. Match the course notes; don't invent a convention.

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

2026-10-11: lateral-directional pipeline works end to end on the CS7 transport
(loader + validation, state-space, modal analysis, time simulation, reference
comparison, figures); 33 tests pass. Matches CS7's precisely stated values to <1%.

Open, in order:
1. DF1 needs stability derivatives (XFLR5 stability log) and an XFLR5→course-convention
   converter; DF1's large `Ixz` makes the `E = 0` assumption questionable.
2. Longitudinal model: needs the Concorde data sheet for CS5 and the full A matrix.
3. Scope question for the planning chat: CS7's long-time figures need `sin(phi)` in the
   gravity term, which a strictly linear tool does not have.
