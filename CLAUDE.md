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
data/
  aircraft/     # per-aircraft data files
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

## Visualization plan

Extend the existing `flight-dynamics-visual` repo (sibling repo, same GitHub
folder) rather than building a new renderer. It's a single static `index.html` —
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

## Conventions — not yet locked

Axis system (body vs. stability), positive sign conventions, state ordering
(velocity-component vs. aerodynamic-angle form), and non-dimensionalization (the
MAE 4070/5070 course convention uses ℓ/V for rotary derivatives, not ℓ/2V — a
known factor-of-2 trap) all need to be pinned down before writing dynamics code.
Match whatever convention the benchmark case-study data and the existing AVL
MATLAB wrapper already use, to minimize conversion risk — don't invent a new
convention from scratch.

## Session habits

- Update the Status section below and add a dated file to `docs/decisions/` before
  ending a session, for any real design decision made (not every commit — a real
  decision is something like "chose formulation X over Y because Z").
- Don't invent conventions or requirements not documented here — flag the gap for
  the planning chat instead of guessing.

## Status

Repo scaffolded 2026-09-09. No implementation yet. Next: lock conventions once
benchmark aircraft case-study data (CS4–CS7) is available.
