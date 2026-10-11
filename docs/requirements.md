# Requirements

None of the numeric thresholds below are finalized — these are the measurement
*categories*, not adopted tolerances. Real thresholds get attached once benchmark
aircraft data is in hand and a first pass shows what's actually achievable. Treat
any number that shows up in code or docs before that point as illustrative, not
adopted.

| ID | Area | Measurable evaluation |
|----|------|------------------------|
| R1 | Visual demonstration (aircraft #1) | Time-domain response rendered as an animated visualization; motion judged consistent with real-world behavior for the benchmark aircraft |
| R2 | Reuse / generality (aircraft #2) | A second, independently-sourced aircraft loads and analyzes through the same documented interface with zero edits to solver/dynamics code |
| R3 | Dynamic prediction | Predicted mode eigenvalues (ωₙ, ζ) compared against independent reference values at matched flight condition |
| R4 | Time response | Response amplitude, period, decay/growth rate compared against reference for documented initial conditions/inputs |
| R5 | Numerical reliability | Convergence of trajectories and derived response metrics demonstrated under tightened tolerance/step size |
| R6 | Input robustness | Missing fields, unit mismatches, invalid inertia data, unsupported assumptions caught with intelligible diagnostics |
| R7 | Practical execution | Runtime recorded on a named ordinary laptop for representative cases |
| R8 | Traceability / usability | Someone else can reproduce a documented case from inputs and instructions alone; assumptions and data provenance preserved |
| R9 | Aerodynamic prediction from geometry *(added 2026-10-11)* | Stability and control derivatives computed from aircraft geometry by a built-in method, with no external aerodynamics tool at run time; each derivative compared against an independent reference at matched geometry and flight condition |
| R10 | Nonlinear flight simulation *(added 2026-10-11)* | Full rigid-body simulation built on the same aerodynamic data, able to fly the competition course under a simple autopilot and to respond to large scripted control inputs; for small disturbances it reproduces the linear model's modes; samples outside the aerodynamic model's valid range are flagged |
| R11 | Performance limits from the aircraft *(added 2026-10-11)* | Top speed, stall, sustained and instantaneous turn limits, and energy use follow from models of the propulsion system, drag, maximum lift and structure, not from numbers typed in as targets; each model compared with test data or the design report where they exist; a lap flown at those limits gives lap time as an output |

**Stretch (not core):** mission-constraint layer (e.g. DBF competition rules),
sensitivity studies, nonlinear extension.

## Success bar

R1 and R2 are both core requirements, not one primary and one stretch:

- **R1 (aircraft #1)** carries the visual/demonstrative bar — this is the primary
  qualitative success criterion for the project.
- **R2 (aircraft #2)** proves reuse numerically by loading a second aircraft
  through the same interface. It does not necessarily need the same visualization
  polish as aircraft #1 unless time allows.

R3–R11 are the quantitative backbone underneath R1/R2 — the visual demo needs to be
convincing *because* the underlying numbers are verified, not instead of that.

## Benchmark aircraft

DF1 and/or a real aircraft with well-tabulated values, likely drawn from one of
Ishan's MAE 4070/5070 flight dynamics case studies (CS4–CS7). Aircraft #2 should be
independently sourced from aircraft #1 (not a parameter sweep of the same aircraft)
to make R2 a meaningful test.

## Scope change, 2026-10-11

Decided by Ishan in an implementation session (recorded here at his direction): the
long-term goal is a tool that does not rely on XFLR5. R9 adds geometry-to-derivatives
prediction by a built-in vortex-lattice method, verified against the existing XFLR5
results. The tool is also to be treated as a product software package: one installable
program that goes from geometry to derivatives, modes, simulation and a report, with user
documentation. Supplying derivatives directly through the aircraft data file (the original
interface) stays supported. Plan and status: `docs/plans/2026-10-11-vortex-lattice.md`.

## Scope change, 2026-10-11 (visualization goal and nonlinear simulation)

Decided by Ishan in an implementation session (recorded here at his direction):

- **R1's demonstration is the competition course.** The visualization's main purpose is to
  show the aircraft flying the AIAA DBF course (design report Figure 3.1.1: 500 ft upwind,
  180 deg turn, 1000 ft downwind with a 360 deg turn, 180 deg turn, 500 ft back to the
  start line). Mission 1 first.
- **A second mode shows extreme manoeuvres and responses** to large control inputs.
- **Nonlinear simulation moves from stretch to core (R10)**, because neither of the above
  can be done with a small-disturbance linear model.
- **The visualizer lives in this repository** and must run on its own: no dependence on
  another repository, on any AI assistant, or on a network connection at run time. It is
  published as a static site.

Plan and status: `docs/plans/2026-10-11-course-flight-and-visualizer.md`.

## Scope change, 2026-10-11 (performance limits)

Decided by Ishan in an implementation session (recorded here at his direction): the first
course flight taught little, because its speed and turn limit were inputs. R11 makes the
limits come from the aircraft, so the simulation can be used to look for performance.
This is part of what was listed as the stretch "mission-constraint layer"; scoring and
Missions 2 and 3 remain stretch. Structural limit load factor is set to 4.5 at Ishan's
direction (the design report states 3.5). Data comes from the design report and from the
propulsion model in the `sizing-scoring-26-27` repository, copied in, not imported.
Plan and status: `docs/plans/2026-10-11-performance-limits.md`.
