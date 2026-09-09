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

**Stretch (not core):** mission-constraint layer (e.g. DBF competition rules),
sensitivity studies, nonlinear extension.

## Success bar

R1 and R2 are both core requirements, not one primary and one stretch:

- **R1 (aircraft #1)** carries the visual/demonstrative bar — this is the primary
  qualitative success criterion for the project.
- **R2 (aircraft #2)** proves reuse numerically by loading a second aircraft
  through the same interface. It does not necessarily need the same visualization
  polish as aircraft #1 unless time allows.

R3–R8 are the quantitative backbone underneath R1/R2 — the visual demo needs to be
convincing *because* the underlying numbers are verified, not instead of that.

## Benchmark aircraft

DF1 and/or a real aircraft with well-tabulated values, likely drawn from one of
Ishan's MAE 4070/5070 flight dynamics case studies (CS4–CS7). Aircraft #2 should be
independently sourced from aircraft #1 (not a parameter sweep of the same aircraft)
to make R2 a meaningful test.
