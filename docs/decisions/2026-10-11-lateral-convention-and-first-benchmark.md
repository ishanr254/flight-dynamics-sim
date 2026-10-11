# Lateral convention locked to MAE 4070/5070 Ch. 9; CS7 is the first benchmark

Date: 2026-10-11

## Decided

The lateral-directional model is the linearized system of Ch. 9 eq. (5)
(`docs/courseNotes/9_Lateral_Dynamics_2026_4_17.pdf`, p.2), implemented as written:

- State `[beta, r, p, phi]`, controls `[delta_l, delta_n]`, about straight level flight.
- Aerodynamic-angle form with non-dimensional coefficients (`CY_*`, `Cl_*`, `Cn_*`).
- One reference length `l` for every moment coefficient and rate derivative; rate
  derivatives are per `p*l/V` and `r*l/V` (not `b/2V`).
- Inertias `A` (roll), `C` (yaw). The notes take the product of inertia `E` as zero; the
  solver keeps it (see the DF1 section below) and reduces to eq. (5) exactly when `E = 0`.
- SI units, radians.

Case Study 7 (civil transport) is the first benchmark aircraft, because it is the only
aircraft on hand with a complete lateral data set and independently stated answers.

Longitudinal is NOT locked yet. Ch. 5 fixes the state `[V, gamma, alpha, q, h]` and
controls `[delta_x, delta_m]`, but see "Open" below.

## Alternatives considered

- Body-axis velocity form (`u, v, w` with dimensional derivatives `Yv, Lp, ...`), the form
  XFLR5 and AVL report. Rejected for the core: every benchmark number we have is in the
  course form, so this would put a conversion between the data and the check.
- `b/2V` rate normalization (US textbook / XFLR5 / AVL). Rejected for the same reason.
  Consequence: XFLR5 or AVL derivatives need an explicit converter (factor of 2, and
  span vs. `l`) that lives outside the solver, in the aircraft-data layer.

## What the CS7 comparison showed

Run `python3 -m cases.cs7_lateral`. Against the values CS7 states precisely:

| Metric | Computed | CS7 |
|---|---|---|
| Roll time constant | 0.401 s | 0.4 s |
| Steady roll rate, delta_l = -5 deg (decoupled) | 2.895 deg/s | 2.9 deg/s |
| Cn_beta factor for neutral spiral | 1.372 | 1.37 |
| Bank angle at 200 s from phi_0 = 30 deg | 6.03 deg | ~6 deg (figure) |

Loosely quoted values agree loosely: Dutch-roll period 5.7 s vs "around 6 s", damping
ratio 0.245 vs "around 0.3"; with the `delta_n = 1.8 r` yaw damper, 0.85 vs 0.7 and a
spiral time constant of 13 s vs 17 s.

Two things to know:

1. **CS7's reference simulation is not fully linear.** Its long-time curves (rudder step
   at 50 s, unstable spiral at 200 s) are reproduced only if the gravity term keeps
   `sin(phi)` instead of `phi`. Checked in a one-off run: with `sin(phi)` the rudder step
   gives beta = -1.25 deg, r = 4.44 deg/s at 50 s (figure: about -1.2, 4.4) and the
   doubled-`Cn_beta` spiral reaches phi = 94.9 deg at 200 s (figure: about 95). The linear
   tool matches wherever bank stays small and drifts once it passes roughly 30 deg, which
   is the validity limit the notes themselves flag.
2. **CS7's quoted spiral time constant ("around -700 s") disagrees with its own figure.**
   The Q4 figure (30 deg to 6 deg in 200 s) implies about 125 s; the model gives 129 s.

## DF1 through the same solver (added later the same day)

The XFLR5 stability logs for the three DF1 mission configurations were converted with
`python3 -m src.aircraft.xflr5` and run through the unchanged pipeline
(`python3 -m cases.df1_lateral`). All lateral modes match XFLR5's own eigenvalues to the
4 significant figures the log prints, e.g. Mission 1: roll -147.96 vs -147.9, Dutch roll
-1.2791 ± 8.3250j vs -1.279 ± 8.325j, spiral +0.05598 vs +0.05598 1/s.

Decision: **keep the product of inertia `E` in the model**, via
`A*dp/dt - E*dr/dt = L` and `C*dr/dt - E*dp/dt = N`. Alternative was to follow the notes
and neglect it. Rejected because for DF1 Mission 1 `E^2/(A*C) = 0.17`: with `E = 0` the
roll eigenvalue is about -123 1/s instead of -148, a 17% error that is purely a modelling
omission. CS7 has `E = 0`, so its results are unchanged.

Data conventions for XFLR5 input: stability axes (so `alpha = theta = 0` in the aircraft
file), moments re-referenced from span to chord, rates from `b/2V` to `l/V`.

Data-quality caveat, not a solver issue: every point mass in the three XFLR5 models sits
on the centreline (y = 0), and the Mission 1 wing has no distributed mass. Roll inertia is
therefore very likely underestimated (0.011 kg m^2 for M1 and M3 vs 0.071 for M2), which
is what makes the M1 roll mode an implausible 7 ms. The tool reproduces XFLR5 faithfully;
the input inertia is what needs checking.

## Open

- Longitudinal: Ch. 5 derives only some A-matrix entries, and the Concorde data sheet
  CS5 refers to ("described at the end of this case study") is not in the solutions PDF.
  Needed: `l`, pitch inertia `B`, `Cm_q`, `Cm_0`, and the thrust model constants. The
  XFLR5 logs do give DF1 longitudinal derivatives and eigenvalues, in body-velocity form.
- DF1 aileron and rudder derivatives are not in the logs (no lateral control was defined
  in the XFLR5 analysis), so only free responses can be simulated for DF1.
- DF1 roll inertia needs a realistic spanwise mass distribution (see caveat above).
- Whether a `sin(phi)` option belongs in a "linear" tool (needed to match CS7's long-time
  figures) is a scope question for the planning chat.
