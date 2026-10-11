# Longitudinal convention: four-state small-perturbation form

Date: 2026-10-11

## Decided

State `[u, alpha, q, theta]`, controls `[delta_x, delta_m]` (throttle, elevator; names
from the course notes), about steady straight flight.

- `u`: change in forward speed (m/s). `alpha`: change in angle of attack (rad).
  `q`: pitch rate (rad/s). `theta`: change in pitch angle (rad).
- Stability axes: x along the trim flight path, z down. Forces X (forward) and Z (down),
  pitching moment M (nose up).
- Equations, with `m` mass, `B` pitch inertia, `V` trim speed, `theta_0` trim climb angle
  (`[condition] theta`, zero for level flight):

      m du/dt              = X_u u + X_w w             - m g cos(theta_0) theta
      m (dw/dt - V q)      = Z_u u + Z_w w + Z_q q     - m g sin(theta_0) theta
      B dq/dt              = M_u u + M_w w + M_q q
      dtheta/dt            = q                          with w = V alpha

- Coefficients in the aircraft file are the dimensional derivatives made non-dimensional
  with `qS = 1/2 rho V^2 S` and the file's single reference length `l`, rates per `q l/V`
  (the same l/V normalization as the lateral section):

      X_u = qS CX_u / V        X_w = qS CX_alpha / V
      Z_u = qS CZ_u / V        Z_w = qS CZ_alpha / V       Z_q = qS (l/V) CZ_q
      M_u = qS l Cm_u / V      M_w = qS l Cm_alpha / V     M_q = qS l (l/V) Cm_q

  The speed derivatives are of the whole force, so `CZ_u` is about `-2 CL` at trim (XFLR5
  prints its `Czu` with that part removed; ours keeps it). `CZ_alpha` is about
  `-(CL_alpha + CD)`.
- Controls enter through a B matrix that is all zeros until control derivatives exist.

## Alternatives considered

- **The five-state course form** of Ch. 5, `[V, gamma, alpha, q, h]`, with a thrust model
  and altitude-dependent density. Deferred, not rejected: it adds the propulsion-feedback
  mode, but it needs thrust data and its only benchmark (the Concorde of CS5) has no data
  sheet in the repo. Nothing here prevents adding it later as a second model.
- **Vertical speed `w` as the state** instead of `alpha`, which would match XFLR5's printed
  matrices entry for entry. Rejected for consistency with the lateral model, which uses
  the sideslip angle; the eigenvalues are identical.
- **Coefficients in the c/2V form** XFLR5 and AVL print. Rejected so that one aircraft
  file uses one normalization throughout.

## Assumptions

- No derivatives with respect to rate of change of angle of attack (`M_wdot`, `Z_wdot`).
  Neither XFLR5 nor a steady vortex lattice provides them. This mainly affects
  short-period damping.
- Thrust does not vary with speed or angle of attack, and acts through the CG.
- Density is constant (no altitude state).

## Results

Implemented in `src/dynamics/longitudinal.py`; run `python3 -m cases.df1_longitudinal`.

**(a) Dynamics code against XFLR5.** XFLR5's derivatives through this solver reproduce
XFLR5's longitudinal eigenvalues to the 4 significant figures the log prints, for all
three DF1 missions. Mission 1: short period -10.93 +/- 14.71j (XFLR5: -10.93 +/- 14.71j),
phugoid -0.0205 +/- 0.4157j (XFLR5: -0.02047 +/- 0.4157j). The state matrix equals XFLR5's
printed matrix after the change of variable `w = V alpha`.

**(b) Derivatives from geometry against XFLR5** (`python3 -m cases.df1_vlm`, lattice
derivatives, XFLR5 inertia):

| | Mission 1 | Mission 2 | Mission 3 |
|---|---|---|---|
| Short-period natural frequency | +1.6% | +1.3% | +1.1% |
| Short-period damping ratio | +0.8% | +1.0% | +1.9% |
| Phugoid natural frequency | +1.2% | +0.7% | -1.7% |
| Phugoid damping ratio | +19% | +20% | +46% |

Phugoid damping is set by drag. The lattice supplies only induced drag; zero-lift drag is
an input (set here to XFLR5's viscous drag coefficient), and XFLR5's own speed derivatives
include viscous effects the lattice does not have. The absolute numbers are small (damping
ratio 0.02 to 0.06), so this is a lightly damped mode either way.

**(c) Against the course notes.** The decoupled alpha-q approximation of Ch. 5 sec. 5
matches the full short period to within 0.2% for all three missions. The simple phugoid
estimate `T = sqrt(2) pi V / g` (about 0.45 V) gives 12.6 s for Mission 1 against 15.1 s
from the full model, 17% short (16% for Mission 2, 23% for Mission 3). XFLR5's own eigenvalue agrees with the full model, so this is the
estimate's error for this aircraft, not a discrepancy in the tool.

**Effect of the mass distribution.** The design-report correction raises Mission 1 pitch
inertia from 0.183 to 0.204 kg m^2, which lowers the short-period frequency from 18.3 to
17.3 rad/s; the phugoid is unaffected.

## Follow-ups

- Control derivatives (elevator, throttle): phase 3 of the vortex-lattice plan.
- The lateral-only CS7 file still loads; a longitudinal-only file is also valid.
- The five-state course form remains open if a complete benchmark data set appears.
