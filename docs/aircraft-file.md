# Aircraft file reference

An aircraft file is a TOML file that holds one aircraft at one flight condition. It is the
interface between "where the numbers came from" and the solver: the dynamics code reads
nothing else. Files can be written by hand, converted from an XFLR5 stability log
(`src.aircraft.xflr5`), or computed from geometry (`src.aero.derive`, input described in
`docs/geometry-file.md`).

Everything is SI, angles in radians. Check a file with:

    python3 -m src.analyze path/to/aircraft.toml

Every problem in the file is reported at once (missing fields, misspelled names, values in
degrees, impossible inertia), not just the first.

## Layout

```toml
name = "My aircraft"            # optional; defaults to the file name
source = "where the data is from"   # optional; printed in reports

[mass]
m = 2.973                       # kg

[reference]
S = 0.36287                     # m^2, reference area
l = 0.307                       # m, the ONE reference length (see below)

[inertia]                       # kg m^2, about the centre of gravity
A = 0.0118                      # roll,  about x   (needed for [lateral])
B = 0.1831                      # pitch, about y   (needed for [longitudinal])
C = 0.1742                      # yaw,   about z   (needed for [lateral])
E = 0.0189                      # xz product of inertia; optional, default 0

[lateral]                       # optional section
# ... 15 coefficients, below

[longitudinal]                  # optional section
# ... 8 coefficients, below

[condition]
V = 27.77                       # m/s, trim speed
rho = 1.225                     # kg/m^3
alpha = 0.0                     # rad, optional; angle between the x axis and the velocity
theta = 0.0                     # rad, optional; trim pitch angle (climb angle if alpha = 0)
```

At least one of `[lateral]` and `[longitudinal]` must be present. A file may have only one.

## Axes

x forward, y to starboard, z down. If the derivatives and inertias are given in stability
axes (x along the trim flight path), set `alpha = 0`; then `theta` is the climb angle.
Files produced by this tool's converters are always in stability axes.

## The reference length and rate normalization

There is one reference length `l` for the whole file, normally the mean chord. It is used
for every moment coefficient and every rate derivative:

- moment = `1/2 rho V^2 S l` x coefficient, for roll, pitch and yaw alike
- rate derivatives are per `p l / V`, `q l / V`, `r l / V`

This is the MAE 4070/5070 course convention. It differs from the convention XFLR5, AVL and
most US textbooks use (span `b` for roll and yaw moments, chord for pitch, rates per
`p b / 2V` and `q c / 2V`). Copying numbers from those sources without converting gives
wrong answers by factors of 2 or more; use the converter.

## `[lateral]`

State `[beta, r, p, phi]`: sideslip (positive with the wind from the right), yaw rate,
roll rate, bank angle. Controls `[delta_l, delta_n]`: roll control (aileron) and yaw
control (rudder). Model: course notes Ch. 9 eq. (5), with the product of inertia kept.

| | side force `CY_` | rolling moment `Cl_` | yawing moment `Cn_` |
|---|---|---|---|
| sideslip | `CY_beta` | `Cl_beta` | `Cn_beta` |
| roll rate | `CY_p` | `Cl_p` | `Cn_p` |
| yaw rate | `CY_r` | `Cl_r` | `Cn_r` |
| roll control | `CY_dl` | `Cl_dl` | `Cn_dl` |
| yaw control | `CY_dn` | `Cl_dn` | `Cn_dn` |

All fifteen are required. Use `0.0` for a control derivative that is not known, and say so
in a comment: zero then means "not available", and control responses cannot be trusted.

## `[longitudinal]`

State `[u, alpha, q, theta]`: change in forward speed (m/s), change in angle of attack,
pitch rate, change in pitch angle. Controls `[delta_x, delta_m]`: throttle and elevator.
The elevator derivatives below are optional (default zero); throttle has none yet.

Forces are X (forward) and Z (down), in stability axes; M is the pitching moment (nose up).
Each coefficient is the dimensional derivative made non-dimensional:

| Coefficient | Definition | Typical sign |
|---|---|---|
| `CX_u` | `V X_u / qS` | negative, about `-2 CD` |
| `CX_alpha` | `V X_w / qS` | positive, about `CL - dCD/dalpha` |
| `CZ_u` | `V Z_u / qS` | negative, about `-2 CL` |
| `CZ_alpha` | `V Z_w / qS` | negative, about `-(CL_alpha + CD)` |
| `CZ_q` | `V Z_q / (qS l)` | negative |
| `Cm_u` | `V M_u / (qS l)` | near zero |
| `Cm_alpha` | `V M_w / (qS l)` | negative for static stability |
| `Cm_q` | `V M_q / (qS l^2)` | negative (pitch damping) |

| `CX_dm` | `X_dm / qS` | small | 
| `CZ_dm` | `Z_dm / qS` | negative (elevator down adds lift) |
| `Cm_dm` | `M_dm / (qS l)` | negative (elevator down pitches nose down) |

with `qS = 1/2 rho V^2 S`. The speed derivatives are of the whole force, which is why
`CZ_u` is about `-2 CL` rather than near zero. The last three are optional and per radian.

## Control sign conventions

| Control | Positive deflection | Sign of the main derivative |
|---|---|---|
| `delta_m` elevator | trailing edge down | `Cm_dm < 0` |
| `delta_l` aileron | starboard trailing edge down, port up | `Cl_dl < 0` (roll to port) |
| `delta_n` rudder | trailing edge to port | `Cn_dn < 0` (nose to port) |

All control derivatives are per radian of deflection.

Not modelled: derivatives with respect to the rate of change of angle of attack, thrust
changes with speed, and altitude effects.

## Where the conventions are decided

- Controls: `docs/decisions/2026-10-11-control-surfaces.md`
- Lateral: `docs/decisions/2026-10-11-lateral-convention-and-first-benchmark.md`
- Longitudinal: `docs/decisions/2026-10-11-longitudinal-convention.md`
