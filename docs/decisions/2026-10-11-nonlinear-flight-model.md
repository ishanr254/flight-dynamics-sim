# Nonlinear flight model

Date: 2026-10-11

## Decided

A six-degree-of-freedom rigid-body simulation that takes the same aircraft file as the
linear models. It needs both the `[lateral]` and `[longitudinal]` sections.

**Axes.** Body axes are the stability axes of the trim condition, fixed to the aircraft:
x along the trim flight direction, y to starboard, z down. This is the frame the aircraft
file's derivatives and inertias are already in. The earth frame is north-east-down.

**State (13).** Position (north, east, down), body velocity (u, v, w), attitude as a unit
quaternion, body rates (p, q, r).

**Equations.** `m (dv/dt + omega x v) = F_aero + F_thrust + m g` and
`I domega/dt + omega x (I omega) = M_aero`, with `I` built from A, B, C and E. Gravity and
attitude kinematics are exact, so bank and pitch angles are not limited to small values.

**Aerodynamics.** Forces and moments scale with the actual dynamic pressure
`1/2 rho V^2`, with angle of attack `atan2(w, u)` and sideslip `asin(v / V)`.

- Lift: `CL = CL0 + CL_alpha alpha + CL_q (q l/V) + CL_dm delta_m`.
- Drag: a parabolic polar `CD = CD_min + k CL^2`, with `k` and `CD_min` chosen so that the
  model's slope at trim equals the file's `CX_alpha`. This gives induced drag that grows
  with load factor in turns, which a straight linearization would not.
- Pitching moment, side force, rolling and yawing moments: linear in angle of attack,
  sideslip, non-dimensional rates and control deflections, exactly as in the file.
- The trim values come from the file's speed derivatives: `CL0 = -CZ_u / 2` and
  `CD0 = -CX_u / 2`.

By construction, linearizing this model about trim returns the linear lateral and
longitudinal models. That is the main check on it.

**Thrust.** Along the body x axis through the CG, commanded directly in newtons between
zero and a maximum. At trim it equals the drag.

**Controls.** Elevator, aileron and rudder, limited to a maximum deflection.

**Integration.** Fixed-step fourth-order Runge-Kutta, so a discrete autopilot can run
between steps. Convergence is checked by halving the step.

## Alternatives considered

- Extending the linear models with `sin(phi)` only: enough to match Case Study 7's
  figures, not enough to fly a course.
- Euler angles instead of a quaternion: simpler, but singular at 90 deg of pitch, which
  extreme manoeuvres can reach.
- Tabulated nonlinear aerodynamics (lift curve with stall): no such data exists for DF1.
- An adaptive integrator (`solve_ivp`): awkward with a sampled controller and gives
  non-uniform output for playback.

## Limits

- **No stall.** Lift stays linear in angle of attack at any angle. Samples outside a
  stated range of angle of attack and sideslip are flagged, not corrected.
- Derivatives are those of one trim condition; they are assumed not to change with speed
  or load factor (apart from dynamic pressure and the drag polar).
- No wind, no ground contact, no propeller or gyroscopic effects, no actuator dynamics.
- Thrust is a commanded force with a ceiling, not a motor and propeller model.

## The course autopilot

`src/simulate/autopilot.py`. It stands in for the pilot, so its gains are choices made
here and lap times depend on them as well as on the aircraft.

- **Guidance:** a sequence of phases per lap (upwind, 180 deg turn, downwind, 360 deg
  turn, downwind, 180 deg turn). Straights follow a line with a cross-track correction;
  turns steer to a heading target.
- **Bank:** commanded from the wanted turn rate, limited to the bank that gives the
  load-factor limit in a level turn (60 deg for 2), and rate-limited to 120 deg/s.
- **Lift:** the load factor needed for the bank, plus a correction for altitude, sets a
  target angle of attack; the elevator is trimmed for it and corrected on angle of attack
  and pitch rate.
- **Rudder:** proportional to sideslip. **Thrust:** drag estimate plus speed error.
- Course layout, speed, altitude and limits are data: `data/courses/dbf_2026_m1.toml`.

Assumptions in that file that are not from the rules or the report: flight altitude
(30 m), and maximum thrust taken as the best static thrust with no loss at speed. The
direction of the turns follows the report's figure.

## Results

Run `python3 -m cases.df1_course`.

**The model reduces to the linear ones.** Linearizing numerically about trim returns all
eight linear eigenvalues (short period, phugoid, roll, Dutch roll, spiral) to better than
one part in 10^9. Released from a 1 deg angle-of-attack or sideslip disturbance, it matches
the linear simulation to 0.03% of the peak response. From 60 deg of bank the two diverge,
as they should.

**DF1 Mission 1 on the competition course** (lattice derivatives, distributed-mass
inertia, three laps):

| | |
|---|---|
| Lap time | 44.4 s each (133.1 s total), identical to within 0.01 s |
| Speed | 27.9 m/s throughout |
| Altitude | 29.6 to 30.1 m against a 30 m target |
| Bank and load factor | 61 deg, 2.08 at most |
| Turn radius | 44 m (steady-turn formula for 60 deg: 46 m) |
| Angle of attack, sideslip | up to 2.7 deg and 1.7 deg |
| Control deflections | elevator 3.6 deg, aileron 4.1 deg, rudder 1.7 deg at most |
| Thrust | 2.3 to 3.7 N |
| Time outside the model's valid range | none |

**Step size.** Total time for three laps: 133.000 s at 0.02 s steps, 133.090 s at 0.01 s,
133.125 s at 0.005 s.

**What this does and does not show.** The aircraft, as modelled, flies the course with
large margins: small control deflections, low angle of attack, thrust far below the
ceiling. The lap time follows almost entirely from the chosen speed and load-factor limit
(the course is about 1.24 km round at that turn radius), so it is not a prediction of
competition lap time: that depends on how fast the real aircraft can fly and how hard the
pilot turns. Nothing here has been compared with flight data.

## Follow-ups

- Manoeuvre mode: scripted large inputs, where the validity flags will matter.
- Thrust that falls with speed; wind; takeoff and landing.
- Missions 2 and 3.
