# DF1 inertia corrected with a component mass distribution

Date: 2026-10-11

## Problem

The XFLR5 models carry every airframe component except the tail surfaces as a point mass
on the centreline (y = 0). Roll inertia was therefore almost entirely missing: Mission 1
had Ixx = 0.011 kg m^2 and a roll time constant of 7 ms.

## Decided

Keep XFLR5's mass, centre of gravity and aerodynamic derivatives, and add only the inertia
that the point masses omit. `data/aircraft/df1_mass_distribution.toml` gives a physical
extent for the main wing, fuselage, tail boom and main gear, from the design report
(Tables 5.1.1, 5.3.1 and the three-view). Each is treated as a uniform box centred on its
existing point mass, so mass and CG are unchanged by construction. The converter
(`src.aircraft.xflr5 --spread`) adds the box inertias to XFLR5's body-axis inertia and
rotates to stability axes.

Both versions of each mission are kept:

- `df1_m*.toml` — exactly as XFLR5 analysed it. Used to verify the solver against XFLR5's
  eigenvalues.
- `df1_m*_distributed.toml` — corrected inertia. Used for results meant to describe the
  aircraft. There is no independent reference for these.

## Effect (Mission 1)

| | XFLR5 inertia | Distributed |
|---|---|---|
| Roll inertia A | 0.0118 kg m^2 | 0.0610 kg m^2 |
| Yaw inertia C | 0.1742 kg m^2 | 0.2436 kg m^2 |
| Roll time constant | 0.007 s | 0.041 s |
| Dutch roll frequency | 8.42 rad/s | 7.14 rad/s |
| Dutch roll damping ratio | 0.152 | 0.129 |
| Spiral time to double | 12.4 s | 12.4 s |

The wing accounts for most of the change.

## Alternatives considered

- Rebuild the whole mass model from the report's weight table: rejected because the
  derivatives and trim point were computed by XFLR5 at its own mass and CG, and Missions 2
  and 3 do not match the report (below). A rebuilt model would be inconsistent with the
  aerodynamics.
- Edit the XFLR5 models and re-export: the right long-term fix, but needs XFLR5.

## Limits and open items

- Still a lower bound on roll inertia: "wing integration + spar" (0.337 kg) stays a point
  because the report does not split the central piece from the full-span spar, and the
  main gear is a uniform bar with no wheel masses at the tips.
- Uniform-box assumptions for the wing and fuselage are approximations, not measurements.
- **Mission 2 XFLR5 mass is 0.5 kg above the report** (3.779 kg vs 7.23 lb = 3.279 kg). The
  model has the wing twice: a 0.398 kg point mass plus a 0.500 kg volume mass.
- **Mission 3 XFLR5 mass is 0.37 kg above the report** (3.709 kg vs 7.37 lb = 3.343 kg): a
  0.300 kg "M3 ballast" that is not in the report's table, and a 0.300 kg banner where the
  report lists 0.51 lb (0.231 kg).
- In both, the battery is at the Mission 1 position (x = -0.062 m), where the report moves
  it (-3.58 in for M2, -4.28 in for M3).
  These change trim speed and CG-dependent derivatives, so they need an XFLR5 re-run.
- The report's Table 4.4.4 gives Mission 1 control derivatives from XFLR5 (aileron rolling
  power, rudder side force and yawing power, per degree). Not yet used; only free
  responses are simulated for DF1.
