# DF1: the XFLR5 model's fin is about half the height in the design report

Date: 2026-10-11. Status: **resolved the same day. Ishan decided to use the design report's
geometry for the fin.**

## Finding

| | Height | Chord | Area |
|---|---|---|---|
| XFLR5 plane files (all three missions) | 0.133 m (5.24 in) | 0.146 m (5.75 in) | 30 in^2 |
| Design report Table 5.1.1 (p.29) | 10.24 in | 5.75 in | 58.88 in^2 |

The report's three-view (p.42) also shows a fin roughly 10 in tall standing on the
tailplane. A possible cause: 0.133 m entered as a half-span, as for the wing and
tailplane, while XFLR5 does not mirror a fin.

## Why it matters

Everything derived from the XFLR5 model inherits the short fin: the XFLR5 logs, the
design report's dynamic-stability table, and this tool's DF1 geometry files (imported from
XFLR5 so that comparisons with XFLR5 are like for like).

Built-in lattice, Mission 1, distributed-mass inertia, changing only the fin height:

| | XFLR5 fin (5.24 in) | Report fin (10.24 in) | Ratio |
|---|---|---|---|
| CYb | -0.104 | -0.292 | 2.8 |
| Cnb | 0.063 | 0.191 | 3.0 |
| Cnr | -0.097 | -0.285 | 3.0 |
| Clb | -0.007 | -0.039 | 5.5 |
| Rudder yawing power (per rad) | -0.051 | -0.139 | 2.7 |
| Dutch-roll frequency | 1.17 Hz | 2.03 Hz | 1.7 |
| Dutch-roll damping ratio | 0.13 | 0.21 | 1.6 |
| Spiral time constant | -18.5 s | -18.8 s | 1.0 |
| Roll time constant | 0.041 s | 0.040 s | 1.0 |

Doubling the height also doubles the aspect ratio, which is why the derivatives rise by
about three rather than two.

## Consequence

The tool's verification against XFLR5 is unaffected: both use the same geometry. But the
DF1 lateral-directional results (Dutch roll, rudder response, anything fin-driven) do not
describe the aircraft as built if the report's fin is the real one. Longitudinal results
and the roll mode are not affected.

## Decided

Use the design report's fin height, 10.24 in (0.2601 m), for everything meant to describe
the aircraft. Two sets of DF1 geometry now exist:

- `data/geometry/df1_m*.toml`: exactly as in the XFLR5 plane files (5.24 in fin). Used
  only to compare the tool with XFLR5, where the geometry must be the same on both sides.
- `data/geometry/df1_m*_asbuilt.toml`: the same import with the fin's tip at 0.2601 m
  (`src.aero.xflr5_plane --tip-span "Fin=0.2601"`). The rudder still spans the whole fin.
  The aircraft files `data/aircraft/df1_m*_asbuilt.toml` are computed from these by the
  built-in lattice, with the distributed-mass inertia, and are what the course flights,
  performance limits, manoeuvres and natural modes use.

The earlier `df1_m*_vlm_distributed.toml` files (short fin, corrected inertia) are removed.

## What did not change

- Mass, centre of gravity and inertia are still those of the XFLR5 model plus the mass
  distribution. A taller fin of the same mass moves the CG and inertias very slightly;
  that is not modelled.
- The design report's own dynamic-stability and controllability tables came from the
  short-fin XFLR5 model, so its Dutch-roll and rudder values will not match the as-built
  results here, and the tool's check against the report's rudder derivatives is still made
  on the XFLR5 geometry.
- Longitudinal results, the roll mode, top speed and lap times are essentially unchanged
  (the full-throttle 4.5 g lap moves from 32.06 s to 32.10 s).

## What did change (Mission 1)

Dutch roll 1.17 Hz to 2.03 Hz, damping ratio 0.13 to 0.21; rudder yawing power 2.7 times
larger; yaw stiffness and yaw damping about 3 times larger. In the manoeuvres, the same
6 deg rudder doublet now gives 8 deg of sideslip instead of 13 deg, and the aileron roll
picks up 1.5 deg of sideslip instead of 5.9 deg.

## Still unverified

Nothing measured confirms either fin height's dynamics. The one flight log shows no
Dutch-roll peak at either frequency, which is expected in a flight with varying speed and
continuous pilot input.
