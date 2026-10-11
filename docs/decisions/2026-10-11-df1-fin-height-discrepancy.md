# DF1: the XFLR5 model's fin is about half the height in the design report

Date: 2026-10-11. Status: **open, awaiting confirmation from Ishan of the as-built fin.**

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

## Not yet done

No corrected geometry file has been created. Once the as-built fin is confirmed, add an
as-built geometry alongside the XFLR5-matched one and use it for results about the
aircraft, keeping the XFLR5-matched files for verification.
