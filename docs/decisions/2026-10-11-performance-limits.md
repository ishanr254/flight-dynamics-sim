# Performance limits: propulsion, drag, stall and structure

Date: 2026-10-11

## Decided

Performance data lives in its own file per aircraft configuration
(`data/performance/*.toml`), separate from the aircraft file. The aircraft file describes
stability and control about one trim point; the performance file describes how fast, how
hard and how long the aircraft can fly.

**Propulsion** (`src/propulsion`), ported from `sizing-scoring-26-27/src/prop`:

- Propeller: thrust and torque from APC's published coefficient tables, `Ct(J)` and
  `Cp(J)` at the nearest tabulated RPMs, with `J = V / (n D)`.
- Motor: `torque = Kt (I - I0)` with `Kt = 60 / (2 pi Kv)`; back-EMF `rpm / Kv`; winding
  resistance `Rm`. `Rm` and `I0` come from the sizing repository's regression on Kv and
  rated power, as no datasheet values are in the repository.
- Battery: nominal voltage 3.7 V per cell, internal resistance `0.013 / capacity` ohm per
  cell (the sizing repository's model), usable fraction 0.85.
- Operating point at a throttle and airspeed: the RPM at which the voltage the motor needs
  equals throttle times the sagged battery voltage. Current is capped by the fuse.

**Drag:** the design report's polar, `CD = CD0 + CL^2 / (pi AR e)` with `CD0 = 0.059` (CFD
and OpenVSP, section 4.4.7) and `e = 0.75`. This replaces, in the nonlinear model only,
the polar reconstructed from the aircraft file, whose drag came from XFLR5's viscous
estimate (about 0.01) and has no fuselage or landing gear.

**Stall:** lift coefficient is capped at `CL_max`, taken from the report's stall speed.
Beyond it lift stays flat and the sample is flagged as outside the model's valid range.

**Structure:** limit load factor 4.5 (Ishan's direction; the report states 3.5) and a
never-exceed speed from the report's V-n diagram.

**Flying at the limits:** full throttle throughout; in turns the autopilot targets the
smaller of a chosen turn load factor, the structural limit and what the wing can lift at
the current speed with a margin from stall.

## Alternatives considered

- Momentum-theory propeller instead of APC tables: no data needed, but poor at the low
  advance ratios where the static test is.
- Putting the report's drag into the aircraft file so linear and nonlinear models agree:
  would change the phugoid comparison against XFLR5, which is a like-for-like check of the
  dynamics code. Kept separate; the inconsistency is noted under Limits.
- A measured lift curve with a real stall: no such data for DF1.

## Limits

- The linear modes still use the low XFLR5-based drag, so phugoid damping from the linear
  model is lower than the performance drag implies.
- Flat-topped lift is not a stall model: no loss of lift, no pitch break, no roll-off.
- Propeller data are the manufacturer's computed tables, not wind-tunnel measurements.
- Motor constants are regression estimates, not Scorpion datasheet values.
- Battery voltage is nominal throughout; it does not fall as the pack discharges.

## Results

Run `python3 -m cases.df1_performance`. DF1 Mission 1 (2.973 kg).

### Models against test data and the report

| | Model | Reference | Diff | Reference is |
|---|---|---|---|---|
| Static thrust | 46.1 N | 46.0 N | +0.1% | static test (calibrated to this) |
| Static current | 55.0 A | 60.4 A | -9% | static test |
| Static electrical power | 1165 W | 1358 W | -14% | static test |
| Level drag at 27.7 m/s | 10.6 N | about 10.7 N | -1% | read from the report's drag polar |
| Top speed | 29.6 m/s | about 34.1 m/s | -13% | read from the report's climb-rate plot |
| Endurance at 27.7 m/s | 7.1 min | 6.0 min | +18% | report prediction |
| Stall speed | 9.65 m/s | 9.66 m/s | | not independent: CL_max was set from it |

**Thrust calibration.** Uncalibrated, the model gives 55.5 N static against 46.0 N
measured. The report's Figure 8.1.2 shows its own MATLAB and eCalc predictions exceeding
the same test by a similar amount, so this is the usual gap between a propeller's
published data and its installed thrust. A single factor of 0.83 is applied to thrust at
all speeds; that the static ratio holds in flight is an assumption. Without it the top
speed is 30.6 m/s instead of 29.6.

**Top speed.** The model's 29.6 m/s is below the 34 m/s where the report's climb-rate
curve reaches zero. The report's curve rests on a thrust model that over-predicted the
static test, which would explain most of the difference, but this has not been checked
against a measured level top speed.

### Envelope

- Stall 9.6 m/s, corner speed 20.5 m/s, top speed 29.6 m/s.
- Best sustained turn: 118 deg/s at 20.1 m/s (load factor 4.3, radius 10 m), about 9 m/s
  below top speed. At 95% of top speed only 2.6 g can be sustained.
- Above the corner speed the structure limits the turn; below it, maximum lift does.
  Thrust limits what can be held without losing speed.

### Full-throttle laps of the course

Second lap of two, so the lap starts and ends at speed. "Target" is the load factor the
autopilot aims for in the turns; "peak" is what occurred.

| Target | Lap time | Speed range | Peak load factor | Peak angle of attack | Battery per lap |
|---|---|---|---|---|---|
| 2.0 | 43.5 s | 28.8-29.6 m/s | 2.08 | 2.3 deg | 0.42 Ah |
| 2.5 | 38.3 s | 28.1-29.6 m/s | 2.63 | 3.9 deg | 0.37 Ah |
| 3.0 | 35.4 s | 27.0-29.6 m/s | 3.21 | 6.0 deg | 0.35 Ah |
| 3.5 | 33.5 s | 25.6-29.6 m/s | 3.88 | 9.0 deg | 0.33 Ah |
| 4.0 | 32.2 s | 24.2-29.6 m/s | 4.44 | 12.2 deg | 0.32 Ah |
| 4.5 | 32.1 s | 24.5-29.6 m/s | 4.45 | 11.9 deg | 0.32 Ah |

What this says:

- **How hard the turns are flown matters far more than top speed.** Going from 2 g to 3 g
  turns saves 8 s a lap; removing the thrust calibration (a 20% thrust increase) saves
  0.6 s on the hardest-turn lap.
- **Returns diminish above about 3.5 g.** From 3.5 to 4.5 the gain is 1.4 s, because the
  harder turn costs more speed (the aircraft slows to about 24.5 m/s) that then has to be
  won back on the straight. The report's 3.5 g structural limit would cost about 1.4 s a
  lap against the 4.5 used here.
- **The hard-turn laps exceed their target**: peak load factor runs about 10% over the
  target because holding altitude in the turn adds load. Targets of 4.0 and 4.5 therefore
  give the same lap, both capped by the 4.5 structural limit.
- **Lift is not the limit on this course.** Peak lift coefficient is 0.95 of 1.41.
- **Energy is not the limit for Mission 1.** A lap uses 0.32-0.42 Ah of 3.57 Ah usable,
  eight or more laps on a charge against three required. Harder turns use less energy per
  lap because the lap is shorter.
- **The report's predicted lap time of 35.0 s** (Table 5.5.1) corresponds here to turns of
  about 3 g.

Step-size check on the hardest lap: 32.10 s at 0.01 s steps, 32.08 s at 0.005 s.

### What this does not show

Nothing here is checked against a measured lap or a measured top speed. The autopilot's
roll-in and roll-out rates are choices, and a pilot who flies a different line will get a
different time. The flight log on hand (paused, see `docs/flight-validation.md`) records
speeds mostly between 20 and 28 m/s, which is at least not in conflict with a top speed
near 30 m/s.

## Follow-ups

- Missions 2 and 3: payload mass, the 14x7 propeller and 4000 mAh pack, banner drag.
- Scoring and the five-minute window.
- Design sweeps: mass, CG, propeller, wing area against lap time and handling.
- A smarter turn strategy (slowing toward the corner speed before a turn) to test whether
  the autopilot's "full throttle everywhere" is actually fastest.
- Battery voltage that falls with discharge; measured motor constants.
