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

Run `python3 -m cases.df1_performance`. DF1 Mission 1 (2.973 kg), as-built geometry
(the fin at the design report's height; this changes the numbers below by 0.1 s at most).

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
static test. The Mission 1 flight log has since been examined
(`docs/flight-validation.md`): the level top speed from flight is about 29.1 m/s at that
day's air density, where the model gives 29.8 m/s. The same data suggest the model has
too much excess thrust at 23-25 m/s, on thin evidence.

### Envelope

- Stall 9.6 m/s, corner speed 20.5 m/s, top speed 29.6 m/s.
- Best sustained turn: 118 deg/s at 20.1 m/s (load factor 4.3, radius 10 m), about 9 m/s
  below top speed. At 95% of top speed only 2.6 g can be sustained.
- Above the corner speed the structure limits the turn; below it, maximum lift does.
  Thrust limits what can be held without losing speed.

### Laps of the course, by structural limit and by turn strategy

Second lap of two, so the lap starts and ends at speed. Each row sets the structural
load-factor limit and lets the autopilot turn up to it. Two strategies: full throttle
everywhere, and closing the throttle before each turn to enter at a chosen speed, holding
it through the turn, then full throttle. "Best entry" is the fastest of the entry speeds
tried (14 to 28 m/s); "corner" is the speed at which the lift the autopilot allows itself
just reaches its turn target.

| Limit | Full throttle: lap | slowest | battery | Slowing: best entry | corner | lap | saved | battery |
|---|---|---|---|---|---|---|---|---|
| 2.0 g | 45.8 s | 28.9 m/s | 0.44 Ah | 14 m/s | 14.3 | 38.6 s | 7.2 s | 0.22 Ah |
| 2.5 g | 39.8 s | 28.4 m/s | 0.38 Ah | 16 m/s | 16.0 | 36.2 s | 3.6 s | 0.23 Ah |
| 3.0 g | 36.5 s | 27.5 m/s | 0.36 Ah | 17 m/s | 17.5 | 34.6 s | 1.9 s | 0.24 Ah |
| 3.5 g | 34.4 s | 26.6 m/s | 0.34 Ah | 19 m/s | 18.9 | 33.5 s | 1.0 s | 0.25 Ah |
| 4.0 g | 33.1 s | 25.7 m/s | 0.33 Ah | 21 m/s | 20.2 | 32.5 s | 0.6 s | 0.26 Ah |
| 4.5 g | 32.1 s | 24.5 m/s | 0.32 Ah | 23 m/s | 21.4 | 31.7 s | 0.4 s | 0.28 Ah |

At the 4.5 g limit the optimum is flat: entry speeds from 21 to 26 m/s are all within
0.1 s of the best. Slowing to 22 m/s shortens the lap's path from 911 m to 826 m and the
time spent turning from 9.7 s to 8.3 s, at the cost of 5.4 s per lap with the throttle
closed.

What this says:

- **The load-factor limit is the main lever.** At full throttle, raising it from 2 g to
  3 g saves 9 s a lap; from 3.5 g (the report's limit) to 4.5 g saves 2.3 s. A 20% thrust
  increase (removing the calibration) saves 0.5 s.
- **Slowing for the turns is faster, and the best entry speed is the corner speed.** In
  every row the best entry speed found lies within about 1.5 m/s of the corner speed for
  that limit. This is the textbook result, and here it comes out of the simulation
  rather than being put in.
- **It pays more the lower the limit.** At 4.5 g it is worth 0.4 s (1%); at the report's
  3.5 g, 1.0 s (3%); at 2 g, 7 s (16%). A structurally limited aircraft gains most from
  being flown this way.
- **It is always much lighter on the battery**: 14% less per lap at 4.5 g, 26% at 3.5 g,
  about half at 2 g. For a mission scored on laps in a time window with a battery limit,
  that matters more than the time.
- **Lift is not the limit on this course at full throttle** (peak lift coefficient 0.95 of
  1.41), and **energy is not the limit for Mission 1** (a lap uses about a tenth of the
  3.57 Ah usable).
- **The report's predicted lap time of 35.0 s** (Table 5.5.1) corresponds here to a limit
  of about 3.3 g flown at full throttle.

Caveats specific to this comparison:

- With the throttle closed the propeller is taken to give zero thrust. A windmilling
  propeller adds drag, so the real aircraft slows down faster than modelled, which would
  favour the slowing strategy a little more.
- In the slow-entry laps altitude sags by up to 4 m in the turns, and at the lower limits
  the peak load factor overshoots the limit by up to 2.5%. The autopilot is not tuned for
  low-speed turns.
- The differences between strategies at 4.5 g (0.4 s) are small compared with what a
  different pilot line or autopilot tuning would change. The differences at lower limits
  are large enough to be robust to that.

Step-size check on the 4.5 g full-throttle lap: 32.06 s at 0.01 s steps, 32.08 s at
0.005 s.

### What this does not show

Level top speed has been checked against one flight log and agrees. Nothing here is
checked against a measured lap. The autopilot's
roll-in and roll-out rates are choices, and a pilot who flies a different line will get a
different time. The flight log on hand (paused, see `docs/flight-validation.md`) records
speeds mostly between 20 and 28 m/s, which is at least not in conflict with a top speed
near 30 m/s.

## Follow-ups

- Missions 2 and 3: payload mass, the 14x7 propeller and 4000 mAh pack, banner drag.
- Scoring and the five-minute window.
- Design sweeps: mass, CG, propeller, wing area against lap time and handling.
- Windmilling-propeller drag with the throttle closed; trading height for speed in turns.
- Battery voltage that falls with discharge; measured motor constants.
