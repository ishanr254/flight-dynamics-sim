# Manoeuvre mode: inputs, the manoeuvre pilot, and what gets flagged

Date: 2026-10-11

## Decided

**Two ways to fly a manoeuvre** (`src/simulate/manoeuvre.py`):

- *Scripted controls*: steps, pulses and doublets of elevator, aileron, rudder and
  throttle added to the trim settings, with nothing correcting the result. This shows the
  aircraft's own response.
- *A manoeuvre pilot*: a small feedback controller given a schedule of bank-angle and
  load-factor targets. Used where the manoeuvre is defined by what the aircraft should do
  (hold 4 g, reverse a turn), not by a fixed stick position.

**Every sample carries two flags**, shown in the viewer as they happen:

1. *Outside the aerodynamic model's valid range.* The aerodynamics are linear about one
   trim point. A sample is flagged when any of these holds:
   - angle of attack beyond the angle at which the lift limit is reached (or outside a
     default range when no lift limit is known);
   - sideslip beyond 15 deg;
   - lift coefficient at its cap (the model's flat-topped stand-in for the stall);
   - roll helix angle `p b / 2V` beyond 0.12, where the wing tips see a change in angle of
     attack of about 7 deg from the roll alone.
2. *Over the structural load limit*: load factor beyond the limit in the performance file,
   in either direction.

A flagged sample is still simulated. The flag says the number should not be believed (or,
for the structural flag, that the manoeuvre would break the aircraft), not that the
simulation stopped.

**The second aircraft** (the Case Study 7 transport) has lateral data only, so it cannot
use the nonlinear model. Its scenarios come from the linear lateral model, with heading
and ground track rebuilt by kinematics at constant speed and height. This is the
small-disturbance picture and is labelled as such.

## Alternatives considered

- Stopping the simulation at the first flag: hides what the model does next, which is
  useful to see precisely because it is not to be trusted.
- A real stall model (lift loss, pitch break, wing drop): no data for DF1.
- An interactive mode with live stick input in the browser: would move the physics into
  the page. The viewer stays a player of flights computed by the simulator.

## Limits

- The 15 deg sideslip and 0.12 helix-angle thresholds are conventional round numbers, not
  derived for this aircraft.
- Control effectiveness is constant: no loss at large deflection, no stall of the
  surfaces. Full-deflection manoeuvres are therefore optimistic, and are flagged only if
  they also drive the aircraft outside the ranges above.
- The manoeuvre pilot's gains are choices, as with the course autopilot.

## Results

Run `python3 -m cases.manoeuvres`. DF1 Mission 1 (as-built geometry, with the fin at the
design report's height) with the performance model, from level cruise at 27.9 m/s and 80 m.

### Manoeuvres

| Manoeuvre | How flown | What it reaches | Valid-range flag | Structural flag |
|---|---|---|---|---|
| Elevator to the stop | 30 deg up for 1 s | 8.25 g, 36 deg angle of attack | 25% of the time | 12% |
| Aileron roll | 10 deg for 1.4 s | 286 deg/s, one roll, 29 m height lost | none | none |
| Full aileron | 30 deg for 0.5 s | 811 deg/s | 17% (roll rate) | none |
| Rudder doublet | 6 deg each way, 0.5 s | 8 deg sideslip | none | none |
| Hard turn reversal | pilot: 75 deg bank each way, holding height | 4.24 g, height within 1 m | none | none |
| Loop | pilot: 3.8 g pull all the way round | over the top at 17.6 m/s, 27 m gained | 8% (lift limit at the top) | none |
| Approach to the stall | pilot: throttle closed, holding height | lift limit at about 9.7 m/s, then sinking | 44% | none |

What these show:

- **The flags fire where they should and only there.** Moderate manoeuvres (a 290 deg/s
  roll, an 8 deg sideslip, a 4.2 g reversal) raise none. Full elevator raises both; full
  aileron raises only the roll-rate one; the loop and the stall approach raise only the
  lift-limit one.
- **Full up elevator at cruise would break the aircraft**: 8.25 g against a 4.5 g limit,
  within a quarter of a second. The elevator has far more authority at cruise speed than
  the structure can take.
- **The loop is possible from level cruise at full throttle** but runs out of lift over
  the top, where speed is under 20 m/s.
- **Full-aileron roll rate (811 deg/s) is not believable**; the model's roll damping is
  linear and the tips would stall. The 10 deg roll (286 deg/s) is inside the valid range.

### Natural modes, from the nonlinear model

Released from a disturbance with controls fixed at trim: short period (5 deg angle of
attack), phugoid (4 m/s fast), roll (aileron pulse), Dutch roll (8 deg sideslip), spiral
(10 deg bank). The Dutch-roll period measured from the nonlinear run matches the linear
model's within 5%, the phugoid period is about 15 s, and the spiral diverges as the linear
model says (bank more than 2.5 times larger after 25 s).

### Second aircraft

The Case Study 7 transport's Dutch roll, spiral and rudder step from the linear lateral
model, with the path rebuilt by kinematics. Its span is not in the case study; 44 m is
assumed for drawing only.

### Viewer

Four groups in the picker (Course, Manoeuvres, Natural modes, second aircraft), up to
three time traces per scenario with a cursor, a second warning for the structural limit,
and a suggested playback speed per scenario (slow motion for the fast modes, faster than
life for the slow ones). The data file is 2.0 MB for 19 scenarios; long or slow flights
are sampled at 5-20 samples per second.

## Follow-ups

- A real stall model, if lift-curve data past the stall ever exists for DF1.
- Control effectiveness that falls off at large deflection.
- Manoeuvres for Missions 2 and 3.
