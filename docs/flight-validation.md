# Validation against flight data

Living record of how the tool's predictions compare with measurements from the real
aircraft. Everything else in this repository is verified against other models (the course
case study and XFLR5); this is the only check against reality.

**Status, 2026-10-11: one comparison made (Mission 2 energy: the model is about a quarter
low); otherwise paused until more flight data is available.** Ishan will fly
dedicated test manoeuvres once back at school; until then nothing further is built on the
existing log. No model-versus-flight comparison has been made yet. One flight log has been
examined. It cannot confirm the predicted modes, for the reasons below. What
it can support, and what is needed for a real comparison, is set out here so the gap is
explicit rather than implied.

## Summary of comparisons with real data

| What | Real data | Model | Verdict |
|---|---|---|---|
| Static thrust, four tests | 37.5-46.0 N | 49.8-55.5 N uncalibrated | Model 20-33% high; corrected by one factor |
| Static current, four tests | 44.7-60.4 A | 50.8-55.0 A | Within -9% to +14% |
| Mission 2, eight laps, whole flight | pack at about 2%: about 4.4 Ah | about 3.2 Ah | Model about a quarter low; points at top speed being over-predicted |
| Mission 2 takeoff roll (report's prediction, not a measurement) | 7.55 m at 12.17 m/s | 7.46 m at 12.16 m/s | Agrees |
| Natural modes | none usable | | No comparison possible yet |
| Lap time, top speed | none measured | | No comparison possible yet |

## Data on hand

`data/flight/df1_log_178_2026-08-29.ulg` — one PX4 log from DF1 (Pixhawk 6C), flown
2026-08-29. Reproduce every number in this section with:

    python3 -m cases.df1_flight_log

| | |
|---|---|
| Airborne | 28.7 s to 257.3 s (about 229 s) |
| Flight modes (airborne) | Manual 109 s, Stabilized 100 s, Altitude 17 s |
| True airspeed | 15.5 to 36.6 m/s, median 24.2 m/s |
| Air density | 1.107 kg/m^3 |
| Bank angle (5th-95th percentile) | -55 to +58 deg; within 10 deg of level 30% of the time |
| Gyro, accelerometer | 200 Hz |
| Body rates, commanded control torques | 50 Hz |
| Attitude | 20 Hz |
| Servo outputs | 10 Hz |
| Airspeed, stick positions | 5 Hz |
| Hands-off windows of 3 s or more | none |

Not in the log, supplied by Ishan on 2026-10-11:

- Configuration: **Mission 1, empty** (so the Mission 1 files apply: 2.973 kg in the
  XFLR5 model and the design report).
- Control rates: believed to be the transmitter's **mid rates, about 75% of maximum
  deflection**, i.e. roughly +/-22 deg at full stick against the design report's +/-30 deg
  maximum. This is an assumption from memory, not a measurement.

Still unknown: measured takeoff weight and CG for this flight.

## Mission 2 flight: eight laps on one pack

Reported by Ishan on 2026-10-11, from memory: on a Mission 2 flight (slightly heavier
than Mission 1) the aircraft flew eight laps and "basically used up all the battery" of a
4500 mAh pack. In a later message the same day he added that the pack was down to about
2% and was really hot after the flight, so close to the full 4.5 Ah was drawn (about
4.4 Ah). There is no log, no recharge figure, and the flight time, lap times, wind and
throttle use are not known. The design report lists a 4000 mAh pack for Mission 2; the
4500 mAh figure is used here as reported. Reproduce with:

    python3 -m cases.df1_m2_energy_check

**What the model says.** The whole flight is simulated: takeoff roll from the start line,
eight laps at full throttle, then a landing circuit, at the report's Mission 2 weight
(3.279 kg), with the 14x7 propeller and a 5 m/s wind down the runway (assumed; the wind
that day is not known).

| Structural limit | Wind | 8 laps done at | Used by then | After landing | Share of the 4.4 Ah reported |
|---|---|---|---|---|---|
| 2.5 g | 5 m/s | 316 s | 3.28 Ah | 3.49 Ah | 79% |
| 3.0 g | 5 m/s | 289 s | 3.04 Ah | 3.23 Ah | 73% |
| 3.5 g | 5 m/s | 272 s | 2.89 Ah | 3.08 Ah | 70% |
| 4.5 g | 5 m/s | 254 s | 2.75 Ah | 2.93 Ah | 67% |
| 3.0 g | none | 291 s | 3.06 Ah | 3.24 Ah | 74% |
| 3.0 g | 8 m/s | 295 s | 3.10 Ah | 3.34 Ah | 76% |

**What agrees.**

- Eight laps fit inside the five-minute window in the model only if the turns are flown at
  about 3 g or harder, and the flight did complete eight.
- The takeoff roll in still air, 7.46 m to liftoff at 12.16 m/s, matches the report's
  7.55 m at 12.17 m/s.
- The model says eight laps take most of a pack, not a small part of one.

**What does not.** The flight used about 4.4 Ah. The model's whole flight uses about
3.2 Ah, three quarters of that. Takeoff, climb and landing account for only about 0.2 Ah
and wind for under 0.1 Ah, so they do not explain the gap. The model under-predicts the
energy used on this flight by about a quarter.

**The most consistent explanation: the real aircraft is slower than the model.** The
motor's current at full throttle depends strongly on airspeed, because the propeller
unloads as speed rises: in the model 50 A at 20 m/s, 45 A at 25 m/s, 38 A at 30 m/s. An
aircraft that tops out lower therefore draws more current as well as taking longer.

| Change to the model (3.0 g, 5 m/s wind) | Top speed | 8 laps done at | After landing | Share of reported |
|---|---|---|---|---|
| As modelled | 31.1 m/s | 289 s | 3.23 Ah | 73% |
| Zero-lift drag +50% | 28.2 m/s | 295 s | 3.69 Ah | 84% |
| Zero-lift drag doubled | 26.1 m/s | 304 s | 4.06 Ah | 92% |
| Thrust factor 0.65 instead of 0.80 | 29.6 m/s | 293 s | 3.51 Ah | 80% |
| Drag +50% and thrust factor 0.65 | 26.7 m/s | 303 s | 4.02 Ah | 91% |

A top speed near 26-27 m/s, against the model's 31 m/s, brings the energy to about 4.0 Ah
with eight laps finishing right at five minutes. That picture also fits the one flight
log on hand (a different flight, Mission 1), where airspeed sat mostly between 20 and
28 m/s. It would mean the real aircraft has noticeably more drag than the report's CFD
value, or less thrust in flight than the static calibration implies, or both.

**What this is not.** It is an inference from one flight remembered without a log, and
other causes cannot be ruled out:

- Electrical losses the model leaves out (speed controller, wiring, motor losses that rise
  with RPM) would raise current at every speed. A pack that came down hot is consistent
  with that as well as with simply being run at about 50 A.
- Battery voltage in the model is the nominal 22.2 V throughout; a fresh pack starts near
  25 V, which raises RPM and current early in the flight.
- A pilot's path is longer than the autopilot's.

**Verdict.** The structure of the model holds up (lap count, takeoff roll, order of
magnitude of energy), but it is optimistic for this flight by about a quarter on energy,
and the evidence points at top speed being over-predicted by roughly 15%. No calibration
has been changed on the strength of this.

**What would settle it:**

- A measured level top speed at full throttle. The existing Mission 1 log may already
  contain one; it has not been examined for this.
- Current logged in flight (a power module on the Pixhawk, or a watt-meter).
- Lap times and the time from throttle-up to landing.

**A side result worth knowing for Mission 2.** In the model, slowing to the corner speed
before each turn cuts the energy for the flight by about 30% at a 3 g limit (3.23 Ah to
2.26 Ah) while being slightly faster. If the real aircraft is draggier than modelled, the
saving is still there, since the turns are where the energy goes.

## What the tool predicts for this flight

Mission 1 with the distributed-mass inertia, coefficients held at their XFLR5 trim values
and only speed and density changed to the flight's:

| | At XFLR5 trim (27.8 m/s, 1.225) | At flight median (24.2 m/s, 1.107) | Over the flight's speed range |
|---|---|---|---|
| Dutch roll frequency | 1.14 Hz | 0.94 Hz | 0.61 to 1.42 Hz |
| Dutch roll damping ratio | 0.13 | 0.12 | about 0.12 to 0.13 |
| Short-period frequency | 2.76 Hz | 2.26 Hz | 1.45 to 3.41 Hz |
| Short-period damping ratio | 0.59 | 0.57 | about 0.57 |
| Roll time constant | 0.041 s | 0.052 s | 0.035 to 0.082 s |

These are indicative only. Holding the coefficients fixed while changing speed is not a
proper re-trim: lift coefficient, and with it several derivatives, changes with speed. The
phugoid and spiral are left out of the table because they depend directly on that. A
correct prediction at a given flight speed needs the lattice to trim at that speed, which
needs the elevator (phase 3 of the vortex-lattice plan).

## Why this log cannot confirm the modes

1. **No free response.** The pilot was on the sticks throughout; there is no window of
   3 s or more with all three held still.
2. **Pilot inputs sit on top of the Dutch roll.** The strongest components of the measured
   body rates are at 0.4 to 0.8 Hz. The predicted Dutch roll at the flight's speeds is
   0.6 to 1.4 Hz, so the two overlap and cannot be separated by looking at a spectrum.
3. **Large bank angles.** The aircraft was beyond 10 deg of bank for 70% of the flight,
   outside the range where the linear model applies.
4. **Closed-loop segments.** In Stabilized and Altitude modes (117 s) the autopilot's rate
   feedback changes the apparent damping, so those segments do not show the bare airframe.
5. **Unknown configuration.** Mass, CG and inertia for this flight are not recorded.

## Comparison this log can support (planned, not done)

Drive the model with the recorded commands and compare predicted with measured body rates,
using the Manual-mode segments (109 s), where commands pass straight to the surfaces.
Roll rate in response to aileron is the most promising: the roll mode is fast, so roll
rate should track aileron deflection closely, and the ratio tests the predicted roll
control power against roll damping.

Needed first:

- Control derivatives in the tool: done 2026-10-11
  (`docs/decisions/2026-10-11-control-surfaces.md`). The comparison itself has not been
  started; this work is paused.
- Surface deflection per unit command: use about 75% of the +/-30 deg maximum (see above),
  and treat the result as uncertain by at least that assumption.
- Measured weight and CG, if they can be recovered; otherwise the Mission 1 model values.
- A way to restrict the comparison to small bank angles, or to accept the linear-model
  error outside them.

What it will show: whether the predicted response to control inputs has the right size
and timing. What it will not show: mode frequencies and damping in isolation.

## Flight test that would validate the modes

In Manual mode, steady level flight at a known speed, three manoeuvres, each followed by
5 to 10 s hands off (30 s or more for the last):

| Manoeuvre | Mode excited | What is read from the log |
|---|---|---|
| Sharp elevator pulse | Short period | Pitch-rate decay: frequency and damping |
| Rudder kick or doublet | Dutch roll | Yaw- and roll-rate oscillation: frequency and damping |
| Slow about 2 m/s, release | Phugoid | Airspeed and pitch oscillation: period and damping |
| Small bank, release | Spiral | Whether bank grows or decays, and how fast |

Also record takeoff weight and CG, and raise the stick and servo logging rates above the
current 5 and 10 Hz.

## Requirements this feeds

R3 (mode predictions against an independent reference) and R4 (time response against
reference) are currently met only against other models. Flight data would give both an
independent physical reference. R1 (motion consistent with real-world behaviour) is
currently a qualitative judgement and would gain a measured basis.
