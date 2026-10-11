# Validation against flight data

Living record of how the tool's predictions compare with measurements from the real
aircraft. Everything else in this repository is verified against other models (the course
case study and XFLR5); this is the only check against reality.

**Status, 2026-10-11: two comparisons made.** Level top speed from the Mission 1 log
agrees with the model (29.1 m/s measured, 29.8 m/s modelled). Energy for a reported
Mission 2 flight does not (model about a quarter low), and the cause is not in the
aerodynamics. Mode validation is still waiting for dedicated test manoeuvres. Ishan will fly
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
| Level top speed, Mission 1 (flight log) | about 29.1 m/s (65 mph) | 29.8 m/s | Agrees within 3%, on about 7 s of data |
| Excess power at 23-25 m/s, full throttle (flight log) | about +5 m/s | +8 to +10 m/s | Model about twice the flight; thin data |
| Mission 2, eight laps, whole flight | pack at about 2%: about 4.4 Ah | about 3.2 Ah | Model about a quarter low; not explained by speed or drag |
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

**An explanation that was considered and then ruled out: a slower aircraft.** The motor's
current at full throttle depends strongly on airspeed, because the propeller unloads as
speed rises: in the model 50 A at 20 m/s, 45 A at 25 m/s, 38 A at 30 m/s. A draggier
aircraft that topped out near 26-27 m/s would use about 4.0 Ah for this flight:

| Change to the model (3.0 g, 5 m/s wind) | Top speed | 8 laps done at | After landing | Share of reported |
|---|---|---|---|---|
| As modelled | 31.1 m/s | 289 s | 3.23 Ah | 73% |
| Zero-lift drag +50% | 28.2 m/s | 295 s | 3.69 Ah | 84% |
| Zero-lift drag doubled | 26.1 m/s | 304 s | 4.06 Ah | 92% |
| Thrust factor 0.65 instead of 0.80 | 29.6 m/s | 293 s | 3.51 Ah | 80% |
| Drag +50% and thrust factor 0.65 | 26.7 m/s | 303 s | 4.02 Ah | 91% |

But the Mission 1 flight log, examined afterwards (next section), puts the level top speed
at about 29 m/s, within 3% of the model. The aircraft is not materially slower than
modelled, at least in the Mission 1 configuration, so this is not the explanation.
Reshaping the model's thrust curve to match the flight's lower excess power at 23-25 m/s
(next section) changes the Mission 2 energy by only 0.1 Ah.

**What is left.** With speed and drag confirmed, the gap has to be on the electrical side
or in how the flight differed from the modelled one:

- Current in flight may be higher than the propeller data and motor model give. At rest
  the model's current is within -9% to +14% of the four static tests, so any error would
  have to appear in flight, at the higher advance ratios where only the manufacturer's
  computed propeller data are available.
- Losses the model leaves out (speed controller, wiring, motor losses that rise with RPM).
  A pack that came down hot is consistent with that.
- The pack may have delivered less than its rated 4.5 Ah. A reading of 2% taken from
  voltage right after a high-current flight does not by itself show that 4.4 Ah was
  drawn; hot, heavily loaded packs commonly give 80-90% of rating.
- The flight may have been longer than the modelled one: extra time before the first lap,
  a wider pattern, a go-around.

**Verdict.** The model under-predicts the energy used on this flight by about a quarter,
and the cause is not identified. It is not top speed or drag. No calibration has been
changed on the strength of this. For planning, treating the model's energy figures as
about 25% optimistic is the cautious reading.

**What would settle it:**

- Current logged in flight (a power module on the Pixhawk, or a watt-meter). The existing
  log has no battery current in it.
- The mAh put back into the pack on recharge.
- Lap times and the time from throttle-up to landing.

**A side result worth knowing for Mission 2.** In the model, slowing to the corner speed
before each turn cuts the energy for the flight by about 30% at a 3 g limit (3.23 Ah to
2.26 Ah) while being slightly faster. Whatever the cause of the energy gap, the saving
should carry over in proportion, since it comes from flying a shorter path at lower power.

## Level top speed and excess power from the Mission 1 log

Done on 2026-10-11 at Ishan's request, after he recalled a top speed of about 85 mph.
Reproduce with:

    python3 -m cases.df1_flight_top_speed

**The 85 mph was a dive.** The highest airspeed in the log is 36.4 m/s (81 mph), at full
throttle while descending at about 11 m/s with the nose 14-24 deg down.

**Method.** The pilot never held full throttle in steady level flight (0.2 s in the whole
log), so top speed cannot be read directly. Instead every full-throttle sample near
wings-level (bank under 25 deg) and near 1 g (within 0.25) gives the specific excess
power, `Ps = dh/dt + (V/g) dV/dt`: the rate of gaining height plus the rate of gaining
speed expressed as height. `Ps` is zero at the level top speed. The model's value is
`(thrust - drag) V / weight`, evaluated at the flight's air density, 1.107 kg/m^3.

**Data.** Full throttle for 34 s of 219 s airborne; 7 s of that passes the filters.

| Airspeed | Samples (10 Hz) | Flight, median Ps | Middle half | Model Ps |
|---|---|---|---|---|
| 23.0 m/s | 13 | +5.2 m/s | +5.0 to +5.9 | +9.9 m/s |
| 25.0 m/s | 14 | +4.8 m/s | +3.6 to +6.8 | +7.7 m/s |
| 27.0 m/s | 15 | +1.3 m/s | -5.7 to +5.5 | +4.9 m/s |
| 31.0 m/s | 10 | -1.4 m/s | -4.0 to -0.4 | -2.5 m/s |
| 32.6 m/s | 9 | -3.5 m/s | -4.6 to -1.5 | -6.3 m/s |

**Result.**

- **Level top speed: about 29.1 m/s (65 mph) from the flight, 29.8 m/s from the model.**
  Without the 1 g filter the flight value is 28.3 m/s. This is the first agreement between
  the performance model and measured flight data, and it supports the model's combination
  of the report's drag (CD0 = 0.059) and the thrust calibrated to the static test, over
  the report's own top speed of about 34 m/s.
- **The model has about twice the flight's excess power at 23-25 m/s** (+8 to +10 m/s
  against about +5 m/s), and somewhat less than the flight above top speed. The real
  thrust-minus-drag curve looks flatter than the model's. If real, the aircraft
  accelerates out of turns and climbs more slowly than modelled.

**How far to trust it.** Seven seconds of data, in bins of about one second each, from a
flight not flown for this purpose. Airspeed is logged at 5 Hz and differentiated. The top
speed is supported by the sign change of `Ps` between 27 and 31 m/s and is probably good
to about 2 m/s. The low-speed discrepancy rests on two bins and should be treated as a
question, not a finding. The log is Mission 1 with the 15x6 propeller; nothing here tests
Mission 2 or the 14x7 propeller.

**The flight test that would pin this down:** in Manual mode, level, slow to about
15 m/s, then full throttle held for 10-15 s without climbing. That one pass gives the
whole excess-power curve and the top speed.

## What the tool predicts for this flight

Mission 1, as-built geometry (the fin at the design report's 10.24 in) with the
distributed-mass inertia. Coefficients are held at their trim values and only speed and
density are changed to the flight's:

| | At trim (27.9 m/s, 1.225) | At flight median (24.2 m/s, 1.107) | Over the flight's speed range |
|---|---|---|---|
| Dutch roll frequency | 2.03 Hz | 1.68 Hz | 1.08 to 2.53 Hz |
| Dutch roll damping ratio | 0.21 | 0.20 | about 0.20 |
| Short-period frequency | 2.79 Hz | 2.27 Hz | 1.45 to 3.43 Hz |
| Short-period damping ratio | 0.59 | 0.57 | about 0.57 |
| Roll time constant | 0.040 s | 0.051 s | 0.034 to 0.080 s |

With the XFLR5 model's shorter fin the Dutch roll would be 1.17 Hz at trim and 0.94 Hz at
the flight's median speed (`docs/decisions/2026-10-11-df1-fin-height-discrepancy.md`).

These are indicative only. Holding the coefficients fixed while changing speed is not a
proper re-trim: lift coefficient, and with it several derivatives, changes with speed. The
phugoid and spiral are left out of the table because they depend directly on that.

## Why this log cannot confirm the modes

1. **No free response.** The pilot was on the sticks throughout; there is no window of
   3 s or more with all three held still.
2. **No Dutch-roll peak stands out.** The strongest components of the measured body
   rates are pilot inputs at 0.4 to 0.8 Hz. The predicted Dutch roll moves between about
   1.1 and 2.5 Hz as speed changes through the flight (0.6 to 1.4 Hz with the XFLR5
   model's shorter fin), so it would be smeared across the spectrum, and the yaw-rate
   spectrum falls off smoothly with no peak at either. The log neither supports nor
   contradicts either fin height.
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
