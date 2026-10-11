# Validation against flight data

Living record of how the tool's predictions compare with measurements from the real
aircraft. Everything else in this repository is verified against other models (the course
case study and XFLR5); this is the only check against reality.

**Status, 2026-10-11: paused until more flight data is available.** Ishan will fly
dedicated test manoeuvres once back at school; until then nothing further is built on the
existing log. No model-versus-flight comparison has been made yet. One flight log has been
examined. It cannot confirm the predicted modes, for the reasons below. What
it can support, and what is needed for a real comparison, is set out here so the gap is
explicit rather than implied.

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

- Control derivatives in the tool (phase 3).
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
