# Takeoff, landing and wind

Date: 2026-10-11

## Decided

**Wind.** A steady, uniform velocity of the air mass over the ground, given as (north,
east). The aircraft's body velocities are relative to the air, so its dynamics are
unchanged; only its path over the ground moves. No gusts, no wind gradient near the
ground.

**Takeoff** (`src/simulate/mission.py`), following the competition rules (a rolling
takeoff from the runway, with the flight window starting at throttle-up):

- From rest at the start line, heading upwind, full throttle.
- The ground roll is one-dimensional: thrust from the propulsion model, drag and lift at
  the ground attitude (zero angle of attack), rolling friction 0.04 on the weight not
  carried by the wing.
- Liftoff at 1.2 times stall speed, the design report's convention (eq. 25). The aircraft
  is then handed to the nonlinear model, level, at the angle of attack that carries its
  weight, and the autopilot climbs to pattern height (up to 5 m/s) while flying lap one.
- Time zero is throttle-up, so lap times count from the start of the flight window. The
  first lap therefore includes the takeoff.

**Landing**, after the last lap is counted:

- One more circuit without the 360 deg turn.
- From the first downwind leg the autopilot slows to 1.3 times stall speed.
- From abeam the start line it descends on a 5 deg glide slope measured along the path to
  a touchdown point 60 m before the start line, on the runway.
- The turn onto final uses the bank that gives the radius needed to roll out on the
  runway line.
- Touchdown when below 0.15 m on final; then a throttle-closed roll-out with the same
  rolling friction until stopped.

## Alternatives considered

- Ground contact inside the six-degree-of-freedom model (gear forces, rotation): much
  more work, and nothing downstream needs it yet.
- A flare before touchdown: omitted; the aircraft arrives at the glide-slope sink rate,
  about 1 m/s.
- Braking on the roll-out: the aircraft has none.

## Checks

- **Takeoff roll against the report.** Mission 2 in still air: 7.46 m to liftoff at
  12.16 m/s. The report's Table 4.4.5 gives 7.55 m (24.78 ft) at 12.17 m/s (39.94 ft/s).
  The two share the liftoff-speed convention but use independent thrust and drag models.
- Wind shifts the ground track by exactly wind times time and leaves airspeed alone.
- The landing touches down on the runway within 5 m of its line, at about approach speed,
  and stops on the runway.
- A small share of samples (under 1%) sit at the lift limit during climb-out or approach
  and are flagged as outside the model's valid range.

## Limits

- The ground rolls ignore pitch attitude changes, rotation, crosswind and steering.
- No flare, no ground effect, no go-around.
- Runway length is not enforced. With no brakes the still-air roll-out is about 130 m.
- Wind speed on any real flight so far is unknown; 5 m/s down the runway is assumed.

## Effect on energy

Small. For the Mission 2 flight (below), takeoff, climb and the landing circuit together
add about 0.2 Ah to about 3.0 Ah for the laps, and changing the wind between 0 and 8 m/s
changes the total by under 0.1 Ah. See `docs/flight-validation.md`.
