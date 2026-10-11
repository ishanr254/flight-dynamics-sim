# Plan: performance limits from the aircraft

Approved 2026-10-11 (items 1-4 of the "what would make this useful" list). Stop for review
after the envelope and the limit lap.

Goal: make top speed, turn limits and energy use outputs of the model, then fly the
course at those limits so lap time is an output too.

- [x] Record models, data sources and assumptions
      (`docs/decisions/2026-10-11-performance-limits.md`)
- [x] 1. Propulsion: motor, battery and propeller model giving thrust and current against
        airspeed and throttle; checked against the report's static thrust test
- [x] 2. Aerodynamic limits: the report's drag polar (CD0, Oswald factor) and maximum lift
        coefficient in the nonlinear model
- [x] 3. Structural limit: load factor 4.5, never-exceed speed
- [x] Performance envelope: stall speed, top speed, sustained and instantaneous turn limits;
      compared with the report
- [x] 4. Autopilot that flies at the limits: full throttle, turns at whichever limit binds
- [x] Limit lap for Mission 1: lap time, energy used, which limit binds where; sweep of
      turn hardness
- [x] Export the limit lap to the viewer; tests; documentation of results

Status 2026-10-11: done; stopped for review. Results are in
`docs/decisions/2026-10-11-performance-limits.md`.

Not included: Missions 2 and 3, scoring, design sweeps (items 5-6), wind, takeoff/landing.

## Follow-up: turn strategy experiment (approved 2026-10-11)

Question: the envelope puts the best sustained turn near 20 m/s, well below the 29.6 m/s
top speed. Is "full throttle everywhere" really the fastest way round, or does slowing
before each turn pay?

- [x] Autopilot option: a turn-entry speed. Throttle is cut on the approach so the aircraft
      arrives at that speed, the speed is held through the turn, then full throttle.
- [x] Sweep entry speed (none, then from top speed down to the corner speed) at turn
      targets of 3.5 g and 4.5 g; compare lap time, path length and energy.
- [x] Record the answer and why in `docs/decisions/2026-10-11-performance-limits.md`.
- [x] Tests; add the best strategy to the viewer only if it differs visibly.

Assumption to state with the result: with the throttle closed the propeller is taken to
give zero thrust, not the extra drag of a windmilling propeller, so the model slows down
less quickly than the real aircraft would.

Status 2026-10-11: done. Answer: slowing to about the corner speed is faster, by 0.4 s at
the 4.5 g limit and more at lower limits, and uses much less battery. The fastest lap
found (entry at 22 m/s) is in the viewer.
