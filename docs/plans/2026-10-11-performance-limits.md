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
