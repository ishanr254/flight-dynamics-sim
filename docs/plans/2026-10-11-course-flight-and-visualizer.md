# Plan: nonlinear simulation, competition course, visualizer

Approved 2026-10-11. Stages 1-3 run first, stopping to show one lap; stages 4-5 follow.

Goal: show DF1 flying the competition course (Mission 1) in a standalone web page in this
repository, with a second mode for extreme manoeuvres.

## Stage 1 — nonlinear flight model
- [x] Record the model and its assumptions
      (`docs/decisions/2026-10-11-nonlinear-flight-model.md`)
- [x] Rigid-body equations with exact gravity and attitude, aerodynamics from the
      aircraft file, control limits, simple thrust
- [x] Check: linearizing it about trim reproduces the linear lateral and longitudinal modes
- [x] Check: small-disturbance time responses match the linear simulation
- [x] Flag samples outside the aerodynamic model's valid range

## Stage 2 — autopilot and one lap
- [x] Course definition from the design report (Figure 3.1.1)
- [x] Autopilot: bank to turn, pitch for altitude, thrust for speed
- [x] Mission 1 lap at about 28 m/s, turns limited to load factor 2
- [x] Lap summary: time, bank, load factor, turn radius, speed and altitude excursions
- [x] Step-size convergence check (R5)

## Stage 3 — export and viewer
- [x] Export a simulation as a motion file the page can load without a server
- [x] Copy the existing visualizer page into `viz/`, with its 3D library vendored
- [x] Replace the hardcoded physics with playback; draw the course on the ground
- [x] Show one lap; stop for review

Status 2026-10-11: stages 1-3 done; stopped for review. Results are in
`docs/decisions/2026-10-11-nonlinear-flight-model.md`; the viewer is described in
`viz/README.md`.

## Stage 4 — manoeuvre mode (approved 2026-10-11)

Goal: a second mode in the viewer for extreme manoeuvres and responses, with the model
saying when it is being pushed past what it can represent.

- [x] Record what is flagged and why (`docs/decisions/2026-10-11-manoeuvre-mode.md`)
- [x] Scripted control inputs through the nonlinear model (steps, pulses, doublets)
- [x] A simple manoeuvre pilot for manoeuvres that need feedback (bank and load-factor
      targets): hard turn reversal, loop, approach to the stall
- [x] Flags carried with every sample: outside the aerodynamic model's valid range
      (angle of attack, sideslip, lift limit, roll helix angle) and over the structural
      load limit
- [x] DF1 manoeuvres: elevator to the stop, aileron roll, rudder doublet, turn reversal,
      loop, approach to the stall
- [x] DF1 natural modes from the nonlinear model: short period, phugoid, roll, Dutch roll,
      spiral
- [x] Second aircraft: the Case Study 7 transport's Dutch roll, spiral and rudder response,
      from the linear model
- [x] Viewer: grouped picker (Course / Manoeuvres / Natural modes / second aircraft),
      a strip of time traces under the readouts, a structural-limit warning, a suggested
      playback speed per scenario
- [x] Tests; documentation of results; stop for review

Status 2026-10-11: stage 4 done; stopped for review. Results are in
`docs/decisions/2026-10-11-manoeuvre-mode.md`.

## Stage 5 — publish (not started)
- [ ] Pages serves the viewer; user documentation
