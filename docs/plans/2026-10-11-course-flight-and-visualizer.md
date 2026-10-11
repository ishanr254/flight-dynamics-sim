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

## Stage 4 — manoeuvre mode (not started)
- [ ] Scripted large control inputs through the same model, with validity warnings
- [ ] Scenario picker: Course / Manoeuvres / natural modes; second aircraft

## Stage 5 — publish (not started)
- [ ] Pages serves the viewer; user documentation
