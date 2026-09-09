# Flight Dynamics Simulator

A general-purpose, reusable linear flight-dynamics analysis tool for preliminary
aircraft stability and control assessment — Cornell MAE senior design project
(Ishan Roy, Fall 2026).

## Scope

Design and implement a general-purpose, reusable linear flight-dynamics analysis
tool for preliminary aircraft stability and control assessment — longitudinal and
lateral-directional modal analysis plus time-domain simulation — built so a new
aircraft can be analyzed by supplying its data through a defined interface, without
modifying the underlying solver. The simulator itself is the engineering design
artifact: functions, constraints, and objectives are defined explicitly; alternative
modeling and software approaches are compared and selected through analysis; the
result is verified against benchmark aircraft data. Mission-specific constraint
layers (competition rules, etc.) are an explicit extension point, not part of the
core deliverable.

No yaw damper / controls demonstration is planned this semester.

Faculty advisor: Professor Perrine Pepiot. Check-ins every other week, plus email
as needed.

## Where things live

- `docs/requirements.md` — requirements table and benchmark aircraft plan
- `docs/timeline.md` — semester plan, mapped to the Fall 2026 academic calendar
- `docs/decisions/` — dated design-decision records, one per real decision, written
  when the decision is actually made
- `CLAUDE.md` — working conventions and architecture notes for implementation
  sessions (Claude Code or otherwise)

Planning (scope, requirements, timeline) happens in a separate Claude Project chat,
not in this repo. This repo is for implementation.
