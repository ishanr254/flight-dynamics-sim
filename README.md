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

## Using it

Needs Python 3.12. Nothing else: no MATLAB, XFLR5 or AVL at run time.

```
python3 -m venv venv && ./venv/bin/pip install -r requirements.txt
./venv/bin/pytest -q                                   # run the checks

# modes of any aircraft file (lateral and/or longitudinal)
./venv/bin/python -m src.analyze data/aircraft/df1_m1_distributed.toml

# stability derivatives from geometry, written as an aircraft file
./venv/bin/python -m src.aero.derive data/geometry/df1_m1.toml --aircraft my_aircraft.toml

# worked cases: comparison tables in the terminal, figures in output/
./venv/bin/python -m cases.cs7_lateral        # course Case Study 7 transport
./venv/bin/python -m cases.df1_lateral        # DF1 lateral modes vs XFLR5
./venv/bin/python -m cases.df1_longitudinal   # DF1 short period and phugoid vs XFLR5
./venv/bin/python -m cases.df1_vlm            # built-in aerodynamics vs XFLR5
```

To analyse a new aircraft, either write an aircraft file by hand
(`docs/aircraft-file.md`), convert an XFLR5 stability log (`src.aircraft.xflr5`), or
describe its geometry (`data/geometry/` for examples) and let the tool compute the
derivatives. No solver code changes are needed.

## Where things live

- `docs/aircraft-file.md` — the aircraft file format and its conventions
- `docs/plans/` — approved plans with their checklists

- `docs/requirements.md` — requirements table and benchmark aircraft plan
- `docs/timeline.md` — semester plan, mapped to the Fall 2026 academic calendar
- `docs/decisions/` — dated design-decision records, one per real decision, written
  when the decision is actually made
- `CLAUDE.md` — working conventions and architecture notes for implementation
  sessions (Claude Code or otherwise)

Planning (scope, requirements, timeline) happens in a separate Claude Project chat,
not in this repo. This repo is for implementation.
