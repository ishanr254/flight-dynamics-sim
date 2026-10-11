# Standalone Python tool; aircraft data in TOML

Date: 2026-10-11

## Decided

- The tool must run for anyone with Python 3.12 and `pip install -r requirements.txt`.
  It does not call MATLAB, XFLR5, AVL, or any sibling repository at run time. Code and
  data from other repos may be copied in, never imported from where they live.
- Aircraft data files are TOML, one file per aircraft-plus-flight-condition, read with
  the standard-library `tomllib`.
- Benchmark reference values live in `data/reference/*.toml`, separate from aircraft data,
  each with a note on how the source obtained the number.
- Aircraft-specific scenarios (which inputs, which figures) live in `cases/`, outside
  `src/`, so solver code stays aircraft-agnostic (R2).

## Alternatives considered

- YAML: equally readable, but needs a third-party parser, one more dependency for a tool
  whose point is being easy to run.
- JSON: no comments, so units and data provenance (R8) could not sit next to the numbers.
- Python modules as data files: no parsing needed, but data files would be executable code
  and could not be validated before being run (R6).
- Reading XFLR5 XML directly as the aircraft format: ties the interface to one tool and
  those files do not contain the stability derivatives anyway.

## Consequence

The visualization plan in `CLAUDE.md` said to extend the sibling `flight-dynamics-visual`
repo in place. Under the standalone rule the visualizer page gets copied into this repo
when that phase starts.
