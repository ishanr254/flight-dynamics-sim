"""Compare computed metrics against a reference-values file."""

import tomllib
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Comparison:
    metric: str
    computed: float
    reference: float
    kind: str
    source: str

    @property
    def relative_difference(self):
        return (self.computed - self.reference) / abs(self.reference)


def load_reference(path):
    return tomllib.loads(Path(path).read_text())["reference"]


def compare(computed, reference_path):
    """One Comparison per reference entry; a reference with no computed value is an error."""
    rows = []
    for ref in load_reference(reference_path):
        if ref["metric"] not in computed:
            raise KeyError(f"no computed value for reference metric '{ref['metric']}'")
        rows.append(Comparison(ref["metric"], float(computed[ref["metric"]]),
                               float(ref["value"]), ref.get("kind", ""),
                               ref.get("source", "")))
    return rows


def format_table(rows):
    width = max(len(r.metric) for r in rows)
    lines = [f"{'metric':<{width}}  {'computed':>10}  {'reference':>10}  {'diff':>7}  how the reference was obtained",
             "-" * (width + 70)]
    for r in rows:
        lines.append(f"{r.metric:<{width}}  {r.computed:>10.4g}  {r.reference:>10.4g}  "
                     f"{r.relative_difference:>+7.1%}  {r.kind}")
    return "\n".join(lines)
