"""Write every scenario the web viewer shows: python3 -m cases.viewer_scenarios

Regenerates viz/data/scenarios.js from the simulator.
"""

from pathlib import Path

from cases.df1_course import course_scenario
from cases.df1_performance import limit_scenario
from src.viz import write_scenarios

SCENARIOS = Path("viz/data/scenarios.js")


def main():
    path = write_scenarios(SCENARIOS, [limit_scenario(), course_scenario()],
                           "cases.viewer_scenarios")
    print(f"viewer data written to {path} ({path.stat().st_size / 1024:.0f} kB)")


if __name__ == "__main__":
    main()
