"""Run with python -m examples.standalone_demo from the repository root."""

import argparse
from pathlib import Path

from examples.common import ROOT, plot_route, synthetic_mission
from src.planning_utils import (a_star, a_star_through_points, heuristic,
                                heuristic_octile, iterative_astar)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "assets" / "planned_route.png")
    args = parser.parse_args()
    grid, start, checkpoints, goal = synthetic_mission()
    print("Synthetic map; start={}, goal={}, required={}".format(start, goal, checkpoints))
    for name, solver, h in [("A* Euclidean", a_star, heuristic),
                             ("A* Octile", a_star, heuristic_octile),
                             ("IDA* Octile", iterative_astar, heuristic_octile)]:
        path, cost = solver(grid, h, start, goal)
        print("{}: cost={:.9f}, path={}".format(name, cost, path))
    path, cost, stats = a_star_through_points(grid, heuristic_octile, start, checkpoints, goal, True)
    print("Ordered mission: grid_cost={:.9f}, geometric_distance={:.9f}, path={}".format(
        cost, stats["geometric_distance"], path))
    plot_route(grid, path, start, goal, checkpoints, args.output)
    print("Saved {}".format(args.output))


if __name__ == "__main__":
    main()
